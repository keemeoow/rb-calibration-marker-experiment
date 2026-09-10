#!/usr/bin/python
# -*- coding: utf-8 -*-
"""zeus_gello.py — MULTI-CLIENT primitive server for the ZEUS(i611) controller. PYTHON 2.

zeus_server.py 와 같은 프로토콜이지만 **동시 접속을 허용한다**. GELLO teleop 이
로봇을 움직이는 동안 촬영 클라이언트가 같은 로봇의 pose/joints 를 읽어야 하는
캘리브레이션 촬영(07_capture_gello.py) 때문에 갈라져 나온 파일이다.

zeus_server.py 는 srv.listen(1) + 단일 accept 루프라 한 번에 클라이언트 하나만
처리한다. teleop 의 control_worker 는 스레드 시작 시 connect() 해서 프로세스가
끝날 때까지 소켓을 놓지 않으므로, 두 번째 클라이언트는 TCP 백로그에만 들어가고
get_state 응답을 영원히 못 받는다 (에러도 안 나고 그냥 멈춘다). 그래서 이 파일은
연결마다 스레드를 띄운다.

zeus_server.py 는 그대로 둔다 — c1.py / run.py 등이 "클라이언트는 나 하나"를
전제로 동작하므로, 그 전제를 바꾸는 건 GELLO 촬영 세션에서만 한다.

=== i611 SDK 스레드 제약 (중요, 이 파일의 구조를 결정한 이유) ===

i611Robot 은 rb.open() 을 호출한 스레드에서만 API 를 호출할 수 있다
(i611_MCS.py 의 _internal_hook() 이 매 호출마다 api_thread_id 를 확인한다).
다른 스레드에서 부르면 그 호출 자체는 별 일 없이 실행되지만 내부 fatal_error
플래그만 몰래 세팅되고, 그 다음에 "정상 스레드"에서 아무 API나 호출하는 순간
"cause fatal exit(21:Invalid call of API from another thread occurred.)" 로
죽는다 - teleop 도중엔 안 터지고 종료(Ctrl+C) 정리 시점에 갑자기 터지는 걸로
보이는 이유가 이것이다.

그래서 연결마다 뜨는 스레드(serve_client)는 소켓 I/O만 하고, 실제 rb.* 호출은
전부 **메인 스레드**가 명령 큐(_cmd_queue)를 통해서만 수행한다. 그 결과:
  - _RB_LOCK 은 더 이상 필요 없다 (rb.* 를 부르는 스레드가 하나뿐이라 자동 직렬화).
  - 연결이 끊길 때의 정리(motion_skip/asyncm)도 직접 부르지 않고 큐에 올린다.

=== zeus_server.py 와 달라진 점 ===

1) 연결마다 스레드(소켓 I/O만). srv.listen(8). 실제 로봇 호출은 메인 스레드가
   명령 큐를 통해 순차 처리.

2) 모션 소유권(motion ownership). 로봇을 움직이는 op
   (movel/movej/grip/stop/stream_start/stream_stop) 는 첫 번째로 요청한 연결이
   소유권을 잡고, 그 연결이 살아있는 동안 다른 연결은 같은 op 에서 거절당한다:
       {"ok": false, "err": "motion is owned by <addr>"}
   읽기 전용 op (ping/get_state) 는 누구나 언제든 쓸 수 있다.
   촬영 클라이언트가 실수로 로봇을 움직이는 사고와, 두 개가 동시에 모션을 밀어
   넣는 사고를 둘 다 막는다. 소유권은 그 연결이 끊어질 때 자동 해제된다.

3) 연결 종료 시 정리(motion_skip + asyncm(2))를 **모션 소유자가 끊어질 때만**
   한다. zeus_server.py 는 아무 클라이언트나 끊기면 무조건 로봇을 세우는데,
   여기서 그러면 촬영 클라이언트가 Ctrl+C 로 빠질 때마다 teleop 중인 로봇이
   멈춘다.

4) {"op":"quit"} 은 모션 소유자(또는 소유자가 없을 때)만 서버를 내릴 수 있다.

프로토콜 자체는 zeus_server.py 와 동일하다:

  {"op":"ping"}                                  -> {"ok":true}
  {"op":"get_state"}                             -> {"ok":true,"pose":[...],"joints":[...],
                                                     "gripper":[d,c,b,a]}
        pose = [x,y,z,rz,ry,rx] MM/DEG, joints = 6 x DEG.
        읽기 전용 — 모션 소유권과 무관하게 아무 클라이언트나 호출 가능.
  {"op":"movel","pose":[...],"lin_speed":<mm/s>,"overlap":<mm>,"acc":<s>} -> {"ok":true}
  {"op":"movej","joints":[...],"jnt_speed":<deg/s>,"overlap":<mm>,"acc":<s>} -> {"ok":true}
  {"op":"stream_start"} / {"op":"stream_stop"}   -> {"ok":true}
  {"op":"grip","state":"open"|"close","timeout_s":3.0} -> {"ok":true,"reached":bool}
        CLOSE 에서 reached=false 는 "물체를 물어서 다 못 닫힘" = 파지 성공이다.
  {"op":"stop"}                                  -> {"ok":true}
  {"op":"bye"}   이 연결만 종료
  {"op":"quit"}  서버 종료 (모션 소유자만)

Run:  python ~/zeus_gello.py          (ZEUS PC, i611 SDK 환경)
"""

import json
import Queue
import socket
import threading
import time

from i611_MCS import *
from i611_extend import *
from rbsys import *
from i611_common import *
from i611_io import *
from i611shm import *

HOST = "0.0.0.0"
PORT = 12350

# ── hard safety caps (a remote client cannot exceed these) ──────────────
MAX_LIN_SPEED = 120.0     # mm/s
MAX_JNT_SPEED = 30.0      # i611 jnt_speed units
DEFAULT_ACC = 0.6         # acctime / dacctime
MAX_OVERLAP = 20.0        # mm of path blending the client may request

# ── gripper wiring (hardware, not task config) ──────────────────────────
GRIP_DOUT_PORT = 48
GRIP_OPEN_BITS = '0100'
GRIP_CLOSE_BITS = '0001'
GRIP_OPEN_SENSE = ['0', '1', '0', '0']
GRIP_CLOSE_SENSE = ['0', '0', '0', '1']

# 로봇을 움직이는 op. 이것만 모션 소유권이 필요하다.
MOTION_OPS = ("movel", "movej", "grip", "stop", "stream_start", "stream_stop")

# 모션 소유권. _OWNER_LOCK 로만 건드린다 (이건 순수 파이썬 dict/변수라 i611 SDK
# 스레드 제약과 무관 - 어느 스레드에서 건드려도 안전하다).
_OWNER_LOCK = threading.Lock()
_motion_owner = None        # 소유 중인 연결의 addr 튜플, 또는 None
_owner_used_stream = False  # 소유자가 stream_start 를 켰는지 (정리 때 필요)

# 연결 스레드 -> 메인 스레드로 rb.* 호출을 위임하는 명령 큐. 메인 스레드만
# 이 큐를 소비하고 실제 i611 API 를 부른다 (스레드 제약 준수).
_cmd_queue = Queue.Queue()

_shutdown = threading.Event()


def read_gripper():
    a = din(48)
    b = din(49)
    c = din(50)
    d = din(51)
    return [d, c, b, a]


def grip(state, timeout_s):
    """Drive the pneumatic gripper. Returns True if the target sensor state
    was reached within the timeout (on CLOSE, False => object held)."""
    dout(GRIP_DOUT_PORT, '0000')
    if state == 'open':
        want, cmd = GRIP_OPEN_SENSE, GRIP_OPEN_BITS
    else:
        want, cmd = GRIP_CLOSE_SENSE, GRIP_CLOSE_BITS
    t0 = time.time()
    while read_gripper() != want:
        dout(GRIP_DOUT_PORT, cmd)
        if time.time() - t0 > timeout_s:
            return False
        time.sleep(0.05)
    return True


def claim_motion(addr):
    """이 연결이 모션을 쓸 수 있으면 (True, None), 아니면 (False, 소유자)."""
    global _motion_owner
    with _OWNER_LOCK:
        if _motion_owner is None:
            _motion_owner = addr
            print "motion owner -> %s" % str(addr)
            return True, None
        if _motion_owner == addr:
            return True, None
        return False, _motion_owner


def release_motion(addr):
    """이 연결이 소유자였으면 해제하고, stream 을 켰었는지 알려준다."""
    global _motion_owner, _owner_used_stream
    with _OWNER_LOCK:
        if _motion_owner != addr:
            return False, False
        used_stream = _owner_used_stream
        _motion_owner = None
        _owner_used_stream = False
        print "motion owner released by %s" % str(addr)
        return True, used_stream


def handle(rb, req, addr):
    """한 요청을 처리한다. 반드시 메인 스레드(rb.open() 을 부른 스레드)에서만
    호출할 것 - i611 SDK 의 스레드 제약 때문에 다른 스레드에서 부르면 이번
    호출은 조용히 넘어가지만 다음 정상 호출 때 fatal exit(21) 로 터진다."""
    global _owner_used_stream

    op = req.get("op")

    if op == "ping":
        return {"ok": True}

    if op == "get_state":
        pose = [float(v) for v in rb.getpos().pos2list()[:6]]
        joints = [float(v) for v in rb.getjnt().jnt2list()[:6]]
        return {"ok": True, "pose": pose, "joints": joints,
                "gripper": read_gripper()}

    if op in MOTION_OPS:
        allowed, owner = claim_motion(addr)
        if not allowed:
            return {"ok": False,
                    "err": "motion is owned by %s (this connection may only "
                           "use ping/get_state)" % str(owner)}

    if op == "movel":
        p = req["pose"]
        speed = min(float(req.get("lin_speed", 60.0)), MAX_LIN_SPEED)
        overlap = max(0.0, min(float(req.get("overlap", 0.0)), MAX_OVERLAP))
        acc = max(0.05, min(float(req.get("acc", DEFAULT_ACC)), 2.0))
        rb.motionparam(MotionParam(lin_speed=speed, jnt_speed=MAX_JNT_SPEED,
                                   pose_speed=50, overlap=overlap,
                                   acctime=acc, dacctime=acc))
        rb.line(Position(p[0], p[1], p[2], p[3], p[4], p[5]))
        return {"ok": True}

    if op == "movej":
        j = req["joints"]
        speed = min(float(req.get("jnt_speed", 10.0)), MAX_JNT_SPEED)
        overlap = max(0.0, min(float(req.get("overlap", 0.0)), MAX_OVERLAP))
        acc = max(0.05, min(float(req.get("acc", DEFAULT_ACC)), 2.0))
        rb.motionparam(MotionParam(lin_speed=MAX_LIN_SPEED, jnt_speed=speed,
                                   pose_speed=50, overlap=overlap,
                                   acctime=acc, dacctime=acc))
        rb.move(Joint(j[0], j[1], j[2], j[3], j[4], j[5]))
        return {"ok": True}

    if op == "stream_start":
        rb.asyncm(1)
        with _OWNER_LOCK:
            _owner_used_stream = True
        return {"ok": True}

    if op == "stream_stop":
        rb.join()
        rb.asyncm(2)
        with _OWNER_LOCK:
            _owner_used_stream = False
        return {"ok": True}

    if op == "grip":
        reached = grip(req.get("state", "open"), float(req.get("timeout_s", 3.0)))
        return {"ok": True, "reached": reached}

    if op == "stop":
        try:
            rb.motion_skip()
        except Exception:
            pass
        return {"ok": True}

    if op == "bye":
        return {"ok": True, "_ctl": "bye"}

    if op == "quit":
        # 서버를 내리는 건 로봇을 쥐고 있는 쪽만. 촬영 클라이언트가 실수로
        # quit 을 보내도 teleop 세션이 죽지 않게 한다.
        with _OWNER_LOCK:
            owner = _motion_owner
        if owner is not None and owner != addr:
            return {"ok": False,
                    "err": "quit refused: motion is owned by %s" % str(owner)}
        return {"ok": True, "_ctl": "quit"}

    return {"ok": False, "err": "unknown op: %s" % op}


def dispatch(req, addr, timeout=10.0):
    """연결 스레드에서 호출: 요청을 메인 스레드에 큐로 넘기고 응답을 기다린다.
    이 함수 자체는 i611 API 를 절대 직접 부르지 않는다."""
    reply_box = Queue.Queue(maxsize=1)
    _cmd_queue.put((req, addr, reply_box))
    try:
        return reply_box.get(timeout=timeout)
    except Queue.Empty:
        return {"ok": False, "err": "timed out waiting for robot thread"}


def serve_client(conn, addr):
    """한 클라이언트 연결을 끝까지 처리한다 (연결당 스레드 하나) - 소켓 I/O만
    하고, 로봇 호출은 전부 dispatch() 로 메인 스레드에 위임한다."""
    print "client connected: %s" % str(addr)
    buf = ""
    try:
        while not _shutdown.is_set():
            chunk = conn.recv(4096)
            if not chunk:
                break
            buf += chunk
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                if not line.strip():
                    continue
                try:
                    req = json.loads(line)
                except Exception, e:
                    conn.sendall(json.dumps({"ok": False, "err": str(e)}) + "\n")
                    continue
                reply = dispatch(req, addr)
                conn.sendall(json.dumps(reply) + "\n")
                ctl = reply.get("_ctl")
                if ctl == "quit":
                    _shutdown.set()
                    return
                if ctl == "bye":
                    return
    except socket.error, e:
        print "socket error (%s): %s" % (str(addr), e)
    finally:
        # 모션 소유자가 사라졌을 때만 로봇을 정리한다. 읽기 전용 클라이언트
        # (촬영 스크립트) 가 빠지는 건 로봇 상태를 건드릴 이유가 없다.
        # 이 정리도 rb.* 호출이라 직접 부르지 않고 큐를 통해 메인 스레드에
        # 맡긴다 - reply 는 안 기다린다(연결이 이미 끊기는 중이라 상관없음).
        was_owner, used_stream = release_motion(addr)
        if was_owner:
            print "motion owner %s vanished -> stopping robot" % str(addr)
            _cmd_queue.put(({"op": "stop"}, addr, Queue.Queue(maxsize=1)))
            if used_stream:
                _cmd_queue.put(({"op": "stream_stop"}, addr, Queue.Queue(maxsize=1)))
        try:
            conn.close()
        except Exception:
            pass
        print "client disconnected: %s" % str(addr)


def accept_loop(srv):
    """전용 스레드: 연결만 accept 하고 소켓 I/O 스레드를 띄운다. i611 API 는
    절대 안 부르므로 메인 스레드가 아니어도 된다."""
    threads = []
    while not _shutdown.is_set():
        try:
            conn, addr = srv.accept()
        except socket.timeout:
            continue
        except socket.error:
            break
        t = threading.Thread(target=serve_client, args=(conn, addr))
        t.daemon = True
        t.start()
        threads.append(t)
        threads[:] = [x for x in threads if x.is_alive()]


def main():
    print "initializing i611 robot ..."
    rb = i611Robot()
    _BASE = Base()
    rb.open()
    IOinit()
    rb.override(80)
    print "robot ready"

    rb.settool(1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    rb.changetool(1)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(8)
    srv.settimeout(1.0)
    print "zeus_gello (multi-client) listening on %s:%d" % (HOST, PORT)
    print "  motion ops (movel/movej/grip/stop/stream_*) : 먼저 잡은 연결 하나만"
    print "  read-only ops (ping/get_state)              : 아무나 동시에"

    acceptor = threading.Thread(target=accept_loop, args=(srv,))
    acceptor.daemon = True
    acceptor.start()

    try:
        # 이 while 문 안에서만 rb.* 를 부른다 - main() 을 부른 스레드가
        # rb.open() 을 호출한 스레드이므로 i611 SDK 의 스레드 제약을 만족한다.
        while not _shutdown.is_set():
            try:
                req, addr, reply_box = _cmd_queue.get(timeout=0.05)
            except Queue.Empty:
                continue
            try:
                reply = handle(rb, req, addr)
            except Exception, e:
                reply = {"ok": False, "err": str(e)}
            try:
                reply_box.put_nowait(reply)
            except Exception:
                pass  # 정리용 fire-and-forget 요청은 받는 쪽이 없어도 됨
            if reply.get("_ctl") == "quit":
                _shutdown.set()
    except KeyboardInterrupt:
        # 메인 스레드가 명령 큐를 기다리는 중 Ctrl+C. 이걸 안 잡으면 아래
        # 정리가 통째로 안 돌아 로봇 컨트롤 세션이 남고, 그게 펜던트/본체
        # 에러로 나타난다.
        print "\nKeyboardInterrupt -> shutting down"
    finally:
        _shutdown.set()
        srv.close()
        acceptor.join(2.0)
        # 여기서부터는 여전히 메인 스레드 - rb.* 직접 호출 안전.
        try:
            rb.motion_skip()
        except Exception:
            pass
        try:
            rb.asyncm(2)
        except Exception:
            pass
        rb.close()
        print "zeus_gello stopped"


if __name__ == "__main__":
    main()
