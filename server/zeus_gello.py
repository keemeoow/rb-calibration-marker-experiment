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

=== zeus_server.py 와 달라진 점 ===

1) 연결마다 스레드. srv.listen(8).

2) 모든 i611 SDK 호출을 _RB_LOCK 하나로 직렬화한다. i611 SDK 의 스레드 안전성은
   문서화돼 있지 않으므로 "동시에 두 개가 들어가는 일은 절대 없다"를 락으로
   보장한다. 결과적으로 blocking 인 movel/movej 가 실행되는 동안 get_state 는
   그만큼 기다린다 — 움직이는 중의 pose 는 어차피 촬영에 못 쓰므로 의도된 동작이다.
   (teleop 은 stream_start 로 asyncm(1) 을 켜므로 movej 가 "큐잉되면 즉시 리턴"
   이라 락 점유 시간이 짧다.)

3) 모션 소유권(motion ownership). 로봇을 움직이는 op
   (movel/movej/grip/stop/stream_start/stream_stop) 는 첫 번째로 요청한 연결이
   소유권을 잡고, 그 연결이 살아있는 동안 다른 연결은 같은 op 에서 거절당한다:
       {"ok": false, "err": "motion is owned by <addr>"}
   읽기 전용 op (ping/get_state) 는 누구나 언제든 쓸 수 있다.
   촬영 클라이언트가 실수로 로봇을 움직이는 사고와, 두 개가 동시에 모션을 밀어
   넣는 사고를 둘 다 막는다. 소유권은 그 연결이 끊어질 때 자동 해제된다.

4) 연결 종료 시 정리(motion_skip + asyncm(2))를 **모션 소유자가 끊어질 때만**
   한다. zeus_server.py 는 아무 클라이언트나 끊기면 무조건 로봇을 세우는데,
   여기서 그러면 촬영 클라이언트가 Ctrl+C 로 빠질 때마다 teleop 중인 로봇이
   멈춘다.

5) {"op":"quit"} 은 모션 소유자(또는 소유자가 없을 때)만 서버를 내릴 수 있다.

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

# i611 SDK 호출 직렬화. 모든 handle() 본문이 이 락 안에서만 돈다.
_RB_LOCK = threading.Lock()

# 모션 소유권. _OWNER_LOCK 로만 건드린다.
_OWNER_LOCK = threading.Lock()
_motion_owner = None        # 소유 중인 연결의 addr 튜플, 또는 None
_owner_used_stream = False  # 소유자가 stream_start 를 켰는지 (정리 때 필요)

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
    """한 요청을 처리한다. 호출자가 _RB_LOCK 을 잡고 들어온다."""
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


def serve_client(rb, conn, addr):
    """한 클라이언트 연결을 끝까지 처리한다 (연결당 스레드 하나)."""
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
                try:
                    with _RB_LOCK:
                        reply = handle(rb, req, addr)
                except Exception, e:
                    reply = {"ok": False, "err": str(e)}
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
        was_owner, used_stream = release_motion(addr)
        if was_owner:
            print "motion owner %s vanished -> stopping robot" % str(addr)
            with _RB_LOCK:
                try:
                    rb.motion_skip()
                except Exception:
                    pass
                if used_stream:
                    try:
                        rb.asyncm(2)
                    except Exception:
                        pass
        try:
            conn.close()
        except Exception:
            pass
        print "client disconnected: %s" % str(addr)


def main():
    print "initializing i611 robot ..."
    rb = i611Robot()
    _BASE = Base()
    rb.open()
    IOinit()
    rb.override(80)
    print "robot ready"

    rb.settool(1, 0.0, 0.0, 97.5, 0.0, 0.0, 0.0)
    rb.changetool(1)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(8)
    # accept() 가 영원히 막히면 _shutdown 을 확인할 기회가 없어 quit/Ctrl+C 후에도
    # 프로세스가 안 죽는다.
    srv.settimeout(1.0)
    print "zeus_gello (multi-client) listening on %s:%d" % (HOST, PORT)
    print "  motion ops (movel/movej/grip/stop/stream_*) : 먼저 잡은 연결 하나만"
    print "  read-only ops (ping/get_state)              : 아무나 동시에"

    threads = []
    try:
        while not _shutdown.is_set():
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            t = threading.Thread(target=serve_client, args=(rb, conn, addr))
            t.daemon = True
            t.start()
            threads.append(t)
            threads = [x for x in threads if x.is_alive()]
    except KeyboardInterrupt:
        # accept() 에서 대기 중 Ctrl+C. 이걸 안 잡으면 아래 정리가 통째로
        # 안 돌아 로봇 컨트롤 세션이 남고, 그게 펜던트/본체 에러로 나타난다.
        print "\nKeyboardInterrupt -> shutting down"
    finally:
        _shutdown.set()
        for t in threads:
            t.join(2.0)
        srv.close()
        with _RB_LOCK:
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

