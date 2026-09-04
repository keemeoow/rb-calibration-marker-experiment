#!/usr/bin/python
# -*- coding: utf-8 -*-
"""zeus_server.py — thin primitive server for the ZEUS(i611) controller. PYTHON 2.

DESIGN RULE: this file contains NO task logic and NO cell knowledge.
All planning, frame transforms, safety-box checks, perception and target
computation happen on the CLIENT (robot/backends/zeus_client.py). The server
only:
    - reports the robot's current values          (get_state)
    - moves the robot where the client says       (movel / movej)
    - toggles queued/streaming motion mode        (stream_start / stream_stop)
    - actuates the gripper                        (grip)
    - stops                                       (stop)
Consequently this file should almost never need editing: no home pose, no
target coordinates, no tolerances live here. The only local settings are
hardware wiring (gripper DIO bits) and hard safety caps that a remote client
must not be able to raise.

Protocol: one JSON object per line over TCP; one reply line per request.
Pose = [x, y, z, rz, ry, rx] in MM / DEG (i611 Position argument order).
Joints = 6 values in DEG.

  {"op":"ping"}                                  -> {"ok":true}
  {"op":"get_state"}                             -> {"ok":true,"pose":[...],"joints":[...],
                                                     "gripper":[d,c,b,a]}
  {"op":"movel","pose":[...],"lin_speed":<mm/s>,
                 "overlap":<mm, optional>,"acc":<s, optional>} -> {"ok":true}
        overlap>0 blends this move into the next one instead of decelerating to
        a full stop — used by the jog tool for continuous motion. Default 0
        keeps the exact point-to-point behaviour generated programs rely on.
        BY DEFAULT (no stream_start active) this call is synchronous: it does
        not return until the i611 SDK has finished the motion (see
        server/c1.py's verify_robot_still(), which depends on that).
  {"op":"movej","joints":[...],"jnt_speed":<deg/s or i611 units>,
                 "overlap":<mm, optional>,"acc":<s, optional>} -> {"ok":true}
        Same overlap semantics and same default-synchronous behaviour as movel.
  {"op":"stream_start"}                          -> {"ok":true}
        Turns on the i611 SDK's program-prefetch queue (rb.asyncm(1)). While
        active, movel/movej calls return as soon as the motion is *queued*
        (i.e. roughly once the previous queued motion enters its overlap/blend
        region), not once the robot physically stops — this is the SDK's
        actual non-blocking primitive (see i611_MCS.py's asyncm()/join()
        docstrings). Use this instead of blasting movej at a fixed rate: pair
        it with overlap>0 so consecutive movej calls blend instead of each
        decelerating to a full stop. The queue depth/backpressure behaviour is
        not documented beyond "prefetches the next motion" — treat it as
        roughly one motion of lookahead, not an unbounded buffer.
  {"op":"stream_stop"}                           -> {"ok":true}
        Waits for the queued motion(s) to finish (rb.join()) and turns
        prefetch back off (rb.asyncm(2)), restoring the default synchronous
        behaviour movel/movej/other clients rely on. Always call this before
        disconnecting/exiting a streaming session — leaving asyncm(1) on
        would silently change behaviour for the next client (e.g. run.py).
  {"op":"grip","state":"open"|"close","timeout_s":3.0}
                                                 -> {"ok":true,"reached":true|false}
        NOTE reached=false on CLOSE means the fingers stalled on an object,
        i.e. something is held. The client interprets it; the server only reports.
  {"op":"stop"}                                  -> {"ok":true}
        Calls motion_skip() to halt in place. NOT verified against a queued
        (stream_start) motion — its interaction with the asyncm(1) prefetch
        queue is untested (i611_extend.py, which defines motion_skip, isn't
        available to read locally). Test an emergency stop at very low speed
        before trusting it during a streaming session.
  {"op":"bye"}   close this connection
  {"op":"quit"}  shut the server down

Run:  python ~/zeus_server.py          (on the ZEUS PC, i611 SDK environment)
"""

import json
import socket
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


def handle(rb, req):
    op = req.get("op")

    if op == "ping":
        return {"ok": True}

    if op == "get_state":
        pose = [float(v) for v in rb.getpos().pos2list()[:6]]
        joints = [float(v) for v in rb.getjnt().jnt2list()[:6]]
        return {"ok": True, "pose": pose, "joints": joints,
                "gripper": read_gripper()}

    if op == "movel":
        p = req["pose"]
        speed = min(float(req.get("lin_speed", 60.0)), MAX_LIN_SPEED)
        overlap = max(0.0, min(float(req.get("overlap", 0.0)), MAX_OVERLAP))
        acc = max(0.05, min(float(req.get("acc", DEFAULT_ACC)), 2.0))
        pose_speed = min(float(req.get("pose_speed", 20.0)), 50.0)  
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
        return {"ok": True}

    if op == "stream_stop":
        rb.join()
        rb.asyncm(2)
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

    if op in ("bye", "quit"):
        return {"ok": True, "_ctl": op}

    return {"ok": False, "err": "unknown op: %s" % op}


def main():
    print "initializing i611 robot ..."
    rb = i611Robot()
    _BASE = Base()
    rb.open()
    IOinit()
    rb.override(80)
    print "robot ready"
    
    rb.settool(1,0.0,0.0,97.5, 0.0,0.0,0.0)
    rb.changetool(1)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(1)
    print "zeus_server listening on %s:%d" % (HOST, PORT)

    running = True
    try:
        while running:
            conn, addr = srv.accept()
            print "client connected: %s" % str(addr)
            buf = ""
            try:
                while True:
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
                            reply = handle(rb, req)
                        except Exception as e:
                            reply = {"ok": False, "err": str(e)}
                        conn.sendall(json.dumps(reply) + "\n")
                        ctl = reply.get("_ctl")
                        if ctl == "quit":
                            running = False
                        if ctl in ("bye", "quit"):
                            raise StopIteration
            except StopIteration:
                pass
            except socket.error as e:
                print "socket error: %s" % e
            finally:
                # A client (e.g. a GELLO streaming session) can vanish mid-
                # session without ever sending stream_stop - leave the robot
                # stopped and back in synchronous mode so the next client
                # (or a human on the pendant) doesn't inherit a dangling
                # asyncm(1) queue.
                try:
                    rb.motion_skip()
                except Exception:
                    pass
                try:
                    rb.asyncm(2)
                except Exception:
                    pass
                conn.close()
                print "client disconnected"
    except KeyboardInterrupt:
        # accept() 에서 대기 중일 때 Ctrl+C 가 눌리면 여기로 온다. 이걸 못 잡으면
        # 아래 srv.close()/rb.close() 가 전혀 실행되지 않아 로봇 컨트롤 세션이 정리되지
        # 않은 채 남고, 그게 펜던트/본체 쪽 에러로 나타난다.
        print "\nKeyboardInterrupt -> shutting down"
    finally:
        srv.close()
        rb.close()
        print "zeus_server stopped"


if __name__ == "__main__":
    main()
