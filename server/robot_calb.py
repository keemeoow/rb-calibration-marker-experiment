#!/usr/bin/python
# -*- coding: utf-8 -*-

from i611_MCS import *
from teachdata import *
from i611_extend import *
from rbsys import *
from i611_common import *
from i611_io import *
from i611shm import *
import sys
import time
import socket
import json

HOST = '0.0.0.0'
PORT = 12348

GRIPPER_IO_PORT = 48
GRIPPER_TIMEOUT_SEC = 5.0

CUBE_SIZE_MM = 30.0
CUBE_GRIP_DEPTH_MM = 2.0
CUBE_CENTER_OFFSET_Z = CUBE_SIZE_MM / 2.0 - CUBE_GRIP_DEPTH_MM

TOOL_GRIPPER_Z = 113.5 # 기존에 150.0 이였음.
TOOL_CUBE_CENTER_Z = TOOL_GRIPPER_Z - CUBE_CENTER_OFFSET_Z

# 큐브를 잡을 때(재-그립) 항상 place 위치 +Z 위에서 접근 후 수직 하강하여 안전하게 잡는다.
GRIP_APPROACH_Z_MM = 50.0
# B(grip-sweep) TCP pose 로 line 이동 시: 목표 +Z 위로 먼저 간 뒤 하강 (급격한 직행 방지)
TCP_APPROACH_Z_MM = 40.0
# save gate 실패 시: 자동 지터 없이 곧바로 사람이 jog하는 manual_recover로 진입한다.

TCP_AXIS_MAP = {'x': 'dx', 'y': 'dy', 'z': 'dz', 'rz': 'drz', 'ry': 'dry', 'rx': 'drx'}
JOINT_AXIS_MAP = {'d1': 'dj1', 'd2': 'dj2', 'd3': 'dj3', 'd4': 'dj4', 'd5': 'dj5', 'd6': 'dj6'}
VALID_AXES = set(list(TCP_AXIS_MAP.keys()) + list(JOINT_AXIS_MAP.keys()))


# ── Socket ──

# Newline-delimited JSON framing.
# 한 메시지가 단일 recv() 청크 크기(예: waypoints_data 응답은 15KB+)를 넘거나
# 여러 메시지가 한 청크에 합쳐져 도착해도 안전하게 한 건씩 잘라서 반환한다.
_RECV_BUF = {'data': b''}


def send_json(conn, obj):
    try:
        msg = json.dumps(obj)
        conn.sendall((msg + '\n').encode('utf-8'))
        print "Sent: {}".format(msg)
    except socket.error as e:
        print "Send error: {}".format(e)


def recv_json(conn):
    """Receive one newline-delimited JSON object (handles large/split messages)."""
    try:
        while b'\n' not in _RECV_BUF['data']:
            chunk = conn.recv(65536)
            if not chunk:
                # peer closed; try to parse any unterminated remainder.
                if _RECV_BUF['data']:
                    line = _RECV_BUF['data']
                    _RECV_BUF['data'] = b''
                    try:
                        return json.loads(line.decode('utf-8').strip())
                    except Exception as e:
                        print "Recv parse error: {}".format(e)
                return None
            _RECV_BUF['data'] += chunk
        line, _, rest = _RECV_BUF['data'].partition(b'\n')
        _RECV_BUF['data'] = rest
        return json.loads(line.decode('utf-8').strip())
    except socket.error as e:
        print "Recv error: {}".format(e)
    except Exception as e:
        print "Recv parse error: {}".format(e)
    return None


# ── Robot helpers ──

def fmt6(v):
    return '[{:.1f}, {:.1f}, {:.1f}, {:.1f}, {:.1f}, {:.1f}]'.format(
        v[0], v[1], v[2], v[3], v[4], v[5])


def get_tcp():
    return rb.getpos().pos2list()[:6]


def get_cube_center():
    rb.changetool(4)
    tcp = rb.getpos().pos2list()[:6]
    rb.changetool(3)
    return tcp


def send_teach(conn, kind, data):
    """teach 기록(recpose/recgrip/recset)을 PC로 전송해 PC에만 저장하도록 한다.

    kind: 'pose'(뷰포인트/A) | 'grip'(그립-스윕/B) | 'set'(큐브 배치).
    로봇 로컬에는 저장하지 않는다. PC(Step2)가 받아서 세션 번호 붙은 파일로 기록한다.
    매번 전체 리스트를 보내 PC가 파일을 통째로 갱신하게 한다(undo 도 동일).
    """
    send_json(conn, {"command": "teach_save", "kind": kind, "data": data})


def get_joints():
    return rb.getjnt().jnt2list()[:6]


def show_pose():
    tcp = get_tcp()
    jnt = get_joints()
    print ''
    print '     joints: [{:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}]'.format(
        jnt[0], jnt[1], jnt[2], jnt[3], jnt[4], jnt[5])
    print '     tcp:    ({:.1f}, {:.1f}, {:.1f}) / ({:.1f}, {:.1f}, {:.1f})'.format(
        tcp[0], tcp[1], tcp[2], tcp[3], tcp[4], tcp[5])
    print ''
    return tcp


def move_tcp(axis, value):
    if axis not in TCP_AXIS_MAP:
        print 'Invalid axis: {}. Use x,y,z,rz,ry,rx'.format(axis)
        return
    current = Position(*rb.getpos().pos2list()[:6])
    rb.line(current.offset(**{TCP_AXIS_MAP[axis]: value}))
    print 'TCP {} += {} done'.format(axis, value)


def move_joint(axis, value):
    if axis not in JOINT_AXIS_MAP:
        print 'Invalid axis: {}. Use d1~d6'.format(axis)
        return
    current = Joint(*rb.getjnt().jnt2list()[:6])
    rb.move(current.offset(**{JOINT_AXIS_MAP[axis]: value}))
    print 'Joint {} += {} done'.format(axis, value)


def undo_one(entry):
    mtype, maxis, mvalue = entry
    print '  {} {},{} -> {}'.format(mtype, maxis, mvalue, -mvalue)
    if mtype == 'p':
        move_tcp(maxis, -mvalue)
    else:
        move_joint(maxis, -mvalue)


# ── Gripper ──

def check_gripper():
    return [din(GRIPPER_IO_PORT + i) for i in [3, 2, 1, 0]]


def gripper_open():
    print 'Gripper opening...'
    dout(GRIPPER_IO_PORT, '0000')
    t0 = time.time()
    while check_gripper() != ['0', '1', '0', '0']:
        dout(GRIPPER_IO_PORT, '0100')
        if time.time() - t0 > GRIPPER_TIMEOUT_SEC:
            print '[WARN] Gripper open timeout!'
            break
        time.sleep(0.05)
    print 'Gripper opened'


def gripper_close():
    print 'Gripper closing...'
    dout(GRIPPER_IO_PORT, '0000')
    t0 = time.time()
    while check_gripper() != ['0', '0', '0', '1']:
        dout(GRIPPER_IO_PORT, '0001')
        if time.time() - t0 > GRIPPER_TIMEOUT_SEC:
            print '[WARN] Gripper close timeout!'
            break
        time.sleep(0.05)
    print 'Gripper closed'


# ── Capture ──

def do_capture(conn, capture_index, set_cube_center=None, set_index=None,
               set_joints=None, set_tcp=None, place_joints=None,
               cube_gripped=False, capture_block="A_placement", grasp_id=0):
    """Returns (status, tcp, cube_tcp) or (None, None, None) on disconnect.

    capture_block / cube_gripped / grasp_id tag each frame so Step3 can separate:
      A_placement  : cube released on table (set_cube_center anchor, method (a))
      B_eyetohand  : cube rigidly gripped, robot sweeps (eye-to-hand, method (b))
    """
    tcp = get_tcp()
    cube_tcp = get_cube_center()
    joints = get_joints()
    print ''
    print '*** CAPTURE {} (block={} gripped={} grasp={}) ***'.format(
        capture_index, capture_block, cube_gripped, grasp_id)
    print '  fingertip:    {}'.format(fmt6(tcp))
    print '  cube center:  {}'.format(fmt6(cube_tcp))

    msg = {
        "command": "capture",
        "capture_gripper_pose_6dof": tcp,
        "capture_cube_center_6dof": cube_tcp,
        "capture_robot_joints_6dof": joints,
        "capture_index": capture_index,
        "cube_gripped": bool(cube_gripped),
        "capture_block": capture_block,
        "grasp_id": int(grasp_id),
    }
    if set_cube_center is not None:
        msg["set_cube_center_6dof"] = set_cube_center
    if set_index is not None:
        msg["set_index"] = set_index
    if set_joints is not None:
        msg["set_joints"] = set_joints
    if set_tcp is not None:
        msg["set_tcp"] = set_tcp
    if place_joints is not None:
        msg["place_joints"] = place_joints

    send_json(conn, msg)
    resp = recv_json(conn)
    if resp is None:
        print 'Client disconnected!'
        return None, None, None

    status = resp.get('status', 'unknown') if isinstance(resp, dict) else 'unknown'
    reason = resp.get('reason') if isinstance(resp, dict) else None
    if reason:
        print '*** Capture {} done (status={}, reason={}) ***'.format(capture_index, status, reason)
    else:
        print '*** Capture {} done (status={}) ***'.format(capture_index, status)
    return status, tcp, cube_tcp


# ── Auto capture ──

def approach_and_close_gripper(rb, place_joints, place_tcp=None,
                                approach_z_mm=GRIP_APPROACH_Z_MM):
    """그리퍼 닫기 전 항상 +Z 위에서 접근 후 하강하여 닫는다.

    place_tcp이 주어지면 (place_tcp + +Z) -> place_tcp 라인 모션으로 접근.
    주어지지 않으면 place_joints로 직접 이동 후 닫는다(폴백).
    """
    if place_tcp is not None:
        above = list(place_tcp[:6])
        above[2] += approach_z_mm
        try:
            print '[Auto] +Z {:.0f}mm approach above grip pose'.format(approach_z_mm)
            rb.line(Position(*above))
            time.sleep(0.3)
            print '[Auto] descend to grip pose'
            rb.line(Position(*place_tcp[:6]))
            time.sleep(0.2)
        except Exception as e:
            print '[WARN] line approach failed: {} -> joint move fallback'.format(e)
            rb.move(Joint(*place_joints[:6]))
            time.sleep(0.3)
    else:
        rb.move(Joint(*place_joints[:6]))
        time.sleep(0.3)
    gripper_close()


def manual_recover(rb, conn, pose_idx, capture_kwargs):
    """Marker detection failed at an auto waypoint. Hand control to the operator to
    jog the robot until the cube is detected, then re-capture from the current pose.

    Returns 'success' / 'skip' / 'quit' / None(disconnect). Jog commands mirror the
    main manual loop (p / j / gotop / gotoj / show).
    """
    print ''
    print '  [Recover] marker not detected here. Manual jog until visible, then c.'
    print '    p <axis>,<v>  j <axis>,<v>  gotop x,y,z[,rz,ry,rx]  gotoj d1..d6'
    print '    show | c: re-capture | s: skip this pose | q: quit'
    while True:
        try:
            line = raw_input('  recover> ').strip()
        except EOFError:
            return 'skip'
        if not line:
            continue
        ll = line.lower()
        if ll == 'c':
            status, _, _ = do_capture(conn, pose_idx, **capture_kwargs)
            if status is None:
                return None
            if status == 'success':
                print '  [Recover] -> OK'
                return 'success'
            print '  [Recover] still failing (status={}); jog more, or s/q'.format(status)
        elif ll == 's':
            return 'skip'
        elif ll == 'q':
            return 'quit'
        elif ll == 'show':
            show_pose()
        elif ll.startswith('p '):
            try:
                parts = line[2:].split(',')
                move_tcp(parts[0].strip(), float(parts[1]))
                show_pose()
            except Exception as e:
                print '  err: {}. Usage: p <axis>,<value>'.format(e)
        elif ll.startswith('j '):
            try:
                parts = line[2:].split(',')
                move_joint(parts[0].strip(), float(parts[1]))
                show_pose()
            except Exception as e:
                print '  err: {}. Usage: j <axis>,<value>'.format(e)
        elif ll.startswith('gotop ') or ll.startswith('goto '):
            try:
                rest = line[6:] if ll.startswith('gotop ') else line[5:]
                vals = [float(v.strip()) for v in rest.split(',')]
                if len(vals) == 6:
                    rb.line(Position(*vals))
                elif len(vals) == 3:
                    t = get_tcp()
                    rb.line(Position(vals[0], vals[1], vals[2], t[3], t[4], t[5]))
                else:
                    print '  usage: gotop x,y,z[,rz,ry,rx]'
                    continue
                show_pose()
            except Exception as e:
                print '  err: {}'.format(e)
        elif ll.startswith('gotoj '):
            try:
                vals = [float(v.strip()) for v in line[6:].split(',')]
                if len(vals) == 6:
                    rb.move(Joint(*vals))
                    show_pose()
                else:
                    print '  usage: gotoj d1,d2,d3,d4,d5,d6'
            except Exception as e:
                print '  err: {}'.format(e)
        else:
            print '  (p / j / gotop / gotoj / show / c / s / q)'


def request_waypoints_from_pc(conn, timeout_sec=10.0):
    """Request capture_waypoints.json content from the PC over the socket.

    Returns the parsed dict on success, or None on failure / timeout.
    """
    print 'Requesting waypoints from PC...'
    send_json(conn, {"command": "request_waypoints"})
    conn.settimeout(timeout_sec)
    try:
        resp = recv_json(conn)
    except socket.timeout:
        print '[ERROR] PC did not respond within {}s'.format(timeout_sec)
        conn.settimeout(None)
        return None
    finally:
        try:
            conn.settimeout(None)
        except Exception:
            pass
    if not isinstance(resp, dict):
        print '[ERROR] invalid response from PC'
        return None
    if resp.get('status') != 'ok':
        print '[ERROR] PC reported error: {}'.format(resp.get('reason', 'unknown'))
        return None
    data = resp.get('waypoints_data')
    if not isinstance(data, dict):
        print '[ERROR] PC response missing waypoints_data'
        return None
    n_wps = len(data.get('waypoints', []))
    print '  received {} waypoints from PC'.format(n_wps)
    return data


def run_auto_capture(rb, conn, waypoint_file=None, speed=30):
    """Run auto capture. If waypoint_file is None or empty, request waypoints
    from PC over the socket. Otherwise, load from local filesystem (legacy).

    기본(semi-auto): 각 capture pose로 이동 후 좌표를 표시하고, 사람이 'c'+Enter로
    확인해야 실제 촬영한다. `--noconfirm` 플래그를 주면 확인 없이 전부 자동 촬영한다.
    """
    confirm = '--noconfirm' not in sys.argv
    if not waypoint_file:
        data = request_waypoints_from_pc(conn)
        if data is None:
            return
    else:
        with open(waypoint_file, 'r') as f:
            data = json.load(f)

    # Multi-set joint-based: waypoints[] has per-waypoint set_index (5+ sets)
    waypoints = data.get('waypoints', [])
    if not waypoints or not any('set_index' in wp for wp in waypoints):
        print '[ERROR] waypoints missing set_index (multi-set format required)'
        send_json(conn, {"command": "quit"})
        return
    _run_auto_multiset(rb, conn, data, speed, confirm=confirm)


def _capture_at_pose(rb, conn, wp, sidx, place_j, set_cc,
                     cube_gripped, capture_block, grasp_id, confirm, label=''):
    """한 waypoint 로 이동 -> (확인) -> 촬영 -> 실패 시 manual recovery.

    이동 방식: wp 에 'capture_joints' 가 있으면 관절 이동(rb.move), 없고 'capture_tcp'
    만 있으면 TCP 직교 이동(rb.line). Phase A(placement)는 관절, Phase B(grip-sweep)는
    set 위치로 평행이동된 TCP 를 쓰므로 line 으로 실행된다.

    반환: 'success' | 'skip' | 'quit' | 'disconnect'
    cube_gripped/capture_block/grasp_id 는 프레임 태그로 do_capture 에 전달되어
    나중에 Step3 --capture_block 로 방법(a/b)을 분리 캘리브할 수 있게 한다.
    """
    cap_j = wp.get('capture_joints')
    cap_tcp = wp.get('capture_tcp')
    pose_idx = wp.get('capture_index', wp.get('pose_index'))
    print ''
    print '  -- {} capture (set={}, capture_index={}, block={}, move={}) --'.format(
        label, sidx, pose_idx, capture_block, 'joint' if cap_j is not None else 'tcp')
    try:
        if cap_j is not None:
            rb.move(Joint(*cap_j[:6]))
        else:
            # 안전 접근: 목표 TCP 의 +Z {approach}mm 위로 먼저 line 이동한 뒤 하강.
            tgt = [float(x) for x in cap_tcp[:6]]
            above = list(tgt)
            above[2] += TCP_APPROACH_Z_MM
            print '    (+Z {:.0f}mm approach above then descend)'.format(TCP_APPROACH_Z_MM)
            rb.line(Position(*above))
            time.sleep(0.2)
            rb.line(Position(*tgt))
    except Exception as e:
        print '  [WARN] move failed: {}. Skipping.'.format(e)
        return 'skip'
    time.sleep(0.5)
    show_pose()

    if confirm:
        action = None
        while action is None:
            ans = raw_input("  Capture? ('c'+Enter=촬영 / s=skip / q=quit): ").strip().lower()
            if ans in ('c', ''):
                action = 'capture'
            elif ans == 's':
                action = 'skip'
            elif ans == 'q':
                action = 'quit'
            else:
                print "  (c=촬영 / s=skip / q=quit 중 입력)"
        if action == 'skip':
            print '  -> skipped by user'
            return 'skip'
        if action == 'quit':
            print '  -> quit by user'
            return 'quit'

    cap_kwargs = {
        "set_cube_center": set_cc,
        "set_index": sidx,
        "set_joints": place_j,
        "set_tcp": None,
        "place_joints": place_j,
        "cube_gripped": cube_gripped,
        "capture_block": capture_block,
        "grasp_id": grasp_id,
    }
    status, _, _ = do_capture(conn, pose_idx, **cap_kwargs)
    if status is None:
        print '[Auto] disconnected, stopping.'
        return 'disconnect'
    if status == 'success':
        print '  [Auto] -> OK'
        return 'success'
    print '  [Auto] -> not detected; entering manual recovery'
    rec = manual_recover(rb, conn, pose_idx, cap_kwargs)
    if rec is None:
        return 'disconnect'
    if rec == 'success':
        return 'success'
    if rec == 'quit':
        return 'quit'
    return 'skip'


def _run_auto_multiset(rb, conn, data, speed, confirm=True,
                        z_clearance_mm=100.0,
                        z_transit_lift_mm=30.0):
    """Multi-set joint-based auto capture (start-command flow).

    각 set(waypoints[].set_index 그룹)마다, capture_block 태그로 두 촬영 방법을 한 번에:
      Phase B (B_eyetohand): 큐브를 그립한 채 각 grip-sweep pose로 이동하며 촬영
                             (cube_gripped=True). 고정 카메라가 움직이는 큐브를 관측
                             = eye-to-hand (method b). 이 set의 그립 하나 = grasp_id.
      -- place_joints 로 이동해 큐브를 바닥에 내려놓고(하강) tool4로 큐브중점 실측 ->
         그리퍼 오픈(큐브 릴리즈) -> +Z clearance --
      Phase A (A_placement): 큐브가 바닥에 놓인 상태로 각 뷰포인트에서 촬영
                             (cube_gripped=False) = placement (method a).
                             set_cube_center 는 위에서 실측한 값을 사용.
      -- 다음 set이 있으면 큐브 재-그립 후 +Z transit lift 하고 이동 --

    waypoint 의 capture_block == 'B_eyetohand' 이면 Phase B, 그 외/없음이면 Phase A.
    (capture_block 없는 구버전 파일은 전부 A_placement 로 동작 = 기존과 동일.)
    """
    waypoints = data.get('waypoints', [])
    if not waypoints:
        print '[ERROR] no waypoints'
        send_json(conn, {"command": "quit"})
        return

    # Group by set_index, preserving first-appearance order.
    sets_order = []
    by_set = {}
    for wp in waypoints:
        sidx = wp.get('set_index')
        if sidx is None:
            print '[ERROR] waypoint capture_index={} missing set_index'.format(wp.get('capture_index', wp.get('pose_index')))
            send_json(conn, {"command": "quit"})
            return
        if 'place_joints' not in wp or ('capture_joints' not in wp and 'capture_tcp' not in wp):
            print '[ERROR] waypoint capture_index={} missing place_joints or capture_joints/capture_tcp'.format(wp.get('capture_index', wp.get('pose_index')))
            send_json(conn, {"command": "quit"})
            return
        if sidx not in by_set:
            by_set[sidx] = []
            sets_order.append(sidx)
        by_set[sidx].append(wp)

    total_caps = len(waypoints)
    n_sets = len(sets_order)
    print ''
    print '=========================================='
    print '  Multi-Set Auto Capture'
    print '  - sets:     {} ({})'.format(n_sets, sets_order)
    print '  - captures: {}'.format(total_caps)
    print '  - speed:    {}'.format(speed)
    print '  - +Z capture clearance: {}mm'.format(z_clearance_mm)
    print '  - +Z transit lift:      {}mm'.format(z_transit_lift_mm)
    print '=========================================='
    print ''
    print 'PRECONDITION: cube must be gripped before starting.'
    print 'Per set: Phase B grip-sweep (cube held) -> place & release -> Phase A placement.'
    raw_input('Press ENTER to confirm cube is gripped and start...')

    rb.override(speed)
    success = 0
    skipped = 0

    # Pre-loop: cube is on the floor with the gripper around it (user just placed
    # it there). Lift +z_transit_lift_mm before the first joint transit to make
    # sure the cube clears the floor on the way to set 0's place_joints.
    print '[Auto] +Z {:.0f}mm initial transit lift (cube clears floor)'.format(z_transit_lift_mm)
    try:
        cur = Position(*rb.getpos().pos2list()[:6])
        rb.line(cur.offset(dz=z_transit_lift_mm))
    except Exception as e:
        print '[WARN] initial transit lift failed: {} (continuing)'.format(e)
    time.sleep(0.3)

    for si, sidx in enumerate(sets_order):
        wps = by_set[sidx]
        place_j = wps[0]['place_joints']
        # set별 큐브 중점: waypoint에 저장된 값 우선, 없으면 파일 최상위로 폴백.
        set_cc = wps[0].get('set_cube_center_6dof') or data.get('set_cube_center')
        # 이 set 의 그립(Phase B 스윕) 하나 = 하나의 grasp. 재-그립 시 gripper->cube
        # 변환이 조금 달라지므로 set 마다 grasp_id 를 달리해 Step3 가 구분하게 한다.
        grasp_id = si

        # capture_block 으로 두 방법을 분리: B(그립 스윕) 먼저, A(placement) 나중.
        block_b = [wp for wp in wps if wp.get('capture_block') == 'B_eyetohand']
        block_a = [wp for wp in wps if wp.get('capture_block') != 'B_eyetohand']

        print ''
        print '======== SET {}/{} (set_index={}: B={} grip-sweep, A={} placement) ========'.format(
            si + 1, n_sets, sidx, len(block_b), len(block_a))

        # ---- 안전 전이: B 는 TCP line 이라 먼 거리/특이점에서 실패할 수 있으므로,
        #      B 시작 전에 관절 이동으로 이 set 영역(place_joints)까지 안전하게 이동한다.
        #      이후 각 B line 은 이 set 위에서의 국소(주로 수직) 이동이 된다.
        #      (큐브는 그립된 상태로 잠시 바닥 근처로 내려갔다가 B 접근에서 다시 올라간다.) ----
        if block_b:
            print '[Auto] -> set {} region via joints (safe transit before B line moves)'.format(sidx)
            rb.move(Joint(*place_j[:6]))
            time.sleep(0.3)

        # ---- Phase B: eye-to-hand. 큐브를 그립한 채(최초 그립 또는 이전 set 재-그립)
        #      각 sweep pose 로 이동하며 촬영. 고정 카메라가 움직이는 큐브를 관측. ----
        if block_b:
            print '[Auto] --- Phase B: {} grip-sweep captures (cube gripped, grasp_id={}) ---'.format(
                len(block_b), grasp_id)
        for wi, wp in enumerate(block_b):
            r = _capture_at_pose(
                rb, conn, wp, sidx, place_j, set_cc,
                cube_gripped=True, capture_block='B_eyetohand', grasp_id=grasp_id,
                confirm=confirm, label='B {}/{}'.format(wi + 1, len(block_b)))
            if r == 'success':
                success += 1
            elif r == 'skip':
                skipped += 1
            elif r == 'quit':
                send_json(conn, {"command": "quit"})
                return
            elif r == 'disconnect':
                return

        # ---- 큐브 내려놓기: place_joints 로 이동(하강) -> tool4 로 큐브중점 실측 ->
        #      그리퍼 오픈(릴리즈) -> +Z clearance ----
        print '[Auto] -> set {} place_joints (lower cube to floor)'.format(sidx)
        rb.move(Joint(*place_j[:6]))
        time.sleep(0.5)
        place_tcp = get_tcp()  # 재-그립 시 +Z 위에서 line 접근하기 위한 기준 TCP
        try:
            measured_cc = get_cube_center()
            print '[Auto] measured cube center (tool4): ' + fmt6(measured_cc)
            set_cc = measured_cc
        except Exception as e:
            print '[WARN] get_cube_center() failed ({}); keeping nominal set_cube_center'.format(e)

        print '[Auto] gripper OPEN (release cube on floor)'
        gripper_open()
        time.sleep(0.3)

        print '[Auto] -> +Z {:.0f}mm clearance'.format(z_clearance_mm)
        try:
            cur = Position(*rb.getpos().pos2list()[:6])
            rb.line(cur.offset(dz=z_clearance_mm))
        except Exception as e:
            print '[WARN] +Z clearance failed: {} (continuing)'.format(e)
        time.sleep(0.5)

        # ---- Phase A: placement. 큐브는 바닥, 그리퍼 카메라가 각 뷰포인트에서 촬영. ----
        if block_a:
            print '[Auto] --- Phase A: {} placement captures (cube on floor) ---'.format(len(block_a))
        for wi, wp in enumerate(block_a):
            r = _capture_at_pose(
                rb, conn, wp, sidx, place_j, set_cc,
                cube_gripped=False, capture_block='A_placement', grasp_id=grasp_id,
                confirm=confirm, label='A {}/{}'.format(wi + 1, len(block_a)))
            if r == 'success':
                success += 1
            elif r == 'skip':
                skipped += 1
            elif r == 'quit':
                send_json(conn, {"command": "quit"})
                return
            elif r == 'disconnect':
                return

        # ---- 다음 set 이 있으면: 큐브 재-그립 후 +Z transit lift 하고 이동 ----
        if si < n_sets - 1:
            print '[Auto] re-grip cube (+Z {:.0f}mm approach above)'.format(
                GRIP_APPROACH_Z_MM)
            approach_and_close_gripper(rb, place_j, place_tcp)
            time.sleep(0.3)
            # Lift +Z transit_lift_mm so the cube clears the floor during the
            # joint transit to the next set's place_joints.
            print '[Auto] +Z {:.0f}mm transit lift (cube clears floor)'.format(z_transit_lift_mm)
            try:
                cur = Position(*rb.getpos().pos2list()[:6])
                rb.line(cur.offset(dz=z_transit_lift_mm))
            except Exception as e:
                print '[WARN] transit lift failed: {} (continuing)'.format(e)
            time.sleep(0.3)

    # Final state: gripper open at last set's place_joints (cube on floor).
    send_json(conn, {"command": "quit"})
    print ''
    print '=========================================='
    print '  Multi-Set Auto Complete'
    print '  - success: {}/{}'.format(success, total_caps)
    print '  - skipped: {}'.format(skipped)
    print '=========================================='


# ── Main ──

def main():
    try:
        rbs = RobSys()
        rbs.open()

        global rb
        rb = i611Robot()
        Base()
        rb.open()
        IOinit(rb)

        m = MotionParam(jnt_speed=100, lin_speed=100, pose_speed=100,
                        overlap=0, acctime=0.8, dacctime=0.8)
        rb.motionparam(m)
        rb.override(100)

        rb.settool(1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        rb.settool(2, 0.0, 35.0, 330.0, 0.0, 0.0, 0.0)
        rb.settool(3, 0.0, 0.0, TOOL_GRIPPER_Z, 0.0, 0.0, 0.0)
        rb.settool(4, 0.0, 0.0, TOOL_CUBE_CENTER_Z, 0.0, 0.0, 0.0)
        rb.changetool(3)
        rb.use_mt(True)
        
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(1)
        print "Server on port {}. Waiting...".format(PORT)

        conn, addr = s.accept()
        print "Client: {}".format(addr)

        # Auto mode
        if '--auto' in sys.argv:
            idx = sys.argv.index('--auto')
            auto_file = sys.argv[idx + 1] if idx + 1 < len(sys.argv) else 'capture_waypoints.json'
            auto_speed = 30
            if '--speed' in sys.argv:
                sidx = sys.argv.index('--speed')
                if sidx + 1 < len(sys.argv):
                    auto_speed = int(sys.argv[sidx + 1])
            try:
                run_auto_capture(rb, conn, auto_file, auto_speed)
            finally:
                try:
                    conn.close()
                    s.close()
                except Exception:
                    pass
            return

        # State
        capture_count = 0
        set_index = -1
        move_history = []
        home_pose = None
        home_joints = None
        set_cube_center = None
        last_place_joints = None
        waypoints = []
        # capture tagging: A_placement (cube released, method a) vs B_eyetohand
        # (cube gripped, robot sweeps, method b). gc(grip)/go(release) toggle gripped.
        capture_block = "A_placement"
        grasp_id = 0
        cube_gripped = False
        # teach 기록 리스트 (PC 로 전송해 PC 에만 저장; 로봇 로컬 파일 없음)
        capture_poses = []           # recpose: A 뷰포인트 풀
        capture_sets = []            # recset : 큐브 set 배치
        grip_poses = []              # recgrip: B 그립-스윕 포즈 풀

        print ''
        print '=========================================='
        print '  p <a>,<v> / j <a>,<v>  : rel move'
        print '  gotop x,y,z[,rz,ry,rx] : TCP abs move'
        print '  gotoj d1,d2,d3,d4,d5,d6: joint abs move'
        print '  show / speed <0-100>'
        print '  c: capture  set: save TCP+cube'
        print '  go: grip open(release)  gc: grip close(grip)'
        print '  block a|b : A=placement(method a) / B=eye-to-hand sweep(method b)'
        print '    (b)eye-to-hand: gc(grip cube) -> block b -> jog widely(z>=150mm,'
        print '     tilt>=30deg) -> c at each pose (cube must stay visible to fixed cams)'
        print '  undo [N|all|<axes>|set]  q: quit'
        print '  recpose | rp          -> A 촬영 뷰포인트 기록 (-> capture_poses.json)'
        print '  recgrip | rg          -> B 그립-스윕 포즈 기록 (-> grip_poses.json)'
        print '  recset  | rs          -> 큐브 set 배치 기록 (-> capture_sets.json)'
        print '    ( ...undo | ...list 지원 )'
        print '  start                 -> auto capture (PC sends waypoints)'
        print '  start <path> [speed]  -> auto capture (local file)'
        print '    (cube must be gripped before start)'
        print '    (각 pose 이동 후 c+Enter 확인 시 촬영; --noconfirm 로 전자동)'
        print '=========================================='
        print ''

        show_pose()

        while True:
            try:
                cmd = raw_input('> ').strip()
            except EOFError:
                break
            if not cmd:
                continue

            cl = cmd.lower()

            # Quit
            if cl == 'q':
                send_json(conn, {"command": "quit"})
                break

            # Start: multi-set auto capture.
            #   "start"         -> request waypoints from PC over socket
            #   "start <path>"  -> load local file (legacy/testing)
            #   "start <path> <speed>" or "start - <speed>" supported
            elif cl.startswith('start'):
                parts = cmd.split(None, 2)
                wp_file = None
                spd = 30
                if len(parts) >= 2 and parts[1] != '-':
                    wp_file = parts[1]
                if len(parts) >= 3:
                    try:
                        spd = int(parts[2])
                    except ValueError:
                        print '[ERROR] invalid speed: {}'.format(parts[2])
                        continue
                try:
                    run_auto_capture(rb, conn, wp_file, spd)
                except IOError as e:
                    print '[ERROR] cannot read {}: {}'.format(wp_file, e)
                    continue
                break

            # Show
            elif cl == 'show':
                show_pose()
                if home_pose is not None:
                    print '  [Set #{}] TCP:  {}'.format(set_index, fmt6(home_pose))
                if set_cube_center is not None:
                    print '  [Set #{}] Cube: [{:.1f}, {:.1f}, {:.1f}]'.format(
                        set_index, set_cube_center[0], set_cube_center[1], set_cube_center[2])

            # Speed
            elif cl.startswith('speed'):
                try:
                    spd = int(cmd.split()[1])
                    rb.override(spd)
                    print 'Speed: {}'.format(spd)
                except Exception:
                    print 'Usage: speed <0-100>'

            # Set
            elif cl == 'set':
                set_index += 1
                home_pose = get_tcp()
                home_joints = get_joints()
                set_cube_center = get_cube_center()
                move_history = []
                print ''
                print '*** Set #{} saved ***'.format(set_index)
                print '  TCP:    {}'.format(fmt6(home_pose))
                print '  Joints: [{:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}]'.format(
                    home_joints[0], home_joints[1], home_joints[2],
                    home_joints[3], home_joints[4], home_joints[5])
                print '  Cube:   [{:.1f}, {:.1f}, {:.1f}] (offset={:.0f}mm)'.format(
                    set_cube_center[0], set_cube_center[1], set_cube_center[2],
                    CUBE_CENTER_OFFSET_Z)

            # Capture block toggle (a = placement / b = eye-to-hand sweep)
            elif cl.startswith('block'):
                parts = cl.split()
                if len(parts) >= 2 and parts[1] in ('a', 'b'):
                    capture_block = "B_eyetohand" if parts[1] == 'b' else "A_placement"
                    print 'capture_block = {} (grasp_id={}, cube_gripped={})'.format(
                        capture_block, grasp_id, cube_gripped)
                else:
                    print 'Usage: block a | block b   (current: {})'.format(capture_block)

            # Gripper
            elif cl == 'go':
                last_place_joints = get_joints()
                gripper_open()
                cube_gripped = False        # cube released on the table

            elif cl == 'gc':
                gripper_close()
                cube_gripped = True         # cube now rigidly held
                grasp_id += 1               # new grasp = new eye-to-hand target transform
                print '[grip] cube_gripped=True, grasp_id={}'.format(grasp_id)

            # Capture
            elif cl == 'c':
                status, tcp, cube_tcp = do_capture(
                    conn, capture_count, set_cube_center,
                    set_index if set_index >= 0 else None,
                    set_joints=home_joints, set_tcp=home_pose,
                    place_joints=last_place_joints,
                    cube_gripped=cube_gripped,
                    capture_block=capture_block,
                    grasp_id=grasp_id)
                if status is None:
                    break
                wp = {
                    "capture_index": capture_count,
                    "capture_joints": get_joints(),
                    "capture_tcp": tcp,
                    "cube_center_6dof": cube_tcp,
                }
                if last_place_joints is not None:
                    wp["place_joints"] = last_place_joints
                else:
                    print '  [WARN] go not called before capture'
                waypoints.append(wp)
                capture_count += 1

            # Undo
            elif cl.startswith('undo'):
                args = cl.split()[1:]

                if args == ['set']:
                    if home_pose is None:
                        print 'No set saved.'
                    else:
                        target = Position(home_pose[0], home_pose[1], 0.0,
                                          home_pose[3], home_pose[4], home_pose[5])
                        rb.line(target)
                        move_history = []
                        show_pose()

                elif not move_history:
                    print 'Nothing to undo.'

                else:
                    if not args:
                        undo_one(move_history.pop())

                    elif args[0] == 'all':
                        while move_history:
                            undo_one(move_history.pop())

                    elif args[0] in VALID_AXES:
                        axis_set = set(a for a in args if a in VALID_AXES)
                        indices = [i for i, h in enumerate(move_history) if h[1] in axis_set]
                        if not indices:
                            print 'No moves on [{}]'.format(','.join(sorted(axis_set)))
                        else:
                            for idx in reversed(indices):
                                undo_one(move_history.pop(idx))
                    else:
                        try:
                            count = min(int(args[0]), len(move_history))
                        except ValueError:
                            print 'Usage: undo [N|all|<axes>|set]'
                            continue
                        for _ in range(count):
                            undo_one(move_history.pop())

                    show_pose()

            # Goto - joint absolute move
            elif cl.startswith('gotoj '):
                try:
                    vals = [float(v.strip()) for v in cmd[6:].strip().split(',')]
                    if len(vals) != 6:
                        print 'Usage: gotoj d1,d2,d3,d4,d5,d6'
                        continue
                    rb.move(Joint(*vals))
                    show_pose()
                except Exception as e:
                    print 'Error: {}'.format(e)

            # Goto - TCP absolute move (gotop / 기존 goto는 별칭으로 호환 유지)
            elif cl.startswith('gotop ') or cl.startswith('goto '):
                try:
                    rest = cmd[6:] if cl.startswith('gotop ') else cmd[5:]
                    vals = [float(v.strip()) for v in rest.strip().split(',')]
                    if len(vals) == 6:
                        rb.line(Position(*vals))
                    elif len(vals) == 3:
                        tcp = get_tcp()
                        rb.line(Position(vals[0], vals[1], vals[2], tcp[3], tcp[4], tcp[5]))
                    else:
                        print 'Usage: gotop x,y,z[,rz,ry,rx]'
                        continue
                    show_pose()
                except Exception as e:
                    print 'Error: {}'.format(e)

            # TCP move
            elif cl.startswith('p '):
                try:
                    parts = cmd[2:].strip().split(',')
                    axis, value = parts[0].strip(), float(parts[1].strip())
                    move_tcp(axis, value)
                    move_history.append(('p', axis, value))
                    show_pose()
                except Exception as e:
                    print 'Error: {}. Usage: p <axis>,<value>'.format(e)

            # Joint move
            elif cl.startswith('j '):
                try:
                    parts = cmd[2:].strip().split(',')
                    axis, value = parts[0].strip(), float(parts[1].strip())
                    move_joint(axis, value)
                    move_history.append(('j', axis, value))
                    show_pose()
                except Exception as e:
                    print 'Error: {}. Usage: j <axis>,<value>'.format(e)

            # Record capture viewpoint pose (teach pool for waypoint generation)
            #   recpose | rp          -> 현재 포즈를 풀에 기록 + capture_poses.json 저장
            #   recpose undo | rp undo-> 마지막 기록 취소
            #   recpose list | rp list-> 기록된 포즈 목록
            elif cl == 'recpose' or cl == 'rp' or cl.startswith('recpose ') or cl.startswith('rp '):
                parts = cl.split()
                sub = parts[1] if len(parts) >= 2 else None
                if sub == 'undo':
                    if capture_poses:
                        capture_poses.pop()
                        # pose_index 재부여(0..N-1 유지)
                        for i, p in enumerate(capture_poses):
                            p['pose_index'] = i
                        send_teach(conn, 'pose', capture_poses)
                        print '[recpose] undo -> {} poses (sent to PC)'.format(len(capture_poses))
                    else:
                        print '[recpose] nothing to undo'
                elif sub == 'list':
                    print '[recpose] {} poses recorded:'.format(len(capture_poses))
                    for p in capture_poses:
                        print '  #{} joints={}'.format(p['pose_index'], fmt6(p['capture_joints']))
                else:
                    pose = {
                        "pose_index": len(capture_poses),
                        "capture_joints": get_joints(),
                        "capture_tcp": get_tcp(),
                        "cube_center_6dof": get_cube_center(),
                    }
                    capture_poses.append(pose)
                    send_teach(conn, 'pose', capture_poses)
                    print ''
                    print '[recpose] #{} saved'.format(pose['pose_index'])
                    print '  joints: {}'.format(fmt6(pose['capture_joints']))
                    print '  tcp:    {}'.format(fmt6(pose['capture_tcp']))
                    print '  cube:   {}'.format(fmt6(pose['cube_center_6dof']))
                    print '  -> sent to PC ({} poses total)'.format(len(capture_poses))

            # Record grip-sweep (eye-to-hand, block B) pose for waypoint gen.
            #   recgrip | rg           -> 현재(큐브 그립 상태) 스윕 포즈 기록
            #   recgrip undo | rg undo -> 마지막 취소
            #   recgrip list | rg list -> 목록
            elif cl == 'recgrip' or cl == 'rg' or cl.startswith('recgrip ') or cl.startswith('rg '):
                parts = cl.split()
                sub = parts[1] if len(parts) >= 2 else None
                if sub == 'undo':
                    if grip_poses:
                        grip_poses.pop()
                        for i, p in enumerate(grip_poses):
                            p['pose_index'] = i
                        send_teach(conn, 'grip', grip_poses)
                        print '[recgrip] undo -> {} poses (sent to PC)'.format(len(grip_poses))
                    else:
                        print '[recgrip] nothing to undo'
                elif sub == 'list':
                    print '[recgrip] {} grip-sweep poses recorded:'.format(len(grip_poses))
                    for p in grip_poses:
                        print '  #{} joints={}'.format(p['pose_index'], fmt6(p['capture_joints']))
                else:
                    pose = {
                        "pose_index": len(grip_poses),
                        "capture_joints": get_joints(),
                        "capture_tcp": get_tcp(),
                        "cube_center_6dof": get_cube_center(),
                    }
                    grip_poses.append(pose)
                    send_teach(conn, 'grip', grip_poses)
                    print ''
                    print '[recgrip] #{} saved (grip-sweep, block B)'.format(pose['pose_index'])
                    print '  joints: {}'.format(fmt6(pose['capture_joints']))
                    print '  tcp:    {}'.format(fmt6(pose['capture_tcp']))
                    print '  cube:   {}'.format(fmt6(pose['cube_center_6dof']))
                    print '  -> sent to PC ({} poses total)'.format(len(grip_poses))

            # Record cube set placement (place_joints + cube center) for waypoint gen.
            #   recset | rs           -> 현재(큐브 그립+바닥에 놓은 상태) 기록
            #   recset undo | rs undo -> 마지막 취소
            #   recset list | rs list -> 목록
            elif cl == 'recset' or cl == 'rs' or cl.startswith('recset ') or cl.startswith('rs '):
                parts = cl.split()
                sub = parts[1] if len(parts) >= 2 else None
                if sub == 'undo':
                    if capture_sets:
                        capture_sets.pop()
                        for i, sset in enumerate(capture_sets):
                            sset['set_index'] = i
                        send_teach(conn, 'set', capture_sets)
                        print '[recset] undo -> {} sets (sent to PC)'.format(len(capture_sets))
                    else:
                        print '[recset] nothing to undo'
                elif sub == 'list':
                    print '[recset] {} sets recorded:'.format(len(capture_sets))
                    for sset in capture_sets:
                        print '  set#{} cube={}'.format(sset['set_index'], fmt6(sset['set_cube_center_6dof']))
                else:
                    sset = {
                        "set_index": len(capture_sets),
                        "place_joints": get_joints(),
                        "place_tcp": get_tcp(),
                        "set_cube_center_6dof": get_cube_center(),
                    }
                    capture_sets.append(sset)
                    send_teach(conn, 'set', capture_sets)
                    print ''
                    print '[recset] set#{} saved'.format(sset['set_index'])
                    print '  place_joints: {}'.format(fmt6(sset['place_joints']))
                    print '  place_tcp:    {}'.format(fmt6(sset['place_tcp']))
                    print '  cube_center:  {}'.format(fmt6(sset['set_cube_center_6dof']))
                    print '  -> sent to PC ({} sets total)'.format(len(capture_sets))

            else:
                print 'Unknown: {}'.format(cmd)

        # Save waypoints
        if waypoints:
            save_data = {
                "set_joints": home_joints,
                "set_tcp": home_pose,
                "set_cube_center": set_cube_center,
                "waypoints": waypoints,
            }
            with open('capture_waypoints.json', 'w') as f:
                json.dump(save_data, f, indent=2)
            print '\nWaypoints saved: {} poses'.format(len(waypoints))

        print '\nTotal captures: {}'.format(capture_count)

    except KeyboardInterrupt:
        print '\nInterrupted'
        try:
            send_json(conn, {"command": "quit"})
        except Exception:
            pass
    except Robot_emo as e:
        print(e)
    except Robot_error as e:
        print(e)
    except Robot_fatalerror as e:
        print(e)
    except Exception as e:
        print(e)
    finally:
        try:
            rb.exit(0)
            rb.close()
            rbs.close()
        except Exception:
            pass
        try:
            s.close()
        except Exception:
            pass


if __name__ == '__main__':
    main()
