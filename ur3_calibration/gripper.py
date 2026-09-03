#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ur3_calibration/gripper.py — Robotiq URCap 소켓 프로토콜(포트 63352) 그리퍼 제어

/home/sstone/2026SimtoReal/ur3_test/control/keyboard_teleop_gui.py 의
RobotiqGripper 클래스를 그대로 재사용한다. 티치펜던트에서 그리퍼가
activate(ACT) 된 상태여야 한다.
"""

import socket
import time

import numpy as np

DEFAULT_PORT = 63352
DEFAULT_SPEED = 30  # 0(가장 느림)-255(가장 빠름). 낮게 잡아 열고/닫을 때 다 천천히 움직이게 한다.


class RobotiqGripper:
    """POS(0=open .. 255=closed)만 제어하는 최소 래퍼."""

    def __init__(self, host: str, port: int = DEFAULT_PORT, speed: int = DEFAULT_SPEED):
        self.sock = socket.create_connection((host, port), timeout=2.0)
        self.sock.settimeout(1.0)
        if self._get("ACT") != 1:
            raise RuntimeError(
                "그리퍼가 activate(ACT)되지 않았습니다. 티치펜던트에서 먼저 activate 하세요."
            )
        self._set("GTO", 1)
        self._set("SPE", int(np.clip(speed, 0, 255)))

    def _cmd(self, line: str) -> str:
        self.sock.sendall((line + "\n").encode("ascii"))
        return self.sock.recv(128).decode("ascii").strip()

    def _get(self, var: str) -> int:
        response = self._cmd(f"GET {var}").split()
        return int(response[1])

    def _set(self, var: str, value: int) -> None:
        self._cmd(f"SET {var} {value}")

    def get_pos(self) -> int:
        return self._get("POS")

    def set_pos(self, pos: float) -> None:
        self._set("POS", int(round(np.clip(pos, 0, 255))))

    def wait_until_stopped(self, timeout: float = 3.0, poll_interval: float = 0.05,
                            initial_delay: float = 0.2) -> int:
        """물리적으로 움직임이 멈출 때까지 대기하고 마지막 OBJ 상태를 반환.

        set_pos()는 명령만 보내고 바로 리턴하므로, 그 직후 바로 다음 이동
        명령을 내리면 그리퍼가 다 닫히기/열리기 전에 팔이 움직여버린다.
        OBJ 레지스터(0=이동 중, 1/2=물체에 걸려 정지, 3=접촉 없이 목표 도달)가
        0이 아니게 될 때까지 기다리면 실제로 멈춘 뒤에만 다음 동작으로 넘어간다.

        set_pos() 직후 곧바로 OBJ를 읽으면 그리퍼가 아직 새 명령을 반영하기
        전이라 "이전 명령이 끝났을 때"의 값(0이 아님)이 그대로 남아있어서
        바로 통과해버리는 경우가 있다. 그래서 먼저 initial_delay만큼 기다려
        모션이 실제로 시작될 시간을 준 뒤에 폴링을 시작한다.
        """
        time.sleep(initial_delay)
        start = time.time()
        obj = self._get("OBJ")
        while obj == 0 and time.time() - start < timeout:
            time.sleep(poll_interval)
            obj = self._get("OBJ")
        return obj

    def close(self) -> None:
        try:
            self.sock.close()
        except Exception:
            pass
