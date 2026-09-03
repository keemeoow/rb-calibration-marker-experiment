#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ur3_calibration/gripper.py — Robotiq URCap 소켓 프로토콜(포트 63352) 그리퍼 제어

/home/sstone/2026SimtoReal/ur3_test/control/keyboard_teleop_gui.py 의
RobotiqGripper 클래스를 그대로 재사용한다. 티치펜던트에서 그리퍼가
activate(ACT) 된 상태여야 한다.
"""

import socket

import numpy as np

DEFAULT_PORT = 63352


class RobotiqGripper:
    """POS(0=open .. 255=closed)만 제어하는 최소 래퍼."""

    def __init__(self, host: str, port: int = DEFAULT_PORT):
        self.sock = socket.create_connection((host, port), timeout=2.0)
        self.sock.settimeout(1.0)
        if self._get("ACT") != 1:
            raise RuntimeError(
                "그리퍼가 activate(ACT)되지 않았습니다. 티치펜던트에서 먼저 activate 하세요."
            )
        self._set("GTO", 1)

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

    def close(self) -> None:
        try:
            self.sock.close()
        except Exception:
            pass
