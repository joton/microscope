#!/usr/bin/env python3

## Copyright (C) 2021 ALBA Synchrotron
##
## This file is part of Microscope.
##
## Microscope is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## Microscope is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with Microscope.  If not, see <http://www.gnu.org/licenses/>.

"""PI E754 piezo stage device.

This class allows piezo stages controlled with PI E754 to be exposed over Pyro.
"""

import logging
import socket
import microscope
import typing
from .pi_message import PI_CTRL_ERROR

_logger = logging.getLogger(__name__)


class PIaxis(microscope.abc.StageAxis):
    def __init__(self, stage, axis):
        self.stage = stage
        self.axis = axis

    def move_by(self, delta: float) -> None:
        if self.close_loop:
            self.stage._axis_set("MVR", self.axis, delta)
        else:
            self.stage._axis_set("SVR", self.axis, delta)

    def move_to(self, pos: float) -> None:
        if self.close_loop:
            self.stage._axis_set("MOV", self.axis, pos)
        else:
            self.stage._axis_set("SVA", self.axis, pos)

    @property
    def position(self) -> float:
        return self.stage._axis_get("POS?", self.axis)

    @property
    def velocity(self) -> float:
        return self.stage._axis_get("VEL?", self.axis)

    @velocity.setter
    def velocity(self, vel) -> None:
        self.stage._axis_set("VEL", self.axis, pos)

    @property
    def limits(self) -> microscope.AxisLimits:
        lower = self.stage._axis_get("TMN?", self.axis)
        upper = self.stage._axis_get("TMX?", self.axis)
        return microscope.AxisLimits(lower, upper)

    @property
    def close_loop(self) -> bool:
        return self.stage._axis_get("SVO?", self.axis) == 1

    @close_loop.setter
    def close_loop(self, on) -> None:
        on = 1 if bool(on) else 0
        self.stage._axis_set("SVO", self.axis, on)

    @property
    def zero_cal(self) -> bool:
        return self.stage._axis_get("ATZ?", self.axis) == 1

    @zero_cal.setter
    def zero_cal(self, on) -> None:
        if bool(on):
            self.stage._send_cmd("ATZ {}".format(int(self.axis)))


class PIe754(microscope.abc.Stage):

    _socket = None

    # TODO: understand who is passing index on construction.
    def __init__(self, host=None, port=None, index=0):
        super().__init__()
        self.host = host
        self.port = port

        self.axis = {"z": PIaxis(self, 1)}

        self.initialize()

    def initialize(self):
        """Initialise the stage.

        Open the connection and initialize main parameters.

        """
        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._socket.connect((self.host, int(self.port)))
        except socket.error as msg:
            _logger.error("ERROR: {}\n".format(msg))

        # Set all axis in close-loop
        for name, axis in self.axis.items():
            axis.close_loop = True

    def _do_shutdown(self) -> None:
        if self._socket is not None:
            self._socket.close()

    def _send_cmd(self, cmd):
        self._socket.send(cmd.encode() + b"\n")

    def _ask_cmd(self, cmd):
        try:
            self._send_cmd(cmd)
            ans = self._socket.recv(1024)
        except:
            raise
        return ans.decode()

    def _axis_get(self, cmd, axis):
        cmd = cmd + " {}".format(int(axis))
        ans = self._ask_cmd(cmd)
        return float(ans.split("=")[1].strip())

    def _axis_set(self, cmd, axis, val):
        cmd = cmd + " {} {}".format(int(axis), val)
        self._send_cmd(cmd)
        return self.get_error()

    def get_error(self):
        err = int(self._ask_cmd("ERR?"))
        if err > 0:
            msg = PI_CTRL_ERROR[err]
            _logger.error("ERROR[{}] {}\n".format(err, msg))
        return err

    @property
    def axes(self) -> typing.Mapping[str, microscope.abc.StageAxis]:
        return self.axis

    def move_by(self, delta: typing.Mapping[str, float]) -> None:
        for name, pos in position:
            self.axis[name].move_by(pos)

    def move_to(self, position: typing.Mapping[str, float]) -> None:
        for name, pos in position:
            self.axis[name].move_to(pos)
