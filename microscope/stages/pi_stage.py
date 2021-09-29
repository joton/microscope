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

"""PI piezo stage devices.

This class allows piezo stages to be controlled over Pyro.
The construction of microscope device could be done as:

PIE754_CONF = dict(url='socket://127.0.0.2:50000')
DEVICES = [
    device(pi_stage.PIe754, host, port, conf=PIE754_CONF),
]

# TO BE TESTED #
For serial controllers one could use ser2net to allow socket communication.
The configuration would be:
/etc/ser2net.conf
3000:telnet:600:/dev/ttyS0:19200 8DATABITS NONE 1STOPBIT

And construct the device as:
    
PIM687_CONF = dict(url='rfc2217://127.0.0.3:3000')
DEVICES = [
    device(pi_stage.PIm687, host, port, conf=PIM687_CONF),
]

"""

import logging
import serial
import threading
import microscope
import typing
from .pi_message import PI_CTRL_ERROR

_logger = logging.getLogger(__name__)


class PIaxis(microscope.abc.StageAxis):
    def __init__(self, stage, axis):
        self.stage = stage
        self.axis = axis
        self.pos = 0

    def refresh(self):
        self.pos = self.stage._axis_get("POS?", self.axis)

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
        return self.pos

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


class PIstage(microscope.abc.Stage):

    _comm = None
    REFRESH_RATE = 0.2

    def __init__(self, url=None, **kwargs):
        super().__init__(**kwargs)
        self.url = url
        self.comm_lock = threading.Lock()

        self.axis = {}
        for k, v in self.axes_ids.items():
            self.axis[k] = PIaxis(self, v)

        self.initialize()

    def initialize(self):
        """Initialise the stage.

        Open the communication and initialize main parameters.

        """

        try:
            self._comm = serial.serial_for_url(self.url)
            self._comm.timeout = 0.1
        except Exception as e:
            if hasattr(e, "message"):
                msg = e.message
            else:
                msg = e
            _logger.error("ERROR: {}\n".format(msg))

        # Set all axis in close-loop
        for name, axis in self.axis.items():
            axis.close_loop = True

        # set refresh loop
        self.refresh_loop = True
        self.thread = threading.Thread(target=self._refresh)
        self.thread.start()

    def _do_shutdown(self) -> None:
        self.refresh_loop = False
        self.thread.join(timeout=2)
        if self._comm is not None:
            with self.comm_lock:
                self._comm.close()

    def _send_cmd(self, cmd):
        with self.comm_lock:
            self._comm.write(cmd.encode() + b"\n")

    def _ask_cmd(self, cmd):
        try:
            with self.comm_lock:
                self._comm.write(cmd.encode() + b"\n")
                ans = self._comm.read(1024)
        except:
            raise
        return ans.decode()

    def _axis_get(self, cmd, axis):
        cmd = cmd + " {}".format(int(axis))
        ans = self._ask_cmd(cmd)
        return float(ans.split("=")[1].split("\n")[0].strip())

    def _axis_set(self, cmd, axis, val):
        cmd = cmd + " {} {}".format(int(axis), val)
        self._send_cmd(cmd)
        return self.get_error()

    def get_error(self):
        err = int(self._ask_cmd("ERR?").strip())
        if err > 0:
            msg = PI_CTRL_ERROR[err]
            _logger.error("ERROR[{}] {}\n".format(err, msg))
        return err

    def _refresh(self):
        while self.refresh_loop:
            for name, axis in self.axis.items():
                axis.refresh()
        
    @property
    def axes(self) -> typing.Mapping[str, microscope.abc.StageAxis]:
        return self.axis

    def move_by(self, delta: typing.Mapping[str, float]) -> None:
        for name, d in delta.items():
            self.axis[name].move_by(d)

    def move_to(self, position: typing.Mapping[str, float]) -> None:
        for name, pos in position.items():
            self.axis[name].move_to(pos)


class PIe754(PIstage):
    axes_ids = {"z": 1}


class PIm687(PIstage):
    axes_ids = {"y": 1, "x": 2}
