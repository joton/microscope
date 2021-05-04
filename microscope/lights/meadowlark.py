#!/usr/bin/env python3

# Copyright (C) 2021 ALBA Synchrotron
#
# This file is part of Microscope.
#
# Microscope is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# Microscope is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with Microscope.  If not, see <http://www.gnu.org/licenses/>.

"""Meadowlark spatial light modulator device.

This class allows slm to be exposed over Pyro.
"""

import logging
import microscope
import microscope._utils
import os
import socket
import threading
import numpy as np

from PyQt5.QtWidgets import QApplication, QDesktopWidget, QLabel, QMainWindow
from PyQt5.QtGui import QImage, QPixmap

_logger = logging.getLogger(__name__)


class HDMIslm(microscope.abc.Device):

    # TODO: understand who is passing index on construction.
    def __init__(self, display_monitor=1, path="patterns", index=0):
        super().__init__()

        # setup the class
        self.patterns_path = path
        self.display_monitor = display_monitor  # the number of the monitor
        self.patterns = dict(enumerate(os.listdir(self.patterns_path)))
        self.sequence = dict(enumerate([(0, 0, 1)]))
        self.idx_image = 0

        self.initialize()

    def _run_qt(self):
        """Initialise the slm.
        Open the connection and initialize main parameters.
        """
        self.app = QApplication([])
        self.label = QLabel()

        widget = QMainWindow()  # define your widget

        image = QImage(f"{self.patterns_path}/{self.patterns[0]}")
        self.label.setPixmap(QPixmap.fromImage(image))

        widget.setCentralWidget(self.label)
        monitor = QDesktopWidget().screenGeometry(self.display_monitor)
        widget.move(monitor.left(), monitor.top())
        widget.showFullScreen()

        # TODO: Test if thread is necessary and works.
        self.app.exec_()

    def initialize(self):
        threading.Thread(target=self._run_qt).start()

    def _do_shutdown(self) -> None:
        self.app.quit()

    def _do_enable(self):
        return True

    def _do_disable(self):
        self.abort()

    def set_sim_sequence(self, sequence):
        # sequence is a list of tuples (angle, phase, wavelength)
        # prepare the list of patterns according to the sequence
        self.patterns = {}
        self.sequence = dict(enumerate(sequence))
        for i, (angle, phase, wavelength) in enumerate(sequence):
            file_name = f"angle{angle}_phase{phase}_nm{wavelength}.jpg"
            patt = os.path.join(self.patterns_path, file_name)
            if not os.path.exists(patt):
                raise FileNotFoundError
            self.patterns[i] = patt

    def getCurrentPosition(self):
        # get current possition in the serie
        return self.idx_image

    def _update_pattern(self):
        path = self.patterns[self.idx_image]
        print(f"Loading pattern: {path}")
        self.label.setPixmap(QPixmap.fromImage(QImage(path)))

    def cycleToPosition(self, target_position):
        # go to the given position in the serie
        self.idx_image = target_position
        self._update_pattern()

    def get_sim_diffraction_angle(self) -> float:
        # get angle at current position
        return self.sequence[self.idx_image][0]

    def set_sim_diffraction_angle(self, theta):
        # TODO: Remove for!
        current_phase = self.sequence[self.idx_image][1]
        current_wavelength = self.sequence[self.idx_image][2]
        for i, (angle, phase, wavelength) in self.sequence.items():
            if (
                theta == angle
                and phase == current_phase
                and wavelength == current_wavelength
            ):
                self.idx_image = i
                self._update_pattern()
                return
        raise RuntimeError(f"No sequence found for angle {theta}")


class D5020(microscope.abc.Device):

    _socket = None
    
    # TODO: understand who is passing index on construction.

    def __init__(self, host=None, port=None, calibration=None, ch=1, index=0):
        super().__init__()

        self.host = host
        self.port = port
        self.ch = ch
        self.sequence = dict(enumerate([(0, 0, 1)]))
        self.idx_image = 0
        self.calibration = calibration
        self.parameter = [ # angle, phase, wavelength
            self.set_angle,
            lambda *args: None,
            lambda *args: None,
        ]
        # Initialize the hardware link
        self.initialize()

    def initialize(self):
        """Initialise the rotator.

        Open the connection and initialize main parameters.

        """
        try:
            self._socket = socket.create_connection((self.host, self.port))

        except socket.error as msg:
            _logger.error("ERROR: {}\n".format(msg))

    def _do_shutdown(self) -> None:
        if self._socket is not None:
            self._socket.close()

    def _vcheck(self, v):  # voltage check
        return max(0, min(v, 10000))

    def set_angle(self, theta: float):
        v = np.interp(theta, self.calibration[0], self.calibration[1])
        self.set_voltage(v)

    def set_voltage(self, voltage: int):
        voltage = int(self._vcheck(voltage))
        cmd = f"inv:{self.ch},{voltage}"
        self._socket.send(cmd.encode() + b"\n")
        ans = self._socket.recv(1024)

    def set_sim_sequence(self, sequence):
        self.sequence = dict(enumerate(sequence))
        
    def getCurrentPosition(self):
        return self.idx_image

    def cycleToPosition(self, target_position):
        self.idx_image = target_position
        self.set_sim_diffraction_angle(target_position)

    def get_parameter(self, ipar):
        return self.sequence[self.idx_image][ipar]

    def set_parameter(self, ipar, value):
        self.parameter[ipar](value)

    def get_sim_diffraction_angle(self) -> float:
        return self.get_parameter[0]

    def set_sim_diffraction_angle(self, theta):
        self.set_parameter[0](theta)

def main():
    hdmi = HDMIslm(display_monitor=2)

    print(f"Trigger mode: {hdmi.trigger_mode}")
    print(f"Trigger type: {hdmi.trigger_type}")

    sequence = [(10, 100, 1000), (20, 200, 2000), (30, 300, 3000)]

    hdmi.set_sim_sequence(sequence)

    print(f"Get current position: {hdmi.getCurrentPosition()}")

    hdmi.cycleToPosition(1)
    print(f"Get current position: {hdmi.getCurrentPosition()}")

    print(f"Get SIM diffraction angle: {hdmi.get_sim_diffraction_angle()}")
    hdmi.set_sim_diffraction_angle(30)
    print(f"Get SIM diffraction angle: {hdmi.get_sim_diffraction_angle()}")


if __name__ == "__main__":
    main()
