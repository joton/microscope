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
import Pyro4

from PyQt5.QtWidgets import QApplication, QDesktopWidget, QLabel, QMainWindow
from PyQt5.QtGui import QImage, QPixmap

_logger = logging.getLogger(__name__)


class HDMIslm(microscope.abc.Modulator):

    # TODO: understand who is passing index on construction.
    def __init__(self, width=None, heigh=None, display_monitor=1, **kwargs):
        super().__init__(**kwargs)

        # setup the class
        self.display_monitor = display_monitor  # the number of the monitor
        self.Nx = width
        self.Ny = heigh
        self.label = None
        self.widget = None

        self.patterns = {}
        self.app = None

        self.set_sequence([(0, 60, 488e-9)])
        self.initialize()

    def _run_qt(self):
        """Initialise the slm.
        Open the connection and initialize main parameters.
        """
        self.app = QApplication([])
        self.label = QLabel()

        self.widget = QMainWindow()  # define your widget
        zero = np.zeros((self.Nx, self.Ny))
        image = QImage(zero, self.Nx, self.Ny, QImage.Format_Grayscale8)
        self.label.setPixmap(QPixmap(image))

        self.widget.setCentralWidget(self.label)
        monitor = QDesktopWidget().screenGeometry(self.display_monitor)
        self.widget.move(monitor.left(), monitor.top())
        self.widget.showFullScreen()

        # TODO: Test if thread is necessary and works.
        self.app.exec_()

    def set_fullscreen(self, full=True):
        if full:
            self.widget.showFullScreen()
        else:
            self.widget.showNormal()

    def initialize(self):
        threading.Thread(target=self._run_qt).start()

    def _do_shutdown(self) -> None:
        if self.app is not None:
            self.app.quit()

    def set_sequence(self, sequence):
        # sequence is a list of tuples (angle, phase, wavelength)
        # prepare the list of patterns according to the sequence
        self.patterns = {}
        self.sequence = dict(enumerate(sequence))
        print("sequence %s" % str(self.sequence))
        for i, (angle, phase, wavelength) in enumerate(sequence):
            self.patterns[i] = self.gen_pattern(angle, phase, wavelength)

    def _update(self):
        _logger.debug(f"Loading pattern {self.idx_image}")
        pattern = self.patterns[self.idx_image]
        image = QImage(pattern, self.Nx, self.Ny, QImage.Format_Grayscale8)
        self.label.setPixmap(QPixmap(image))

    # Phase functions:
    def linearGrating(self, period, theta_deg, phi_deg):
        # period in pixels
        k = 1 / period
        kx = k * np.cos(np.deg2rad(theta_deg))
        ky = k * np.sin(np.deg2rad(theta_deg))
        x = np.arange(self.Nx)
        y = np.arange(self.Ny)
        x_mesh, y_mesh = np.meshgrid(x, y)
        # phase in waves (phase in rads / 2pi)
        phase = kx * x_mesh + ky * y_mesh + np.deg2rad(phi_deg)
        return np.mod(phase, 1)

    def fresnelLens(self, focal, xcenter, ycenter, wavelength):
        # Convergent lens: focal > 0
        # Divergent lens: focal < 0
        # xcenter and ycenter in pixels
        # wavelength in meters
        pixelsize = 8e-6
        x = np.arange(self.Nx) - xcenter
        y = np.arange(self.Ny) - ycenter
        x_mesh, y_mesh = np.meshgrid(x, y, indexing="xy")
        # phase in waves (phase in rads / 2pi)
        phase = (
            0.5 * (x_mesh ** 2 + y_mesh ** 2) * pixelsize ** 2 / (wavelength * focal)
        )
        return np.mod(phase, 1)

    # Operators:
    def binarize(self, A, value1, value2):
        threshold = 0.5
        binarized_array = A
        binarized_array[binarized_array < threshold] = value1
        binarized_array[binarized_array >= threshold] = value2
        return binarized_array

    def gen_pattern(self, angle, phase, wavelength):
        grating = self.binarize(self.linearGrating(10, angle, phase), 0, 0.5)
        fresnel = self.fresnelLens(0.5, self.Nx / 2, self.Ny / 2, wavelength)

        total = np.mod(grating + fresnel, 1)
        m, M = 0, 1
        max_, min_ = 255, 0
        normalized = (max_ - min_) / (M - m) * (total - m) + min_
        return normalized.astype(np.uint8)


class D5020(microscope.abc.Modulator):

    _socket = None

    # TODO: understand who is passing index on construction.

    def __init__(
        self,
        host=None,
        port=None,
        calibration=None,
        minmax=None,
        coef=None,
        ch=1,
        index=0,
        **kwargs,
    ):
        super().__init__(**kwargs)

        self.host = host
        self.port = port
        self.ch = ch
        self.idx_image = 0
        self.calibration = calibration
        self.coef = np.array(coef)  # coeficients for np.poli1d
        self.minmax = minmax

        self.set_sequence([(0, 60, 488e-9)])
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

    def set_sequence(self, sequence):
        # sequence is a list of tuples (angle, phase, wavelength)
        self.sequence = dict(enumerate(sequence))
        print("sequence %s" % str(self.sequence))

    def _update(self):
        angle = self.sequence[self.idx_image][0]
        _logger.debug(f"Set angle {angle}[{self.idx_image}]")
        self.set_angle(angle)

    def _vcheck(self, v):  # voltage check
        return max(0, min(v, 10000))

    def set_angle(self, theta: float):
        v = self.calc_voltage(theta)
        print("Voltage %0.3f for angle %0.0f" % (v, theta))
        self.set_voltage(v)

    def set_voltage(self, voltage: int):
        voltage = int(self._vcheck(voltage))
        cmd = f"inv:{self.ch},{voltage}"
        self._socket.send(cmd.encode() + b"\n")
        ans = self._socket.recv(1024)

    def calc_voltage(self, theta: float):
        theta %= 360
        if theta < self.minmax[0]:
            theta += 180
        if theta > self.minmax[1]:
            theta -= 180
        if self.coef is not None:
            angle = np.array([0] * len(self.coef))
            angle[-1] = theta
            p = np.poly1d(self.coef - angle)
            raiz = np.roots(p)
            raiz = raiz[np.isreal(raiz)]
            raiz = np.real(raiz)
            raiz = raiz[raiz > 1]
            v = float(raiz)
        else:
            v = np.interp(theta, self.calibration[0], self.calibration[1])
        return v


def main():
    import os

    os.environ["QT_QPA_PLATFORM"] = "minimal"

    sequence = [(0, 60, 488e-9), (60, 0, 488e-9), (90, 0, 488e-9)]
    hdmi = HDMIslm(100, 200, display_monitor=2)
    hdmi.set_sequence(sequence)

    print(f"Get current position: {hdmi.position}")

    hdmi.position = 1
    print(f"Get current position: {hdmi.position}")
    print(f"Get diffraction angle: {hdmi.angle}")

    hdmi.angle = 90
    print(f"Get diffraction position: {hdmi.position}")
    print(f"Get diffraction angle: {hdmi.angle}")

    hdmi.position = 0
    print(f"Get diffraction angle: {hdmi.angle}")

    hdmi.shutdown()


if __name__ == "__main__":
    main()
