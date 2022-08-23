#!/usr/bin/env python

import time
import argparse
import serial
import numpy as np
import pandas as pd


class SerialController:
    """Generic serial controller that will open and close serial connections"""

    # Serial port
    ser = None

    def __init__(self, port):
        """Constructor

        Initialises connection to serial port.

        Parameters
        ----------
        port : str
            Serial port of device
        """

        self.ser = serial.Serial(port)

    def __del__(self):
        """Destructor

        Closes connection to serial port.
        """

        self.ser.close()


class TeensyController(SerialController):
    """Controller for the teensy firmware used to read values from the SPS"""

    def __init__(self, port):
        """Constructor

        Opens serial connection and checks functionality

        Parameters
        ----------
        port : str
            Serial port of device
        """
        super().__init__(port)
        self.check()

    def get_voltage(self) -> float:
        """Measure voltage from SPS

        Returns
        -------
        float
            Measured voltage in V
        """
        self.ser.write(b"v\n")
        reply = self.ser.readline()
        return float(reply)

    def get_current(self) -> float:
        """Measure current from SPS

        No calculations are done on the raw values taken by the teensy. This
        means that the values are in volts and must be converted to current
        based on the configuration of the board.

        Returns
        -------
        float
            Measured current in V
        """
        self.ser.write(b"i\n")
        reply = self.ser.readline()
        return float(reply)

    def check(self):
        """Performs a check of the connection to the board

        Raises
        ------
        RuntimeError
            Checks that teensy replies "ok" when sent "check"
        """
        self.ser.write(b"check\n")
        reply = self.ser.readline()
        if (reply != "ok"):
            raise RuntimeError("Teensy check failed")


class SMUController(SerialController):
    """Controller for the Keithley 2400 SMU used to supply known voltage to the
    SPS

    Uses SCPI (Standard Control of Programmable Instruments) to control the SMU
    through its RS232 port. Written for the Keithley 2400 SMU, but should work
    for any other SMU that uses SCPI.
    """

    def get_voltage(self) -> float:
        """Measure voltage supplied to the SPS from SMU

        Returns
        -------
        float
            Measured voltage
        """
        pass

    def get_current(self) -> float:
        """Measure current supplied to the SPS from SMU

        Returns
        -------
        float
            Measured current
        """
        pass

    def set_voltage(self, v):
        """Set voltage applied to the SPS.

        Parameters
        ----------
        v : float
            New voltage
        """
        pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="""Automated data recorder
        for Soil Power Sensor board using serial to control Keithley 2400 SMU and
        teensy""")

    parser.add_argument("start", type=float, help="Start voltage in V")
    parser.add_argument("stop", type=float, help="End voltage in V")
    parser.add_argument("step", type=float, help="Step between voltages in V")
    parser.add_argument("smu_port", type=str, help="SMU serial port")
    parser.add_argument("teensy_port", type=str, help="Teensy serial port")
    parser.add_argument("data_file", type=str, help="Path to store data file")

    args = parser.parse_args()


    teensy = TeensyController(args.teensy_port)
    #smu = SMUController(args.smu_port)

    data = {
        "V_in": [],
        "I_in": [],
        "V_i": [],
        "V_2x": []
    }

    for v in np.arange(args.start, args.stop, args.step):
        # Set voltage
        SMUController.set_voltage(v)
        # Sleep for 1ms
        time.sleep(0.001)

        # Measure input
        #data["V_in"] = SMUController.get_voltage()
        #data["I_in"] = SMUController.get_current()

        # Measure SPS output
        data["V_i"] = teensy.get_voltage()
        data["V_2x"] = teensy.get_current()

    data_df = pd.DataFrame(data)
    data_df.to_csv(args.data_file)
