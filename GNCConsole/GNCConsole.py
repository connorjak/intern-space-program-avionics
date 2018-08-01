"""Main file for Intern space program GNC Console."""
# pylint: disable=C0103, R0915, R0902, C0111
import tkinter as tk
import tkinter.filedialog as filedialog
import os
import sys
import datetime
from enum import Enum
import serial


accData = [1, 2, 3]
gyroData = [4, 5, 6]
magData = [7, 8, 9]
altitude = 10
temp = 11
pres = 12


class flightState(Enum):
    """Enum that contains all of the different states of flight for the rocket."""  # noqa

    SCRUB = 0
    GROUND = 1
    BOOST = 2
    SEPARATE = 3
    STABILIZE = 4
    SEARCH = 5
    ORBIT = 6
    ABORT = 7
    LANDED = 8


class GUI():
    """User Interface class."""

    currState = flightState.SCRUB

    def __init__(self, master):
        """Run on GUI Initialization."""
        # These variable are set in other methods, they are defined here for
        # Documentation
        self.comJob = None  # Serial Communication Job reference
        self.logFile = None  # Log File object
        self.ser = None  # Serial communication reference
        master.title("Intern Space Program GNC Console")
        self.accString = tk.StringVar()
        self.accString.set('Accleration\nX: %.5f\nY: %.5f\nZ: %.5f' %
                           (accData[0], accData[1], accData[2]))
        self.gyroString = tk.StringVar()
        self.gyroString.set('Gyros\nX: %.5f\nY: %.5f\nZ: %.5f' %
                            (gyroData[0], gyroData[1], gyroData[2]))
        self.magString = tk.StringVar()
        self.magString.set('Magnetometer\nX: %.5f\nY: %.5f\nZ: %.5f' % (
            magData[0], magData[1], magData[2]))
        self.presString = tk.StringVar()
        self.presString.set('Pressure: \n%.5f' % (pres))
        self.tempString = tk.StringVar()
        self.tempString.set('Tempature: \n%.5f' % (temp))
        self.altString = tk.StringVar()
        self.altString.set('Alititude: \n%.5f' % (altitude))
        self.COMPort = tk.Entry(master)
        self.COMPort.delete(0, tk.END)
        self.COMPort.insert(0, 'COM4')
        self.fileLocation = tk.Entry(master)
        self.fileLocation.delete(0, tk.END)
        self.fileLocation.insert(0, os.getcwd() + r'\Logs\%s.csv' %
                                 datetime.datetime.now().strftime(
                                     "%Y-%m-%d-%H-%M-%S"))
        self.fileDialogButton = tk.Button(
            master, text='Set Location', command=self.fileDialogOp
        )
        self.scrubState = tk.Button(
            master, text='SCRUB', height=10, width=20,
            command=lambda: self.sendflightState(flightState.SCRUB))
        self.groundMode = tk.Button(
            master, text='GROUND\nREADY', height=10, width=20,
            command=lambda: self.sendflightState(flightState.GROUND))
        self.launch = tk.Button(
            master, text='BOOST', width=20, height=10,
            command=lambda: self.sendflightState(flightState.BOOST))
        self.seperate = tk.Button(
            master, text='SEPARATE', height=10, width=20,
            command=lambda: self.sendflightState(flightState.SEPARATE))
        self.stabilize = tk.Button(
            master, text='STABILIZE', height=10, width=20,
            command=lambda: self.sendflightState(flightState.STABILIZE))
        self.search = tk.Button(
            master, text='SEARCH', height=10, width=20,
            command=lambda: self.sendflightState(flightState.SEARCH))
        self.orbit = tk.Button(
            master, text='ORBIT', height=10, width=20,
            command=lambda: self.sendflightState(flightState.ORBIT))
        self.landed = tk.Button(
            master, text='LANDED', height=10, width=20,
            command=lambda: self.sendflightState(flightState.LANDED))
        self.abort_button = tk.Button(
            master, text="ABORT!", command=self.abort,
            height=10, background='red')
        self.close_button = tk.Button(master, text="Close", command=self.quit)
        self.accDisp = tk.Label(
            master, textvariable=self.accString, font=("TkDefaultFont", 10))
        self.gyroDisp = tk.Label(
            master, textvariable=self.gyroString, font=("TkDefaultFont", 10))
        self.magDisp = tk.Label(
            master, textvariable=self.magString, font=("TkDefaultFont", 10))
        self.presDisp = tk.Label(
            master, textvariable=self.presString, font=("TkDefaultFont", 10))
        self.tempDisp = tk.Label(
            master, textvariable=self.tempString, font=("TkDefaultFont", 10))
        self.altDisp = tk.Label(
            master, textvariable=self.altString, font=("TkDefaultFont", 10))
        self.startCom = tk.Button(
            master, text="Start Communication", command=self.startComProcess)
        self.stopCom = tk.Button(
            master, text="Stop Communication", command=self.stopComProcess)

        self.scrubState.grid(row=1, column=0, sticky=(tk.E, tk.W, tk.S))
        self.groundMode.grid(row=1, column=1, sticky=(tk.E, tk.W, tk.S))
        self.launch.grid(row=1, column=2, sticky=(tk.E, tk.W, tk.S))
        self.seperate.grid(row=1, column=3, sticky=(tk.E, tk.W, tk.S))
        self.stabilize.grid(row=1, column=4, sticky=(tk.E, tk.W, tk.S))
        self.search.grid(row=1, column=5, sticky=(tk.E, tk.W, tk.S))
        self.orbit.grid(row=1, column=6, sticky=(tk.E, tk.W, tk.S))
        self.landed.grid(row=1, column=7, sticky=(tk.E, tk.W, tk.S))
        self.abort_button.grid(row=2, columnspan=8, sticky=(tk.E, tk.W, tk.N))
        self.fileLocation.grid(row=3, columnspan=3, sticky=(tk.E, tk.W))
        self.fileDialogButton.grid(row=3, column=3, sticky=(tk.E, tk.W))
        self.COMPort.grid(row=4, column=2, sticky=(tk.E, tk.W, tk.N, tk.S))
        self.startCom.grid(row=4, column=3, sticky=(tk.E, tk.W, tk.N, tk.S))
        self.stopCom.grid(row=4, column=4, sticky=(tk.E, tk.W, tk.N, tk.S))
        self.accDisp.grid(row=5, column=0)
        self.gyroDisp.grid(row=5, column=1)
        self.magDisp.grid(row=5, column=2)
        self.presDisp.grid(row=5, column=3)
        self.tempDisp.grid(row=5, column=4)
        self.altDisp.grid(row=6, column=3)
        self.close_button.grid(row=7, columnspan=7)

    def sendflightState(self, sendState):
        """Send the current flight state to the transciever, and thus the rocket."""   # noqa
        self.ser.write(b's%i' % sendState.value)

    def fileDialogOp(self):
        """Select the file to write the log to."""
        self.fileLocation.delete(0, tk.END)
        self.fileLocation.insert(0,
                                 filedialog.asksaveasfilename(
                                     title='Select Log File Location',
                                     filetypes=[('csv file', '*.csv')],
                                     defaultextension='.csv'))

    def quit(self):
        """Quit the application."""
        try:
            self.ser.close()
        except AttributeError:
            pass
        root.quit()
        sys.exit()

    def abort(self):
        """Abort Rocket flight."""
        self.ser.write(b's%i' % flightState.ABORT.value)
        print("ABORTING!")

    def startComProcess(self):
        """Start serial communication with the transciever."""
        # if statement prevents attempting to open serial if already in use
        # And prevents creation of multiple log files
        if self.ser is None or self.ser.closed:
            self.ser = serial.Serial(self.COMPort.get(), 115200)
            self.logFile = open(self.fileLocation.get(), 'w')
            root.after(1, self.getData)

    def stopComProcess(self):
        """Stop serial communication with the transciever."""
        # If statements prevent error stemming from uassigned variables
        # Else statements will display warning messages if variable is
        # unassigned
        if self.comJob is not None:
            root.after_cancel(self.comJob)
        if self.logFile is not None:
            self.logFile.close()
        if self.ser is not None and self.ser.open:
            self.ser.close()

    def getData(self):
        """Get telemetry Data from transciever."""
        self.ser.write(b'g')
        readString = self.ser.readline()
        print(readString)
        readString = readString.decode("utf-8")
        splittedString = readString.split('\t')
        for i, num in enumerate(splittedString):
            try:
                splittedString[i] = int(float(num))
            except ValueError:
                pass
        self.accString.set('Accleration\nX: %.5f\nY: %.5f\nZ: %.5f' %
                           (splittedString[0], splittedString[1],
                            splittedString[2]))
        self.logFile.write(readString)
        self.comJob = root.after(10, self.getData)


if __name__ == "__main__":
    root = tk.Tk()
    root.rowconfigure((1), weight=1, uniform='uniformCategory')
    root.columnconfigure((0, 1, 2, 3, 4, 5, 6), weight=1)
    GUI = GUI(root)
    root.mainloop()
