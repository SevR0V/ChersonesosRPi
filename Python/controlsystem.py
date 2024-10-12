from enum import IntEnum
import numpy as np
from utils import constrain, normalized, PID, ExpMovingAverageFilter


class ControlAxes(IntEnum):
    FORWARD     = 0
    STRAFE      = 1
    DEPTH       = 2
    ROLL        = 3
    PITCH       = 4    
    YAW         = 5

class Motors(IntEnum):
    H_FRONTLEFT     = 0
    H_FRONTRIGHT    = 1
    H_REAR          = 2
    V_FRONTLEFT     = 3
    V_FRONTRIGHT    = 4
    V_REAR          = 5

class YFrameControlSystem:
    def __init__(self):
        # (forward,strafe,depth,roll,pitch,yaw)
        self.axesInputs     = (0, 0, 0, 0, 0, 0)
        self.axesValues     = (0, 0, 0, 0, 0, 0)        
        self.stabs          = (0, 0, 0, 0, 0, 0)
        self.PIDs = (None, 
                     None, 
                     PID(10, 0, 0, 0), 
                     PID(10, 0, 0, 0), 
                     PID(10, 0, 0, 0), 
                     PID(10, 0, 0, 0))
        # (hor1,hor2,hor3,ver1,ver2,ver3)
        self.motsOutputs    = (0, 0, 0, 0, 0, 0)
    
    def setPIDConstants(self, controlAxis: ControlAxes, constants):
        Kp, Ki, Kd = constants
        self.PIDs[controlAxis].setConstants(Kp, Ki, Kd)

    def setPIDSetpoint(self, controlAxis: ControlAxes, setpoint):        
        self.PIDs[controlAxis].setSetpoint(setpoint)   

    def setAxesInputs(self, inputs):
        self.axesInputs = inputs

    def setAxisInput(self, controlAxis: ControlAxes, input):
        self.axesInputs[controlAxis] = input


    def calculateHorizontalThrust(self):
        mdf1 = 0.6 if self.axesInputs[ControlAxes.STRAFE]>0 else 1
        mdf2 = 1 if self.axesInputs[ControlAxes.STRAFE]>0 else 0.6
        mdf3 = 1.2 if self.axesInputs[ControlAxes.STRAFE]>0 else 0.6

        if np.abs(self.axesInputs[ControlAxes.YAW]) >= 1 or not self.stabs[ControlAxes.YAW] : 
            self.PIDs[ControlAxes.YAW].setSetpoint(self.axesValues[ControlAxes.YAW])

        yawPID = self.PIDs[ControlAxes.YAW].update(self.yaw, self.timer.getInterval()) if (np.abs(self.axesInputs[ControlAxes.YAW]) < 1) and self.stabs[ControlAxes.YAW] else 0

        self.motsOutputs[Motors.H_FRONTLEFT] = self.axesInputs[ControlAxes.STRAFE] * mdf1/2 + np.sqrt(3) * self.axesInputs[ControlAxes.FORWARD] / 2 + 0.32 * (self.axesInputs[ControlAxes.YAW] + yawPID)
        self.motsOutputs[Motors.H_FRONTRIGHT] = - self.axesInputs[ControlAxes.STRAFE] * mdf2/2 + np.sqrt(3) * self.axesInputs[ControlAxes.FORWARD]  / 2 - 0.32 * (self.axesInputs[ControlAxes.YAW] + yawPID)
        self.motsOutputs[Motors.H_REAR] = - self.axesInputs[ControlAxes.STRAFE] * mdf3 + 0.32 * (self.axesInputs[ControlAxes.YAW] + yawPID)