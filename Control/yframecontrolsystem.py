from enum import IntEnum
import numpy as np
from utils import constrain, PID, ExpMovingAverageFilter, map_value

class ControlAxes(IntEnum):
    FORWARD     = 0
    STRAFE      = 1
    DEPTH       = 2
    ROLL        = 3
    PITCH       = 4    
    YAW         = 5

class Thrusters(IntEnum):
    # Y-frame specific
    H_FRONTLEFT     = 0
    H_FRONTRIGHT    = 1
    H_REAR          = 2
    V_FRONTLEFT     = 3
    V_FRONTRIGHT    = 4
    V_REAR          = 5

class YFrameControlSystem:
    def __init__(self):
        # Thrusters calibration values
        self.__thrustersDirCorr = [1, 1, 1, 1, 1, 1]
        self.__thrustersOrder = [Thrusters.H_FRONTLEFT, 
                  Thrusters.H_FRONTRIGHT,
                  Thrusters.H_REAR, 
                  Thrusters.V_FRONTLEFT,
                  Thrusters.V_FRONTRIGHT,
                  Thrusters.V_REAR]
        self.__trustersXValues = [-100, 100]
        self.__thrusterIncrement = 0.5
        # (forward,strafe,depth,roll,pitch,yaw)
        self.__axesInputs     = [0, 0, 0, 0, 0, 0]
        self.__axesValues     = [0, 0, 0, 0, 0, 0]        
        self.__stabs          = [0, 0, 0, 0, 0, 0]
        self.__PIDValues      = [0, 0, 0, 0, 0, 0]
        self.__PIDs = [None, 
                     None, 
                     PID(10, 0, 0, 0),
                     PID(10, 0, 0, 0),
                     PID(10, 0, 0, 0),
                     PID(10, 0, 0, 0)]
        self.__filters = [None, 
                          None, 
                          ExpMovingAverageFilter(0.8),
                          None, 
                          None, 
                          None]
        # (hor1,hor2,hor3,ver1,ver2,ver3)
        self.__motsOutputsSetpoint = [0, 0, 0, 0, 0, 0]
        self.__motsOutputsReal = [0, 0, 0, 0, 0, 0]
        # Operating period
        self.__dt = 1/500

    def __updateControl(self):
        self.__updatePID()
        self.__calculateHorizontalThrust()
        self.__calculateVerticalThrust()
        self.__thrustersCalibrate()
        self.__smoothRPMBuildUp()

    def __updatePID(self):
        if not ((np.abs(self.__axesInputs[ControlAxes.DEPTH])<1) and self.__stabs[ControlAxes.DEPTH]):
            self.setPIDSetpoint(ControlAxes.DEPTH, self.__axesValues[ControlAxes.DEPTH])
        
        if np.abs(self.__axesInputs[ControlAxes.YAW]) >= 1 or not self.__stabs[ControlAxes.YAW] : 
            self.setPIDSetpoint(ControlAxes.YAW, self.__axesValues[ControlAxes.YAW])

        yawPID = self.__PIDs[ControlAxes.YAW].update(self.__axesValues[ControlAxes.YAW], self.__dt) if (np.abs(self.__axesInputs[ControlAxes.YAW]) < 1) and self.__stabs[ControlAxes.YAW] else 0 
        rollPID = self.__PIDs[ControlAxes.ROLL].update(self.__axesValues[ControlAxes.ROLL], self.__dt) if self.__stabs[ControlAxes.ROLL] else 0
        pitchPID = self.__PIDs[ControlAxes.PITCH].update(self.__axesValues[ControlAxes.PITCH], self.__dt) if self.__stabs[ControlAxes.PITCH] else 0
        depthPID = -self.__PIDs[ControlAxes.DEPTH].update(self.__axesValues[ControlAxes.DEPTH], self.__dt) if self.__stabs[ControlAxes.DEPTH] else 0

        self.__PIDValues[ControlAxes.YAW] = constrain(yawPID, -100, 100)
        self.__PIDValues[ControlAxes.ROLL] = constrain(rollPID, -100, 100)
        self.__PIDValues[ControlAxes.PITCH] = constrain(pitchPID, -100, 100)
        self.__PIDValues[ControlAxes.DEPTH] = constrain(depthPID, -100, 100)

    def __calculateHorizontalThrust(self):

        # HFL = self.__axesInputs[ControlAxes.STRAFE] / 2 + np.sqrt(3) * self.__axesInputs[ControlAxes.FORWARD] / 2 + 0.32 * (self.__axesInputs[ControlAxes.YAW] + self.__PIDValues[ControlAxes.YAW])
        # HFR = - self.__axesInputs[ControlAxes.STRAFE] / 2 + np.sqrt(3) * self.__axesInputs[ControlAxes.FORWARD]  / 2 - 0.32 * (self.__axesInputs[ControlAxes.YAW] + self.__PIDValues[ControlAxes.YAW])
        # HRR = - self.__axesInputs[ControlAxes.STRAFE] + 0.32 * (self.__axesInputs[ControlAxes.YAW] + self.__PIDValues[ControlAxes.YAW])
        # alt thrust calculatation
        HFL = self.__axesInputs[ControlAxes.STRAFE] / 2 + self.__axesInputs[ControlAxes.FORWARD] + 0.32 * (self.__axesInputs[ControlAxes.YAW] + self.__PIDValues[ControlAxes.YAW])
        HFR = - self.__axesInputs[ControlAxes.STRAFE] / 2 + self.__axesInputs[ControlAxes.FORWARD] - 0.32 * (self.__axesInputs[ControlAxes.YAW] + self.__PIDValues[ControlAxes.YAW])
        HRR = - self.__axesInputs[ControlAxes.STRAFE] + 0.32 * (self.__axesInputs[ControlAxes.YAW] + self.__PIDValues[ControlAxes.YAW])

        self.__motsOutputsSetpoint[Thrusters.H_FRONTLEFT] = constrain(HFL, -100, 100)
        self.__motsOutputsSetpoint[Thrusters.H_FRONTRIGHT] = constrain(HFR, -100, 100)
        self.__motsOutputsSetpoint[Thrusters.H_REAR] = constrain(HRR, -100, 100)
        
    def __calculateVerticalThrust(self):
        VFL = self.__PIDValues[ControlAxes.ROLL] + self.__PIDValues[ControlAxes.DEPTH] + self.__PIDValues[ControlAxes.PITCH] + self.__axesInputs[ControlAxes.DEPTH]
        VFR = -self.__PIDValues[ControlAxes.ROLL] + self.__PIDValues[ControlAxes.DEPTH] + self.__PIDValues[ControlAxes.PITCH] + self.__axesInputs[ControlAxes.DEPTH]
        VRR = -self.__PIDValues[ControlAxes.PITCH] + self.__PIDValues[ControlAxes.DEPTH] + self.__axesInputs[ControlAxes.DEPTH]

        self.__motsOutputsSetpoint[Thrusters.V_FRONTLEFT] = constrain(VFL, -100, 100)
        self.__motsOutputsSetpoint[Thrusters.V_FRONTRIGHT] = constrain(VFR, -100, 100)
        self.__motsOutputsSetpoint[Thrusters.V_REAR] = constrain(VRR, -100, 100)

    def __thrustersCalibrate(self):
        if not (len(self.__motsOutputsSetpoint) == len(self.__thrustersDirCorr)) and not (len(self.__motsOutputsSetpoint) == len(self.__thrustersOrder)):
            return None
        reThrusters = [0.0] * len(self.__motsOutputsSetpoint)
        for i in range(len(self.__motsOutputsSetpoint)):
            reThrusters[i] = self.__motsOutputsSetpoint[self.__thrustersOrder[i]]
            reThrusters[i] = map_value(reThrusters[i], -100, 100, self.__trustersXValues[0], self.__trustersXValues[1]) * self.__thrustersDirCorr[i]
        self.__motsOutputsSetpoint = reThrusters
    
    def __smoothRPMBuildUp(self):
        for i in range(6):
            inc = self.__thrusterIncrement if self.__motsOutputsSetpoint[i] > 0 else 0-self.__thrusterIncrement
            self.__motsOutputsReal[i] += inc if abs(self.__motsOutputsReal[i]) < abs(self.__motsOutputsSetpoint[i]) else 0
            self.__motsOutputsReal[i] -= inc if abs(self.__motsOutputsReal[i]) > abs(self.__motsOutputsSetpoint[i]) else 0
            if (self.__motsOutputsSetpoint[i] > 0 and self.__motsOutputsReal[i] < 0) or (self.__motsOutputsSetpoint[i] < 0 and self.__motsOutputsReal[i] > 0):
                self.__motsOutputsReal[i] = 0
            if self.__motsOutputsSetpoint[i] == 0:
                self.__motsOutputsReal[i] = 0
            if abs(abs(self.__motsOutputsReal[i]) - abs(self.__motsOutputsSetpoint[i])) < self.__thrusterIncrement:
                self.__motsOutputsReal[i] = self.__motsOutputsSetpoint[i]
    
    # Setters
    def setPIDConstants(self, controlAxis: ControlAxes, constants):
        if self.__PIDs[controlAxis] is not None:
            Kp, Ki, Kd = constants
            self.__PIDs[controlAxis].setConstants(Kp, Ki, Kd)

    def setPIDSetpoint(self, controlAxis: ControlAxes, setpoint): 
        if self.__PIDs[controlAxis] is not None:       
            self.__PIDs[controlAxis].setSetpoint(setpoint)   

    def setAxesInputs(self, inputs):
        self.__axesInputs = inputs

    def setAxisInput(self, controlAxis: ControlAxes, input):
        self.__axesInputs[controlAxis] = input

    def setAxesValues(self, inputs):
        self.__axesValues = inputs
        for i in range(6):
            if self.__filters[i] is not None: 
                self.__axesValues[i] = self.__filters[i].update(inputs[i])

    def setAxisValue(self, controlAxis: ControlAxes, input):
        self.__axesValues[controlAxis] = input
        if self.__filters[controlAxis] is not None: 
            self.__axesValues[controlAxis] = self.__filters[controlAxis].update(input)

    def setStabilizations(self, inputs):
        self.__stabs = inputs

    def setStabilization(self, controlAxis: ControlAxes, input):
        self.__stabs[controlAxis] = input

    def setdt(self, dt):
        self.__dt = dt

    def set_thrusters_calibration_values(self, thrustersDirCorr, thrustersOrder, trustersXValues, thrusterIncrement):
        self.__thrustersDirCorr = thrustersDirCorr
        self.__thrustersOrder = thrustersOrder
        self.__trustersXValues = trustersXValues
        self.__thrusterIncrement = thrusterIncrement  

    # Getters
    def getPIDSetpoint(self, controlAxis: ControlAxes): 
        if self.__PIDs[controlAxis] is not None:       
            return self.__PIDs[controlAxis].setpoint
        return 0

    def getMotsControls(self):
        self.__updateControl()
        return self.__motsOutputsReal

    def getMotControl(self, controlAxis: ControlAxes):        
        self.__updateControl()
        return self.__motsOutputsReal[controlAxis]
    
    def getAxisValue(self, controlAxis: ControlAxes):
        return self.__axesValues[controlAxis]
    
    def getAxesValues(self):
        return self.__axesValues