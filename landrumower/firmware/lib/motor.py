from machine import Pin, PWM
import time, math

from lib.pid import Pid

FREQ = 20000
WHEELDIAMETER = 0.22
TICKSPERREVOLUTION = 340

# acceleration pid
KP = 2.0
KI = 0.0
KD = 0.0

class Motor:
    impMinTime = time.ticks_ms()
    nextSpeedMeasureTime = time.ticks_ms()
    lastMeasuredTime = time.ticks_ms()
    brakeTimeStart = 0
    brakeDuration = 500
    mustStopTime = 0
    odomTicks = 0
    lastMeasuredTicks = 0
    currentSpeedSetPoint = 0
    currentSpeed = 0
    currentSpeedLp = 0
    currentRpm = 0
    currentRpmLp = 0
    currentRpmSetPoint = 0
    currentPwm = 0
    currentTrqPwm = 0
    targetBrakePwm = 0
    currentBrakePwm = 0
    positiveDirection = True
    minPwm = 0
    maxPwm = 65535
    minBreakPwm = 0
    maxBreakPwm = 65535
    driverResetTimeout = 0
    electricalCurrent = 0.0
    overload = False
    overloadTimeout = 0

    def __init__(self, 
                 type: str,
                 pidControl: bool,
                 pinDir: Pin, 
                 pinBrake: PWM, 
                 pinPwm: PWM, 
                 pinImp: Pin, 
                 brakePinHighActive: bool, 
                 directionPinHighPositive: bool, 
                 overloadThreshold: float) -> None:
        self.type = type
        self.pidControl = pidControl
        self.pinDir = pinDir
        self.pinBrake = pinBrake
        self.pinPwm = pinPwm
        self.pinImp = pinImp
        self.pinImp.irq(trigger=Pin.IRQ_RISING, handler=self.odometryIsr)
        self.pinPwm.freq(FREQ)
        self.pinBrake.freq(FREQ)
        self.brakePinHighActive = brakePinHighActive
        self.directionPinHighPositive = directionPinHighPositive
        self.overLoadThreshold = overloadThreshold
        self.motorControl = Pid(KP, KI, KD, 0.01)
    
    def setSpeed(self, setPoint: float) -> None:
        if not self.overload:
            # stop first if direction change
            if self.currentSpeedSetPoint * setPoint < 0:
                self.currentSpeedSetPoint = setPoint
                self.positiveDirection = not self.positiveDirection
                self.stop()
                return
            # set direction 
            self.currentSpeedSetPoint = setPoint
            if setPoint < 0:
                self.positiveDirection = False
            else:
                self.positiveDirection = True
            # decision picontrol or not
            if self.pidControl and self.type == 'gear':
                self.currentRpmSetPoint = abs(60 * setPoint/(100 * math.pi * WHEELDIAMETER))
            else:
                self.currentRpmSetPoint = abs(setPoint)
        else:
            self.positiveDirection = True
            self.currentSpeedSetPoint = 0
            self.currentRpmSetPoint = 0
    
    def setRpm(self, setPoint: int) -> None:
        if setPoint >= 0:
            self.positiveDirection = True
        else:
            self.positiveDirection = False
        self.currentRpmSetPoint = abs(int(setPoint))
    
    def run(self) -> None:
        self.calcSpeed()
        self.control()
        self.checkOverCurrent()
        self.setDriverPins()
    
    def calcSpeed(self) -> None:
        if time.ticks_diff(self.nextSpeedMeasureTime, time.ticks_ms()) > 0: return
        currentTicks = self.odomTicks - self.lastMeasuredTicks
        self.lastMeasuredTicks = self.odomTicks
        self.currentRpm = 60000 * (float(currentTicks)/TICKSPERREVOLUTION) / (time.ticks_diff(time.ticks_ms(), self.lastMeasuredTime))
        self.currentRpmLp = (0.3*self.currentRpm + 0.7*self.currentRpmLp)
        self.currentSpeed = 1000 * (math.pi * WHEELDIAMETER * float(currentTicks)/TICKSPERREVOLUTION) / (time.ticks_diff(time.ticks_ms(), self.lastMeasuredTime))
        self.currentSpeedLp = 0.3*self.currentSpeed + 0.7*self.currentSpeedLp
        if self.currentRpmLp < 0.01:
            self.currentRpmLp = 0.0
            self.currentSpeedLp = 0.0
        self.lastMeasuredTime = time.ticks_ms()
        self.nextSpeedMeasureTime = time.ticks_add(time.ticks_ms(), 50)

    def control(self) -> None:
        # stop control if driver in reset state
        if time.ticks_diff(self.driverResetTimeout, time.ticks_ms()) > 0:
            return

        # motor should be stopped
        if self.currentRpmSetPoint == 0:
            if self.mustStopTime == 0:
                self.mustStopTime = time.ticks_add(time.ticks_ms(), 50)
                self.stop()
            if time.ticks_diff(self.mustStopTime, time.ticks_ms()) < 0 and self.currentRpm != 0:
                self.handBrake()
            return

        # motor should run
        if self.pidControl and self.type == 'gear':
            self.mustStopTime = 0
            self.brakeTimeStart = 0
            output = self.motorControl.control(self.currentRpmSetPoint, self.currentRpmLp)  
            self.currentPwm = self.currentPwm + int(output)
            if self.currentPwm < 0:
                self.negTrq()
            else:
                self.posTrq()
        else:
            self.mustStopTime = 0
            self.brakeTimeStart = 0
            self.currentTrqPwm = int((self.currentRpmSetPoint * 65535)/255)
            self.currentBrakePwm = 0 if self.brakePinHighActive else 65535
    
    def checkOverCurrent(self) -> None:
        if not self.overload and self.electricalCurrent > self.overLoadThreshold:
            self.overload = True
            self.stop()
            self.overloadTimeout = time.ticks_add(time.ticks_ms(), 2000)
        elif time.ticks_diff(self.overloadTimeout, time.ticks_ms()) < 0:
            self.overload = False
    
    def lockMotorControl(self) -> None:
        self.stop()
        self.driverReset()
    
    def setDriverPins(self) -> None:
        self.currentTrqPwm = max(0, self.currentTrqPwm)
        self.currentTrqPwm = min(self.currentTrqPwm, 65535)
        self.currentBrakePwm = max(0, self.currentBrakePwm)
        self.currentBrakePwm = min(self.currentBrakePwm, 65535)
        self.pinDir.value(int(self.directionPinHighPositive) if self.positiveDirection else int(not self.directionPinHighPositive)) 
        self.pinPwm.duty_u16(self.currentTrqPwm)
        self.pinBrake.duty_u16(self.currentBrakePwm)
    
    def posTrq(self) -> None:
        if self.brakePinHighActive and self.currentBrakePwm != 0 or not self.brakePinHighActive and self.currentBrakePwm != 65535:
            self.releaseBrake()
        self.currentTrqPwm = self.currentPwm

    def negTrq(self) -> None:
        if self.currentTrqPwm != 0:
            self.currentTrqPwm = 0
        self.currentBrakePwm = self.currentBrakePwm + abs(self.currentPwm) if self.brakePinHighActive else self.currentBrakePwm + self.currentPwm

    def releaseBrake(self) -> None:
        if self.brakePinHighActive:
            self.currentBrakePwm = 0
        else:
            self.currentBrakePwm = 65535
    
    def stop(self) -> None:
        self.currentPwm = 0
        self.currentTrqPwm = 0
        self.currentBrakePwm = 0 if self.brakePinHighActive else 65535
        self.motorControl.reset()
        # self.odomTicks = 0

    def handBrake(self) -> None:
        self.currentPwm = 0
        self.currentTrqPwm = 0
        self.motorControl.reset()
        if self.brakeTimeStart == 0: # initialze ramp
            self.brakeTimeStart = time.ticks_ms()

        rampFinished = time.ticks_diff(time.ticks_ms(), self.brakeTimeStart) > self.brakeDuration
        if rampFinished:
            self.currentBrakePwm = 65535 if self.brakePinHighActive else 0
            return
        progress = time.ticks_diff(time.ticks_ms(), self.brakeTimeStart) / self.brakeDuration
        self.currentBrakePwm = int(progress * 65535) if self.brakePinHighActive else int((1 - progress) * 65535)
    
    def driverReset(self) -> None:
        self.driverResetTimeout = time.ticks_add(time.ticks_ms(), 500)
        self.pinDir.value(0) 
        self.pinPwm.duty_u16(0)
        self.pinBrake.duty_u16(0)
    
    def odometryIsr(self, pin) -> None:
        if self.pinImp.value() == 0: return
        if time.ticks_diff(self.impMinTime, time.ticks_ms()) > 0: return
        self.impMinTime = time.ticks_add(time.ticks_ms(), 3)
        self.odomTicks += 1