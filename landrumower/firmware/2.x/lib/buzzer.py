from machine import Pin, PWM
import time

LOUDNESS = 65000

class Buzzer:
    mainUnitState: str = ""
    sound: bool = False
    playPattern: list[int] = []
    currentTime: int = 0
    loudness: int = 0
    nextRunTime: int = 0
    runFrequency: int = 20 #50Hz

    def __init__(self, freq: int, pinPwm: PWM) -> None:
        self.freq = freq
        self.pinPwm = pinPwm

        pinPwm.freq(freq)
    
    def stopPlaying(self) -> None:
        self.sound = False
        self.loudness = 0
        self.playPattern = []
        self.currentTime = 0

    def playInfo(self, loudness: int) -> None:
        self.stopPlaying()
        self.loudness = loudness
        self.playPattern = [1000]
    
    def playImuCalibration(self, loudness: int) -> None:
        self.stopPlaying()
        self.loudness = loudness
        self.playPattern = [100, -1000, 100, -1000, 100, -1000, 100, -1000, 100, -1000, 100, -1000, 100, -1000, 100, -1000, 100, -1000, 100, -1000, 100, -1000, 100]

    def playWanrning(self, loudness: int) -> None:
        self.stopPlaying()
        self.loudness = loudness
        self.playPattern = [500, -1000, 500, -1000, 500]
    
    def playShutdown(self, loudness: int) -> None:
        self.stopPlaying()
        self.loudness = loudness
        self.playPattern = [100, -20, 100, -3000, 100, -20, 100, -3000, 100, -20, 100, -3000, 100, -20, 100, -3000, 100, -20, 100, -3000, 100, -20, 100, -3000, 100, -20, 100, -3000, 100, -20, 100, -3000] 
    
    def checkPlayPattern(self, mainUnitState: str) -> None:
        self.mainUnitState = mainUnitState
        if self.mainUnitState == "boot" or self.mainUnitState == "idle" or self.mainUnitState == "charge" or self.mainUnitState == "dock" or self.mainUnitState == "gps recovery" or self.mainUnitState == "gps wait fix" or self.mainUnitState == "gps wait float":
            self.playInfo(LOUDNESS)
        elif self.mainUnitState == "imu calibration":
            self.playImuCalibration(LOUDNESS)
        elif self.mainUnitState == "mow" or self.mainUnitState == "error" or self.mainUnitState == "escape forward" or self.mainUnitState == "escape reverse":
            self.playWanrning(LOUDNESS)
    
    def run(self, mainUnitState: str) -> None:
        if (time.ticks_diff(time.ticks_ms(), self.nextRunTime)) > 0:
            self.nextRunTime = time.ticks_add(time.ticks_ms(), self.runFrequency)
            if self.mainUnitState != mainUnitState and self.mainUnitState != "shutdown":
                self.checkPlayPattern(mainUnitState)
            if time.ticks_diff(self.currentTime, time.ticks_ms()) < 0:
                if self.playPattern == []:
                    self.stopPlaying()
                elif self.playPattern[0] > 0:
                    self.sound = True
                    self.currentTime = time.ticks_add(time.ticks_ms(), abs(self.playPattern[0]))
                    self.playPattern.pop(0)
                else:
                    self.sound = False
                    self.currentTime = time.ticks_add(time.ticks_ms(), abs(self.playPattern[0]))
                    self.playPattern.pop(0)

            if self.sound:
                self.pinPwm.duty_u16(self.loudness)
            else:
                self.pinPwm.duty_u16(0)

