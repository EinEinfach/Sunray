from machine import Pin, PWM
import time

class Buzzer:
    programState: str = ""
    sound: bool = False
    playPattern: list[int] = []
    currentTime: int = 0
    loudness: int = 0

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
    
    def checkPlayPattern(self, programState: str) -> None:
        self.programState = programState
        if self.programState == "boot" or self.programState == "idle" or self.programState == "charge" or self.programState == "dock" or self.programState == "gps revovery" or self.programState == "gps wait fix" or self.programState == "gps wait float":
            self.playInfo(60000)
        elif self.programState == "imu calibration":
            self.playImuCalibration(60000)
        elif self.programState == "mow" or self.programState == "error" or self.programState == "escape forward" or self.programState == "escape reverse":
            self.playWanrning(60000)
    
    def run(self, programState: str) -> None:
        if self.programState != programState and self.programState != "shutdown":
            self.checkPlayPattern(programState)
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

