import time

class Pid:
    scale = 1000 # scale factor for fixed-point arithmetic
    prevError = 0
    prevTime = time.ticks_ms()
    integral = 0
    output = 0
    maxOutput = 65535
    minOutput = -65535
    
    def __init__(self, kP: float, kI: float, kD: float, dT: float) -> None:
        self.kP = int(kP * self.scale)
        self.kI = int(kI * self.scale)
        self.kD = int(kD * self.scale)
        self.dT = dT
    
    def setParameters(self, kP = None, kI = None, kD = None) -> None:
        if kP is not None and int(kP * self.scale) != self.kP: 
            self.kP = int(kP * self.scale)
        if kI is not None and int(kI * self.scale) != self.kI: 
            self.kI = int(kI * self.scale)
        if kD is not None and int(kD * self.scale) != self.kD: 
            self.kD = int(kD * self.scale)
    
    def control(self, target, current) -> int:
        currentTime = time.ticks_ms()
        deltaTime = (currentTime - self.prevTime)
        if deltaTime > self.dT:
            error = (target - current) * self.scale
            self.integral += (error * deltaTime) // self.scale
            derivative = ((error - self.prevError) * self.scale) // deltaTime if deltaTime > 0  else 0
            self.output = (self.kP * error + self.kI * self.integral + self.kD * derivative) // self.scale
            self.prevError = error
            self.prevTime = currentTime
            self.output = max(self.minOutput, min(self.maxOutput, self.output))
        return int(self.output // self.scale)
    
    def reset(self) -> None:
        self.prevError = 0.0
        self.prevTime = time.ticks_ms()
        self.integral = 0.0
        self.output = 0.0
        