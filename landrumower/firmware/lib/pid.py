import time

class Pid:
    prevError = 0.0
    prevTime = time.ticks_ms()
    integral = 0.0
    output = 0.0
    maxOutput = 65535
    minOutput = -65535
    
    def __init__(self, kP: float, kI: float, kD: float, dT: float) -> None:
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.dT = dT
    
    def control(self, target, current) -> float:
        currentTime = time.ticks_ms()
        deltaTime = (currentTime - self.prevTime) /1000.0
        if deltaTime > self.dT:
            error = target - current
            self.integral += error * deltaTime
            derivative = (error - self.prevError) / deltaTime if deltaTime > 0  else 0
            self.output = self.kP * error + self.kI * self.integral + self.kD * derivative
            self.prevError = error
            self.prevTime = currentTime
            self.output = max(self.minOutput, min(self.maxOutput, self.output))
            return self.output
        else:
            return self.output
    
    def reset(self) -> None:
        self.prevError = 0.0
        self.prevTime = time.ticks_ms()
        self.integral = 0.0
        self.output = 0.0
        