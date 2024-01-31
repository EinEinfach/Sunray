# Landrumower MCU (Raspberry Pi Pico) fimrware (based on RM18.ino Alfred MCU fw)

# packeage imports
from machine import Pin
from machine import PWM
from machine import ADC
from machine import I2C
from machine import UART
from machine import WDT
import time

from utime import sleep

# local imports
from ina226 import INA226

VER = "Landrumower RPI Pico 1.0.0"

# pin definition
pinLift1 = Pin(20, Pin.IN, Pin.PULL_UP)
pinBumperX = Pin(18, Pin.IN, Pin.PULL_UP)
pinBumperY = Pin(19, Pin.IN, Pin.PULL_UP)
# pinChargeV = ADC(Pin(26))

pinMotorRightImp = Pin(2, Pin.IN)
pinMotorRightPWM = PWM(Pin(3))
pinMotorRightDir = Pin(4, Pin.OUT)
pinMotorRightBrake = Pin(5, Pin.OUT)

pinMotorLeftImp = Pin(6, Pin.IN)
pinMotorLeftPWM = PWM(Pin(7))
pinMotorLeftDir = Pin(8, Pin.OUT)
pinMotorLeftBrake = Pin(9, Pin.OUT)

pinMotorMowImp = Pin(10, Pin.IN)
pinMotorMowPWM = PWM(Pin(11))
pinMotorMowDir = Pin(12, Pin.OUT)
pinMotorMowBrake = Pin(13, Pin.OUT)

# activate debug
DEBUG = False
DEBUG2 = True

# bl driver spec
FREQ = 20000 #JYQD2021 best results
FREQ_MOW = 10000 #No data for mow bl driver available
INABATADRESS = 64 # (A0 and A1 on GND)
INAMOWADRESS = 65 # (A0 on VCC)
INALEFTADRESS = 68
INARIGHTADRESS = 69

# global variables
odomTicksLeft = 0
odomTicksRight = 0
odomTicksMow = 0

motorLeftTicksTimeout = time.ticks_ms()
motorRightTicksTimeout = time.ticks_ms()
motorMowTicksTimeout = time.ticks_ms()

stopButton = False

batVoltage = 0
chgVoltage = 0
chgCurrent = 0
chgCurrentLP = 0
mowCurr = 0
mowCurrLP = 0
motorLeftCurr = 0
motorRightCurr = 0
motorLeftCurrLP = 0
motorRightCurrLP = 0
batteryTemp = 0    
leftSpeedSet = 0
rightSpeedSet = 0
mowSpeedSet = 0
motorOverload = False
motorMowFault = False
bumperX = 0
bumperY = 0
liftLeft = 0
liftRight = 0
rain = 0
rainLP = 0
raining = 0
lift = 0
bumper = 0
ovCheck = 0 
enableTractionBrakesLeft = False
enableTractionBrakesRight = False
chargerConnected = False

cmd = ""
cmdResponse = ""

motorTimeout = time.ticks_add(time.ticks_ms(), 0)
motorOverloadTimeout = time.ticks_add(time.ticks_ms(), 0)
nextBatTime = time.ticks_add(time.ticks_ms(), 0)
nextMotorSenseTime = time.ticks_add(time.ticks_ms(), 0)
nextMotorControlTime = time.ticks_add(time.ticks_ms(), 0)
mowBrakeStateTimout = time.ticks_add(time.ticks_ms(), 0)
mowBrakeState = 0

nextInfoTime = time.ticks_add(time.ticks_ms(), 0)
lps = 0


pin = Pin("LED", Pin.OUT)

UART0 = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1), bits=8, parity=None, stop=1, timeout=10)
I2C0 = I2C(0, sda=Pin(16), scl=Pin(17), freq=400000, timeout=1000)

# calibrate INA I2C devices
inabat = INA226(I2C0, INABATADRESS)
inamow = INA226(I2C0, INAMOWADRESS)
inaleft = INA226(I2C0, INALEFTADRESS)
inaright = INA226(I2C0, INARIGHTADRESS)

try: 
    inabat.set_calibration()
except Exception as e:
    print(f"INA226 error: {e}")

# cmd answer with crc
def cmdAnswer(s: str) -> None:
    global cmdResponse
    crc = hex(sum(s.encode('ascii')) % 256)
    cmdResponse = s+","+crc+"\r\n"

def OdometryMowISR(pin) -> None:
    global motorMowTicksTimeout
    global odomTicksMow

    if pinMotorMowImp.value() == 0: return
    if time.ticks_diff(motorMowTicksTimeout, time.ticks_ms()) > 0: return
    motorMowTicksTimeout = time.ticks_add(time.ticks_ms(), 1)
    odomTicksMow += 1

def OdometryLeftISR(pin) -> None:
    global motorLeftTicksTimeout
    global odomTicksLeft

    if pinMotorLeftImp.value() == 0: return
    if time.ticks_diff(motorLeftTicksTimeout, time.ticks_ms()) > 0: return
    motorLeftTicksTimeout = time.ticks_add(time.ticks_ms(), 1)
    odomTicksLeft += 1

def OdometryRightISR(pin) -> None:
    global motorRightTicksTimeout
    global odomTicksRight

    if pinMotorRightImp.value() == 0: return
    if time.ticks_diff(motorRightTicksTimeout, time.ticks_ms()) > 0: return
    motorRightTicksTimeout = time.ticks_add(time.ticks_ms(), 1)
    odomTicksRight += 1

def mower() -> None:
    global mowSpeedSet
    global mowBrakeStateTimout
    global mowBrakeState

    if abs(mowSpeedSet) > 0:
        if time.ticks_diff(mowBrakeStateTimout,time.ticks_ms()) <= 0:
            mowBrakeStateTimout = time.ticks_add(time.ticks_ms(), 1000)
            if mowSpeedSet >= 0:
                pinMotorMowDir.value(1)
            else:
                pinMotorMowDir.value(0)
            if mowBrakeState >= 3:
                pinMotorMowPWM.freq(FREQ_MOW)
                pinMotorMowPWM.duty_u16(int((abs(mowSpeedSet)*65535)/255))
            if mowBrakeState < 20:
                mowBrakeState += 1
    else:
        mowBrakeStateTimout = time.ticks_ms()
        mowBrakeState = 0
        pinMotorMowPWM.freq(FREQ_MOW)
        pinMotorMowPWM.duty_u16(0)
        pinMotorMowBrake.value(1)

# 0 = off, 255 = full speed
def motor() -> None:
    global enableTractionBrakesLeft
    global enableTractionBrakesRight
    global leftSpeedSet
    global rightSpeedSet

    enableTractionBrakesLeft = False
    enableTractionBrakesRight = False

    if motorOverload:
        leftSpeedSet = 0
        rightSpeedSet = 0
    
    # traction brakes
    if leftSpeedSet == 0:
        enableTractionBrakesLeft = True
    if rightSpeedSet == 0:
        enableTractionBrakesRight = True
    
    # left traction motor
    if leftSpeedSet <= 0:
        pinMotorLeftDir.value(0)
    else:
        pinMotorLeftDir.value(1)
    pinMotorLeftBrake.value(int(enableTractionBrakesLeft))
    pinMotorLeftPWM.freq(FREQ)
    pinMotorLeftPWM.duty_u16(int((abs(leftSpeedSet)*65535)/255))

    # right traction motor
    if rightSpeedSet <= 0:
        pinMotorRightDir.value(1)
    else:
        pinMotorRightDir.value(0)
    pinMotorRightBrake.value(int(enableTractionBrakesRight))
    pinMotorRightPWM.freq(FREQ)
    pinMotorRightPWM.duty_u16(int((abs(rightSpeedSet)*65535)/255))

def readSensorHighFrequency() -> None:
    global chgVoltage
    global chargerConnected
    global chgCurrentLP
    global batVoltageLP

    try:
        chgCurrentLP = inabat.current
        if chgCurrentLP < 0:
            chgCurrentLP = abs(chgCurrentLP)
            chargerConnected = True
            chgVoltage = batVoltageLP
        else:
            chargerConnected = False
            chgVoltage = 0
    except Exception as e:
        print(f"Error while reading INA data: {e}")
        chargerConnected = False
        chgCurrentLP = 0
        chgVoltage = 0

def readSensors() -> None:
    global batVoltage
    global batVoltageLP
    global batteryTemp
    global ovCheck
    global motorMowFault
    global rain
    global rainLP
    global raining
    global liftRight
    global liftLeft
    global lift
    global bumperX
    global bumperY
    global bumper

    # battery voltage
    try:
        batVoltage = inabat.bus_voltage + inabat.shunt_voltage
        w = 0.99
        batVoltageLP = w * batVoltageLP + (1-w) * batVoltage
    except Exception as e:
        print(f"Error while reading INA data: {e}")
        batVoltage = 0
        batVoltageLP = 0

    # rain 

    # lift
    liftLeft = pinLift1.value()
    liftRight = 0
    lift = int(liftLeft or liftRight)

    # bumper
    bumperX = pinBumperX.value()
    bumperY = pinBumperY.value()
    bumper = int(bumperX or bumperY)

    # dummys
    batteryTemp = 20 
    ovCheck = False 
    motorMowFault = False
    rain = 0
    rainLP = 0
    raining = (rainLP > 50)

def readMotorCurrent() -> None:
    global nextMotorSenseTime
    global mowCurrLP
    global motorRightCurrLP
    global motorLeftCurrLP
    global motorOverload
    global motorOverloadTimeout

    motorLeftCurrLP = abs(inaleft.current)
    motorRightCurrLP = abs(inaright.current)
    mowCurrLP = abs(inamow.current)

    if mowCurrLP > 3 or motorLeftCurrLP > 1.5 or motorRightCurrLP > 1.5:
        motorOverload = True
        motorOverloadTimeout = time.ticks_add(time.ticks_ms, 2000)
    

# request motor
# AT+M,20,20,1
def cmdMotor() -> None:
    global leftSpeedSet
    global rightSpeedSet
    global mowSpeedSet
    global motorTimeout

    cmd_splited = cmd.split(",")
    if len(cmd_splited) != 4:
        return
    right = int(cmd_splited[1])
    left = int(cmd_splited[2])
    mow = int(cmd_splited[3])
    if DEBUG:
        print(f"left={left}")
        print(f"right={right}")
        print(f"mow={mow}")
    leftSpeedSet = left
    rightSpeedSet = right
    mowSpeedSet = mow
    motorTimeout = time.ticks_add(time.ticks_ms(), 3000)
    s = f"M,{odomTicksRight},{odomTicksLeft},{odomTicksMow},{chgVoltage},{int(bumper)},{int(lift)},{int(stopButton)}"
    cmdAnswer(s)

# perform hang test (watchdog should trigger)
def cmdTriggerWatchdog() -> None:
    s = f"Y"
    cmdAnswer(s)
    while True: # never returns
        pass

# request version
def cmdVersion() -> None:
    s = f"V,{VER}"
    cmdAnswer(s)

# request summary
def cmdSummary() -> None:
    s = f"S,{batVoltage},{chgVoltage},{chgCurrentLP},{int(lift)},{int(bumper)},{int(raining)},{int(motorOverload)},{mowCurrLP},{motorLeftCurrLP},{motorRightCurrLP},{batteryTemp}"
    cmdAnswer(s)

# process request
def processCmd(checkCRC: bool) -> None:
    global cmdResponse
    global cmd

    cmdResponse = ""
    if len(cmd) < 4:
        return
    expectedCRC = 0
    try:
        idx = cmd.rindex(",")
    except Exception as e:
        print(f"Received data are corrupt: {e}")
        return
    if idx < 1:
        if checkCRC:
            if DEBUG:
                print("CRC ERROR")
            return
    else:
        cmd_splited = cmd.split(",")
        crc = cmd_splited[-1]
        cmd_no_crc = cmd.replace(','+crc, '')
        crc = hex(int(crc, 16))
        expectedCRC = hex(sum(cmd_no_crc.encode('ascii')) % 256)
        if expectedCRC != crc:
            if (checkCRC):
                if DEBUG:
                    print("CRC ERROR")
                    print(f'{crc}, {expectedCRC}')
                return
        else:
            cmd = cmd_no_crc
        if cmd_splited[0][0] != "A": return
        if cmd_splited[0][1] != "T": return
        if cmd_splited[0][2] != "+": return
        if cmd_splited[0] == "AT+V": cmdVersion()
        if cmd_splited[0] == "AT+M": cmdMotor()
        if cmd_splited[0] == "AT+S": cmdSummary()
        if cmd_splited[0] == "AT+Y":
            if len(cmd) <= 4:
                cmdTriggerWatchdog() # for developers
            else:
                pass
                #if cmd_splited[4] == "2": cmdGNSSReboot() # for developers
                #if cmd_splited[4] == "3": cmdSwitchOffRobot() # for developers

# process uart input
def processConsole() -> None:
    global cmd
    if UART0.any() > 0:
        rawData = UART0.readline()
        cmd = rawData.decode("ascii")
        if DEBUG:
            print(f"Received command: {cmd}")
        processCmd(True)
        if DEBUG:
            print(f"Response: {cmdResponse}")
        UART0.write(cmdResponse)
        cmd = ""

def printInfo() -> None:
    print(f"tim={time.ticks_add(time.ticks_ms(), 0)}, lps={lps}, bat={batVoltage}V, chg={chgVoltage}V/{chgCurrentLP}A, mF={motorMowFault}, imp={odomTicksLeft},{odomTicksRight},{odomTicksMow}, lift={liftLeft},{liftRight}, bump={bumperX},{bumperY}, rain={rain}, ov={ovCheck}")
    
if DEBUG2:
    i2c_devices = I2C0.scan()
    for device in i2c_devices:
        print(f"Found devices on I2C bus: {hex(device)}")

pinMotorMowImp.irq(trigger=Pin.IRQ_RISING, handler=OdometryMowISR)
pinMotorLeftImp.irq(trigger=Pin.IRQ_RISING, handler=OdometryLeftISR)
pinMotorRightImp.irq(trigger=Pin.IRQ_RISING, handler=OdometryRightISR)

# activate watchdog
wdt = WDT(timeout=6000)

# main loop
while True:

    if time.ticks_diff(nextMotorControlTime, time.ticks_ms()) <= 0:
        nextMotorControlTime = time.ticks_add(time.ticks_ms(), 20)
        motor()
        mower()
        readSensorHighFrequency()

    if time.ticks_diff(motorTimeout, time.ticks_ms()) <= 0:
        # if DEBUG:
        #     print("Motor timeout")
        pin.value(1)
        leftSpeedSet = 0
        rightSpeedSet = 0
        mowSpeedSet = 0
    else:
        pin.value(0)

    processConsole()

    if time.ticks_diff(nextInfoTime, time.ticks_ms()) <= 0:
        nextInfoTime = time.ticks_add(time.ticks_ms(), 1000)    
        if DEBUG2:
            printInfo()
        lps = 0
    
    if time.ticks_diff(nextBatTime, time.ticks_ms()) <= 0:
        nextBatTime = time.ticks_add(time.ticks_ms(), 100)
        readSensors()
    
    if time.ticks_diff(nextMotorSenseTime, time.ticks_ms()) <= 0:
        nextMotorSenseTime = time.ticks_add(time.ticks_ms(), 100)
        readMotorCurrent()
    
    if time.ticks_diff(motorOverloadTimeout, time.ticks_ms()) <= 0:
        motorOverload = False

    
    lps += 1
    wdt.feed()
pin.off()

