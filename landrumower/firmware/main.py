# Landrumower MCU (Raspberry Pi Pico) fimrware (based on RM18.ino Alfred MCU fw)

# Configuration:

# activate debug
DEBUG = False
DEBUG2 = False

# activate hil
HIL = False

# overload current for motors
OVERLOADCURRENT_GEAR = 0.8
OVERLOADCURRENT_MOW = 3

# critical voltage pico will cut off power supply
CRITICALVOLTAGE = 17

# bl drver specific settings
FREQ = 20000 #JYQD2021 best results
FREQ_MOW = 10000 #No data for mow bl driver available

# INA
INABATADRESS = 64 # (A0 and A1 on GND)
INAMOWADRESS = 65 # (A0 on VCC)
INALEFTADRESS = 68
INARIGHTADRESS = 69

# LCD
LCD = True #If you don't have a LCD set this to False
LCDADRESS = 39 
LCD_NUM_ROWS = 2 
LCD_NUM_COLUMNS = 16

# Current facttor
CURRENTFACTOR = 10 #If you changed your ina shunt from 100mOhm to 10mOhm set this value to 10, without any modifications set it to 1


# packeage imports
from machine import Pin
from machine import PWM
from machine import ADC
from machine import I2C
from machine import UART
from machine import WDT

import time, sys, select
from utime import sleep_ms

# local imports
from lib.ina226 import INA226
from lib.lcd_api import LcdApi
from lib.pico_i2c_lcd import I2cLcd

VERNR = "1.10.0"
VER = f"Landrumower RPI Pico {VERNR}" # Trying to fix driver recovery function. Reduced overload current for gear motors

# pin definition
pinRain = ADC(Pin(28))
pinLift1 = Pin(20, Pin.IN, Pin.PULL_DOWN)
pinBumperX = Pin(18, Pin.IN, Pin.PULL_DOWN)
pinBumperY = Pin(19, Pin.IN, Pin.PULL_DOWN)
pinBatterySwitch = Pin(22, Pin.OUT)
pinStopButton = Pin(21, Pin.IN, Pin.PULL_UP)
# pinChargeV = ADC(Pin(26))

pinMotorRightImp = Pin(2, Pin.IN)
pinMotorRightPWM = PWM(Pin(3))
pinMotorRightDir = Pin(4, Pin.OUT, Pin.PULL_DOWN)
pinMotorRightBrake = Pin(5, Pin.OUT, Pin.PULL_DOWN)

pinMotorLeftImp = Pin(6, Pin.IN)
pinMotorLeftPWM = PWM(Pin(7))
pinMotorLeftDir = Pin(8, Pin.OUT, Pin.PULL_DOWN)
pinMotorLeftBrake = Pin(9, Pin.OUT, Pin.PULL_DOWN)

pinMotorMowImp = Pin(10, Pin.IN)
pinMotorMowPWM = PWM(Pin(11))
pinMotorMowDir = Pin(12, Pin.OUT)
pinMotorMowBrake = Pin(13, Pin.OUT)

# global variables
odomTicksLeft = 0
odomTicksRight = 0
odomTicksMow = 0

motorLeftTicksTimeout = time.ticks_ms()
motorRightTicksTimeout = time.ticks_ms()
motorMowTicksTimeout = time.ticks_ms()
stopButtonTimeout = time.ticks_ms()

stopButton = 0

batVoltage = 0
batVoltageLP = 0
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
rain = 65535
rainLP = 65535
raining = 0
lift = 0
bumper = 0
ovCheck = 0 
enableTractionBrakesLeft = False
enableTractionBrakesRight = False
chargerConnected = False
motorControlLocked = False
motorConrolLockedTime = time.ticks_add(time.ticks_ms(), 0)

cmd = ""
cmdResponse = ""

motorTimeout = time.ticks_add(time.ticks_ms(), 0)
motorOverloadTimeout = time.ticks_add(time.ticks_ms(), 0)
nextBatTime = time.ticks_add(time.ticks_ms(), 0)
nextMotorSenseTime = time.ticks_add(time.ticks_ms(), 0)
nextMotorControlTime = time.ticks_add(time.ticks_ms(), 0)
mowBrakeStateTimout = time.ticks_add(time.ticks_ms(), 0)
mowBrakeState = 0
switchBatteryDebounceCtr = 0
requestShutdown = False

nextInfoTime = time.ticks_add(time.ticks_ms(), 0)
lps = 0

lcdRequestedMessage1 = ""
lcdRequestedMessage2 = ""
lcdPrintedMessage1 = ""
lcdPrintedMessage2 = ""

pin = Pin("LED", Pin.OUT)

UART0 = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1), bits=8, parity=None, stop=1, timeout=10)
I2C0 = I2C(0, sda=Pin(16), scl=Pin(17), freq=400000, timeout=1000)  # I2C0 has 3.3V logic
I2C1 = I2C(1, sda=Pin(14), scl=Pin(15), freq=400000, timeout=1000)  # I2C1 has 5.0V logic

# calibrate INA I2C devices
try:
    if not HIL:
        inabat = INA226(I2C0, INABATADRESS)
        inamow = INA226(I2C0, INAMOWADRESS)
        inaleft = INA226(I2C0, INALEFTADRESS)
        inaright = INA226(I2C0, INARIGHTADRESS)
        inabat.set_calibration()
        inamow.set_calibration()
        inaleft.set_calibration()
        inaright.set_calibration()
except Exception as e:
    print(f"I2C error, could not calibrate INA226 for current measurement: {e}")

# calibrate LCD I2C device
try:
    if LCD:
        lcd = I2cLcd(I2C1, LCDADRESS, LCD_NUM_ROWS, LCD_NUM_COLUMNS)
except Exception as e:
    print(f"I2C error, could not calibrate LCD: {e}")

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
    global motorControlLocked
    global motorConrolLockedTime

    enableTractionBrakesLeft = False
    enableTractionBrakesRight = False

    if not motorControlLocked:
        pinMotorLeftBrake.value(0)
        pinMotorLeftDir.value(0)
        pinMotorLeftPWM.duty_u16(0)
        
        pinMotorRightBrake.value(0)
        pinMotorRightDir.value(0)
        pinMotorRightPWM.duty_u16(0)

        pinMotorMowBrake.value(0)
        pinMotorMowDir.value(0)
        pinMotorMowPWM.duty_u16(0)
        if DEBUG:
            print(f"Motor control locked. Time before unlock: {time.ticks_diff(motorConrolLockedTime, time.ticks_ms())}")
        return
        
    if motorOverload:
        leftSpeedSet = 0
        rightSpeedSet = 0
        if DEBUG:
            print(f"Motor overload. Time before unlock: {time.ticks_diff(motorOverloadTimeout, time.ticks_ms())}")
    
    # traction brakes
    if leftSpeedSet == 0:
        enableTractionBrakesLeft = True
    if rightSpeedSet == 0:
        enableTractionBrakesRight = True
    
    # left traction motor
    if leftSpeedSet > 0:
        pinMotorLeftDir.value(1)
    else:
        pinMotorLeftDir.value(0)
    pinMotorLeftBrake.value(int(enableTractionBrakesLeft))
    pinMotorLeftPWM.freq(FREQ)
    pinMotorLeftPWM.duty_u16(int((abs(leftSpeedSet)*65535)/255))

    # right traction motor
    if rightSpeedSet < 0:
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
        if not HIL:
            chgCurrentLP = inabat.current
            chgCurrentLP = chgCurrentLP * CURRENTFACTOR
            if chgCurrentLP <= 0.1:
                chgCurrentLP = abs(chgCurrentLP)
                chargerConnected = True
                chgVoltage = batVoltageLP
            else:
                chargerConnected = False
                chgVoltage = 0
        else:
            chgCurrentLP = 1.0
            chargerConnected = False
            chgVoltage = 0
    except Exception as e:
        print(f"Error while reading INA(Battery) data: {e}")
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
    global stopButton
    global stopButtonTimeout

    # battery voltage
    w = 0.99
    try:
        if not HIL:
            batVoltage = inabat.bus_voltage + inabat.shunt_voltage 
        else:
            batVoltage = 28.0
    except Exception as e:
        print(f"Error while reading INA(Battery) data: {e}")
        batVoltage = 0
    batVoltageLP = w * batVoltageLP + (1 - w) * batVoltage

    # rain 
    w = 0.99
    rain = pinRain.read_u16()
    rainLP = w * rainLP + (1 - w) * rain
    raining = int((((rainLP * 100) / 65535) < 50))

    # lift
    liftLeft = pinLift1.value()
    liftRight = 0
    lift = int(liftLeft or liftRight)

    # bumper
    bumperX = pinBumperX.value()
    bumperY = pinBumperY.value()
    bumper = int(bumperX or bumperY)

    # stop button
    if pinStopButton.value() == 1:
        stopButton = 1
    else:
        stopButton = 0

    # dummys
    batteryTemp = 20 
    ovCheck = False 
    motorMowFault = False

def readMotorCurrent() -> None:
    global nextMotorSenseTime
    global mowCurrLP
    global motorRightCurrLP
    global motorLeftCurrLP
    global motorOverload
    global motorOverloadTimeout
    try:
        if not HIL:
            motorLeftCurrLP = abs(inaleft.current)
            motorLeftCurrLP = motorLeftCurrLP * CURRENTFACTOR
            motorRightCurrLP = abs(inaright.current)
            motorRightCurrLP = motorRightCurrLP * CURRENTFACTOR
            mowCurrLP = abs(inamow.current)
            mowCurrLP = mowCurrLP * CURRENTFACTOR
        else:
            motorLeftCurrLP = 0
            motorRightCurrLP = 0
            mowCurrLP = 0
    except Exception as e:
        motorOverload = True
        motorOverloadTimeout = time.ticks_add(time.ticks_ms, 2000)
        print(f"Error while reading INA(Motors) data: {e}")

    if mowCurrLP > OVERLOADCURRENT_MOW or motorLeftCurrLP > OVERLOADCURRENT_GEAR or motorRightCurrLP > OVERLOADCURRENT_GEAR:
        motorOverload = True
        motorOverloadTimeout = time.ticks_add(time.ticks_ms, 2000)

def keepPowerOn() -> None:
    global switchBatteryDebounceCtr
    global requestShutdown
    if (batVoltageLP < CRITICALVOLTAGE and batVoltage < CRITICALVOLTAGE) or requestShutdown:
        switchBatteryDebounceCtr +=1
    else:
        switchBatteryDebounceCtr = 0

    if switchBatteryDebounceCtr == 10 or switchBatteryDebounceCtr == 20 or switchBatteryDebounceCtr == 30:
        if requestShutdown:
            print("Main unit requests shutdown. Delay time started")
        else:
            print("Battery voltage critical level reached")
            print("SHUTTING DOWN")
            pinBatterySwitch.value(0)
    if switchBatteryDebounceCtr > 40:
        print(f"SHUTTING DOWN")
        pinBatterySwitch.value(0)
    
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

# perform reset motor faults
def cmdResetMotorFaults() -> None:
    global motorControlLocked
    global motorConrolLockedTime
    global lcdRequestedMessage1
    global lcdRequestedMessage2
    motorConrolLockedTime = time.ticks_add(time.ticks_ms(), 400)
    motorControlLocked = True
    lcdRequestedMessage1 = "motor drivers"
    lcdRequestedMessage2 = "recovery"
    if DEBUG:
        print("Motor faults request. Reseting drivers")
    pinMotorRightPWM.duty_u16(0) 
    pinMotorRightDir.value(0) 
    pinMotorRightBrake.value(0)  

    pinMotorLeftPWM.duty_u16(0)
    pinMotorLeftDir.value(0)  
    pinMotorLeftBrake.value(0)
    s = f"R"
    cmdAnswer(s)

# perform shutdown
def cmdShutdown() -> None:
    global requestShutdown
    requestShutdown = True
    s = f"Y3"
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
    global lcdRequestedMessage1
    global lcdRequestedMessage2
    lcdRequestedMessage1 = f"{round(batVoltageLP, 2)}V/{round(chgCurrentLP, 2)}A"
    lcdRequestedMessage2 = f""
    s = f"S,{batVoltageLP},{chgVoltage},{chgCurrentLP},{int(lift)},{int(bumper)},{int(raining)},{int(motorOverload)},{mowCurrLP},{motorLeftCurrLP},{motorRightCurrLP},{batteryTemp}"
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
        print(f"Received data are corrupt. Data: {cmd}. Exception: {e}")
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
        if cmd_splited[0] == "AT+R": cmdResetMotorFaults()
        if cmd_splited[0] == "AT+S": cmdSummary()
        if cmd_splited[0] == "AT+Y3": cmdShutdown()
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
    if HIL:
        #read input from usb in case of HIL (hardware in the loop)
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            rawData = sys.stdin.readline().strip()
            cmd = rawData
            print(f"Received command via USB: {cmd}")
            processCmd(True)
            print(cmdResponse)  # Send response back to USB console
            cmd = ""
    else:
        #read input from uart (normal operation)
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
    print(f"tim={time.ticks_add(time.ticks_ms(), 0)}, lps={lps}, bat={batVoltageLP}V, chg={chgVoltage}V/{chgCurrentLP}A, mF={motorMowFault}, imp={odomTicksLeft},{odomTicksRight},{odomTicksMow}, curr={motorLeftCurrLP},{motorRightCurrLP},{mowCurrLP},lift={liftLeft},{liftRight}, bump={bumperX},{bumperY}, rain={rainLP, raining}, stop={stopButton}")

def printLcd() -> None:
    global lcdRequestedMessage1
    global lcdRequestedMessage2
    global lcdPrintedMessage1
    global lcdPrintedMessage2
    if lcdRequestedMessage1 != lcdPrintedMessage1 or lcdRequestedMessage2 != lcdPrintedMessage2:
        lcdPrintedMessage1 = lcdRequestedMessage1
        lcdPrintedMessage2 = lcdRequestedMessage2
        lcd.clear()
        lcd.putstr(lcdRequestedMessage1)
        lcd.move_to(0, 1)
        lcd.putstr(lcdRequestedMessage2)

# setup
# activate watchdog
wdt = WDT(timeout=6000) 

# swicht on battery
pinBatterySwitch.value(1)

# define here setup debug outputs
if DEBUG2:
    i2c_devices = I2C0.scan()
    for device in i2c_devices:
        print(f"Found devices on I2C bus: {hex(device)}")

# set ISRs
pinMotorMowImp.irq(trigger=Pin.IRQ_RISING, handler=OdometryMowISR)
pinMotorLeftImp.irq(trigger=Pin.IRQ_RISING, handler=OdometryLeftISR)
pinMotorRightImp.irq(trigger=Pin.IRQ_RISING, handler=OdometryRightISR)

# initial lcd message
lcdRequestedMessage1 = "booting..." 
lcdRequestedMessage2 = f"Ver: {VERNR}"

print(VER)

# main loop
while True:

    if time.ticks_diff(nextMotorControlTime, time.ticks_ms()) <= 0:
        nextMotorControlTime = time.ticks_add(time.ticks_ms(), 20)
        motor()
        mower()
        readSensorHighFrequency()

    # motor timeout
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
        keepPowerOn()
        if DEBUG2:
            printInfo()
        if LCD:
            printLcd()
        lps = 0
    
    # next measure time for sensors
    if time.ticks_diff(nextBatTime, time.ticks_ms()) <= 0:
        nextBatTime = time.ticks_add(time.ticks_ms(), 100)
        readSensors()

    # next measure time for motor current
    if time.ticks_diff(nextMotorSenseTime, time.ticks_ms()) <= 0:
        nextMotorSenseTime = time.ticks_add(time.ticks_ms(), 100)
        readMotorCurrent()
    
    # remove motor overload after defined time (2s)
    if time.ticks_diff(motorOverloadTimeout, time.ticks_ms()) <= 0:
        motorOverload = False

    # remove motor control lock after defined time (400ms)
    if motorControlLocked and time.ticks_diff(motorConrolLockedTime, time.ticks_ms()) <= 0:
        motorControlLocked = False

    lps += 1
    wdt.feed()
pin.off()