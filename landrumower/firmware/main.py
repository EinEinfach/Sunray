# Landrumower MCU (Raspberry Pi Pico) fimrware (based on RM18.ino Alfred MCU fw)

# Configuration:

# activate debug
DEBUG = False
INFO = True
INFOTIME = 1000

# activate hil
HIL = False

# overload current for motors
OVERLOADCURRENT_GEAR = 10
OVERLOADCURRENT_MOW = 10

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

import time, sys, select, _thread
from utime import sleep_ms

# local imports
from lib.ina226 import INA226
from lib.pico_i2c_lcd import I2cLcd
from lib.motor import Motor
from lib.pid import Pid

VERNR = "2.1.1"
VER = f"Landrumower RPI Pico {VERNR}" # Do not reset pid if requested speed = 0 and current speed != 0

class PicoMowerDriver:
    cmd: str = ""
    cmdResponse: str = ""
    # pin definitions
    led = Pin("LED", Pin.OUT)
    pinRain = ADC(Pin(28))
    pinLift1 = Pin(20, Pin.IN, Pin.PULL_DOWN)
    pinBumperX = Pin(18, Pin.IN, Pin.PULL_DOWN)
    pinBumperY = Pin(19, Pin.IN, Pin.PULL_DOWN)
    pinBatterySwitch = Pin(22, Pin.OUT)
    pinStopButton = Pin(21, Pin.IN, Pin.PULL_UP)
    # pinChargeV = ADC(Pin(26))

    pinMotorRightImp = Pin(2, Pin.IN)
    pinMotorRightPwm = PWM(Pin(3))
    pinMotorRightDir = Pin(4, Pin.OUT, Pin.PULL_DOWN)
    pinMotorRightBrake = PWM(Pin(5)) 

    pinMotorLeftImp = Pin(6, Pin.IN)
    pinMotorLeftPwm = PWM(Pin(7))
    pinMotorLeftDir = Pin(8, Pin.OUT, Pin.PULL_DOWN)
    pinMotorLeftBrake = PWM(Pin(9))

    pinMotorMowImp = Pin(10, Pin.IN)
    pinMotorMowPwm = PWM(Pin(11))
    pinMotorMowDir = Pin(12, Pin.OUT)
    pinMotorMowBrake = PWM(Pin(13))

    # lcd messages
    lcd1: str = "" 
    lcd2: str = ""
    lcdPrio1: str = ""
    lcdPrio2: str = ""
    lcdPrioMessage: bool = False
    lcdPrioMessageTime: int = 0

    # motor variables
    leftSpeedSet: int = 0
    rightSpeedSet: int = 0
    mowSpeedSet: int = 0
    motorTimeout: int = 0
    motorControlLocked: bool = False
    nextMotorControlTime: int = 0
    motorMowFault: bool = False
    nextMotorSenseTime: int = 0

    # battery variables
    chgVoltage: float = 0.0
    chargerConnected: bool = False 
    chgCurrentLP: float = 0.0
    batVoltage: float = 0.0
    batVoltageLP: float = 0.0
    batteryTemp: float = 0.0
    ovCheck: bool = False
    switchBatteryDebounceCtr: int = 0
    nextBatTime: int = 0

    # sensors
    rain: int = 0
    rainLP: int = 0
    raining: int = 0
    liftLeft: int = 0
    liftRight: int = 0
    lift: int = 0
    bumperX: int = 0
    bumperY: int = 0
    bumper: int = 0
    stopButton: int = 0

    # common
    requestShutdown: bool = False
    nextInfoTime: int = time.ticks_ms()
    nextKeepPowerOnTime: int = time.ticks_ms()
    lps: int = 0


    def __init__(self) -> None:
        self.wdt = WDT(timeout=6000)

        self.pinBatterySwitch.value(1)

        print("Landrumower Driver")
        print(f"Version: {VER}")
        self.uart0 = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1), bits=8, parity=None, stop=1, timeout=10)
        self.i2c0 = I2C(0, sda=Pin(16), scl=Pin(17), freq=400000)  # I2C0 has 3.3V logic
        self.i2c1 = I2C(1, sda=Pin(14), scl=Pin(15), freq=400000)  # I2C1 has 5.0V logic

        self.pinMotorRightPwm.freq(FREQ)
        self.pinMotorRightBrake.freq(FREQ)
        self.pinMotorLeftPwm.freq(FREQ)
        self.pinMotorLeftBrake.freq(FREQ)
        self.pinMotorMowPwm.freq(FREQ_MOW)
        self.pinMotorRightBrake.freq(FREQ_MOW)

        if HIL:
            if DEBUG:
                print("HIL mode activated")
        else:
            try:
                self.inabat = INA226(self.i2c0, INABATADRESS)
                self.inamow = INA226(self.i2c0, INAMOWADRESS)
                self.inaleft = INA226(self.i2c0, INALEFTADRESS)
                self.inaright = INA226(self.i2c0, INARIGHTADRESS)
                self.inabat.set_calibration()
                self.inamow.set_calibration()
                self.inaleft.set_calibration()
                self.inaright.set_calibration()
            except Exception as e:
                print(f"ERROR: INA226 not found: {e}")
        if LCD:
            try:
                self.lcd = I2cLcd(self.i2c1, LCDADRESS, LCD_NUM_ROWS, LCD_NUM_COLUMNS)
                self.lcd.backlight_on()
                self.lcd.clear()
                self.lcd1 = "Landrumower pico"
                self.lcd2 = f"Version: {VERNR}"
            except Exception as e:
                print(f"ERROR: LCD not found: {e}")   

        self.motorLeft = Motor('gear',
                               self.pinMotorLeftDir, 
                               self.pinMotorLeftBrake, 
                               self.pinMotorLeftPwm, 
                               self.pinMotorLeftImp, 
                               brakePinHighActive=True, 
                               directionPinHighPositive=True, 
                               overloadThreshold=OVERLOADCURRENT_GEAR) 
        self.motorRight = Motor('gear',
                                self.pinMotorRightDir, 
                                self.pinMotorRightBrake, 
                                self.pinMotorRightPwm, 
                                self.pinMotorRightImp, 
                                brakePinHighActive=True, 
                                directionPinHighPositive=False, 
                                overloadThreshold=OVERLOADCURRENT_GEAR)
        self.motorMow = Motor('mow', 
                              self.pinMotorMowDir, 
                              self.pinMotorMowBrake, 
                              self.pinMotorMowPwm, 
                              self.pinMotorMowImp, 
                              brakePinHighActive=False, 
                              directionPinHighPositive=True, 
                              overloadThreshold=OVERLOADCURRENT_MOW)
        
        self.wdt.feed()
    
    # uart input
    def processConsole(self) -> None:
        try:
            if HIL:
                #read input from usb in case of HIL (hardware in the loop)
                if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                    rawData = sys.stdin.readline().strip()
                    self.cmd = rawData
                    print(f"Received command via USB: {self.cmd}")
                    self.processCmd(False)
                    print(self.cmdResponse)  # Send response back to USB console
                    self.cmd = ""
            else:
                #read input from uart (normal operation)
                if self.uart0.any() > 0:
                    rawData = self.uart0.readline()
                    self.cmd = rawData.decode("ascii")
                    if DEBUG:
                        print(f"Received command: {self.cmd}")
                    self.processCmd(True)
                    if DEBUG:
                        print(f"Response: {self.cmdResponse}")
                    self.uart0.write(self.cmdResponse)
                    self.cmd = ""
        except Exception as e:
            print(f"Received data are corrupt. Data: {self.cmd}. Exception: {e}")
            self.cmd = ""

    # process command
    def processCmd(self, checkCrc: bool) -> None:
        try:
            self.cmdResponse = ""
            if len(self.cmd) < 4:
                return
            expectedCRC = 0
            idx = self.cmd.rindex(",")
            if idx < 1:
                if checkCrc:
                    if DEBUG:
                        print("CRC ERROR")
                    return
            else:
                cmd_splited = self.cmd.split(",")
                crc = cmd_splited[-1]
                cmd_no_crc = self.cmd.replace(','+crc, '')
                crc = hex(int(crc, 16))
                expectedCRC = hex(sum(cmd_no_crc.encode('ascii')) % 256)
                if expectedCRC != crc and checkCrc:
                    if DEBUG:
                        print("CRC ERROR")
                        print(f'{crc}, {expectedCRC}')
                    return
                else:
                    self.cmd = cmd_no_crc
                if cmd_splited[0][0] != "A": return
                if cmd_splited[0][1] != "T": return
                if cmd_splited[0][2] != "+": return
                if cmd_splited[0] == "AT+V": self.cmdVersion()
                if cmd_splited[0] == "AT+M": self.cmdMotor()
                if cmd_splited[0] == "AT+E": self.cmdMotorControlTest()
                if cmd_splited[0] == "AT+R": self.cmdResetMotorFaults()
                if cmd_splited[0] == "AT+S": self.cmdSummary()
                if cmd_splited[0] == "AT+Y3": self.cmdShutdown()
                if cmd_splited[0] == "AT+Y":
                    if len(self.cmd) <= 4:
                        self.cmdTriggerWatchdog() # for developers
                    else:
                        pass
                        #if cmd_splited[4] == "2": cmdGNSSReboot() # for developers
                        #if cmd_splited[4] == "3": cmdSwitchOffRobot() # for developers
        except Exception as e:
            print(f"Received data are corrupt. Data: {self.cmd}. Exception: {e}")
            return
    
    # cmd answer with crc
    def cmdAnswer(self, s: str) -> None:
        try:
            if DEBUG:
                print(f"Answer: {s}")
            crc = hex(sum(s.encode('ascii')) % 256)
            self.cmdResponse = s+","+crc+"\r\n"
        except Exception as e:
            print(f"Cmd answer corrupt. Exception: {e}")

    def cmdVersion(self) -> None:
        s = f"V,{VER}"
        self.cmdAnswer(s)
    
    # request motor control test
    # AT+E
    def cmdMotorControlTest(self) -> None:
        loopCall = 1
        while loopCall <= 5:
            if loopCall%2 == 0:
                testSpeed = 0
            else:
                testSpeed = 0.1 * loopCall
                testSpeed = min(testSpeed, 0.4)
            
            self.motorLeft.setSpeed(testSpeed)
            self.motorRight.setSpeed(testSpeed)
            if testSpeed == 0:
                testTime = time.ticks_add(time.ticks_ms(), 5000)
            else:
                testTime = time.ticks_add(time.ticks_ms(), 10000)
            nextInfoTime = time.ticks_ms()
            while time.ticks_diff(testTime, time.ticks_ms()) >= 0:
                if time.ticks_diff(nextInfoTime, time.ticks_ms()) <= 0:
                    nextInfoTime = time.ticks_add(time.ticks_ms(), 100)
                    print(f"left pwm: {self.motorLeft.currentTrqPwm} left sp: {testSpeed} left: {self.motorLeft.currentSpeedLp} right pwm: {self.motorRight.currentTrqPwm} right sp: {testSpeed} right: {self.motorRight.currentSpeedLp}")
                self.motorLeft.run()
                self.motorRight.run()
                time.sleep(0.003)
                self.wdt.feed()
            loopCall += 1

    # request motor
    # AT+M,20,20,1
    def cmdMotor(self) -> None:
        try:
            cmd_splited = self.cmd.split(",")
            if len(cmd_splited) == 4:
                right = int(cmd_splited[1])
                left = int(cmd_splited[2])
                mow = int(cmd_splited[3])
            elif len(cmd_splited) == 6:
                right = float(cmd_splited[4])
                left = float(cmd_splited[5])
                mow = int(cmd_splited[3])
            else:
                return
            if DEBUG:
                print(f"left={left}")
                print(f"right={right}")
                print(f"mow={mow}")
            self.motorLeft.setSpeed(left)
            self.motorRight.setSpeed(right)
            self.motorMow.setSpeed(mow)
            self.motorTimeout = time.ticks_add(time.ticks_ms(), 30000)
            s = f"M,{self.motorRight.odomTicks},{self.motorLeft.odomTicks},{self.motorMow.odomTicks},{self.chgVoltage},{int(self.bumper)},{int(self.lift)},{int(self.stopButton)}"
            self.cmdAnswer(s)
        except Exception as e:
            print(f"Command motor invalid. Exception: {e}")
    
    # perform reset motor faults
    def cmdResetMotorFaults(self) -> None:
        self.motorLeft.lockMotorControl()
        self.motorRight.lockMotorControl()
        self.motorMow.lockMotorControl()
        self.lcdPrioMessage = True
        self.lcdPrioMessageTime = time.ticks_add(time.ticks_ms(), 5000)
        self.motorControlLocked = True
        self.lcd1 = "motor drivers"
        self.lcd2 = "recovery"
        if DEBUG:
            print("Motor faults request. Reseting drivers")
        self.motorLeft.driverReset()
        self.motorRight.driverReset()
        s = f"R"
        self.cmdAnswer(s)

    # request summary
    def cmdSummary(self) -> None:
        try:
            self.lcd1 = f""
            self.lcd2  = f"{round(self.batVoltageLP, 1)}V/{round(self.chgCurrentLP, 1)}A"
            cmd_splited = self.cmd.split(",")
            for cmdDataIdx in range(len(cmd_splited)):
                if cmdDataIdx == 1:
                    self.lcd1 = self.sunrayStateToText(int(cmd_splited[1]))
                    if DEBUG:
                        print(f"Sunray state: {self.lcd1}")
                elif cmdDataIdx == 2:
                    if DEBUG:
                        print(f"Received new kP parameter: {cmd_splited[2]}")
                    self.motorLeft.motorControl.setParameters(kP = float(cmd_splited[2]))
                    self.motorRight.motorControl.setParameters(kP = float(cmd_splited[2]))
                elif cmdDataIdx == 3:
                    if DEBUG:
                        print(f"Received new kI parameter: {cmd_splited[3]}")
                    self.motorLeft.motorControl.setParameters(kI = float(cmd_splited[3]))
                    self.motorRight.motorControl.setParameters(kI = float(cmd_splited[3]))
                elif cmdDataIdx == 4:
                    if DEBUG:
                        print(f"Received new kD parameter: {cmd_splited[4]}")
                    self.motorLeft.motorControl.setParameters(kD = float(cmd_splited[4]))
                    self.motorRight.motorControl.setParameters(kD = float(cmd_splited[4]))
            s = f"S,{self.batVoltageLP},{self.chgVoltage},{self.chgCurrentLP},{int(self.lift)},{int(self.bumper)},{int(self.raining)},{int(self.motorLeft.overload or self.motorRight.overload or self.motorMow.overload)},{self.motorMow.electricalCurrent},{self.motorLeft.electricalCurrent},{self.motorRight.electricalCurrent},{self.batteryTemp}"
            self.cmdAnswer(s)
        except Exception as e:
            print(f"Command summary invalid. Exception: {e}")

    # perform shutdown
    def cmdShutdown(self) -> None:
        if DEBUG:
            print("Shutdown request")
        self.motorLeft.stop()
        self.motorRight.stop()
        self.motorMow.stop()
        self.requestShutdown = True
        self.lcdPrioMessage = True
        self.lcdPrioMessageTime = time.ticks_add(time.ticks_ms(), 30000)
        self.lcdPrio1 = "shutdown"
        self.lcdPrio2 = ""
        s = f"Y3"
        self.cmdAnswer(s)
    
    # perform hang test (watchdog should trigger)
    def cmdTriggerWatchdog(self) -> None:
        if DEBUG:
            print("Watchdog trigger request")
        s = f"Y"
        self.cmdAnswer(s)
        while True: # never returns
            pass
    
    def sunrayStateToText(self, sunrayState: int) -> str:
        if sunrayState == 0:
            return "idle"
        elif sunrayState == 1:
            return "mow"
        elif sunrayState == 2:
            return "charge"
        elif sunrayState == 3:
            return "error"
        elif sunrayState == 4:
            return "dock"
        elif sunrayState == 5:
            return "escape forward"
        elif sunrayState == 6:
            return "escape reverse"
        elif sunrayState == 7:
            return "gps revovery"
        elif sunrayState == 8:
            return "gps wait fix"
        elif sunrayState == 9:
            return "gps wait float"
        elif sunrayState == 10:
            return "imu calibration"
        elif sunrayState == 11:
            return "kidnap wait"
        else:
            return "unknown"
    
    def readSensorHighFrequency(self) -> None:
        try:
            if not HIL:
                self.chgCurrentLP = self.inabat.current
                self.chgCurrentLP = self.chgCurrentLP * CURRENTFACTOR
                if self.chgCurrentLP <= 0.1:
                    self.chgCurrentLP = abs(self.chgCurrentLP)
                    self.chargerConnected = True
                    self.chgVoltage = self.batVoltageLP
                else:
                    self.chargerConnected = False
                    self.chgVoltage = 0
            else:
                self.chgCurrentLP = 1.0
                self.chargerConnected = False
                self.chgVoltage = 0
        except Exception as e:
            print(f"Error while reading INA(Battery) data: {e}")
            self.chargerConnected = False
            self.chgCurrentLP = 0
            self.chgVoltage = 0
    
    def keepPowerOn(self) -> None:
        if (self.batVoltageLP < CRITICALVOLTAGE and self.batVoltage < CRITICALVOLTAGE) or self.requestShutdown:
            self.switchBatteryDebounceCtr +=1
        else:
            self.switchBatteryDebounceCtr = 0

        if self.switchBatteryDebounceCtr == 10 or self.switchBatteryDebounceCtr == 20 or self.switchBatteryDebounceCtr == 30:
            self.lcdPrioMessage = True
            self.lcdPrioMessageTime = time.ticks_add(time.ticks_ms(), 30000)
            self.lcdPrio1 = "shutdown" + "." * (int(self.switchBatteryDebounceCtr/10))
            if self.requestShutdown:
                print("Main unit requests shutdown. Delay time started")
            else:
                print("Battery voltage critical level reached")
                print("SHUTTING DOWN")
                self.pinBatterySwitch.value(0)
        if self.switchBatteryDebounceCtr > 40:
            print(f"SHUTTING DOWN")
            self.pinBatterySwitch.value(0)
    
    def printInfo(self) -> None:
        print((f"tim={time.ticks_add(time.ticks_ms(), 0)}, "
              f"lps={self.lps}, "
              f"bat={self.batVoltageLP}V, "
              f"chg={self.chgVoltage}V/{self.chgCurrentLP}A, "
              f"mF={self.motorMowFault}, "
              f"imp={self.motorLeft.odomTicks},{self.motorRight.odomTicks},{self.motorMow.odomTicks}, "
              f"curr={self.motorLeft.electricalCurrent}, {self.motorRight.electricalCurrent}, {self.motorMow.electricalCurrent}, "
              f"lift={self.liftLeft},{self.liftRight}, "
              f"bump={self.bumperX},{self.bumperY}, "
              f"rain={self.rainLP, self.raining}, "
              f"stop={self.stopButton}, "
              f"rightSp={self.motorRight.currentSpeedSetPoint}, "
              f"right={self.motorRight.currentSpeedLp}, "
              f"speedLeft={self.motorLeft.currentSpeedLp}, "
              f"left={self.motorLeft.currentSpeedLp}, "
              f"speedMow={self.motorMow.currentRpmSetPoint}, "
              f"mow={self.motorMow.currentRpmLp}"
              ))

    def readSensors(self) -> None:
        # battery voltage
        w = 0.99
        try:
            if not HIL:
                self.batVoltage = self.inabat.bus_voltage + self.inabat.shunt_voltage 
            else:
                self.batVoltage = 28.0
        except Exception as e:
            print(f"Error while reading INA(Battery) data: {e}")
            self.batVoltage = 0
        self.batVoltageLP = w * self.batVoltageLP + (1 - w) * self.batVoltage

        # rain 
        w = 0.99
        self.rain = self.pinRain.read_u16()
        self.ainLP = w * self.rainLP + (1 - w) * self.rain
        self.raining = int((((self.rainLP * 100) / 65535) < 50))

        # lift
        self.liftLeft = self.pinLift1.value()
        self.liftRight = 0
        self.lift = int(self.liftLeft or self.liftRight)

        # bumper
        self.bumperX = self.pinBumperX.value()
        self.bumperY = self.pinBumperY.value()
        self.bumper = int(self.bumperX or self.bumperY)

        # stop button
        if self.pinStopButton.value() == 1:
            self.stopButton = 1
        else:
            self.stopButton = 0

        # dummys
        self.batteryTemp = 20 
        self.ovCheck = False 
        self.motorMowFault = False
    
    def readMotorCurrent(self) -> None:
        try:
            if not HIL:
                self.motorLeft.electricalCurrent = abs(self.inaleft.current) * CURRENTFACTOR
                self.motorRight.electricalCurrent = abs(self.inaright.current) * CURRENTFACTOR
                self.motorMow.electricalCurrent = abs(self.inamow.current) * CURRENTFACTOR
            else:
                self.motorLeft.electricalCurrent = 0.0
                self.motorRight.electricalCurrent = 0.0
                self.motorMow.electricalCurrent = 0.0
        except Exception as e:
            self.motorLeft.electricalCurrent = 99
            self.motorRight.electricalCurrent = 99
            self.motorMow.electricalCurrent = 99
            print(f"Error while reading INA(Motors) data: {e}")
        
    def mainLoop(self) -> None:
        while True:

            self.motorLeft.run()
            self.motorRight.run()
            self.motorMow.run()
            
            # sensor measure time (50Hz)
            if time.ticks_diff(self.nextMotorControlTime, time.ticks_ms()) <= 0:
                self.nextMotorControlTime = time.ticks_add(time.ticks_ms(), 20)
                self.readSensorHighFrequency()
            
            # motor timeout
            if time.ticks_diff(self.motorTimeout, time.ticks_ms()) <= 0: 
                self.led.value(1)
                self.motorLeft.setSpeed(0)
                self.motorRight.setSpeed(0)
                self.motorMow.setSpeed(0)
            else:
                self.led.value(0)
            
            # keep power on time (1Hz)
            if time.ticks_diff(self.nextKeepPowerOnTime, time.ticks_ms()) <= 0:
                self.nextKeepPowerOnTime = time.ticks_add(time.ticks_ms(), 1000)
                self.keepPowerOn()
            
            # next info time (DEBUGTIME)
            if time.ticks_diff(self.nextInfoTime, time.ticks_ms()) <= 0:
                self.nextInfoTime = time.ticks_add(time.ticks_ms(), INFOTIME)    
                if INFO:
                    self.printInfo()
                self.lps = 0
            
            # next measure time for sensors
            if time.ticks_diff(self.nextBatTime, time.ticks_ms()) <= 0:
                self.nextBatTime = time.ticks_add(time.ticks_ms(), 100)
                self.readSensors()
            
            # next measure time for motor current
            if time.ticks_diff(self.nextMotorSenseTime, time.ticks_ms()) <= 0:
                self.nextMotorSenseTime = time.ticks_add(time.ticks_ms(), 100)
                self.readMotorCurrent()
            
            self.processConsole()
            self.lps += 1
            self.wdt.feed()

    def printLcd(self) -> None:
        if self.lcdPrioMessage:
            self.lcdPrioMessage = False
            self.lcd.clear()
            self.lcd.move_to(0, 0)
            self.lcd.putstr(self.lcdPrio1)
            self.lcd.move_to(0, 1)
            self.lcd.putstr(self.lcdPrio2)
            return
        if time.ticks_diff(self.lcdPrioMessageTime, time.ticks_ms()) <= 0:
            lcdMessage1 = self.lcd1
            lcdMessage2 = self.lcd2
            if len(self.lcd1) < LCD_NUM_COLUMNS:
                lcdMessage1 = ('{: <{}}'.format(self.lcd1, LCD_NUM_COLUMNS))
            if len(self.lcd2) < LCD_NUM_COLUMNS:
                lcdMessage2 = ('{: <{}}'.format(self.lcd2, LCD_NUM_COLUMNS))
            self.lcd.move_to(0, 0)
            self.lcd.putstr(lcdMessage1)
            self.lcd.move_to(0, 1)
            self.lcd.putstr(lcdMessage2)

    def secondLoop(self) -> None:
        while True:
            self.printLcd();
            sleep_ms(1000)

if __name__ == '__main__':
    landrumowerDriver = PicoMowerDriver()
    if LCD:
        _thread.start_new_thread(landrumowerDriver.secondLoop, ())
    landrumowerDriver.mainLoop()
    