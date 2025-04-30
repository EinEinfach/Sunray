# Configuration:
WATCHDOG = False    # set to true in productive enviroment
DEBUG = False       # activate debug information over usb
INFO = True         # activate info over usb
INFOTIME = 10000    # information time period
HIL = False         # use pico for testing as hardware in the loop (ignore missing i2c devices)
PICOMOTORCONTROL = True     # activate pid control on pico
KP = 8.0            # Kp factor for pico pid controller
KI = 0.1           # KI factor for pico pid controller
KD = 0.005            # KD factor for pico pid controller
OVERLOADCURRENT_GEAR = 10   # overload current for gear motors
OVERLOADCURRENT_MOW = 10    # overload current for mow motor
CRITICALVOLTAGE = 17        # critical voltage pico will cut off power supply
FREQ = 20000            # pwm frequency for motor drivers (JYQD2021 best results with 20kHz)
FREQ_MOW = 10000        # pwm frequency for mow motor driver (No data for mow driver available)
CURRENTFACTOR = 10      # If you changed your ina shunt from 100mOhm to 10mOhm set this value to 10, without any modifications set it to 1
INABATADRESS = 64       # I2C adress for INA226 battery (A0 and A1 on GND)
INAMOWADRESS = 65       # I2C adress for INA226 mow motor (A0 on VCC)
INALEFTADRESS = 68      # I2C adress for left gear motor
INARIGHTADRESS = 69     # I2C adress for right gear motor
LCD = True              # If you don't have a LCD set this to False
LCDADRESS = 39          # I2C adress for LCD
LCD_NUM_ROWS = 2        # number of LCD rows
LCD_NUM_COLUMNS = 16    # number of LCD columns
FREQ_BUZZER = 1000      # pwm frequency buzzer

# packeage imports
from machine import Pin
from machine import PWM
from machine import ADC
from machine import I2C
from machine import UART
from machine import WDT

import time, sys, select, _thread

# local imports
from lib.ina226 import INA226
from lib.pico_i2c_lcd import I2cLcd
from lib.motor import Motor
from lib.buzzer import Buzzer

VERNR = "2.4.0"
VER = f"Landrumower RPI Pico {VERNR}" # Reinit i2c 

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

    pinBuzzer = PWM(Pin(26))

    # lcd messages
    lcd1: str = "" 
    lcd2: str = ""
    lcd1Printed: str = ""
    lcd2Printed: str = ""
    lcdPrio1: str = ""
    lcdPrio2: str = ""
    lcdPrioMessage: bool = False
    lcdPrioMessageTime: int = 0
    nextLcdTime: int = 0

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
    chgCurrentLp: float = 0.0
    batVoltage: float = 0.0
    batVoltageLp: float = 24.0
    batteryTemp: float = 0.0
    ovCheck: bool = False
    switchBatteryDebounceCtr: int = 0
    nextBatTime: int = 0

    # sensors
    rain: int = 65535
    rainLp: int = 65535
    raining: int = 0
    liftLeft: int = 0
    liftRight: int = 0
    lift: int = 0
    bumperX: int = 0
    bumperY: int = 0
    bumper: int = 0
    stopButton: int = 0

    # common
    mainUnitState: str = "boot"
    stopLoop: bool = False
    debugMessages = []
    requestShutdown: bool = False
    nextInfoTime: int = time.ticks_ms()
    nextKeepPowerOnTime: int = time.ticks_ms()
    lps: int = 0
    msgs: int = 0

    # interfaces
    i2cAvailable: bool = False

    def __init__(self) -> None:
        if WATCHDOG: self.wdt = WDT(timeout=6000)

        self.pinBatterySwitch.value(1)

        print("Landrumower Driver")
        print(f"Version: {VER}")
        self.uart0 = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1), bits=8, parity=None, stop=1, timeout=2)
        self.i2c0 = I2C(0, sda=Pin(16), scl=Pin(17), freq=400000)  # I2C0 has 3.3V logic
        self.i2c1 = I2C(1, sda=Pin(14), scl=Pin(15), freq=400000)  # I2C1 has 5.0V logic

        self.pinMotorRightPwm.freq(FREQ)
        self.pinMotorRightBrake.freq(FREQ)
        self.pinMotorLeftPwm.freq(FREQ)
        self.pinMotorLeftBrake.freq(FREQ)
        self.pinMotorMowPwm.freq(FREQ_MOW)
        self.pinMotorRightBrake.freq(FREQ_MOW)

        if HIL:
            print("HIL mode activated")
        else:
            self.initI2C()
        if LCD:
            try:
                self.lcd = I2cLcd(self.i2c1, LCDADRESS, LCD_NUM_ROWS, LCD_NUM_COLUMNS)
                self.lcd.backlight_on()
                self.lcd.clear()
                self.lcd1 = "Landrumower pico"
                self.lcd2 = f"Version: {VERNR}"
            except Exception as e:
                print(f"ERROR: LCD not found: {e}")   
        try:
            self.motorLeft = Motor('gear',
                                PICOMOTORCONTROL,
                                KP,
                                KI,
                                KD,
                                self.pinMotorLeftDir, 
                                self.pinMotorLeftBrake, 
                                self.pinMotorLeftPwm, 
                                self.pinMotorLeftImp, 
                                brakePinHighActive=True, 
                                directionPinHighPositive=True, 
                                overloadThreshold=OVERLOADCURRENT_GEAR) 
            self.motorRight = Motor('gear',
                                    PICOMOTORCONTROL,
                                    KP,
                                    KI,
                                    KD,
                                    self.pinMotorRightDir, 
                                    self.pinMotorRightBrake, 
                                    self.pinMotorRightPwm, 
                                    self.pinMotorRightImp, 
                                    brakePinHighActive=True, 
                                    directionPinHighPositive=False, 
                                    overloadThreshold=OVERLOADCURRENT_GEAR)
            self.motorMow = Motor('mow', 
                                PICOMOTORCONTROL,
                                KP,
                                KI,
                                KD,
                                self.pinMotorMowDir, 
                                self.pinMotorMowBrake, 
                                self.pinMotorMowPwm, 
                                self.pinMotorMowImp, 
                                brakePinHighActive=False, 
                                directionPinHighPositive=True, 
                                overloadThreshold=OVERLOADCURRENT_MOW)
        except Exception as e:
            print(f"Could not initilaze motors: {e}")
        
        self.buzzer = Buzzer(FREQ_BUZZER, self.pinBuzzer)

        if WATCHDOG: self.wdt.feed()
    
    def initI2C(self) -> None:
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

    # uart input
    def processConsole(self) -> None:
        try:
            #read input from uart (normal operation)
            if self.uart0.any() > 0:
                rawData = self.uart0.readline()
                self.cmd = rawData.decode("ascii")
                if DEBUG:
                    self.debugMessages.append(f"Received command: {self.cmd}")
                self.processCmd(True)
                if DEBUG:
                    self.debugMessages.append(f"Response: {self.cmdResponse}")
                self.uart0.write(self.cmdResponse)
                self.cmd = ""
            #read input from usb, user cmd operation (hardware in the loop, no crc check)
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                rawData = sys.stdin.readline().strip()
                if rawData != "":
                    self.cmd = rawData
                    print(f"Received command via USB: {self.cmd}")
                    self.processCmd(False)
                    print(self.cmdResponse)  # Send response back to USB console
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
                        self.debugMessages.append("CRC ERROR")
                    return
            else:
                cmd_splited = self.cmd.split(",")
                crc = cmd_splited[-1]
                cmd_no_crc = self.cmd.replace(','+crc, '')
                crc = hex(int(crc, 16))
                expectedCRC = hex(sum(cmd_no_crc.encode('ascii')) % 256)
                if expectedCRC != crc and checkCrc:
                    if DEBUG:
                        self.debugMessages.append(f"CRC ERROR. Received: {crc}, expected: {expectedCRC}")
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
                self.debugMessages.append(f"Answer: {s}")
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
        while loopCall <= 7:
            if loopCall%2 == 0:
                testSpeed = 0
            else:
                testSpeed = 100 * loopCall
                testSpeed = min(testSpeed, 600)
            
            self.motorLeft.setSpeed(testSpeed)
            self.motorRight.setSpeed(testSpeed)
            if testSpeed == 0:
                testTime = time.ticks_add(time.ticks_ms(), 5000)
            else:
                testTime = time.ticks_add(time.ticks_ms(), 20000)
            nextInfoTime = time.ticks_ms()
            while time.ticks_diff(testTime, time.ticks_ms()) >= 0:
                if time.ticks_diff(nextInfoTime, time.ticks_ms()) <= 0:
                    nextInfoTime = time.ticks_add(time.ticks_ms(), 100)
                    print(f"left pwm: {int(100*self.motorLeft.currentTrqPwm/65535)} left sp: {testSpeed} left: {self.motorLeft.currentSpeed} left rpm sp: {self.motorLeft.currentRpmSetPoint} left rpm: {self.motorLeft.currentRpm} right pwm: {int(100*self.motorRight.currentTrqPwm/65535)} right sp: {testSpeed} right: {self.motorRight.currentSpeed} right rpm sp: {self.motorRight.currentRpmSetPoint} right rpm: {self.motorRight.currentRpm}")
                self.motorLeft.run()
                self.motorRight.run()
                time.sleep(0.003)
                if WATCHDOG: self.wdt.feed()
            loopCall += 1

    # request motor
    # AT+M,20,20,1
    def cmdMotor(self) -> None:
        try:
            cmd_splited = self.cmd.split(",")
            rightPwm = 0
            leftPwm = 0
            mowPwm = 0
            rightSpeed = 0.0
            leftSpeed = 0.0
            if len(cmd_splited) == 4:
                rightPwm = int(cmd_splited[1])
                leftPwm = int(cmd_splited[2])
                mowPwm = int(cmd_splited[3])
            elif len(cmd_splited) == 6:
                rightPwm = int(cmd_splited[1])
                leftPwm = int(cmd_splited[2])
                mowPwm = int(cmd_splited[3])
                rightSpeed = int(cmd_splited[4])
                leftSpeed = int(cmd_splited[5])
            else:
                return
            if DEBUG:
                self.debugMessages.append(f"leftPwm={leftPwm}, rightPwm={rightPwm}, mow={mowPwm}, leftSpeed={leftSpeed}mm/s, rightSpeed={rightSpeed}mm/s")
            if PICOMOTORCONTROL:
                self.motorLeft.setSpeed(int(leftSpeed))
                self.motorRight.setSpeed(int(rightSpeed))
                self.motorMow.setSpeed(mowPwm)
            else:
                self.motorLeft.setSpeed(leftPwm)
                self.motorRight.setSpeed(rightPwm)
                self.motorMow.setSpeed(mowPwm)
            self.motorTimeout = time.ticks_add(time.ticks_ms(), 3000)
            s = f"M,{self.motorRight.odomTicks},{self.motorLeft.odomTicks},{self.motorMow.odomTicks},{self.chgVoltage},{int(self.bumper)},{int(self.lift)},{int(self.stopButton)}"
            #s = f"t1:{self.time1} t2:{self.time2} t3:{self.time3} t4:{self.time4} t5:{self.time5} t6:{self.time6} t7:{self.time7}"
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
        print("Motor faults request. Reseting drivers")
        self.motorLeft.driverReset()
        self.motorRight.driverReset()
        s = f"R"
        self.cmdAnswer(s)

    # request summary
    def cmdSummary(self) -> None:
        try:
            self.lcd1 = f""
            self.lcd2  = f"{round(self.batVoltageLp, 1)}V/{round(self.chgCurrentLp, 1)}A"
            cmd_splited = self.cmd.split(",")
            for cmdDataIdx in range(len(cmd_splited)):
                if cmdDataIdx == 1:
                    self.lcd1 = self.sunrayStateToText(int(cmd_splited[1]))
                    if DEBUG:
                        self.debugMessages.append(f"Sunray state: {self.lcd1}")
                elif cmdDataIdx == 2:
                    if DEBUG:
                        self.debugMessages.append(f"Received new kP parameter: {cmd_splited[2]}")
                    self.motorLeft.motorControl.setParameters(kP = float(cmd_splited[2]))
                    self.motorRight.motorControl.setParameters(kP = float(cmd_splited[2]))
                elif cmdDataIdx == 3:
                    if DEBUG:
                        self.debugMessages.append(f"Received new kI parameter: {cmd_splited[3]}")
                    self.motorLeft.motorControl.setParameters(kI = float(cmd_splited[3]))
                    self.motorRight.motorControl.setParameters(kI = float(cmd_splited[3]))
                elif cmdDataIdx == 4:
                    if DEBUG:
                        self.debugMessages.append(f"Received new kD parameter: {cmd_splited[4]}")
                    self.motorLeft.motorControl.setParameters(kD = float(cmd_splited[4]))
                    self.motorRight.motorControl.setParameters(kD = float(cmd_splited[4]))
            s = f"S,{self.batVoltageLp},{self.chgVoltage},{self.chgCurrentLp},{int(self.lift)},{int(self.bumper)},{int(self.raining)},{int(self.motorLeft.overload or self.motorRight.overload or self.motorMow.overload)},{self.motorMow.electricalCurrent},{self.motorLeft.electricalCurrent},{self.motorRight.electricalCurrent},{self.batteryTemp}"
            self.cmdAnswer(s)
        except Exception as e:
            print(f"Command summary invalid. Exception: {e}")

    # perform shutdown
    def cmdShutdown(self) -> None:
        print("Shutdown request")
        self.motorLeft.stop()
        self.motorRight.stop()
        self.motorMow.stop()
        self.buzzer.mainUnitState = "shutdown"
        self.buzzer.playShutdown(60000)
        self.requestShutdown = True
        self.lcdPrioMessage = True
        self.lcdPrioMessageTime = time.ticks_add(time.ticks_ms(), 30000)
        self.lcdPrio1 = "shutdown"
        self.lcdPrio2 = ""
        s = f"Y3"
        self.cmdAnswer(s)
    
    # perform hang test (watchdog should trigger)
    def cmdTriggerWatchdog(self) -> None:
        print("Watchdog trigger request")
        s = f"Y"
        self.cmdAnswer(s)
        while True: # never returns
            pass
    
    def sunrayStateToText(self, sunrayState: int) -> str:
        if sunrayState == 0:
            self.mainUnitState = "idle"
            return "idle"
        elif sunrayState == 1:
            self.mainUnitState = "mow"
            return "mow"
        elif sunrayState == 2:
            self.mainUnitState = "charge"
            return "charge"
        elif sunrayState == 3:
            self.mainUnitState = "error"
            return "error"
        elif sunrayState == 4:
            self.mainUnitState = "dock"
            return "dock"
        elif sunrayState == 5:
            self.mainUnitState = "escape forward"
            return "escape forward"
        elif sunrayState == 6:
            self.mainUnitState = "escape reverse"
            return "escape reverse"
        elif sunrayState == 7:
            self.mainUnitState = "gps recovery"
            return "gps recovery"
        elif sunrayState == 8:
            self.mainUnitState = "gps wait fix"
            return "gps wait fix"
        elif sunrayState == 9:
            self.mainUnitState = "gps wait float"
            return "gps wait float"
        elif sunrayState == 10:
            self.mainUnitState = "imu calibration"
            return "imu calibration"
        elif sunrayState == 11:
            self.mainUnitState = "kidnap wait"
            return "kidnap wait"
        else:
            return "unknown"
    
    def readSensorHighFrequency(self) -> None:
        try:
            if not HIL:
                self.chgCurrentLp = self.inabat.current
                self.chgCurrentLp = self.chgCurrentLp * CURRENTFACTOR
                if self.chgCurrentLp <= 0.1:
                    self.chgCurrentLp = abs(self.chgCurrentLp)
                    self.chargerConnected = True
                    self.chgVoltage = self.batVoltageLp
                else:
                    self.chargerConnected = False
                    self.chgVoltage = 0
            else:
                self.chgCurrentLp = 1.0
                self.chargerConnected = False
                self.chgVoltage = 0
        except Exception as e:
            print(f"Error while reading INA(Battery) data: {e}")
            self.chargerConnected = False
            self.chgCurrentLp = 0
            self.chgVoltage = 0
    
    def keepPowerOn(self) -> None:
        if (self.batVoltageLp < CRITICALVOLTAGE and self.batVoltage < CRITICALVOLTAGE) or self.requestShutdown:
            self.switchBatteryDebounceCtr +=1
            if self.switchBatteryDebounceCtr % 5 == 0:
                self.lcdPrioMessage = True
                self.lcdPrioMessageTime = time.ticks_add(time.ticks_ms(), 30000)
                self.lcdPrio1 = "shutdown" + "." * int(self.switchBatteryDebounceCtr/5)
                if self.requestShutdown:
                    print("Main unit requests shutdown. Delay time started")
                else:
                    print("Battery voltage critical level reached")
                    print("SHUTTING DOWN")  
        else:
            self.switchBatteryDebounceCtr = 0
        
        if self.switchBatteryDebounceCtr > 40:
            print(f"SHUTTING DOWN")
            self.pinBatterySwitch.value(0)
    
    def printInfo(self) -> None:
        print((f"tim={time.ticks_add(time.ticks_ms(), 0)}, "
              f"lps={self.lps}/{int(INFOTIME/1000)}s, "
              f"msgBuf={len(self.debugMessages)}, "
              f"bat={self.batVoltageLp}V, "
              f"chg={self.chgVoltage}V/{self.chgCurrentLp}A, "
              f"mF={self.motorMowFault}, "
              f"imp={self.motorLeft.odomTicks},{self.motorRight.odomTicks},{self.motorMow.odomTicks}, "
              f"curr={self.motorLeft.electricalCurrent}, {self.motorRight.electricalCurrent}, {self.motorMow.electricalCurrent}, "
              f"lift={self.liftLeft},{self.liftRight}, "
              f"bump={self.bumperX},{self.bumperY}, "
              f"rain={self.rainLp, self.raining}, "
              f"stop={self.stopButton}, "
              f"rightSp={self.motorRight.currentSpeedSetPoint}, "
              f"right={self.motorRight.currentSpeed}, "
              f"rightPwm={self.motorRight.currentPwm}, "
              f"leftSp={self.motorLeft.currentSpeedSetPoint}, "
              f"left={self.motorLeft.currentSpeed}, "
              f"leftPwm={self.motorLeft.currentPwm}, "
              f"speedMow={self.motorMow.currentRpmSetPoint}, "
              f"mow={self.motorMow.currentRpm}, "
              f"mowPwm={self.motorMow.currentPwm}"
              ))
        self.lps = 0
    
    def readSensors(self) -> None:
        # battery voltage
        w = 99
        try:
            if not HIL:
                batVoltage = int((self.inabat.bus_voltage + self.inabat.shunt_voltage) * 1000) #mV
            else:
                batVoltage = 28000 #mV
        except Exception as e:
            print(f"Error while reading INA(Battery) data: {e}")
            batVoltage = 0
        batVoltageLp = int((self.batVoltageLp) * 1000)
        batVoltageLp = (w * batVoltageLp + (100 - w) * batVoltage) // 100
        #scale to V
        self.batVoltage = round(batVoltage / 1000, 2)
        self.batVoltageLp = round(batVoltageLp / 1000, 2)

        # rain 
        self.rain = self.pinRain.read_u16()
        self.rainLp = int(w * self.rainLp + (100 - w) * self.rain) // 100
        self.raining = int((((self.rainLp * 100) / 65535) < 50))

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
            self.initI2C()
        
    def mainLoop(self) -> None:
        loopCnt = 0
        while not self.stopLoop:
            # run
            time1 = loopTime = time.ticks_ms()
            self.motorMow.run()
            self.motorRight.run()
            self.motorLeft.run()
            time1 = time.ticks_diff(time.ticks_ms(), time1)
            
            # motor timeout
            time2 = time.ticks_ms()
            if time.ticks_diff(self.motorTimeout, time.ticks_ms()) <= 0: 
                self.led.value(1)
                self.motorLeft.setSpeed(0)
                self.motorRight.setSpeed(0)
                self.motorMow.setSpeed(0)
            else:
                self.led.value(0)
            time2 = time.ticks_diff(time.ticks_ms(), time2)
            
            # keep power on time (1Hz)
            time3 = time.ticks_ms()
            if time.ticks_diff(self.nextKeepPowerOnTime, time.ticks_ms()) <= 0:
                self.nextKeepPowerOnTime = time.ticks_add(time.ticks_ms(), 1000)
                self.keepPowerOn()
            time3 = time.ticks_diff(time.ticks_ms(), time3)
            
            # next measure time for motor current
            time4 = time.ticks_ms()
            if time.ticks_diff(self.nextMotorSenseTime, time.ticks_ms()) <= 0:
                self.nextMotorSenseTime = time.ticks_add(time.ticks_ms(), 100)
                self.readMotorCurrent()
            time4 = time.ticks_diff(time.ticks_ms(), time4)
            
            time5 = time.ticks_ms()
            self.processConsole()
            time5 = time.ticks_diff(time.ticks_ms(), time5)
            self.lps += 1
            loopCnt += 1
            loopTimeDiff = time.ticks_diff(time.ticks_ms(), loopTime)
            if loopTimeDiff > 20 and INFO:
                print(f"mainLoopTime: {loopTimeDiff}ms motorControlTime:{time1}ms motorTimeoutTime:{time2}ms keepPowerOnTime:{time3}ms motorSenseTime:{time4}ms processConsoleTime:{time5}ms loopRatio: {1/loopCnt}")
                loopCnt = 0
            if WATCHDOG: self.wdt.feed()

    def printLcd(self) -> None:
        if self.lcdPrioMessage:
            self.lcdPrioMessage = False
            self.lcd.move_to(0, 0)
            self.lcd.putstr(self.prepareLcdMessage(self.lcdPrio1))
            self.lcd1Printed = self.lcdPrio1
            self.lcd.move_to(0, 1)
            self.lcd.putstr(self.prepareLcdMessage(self.lcdPrio2))
            self.lcd2Printed = self.lcdPrio2
            return
        if time.ticks_diff(self.lcdPrioMessageTime, time.ticks_ms()) <= 0 and (self.lcd1 != self.lcd1Printed or self.lcd2 != self.lcd2Printed):
            self.lcd.move_to(0, 0)
            self.lcd.putstr(self.prepareLcdMessage(self.lcd1))
            self.lcd1Printed = self.lcd1
            self.lcd.move_to(0, 1)
            self.lcd.putstr(self.prepareLcdMessage(self.lcd2))
            self.lcd2Printed = self.lcd2
    
    def prepareLcdMessage(self, message: str) -> str:
        if len(message) < LCD_NUM_COLUMNS:
            modifiedMessage = ('{: <{}}'.format(message, LCD_NUM_COLUMNS))
        else:
            modifiedMessage = message[:LCD_NUM_COLUMNS]
        return modifiedMessage

    def printDebug(self) -> None:
        message = None
        if self.debugMessages != []:
            message = self.debugMessages[0]
            self.debugMessages.pop(0)
        if DEBUG and message is not None:
            print(message)    

    def secondLoop(self) -> None:
        loopCnt = 0
        while not self.stopLoop:
            # sensor measure time (50Hz)
            time1 = loopTime = time.ticks_ms()
            if time.ticks_diff(self.nextMotorControlTime, time.ticks_ms()) <= 0:
                self.nextMotorControlTime = time.ticks_add(time.ticks_ms(), 20)
                self.readSensorHighFrequency()
            time1 = time.ticks_diff(time.ticks_ms(), time1)
            # next measure time for sensors
            time2 = time.ticks_ms()
            if time.ticks_diff(self.nextBatTime, time.ticks_ms()) <= 0:
                self.nextBatTime = time.ticks_add(time.ticks_ms(), 500)
                self.readSensors()
            time2 = time.ticks_diff(time.ticks_ms(), time2)
            # buzzer
            time3 = time.ticks_ms()
            self.buzzer.run(self.mainUnitState)
            time3 = time.ticks_diff(time.ticks_ms(), time3)
            # lcd
            time4 = time.ticks_ms()
            if LCD and time.ticks_diff(self.nextLcdTime, time.ticks_ms()) < 0:
                self.nextLcdTime = time.ticks_add(time.ticks_ms(), 1000)
                self.printLcd()
            time4 = time.ticks_diff(time.ticks_ms(), time4)
            # print debug messages
            time5 = time.ticks_ms()
            self.printDebug()
            time5 = time.ticks_diff(time.ticks_ms(), time5)
            # next info time (INFOTIME)
            time6 = time.ticks_ms()
            if time.ticks_diff(self.nextInfoTime, time.ticks_ms()) <= 0:
                self.nextInfoTime = time.ticks_add(time.ticks_ms(), INFOTIME)    
                if INFO:
                    self.printInfo()
            time6 = time.ticks_diff(time.ticks_ms(), time6)
            loopTimeDiff = time.ticks_diff(time.ticks_ms(), loopTime)
            loopCnt += 1
            if loopTimeDiff > 20 and INFO:
                print(f"secondLoopTime: {loopTimeDiff}ms readSensorHighFrequencyTime:{time1}ms readBatTime:{time2}ms buzzerRunTime:{time3}ms lcdTime:{time4}ms printDebugTime:{time5}ms printInfoTime:{time6} loopRatio: {1/loopCnt}")
                loopCnt = 0
            

if __name__ == '__main__':
    landrumowerDriver = PicoMowerDriver()
    _thread.start_new_thread(landrumowerDriver.secondLoop, ())
    landrumowerDriver.mainLoop()
    print("Code stopped")
    landrumowerDriver.led.value(0)
    sys.exit()
    
