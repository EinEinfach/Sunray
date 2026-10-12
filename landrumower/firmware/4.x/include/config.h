#ifndef CONFIG_H
#define CONFIG_H

#define FW_NAME "Landrumower RPI Pico"
#define FW_VERSION "4.0.0"

// ---------------------------------------------------------------- motor control

// true:  the Pico controls the wheel speed, Sunray sends mm/s in AT+M
// false: Sunray controls, the Pico passes the PWM values of AT+M through
#define PICO_MOTOR_CONTROL true

#define CONTROL_PERIOD_US 5000      // control loop on core 0
#define MOTOR_COMMAND_TIMEOUT_S 3.0f // motors stop without AT+M for this long

// speed controller of the wheel motors, output is duty (0..1) per mm/s.
// AT+S can replace these values at runtime.
#define MOTOR_KP 0.002f
#define MOTOR_KI 0.02f
#define MOTOR_KD 0.0f

#define TICKS_PER_REVOLUTION 340
#define WHEEL_DIAMETER_MM 220.0f
#define STANDSTILL_SPEED_MM_S 20.0f   // below this a wheel counts as stopped
#define STANDSTILL_TIMEOUT_US 200000  // no impulse for this long means speed zero
#define MOW_STANDSTILL_TICKS_S 5.0f
#define ENCODER_MIN_EDGE_US 1000      // shorter impulse distances are spikes

// Braking. PWM on the brake input of the JYQD drivers is an experiment: the usable duty
// range has to be measured on the real driver. With BRAKE_PWM false the brake is only
// switched on and off.
#define BRAKE_PWM true
#define BRAKE_MIN_DUTY 0.0f
#define BRAKE_MAX_DUTY 1.0f
#define STOP_BRAKE_DELAY_S 0.05f // coast this long before braking on stop
#define STOP_BRAKE_RAMP_S 0.5f   // then ramp the brake to full

#define MOTOR_PWM_HZ 20000 // JYQD2021 works best with 20 kHz
#define BRAKE_PWM_HZ 20000

// ---------------------------------------------------------------- console

// state of the control loop on USB as "D,..." lines, 0 to switch off
#define TELEMETRY_PERIOD_MS 100
// mower state and detected hardware on USB as "I,..." lines, 0 to switch off
#define INFO_PERIOD_MS 1000

// ---------------------------------------------------------------- LCD and buzzer

#define LCD_ENABLED true
#define LCD_ADDRESS 0x27
#define LCD_RETRY_MS 5000 // look for a missing display this often

// The buzzer is switched on and off, without PWM: GP26 shares its PWM slice with the
// mow motor PWM on GP11, so a buzzer frequency would change the motor PWM frequency.
#define BUZZER_ENABLED true

// ---------------------------------------------------------------- pins and interfaces

#define UART Serial1 // link to Sunray on the Raspberry Pi, CRC required
#define USB Serial   // console, CRC optional
#define UART_BAUDRATE 115200
#define USB_BAUDRATE 115200
#define UART0_TX 0
#define UART0_RX 1

#define I2C1_SDA 14 // LCD
#define I2C1_SCL 15
#define I2C1_CLOCK_HZ 100000
#define PIN_BUZZER 26

#define RIGHT_IMP 2
#define RIGHT_PWM 3
#define RIGHT_DIR 4
#define RIGHT_BRAKE 5
#define RIGHT_DIRECTION_HIGH_IS_FORWARD true
#define RIGHT_BRAKE_HIGH_ACTIVE true

#define LEFT_IMP 6
#define LEFT_PWM 7
#define LEFT_DIR 8
#define LEFT_BRAKE 9
#define LEFT_DIRECTION_HIGH_IS_FORWARD false
#define LEFT_BRAKE_HIGH_ACTIVE true

#define MOW_IMP 10
#define MOW_PWM 11
#define MOW_DIR 12
#define MOW_BRAKE 13
#define MOW_DIRECTION_HIGH_IS_FORWARD false
#define MOW_BRAKE_HIGH_ACTIVE true

// ---------------------------------------------------------------- simulation (pico_sim)

#define SIM_WHEEL_MAX_TICKS_S 350.0f // about 0.7 m/s at full drive
#define SIM_MOW_MAX_TICKS_S 500.0f

#endif // CONFIG_H
