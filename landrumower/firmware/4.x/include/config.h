#ifndef CONFIG_H
#define CONFIG_H

#define FW_NAME "Landrumower RPI Pico"
#define FW_VERSION "4.0.0"

// serial interfaces
#define UART Serial1 // link to Sunray on the Raspberry Pi, CRC required
#define USB Serial   // console, CRC optional
#define UART_BAUDRATE 115200
#define USB_BAUDRATE 115200
#define UART0_TX 0
#define UART0_RX 1

#endif // CONFIG_H
