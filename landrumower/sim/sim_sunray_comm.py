import serial
import time

# Öffne die serielle Verbindung
try:
    ser = serial.Serial('/dev/cu.usbmodem14201', baudrate=115200, timeout=1)  # Passe den Port ggf. an!
    try:
    # 10 Sekunden lang alle 20ms Befehl 1 senden
        start_time = time.time()
        print("---------speed change---------")
        while time.time() - start_time < 10:
            loopStart = time.time()
            ser.write(b'AT+M,0,0,0,200,200,0x13\r\n')
            if ser.in_waiting:
                response = ser.read(ser.in_waiting)
                print(response.decode(errors='ignore').strip())
            print(f"Loop duration: {(time.time() - loopStart)*1000}")
            time.sleep(0.02)  # 20ms warten

        # 5 Sekunden lang alle 20ms Befehl 2 senden
        start_time = time.time()
        print("---------speed change---------")
        while time.time() - start_time < 10:
            loopStart = time.time()
            ser.write(b'AT+M,0,0,0,0,0,0x13\r\n')
            if ser.in_waiting:
                response = ser.read(ser.in_waiting)
                print(response.decode(errors='ignore').strip())
            print(f"Loop duration: {(time.time() - loopStart)*1000}")
            time.sleep(0.02)  # 20ms warten

    finally:
        ser.close()
        print("✅ Verbindung geschlossen.")
        
except Exception as e:
    print(f"Could not open serial connection: {e}")



