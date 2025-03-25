import serial

# Update port according to your OS:
# On Windows: COM46
# On Linux/Mac: /dev/ttyS46 or /dev/ttyUSB0 (check with `dmesg | grep tty`)

port = "COM42"  # Change if you're on Linux/Mac
baud_rate = 115200

try:
    ser = serial.Serial(port, baud_rate, timeout=1)
    print(f"Listening on {port} at {baud_rate} baud...")
    count = 0
    angle = 0 
    previous = 0
    while True:
        if ser.in_waiting > 0:
            try:
                byte = ser.read()
                # print(byte)
                byte_int = int.from_bytes(byte, byteorder='big')
                if byte_int == 0xA5 and previous == 0x5A:
                    count = 2
                else:
                    count += 1
                    if count == 13:
                        angle = byte_int*256 + previous
                        if angle > 0x7FFF:  # if greater than 32767, it's negative in signed form
                            angle -= 0x10000  # subtract 65536
                        
                        print(angle/10)
                previous = byte_int


            except Exception as e:
                print(byte, e)
except KeyboardInterrupt:
    print("Stopped by user.")
except Exception as e:
    print(f"Error: {e}")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
