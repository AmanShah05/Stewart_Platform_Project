from ball_tracking import detect_yellow_ball
import time
import struct
from smbus2 import SMBus

addr = 0x8  # Arduino slave address
bus = SMBus(1)  # /dev/i2c-1

def send_position(x, y):
    try:
        # Pack x and y as 4-byte floats in big-endian order
        x_bytes = struct.pack('>f', x)
        y_bytes = struct.pack('>f', y)
        
        # Send each byte of x
        for byte in x_bytes:
            bus.write_byte(addr, byte)

        # Send each byte of y
        for byte in y_bytes:
            bus.write_byte(addr, byte)

    except Exception as e:
        print(f"Error sending data: {e}")

# Detect the yellow ball to get x and y positions
x, y = detect_yellow_ball()
send_position(x, y)
