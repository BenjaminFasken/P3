
import serial
import time

if __name__ == '__main__':

    serial_bot = serial.Serial('COM6', 115200, timeout=1)
    serial.rs485_mode = True

    serial_bot.flush()

    serial_bot.write([0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x08, 0x2F, 0x4E])
    serial_bot.read(9)
    serial_bot.flush()
    t = time.time()
    serial_bot.write([0xFF, 0xFF, 0xFD, 0x00, 0x02, 0x03, 0x00, 0x08, 0x2F, 0x72])
    serial_bot.read(9)
    serial_bot.flush()
    serial_bot.write([0xFF, 0xFF, 0xFD, 0x00, 0x03, 0x03, 0x00, 0x08, 0x2C, 0xE6])
    serial_bot.read(9)
    serial_bot.flush()
    serial_bot.write([0xFF, 0xFF, 0xFD, 0x00, 0x04, 0x03, 0x00, 0x08, 0x2F, 0x0A])
    serial_bot.read(9)
    serial_bot.flush()
    serial_bot.write([0xFF, 0xFF, 0xFD, 0x00, 0x05, 0x03, 0x00, 0x08, 0x2C, 0x9E])
    serial_bot.read(9)
    serial_bot.flush()
    serial_bot.write([0xFF, 0xFF, 0xFD, 0x00, 0x05, 0x03, 0x00, 0x08, 0x2C, 0x9E])
    serial_bot.read(9)
    serial_bot.flush()
    serial_bot.write([0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x06, 0x00, 0x03, 0x40, 0x00, 0x01, 0xDB, 0x66])
    serial_bot.read(9)
    serial_bot.flush()
    serial_bot.write([0xFF, 0xFF, 0xFD, 0x00, 0x02, 0x06, 0x00, 0x03, 0x40, 0x00, 0x01, 0xEB, 0x65])
    serial_bot.read(9)
    serial_bot.flush()
    serial_bot.write([0xFF, 0xFF, 0xFD, 0x00, 0x03, 0x06, 0x00, 0x03, 0x40, 0x00, 0x01, 0xF8, 0xE4])
    serial_bot.read(9)
    serial_bot.flush()
    serial_bot.write([0xFF, 0xFF, 0xFD, 0x00, 0x04, 0x06, 0x00, 0x03, 0x40, 0x00, 0x01, 0x8B, 0x63])
    serial_bot.read(9)
    serial_bot.flush()
    serial_bot.write([0xFF, 0xFF, 0xFD, 0x00, 0x05, 0x06, 0x00, 0x03, 0x40, 0x00, 0x01, 0x98, 0xE2])
    serial_bot.read(9)
    serial_bot.flush()
    serial_bot.write([0xFF, 0xFF, 0xFD, 0x00, 0x06, 0x06, 0x00, 0x03, 0x40, 0x00, 0x01, 0xA8, 0xE1])
    serial_bot.read(9)
    serial_bot.flush()
    print("Time: "+str(time.time()-t)+" \nReset done. Thank you.")