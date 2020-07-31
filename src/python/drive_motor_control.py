import struct
from time import time

import serial


STRUCT_C0_SEND = struct.Struct('>chh')
STRUCT_C0_RECV = struct.Struct('>hhc')


def main():
    print('Connecting to microcontroller...')
    with serial.Serial(port='/dev/ttyACM0', baudrate=115200) as conn:
        print('Connected.\n')

        while True:
            try:
                send = input('Send <lv,rv>: ')
                send = send.split(',')
                send = STRUCT_C0_SEND.pack(b'\xC0', int(send[0]), int(send[1]))
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(e)
                continue

            t0 = time()
            conn.write(send)
            conn.flush()
            print(f'Sent: {send}')

            result = conn.read()
            print(f'Result: {result}')
            if result != b'\xAA':
                print('Did not receive ACK!')
                continue

            lmv, rmv, bumpers = STRUCT_C0_RECV.unpack(conn.read(5))
            t1 = time()
            print(f'Bumpers: {bumpers}')
            print(f'LMV    : {lmv}')
            print(f'RMV    : {rmv}')
            print(f'Time   : {t1 - t0}')

        print('\nDisconnecting from microcontroller...')
    print('Disconnected.')


if __name__ == '__main__':
    main()
