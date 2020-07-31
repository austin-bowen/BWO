import struct

import serial


STRUCT_C0_SEND = struct.Struct('>chh')
STRUCT_C0_RECV = struct.Struct('>hhc')


def main():
    import gamesir

    print('Connecting to GameSir controller...')
    controller = gamesir.get_controllers()[0]

    print('Connecting to microcontroller...')
    with serial.Serial(port='/dev/ttyACM0', baudrate=115200) as conn:
        print('Connected.\n')

        lv = rv = 0
        for event in controller.read_loop():
            if event.type not in controller.EVENT_TYPES:
                continue

            #print(GameSirController.EventCode(event.code), event.value)

            if event.type == controller.EventCode.LEFT_JOYSTICK_Y:
                value = event.value
                # Scale from [0, 255] to [-1., 1.]
                value = (value - 128) / 128
                # Scale from [-1., 1.] to [-50, 50]
                value *= 50
                lv = int(value)

            elif event.type == controller.EventCode.RIGHT_JOYSTICK_Y:
                value = event.value
                # Scale from [0, 255] to [-1., 1.]
                value = (value - 128) / 128
                # Scale from [-1., 1.] to [-50, 50]
                value *= 50
                rv = int(value)

            else:
                continue

            try:
                send = STRUCT_C0_SEND.pack(b'\xC0', lv, rv)
            except Exception as e:
                print(e)
                continue

            conn.write(send)
            conn.flush()
            print(f'Sent: {send}')

            result = conn.read()
            print(f'Result: {result}')
            if result != b'\xAA':
                print('Did not receive ACK!')
                continue

            lmv, rmv, bumpers = STRUCT_C0_RECV.unpack(conn.read(5))
            print(f'Bumpers: {bumpers}')
            print(f'LMV    : {lmv}')
            print(f'RMV    : {rmv}')

        print('\nDisconnecting from microcontroller...')
    print('Disconnected.')


if __name__ == '__main__':
    main()
