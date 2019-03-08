import itertools
import numpy
import time

import pypot.dynamixel

AMP = 30
FREQ = 0.5

if __name__ == '__main__':
    ports = pypot.dynamixel.get_available_ports()
    print('available ports:', ports)

    if not ports:
        raise IOError('No port available.')

    port = ports[0]
    print('Using the first on the list', port)

    dxl_io = pypot.dynamixel.Dxl320IO(port, baudrate=1000000)
    print('Connected!')

    found_ids = dxl_io.scan()
    print('Found ids:', found_ids)

    ids = found_ids[:-1]

    dxl_io.set_torque_limit(dict(zip(ids, itertools.repeat(100))))

    dxl_io.set_max_torque(dict(zip(ids, itertools.repeat(100))))

    a = dxl_io.get_torque_limit(ids);
    for b in a:
        print(b)

    print("-------------------")
    a = dxl_io.get_max_torque(ids);
    for b in a:
        print(b)

    print("-------------------")



    if len(found_ids) < 2:
        raise IOError('You should connect at least two motors on the bus for this test.')

    ids = found_ids[:2]

    dxl_io.enable_torque(ids)

    speed = dict(zip(ids, itertools.repeat(200)))
    dxl_io.set_moving_speed(speed)
    pos = dict(zip(ids, itertools.repeat(0)))
    dxl_io.set_goal_position(pos)


    t0 = time.time()
    while True:
        t = time.time()
        if (t - t0) > 5:
            break

        pos = AMP * numpy.sin(2 * numpy.pi * FREQ * t)
        dxl_io.set_goal_position(dict(zip(ids, itertools.repeat(pos))))

        time.sleep(0.02)
