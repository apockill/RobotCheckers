import Robot
from time import sleep


if '__main__' == __name__:
    sleep(3)

    while True:
        Robot.moveTo(stretch=float(raw_input('stretch?:')), relative=False)
