__author__ = 'AlexThiel'

import Robot

while True:
    Robot.moveTo(rotation=float(raw_input("Rotation?:")), stretch=float(raw_input("stretch?:")))
