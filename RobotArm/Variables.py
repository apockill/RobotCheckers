    #ROBOT CONSTANTS (HARDWARE SPECIFIC)
stretchMin              = 30.0
stretchMax              = 210.0
heightMin               = -55
heightMax               = 150.0
rotationMin             = -90.0
rotationMax             = 90.0
handRotMin              = -84
handRotMax              = 74
handAngleOpen           = 25.0
handAngleClose          = 70.0



    #ROBOT CONSTANTS (SOFTWARE SPECIFIC)
stationaryTolerance     =   50          #When checking if the robot is moving, then each servo must be within -this variable- between two readings. If not, the robot is "not stationary"

    #ROBOT/CAMERA CONSTANTS
targetTolerance         = 15            #The number of pixels that coordinates of a target must be to the desired point (on camera) to stop the robot from moving.


    #Calibration Constants/Variables
framesForCalibration    =  10           #How many frames to sample from when calibrating the robot's movement-to-pixel ratio.
trialsForCalibration    =  5
pixelsPerX              =  2.2372727    #How many pixels move with each change in degree rotation. This value IS NOT CONSTANT, will be changed if calibration function is run.
pixelsPerY              =  4.2653333    #Pixels moved per unit stretch
pixFromCamToArm         =  180



    #Camera hardware specific inputs (Cannot be derived from a program, must be entered beforehand for it to perform well)
camHorzFOV              = 47.9750       #Horizontal FOV FROM THE CENTER to the left/right of the screen
camVertFOV              = 61.7054       #Vertical FOV FROM THE CENTER to the top/bottom of the screen
pixelsPerDegree         =  9.4584             #10.1885       #DERIVED VARIABLE. IT'S CAMHORZFOV / HORZ PIXELS, AVGED WITH THE VICE VERSA FOR VERTICAL.


    #VISION CONSTANTS
keyPointsToTrack        = 500

