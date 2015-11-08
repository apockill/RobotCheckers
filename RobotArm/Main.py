import math
import cv2
import Common
import Robot
import CheckersAI
import Variables as V
import numpy as np
from Video import Vision
from Video import ImageStitching
from  time import sleep
from threading import Thread



#GENERIC FUNCTIONS
def focusOnTarget(getTargetCoords, **kwargs):
    """
    :param getTargetCoords: FOCUSES ON TARGET
    :param kwargs:
        "targetFocus":  (Default [screenXDimension, screenYdimension]) HOLDS AN ARRAY THAT TELLS YOU WHERE ON THE SCREEN TO FOCUS THE OBJECT
        "tolerance":    (defaults to Variables default) How many pixels away till the object is considered "focused" on.
        "ppX":          (Default -1) The amount of pixels that one "stretch" move of the robot will move. This changes according to what height the robot is at
                                    Which is the reason it is even adjustable. If this is not put in, the robot will target very slowly, but be most likely
                                    to reach the target. So you are trading reliability (at all heights) for speed at a specific situation. Same for ppY, except
                                    relating to the "rotation" movement of the robot.
        "ppY":          (Default -1)
    :return: THE ROBOT WILL HAVE THE OBJECT FOCUSED ON THE PIXEL OF TARGETFOCUS

    RECOMMENDED ppX and ppY VALUES FOR DIFFERENT HEIGHTS:
        HEIGHT = MAX: ppX = 3,    ppY = 12  (avg 3-4 moves)
        HEIGHT = 70:  ppX = 3.75  ppY = 15  (avg 2-4 moves)
        HEIGHT = 0:   ppx = 5.3   ppy = 38  (avg 3-5 moves)
    """

    ppX           = kwargs.get("ppX",           -1)     #Pixels per X move
    ppY           = kwargs.get("ppY",           -1)     #Pixels per Y move
    targetFocus   = kwargs.get("targetFocus",   [screenDimensions[0] / 2, screenDimensions[1] / 2])   #The XY value of WHERE on the screen we want the robot to focus on.
    tolerance     = kwargs.get("tolerance",     15)
    maxMoves      = kwargs.get("maxMoves",      1000)
    jump          = kwargs.get("jump",          0)      #jump several incriments at once when far from the target area
    waitTime      = kwargs.get("wait",          .1)     #Miliseconds to wait between moves
    targetSamples = kwargs.get('targetSamples', 1)

    sign        = lambda x: (1, -1)[x < 0]  #sign(x) will return the sign of a number'
    moveCount   = 0                         #The amount of moves that it took to focus on the object

    while True:  #Robot arm will slowly approach the target
        try: #GET THE POSITION OF THE TARGET ON THE SCREEEN. IF TARGETSAMPLES > 1, GET THE AVERAGE OF THE SAMPLES
            avgCoords = [0, 0]
            for s in range(targetSamples):
                coords = getTargetCoords()
                avgCoords = [a + b for a, b in zip(coords, avgCoords)] #Sum the lists
                lastFrame = vid.frameCount
                while lastFrame == vid.frameCount and not s == targetSamples - 1: pass  #Wait for new frame
            avgCoords = [n / targetSamples for n in avgCoords]

        except NameError as e:  #IF TARGET NOT FOUND, THROW AN ERROR
            print "ERROR: focusOnTarget(", locals().get("args"), "): error (", e, "). Object not found. Leaving Function..."
            raise  #RE-Raise the exception for some other program to figure out


        #CALCULATE THE DISTANCE FROM THE TARGETFOCUS TO THE TARGET COORDINATES
        distance = ((coords[0] - targetFocus[0]) ** 2 + (coords[1] - targetFocus[1]) ** 2) ** 0.5  #For debugging
        xMove = float(0)
        yMove = float(0)
        yDist = targetFocus[0] - coords[0]
        xDist = targetFocus[1] - coords[1]

        #FIGURE OUT WHAT DIRECTION TO MOVE, AND HOW MUCH
        if abs(xDist) > tolerance:
            xMove  = sign(xDist) * 1
            xMove += sign(xDist) * jump * (abs(xDist) > tolerance * 5)  #If far from the target, then move a little faster
            xMove += (xDist / ppX)   * (abs(xDist) > tolerance     and not ppX == -1)  #If a pp? setting was sent in kwArgs, then perform it.

        if abs(yDist) > tolerance:
            yMove  = sign(yDist) * .5
            yMove += sign(yDist) * jump * (abs(yDist) > tolerance * 5)
            yMove += (yDist / ppY)   * (abs(yDist) > tolerance     and not ppY == -1)


        #PERFORM THE MOVE
        if not (int(xMove) == 0 and int(yMove) == 0):
            moveCount += 1
            Robot.moveTo(rotation = -yMove, stretch = xMove)  #TODO: make a variable that flips these around according to orientation of camera
            #print Robot.getPosArgsCopy()
            if yDist < tolerance * 2 or xDist < tolerance * 1.25:  #Slow down a bit when approaching the target
                sleep(.3)
            else:
                sleep(.1)

            if Robot.pos["stretch"] == Robot.stretchMax or Robot.pos["stretch"] == Robot.stretchMin:
                if yMove == 0:  #If the robot can still focus more on the rotation
                    print "focusOnTarget(): Object out of Stretching reach, but Rotation was focused"
                    break

                if Robot.pos["rotation"] == Robot.rotationMax or Robot.pos["rotation"] == Robot.rotationMin:
                    print "focusOnTarget(): Object out of BOTH Stretching and Rotation Reach. Quiting Targeting..."
                    break

            if Robot.pos["rotation"] == Robot.rotationMax or Robot.pos["rotation"] == Robot.rotationMin:
                if xMove == 0:  #If the robot can still focus more on the rotation
                    print "focusOnTarget(): Object out of Rotating reach, but Stretch was focused"
                    break


            if moveCount >= maxMoves:
                print "focusOnTarget(): ERROR: Robot moved ", moveCount, " which is over the move limit."
                break

        else:
            print "focusOnTarget(", locals().get("args"), "): Object Targeted in ", moveCount, "moves.", " Final distance from target: ", distance
            break

def focusOnCoord(coords, **kwargs):  #Arguments: targetFocus (The pixel location where you want the target to be alligned. Default is the center of the screen
    targetFocus = kwargs.get('targetFocus', [screenDimensions[0] / 2, screenDimensions[1] / 2])
    if len(coords) == 0:
        print "Trouble focusing on coord:", coords
        return
    diffX = coords[0] - targetFocus[0]
    diffY = coords[1] - targetFocus[1]
    #ratio = (Robot.pos['height'] + 60.0) / (Robot.heightMax + 60.0)
    #print Robot.pos['height']
    #ratio = 1  #for not, while I get THAT working...
    xMove = (diffX / (V.pixelsPerX ))
    yMove = (diffY / (V.pixelsPerY  ))
    print "Xmove: ", xMove, "Ymove: ", yMove
    Robot.moveTo(rotation = yMove / 2, stretch = xMove * 2, waitForRobot = True)
    #Robot.moveTo(x = xMove, y = -yMove, waitForRobot = True)

def calibrateRobotCamera(target):
    """
        Figures out important variables for the robot from a series of trials and repeats and then averaging the result
    avgPixelPerX    Using cartesian coordinates, how many pixels (at max height) does an object move per unit x move of the robot?
    avgPixelPerY    Same as above, but for y
    """
    xToMove             = 110.0   #How many degrees to turn during the rotation test.
    yToMove             = 75.0   #How many 'units' to stretch
    avgPixelPerX        = 0.0
    avgPixelPerY        = 0.0
    #Robot.moveTo(height = 90, relative = False, wiatForRobot = True)#(V.heightMax - V.heightMin) / 2)
    for x in range(0, V.trialsForCalibration):
        focusOnTargetManual(target)

        #X CALIBRATION: GO TO COORD 1
        Robot.moveTo(x = xToMove / 2, waitForRobot = True)
        coords1 = objTracker.getTargetAvgCenter(target, V.framesForCalibration)
        #GO TO COORD 2
        Robot.moveTo(x = -xToMove, waitForRobot = True)
        coords2 = objTracker.getTargetAvgCenter(target, V.framesForCalibration)
        #GO TO COORD 3
        Robot.moveTo(x = xToMove, waitForRobot = True)
        coords3 = objTracker.getTargetAvgCenter(target, V.framesForCalibration)

        Robot.moveTo(x = -xToMove / 2)                       #Return robot
        avgPositiveX =  (coords2[0] - coords1[0]) / xToMove
        avgNegativeX =  (coords2[0] - coords3[0]) / xToMove
        avgPixelPerX += (avgPositiveX + avgNegativeX) / 2.0
        print 'Avg Positive X: %s' % avgPositiveX
        print 'Avg Negative X: %s' % avgNegativeX

        #Y CALIBRATION: GO TO COORD 1
        focusOnTargetManual(target)
        Robot.moveTo(y = yToMove / 2, waitForRobot = True)
        coords1 = objTracker.getTargetAvgCenter(target, V.framesForCalibration)
        #GO TO COORD 2
        Robot.moveTo(y = -yToMove, waitForRobot = True)
        coords2 = objTracker.getTargetAvgCenter(target, V.framesForCalibration)
        #GO TO COORD 3
        Robot.moveTo(y = yToMove, waitForRobot = True)
        coords3 = objTracker.getTargetAvgCenter(target, V.framesForCalibration)

        Robot.moveTo(y = -yToMove / 2)                       #Return robot
        avgPositiveY =  (coords1[1] - coords2[1]) / yToMove
        avgNegativeY =  (coords3[1] - coords2[1]) / yToMove
        avgPixelPerY += (avgPositiveY + avgNegativeY) / 2.0

        print 'Avg Positive Y: %s' % avgPositiveY
        print 'Avg Negative Y: %s' % avgNegativeY

    print 'Calibration over'
    return avgPixelPerX / V.trialsForCalibration, avgPixelPerY / V.trialsForCalibration

def waitTillStill(**kwargs):
    maxTime     = kwargs.get("timeout", 1)  #If it goes over maxTime seconds, the function will quit
    maxMovement = kwargs.get("movement", 8)
    if maxMovement <= 5:
        print "waitTillStill(", locals().get("args"), "): It is unwise to set the movement parameter to < than 4. It is set at ", maxMovement

    timer = Common.Timer(maxTime)  #Will wait one second max before continuing
    while objTracker.getMovement() > maxMovement:
        if timer.timeIsUp():
            print "waitTillStill(", locals().get("args"), "): Timer has timed out. Consider lowering acceptableMovement. Continuing...", objTracker.getMovement()
            break

def getAngle(quad):
    """
    :param quad: The 4 coordinate point array
    :return: Returns the angle of the block in the long side, which can be used to orient the wrist of the robot.
    """
    side1 = ((quad[0][0] - quad[1][0]) ** 2.0 + (quad[0][1] - quad[1][1]) ** 2.0) ** 0.5
    side2 = ((quad[1][0] - quad[2][0]) ** 2.0 + (quad[1][1] - quad[2][1]) ** 2.0) ** 0.5
    if side2 < side1:
        angle = math.atan((quad[0][1] - quad[1][1]) * 1.0 / (quad[0][0] - quad[1][0]) * 1.0)
    else:
        angle = math.atan((quad[1][1] - quad[2][1]) * 1.0 / (quad[1][0] - quad[2][0]) * 1.0 )

    angle = math.degrees(angle)
    return angle



###########  JENGA FUNCTIONS ###########
def stackJenga():
  #Main Function
    print "Playing Jenga!"
    #SET UP VARIABLES AND RESET ROBOT POSITION
    blocks = (1, 0, 2)  #The order to place the blocks
    markerLKP = {"rotation": 30, "stretch": 100}  #Keep track of the "marker block"'s "Last Known Position" (LKP)
    searchPos = {'rotation': -15, 'stretch': Robot.stretchMax / 2.5, 'height': Robot.heightMax, 'wrist': 0}
    originalMarker = markerLKP.copy()
    originalSearchPos = searchPos.copy()
    Robot.moveTo(relative = False, waitForRobot = True, **searchPos)  #Get robot in position before testing movementConstant
    movementConstant = (isObjectGrabbed() + isObjectGrabbed() + isObjectGrabbed()) / 3  #Get an average of how much movement there is when there is no block


    for l in (1, 2, 3, 4, 5):  #For each layer
        #FIND A BLOCK, PICK UP THE BLOCK, AND PLACE IT IN THE CORRECT SPOT
        b = 0  #Current block you are putting. THE ORDER IS IN THE "blocks" ARRAY, HOWEVER!!!

        while b in blocks:  #Blocks to place, in order
            print "stackJenga(XXX): Currently on layer ", l, " and on block ", b

            #GO TO POSITION FOR SEARCHING FOR BLOCKS, BUT DO IT SLOWLY AS TO NOT MOVE ROBOT'S BASE
            sleep(.2)
            Robot.moveTo(relative = False, waitForRobot = True, **searchPos)


            #START SEARCHING FOR BLOCKS
            while len(objTracker.getShapes(4)) == 0:  #Search for a view that has a block in it
                if Robot.pos["rotation"] > -69:
                    Robot.moveTo(rotation = -10)
                    waitTillStill()
                else:
                    print "stackJenga(", locals().get("args"), "): No shapes found after one sweep."
                    searchPos = originalSearchPos.copy()
                    Robot.moveTo(rotation = -15, stretch = V.stretchMax / 2.5, height = V.heightMax, relative = False)  #Go back and sweep again
            searchPos = {"rotation": Robot.pos["rotation"], "stretch": V.stretchMax / 2.5, "height": V.heightMax}


            #PICK UP A BLOCK
            try:
                pickUpBlock(movementConstant, Robot.getPosArgsCopy())  #IF pickUpBlock() FAILS 3 ATTEMPTS, RESTART THIS FOR LOOP
            except Exception as e:
                Robot.setGrabber(0)
                print "ERROR: stackJenga(XXX): ", e, " while picking up block. Restarting for loop and attempting again.."
                continue  #Since b never got upped one, then there is no harm in using continue


            #MOVE TO, FOCUS ON, AND RECORD NEW POSITION, OF THE MARKER BLOCK
            Robot.moveTo(rotation = markerLKP["rotation"] / 2, height = 70, relative = False)
            sleep(.2)
            Robot.moveTo(rotation = markerLKP["rotation"], stretch = markerLKP["stretch"], height = 70, relative = False, waitForRobot = True)
            waitTillStill()
            error = False

            if len(objTracker.getShapes(4)) == 0:  #If marker not seen in the middle, go to the right and check for it
                Robot.moveTo(rotation = markerLKP["rotation"] + 10, stretch = 100, relative = False, waitForRobot = True)
            if len(objTracker.getShapes(4)) == 0:  #If marker still not to the right, move to the left and check for it
                Robot.moveTo(rotation = markerLKP["rotation"] - 10, stretch = 100, relative = False, waitForRobot = True)

            if len(objTracker.getShapes(4)) > 0:  #If the marker HAS been seen, then focus on it.
                try:
                    targetFocus = [screenDimensions[0] * .75, screenDimensions[1] / 2]  #The right edge of the screen
                    focusOnTarget(lambda: objTracker.bruteGetFrame(lambda: objTracker.getNearestShape(4, nearestTo = targetFocus ).center), targetFocus = targetFocus, tolerance = 9, **{"ppX": 3.7, "ppY": 15})
                    print "stackJenga(XXX): Successfully focused on marker block"
                except Exception as e:
                    print "ERROR: stackJenga(XXX): ", e, " while searching for marker."
                    error = True
            else:
                error = True  #If the marker has NOT been seen, catch it right below
            if error:  #If the robot messed up either focusing on the object or having never seen it, then...
                print "stackJenga(XXX): Failed too many times searching for marker. Placing current off to the side, RESETTING markerLKP"
                placeBlock(searchPos, 0, 0)
                markerLKP = originalMarker.copy()
                continue
            waitTillStill()


            #RECORD NEW ADJUSTED "LAST KNOWN POSITION" OF markerLKP, THROUGH AN AVG OF LAST AND CURRENT.
            if abs(Robot.pos["rotation"] - markerLKP["rotation"]) < 20 and abs(Robot.pos["stretch"] - markerLKP["stretch"]) < 30 and not (l == 1 and b == 0):  #If the marker is in a semi-valid position, then RECORD that position. (used to be a big issue)
                markerLKP = {"rotation": (markerLKP["rotation"] + Robot.pos["rotation"]) / 2, "stretch": (markerLKP["stretch"] + Robot.pos["stretch"]) / 2}  #Get avg of last known position and new known position (messes things up less)
                print "stackJenga(XXX): New location of markerLKP: ", markerLKP  #Print new location of the marker
            else:
                print "stackJenga(XXX): SOMETHINGS GONE WRONG with markerLKP! markerLKP: ", markerLKP


            #PLACE BLOCK ONTO THE CORRECT POSITION ON THE STACK
            placeBlockAtPosition(l, blocks[b])
            b += 1  #Mark block on layer as completed, so for loop will move on to next block.

    #MISSION COMPLETE! INITIATE DANCE SEQUENCE!
    Robot.moveTo(relative = False, **Robot.home)
    sleep(.6)
    Robot.moveTo(rotation = -30)
    sleep(.3)
    Robot.moveTo(rotation = 30)
    sleep(.3)
    Robot.moveTo(height = 0)
    sleep(.3)
    Robot.moveTo(height = Robot.heightMax)
    sleep(.3)

def isObjectGrabbed():
    currentWrist = Robot.pos["wrist"]
    Robot.moveTo(wrist = -40, relative = False)
    Robot.moveTo(wrist = 20, relative = False)
    sleep(.01)
    movement = objTracker.getMovement()

    Robot.moveTo(wrist = currentWrist, relative = False)
    print "isObjectGrabbed(", locals().get("args"), "): Movement = ", movement
    return movement

def pickUpBlock(movConstant, objectLastSeen, **kwargs):
    """
    Robot will attempt to pick up block. If it fails, it will RECURSIVELY attempt to pick it up again from the last known position. kwarg "attempts" is used to cut off the robot after
    the 3rd (configurable) attempt.
    :param movementConstant: this is the average pixel change per frame when the robot is stationary and there is no jenga block in it's grabber when the grabber moves.
                            It is used with isObjectGrabbed to detect if the robot successfuly grabbed a jenga block
    :param startingPos:     Self explanatory. String of robot position, from where to start
    :return:
    kwargs:
        attempts: Counts how many times before this program has run itself recursively, in order to determine if it should stop attempting and return an error.
        attemptRefocus: (Defaults to true). If this is true, the robot will attempt to refocus on the block using a different manner if it is >50 or <-50 degrees position.
    """
    attempts = kwargs.get("attempts", 0)                                                                            #Total times this func has been run recursively (at 3, it raises an error)
    attemptRefocus = kwargs.get("attemptRefocus", True)                                                             #Should robot attempt to do fancy refocusing on certain blocks
    targetCenter = [screenDimensions[0] / 2, screenDimensions[1] / 2]                                               #Target center of screen
    heightSettings = {"150": {"ppX": 3, "ppY": 12}, "70": {"ppX": 3.7, "ppY": 15}, "0": {"ppX": 5.3, "ppY": -1}}    #Holds the best ppX ppY settings for different heights.
    getShapeCenter = lambda: objTracker.bruteGetFrame(lambda: objTracker.getNearestShape(4).center)                 #This function is used a lot, so it was useful to save as a variable.

    #CHECK IF THIS FUNCTION HAS BEEN RUN RECURSIVELY MORE THAN THE SET LIMIT, AND QUIT IF IT HAS
    print "pickUpPiece(", locals().get("args"), "): Attempt number: ", attempts
    if attempts == 3:  #If max attempts have been hit
        print "pickUpPiece(", locals().get("args"), "): Too many recursive attempts. Raising error"
        raise Exception("BlockNotGrabbed")


    #GET ROBOT INTO POSITION AND CHECK IF THE OBJECT IS STILL THERE. QUIT IF IT IS NOT.
    waitTillStill()
    Robot.setGrabber(0)
    Robot.moveTo(relative = False, **objectLastSeen)
    waitTillStill()
    sleep(.1)
    if len(objTracker.getShapes(4)) == 0:  #If no objects in view
        print "pickUpPiece(", locals().get("args"), "): No objects seen at start. Raising error..."
        raise NameError("ObjNotFound")


    #BEGIN FOCUSING PROCESS UNTIL ROBOT IS AT 0 HEIGHT AND COMPLETELY FOCUSED ON OBJECT
    try:
        if Robot.pos["height"] == 150:
            print "pickUpBlock(", locals().get("args"), "): Focus at height 150"
            objectLastSeen = Robot.getPosArgsCopy(dontRecord = ["wrist, grabber, height"])
            focusOnTarget(getShapeCenter, **heightSettings["150"])  #Tries many times (brute) to get the nearest shapes .center coords & focus
            objectLastSeen = Robot.getPosArgsCopy(dontRecord = ["wrist, grabber, height"])
            Robot.moveTo(height = 70, relative = False)
            waitTillStill()

        if Robot.pos["height"] == 70:
            print "pickUpBlock(", locals().get("args"), "): Focus at height 70"
            focusOnTarget(getShapeCenter, **heightSettings["70"])
            objectLastSeen = Robot.getPosArgsCopy(dontRecord = ["wrist, grabber, height"])
            Robot.moveTo(height = 0, relative = False)
            waitTillStill()

        if Robot.pos["height"] == 0:
            print "pickUpBlock(", locals().get("args"), "): Focus at height 0"
            focusOnTarget(getShapeCenter,  tolerance = 7, **heightSettings["0"])
            objectLastSeen = Robot.getPosArgsCopy(dontRecord = ["wrist, grabber, height"])

        shape = objTracker.bruteGetFrame(lambda: objTracker.getNearestShape(4)).vertices
    except NameError as e:
        print "ERROR: pickUpBlock(", locals().get("args"), "): ", e
        pickUpBlock(movConstant, objectLastSeen, attempts = attempts + 1)
        return False
        #raise Exception("PickupFailed")


    #IF THE OBJECT IS > 50 OR < -50 DEGREES, DO ANOTHER ROUND OF FOCUSING ON IT, AND PERFORM A DIFFERENT "DROP DOWN" MANUEVER
    angle = getAngle(shape)  #Get angle of object in camera
    if (angle > 50 or angle < -50) and attemptRefocus:
        try:
            print "pickUpBlock(", locals().get("args"), "): Performing re-focusing manuever on block of angle: ", angle
            targetFocus = [screenDimensions[0] / 3.5, screenDimensions[1] / 2]
            focusOnTarget(lambda: objTracker.bruteGetFrame(lambda: objTracker.getNearestShape(4, nearestTo = targetFocus).center),  tolerance = 8, targetFocus = targetFocus, **heightSettings["0"])
            objectLastSeen = Robot.getPosArgsCopy(dontRecord = ["wrist, grabber, height"])
        except NameError as e:  #Since it failed, try again but this time send the function a "normalPickup = True", so it won't attempt to do it again
            print "ERROR: pickUpBlock(", locals().get("args"), "): ", e, " when trying to RE-FOCUS on a >50 <-50 block"
            pickUpBlock(movConstant, objectLastSeen, attempts = attempts + 1, attemptRefocus = False)
            return False

        #MOVE SO OBJECT IS UNDER GRABBER
        Robot.moveTo(height = -05, relative = False)  #Ease into it
        waitTillStill()
        Robot.moveTo(stretch = 26)
        waitTillStill()

    else:
        #MOVE SO OBJECT IS UNDER GRABBER
        Robot.moveTo(stretch = 52, rotation = 2.25)  #Added a rotation to fix weird issues that had been occurring...
        waitTillStill()


    #MOVE WRIST, AND DESCEND ONTO OBJECT AND PICK IT UP
    Robot.moveTo(wrist = angle, height = -20, relative = False)
    sleep(.075)
    Robot.moveTo(height = -38, relative = False)
    sleep(.05)
    Robot.setGrabber(1)  #Pick up object
    sleep(.125)
    Robot.moveTo(height = -5, relative = False)
    sleep(.2)


    #MEASURE THE MOVEMENT OF THE OBJECT AS THE WRIST MOVES
    Robot.moveTo(wrist = -40, relative = False)  #Start rotating the wrist
    Robot.moveTo(wrist = 20, relative = False)
    timer = Common.Timer(.4)
    highestMovement = 0
    while not timer.timeIsUp():  #Gets the highest measured movement in .3 seconds, to DEFINITELY catch the wrist moving. Eliminates problems.
        newMovement = objTracker.getMovement()
        if newMovement > highestMovement: highestMovement = newMovement
        if highestMovement > movConstant + 1.5: break
    print "pickUpBlock(", locals().get("args"), "): highestMovement: ", highestMovement


    #IF MOVEMENT IS < MOVCONSTANT, THEN NO OBJECT WAS PICKED UP. RE-RUN FUNCTION.
    if highestMovement < movConstant + 3:
        print "pickUpBlock(", locals().get("args"), "): Failed to suck in object. Retrying..."
        pickUpBlock(movConstant, objectLastSeen, attempts = attempts + 1, attemptRefocus = not attemptRefocus)
        return False

    Robot.moveTo(wrist = 0, relative = False)  #Return wrist
    return True

def placeBlock(position, height, wrist):
    currentPosition = Robot.getPosArgsCopy()
    Robot.moveTo(rotation = position["rotation"], stretch = position["stretch"], relative = False)  #GET IN POSITION OVER DROP ZONE
    Robot.moveTo(height = height, wrist = wrist, waitForRobot = True, relative = False)
    Robot.setGrabber(0)
    Robot.moveTo(height = currentPosition["height"], relative = False)
    Robot.moveTo(relative = False, **currentPosition)

def placeBlockAtPosition(layer, position):
    """
    FOR THE TOWER BUILDING. Aligns the brick to either a horizontal or vertical position, determined mathematically by the "layer" variable.
    The "layer" variable determines the height.
    The "position" variable is a string- it is either 0, 1, 2 determining where on the current layer of the tower (1 = left, 2 = middle, or something akin).
    that the brick should go on.
    :param layer: Layer on the building. Starts at 1. 1st layer is horizontal.
    :param position: "right", "left
    :return: A brick placed
    """
    #SET UP VARIABLES
    currentPosition = Robot.getPosArgsCopy()  #Back up the position in case of error
    heightForLayer = {"1": -32, "2": -15, "3": 3, "4": 24, "5": 40.5}
    print "placeBlockAtPosition(", locals().get("args"), "): Current pos: ", currentPosition

    #PICK THE RELEVANT MOVEMENTS FOR EACH LAYER/POSITION, AND PERFORM IT
    if not layer % 2:  #IF THIS IS THE VERTICAL LAYER
        print "PLACING DOWN VERTICAL BLOCK ON POSITION ", position, "ON LAYER ", layer
        #HOW MUCH STRETCH / ROTATION IS NECESSARY FOR EACH POSITION. [POSITION]
        rotToMoveSide     = [19,  0, -19]
        rotToMoveSlip     = [-12, 0,   10]
        stretchToMove     = [-2,  0,  -2]  #Adjust the angle so it is truly vertical.
        wristToMove       = [-10, 0,  10]
        Robot.moveTo(height = heightForLayer[str(layer)] + 34, wrist = wristToMove[position], relative = False)  #Move Wrist BEFORE going all the way down, or else you might knock some blocks
        Robot.moveTo(rotation =  rotToMoveSide[position], stretch = stretchToMove[position])  #Go besides (but offset) to the end position of the block
        sleep(.1)
        Robot.moveTo(height = heightForLayer[str(layer)], relative = False)  #Finish going down
        waitTillStill()
        Robot.moveTo(rotation = rotToMoveSlip[position] * .75)  #Slip block in to the correct position sideways (Hopefully pushing other blocks into a better position
        sleep(.1)
        Robot.moveTo(rotation = rotToMoveSlip[position] * .25)  #Slip block in to the correct position sideways (Hopefully pushing other blocks into a better position

    else:           #IF THIS IS THE HORIZONTAL LAYER
        stretchToMoveSide = [-45, 0, 49]  #To get parallel to the part
        stretchToMoveSlip = [25, 0, -25]  #To slip it in all the way
        wristToMove = 73.5  #Equivalent to 90 degrees, for some reason
        Robot.moveTo(height = heightForLayer[str(layer)] + 34, wrist = wristToMove, relative = False)  #Move Wrist BEFORE going all the way down, or else you might knock some blocks
        Robot.moveTo(stretch = stretchToMoveSide[position])
        sleep(.1)
        Robot.moveTo(height = heightForLayer[str(layer)], relative = False)  #Finish going down
        waitTillStill()
        Robot.moveTo(stretch = stretchToMoveSlip[position] * .75)  #Ease into the correct position by splitting it into two moves
        sleep(.1)
        Robot.moveTo(stretch = stretchToMoveSlip[position] * .25)

    #DROP BRICK AND GO BACK TO ORIGINAL POSITION
    sleep(.1)
    print Robot.pos
    Robot.setGrabber(0)
    Robot.moveTo(height = currentPosition["height"], relative = False)
    sleep(.2)
    Robot.moveTo(relative = False, **currentPosition)
    waitTillStill()



########### Checkers FUNCTIONS ###########
def playCheckers():
    global streamVideo
    print "playCheckers(): Beginning Checkers!"
    streamVideo = True
    #cornerInfo = getBoardCorners()  #Get the location of the boards corners at two different robot heights

    cornerInfo =  [{'corners': [{'y':  -33.58238318627188, 'x': 172.76638428678885}, {'y': 114.53631117107041, 'x':  141.4405649851687}, {'y': 153.9866725937722,  'x': 289.60681045772805}, {'y': -0.0, 'x': 321.0}], 'distFromBase': 143, 'height': 100},
                   {'corners': [{'y': -33.964001177024976, 'x': 174.7296386536842},  {'y': 117.63013257263668, 'x': 140.18613309077298}, {'y': 149.81686491405043, 'x':  294.0321529821614}, {'y': -0.0, 'x': 324.0}], 'distFromBase': 161, 'height': 40},
                   {'corners': [{'y':  -33.58238318627188, 'x': 172.76638428678885}, {'y': 118.91570779200975, 'x': 141.71822197701093}, {'y': 161.44160354203024, 'x': 291.24836247741877}, {'y': -0.0, 'x': 329.0}], 'distFromBase': 172, 'height': 8},
                   {'corners': [{'y':  -33.16680193438369, 'x': 188.09828082533173}, {'y': 128.55752193730785, 'x': 153.20888862379562}, {'y': 171.62260556720332, 'x':  309.6153763273461}, {'y': 18.16057681630151, 'x': 346.52444855983714}], 'distFromBase': 191, 'height': -6},
                   {'corners': [{'y':  -33.16680193438369, 'x': 188.09828082533173}, {'y': 127.2754516241584,  'x': 146.41365856321778}, {'y': 164.3150469750618,  'x':  309.0316575006244}, {'y': 6.055985033737379, 'x': 346.94715021926777}], 'distFromBase': 191, 'height': -15}]
    cornerInfoHigh    = cornerInfo[0]
    cornerInfoLow     = cornerInfo[-1]
    #groundFormula     = getGroundFormula(cornerInfoLow['corners'])
    groundFormula     = lambda stretch: -0.0994149347433  * stretch + -45.0

    redThreshold = 0     #Used to measure what piece is pink and what piece is green. This constant is set later on using getAverageColor()
    firstLoop    = True  #This will tell the robot to calibrate the average color on the first round of the game.
    AI = CheckersAI.DraughtsBrain({'PIECE':     15,  #Checkers AI class. These values decide how the AI will play
                                   'KING':     30,
                                   'BACK':      3,
                                   'KBACK':     3,
                                   'CENTER':    6,
                                   'KCENTER':   7,
                                   'FRONT':     6,
                                   'KFRONT':    3,
                                   'MOB':     6}, 7)

    while not exitApp:
        #stitchedFrame = cv2.imread("F:\Google Drive\Projects\Git Repositories\RobotStorage\RobotArm\stitched.png")

        #GET A STITCHED IMAGE THAT IS AN OVERVIEW OF THE BOARD AREA
        stitchedFrame = getBoardOverview(cornerInfoHigh, finalImageWidth = 1000)
        #cv2.imwrite('F:\Google Drive\Projects\Git Repositories\RobotStorage\RobotArm\stitched.png', stitchedFrame)


        #FIND THE BOARD IN THE STITCHED IMAGE
        shapeArray, edgedFrame = objTracker.getShapes(sides=4, peri=0.05,  minArea= (stitchedFrame.shape[0] * stitchedFrame.shape[1]) / 4,
                                                      threshHold=cv2.THRESH_OTSU, frameToAnalyze=stitchedFrame, returnFrame = True)
        if len(shapeArray) == 0:  #Make sure that the board was correctly found. If not, restart the loop and try again.
            print "__playCheckers()___: No board Found"
            continue

        #cv2.imwrite("F:\Google Drive\Projects\Git Repositories\RobotStorage\RobotArm\stitched.png", stitchedFrame)


        #ISOLATE THE BOARD AND FIND THE CIRCLES IN IT
        warped      = objTracker.getTransform(shapeArray[0], frameToAnalyze=stitchedFrame, transformHeight=600, transformWidth=600)  #Isolate board
        circleArray = objTracker.getCircles(frameToAnalyze = warped, minRadius = 40)  #Get circles on screen


        #GET THE BOARD STATE
        if firstLoop: redThreshold = getAverageColor(circleArray)[2]  #  If this is the first time being run, get the average color of the pieces on board
        boardState, warped         = getBoardState(warped, circleArray, [600, 600], redThreshold)  #Find and label all circles with team, color, and location


        #SET FRAMES FOR THE WINDOWS:
        vid.windowFrame["Main"]        = objTracker.drawShapes([shapeArray[0]], frameToDraw=stitchedFrame)
        vid.windowFrame["Perspective"] = objTracker.drawCircles(circleArray,    frameToDraw=warped)


        #GET BEST MOVE FROM ROBOT
        move                = AI.best_move(board=boardState)  #Get the best possible move for the robot to perform
        print move
        # move                = AI.best_move(board=boardState)  #Get the best possible move for the robot to perform
        # columnFrom, rowFrom = list(move.source[::-1])
        # columnTo,     rowTo = list(move.destination[::-1])
        # print "playCheckers(): Move From [", columnFrom, rowFrom, "] to [", columnTo, rowTo, "]"
        #
        #
        # #PICK UP THE PIECE AND MOVE IT TO THE CORRECT LOCATION
        # pickupLocation = pickUpPiece(columnFrom, rowFrom, cornerInfoLow, groundFormula)  #Pick up piece
        #
        #
        # placePiece(columnFrom, rowFrom, columnTo, rowTo, cornerInfoLow, groundFormula, pickupLocation)  #Place piece


        #WAIT FOR PERSON TO WAVE HAND IN FRONT OF CAMERA BEFORE RESTARTING THE CYCLE
        raw_input("Press ENTER to continue:")

def getBoardState(frame, circleArray, screenDimensions, redThreshold):
    global boardSize

    board = [[0 for i in range(boardSize)] for j in range(boardSize)]
    squareSize = (screenDimensions[0] / boardSize)
    dist       = lambda a, b: ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** .5  #sign(x) will return the sign of a number'


    frameToDraw = frame.copy()

    for column in range(boardSize):
        for row in range(boardSize):
           # print "row: ", row, "column: ", column, "squareSize: ", squareSize
            location = [squareSize * row + squareSize / 2, squareSize * column + squareSize / 2]
           # print location
            if len(circleArray) == 0: continue

            circleArray = sorted(circleArray, key = lambda c: (c.center[0] - location[0]) ** 2 + (c.center[1] - location[1]) ** 2)
            nearest     = circleArray[0]


            if dist(nearest.center, location) < squareSize / 2:
                fromX = int(nearest.center[0] - nearest.radius / 2)
                toX   = int(nearest.center[0] + nearest.radius / 2)
                fromY = int(nearest.center[1] - nearest.radius / 2)
                toY   = int(nearest.center[1] + nearest.radius / 2)

                if nearest.color[2] > redThreshold:
                    board[column][row] = 1  #IF IS RED PIECE
                    color = (0, 0, 255)
                else:
                    board[column][row] = 2  #IF IS GREEN PIECE
                    color = (0, 255, 0)

                cv2.rectangle(frameToDraw, tuple([fromX, fromY]), tuple([toX, toY]), color, 3)
                cv2.putText(frameToDraw, str(row) + "," + str(column), (nearest.center[0] - 25, nearest.center[1] + 10), cv2.FONT_HERSHEY_SIMPLEX, 1, 0)
                del circleArray[0]
        #print board[column]
    return board, frameToDraw

def getBoardOverview(cornerInfo, **kwargs):
    """
    Gets several images of the board and stitches them together to return a stitched image of the whole board.
    It then resizes the image if the appropriate kwarg is set.
    """

    corners = cornerInfo['corners']
    finalWidth = kwargs.get("finalImageWidth", -1)
    Robot.moveTo(height=150)

    #Location of the top middle of the board and the bottom middle of the board
    picturePositions = [getSquarePosition(2.5, 1, corners),
                        getSquarePosition(2.5, 4, corners)]

    images_array = []
    for index, position in enumerate(picturePositions):
        Robot.moveTo(stretchDistFromBase=cornerInfo['distFromBase'], relative = False, **position)
        sleep(1.5)
        images_array.append(vid.frame)

    final_img = ImageStitching.stitchImages(images_array[0], images_array[1:], 0)


    # RESIZE THE IMAGE IF THE finalWidth FUNCTION AN ARGUMENT
    if not finalWidth == -1:
        resized = vid.resizeFrame(final_img, finalWidth)
        return resized

    return final_img

def getSquarePosition(column, row, corners):
    """
    This function returns the estimated robot rotation/stretch that a board square is located in.
    It does this by using the cornerLocations information to estimate the location of a square on the board.
    May not always work accurately, but should speed things up.

    This function will return a coordinate in this format: {"x": ?, "y": ?}
    This format can be sent to Robot.moveTo() command easily.

    Row and column: Where on the board you wish the camera to jump to
    :param corners: The array gotten from getBoardCorners()
    """

    global boardSize

    #For simplicity, get the bottom Right (bR) bottom Left (bL) and so on for each corner.
    dist   = lambda p1, p2: ((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2) ** .5
    ptDist = lambda d, a, b: [a[0] + ((b[0] - a[0]) / dist(a, b)) * d, a[1] + ((b[1] - a[1]) / dist(a, b)) * d]

    #GET [x,y] FORMAT COORDINATES FOR EACH CORNER OF THE CHECKERBOARD
    bR = [float(corners[0]["x"]), float(corners[0]["y"])]
    bL = [float(corners[1]["x"]), float(corners[1]["y"])]
    tR = [float(corners[3]["x"]), float(corners[3]["y"])]
    tL = [float(corners[2]["x"]), float(corners[2]["y"])]

    #GET LENGTH OF EACH SIDE
    lenBot = dist(bR, bL)
    lenTop = dist(tR, tL)
    lenRit = dist(bR, tR)
    lenLef = dist(bL, tL)


    #GET POINTS ON EACH SIDE THAT ARE CLOSEST TO THE DESIRED SPOT. ADD .5 TO ROW AND COLUMN CENTER VIEW ON THE MIDDLE OF THE SQUARE
    ptTop = ptDist((lenTop / boardSize) * (column + .5), tL, tR)
    ptBot = ptDist((lenBot / boardSize) * (column + .5), bL, bR)
    ptLef = ptDist((lenLef / boardSize) * (row + .5),    tL, bL)
    ptRit = ptDist((lenRit / boardSize) * (row + .5),    tR, bR)


    #GET THE POINT OF INTERSECTION, WHICH WILL BE THE FINAL POINT FOR THE ROBOT TO GO TO
    s1_x = ptBot[0] - ptTop[0]
    s1_y = ptBot[1] - ptTop[1]
    s2_x = ptRit[0] - ptLef[0]
    s2_y = ptRit[1] - ptLef[1]
    t = (s2_x * (ptTop[1] - ptLef[1]) - s2_y * (ptTop[0] - ptLef[0])) / (-s2_x * s1_y + s1_x * s2_y)
    #cornerOfSquare = [ptTop[0] + (t * s1_x), ptTop[1] + (t * s1_y)]

    finalCoords = {'x': ptTop[0] + (t * s1_x), 'y': ptTop[1] + (t * s1_y)}

    #print "getSquarePosition(): bottomL: ", lenBot, " topL: ", lenTop, " rightL: ", lenRit, " leftL: ", lenLef
    #print "getSquarePosition(): topPoint: ", ptTop, " botPoint: ", ptBot, " lefPoint: ", ptLef, " ritPoint: ", ptRit
    #print "getSquarePosition(): topLeft: ", tL, " topRight: ", tR, " bottomRight: ", bR, " bottomLeft: ", bL

    print "getSquarePosition(): For Row: ", row, " Col: ", column, " FinalCoords: ", finalCoords
    return finalCoords


#   #PICKUP AND PLACING FUNCTIONS
def pickUpPiece(column, row, cornerInfo, groundHeightFormula, attempts=0, **kwargs):
    global camDistFromGrabber
    global boardLength
    global boardSize

    global averageArea   #TODO: delete later, for testing only

    #SET UP VARIABLES
        #Coordinate math functions

    attempts            = attempts  #How many times the program has attempted to pick up this piece
    maxAttempts         = 3

    #column  = 3
    #row     = 4

    #DETERMINE MATHEMATICALLY THE "STRETCH" DIST BETWEEN THE CAMERA AND THE SUCKER
        #GET [x,y] FORMAT COORDINATES FOR EACH CORNER OF THE CHECKERBOARD
    corners = cornerInfo['corners']
    bR = [float(corners[0]["x"]), float(corners[0]["y"])]
    bL = [float(corners[1]["x"]), float(corners[1]["y"])]
    tR = [float(corners[3]["x"]), float(corners[3]["y"])]
    tL = [float(corners[2]["x"]), float(corners[2]["y"])]
        #GET LENGTH OF EACH SIDE
    dist   = lambda  p1, p2: ((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2) ** .5
    avgLen            = (dist(bR, bL) + dist(tR, tL) + dist(bR, tR) + dist(bL, tL)) / 4
    stretchPerInch    = avgLen / boardLength
    stretchFromSucker = stretchPerInch * camDistFromGrabber


    #CHECK IF THIS FUNCTION HAS BEEN RUN RECURSIVELY MORE THAN THE SET LIMIT, AND QUIT IF IT HAS
    if attempts >= maxAttempts:
        print "pickUpPiece(", locals().get("args"), "): Too many recursive attempts. Raising error"
        raise Exception("PieceNotGrabbed")



    #GET ROBOT INTO POSITION AND CHECK IF THE OBJECT IS STILL THERE. QUIT IF IT IS NOT.
    Robot.moveTo(stretchDistFromBase=cornerInfo['distFromBase'], relative=False, **getSquarePosition(column, row, corners))
    sleep(.2)



    #BEGIN FOCUSING PROCESS UNTIL ROBOT IS AT finalHeight HEIGHT AND COMPLETELY FOCUSED ON OBJECT
    stretchBefore  = Robot.pos["stretch"]
    rotationBefore = Robot.pos['rotation']
    avgRadius      = 0
    settings = [{'height': cornerInfo['height'] + 15, 'jump': 0, 'focusTolerance': 2, 'wait':  0, 'avgSampleSize':  50, 'targetSamples':   8},
                {'height':      cornerInfo['height'], 'jump': 0, 'focusTolerance': 6, 'wait':  0, 'avgSampleSize':  70, 'targetSamples':   1},
                {'height': cornerInfo['height'] - 15, 'jump': 0, 'focusTolerance': 2, 'wait':  0, 'avgSampleSize': 100, 'targetSamples':  15}]
                #{'height':                    0, 'jump': 1, 'focusTolerance': 6, 'wait':  0, 'avgSampleSize':  30, 'targetSamples':  1},
                #{'height':                  -30, 'jump': 0, 'focusTolerance': 5, 'wait':  0, 'avgSampleSize': 100, 'targetSamples': 10}]


    for index, s in enumerate(settings):
        Robot.moveTo(height=s['height'], relative=False)
        sleep(.25)

        totalRadius = 0
        i = 0
        averageRadius = (screenDimensions[0]/10)  #Default value to start searching for the piece
        searchTolerance = 0.99  #It will search for the circle nearest to the center and find its average radius. This search tolerance will widen each time it doesn't find the target.
        while i < s['avgSampleSize']:
        #for i in range(s['avgSampleSize']):  #Get average radius for nearestCircle at this height (Not proud of the code here....)

            #GET THE AVERAGE RADIUS FOR THE CHECKER PIECE AT THIS HEIGHT-
            #THIS WILL BE USED TO BETTER RECOGNIZE IT DURING THE FOCUSONTARGET() FUNCTION

            getAllPieces = lambda: objTracker.getCircles(minRadius=avgRadius * searchTolerance, maxRadius=avgRadius / searchTolerance)

            try:
                newRadius = objTracker.bruteGetFrame(lambda: [objTracker.sortShapesByDistance(getAllPieces(), returnNearest=True).radius], maxAttempts=10)[0]
                totalRadius +=  newRadius
                #print 'avgRadius right now', newRadius

            except NameError as e:
                #Loosen the searchTolerance and try again
                print 'pickUpPiece(): ERROR: ', e, ' Could not find target when getting avgRadius. Curr. iteration: ', i, ' Curr. avgRadius: ', totalRadius / (i + 1), 'Curr. searchTolerance: ', searchTolerance
                i -= 1
                searchTolerance *= .9

            #WAIT FOR NEW FRAME
            lastFrame = vid.frameCount
            while lastFrame == vid.frameCount: pass

            i += 1

        avgRadius = totalRadius / s['avgSampleSize']
        averageArea = avgRadius
        print "pickUpPiece(): Updated avgRadius: ", avgRadius



        #SET UP THE CHECKER-PIECE FINDING ALGORITHMS FOR FOCUSONTARGET()
        getAllPiecesAccurate = lambda: objTracker.getCircles(minRadius=avgRadius * .8, maxRadius=avgRadius / .8)
        getNearestPiece      = lambda: objTracker.bruteGetFrame(lambda: objTracker.sortShapesByDistance(getAllPiecesAccurate(), returnNearest=True).center, maxAttempts=100)

        focusOnTarget(getNearestPiece, tolerance=s['focusTolerance'], jump=s['jump'], wait=s['wait'], targetSamples = s['targetSamples'])


    lastSeenPos       = Robot.getPosArgsCopy()
    pickupHeight      = groundHeightFormula(Robot.pos['stretch']) + 2  #The plus 2 makes it so the sucker isn't firmly pressed against the ground/piece, but more lightly so.

    print "pickupHeight = ", pickupHeight
    stretchAfter      = Robot.pos["stretch"]
    rotationAfter     = Robot.pos['rotation']
    stretchPerHeight  = (stretchBefore  - stretchAfter)  / (settings[0]['height'] - settings[-1]['height'])
    rotationPerHeight = (rotationBefore - rotationAfter) / (settings[0]['height'] - settings[-1]['height'])
    stretchAdjust     = (Robot.pos["height"] - pickupHeight) * stretchPerHeight
    rotationAdjust    = (Robot.pos["height"] - pickupHeight) * rotationPerHeight

    #NOW THAT CAMERA IS CENTERED ON OBJECT, JUMP OVER IT AND MOVE DOWN
    print "stretchBefore: ", stretchBefore, "stretchAfter: ", stretchAfter, "stretchPerHeight: ", stretchPerHeight
    print "rhBefore: ", rotationBefore, "rAfter: ", rotationAfter, "rPerHeight: ", rotationPerHeight

    sleep(.1)

    print "Moving an adjuststretch of: ", stretchAdjust, "Moving an adjustRotate of: ", rotationAdjust
    #Robot.moveTo(stretch=stretchFromSucker + stretchAdjust, rotationAdjust=rotationAdjust)
    Robot.moveTo(stretch=stretchFromSucker, rotation=rotationAdjust * 2)

    print "moving height"
    sleep(.1)
    Robot.moveTo(height=pickupHeight, relative=False)


    #PICK UP THE PIECE
    pickupLocation = Robot.getPosArgsCopy(onlyRecord=['rotation', 'stretch'])
    Robot.setGrabber(True)
    sleep(.5)


    #CHECK TO SEE IF THE PIECE HAS BEEN PICKED UP
    Robot.moveTo(height=20)
    sleep(.1)
    Robot.moveTo(relative=False, **lastSeenPos)
    sleep(1)

    getAllPiecesAccurate = lambda: objTracker.getCircles(minRadius=avgRadius * .8, maxRadius=avgRadius / .8)
    getNearestPiece      = lambda: objTracker.bruteGetFrame(lambda: objTracker.sortShapesByDistance(getAllPiecesAccurate(), returnNearest=True).center, maxAttempts=10)
    try:  #See if there is a piece there
        getNearestPiece()
        print "pickUpPiece(): Failed to pick up piece"
    except NameError as e:  #If no piece is found, an error is thrown- meaning that the robot has successfully found the piece
        print "pickUpPiece(): Successfully picked up piece"

    return stretchFromSucker

def placePiece(columnFrom, rowFrom, columnTo, rowTo, cornerInfo, groundHeightFormula, stretchToSucker):
    # print "From: ", columnFrom, rowFrom, " To:", columnTo, rowTo, " pickupLocation: ", pickupLocationPolar
    #
    # corners = cornerInfo['corners']
    #
    # print 'Corners: ', corners
    # pickupLocXY = {'x': 0, 'y': 0}
    # pickupLocXY['x'], pickupLocXY['y'] = Robot.convertToCartesian(pickupLocationPolar['rotation'], pickupLocationPolar['stretch'], cornerInfo['distFromBase'])
    #
    # print "newPickupLocation: ", pickupLocXY
    #
    # posFromCalc = getSquarePosition(columnFrom, rowFrom, corners)
    # posToCalc   = getSquarePosition(columnTo,     rowTo, corners)
    #
    # print 'posFrom calculated: ', posFromCalc
    # print 'posTo calculated: ',   posToCalc
    #
    # toFromDiff = {'x': posToCalc['x'] - posFromCalc['x'], 'y': posToCalc['y'] - posFromCalc['y']}
    #
    # print 'toFromDiff: ', toFromDiff
    #
    # adjustedMove = {'x': toFromDiff['x'] + pickupLocXY['x'], 'y': toFromDiff['y'] + pickupLocXY['y']}
    # print 'adjustedMove: ', adjustedMove

    print " To:", columnTo, rowTo, " stretchToSucker: ", stretchToSucker

    corners = cornerInfo['corners']

    print 'Corners: ', corners

    posToCalc   = getSquarePosition(columnTo,     rowTo, corners)

    print 'posTo calculated: ',   posToCalc



    #DROP PIECE IN CORRECT LOCATION
    Robot.moveTo(height=0, relative=False)
    sleep(.1)
    Robot.moveTo(stretchDistFromBase=cornerInfo['distFromBase'], relative=False, **posToCalc)
    Robot.moveTo(stretch=stretchToSucker)
    sleep(.5)

    dropOffHeight = groundHeightFormula(Robot.pos['stretch']) + 2
    print "dropoff height: ", dropOffHeight

    Robot.moveTo(height = (dropOffHeight * 3) / 5)
    sleep(.2)
    Robot.moveTo(height = (dropOffHeight * 1) / 5)
    sleep(.2)
    Robot.moveTo(height = (dropOffHeight * 1) / 5)
    sleep(.2)
    Robot.setGrabber(0)
    Robot.moveTo(height = 5)
    sleep(.1)
    Robot.moveTo(height = 10)
    sleep(.1)
    Robot.moveTo(height = 15)
    sleep(.1)
    Robot.setGrabber(False)

def getPlacementLocation(column, row, corners, groundHeightFormula):
    """
    This function finds and focuses on an empty square at (column, row) that the robot will be
    moving the piece to later on. It is usefull because that way the robot can later pick up the
    piece and immediately move to the correct placement location, minimizing the time that the
    robots pump is turned on.

    :param column:
    :param row:
    :param cornersLow:
    :return:
    """

    global camDistFromGrabber
    global boardLength
    global boardSize
    global averageArea   #TODO: delete later, for testing only

    #SET UP VARIABLES
        #Coordinate math functions
    dist   = lambda  p1, p2: ((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2) ** .5
    ptDist = lambda d, a, b: [a[0] + ((b[0] - a[0]) / dist(a, b)) * d, a[1] + ((b[1] - a[1]) / dist(a, b)) * d]


    #DETERMINE MATHEMATICALLY THE "STRETCH" DIST BETWEEN THE CAMERA AND THE SUCKER
        #GET [x,y] FORMAT COORDINATES FOR EACH CORNER OF THE CHECKERBOARD
    bR = [float(corners[0]["x"]), float(corners[0]["y"])]
    bL = [float(corners[1]["x"]), float(corners[1]["y"])]
    tR = [float(corners[3]["x"]), float(corners[3]["y"])]
    tL = [float(corners[2]["x"]), float(corners[2]["y"])]
        #GET LENGTH OF EACH SIDE
    avgLen            = (dist(bR, bL) + dist(tR, tL) + dist(bR, tR) + dist(bL, tL)) / 4
    stretchPerInch    = avgLen / boardLength
    stretchFromSucker = stretchPerInch * camDistFromGrabber

    print    "stretchPerInch: ", stretchPerInch
    print "stretchFromSucker: ", stretchFromSucker


    #GET ROBOT INTO POSITION OVER SQUARE, HIGH UP
    Robot.moveTo(relative=False, **getSquarePosition(column, row, corners))
    sleep(.2)



    #BEGIN FOCUSING PROCESS UNTIL ROBOT IS AT finalHeight HEIGHT AND COMPLETELY FOCUSED ON OBJECT
    stretchBefore  = Robot.pos["stretch"]
    rotationBefore = Robot.pos['rotation']
    #avgArea        = 0
    settings = [#{'height': 150, 'jump': 4, 'focusTolerance': 8, 'wait': 0,  'avgSampleSize':  100, 'targetSamples': 1, 'maxDist':400},
                {'height':  0, 'jump': 3, 'focusTolerance': 15, 'wait': 0,  'avgSampleSize':  100, 'targetSamples': 1, 'maxDist': 300},
                #{'height':  -15, 'jump': 0, 'focusTolerance': 10, 'wait': 0,  'avgSampleSize':  100, 'targetSamples': 1, 'maxDist':350},
                {'height': -30, 'jump': 0, 'focusTolerance': 8, 'wait': .1, 'avgSampleSize':  100, 'targetSamples': 1, 'maxDist': 300}]

    for index, s in enumerate(settings):
        Robot.moveTo(height=s['height'], relative=False)
        sleep(.25)

        #GET THE AVERAGE RADIUS FOR THE CHECKER PIECE AT THIS HEIGHT-
        #THIS WILL BE USED TO BETTER RECOGNIZE IT DURING THE FOCUSONTARGET() FUNCTION
        # if avgArea == 0:
        #     #getAllPiecesAccurate = lambda: objTracker.getShapes(4, minArea=averageArea * .1, maxArea=averageArea * (1 / .1), bilateralConstant=1)
        #       #  getNearestPiece      = lambda: [objTracker.sortShapesByDistance(getAllPiecesAccurate(), returnNearest=True, deleteSimilar=False)]
        #
        #     getAllPieces = lambda: objTracker.getShapes(4, minArea=900 * .5, maxArea= 900 * (1 / .5))
        # else:
        #     getAllPieces = lambda: objTracker.getShapes(4, minArea=avgArea * .7, maxArea=avgArea * (1 / .7))

        #GET AVERAGE AREA FOR SQUARE IN THE SPOT RIGHT NOW, TO FINE TUNE TRACKING
        # totalArea = 0
        # for i in range(s['avgSampleSize']):
        #    totalArea += objTracker.bruteGetFrame(lambda: [objTracker.sortShapesByDistance(getAllPieces(), returnNearest=True).area], maxAttempts=100)[0]
        #    print 'avgArea right now', totalArea / (i + 1)
        # avgArea   = totalArea / s['avgSampleSize']
        # averageArea = avgArea  #TODO: DELETE LATER, USES GLOBAL VARIABLES TO COMMUNICATE WITH MAIN THREAD

        #print "getPlacementLocation(): Updated avgArea: ", avgArea

        #SET UP THE CHECKER-PIECE FINDING ALGORITHMS FOR FOCUSONTARGET()
        getAllSquares        = lambda: objTracker.getShapes(4)  #, minArea=avgArea * .8, maxArea=avgArea * (1 / .8))
        getNearestPiece      = lambda: objTracker.bruteGetFrame(lambda: objTracker.sortShapesByDistance(getAllSquares(), returnNearest=True, maxDist=300).center, maxAttempts=100)

        focusOnTarget(getNearestPiece, tolerance=s['focusTolerance'], jump=s['jump'], wait=s['wait'], targetSamples = s['targetSamples'])


    lastSeenPos       = Robot.getPosArgsCopy()
    pickupHeight      = groundHeightFormula(Robot.pos['stretch'])
    print "pickupHeight = ", pickupHeight
    stretchAfter      = Robot.pos["stretch"]
    rotationAfter     = Robot.pos['rotation']
    stretchPerHeight  = (stretchAfter  - stretchBefore)  / (settings[0]['height'] + settings[-1]['height'])
    rotationPerHeight = (rotationAfter - rotationBefore) / (settings[0]['height'] + settings[-1]['height'])
    stretchAdjust     = -(Robot.pos["height"] - pickupHeight) * stretchPerHeight
    rotationAdjust    = (Robot.pos["height"] - pickupHeight) * rotationPerHeight

    #NOW THAT CAMERA IS CENTERED ON OBJECT, JUMP OVER IT AND MOVE DOWN
    print "stretchBefore: ", stretchBefore, "stretchAfter: ", stretchAfter, "stretchPerHeight: ", stretchPerHeight
    print "rhBefore: ", rotationBefore, "rAfter: ", rotationAfter, "rPerHeight: ", rotationPerHeight
    print "Moving an adjuststretch of: ", stretchAdjust, "Moving an adjustRotate of: ", rotationAdjust
    sleep(.1)
    placementLocation = Robot.getPosArgsCopy()
    placementLocation['stretch']  += stretchFromSucker + stretchAdjust
    placementLocation['rotation'] += rotationAdjust
    placementLocation['height']    = pickupHeight + 30


    print "getPlacementLocation(): Location: ", placementLocation
    Robot.moveTo(relative=False, **placementLocation)
    Robot.moveTo(height=-25)
    sleep(5)

    return placementLocation


#   #CALIBRATION FUNCTIONS
def getBoardCorners():
    """
        Get the robots cartesian locations centered on each corner of the board in this format:
    cornerInfo = [{'corners': [{'y': -33.77319218164843,  'x': 173.74801147023652}, {'y': 120.71486133425334, 'x': 138.86656276099006}, {'y': 154.45614415655808, 'x': 290.489758050587},   {'y': 16.956849822713803, 'x': 323.5559692604819}],  'distFromBase': 144, 'height': 100},
                  {'corners': [{'y': -33.77319218164843,  'x': 173.74801147023652}, {'y': 114.53631117107041, 'x': 141.4405649851687},  {'y': 153.51720103098629, 'x': 288.72386286486915}, {'y':               -0.0, 'x': 324.0}],              'distFromBase': 158, 'height': 40},
                  {'corners': [{'y': -35.87209113079042,  'x': 184.5459104881608},  {'y': 124.05800866950207, 'x': 147.84657752196276}, {'y': 156.62672241014363, 'x': 307.3972508449869},  {'y': 5.9512705951136775, 'x': 340.9480640483294}],  'distFromBase': 181, 'height': 8},
                  {'corners': [{'y': -37.207754098426236, 'x': 191.41730077229448}, {'y': 129.2003095469944,  'x': 153.9749330669146},  {'y': 162.98258940649728, 'x': 319.87134218362405}, {'y': 6.195604285235647,  'x': 354.9459317805189}],  'distFromBase': 195, 'height': -4},
                  {'corners': [{'y': -36.062900126166966, 'x': 185.52753767160848}, {'y': 125.34358388887516, 'x': 149.3786664082007},  {'y': 163.37610384949,    'x': 307.26576231490657}, {'y': 6.055985033737379,  'x': 346.94715021926777}], 'distFromBase': 189, 'height': -13}]


        This function allows the robot to find the corners of the board by looking for the tiny square markers.
        It first finds the bottom right corner, where it looks for 4 squares of similar sizes. Now that it knows
        the approximate size of the squares, it is able to find the rest of the corners with ease, by looking for
        squares of the same shape.
    :param vid:
    :return:
    """

    global streamVideo
    streamVideo = True


    #This defines the area where the robot will look for corners of the board. TODO: move these constants somewhere in the main program, or prompt user for them
    ## It will then find the marker and refine that to an exact location.
    searchPositions = [{'stretch':  29, 'rotation':  13},
                       {'stretch':  32, 'rotation': -37},
                       {'stretch': 176, 'rotation': -26},
                       {'stretch': 173, 'rotation':   0}]


    #This defines the settings the robot should use when targeting the markers at different heights.
    setting = [{'height': 100, 'jump': 2, 'tolerance': 6, 'wait': .1},
               {'height':  40, 'jump': 1, 'tolerance': 7, 'wait': 0},
               {'height':   8, 'jump': 0, 'tolerance': 6, 'wait': 0},
               {'height':  -6, 'jump': 0, 'tolerance': 5, 'wait': 0},
               {'height': -15, 'jump': 0, 'tolerance': 4, 'wait': 0}]

    tolerance       = 0.3                       #Find 4 shapes (a marker) that are [tolerance]% different
    cornerPositions = [{'corners': [], 'height': 0, 'distFromBase':0} for i in range(0, len(setting))]            #Corner positions at each height. There are 4 heights, thus 4 sets of 4 corners.
    cornerPosPolar  = [{'corners': [], 'height': 0, 'distFromBase':0} for i in range(0, len(setting))]
    markerArea      = {}                        #The area of the marker square at different heights



    #GET THE POSITION OF EACH CORNER AT EACH HEIGHT, AND FILL OUT cornerPosPolar
    for index, position in enumerate(searchPositions):  #For each corner
        Robot.moveTo(relative = False, **searchPositions[index])
        sleep(.25)

        #cornerPositions.append({'corners': [], 'height': 0})
        #cornerPosPolar.append( {'corners': [], 'height': 0})

        for i, s in enumerate(setting):  #For each height find the marker, focus on it, and record the position of the corners
            Robot.moveTo(height=s['height'], relative=False)
            sleep(1)

            markerArea[s['height']] = getMarkerPerimeter(tolerance)

            markerCoordinate = lambda: getMarker(markerArea[s['height']], tolerance)
            focusOnTarget(markerCoordinate, tolerance=s['tolerance'], jump=s['jump'], wait=s['wait'])

            cornerPosPolar[i]['height'] = s['height']
            cornerPosPolar[i]['corners'].append(Robot.getPosArgsCopy(onlyRecord=["rotation", "stretch"]))



    #CONVERT CORNERPOSPOLAR TO CARTESIAN BY FILLING OUT CORNERPOSITIONS
    for index, height in enumerate(cornerPosPolar):
        cornerPositions[index]['height']       = cornerPosPolar[index]['height']
        cornerPositions[index]['distFromBase'] = getStretchFromBase(cornerPosPolar[index]['corners'])

        for i, corner in enumerate(cornerPosPolar[index]['corners']):
            cartesian = Robot.convertToCartesian(corner['rotation'], corner['stretch'], cornerPositions[index]['distFromBase'])
            cartesianDictionary = {'x': cartesian[0], 'y': cartesian[1]}
            cornerPositions[index]['corners'].append(cartesianDictionary)


    print "getBoardCorners(): Polar Corners: ", cornerPosPolar
    print "getBoardCorners(): Corners: ", cornerPositions

    return cornerPositions

def getStretchFromBase(polarCorners):
    """
    The purpose of this function is difficult to explain, but I'll give it a try. A problem I ran into with cartesian coordinates was that the uArm's 0 stretch point
    was not exactly 0 distance from the pivot of the robot. To find the exact distance in "stretch" units, I modified the Robot.convertCartesian functions to use a
    distFromBase and add that on. However, this distance from the base must be found- and even worse, its different at different heights!!!

    How to fix this? You use the checker board corners. We know the corners at various heights, in 'stretch, rotation' form. We know that the checkerboard is
    completely square, thus the cartesian coordinates must reflect this fact. By testing many values (0 to 250) of possible distFromBase and comparing how
    'square' the cartesian coordinates are, you can find the optimal distFromBase.

    This function returns this optimal distFromBase for any set of corners at a certain height.

    polarCorners should be in this format:
        polarCorners = [{'stretch':  0, 'rotation': 12}, {'stretch':  6, 'rotation': -39}, {'stretch': 157, 'rotation': -27}, {'stretch': 157, 'rotation': 1}]

        The values will be different, of course.
    """

    #polarCorners = [{'stretch':  0, 'rotation': 12}, {'stretch':  6, 'rotation': -39}, {'stretch': 157, 'rotation': -27}, {'stretch': 157, 'rotation': 1}]

                    #[{'stretch':  0, 'rotation': 12}, {'stretch':  6, 'rotation': -39}, {'stretch': 157, 'rotation': -27}, {'stretch': 157, 'rotation': 1}]

    dist   = lambda p1, p2: ((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2) ** .5

    #GET [rotation,stretch] FORMAT COORDINATES FOR EACH CORNER OF THE CHECKERBOARD
    bR = [float(polarCorners[0]["rotation"]), float(polarCorners[0]["stretch"])]
    bL = [float(polarCorners[1]["rotation"]), float(polarCorners[1]["stretch"])]
    tR = [float(polarCorners[3]["rotation"]), float(polarCorners[3]["stretch"])]
    tL = [float(polarCorners[2]["rotation"]), float(polarCorners[2]["stretch"])]

    bestDist = 0      #Best value of testDist so far
    bestDev  = 10000  #Lowest standard deviation so far

    for testDist in range(0, 250):  #Try each stretch value from 0 to 200 to see which one will make each side equal to eachother in polar coordinates

        bRCart = list(Robot.convertToCartesian(bR[0], bR[1], testDist))
        bLCart = list(Robot.convertToCartesian(bL[0], bL[1], testDist))
        tRCart = list(Robot.convertToCartesian(tR[0], tR[1], testDist))
        tLCart = list(Robot.convertToCartesian(tL[0], tL[1], testDist))

        bRbL = dist(bRCart, bLCart)
        bLtL = dist(bLCart, tLCart)
        tLtR = dist(tLCart, tRCart)
        tRbR = dist(tRCart, bRCart)

        stDev = np.std([bRbL, bLtL, tLtR, tRbR], axis=0)

        if stDev < bestDev:
            bestDev  = stDev
            bestDist = testDist
            #print "Distances of each side with testDist of ", testDist, " : ", bRbL, bLtL, tLtR, tRbR, "stDev: ", stDev

    return bestDist

def getGroundFormula(cornersLow):
    """
    Move the robot to the bottom of the board and move down until the robots touch sensor triggers
    and then record that spot. Do the same for the top of the board. Use this information to calculate
    the "height" of the ground anywhere along the board, to be used when picking up pieces.

    The function returns another lambda function, which has one parameter: stretch, and returns one parameter: height
    :return:
    """
    groundHeight = {'bottom': {'stretch': 0, 'height': 0}, 'top': {'stretch': 0, 'height': 0}}  #Ground height at the bottom and top of the board

    #Location of the top middle of the board and the bottom middle of the board
    heightPositions = [getSquarePosition(2.5, 5.5, cornersLow),
                       getSquarePosition(2.5, -.5, cornersLow)]

    Robot.moveTo(relative=False, height=0, **heightPositions[0])
    sleep(.2)
    while Robot.pos['height'] > -45 and Robot.getOutput('touch')['touch']:
        Robot.moveTo(height = -5)
        sleep(.2)
    groundHeight['bottom']['stretch'] = Robot.pos['stretch']
    groundHeight['bottom']['height']  = Robot.pos['height']




    Robot.moveTo(height=0, relative = False)
    sleep(.1)
    Robot.moveTo(relative=False, **heightPositions[1])
    sleep(.1)
    while Robot.pos['height'] > -75 and Robot.getOutput('touch')['touch']:
        Robot.moveTo(height = -7.5)
        sleep(.2)

    groundHeight['top']['stretch'] = Robot.pos['stretch']
    groundHeight['top']['height']  = Robot.pos['height']

    Robot.moveTo(height=0, relative=False)
    sleep(.1)
    Robot.moveTo(stretch = 0, relative=False)
    sleep(.5)

    slope = (groundHeight['top']['height'] - groundHeight['bottom']['height']) / (groundHeight['top']['stretch'] - groundHeight['bottom']['stretch'])
    intercept = groundHeight['bottom']['height'] - slope * groundHeight['bottom']['stretch']

    print "getGroundFormula(): GroundHeight: ", groundHeight
    print "getGroundFormula(): Final formula:  lambda stretch:", slope, " * stretch +", intercept
    return lambda stretch: slope * stretch + intercept

def getAverageColor(circleArray):
    """
    The pieces on the board are determined to be one side or another by their color. Since color can be be read differently
    in different lightings, this function can be used to calibrate at the start of the game.

    What it does is it takes the average color of each circle and returns it. This is then used to find the "midpoint"
    for determining if one piece is from player A or player B. It must only be run when there is an equal number
    of each type of piece on the board, or it will skew the results.
    """
    avgColor = [0, 0, 0]
    for circle in circleArray:
        avgColor = [avgColor[0] + circle.color[0],
                    avgColor[1] + circle.color[1],
                    avgColor[2] +  circle.color[2]]
    avgColor = [avgColor[0] / len(circleArray), avgColor[1] / len(circleArray), avgColor[2] / len(circleArray)]
    print "getAverageColor(): ", avgColor
    return avgColor

def getMarkerPerimeter(tolerance, **kwargs):
    """
    This function searches for 4 nearby squares that are of similar size.
    There are 4 squares at each corner, and they represent "markers".
    The key to identifying a marker is knowing the pixel area that it takes up, otherwise its
    possible to confuse the big checkerboard squares as markers, despite how much bigger they are.
    This function will find the smalles 4 similar squares and return their average area.
    :return:
    """
    #compareRange = kwargs.get('compareRange', True)
    accuracyAttempts  = kwargs.get('accuracyAttempts', 50)
    attempts = 0

    #print "CompareRange: ", compareRange
    #  TODO: Make sure that if it doesn't see 4 small blocks, that it doesn't confuse the big ones as the markers

    global streamVideo
    streamVideo = False

    avgPerimeter    = 0    #The average pixel area of the markers.
    similarShapeSets = []  #Records sets of 4 shapes that are similar in size, later to be whittled down more.

    while avgPerimeter == 0:
        shapeArray = objTracker.getShapes(4, minArea = 500, maxArea = (vid.frame.shape[0] * vid.frame.shape[1]) / 3)[::-1]
        shapeArray = sorted(shapeArray, key = lambda s: s.perimeter)
        shapeArrayCopy = shapeArray[:]


        for shapeIndex, shape in enumerate(shapeArray):
            shapeArrayCompare = shapeArray[:shapeIndex] + shapeArray[(shapeIndex + 1):]  #Get shapeArray without the current shape being analyzed
            similarShapes     = [shape]
            for compareIndex, shapeToCompare in enumerate(shapeArrayCompare):
                if shape.perimeter - shape.perimeter * tolerance < shapeToCompare.perimeter < shape.perimeter + shape.perimeter * tolerance:
                    similarShapes.append(shapeToCompare)


            if len(similarShapes) == 4:  #If more than 3 shapes have been found of similar size
                similarShapeSets.append(similarShapes)

            #print "similar: ",    similarShapes, "\n"

        vid.windowFrame["Main"] = objTracker.drawShapes(shapeArray)

        if len(similarShapeSets) >= 1:
            similarShapeSets = sorted(similarShapeSets, key = lambda ss: sum(ss[i].perimeter for i in range(len(ss))) / len(ss))  #Sort by perimeter smallest to largest

            sizeRange        = shapeArray[-1].perimeter / (sum(similarShapeSets[0][i].perimeter for i in range(len(similarShapeSets[0]))) / len(similarShapeSets[-1]))  #How much smaller the last similarShapeSet is compared to the first one

            #print "shapeArray: ", shapeArray, "\n"
            #print "similarShapeSets Sorted", similarShapeSets,
            #print "Difference between areas: ", sizeRange, "\n"

            if len(similarShapeSets[0]) == 4 and (sizeRange > 1.5 or attempts > accuracyAttempts):  #if have passed accuracyAttempts, stop trying to compare the size
                avgPerimeter = sum([s.perimeter for i, s in enumerate(similarShapeSets[0])]) / len(similarShapeSets[0])
                print "getMarkerPerimeter(): avgPerimeter of markers found to be: ", avgPerimeter

        if avgPerimeter == 0:  #If the 4 markers were never identified
            #print "getMarkerPerimeter(): ERROR: Marker not found! Trying again..."
            #WAIT FOR NEW FRAME
            lastFrame = vid.frameCount
            while lastFrame == vid.frameCount:
                pass

        attempts += 1

    streamVideo = True

    return avgPerimeter

def getMarker(avgPerimeter, tolerance, **kwargs):


    maxAttempts = kwargs.get("maxAttempts", 100)

    squaresInMarker = 4  #The camera must detect exactly 4 squares of the same size to consider it to be a marker
    attempts = 0

    shapes = []
    while attempts <= maxAttempts:  #Find at least 4 squares of the correct shape and size

        shapes = objTracker.getShapes(4, minPerimeter = avgPerimeter - avgPerimeter * tolerance * 2, maxPerimeter = avgPerimeter + avgPerimeter * tolerance * 2)

        shapes = objTracker.sortShapesByDistance(shapes)[:4]

        markerCentroid = tuple(objTracker.getShapesCentroid(shapes))

        #shapes = objTracker.getShapes(4, minArea = avgArea - avgArea * tolerance * 2, maxArea = avgArea + avgArea * tolerance * 2)
        attempts += 1

        #IF NOT SUCCESSFULL, PULL ANOTHER FRAME
        if len(shapes) < squaresInMarker:

            if attempts != maxAttempts:
                lastFrame = vid.frameCount
                while lastFrame == vid.frameCount:
                    pass
            else:
                print "getMarker:() ERROR: ATTEMPTS OVER ", maxAttempts, " AT HEIGHT", Robot.pos['height']
                raise NameError("ObjNotFound")
        else:
            break  #IF it found the marker, leave the loop

    #Shorten the list to the squares closest to center screen (Most likely to be relevant markers)
    shapes = objTracker.sortShapesByDistance(shapes)[:4]
    cv2.imshow('lol', objTracker.drawShapes(shapes))
    cv2.waitKey(1)
    markerCentroid = tuple(objTracker.getShapesCentroid(shapes))

    if len(markerCentroid) == 0:
        print "getMarker:(): ERROR: No marker centroid detected"
        raise NameError("ObjNotFound")

    return markerCentroid








########### MAIN THREADS ###########
def runRobot():
    #SET UP VARIABLES AND RESET ROBOT POSITION
    print 'runRobot(', locals().get('args'), '): Setting up Robot Thread....'
    #global exitApp
    global keyPressed  #Is set in the main function, because CV2 can only read keypresses in the main function. Is a character.

    Robot.moveTo(relative = False, waitForRobot = True, **Robot.home)
    Robot.setGrabber(0)

    playCheckers()

    #MAIN ROBOT FUNCTION
    while not exitApp:

        if keyPressed == 'h':  #HELP COMMAND
            print 'Help: \n x: MOVETOXY'


        if keyPressed == 'x':  #MOVE TO XY
            #Robot.moveTo(height = float(raw_input('Height?:')), rotation = float(raw_input('Rotation?:')), stretch = float(raw_input('Stretch:')), relative = False)
            #Robot.moveTo(height = 150, rotation = float(raw_input('Rotation?:')), stretch = float(raw_input('Stretch:')), relative = False)
            #Robot.moveTo(height = 150, stretch = float(raw_input('Stretch:')), relative = False)
            #Robot.moveTo(x=float(raw_input('X?: ')), y=float(raw_input('Y?: ')), relative=False)
            Robot.moveTo(stretch=float(raw_input('Stretch?:')))
            #Robot.moveTo(height=float(raw_input("height?:")), relative=False)

        if keyPressed == 'p':  #PLAY CHECKERS
            playCheckers()

        if keyPressed == 'c':  #PLAY CHECKERS
            print getSquarePosition(int(raw_input("Column?")), int(raw_input("Row?")), [{'y': -22, 'x': 109}, {'y': 72, 'x': 89}, {'y': 136, 'x': 237}, {'y': 2, 'x': 270}])

if '__main__' == __name__:
    print 'Start!'
    #SET UP VIDEO CLASS AND WINDOWS VARIABLES
    vid                 = Vision.Video()
    vid.createNewWindow('Main',             xPos = 1980,   yPos = 10)
    vid.createNewWindow('Perspective',      xPos = 2800, yPos = 10)
    sleep(3)
    vid.getVideo()      #Get the first few frames
    vid.setCamResolution(1000, 1000)  #Sets the resolution as high as the camera allows


    #SETUP UP OTHER VARIABLES/CLASSES
    objTracker          = Vision.ObjectTracker(vid, rectSelectorWindow = 'KeyPoints')
    screenDimensions    = vid.getDimensions()
    keyPressed          = ''   #Keeps track of the latest key pressed

    global averageArea  #TODO: DELETE NOW
    averageArea = 1000

    global exitApp
    global streamVideo         #RobotThread communicates using this. It tells main thread to stream video onto window or to hold on a particular frame.
    global camDistFromGrabber
    global boardLength
    global boardSize

    camDistFromGrabber = 1.45  #(inches) Horizontal inches from the camera to the sucker of the robot. Used later for picking up checker pieces  #TODO: put in setup somewhere
    boardLength        = 6.5   #(inches) Side length of board in inches
    boardSize          = 6     #Squares per side of board

    exitApp            = False
    streamVideo        = True  #If true, then the camera will stream video. If not, then the camera will wait for new frames to be called for from the runCheckers() thread


    #START SEPERATE THREAD FOR MOVING ROBOT
    robotThread = Thread(target = runRobot)
    robotThread.start()


    #DISPLAY VIDEO/IMAGE REC. MAIN THREAD.
    while not exitApp:
        vid.getVideo()


        #DETECT IF CAMERA HAS BEEN UNPLUGGED, AND IF SO TRY TO RECONNECT EVERYTHING:
        #if objTracker.getMovement() == 0:
        #    print 'ERROR: __main__(XXX): Camera NOT detected.'
        #    sleep(1)
        #    print '__main(XXX)__: Attempting to reconnect...'
        #    vid.cap = cv2.VideoCapture(1)

        if streamVideo:
            markedUpFrame = vid.frame.copy()
            if True:
                getAllPieces    = lambda: objTracker.getCircles(minRadius = averageArea * .85, maxRadius = averageArea / .85)
                getNearestPiece = lambda: [objTracker.sortShapesByDistance(getAllPieces(), returnNearest = True)]
                cv2.circle(markedUpFrame, (int(screenDimensions[0] / 2), int(screenDimensions[1] / 2)), 6, (0, 255, 0), 3, 255)  #Draw center of screen
                markedUpFrame = objTracker.drawCircles(getNearestPiece(), frameToDraw = markedUpFrame)


            if False:
                getAllSquares   = lambda: objTracker.getShapes(4)  #, minArea=averageArea * .8, maxArea=averageArea * (1 / .8), bilateralConstant=1)
                getNearestPiece = lambda: [objTracker.sortShapesByDistance(getAllSquares(), returnNearest=True, maxDist=300)]
                cv2.circle(markedUpFrame, (int(screenDimensions[0] / 2), int(screenDimensions[1] / 2)), 6, (0, 255, 0), 3, 255)  #Draw center of screen
                markedUpFrame = objTracker.drawShapes(getNearestPiece(), frameToDraw = markedUpFrame)
                #markedUpFrame = objTracker.drawShapes(getAllSquares(), frameToDraw=markedUpFrame)

            vid.windowFrame['Main'] = markedUpFrame





        #DISPLAY WHATEVER FRAME IS NEXT
        vid.display('Main')
        vid.display('Perspective')

        #RECORD KEYPRESSES AS A CHARACTER IN A STRING
        ch = cv2.waitKey(100)                                     #Wait between frames, and also check for keys pressed.
        keyPressed = chr(ch + (ch == -1) * 256).lower().strip()  #Convert ascii to character, and the (ch == -1)*256 is to fix a bug. Used mostly in runRobot() function
        if keyPressed == chr(27): exitApp = True                 #If escape has been pressed, close program
        if keyPressed == 'p':                                    #Pause and unpause when spacebar has been pressed
            vid.paused = not vid.paused
            print '__main(XXX)__: PAUSED: ', vid.paused


    #CLOSE EVERYTHING CORRECTLY
    robotThread.join(1)  #Ends the robot thread, waits 1 second to do so.
    Robot.setGrabber(1)
    cv2.destroyWindow('window')


print 'End!'