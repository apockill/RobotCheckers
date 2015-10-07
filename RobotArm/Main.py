import math
from time import sleep
from threading import Thread
import cv2
import Common
import Variables as V
import Robot                    #All robot related commands
from Video import Vision        #All camera and object-rec commands





#DEPRECATED FUNCTIONS (REPLACE SOON)
def pickUpTarget(target):
    print "pickUpTarget(target) THIS FUNCTION IS DEPRECATED, remove soon!"
    #SET UP
    currentPosition = Robot.getPosArgsCopy()  #Records position to return to after picking up object
    targetFocus = [screenDimensions[0] / 2 - V.pixFromCamToArm, screenDimensions[1] / 2]
    focusOnTarget(target, targetFocus = targetFocus)
    focusOnTargetManual(target, targetFocus = targetFocus)

    Robot.moveTo(height = (V.heightMax + 60) / 4, waitForRobot = True, relative = False)
    focusOnTargetManual(target, targetFocus = targetFocus)

    #DESCEND ONTO OBJECT
    while Robot.getOutput('touch')["touch"] == 1:
        if Robot.pos["height"]-V.heightMin > 30:
            Robot.moveTo(height = -30)
        else:
            Robot.moveTo(height = Robot.pos["height"] + V.heightMin)

    Robot.setGrabber(1)              #Grab
    Robot.moveTo(relative = False, **currentPosition)  #Return to original position

def focusOnTargetManual(target, **kwargs):  #Arguments: targetFocus (The pixel location where you want the target to be alligned. Default is the center of the screen
    print "focusOnTargetManual() THIS FUNCTION IS DEPRECATED, remove soon!"
    targetFocus = kwargs.get('targetFocus', [screenDimensions[0] / 2, screenDimensions[1] / 2])  #The XY value of WHERE on the screen we want the robot to focus on.
    moveIncrement = 3
    while True:  #Robot arm will slowly approach the target`
        coords = objTracker.getTargetCenter(target)
        if len(coords) == 0: continue  #DO NOT MOVE. Prevents empty coords from continuing.
        xMove = 0
        yMove = 0
        if not Common.isWithinTolerance(coords[0], targetFocus[0]):
            if coords[0] < targetFocus[0]: xMove   -= moveIncrement
            if coords[0] > targetFocus[0]: xMove   += moveIncrement
        if not Common.isWithinTolerance(coords[1], targetFocus[1]):
            if coords[1] < targetFocus[1]: yMove   -= moveIncrement
            if coords[1] > targetFocus[1]: yMove   += moveIncrement
        if Common.isWithinTolerance(coords[1], targetFocus[1]) and Common.isWithinTolerance(coords[0], targetFocus[0]):
            break  #Once the target has been focused on, quit
        Robot.moveTo(x = xMove, y = -yMove)





#GENERIC FUNCTIONS
def focusOnTarget(getTargetCoords, **kwargs):  #Arguments: targetFocus (The pixel location where you want the target to be alligned. Default is the center of the screen
    """
    :param getTargetCoords: FOCUSES ON TARGET
    :param kwargs:
        "targetFocus": (default [screenXDimension, screenYdimension]) HOLDS AN ARRAY THAT TELLS YOU WHERE ON THE SCREEN TO FOCUS THE OBJECT
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
    ppX         = kwargs.get("ppX", -1)  #Pixels per X move
    ppY         = kwargs.get("ppY", -1)  #Pixels per Y move
    targetFocus = kwargs.get('targetFocus', [screenDimensions[0] / 2, screenDimensions[1] / 2])  #The XY value of WHERE on the screen we want the robot to focus on.
    tolerance   = kwargs.get('tolerance',   V.targetTolerance)
    sign        = lambda x: (1, -1)[x < 0]  #sign(x) will return the sign of a number'
    moveCount   = 0  #The amount of moves that it took to focus on the object

    while True:  #Robot arm will slowly approach the target
        try:
            coords = getTargetCoords()
        except NameError as e:
            print "ERROR: focusOnTarget(", locals().get("args"), "): error (", e, "). Object not found. Leaving Function..."
            raise  #RE-Raise the exception for some other program to figure out

        distance = ((coords[0] - targetFocus[0]) ** 2 + (coords[1] - targetFocus[1]) ** 2) ** 0.5  #For debugging
        xMove = 0
        yMove = 0
        xDist = coords[0] - targetFocus[0]
        yDist = coords[1] - targetFocus[1]
        if abs(xDist) > .85 * tolerance:  #I am stricter on x tolerance, bc the robot has more x incriments for movement.
            xMove = sign(xDist)     + (xDist / ppX) * (abs(xDist) > tolerance and not ppX == -1)

        if abs(yDist) > tolerance:
            yMove = sign(yDist) * .5 + (yDist / ppY) * (abs(yDist) > tolerance and not ppY == -1)

        #print "focusOnTarget(", locals().get("args"), "): xMove: ", xMove, "yMove: ",  yMove
        if not (int(xMove) == 0 and int(yMove) == 0):
            moveCount += 1
            Robot.moveTo(rotation = yMove, stretch = xMove)
            if yDist < tolerance * 2 or xDist < tolerance * 1.25:  #Slow down a bit when approaching the target
                sleep(.4)
            else:
                sleep(.2)

            if Robot.pos["stretch"] == V.stretchMax or Robot.pos["stretch"] == V.stretchMin:
                print "focusOnTarget(", locals().get("args"), "): Object out of Stretching reach."
                break
            if Robot.pos["rotation"] == V.rotationMax or Robot.pos["rotation"] == V.rotationMin:
                print "focusOnTarget(", locals().get("args"), "): Object out of Rotating reach."
                break
            if moveCount >= 40:
                raise Exception("TooManyMoves")

        else:
            print "focusOnTarget(", locals().get("args"), "): Object Targeted in ", moveCount, "moves."
            break

def focusOnCoord(coords, **kwargs):  #Arguments: targetFocus (The pixel location where you want the target to be alligned. Default is the center of the screen
    targetFocus = kwargs.get('targetFocus', [screenDimensions[0] / 2, screenDimensions[1] / 2])
    if len(coords) == 0:
        print "Trouble focusing on coord:", coords
        return
    diffX = coords[0] - targetFocus[0]
    diffY = coords[1] - targetFocus[1]
    ratio = (Robot.pos['height'] + 60.0) / (V.heightMax + 60.0)
    #print Robot.pos['height']
    #ratio = 1  #for not, while I get THAT working...
    xMove = (diffX / (V.pixelsPerX ))
    yMove = (diffY / (V.pixelsPerY  ))
    print "Xmove: ", xMove, "Ymove: ", yMove
    Robot.moveTo(rotation = yMove/2, stretch = xMove*2, waitForRobot = True)
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
    acceptableMovement = kwargs.get("movement", 8)
    if acceptableMovement <= 5:
        print "waitTillStill(", locals().get("args"), "): It is unwise to set the movement parameter to < than 4. It is set at ", acceptableMovement
    timer = Common.Timer(1)  #Will wait one second max before continuing
    while objTracker.getMovement() > acceptableMovement:
        if timer.timeIsUp():
            print "waitTillStill(", locals().get("args"), "): Timer has timed out. Consider lowering acceptableMovement. Continuing...", objTracker.getMovement()
            break

    sleep(.1)

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
def stackJenga():  #Main Function
    print "Playing Jenga!"
    #SET UP VARIABLES AND RESET ROBOT POSITION
    blocks = (1, 0, 2)  #The order to place the blocks
    markerLKP = {"rotation": 30, "stretch": 100}  #Keep track of the "marker block"'s "Last Known Position" (LKP)
    searchPos = {'rotation': -15, 'stretch': V.stretchMax / 2.5, 'height': V.heightMax, 'wrist': 0}
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
    Robot.moveTo(height = V.heightMax)
    sleep(.3)

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
    print "pickUpBlock(", locals().get("args"), "): Attempt number: ", attempts
    if attempts == 3:  #If max attempts have been hit
        print "pickUpBlock(", locals().get("args"), "): Too many recursive attempts. Raising error"
        raise Exception("BlockNotGrabbed")


    #GET ROBOT INTO POSITION AND CHECK IF THE OBJECT IS STILL THERE. QUIT IF IT IS NOT.
    waitTillStill()
    Robot.setGrabber(0)
    Robot.moveTo(relative = False, **objectLastSeen)
    waitTillStill()
    sleep(.1)
    if len(objTracker.getShapes(4)) == 0:  #If no objects in view
        print "pickUpBlock(", locals().get("args"), "): No objects seen at start. Raising error..."
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

def isObjectGrabbed():
    currentWrist = Robot.pos["wrist"]
    Robot.moveTo(wrist = -40, relative = False)
    Robot.moveTo(wrist = 20, relative = False)
    sleep(.01)
    movement = objTracker.getMovement()

    Robot.moveTo(wrist = currentWrist, relative = False)
    print "isObjectGrabbed(", locals().get("args"), "): Movement = ", movement
    return movement

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
def checkersMain():
    pass




#MAIN THREADS
def runRobot():
    #SET UP VARIABLES AND RESET ROBOT POSITION
    print "runRobot(", locals().get("args"), "): Setting up Robot Thread...."
    global exitApp
    global keyPressed  #Is set in the main function, because CV2 can only read keypresses in the main function. Is a character.

    Robot.moveTo(relative = False, waitForRobot = True, **Robot.home)
    Robot.setGrabber(0)

    #MAIN ROBOT FUNCTION
    while not exitApp:

        if keyPressed == 'h':  #HELP COMMAND
            print "Help: \n x: MOVETOXY"


        if keyPressed == 'x':  #MOVE TO XY
            Robot.moveTo(height = float(raw_input("Height?:")), rotation = float(raw_input("Rotation?:")), stretch = float(raw_input("Stretch:")), relative = False)


if '__main__' == __name__:
    print 'Start!'
    #SET UP VIDEO CLASS AND WINDOWS VARIABLES
    vid                 = Vision.Video()
    vid.createNewWindow("Main",             xPos = 10,  yPos = 10)
    vid.createNewWindow("Perspective",      xPos = 500,  yPos = 10)

    #SETUP UP OTHER VARIABLES/CLASSES
    objTracker          = Vision.ObjectTracker(vid, rectSelectorWindow = "KeyPoints")
    screenDimensions    = vid.getDimensions()
    global exitApp
    exitApp = False
    keyPressed          = ''  #Keeps track of the latest key pressed

    #START SEPERATE THREAD FOR MOVING ROBOT
    robotThread = Thread(target = runRobot)
    robotThread.start()
    vid.getVideo()

    #DISPLAY VIDEO/IMAGE REC. MAIN THREAD.
    while not exitApp:
        vid.getVideo()

        #IF THE CAMERA HAS BEEN UNPLUGGED, RECONNECT EVERYTHING:
        if objTracker.getMovement() == 0:
            print "ERROR: __main__(XXX): Camera NOT detected."
            sleep(1)
            print "__main(XXX)__: Attempting to reconnect..."
            vid.cap = cv2.VideoCapture(1)


        #DO FRAME OPERATIONS:
        testFrame = cv2.imread("F:\Google Drive\Projects\Git Repositories\RobotStorage\RobotArm\stitched.png")

        shapeArray, edgedFrame = objTracker.getShapes(sides=4, threshHold = cv2.THRESH_OTSU, frameToAnalyze= testFrame, returnFrame = True)
        warped = objTracker.getTransform(shapeArray[0], frameToAnalyze=testFrame, transformHeight= 600, transformWidth=600)
        circleArray = objTracker.getCircles(frameToAnalyze = warped)


        #SET FRAMES FOR THE WINDOWS:
        vid.windowFrame["Main"]        = objTracker.drawShapes([shapeArray[0]], frameToDraw=testFrame)
        vid.windowFrame["Perspective"] = objTracker.drawCircles(circleArray,    frameToDraw=warped)

        #UPDATE WINDOWS
        vid.display("Main")
        vid.display("Perspective")

        ch = cv2.waitKey(10000)                                      #Wait between frames, and also check for keys pressed.

        keyPressed = chr(ch + (ch == -1) * 256).lower().strip()  #Convert ascii to character, and the (ch == -1)*256 is to fix a bug. Used mostly in runRobot() function
        if keyPressed == chr(27): exitApp = True                 #If escape has been pressed, close program
        if keyPressed == 'p':                                    #Pause and unpause when spacebar has been pressed
            vid.paused = not vid.paused
            print "__main(XXX)__: PAUSED: ", vid.paused


    #CLOSE EVERYTHING CORRECTLY
    robotThread.join(1000)  #Ends the robot thread, waits 1 second to do so.
    Robot.setGrabber(1)
    cv2.destroyWindow('window')


print 'End!'