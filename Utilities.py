import math
#from States import *
from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
import time

GOAL_WIDTH = 1900
FIELD_LENGTH = 10280
FIELD_WIDTH = 8240

boosts = [
    [3584, 0,0],
    [-3584, 0,0],
    [3072, 4096,0],
    [3072, -4096,0],
    [-3072, 4096,0],
    [-3072, -4096,0]
    ]


class renderCall:
    def __init__(self,_function,*args):
        self.function = _function
        self.args = args

    def run(self):
        self.function(self.args[0],self.args[1],self.args[2]())



class FlipStatus:
    def __init__(self):
        self.started = False
        self.flipStartedTimer = None
        self.flipDone = False

class Boost_obj:
    def __init__(self,location,bigBoost, spawned):
        self.location = Vector(location) #list of 3 coordinates
        self.bigBoost = bigBoost # bool indicating if it's a cannister or just a pad
        self.spawned = spawned  # bool indicating whether it's currently spawned


class physicsObject:
    def __init__(self):
        self.location = Vector([0, 0, 0])
        self.velocity = Vector([0, 0, 0])
        self.rotation = Vector([0, 0, 0])
        self.avelocity = Vector([0, 0, 0])
        self.local_location = Vector([0, 0, 0])
        self.boostLevel = 0
        self.team = -1
        self.matrix = []

class Vector:
    def __init__(self, content): #accepts list of float/int values
        self.data = content

    def __str__(self):
        return str(self.data)

    def __repr__(self):
        return str(self)

    def __len__(self):
        return len(self.data)

    def __getitem__(self, item):
        return self.data[item]

    def raiseLengthError(self,other, operation):
        raise ValueError(f"Tried to perform {operation} on 2 vectors of differing lengths")

    def __mul__(self, other):
        if len(self.data) == len(other.data):
            return Vector([self.data[i] * other[i] for i in range(len(other))])
        else:
            self.raiseLengthError(other,"multiplication")

    def __add__(self, other):
        if len(self.data) == len(other.data):
            return Vector([self.data[i] + other[i] for i in range(len(other))])
        else:
            self.raiseLengthError(other, "addition")

    def __sub__(self, other):
        if len(self.data) == len(other.data):
            return Vector([self.data[i] - other[i] for i in range(len(other))])
        else:
            self.raiseLengthError(other, "subtraction")

    def magnitude(self):
        return math.sqrt(sum([x*x for x in self]))

    def normalize(self):
        mag = self.magnitude()
        if mag != 0:
            return Vector([x/mag for x in self])
        else:
            return Vector([0 for _ in range(len(self.data))])

    def dotProduct(self,other):
        product = 0
        for i,j in zip(self,other):
            product += i*j
        return product

    def scale(self,scalar):
        return Vector([x*scalar for x in self.data])


    def correction_to(self, ideal):
        # The in-game axes are left handed, so use -x
        current_in_radians = math.atan2(self[1], -self[0])
        ideal_in_radians = math.atan2(ideal[1], -ideal[0])

        correction = ideal_in_radians - current_in_radians

        # Make sure we go the 'short way'
        if abs(correction) > math.pi:
            if correction < 0:
                correction += 2 * math.pi
            else:
                correction -= 2 * math.pi

        return correction


    def toList(self):
        return self.data


def demoMagic(agent):
    #find ideal target
    currentSpd = agent.getCurrentSpd()
    if currentSpd <1900:
        if agent.me.boostLevel <=0:
            agent.activeState.active = False
    e_goal = Vector([0, 5100 * -sign(agent.team), 200])
    best = None
    distance = math.inf
    for e in agent.enemies:
        if e.location[2] <= 120:
            _distance = distance2D(e_goal,e.location)
            if _distance < distance:
                distance = _distance
                best = e

    if best != None:
        if currentSpd <=100:
            currentSpd = 100 #just to help avoid rediculous predictions

        currentTimeToTarget = distance / currentSpd
        lead = clamp(agent.deltaTime*60,agent.deltaTime*5,agent.deltaTime*distance/500)
        #print(f"{agent.deltaTime*60} {agent.deltaTime*5} {agent.deltaTime*distance/500}")
        difference = best.velocity.scale(lead)
        targetPos = e.location + difference
        agent.renderCalls.append(
            renderCall(agent.renderer.draw_line_3d, agent.me.location.data, targetPos.data, agent.renderer.purple))

        if currentTimeToTarget <= agent.deltaTime*30:
            targetLocal = toLocal(targetPos,agent.me)
            angle = math.degrees(math.atan2(targetLocal[1],targetLocal[0]))
            if abs(angle) <= 40:
                agent.setJumping(0)
                #print("jumping")

        return testMover(agent,targetPos,2300)
        #return greedyMover(agent,targetPos)



    else:
        return None




def kickOffTest(agent):
    if agent.gameInfo.is_kickoff_pause:
        if len(agent.allies) > 0:
            myDist = distance2D(agent.me.location,agent.ball.location)
            for ally in agent.allies:
                if distance2D(ally.location,agent.ball.location) < myDist:
                    return False
        return True
    return False

def flipHandler(agent,flip_status):
    if flip_status.started:
        jump = False
    else:
        jump = True
        flip_status.started = True
        flip_status.flipStartedTimer = time.time()

    if time.time() - flip_status.flipStartedTimer >= 0.15:
        jump = True
        flip_status.flipDone = True

    return jump


def quad(a,b,c):
    inside = (b**2) - (4*a*c)
    if inside < 0 or a == 0:
        return 0.1
    else:
        n = ((-b - math.sqrt(inside))/(2*a))
        p = ((-b + math.sqrt(inside))/(2*a))
        if p > n:
            return p
        return n



def get_car_facing_vector(car):
    pitch = float(car.rotation[0])
    yaw = float(car.rotation[1])
    facing_x = math.cos(pitch) * math.cos(yaw)
    facing_y = math.cos(pitch) * math.sin(yaw)

    return Vector([facing_x, facing_y])






def rotator_to_matrix(our_object):
    r = our_object.rotation
    CR = math.cos(r[2])
    SR = math.sin(r[2])
    CP = math.cos(r[0])
    SP = math.sin(r[0])
    CY = math.cos(r[1])
    SY = math.sin(r[1])

    matrix = []
    matrix.append(Vector([CP*CY, CP*SY, SP]))
    matrix.append(Vector([CY*SP*SR-CR*SY, SY*SP*SR+CR*CY, -CP * SR]))
    matrix.append(Vector([-CR*CY*SP-SR*SY, -CR*SY*SP+SR*CY, CP*CR]))
    return matrix

def getLocation(_object):
    if type(_object) == Vector:
        return _object
    if type(_object) == physicsObject:
        return _object.location
    raise ValueError(f"{type(_object)} is not a valid input for 'getLocation' function ")



def clamp(_max,_min,value):
    if value > _max:
        return _max
    if value < _min:
        return _min
    return value

def sign(x):
    if x <= 0:
        return -1
    else:
        return 1

def steer(angle):
    final = ((35 * angle) ** 3) / 20
    return clamp(1,-1,final)

def newSteer(angle):
    if angle < -180:
        angle += 360
    if angle > 180:
        angle -= 360
    turn = Gsteer(angle)
    slide = False

    if abs(math.degrees(angle)) >=80:
        slide = True

    return (turn,slide)

def slideSteer(angle,distance):
    sliding = False
    degrees = math.degrees(angle)

    if distance < 1000:
        if abs(degrees) > 70 and abs(degrees) < 180:
            sliding = True
        """
        if abs(degrees) < 3:
            return(0,False)
        """

        return (clamp(1, -1, (degrees/360)*8),sliding)
    else:
        if abs(degrees) < 3:
            return(0,False)

        return (clamp(1, -1, (degrees/360)*8), sliding)


def saferBoostGrabber(agent):
    minDistance = distance2D(Vector([0, 5100 * sign(agent.team), 200]),agent.ball.location)
    closestBoost = physicsObject()
    closestBoost.location = Vector([0, 5100 * sign(agent.team), 200])
    bestDistance = math.inf
    bestAngle = 0

    for boost in agent.boosts:
        if boost.spawned:
            goalDistance = distance2D(boost.location, Vector([0, 5100 * sign(agent.team), 200]))
            if goalDistance < minDistance*.7:
                distance = distance2D(agent.me.location,boost.location)
                localCoords = toLocal(boost.location, agent.me)
                angle = abs(math.degrees(math.atan2(localCoords[1], localCoords[0])))
                if boost.bigBoost:
                    distance = distance*.5
                distance += angle * 5
                if distance < bestDistance:
                    bestDistance = distance
                    closestBoost = boost
                    bestAngle = angle

    agent.renderCalls.append(renderCall(agent.renderer.draw_line_3d, agent.me.location.data, closestBoost.location.data,
                                        agent.renderer.yellow))

    return efficientMover(agent, closestBoost.location, agent.maxSpd)

def boostHungry(agent):
    closestBoost = agent.me
    bestDistance = math.inf
    bestAngle = 0

    for boost in agent.boosts:
        if boost.spawned:
            distance = distance2D(boost.location, agent.me)
            localCoords = toLocal(closestBoost.location, agent.me)
            angle = abs(math.degrees(math.atan2(localCoords[1], localCoords[0])))
            distance +=  angle*5
            distance += distance2D(agent.me.location,agent.ball.location)
            if boost.bigBoost:
                distance *= .5
            if distance < bestDistance:
                bestDistance = distance
                closestBoost = boost
                bestAngle = angle

    agent.renderCalls.append(renderCall(agent.renderer.draw_line_3d, agent.me.location.data, closestBoost.location.data,
                                        agent.renderer.yellow))
    return efficientMover(agent, closestBoost.location, agent.maxSpd)

def distance1D(origin,destination,index):
    return abs(getLocation(origin)[index] - getLocation(destination)[index])


def defendGoal(agent):
    return turtleTime(agent)


def findOppositeSideVector(agent,objVector,antiTarget, desiredBallDistance):
    angle = math.degrees(angle2(objVector,antiTarget))
    targetDistance = distance2D(objVector, getLocation(antiTarget))
    oppositeVector = (getLocation(antiTarget) - objVector).normalize()
    totalOffset = desiredBallDistance
    return getLocation(antiTarget) - (oppositeVector.scale(targetDistance + desiredBallDistance))


def findOppositeSide(agent,targetLoc,antiTarget, desiredBallDistance):
    angle = math.degrees(angle2(targetLoc,antiTarget))
    targetDistance = distance2D(targetLoc, getLocation(antiTarget))
    oppositeVector = (getLocation(antiTarget) - targetLoc).normalize()
    totalOffset = desiredBallDistance
    return getLocation(antiTarget) - (oppositeVector.scale(targetDistance + desiredBallDistance))

def findGoalAngle(agent):
    center = Vector([0, 5150 * -sign(agent.team), 200])
    return math.degrees(angle2(agent.ball, center)) * sign(agent.team)

def determineVelocityToGoal(agent):
    myGoal = center = Vector([0, 5150 * -sign(agent.team), 200])
    startingDistance = distance2D(myGoal,agent.ball.location)
    if startingDistance < distance2D(myGoal,agent.ball.location + agent.ball.velocity):
        return True
    else:
        return False

def backmanDefense(agent):
    center = Vector([0, 4700 * sign(agent.team), 200])

    if distance2D(center,agent.ball.location) < 3500:
        return turtleTime(agent)
    else:
        if agent.me.boostLevel < 50:
            return saferBoostGrabber(agent)
        else:
            if distance2D(agent.me.location,center) > 600:
                return efficientMover(agent, center, 2200)
            else:
                return efficientMover(agent,center,500)


def secondManSupport(agent):
    if agent.me.boostLevel < 50:
        return saferBoostGrabber(agent)

    else:
        if agent.ball.location[0] > 0:
            x = -250
        else:
            x = 250
        return efficientMover(agent,Vector([x,agent.ball.location[1]*.7,100]),2200)


def noOwnGoalDefense(agent):
    leftCorner = Vector([-sign(agent.team) * 4096, 5000 * sign(agent.team), 200])
    rightCorner = Vector([sign(agent.team) * 4096, 5000 * sign(agent.team), 200])
    leftPost = Vector([-sign(agent.team) * 800, 5000 * sign(agent.team), 200])
    rightPost = Vector([sign(agent.team) * 800, 5000 * sign(agent.team), 200])
    center = Vector([0, 5000 * sign(agent.team), 200])

    ballGoalDist = distance2D(agent.ball.location, center)
    carGoalDist = distance2D(agent.me.location, center)

    if carGoalDist < ballGoalDist:
        return (center, False)

    if ballGoalDist > 6000:
        #print(ballGoalDist)
        return (center,True)


    ballToLeft = distance1D(leftCorner,agent.ball.location,0)
    ballToRight = distance1D(rightCorner,agent.ball.location,0)

    carToLeft = distance1D(leftCorner,agent.me.location,0)
    carToRight = distance1D(rightCorner,agent.me.location,0)


    if carToLeft < ballToLeft:
        return (leftPost, True)

    if carToRight < ballToRight:
        return (rightPost, True)





    return (center, True)





def turtleTime(agent):
    #7-21 @ 15:30
    goalDistance = distance2D(agent.ball.location,Vector([0, 5100 * sign(agent.team), 200]))
    defendTarget = Vector([0, 5250 * sign(agent.team), 200])

    if agent.selectedBallPred == None:
        targetStruct = findSoonestBallTouchable(agent)
    else:
        targetStruct = agent.selectedBallPred

    if targetStruct != None:
        targetVec = Vector([targetStruct.physics.location.x, targetStruct.physics.location.y,
                            targetStruct.physics.location.z])
        agent.ballDelay = targetStruct.game_seconds - agent.gameInfo.seconds_elapsed

    else:
        targetVec = agent.ball.location
        agent.ballDelay = 0

    _enemyInfluenced = True
    if goalDistance < 1300:
        _enemyInfluenced = False
    elif targetVec[2] > 165:
        _enemyInfluenced = False

    flipDecider(agent,enemyInfluenced= _enemyInfluenced)

    if distance2D(targetVec,defendTarget) < 5000:
        if ballHeadedTowardsMyGoal(agent):
            defendTarget, reposition = noOwnGoalDefense(agent)
            if reposition:
                agent.renderCalls.append(
                    renderCall(agent.renderer.draw_line_3d, agent.me.location.data, defendTarget.data,
                               agent.renderer.blue))
                return testMover(agent, defendTarget, 2300)
    targ_local = toLocal(targetVec,agent.me)
    goal_local = toLocal(Vector([0, 5100 * sign(agent.team), 200]),agent.me)
    targ_angle = math.degrees(math.atan2(targ_local[1],targ_local[0]))
    goal_angle = math.degrees(math.atan2(goal_local[1],goal_local[0]))

    distance = distance2D(defendTarget,targetVec)
    oppositeVector = (getLocation(defendTarget) - targetVec).normalize()
    destination =  getLocation(defendTarget) - (oppositeVector.scale(distance - clamp(110,50,70))) #targetVec[2]/2
    # if destination[2] < 140:
    #     result = testMover(agent, destination, 2300)
    # else:
    result = timeDelayedMovement(agent, destination, agent.ballDelay)

    destination.data[2] = 95
    agent.renderCalls.append(renderCall(agent.renderer.draw_line_3d,agent.me.location.data,destination.data,agent.renderer.blue))
    return result

def prepareShot(agent):
    leftPost = Vector([-sign(agent.team) * 700, 5100 * -sign(agent.team), 200])
    rightPost = Vector([sign(agent.team) * 700, 5100 * -sign(agent.team), 200])
    center = Vector([0, 5150 * -sign(agent.team), 200])


def flipDecider(agent,enemyInfluenced=False):
    if not enemyInfluenced:
        if not agent.onWall:
            if findDistance(agent.me.location,agent.ball.location) < 250:
                agent.setJumping(0)

    else:
        if findDistance(agent.me.location, agent.ball.location) < 250:
            if len(agent.enemies) > 0:
                closest = agent.enemies[0]
                cDist = math.inf
                for e in agent.enemies:
                    x = findDistance(e.location,agent.ball.location)
                    if x < cDist:
                        cDist = x
                        closest = e
                if cDist < 350:
                    agent.setJumping(0)





def lineupShot(agent,multi):
    variance = 5

    leftPost = Vector([-sign(agent.team) * 700, 5200 * -sign(agent.team), 200])
    rightPost = Vector([sign(agent.team) * 700, 5200 * -sign(agent.team), 200])
    center = Vector([0, 5200 * -sign(agent.team), 200])
    if agent.selectedBallPred == None:
        targetStruct = findSoonestBallTouchable(agent)
    else:
        targetStruct = agent.selectedBallPred

    if targetStruct != None:
        targetVec = Vector([targetStruct.physics.location.x, targetStruct.physics.location.y,
                            targetStruct.physics.location.z])
        agent.ballDelay = targetStruct.game_seconds - agent.gameInfo.seconds_elapsed

    else:
        targetVec = agent.ball.location
        agent.ballDelay = 0

    dist = distance2D(agent.me.location, targetVec)
    goalDist = distance2D(center, targetVec)
    ballToGoalDist = distance2D(targetVec, center)
    targetLocal = toLocal(targetVec,agent.me)
    carToBallAngle = math.degrees(math.atan2(targetLocal[1],targetLocal[0]))
    carToGoalDistance = distance2D(center,agent.me.location)

    if carToBallAngle > 180:
        carToBallAngle-=360
    elif carToBallAngle < -180:
        carToBallAngle+=360


    shotAngles = [math.degrees(angle2(targetVec, leftPost)),
                  math.degrees(angle2(targetVec, center)),
                  math.degrees(angle2(targetVec, rightPost))]

    correctedAngles=[x + 90 * -sign(agent.team) for x in shotAngles]

    _enemyInfluenced = True
    if targetVec[2] > 165:
        _enemyInfluenced = False

    flipDecider(agent, enemyInfluenced= _enemyInfluenced)

    #blue team
    #+90 makes goal angles accurate
    #positive to local left of goal, negative to local right

    #orangeTeam
    # -90 to correct angles
    # negative to right of goal, positive to left


    if ballToGoalDist < 1200:
        goalAngle = correctedAngles[1]
        goalSpot = center

    else:
        if correctedAngles[1] < -variance:
            goalAngle = correctedAngles[0]
            goalSpot = leftPost

        elif correctedAngles[1] > variance:
            goalAngle = correctedAngles[2]
            goalSpot = rightPost

        else:
            goalAngle = correctedAngles[1]
            goalSpot = center

    if abs(correctedAngles[1]) > 70:
        if agent.me.boostLevel < 50:
            if len(agent.allies) < 1:
                return boostHungry(agent)

    #blue team
    # ball straight ahead local is -0
    # ball on left is negative degrees ball on right is positive

    #orange team
    # straight ahead is 0
    # positive if ball to the right

    # if targetToBallAngle > 180:
    #     targetToBallAngle-=360
    # elif targetToBallAngle < -180:
    #     targetToBallAngle+=360
    #
    # if goalAngle < -180:
    #     goalAngle += 360
    # if goalAngle > 180:
    #     goalAngle -= 360

    # ball_radius = 92.5
    # carLength = 118
    # offset = (ball_radius+(carLength/2))
    # ballToGoalMulti = clamp(1,0,ballToGoalDist/5000)
    # distMulti = clamp(3,.1,dist/500)
    # goalOffset = clamp(350,minMin,(abs(goalAngle)+abs(targetToBallAngle)*(distMulti+ballToGoalMulti)))
    # goalLocal = toLocal(goalSpot,agent.me)
    # goalLocalDegrees = math.degrees(math.atan2(goalLocal[1],goalLocal[0]))*clamp(2,1,dist/100)
    # if goalLocalDegrees < -180:
    #     goalLocalDegrees += 360
    # if goalLocalDegrees > 180:
    #     goalLocalDegrees -= 360

    #goalOffset = abs(goalLocalDegrees)+abs(targetToBallAngle)

    #print(goalLocalDegrees,targetToBallAngle,goalOffset)
    #if abs(goalLocalDegrees) <= 120:
    targetLoc = None
    rush = False
    if agent.ball.location[2] <= 145:
        if dist >40:
            rush = True
    if carToGoalDistance < ballToGoalDist:
        if len(agent.allies) > 0:
            targetLoc = findOppositeSide(agent,targetVec,goalSpot, 400)
            rush = True

    elif abs(targetVec[0]) < 2300: ##hook a shot in
        if ballToGoalDist < 5500 and ballToGoalDist > 1000:
            if targetVec[2] < 150:
                _direction  = direction(center,targetVec)
                targetLoc = targetVec - _direction.scale(110 + dist * 0.5)


    if not targetLoc:
        if dist < 300:
            if abs(carToBallAngle) < 30:
                if is_in_strike_zone(agent, targetVec): #trying to just shove rrr in
                    targetLoc = targetVec
                    rush = True
        if not targetLoc:
            targetLoc = findOppositeSide(agent,targetVec,goalSpot,clamp(125,20,(dist/1.5)-abs(carToBallAngle))) #try to center it

    # else:
    #     localVector = toLocal(targetVec,agent.me)
    #     targetToBallAngle = math.degrees(math.atan2(localVector[1],localVector[0]))
    #     ballToGoalMulti = clamp(1,0,ballToGoalDist/5000)
    #     distMulti = clamp(3,.1,dist/500)
    #     #goalOffset = clamp(350,minMin,(abs(goalAngle)+abs(targetToBallAngle)*(distMulti+ballToGoalMulti)))
    #     goalOffset = clamp(120,20,(abs(targetToBallAngle)+abs(goalLocalDegrees))*(dist + ballToGoalDist)/2500)
    #     targetLoc = findOppositeSideVector(agent,targetVec,goalSpot, goalOffset)
    #     #print("awkward lineup!",time.time())



    # if agent.ball.location[2] < 140:
    #     result = testMover(agent, targetLoc, 2200)
    # else:
    if rush:
        result = testMover(agent,targetLoc,2200)
    else:
        result = timeDelayedMovement(agent, targetLoc, agent.ballDelay)

    targetLoc.data[2] = 95
    agent.renderCalls.append(renderCall(agent.renderer.draw_line_3d, agent.me.location.data, targetLoc.data,
                                        agent.renderer.purple))

    agent.renderCalls.append(renderCall(agent.renderer.draw_line_3d, agent.ball.location.data, goalSpot,
                                        agent.renderer.red))
    return result

def maxSpeedAdjustment(agent,target):
    tar_local = toLocal(target,agent.me)
    angle = math.degrees(math.atan2(tar_local[1],tar_local[0]))
    distCorrection = distance2D(agent.me.location,target)/300
    if angle > 180:
        angle -= 360
    elif angle <-180:
        angle += 360

    if distCorrection > abs(angle):
        angle = 0

    else:
        if angle > 0:
            angle -= distCorrection
        else:
            angle += distCorrection

    cost = 2200/180

    if abs(angle) <-10:
        return 2200

    elif abs(angle) >=90:
        return 500

    else:
        return clamp(2200,1000,2200 - (angle*cost))






def is_in_strike_zone(agent, ball_vec):
    leftPost = Vector([-sign(agent.team) * 700, 5100 * -sign(agent.team), 200])
    rightPost = Vector([sign(agent.team) * 700, 5100 * -sign(agent.team), 200])
    #center = Vector([0, 5100 * -sign(agent.team), 200])
    if angle2(leftPost, agent.me) < angle2(ball_vec,agent.me) < angle2(rightPost,agent.me):
        return True
    return False

def inaccurateArrivalEstimator(agent,destination):
    distance = clamp(math.inf,1,distance2D(agent.me.location,destination))
    currentSpd = clamp(2300,1,agent.getCurrentSpd())
    if agent.me.boostLevel > 0:
        maxSpd = clamp(2200,currentSpd,currentSpd+ (distance*.7))
    else:
        maxSpd = clamp(2200, currentSpd, currentSpd + (distance*.5))
    avgSpd = clamp(2200,currentSpd,currentSpd + currentSpd*(currentSpd/maxSpd))
    return distance/avgSpd


def direction(source, target) -> Vector:
    return (getLocation(source) - getLocation(target)).normalize()

def angle2(target_location,object_location):
    difference = getLocation(target_location) - getLocation(object_location)
    return math.atan2(difference[1], difference[0])

def getVelocity(_obj):
    return math.sqrt(sum([x*x for x in _obj]))

def getVelocity2D(_obj):
    return math.sqrt(sum[_obj.velocity[0]**2,_obj.velocity[0]**2])

def findDistance(origin,destination):
    difference = getLocation(origin) - getLocation(destination)
    return abs(math.sqrt(sum([x * x for x in difference])))

def distance2D(origin,destination):
    _origin = getLocation(origin)
    _destination = getLocation(destination)
    _origin = Vector([_origin[0],_origin[1]])
    _destination = Vector([_destination[0],_destination[1]])
    difference = _origin - _destination
    return abs(math.sqrt(sum([x * x for x in difference])))

def localizeVector(target_object,our_object):
    x = (getLocation(target_object) - getLocation(our_object.location)).dotProduct(our_object.matrix[0])
    y = (getLocation(target_object) - getLocation(our_object.location)).dotProduct(our_object.matrix[1])
    z = (getLocation(target_object) - getLocation(our_object.location)).dotProduct(our_object.matrix[2])
    #print("called")
    return Vector([x, y, z])

def toLocal(target,our_object):
    if isinstance(target,physicsObject):
        return target.local_location
    else:
        return localizeVector(target,our_object)

def testMover(agent, target_object,targetSpd):
    if targetSpd > 2200:
        targetSpd = 2200
    currentSpd = agent.getCurrentSpd()

    #print(currentSpd)
    if targetSpd < currentSpd+150 or agent.me.boostLevel <=0:
        return efficientMover(agent,target_object,targetSpd)

    if not agent.forward:
        return efficientMover(agent, target_object, targetSpd)

    location = toLocal(target_object, agent.me)
    controller_state = SimpleControllerState()
    angle_to_target = math.atan2(location.data[1], location.data[0])
    #_distance = distance2D(agent.me, target_object)
    _distance = distance2D(agent.me, target_object)
    #print(math.degrees(angle_to_target))

    steering, slide = newSteer(angle_to_target)

    if abs(steering) >=95:
        optimalSpd = maxSpeedAdjustment(agent, target_object)

        if targetSpd > optimalSpd:
            targetSpd = optimalSpd

    controller_state.steer = steering
    controller_state.handbrake = slide
    if agent.forward:
        controller_state.throttle = 1.0
    else:
        controller_state.throttle = -1.0

    if targetSpd > currentSpd:
        if agent.onSurface and not slide:
            if targetSpd > 1400 and currentSpd < 2200:
                if agent.forward:
                    controller_state.boost = True
    elif targetSpd < currentSpd:
        if agent.getActiveState() != 3:
            if agent.forward:
                controller_state.throttle = -1
            else:
                controller_state.throttle = 1
        else:
            if currentSpd - targetSpd < 25:
                controller_state.throttle = 0
            else:
                if agent.forward:
                    controller_state.throttle = -1
                else:
                    controller_state.throttle = 1

    return controller_state


# def testMover(agent, target_object,targetSpd):
#     if targetSpd > 2200:
#         targetSpd = 2200
#     currentSpd = agent.getCurrentSpd()
#     if targetSpd < currentSpd+150 or agent.me.boostLevel <=0:
#         return efficientMover(agent,target_object,targetSpd)
#
#     location = toLocal(target_object, agent.me)
#     controller_state = SimpleControllerState()
#     angle_to_target = math.atan2(location.data[1], location.data[0])
#     #_distance = distance2D(agent.me, target_object)
#     _distance = distance2D(agent.me, target_object)
#     #print(math.degrees(angle_to_target))
#
#     current_vel = getVelocity(agent.me.velocity)
#     steering, slide = newSteer(angle_to_target)
#     #steering, slide = slideSteer(angle_to_target,_distance)
#
#     controller_state.steer = steering
#     controller_state.handbrake = slide
#     controller_state.throttle = 1.0
#
#     if targetSpd >=2200:
#         maxSpeed = 2200
#         if _distance < 800:
#             maxSpeed-= (abs(steering*.5))*maxSpeed
#
#         #throttle
#         if maxSpeed > current_vel:
#             controller_state.throttle = 1.0
#             if agent.onSurface and not slide:
#                 if maxSpeed > 1400 and current_vel < 2200:
#                     if agent.onSurface:
#                         controller_state.boost = True
#         elif maxSpeed < current_vel:
#             if agent.getActiveState() != 3:
#                 controller_state.throttle = -1
#             else:
#                 if current_vel - maxSpeed < 5:
#                     controller_state.throttle = 0
#                 else:
#                     controller_state.throttle = -1
#
#     else:
#         if currentSpd < targetSpd:
#             controller_state.throttle = 1.0
#             if not slide:
#                 if (targetSpd-currentSpd) > 50:
#                     if agent.onSurface:
#                         controller_state.boost = True
#
#         elif currentSpd > targetSpd:
#             if agent.getActiveState() != 3:
#                 controller_state.throttle = -1
#             else:
#                 if currentSpd - targetSpd < 25:
#                     controller_state.throttle = 0
#                 else:
#                     controller_state.throttle = -1
#
#     return controller_state

def timeDelayedMovement(agent,targetVec,delay):
    dist = distance2D(agent.me.location,targetVec)
    if dist > 700:
        return testMover(agent, targetVec, 2200)

    arrivalEstimate = inaccurateArrivalEstimator(agent, targetVec)
    if arrivalEstimate > delay:
        #print(arrivalEstimate,delay)
        return testMover(agent, targetVec, 2200)

    else:
        return testMover(agent, targetVec, dist/delay)

# def timeDelayedMovement(agent,targetVec,delay):
#     targetDistance = distance2D(agent.me.location,targetVec)
#     currentSpd = agent.getCurrentSpd()
#     if currentSpd == 0:
#         currentSpd = 0.0001
#     currentTimeToTarget = targetDistance/currentSpd
#     if delay == 0:
#         optimalspeed = 2200
#     else:
#         optimalspeed = targetDistance/delay
#     if currentTimeToTarget > delay:
#         if optimalspeed > currentSpd + 150 and optimalspeed > currentSpd+currentSpd/10:
#             return testMover(agent, targetVec, 2200)
#         else:
#             return efficientMover(agent, targetVec, 2200)
#
#     else:
#         if targetDistance <= 500:
#             return efficientMover(agent,targetVec,optimalspeed)
#         else:
#             return efficientMover(agent, targetVec, 2200)


def efficientMover(agent,target_object,target_speed):
    controller_state = SimpleControllerState()
    location = toLocal(target_object, agent.me)
    angle_to_target = math.atan2(location.data[1], location.data[0])
    #_distance = distance2D(agent.me, target_object)
    _distance = distance2D(agent.me, target_object)
    current_speed = getVelocity(agent.me.velocity)
    steerDirection, slideBool = newSteer(angle_to_target)
    controller_state.steer = steerDirection
    controller_state.handbrake = slideBool


    if not agent.forward:
        if agent.onSurface:
            if _distance > 800:
                _angle = math.degrees(angle_to_target)
                if _angle < -180:
                    _angle += 360
                if _angle > 180:
                    _angle -= 360

                if abs(_angle) > 120:
                    agent.setHalfFlip()
    if abs(steerDirection) >=.95:
        optimalSpd = maxSpeedAdjustment(agent, target_object)

        if target_speed > optimalSpd:
            target_speed = optimalSpd

    if current_speed < target_speed:
        if agent.forward:
            controller_state.throttle = 1
        else:
            controller_state.throttle = -1
        flip = False
        if agent.getActiveState() == 3 or agent.getActiveState() == 4 or agent.getActiveState() == 5:
            if agent.ballDelay > 2.5:
                flip = True
        if _distance > 2500:
            flip = True
        if flip:
            if abs(math.degrees(angle_to_target)) <= clamp(3,0,_distance/1000):
                if agent.onSurface:
                    if agent.forward:
                        agent.setJumping(1)
                    else:
                        agent.setJumping(3)

    else:
        if agent.forward:
            controller_state.throttle = 1
        else:
            controller_state.throttle = -1


        if current_speed > target_speed:
            if agent.getActiveState() != 3:
                if agent.forward:
                    controller_state.throttle = -1
                else:
                    controller_state.throttle = 1
            else:
                if current_speed - target_speed < 25:
                    controller_state.throttle = 0
                else:
                    if agent.forward:
                        controller_state.throttle = -1
                    else:
                        controller_state.throttle = 1

    return controller_state

# def efficientMover(agent,target_object,target_speed):
#     controller_state = SimpleControllerState()
#     location = toLocal(target_object, agent.me)
#     angle_to_target = math.atan2(location.data[1], location.data[0])
#     #_distance = distance2D(agent.me, target_object)
#     _distance = distance2D(agent.me, target_object)
#     current_speed = getVelocity(agent.me.velocity)
#     steerDirection, slideBool = newSteer(angle_to_target)
#     if current_speed < target_speed:
#         if _distance > 2000:
#             if abs(math.degrees(angle_to_target)) <= clamp(3,0,_distance/1000):
#                 if agent.onSurface:
#                     agent.setJumping(1)
#
#     controller_state.steer = steerDirection
#     controller_state.handbrake =  slideBool
#     controller_state.throttle = 1
#     if not slideBool:
#         if _distance < 150:
#             if abs(angle_to_target) >1.75:
#                 controller_state.throttle = 0.5
#
#
#     if current_speed > target_speed:
#         if agent.getActiveState() != 3:
#             controller_state.throttle = -1
#         else:
#             if current_speed - target_speed < 25:
#                 controller_state.throttle = 0
#             else:
#                 controller_state.throttle = -1
#
#     return controller_state


def Gsteer(angle):
    final = ((10 * angle+sign(angle))**3) / 20
    return clamp(1,-1,final)


def greedyMover(agent,target_object):
    controller_state = SimpleControllerState()
    controller_state.handbrake = False
    location = toLocal(target_object, agent.me)
    angle = math.atan2(location.data[1], location.data[0])
    controller_state.throttle = 1
    if getVelocity(agent.me.velocity) < 2200:
        if agent.onSurface:
            controller_state.boost = True
    controller_state.jump = False

    controller_state.steer = Gsteer(angle)

    return controller_state




def exampleController(agent, target_object,target_speed):
    distance = distance2D(agent.me.location,target_object.location)
    if distance > 400:
        #print("switching to efficient")
        agent.state = efficientMover
        return efficientMover(agent,target_object,target_speed)

    controller_state = SimpleControllerState()
    controller_state.handbrake = False

    car_direction = get_car_facing_vector(agent.me)
    car_to_ball =  agent.me.location - target_object.location

    steer_correction_radians = steer(car_direction.correction_to(car_to_ball))

    current_speed = getVelocity(agent.me.velocity)
    #steering
    controller_state.steer = steer(steer_correction_radians)

    #throttle
    if target_speed > current_speed:
        controller_state.throttle = 1.0
        if target_speed > 1400 and current_speed < 2250:
            controller_state.boost = True
    elif target_speed < current_speed:
        controller_state.throttle = 0

    return controller_state



def isBallHittable(ballStruct):
    if ballStruct.physics.location.z<= 92+84:
        return True
    # if ballStruct.physics.location.x > 4096 - 120:
    #     if ballStruct.physics.location.z <= 250:
    #         return True
    # if ballStruct.physics.location.x < -4096 + 120:
    #     if ballStruct.physics.location.z <= 250:
    #         return True
    return False


def findSoonestBallTouchable(agent):
    if agent.ballPred != None:
        bestStruct = agent.ballPred.slices[359]
        quickest = 99999999
        spd = clamp(2200, 300, abs(agent.getCurrentSpd()))

        if agent.ballPred is not None:
            for i in range(0, agent.ballPred.num_slices):
                if agent.ballPred.slices[i].physics.location.z <= 155:
                    distance = distance2D(agent.me.location, Vector([agent.ballPred.slices[i].physics.location.x,
                                                                     agent.ballPred.slices[i].physics.location.y,
                                                                     agent.ballPred.slices[i].physics.location.z]))
                    adjustedSpd = clamp(2200,1400, spd+ distance * .7)
                    timeEstimate = distance / adjustedSpd
                    if timeEstimate < quickest:
                        bestStruct = agent.ballPred.slices[i]
                        quickest = timeEstimate
            return bestStruct

    return None

def findSuitableBallPosition2(agent, heightMin,speed,origin):
    applicableStructs = []
    spd = clamp(2300,100,speed)
    if agent.ballPred is not None:
        for i in range(0, agent.ballPred.num_slices):
            if isBallHittable(agent.ballPred.slices[i]):
                applicableStructs.append(agent.ballPred.slices[i])

    for pred in applicableStructs:
        distance = findDistance(origin,Vector([pred.physics.location.x,pred.physics.location.y,pred.physics.location.z]))
        adjustSpd = clamp(2200,spd,speed+distance*.7)
        if distance/adjustSpd < (pred.game_seconds - agent.gameInfo.seconds_elapsed):
            return pred

    return agent.ballPred.slices[1]

def ballHeadedTowardsMyGoal(agent):
    myGoal = Vector([0, 5100 * sign(agent.team), 200])
    if (distance1D(myGoal, agent.ball.location, 1)  - distance1D(myGoal, agent.ball.location + agent.ball.velocity, 1)) > 0:
        if agent.ball.velocity.magnitude() > 50:
            return True

    return False

def openGoalOpportunity(agent):
    enemyGoal = Vector([0, 5100 * -sign(agent.team), 200])
    ballDistance = distance2D(agent.ball.location,enemyGoal)+200

    for e in agent.enemies:
        if distance2D(e.location,enemyGoal,) < ballDistance:
            return False

    return True

def challengeDecider(agent):
    myDistance = distance2D(agent.me.location,agent.ball.location)
    for e in agent.enemies:
        if distance2D(e.location,agent.ball.location) < myDistance-200:
            return False
    return True


def dpp(target_loc,target_vel,our_loc,our_vel):
    target_loc = getLocation(target_loc)
    our_loc = getLocation(our_loc)
    our_vel = getLocation(our_vel)
    d = distance2D(target_loc,our_loc)
    if d != 0:
        return (((target_loc.data[0] - our_loc.data[0]) * (target_vel.data[0] - our_vel.data[0])) + ((target_loc.data[1] - our_loc.data[1]) * (target_vel.data[1] - our_vel.data[1])))/d
    else:
        return 0

def timeZ(ball):
    rate = 0.97
    return quad(-325, ball.velocity.data[2] * rate, ball.location.data[2]-92.75)


def radius(v):
    return 139.059 + (0.1539 * v) + (0.0001267716565 * v * v)

def ballLowEnough(agent):
    if agent.ball.location[2] < 140:
        return True
    return False

def ballReady(agent):
    ball = agent.ball
    if abs(ball.velocity.data[2]) < 140 and timeZ(agent.ball) < 1:
        return True
    return False

def ballProject(agent):
    goal = Vector([0,-sign(agent.team)*FIELD_LENGTH/2,100])
    goal_to_ball = (agent.ball.location - goal).normalize()
    difference = agent.me.location - agent.ball.location
    return difference * goal_to_ball


# def lineupShot(agent, multi):
#     variance = 25
#
#     leftPost = Vector([-sign(agent.team) * 700, 5100 * -sign(agent.team), 200])
#     rightPost = Vector([sign(agent.team) * 700, 5100 * -sign(agent.team), 200])
#     center = Vector([0, 5100 * -sign(agent.team), 200])
#
#     targetStruct = findSoonestBallTouchable(agent)
#     if targetStruct != None:
#         targetVec = Vector([targetStruct.physics.location.x, targetStruct.physics.location.y,
#                             targetStruct.physics.location.z])
#
#     else:
#         targetVec = agent.ball.location
#
#     dist = distance2D(agent.me.location, targetVec)
#     goalDist = distance2D(center, targetVec)
#     ballToGoalDist = distance2D(targetVec, center)
#
#     targetVec.data[2] = 75
#
#     shotAngles = [math.degrees(angle2(targetVec, leftPost)) * sign(agent.team),
#                   math.degrees(angle2(targetVec, center)) * sign(agent.team),
#                   math.degrees(angle2(targetVec, rightPost)) * sign(agent.team)]
#     # correctedAngles = [(x-90)*-sign(agent.team) for x in shotAngles]
#     # correctedAngles = [x + (90* sign(agent.team)) for x in shotAngles]
#     correctedAngles = [x - 90 for x in shotAngles]
#     # correctedAngles = shotAngles
#
#     # if abs(correctedAngles[1]) > 75:
#     #     flipDecider(agent, enemyInfluenced=True)
#     # else:
#     #     flipDecider(agent, enemyInfluenced=True)
#
#     flipDecider(agent, enemyInfluenced=True)
#
#     if ballToGoalDist < 1800:
#         goalAngle = correctedAngles[1]
#         goalSpot = center
#
#     else:
#         if correctedAngles[1] > variance:
#             goalAngle = correctedAngles[0]
#             goalSpot = leftPost
#
#         elif correctedAngles[1] < -variance:
#             goalAngle = correctedAngles[2]
#             goalSpot = rightPost
#
#         else:
#             goalAngle = correctedAngles[1]
#             goalSpot = center
#
#     # targetToBallAngle = clamp(600,-600,abs(math.degrees(angle2(targetVec, agent.me)*2))*abs(sign(agent.team) + (-90)*dist/5))
#
#     targetToBallAngle = clamp(600, -600, abs(math.degrees(angle2(targetVec, agent.me))) - 90)
#     print(targetToBallAngle)
#     # print(correctedAngles[0],targetToBallAngle)
#
#     if abs(targetToBallAngle) + abs(correctedAngles[1]) * clamp(5, 1, goalDist / 2000) > 10:
#         clampMax = clamp(300, 90, 90 * (dist / 200))
#     else:
#         clampMax = 30
#
#     goaloffset = abs(goalAngle) * clampMax / 25 + (abs(targetToBallAngle))
#     _multi = targetToBallAngle / 90
#
#     # totalOffset = clamp(clampMax, 30, goaloffset)*multi
#     totalOffset = clamp(clampMax, 30, goaloffset) * multi
#     targetLoc = findOppositeSide(agent, targetVec, goalSpot, totalOffset)
#
#     agent.renderCalls.append(renderCall(agent.renderer.draw_line_3d, agent.me.location.data, targetLoc.data,
#                                         agent.renderer.purple))
#
#     agent.renderCalls.append(renderCall(agent.renderer.draw_line_3d, agent.ball.location.data, goalSpot,
#                                         agent.renderer.red))
#
#     if not ballLowEnough(agent):
#         return timeDelayedMovement(agent, targetLoc, agent.ballDelay)
#     else:
#         return testMover(agent, targetLoc, agent.maxSpd)












