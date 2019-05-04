import math
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
        if len(self.data) == len(other):
            return Vector([self.data[i] * other[i] for i in range(len(other))])
        else:
            self.raiseLengthError(other,"multiplication")

    def __add__(self, other):
        if len(self.data) == len(other):
            return Vector([self.data[i] + other[i] for i in range(len(other))])
        else:
            self.raiseLengthError(other, "addition")

    def __sub__(self, other):
        if len(self.data) == len(other):
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




def kickOffTest(agent):
    if agent.gameInfo.is_kickoff_pause:
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
    turn = Gsteer(angle)
    slide = False
    if abs(math.degrees(angle)) >=100:
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
                distance /=2
            if distance < bestDistance:
                bestDistance = distance
                closestBoost = boost
                bestAngle = angle


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


def noOwnGoalDefense(agent):
    rightCorner = Vector([4096, 5100 * sign(agent.team), 200])
    leftCorner = Vector([-4096, 5100 * sign(agent.team), 200])

    b_dist_right = distance1D(rightCorner,agent.ball.location,0)
    b_dist_left = distance1D(leftCorner,agent.ball.location,0)
    a_dist_right = distance1D(rightCorner,agent.me.location,0)
    a_dist_left =  distance1D(leftCorner,agent.me.location,0)

    if b_dist_right > a_dist_right:
        return leftCorner

    if b_dist_left > a_dist_left:
        return rightCorner

    return rightCorner

    # if distance1D(rightCorner,agent.me.location,0) > distance1D(rightCorner,agent.ball.location,0):
    #     return leftCorner
    #
    # else: #distance1D(leftCorner, agent.me.location) > distance1D(leftCorner, agent.ball.location):
    #     return rightCorner





def turtleTime(agent):
    flipDecider(agent, enemyInfluenced=True)
    defendTarget = Vector([0, 5250 * sign(agent.team), 200])
    if agent.selectedBallPred == None:
        targetVec = agent.ball.location
    else:
        targetVec = Vector([agent.selectedBallPred.physics.location.x,agent.selectedBallPred.physics.location.y,agent.selectedBallPred.physics.location.z])

    if distance2D(targetVec,defendTarget) < 3000:
        if ballHeadedTowardsMyGoal(agent):
            defendTarget = noOwnGoalDefense(agent)
    distance = distance2D(defendTarget,targetVec)
    oppositeVector = (getLocation(defendTarget) - targetVec).normalize()
    destination =  getLocation(defendTarget) - (oppositeVector.scale(distance - clamp(120,50,100))) #targetVec[2]/2
    destination.data[2] = 75
    agent.renderCalls.append(renderCall(agent.renderer.draw_line_3d,agent.me.location.data,destination.data,agent.renderer.blue))
    #agent.renderer.draw_line_3d(agent.me.location.data,destination.data,agent.renderer.create_color(0,0,255,255))
    #if agent.ballDelay < 0.1:
        #return testMover(agent,destination,agent.maxSpd)
    #else:
    return timeDelayedMovement(agent, destination, agent.ballDelay)

def prepareShot(agent):
    leftPost = Vector([-sign(agent.team) * 700, 5100 * -sign(agent.team), 200])
    rightPost = Vector([sign(agent.team) * 700, 5100 * -sign(agent.team), 200])
    center = Vector([0, 5150 * -sign(agent.team), 200])


def flipDecider(agent,enemyInfluenced=False):
    if not enemyInfluenced:
        if findDistance(agent.me.location,agent.ball.location) < 200:
            agent.setJumping(0)

    else:
        if findDistance(agent.me.location, agent.ball.location) < 200:
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
    flipDecider(agent,enemyInfluenced=True)

    leftPost = Vector([-sign(agent.team) * 700, 5100 * -sign(agent.team), 200])
    rightPost = Vector([sign(agent.team) * 700, 5100 * -sign(agent.team), 200])
    center = Vector([0, 5100 * -sign(agent.team), 200])

    targetStruct = findSoonestBallTouchable(agent)
    if targetStruct != None:
        targetVec = Vector([targetStruct.physics.location.x, targetStruct.physics.location.y,
                            targetStruct.physics.location.z])

    else:
        targetVec = agent.ball.location

    targetVec.data[2] = 75

    shotAngles = [math.degrees(angle2(targetVec,leftPost))*sign(agent.team),math.degrees(angle2(targetVec,center))*sign(agent.team),math.degrees(angle2(targetVec,rightPost))*sign(agent.team)]
    correctedAngles = [x-90 for x in shotAngles]

    # if correctedAngles[0] > 45:
    #     goalAngle = correctedAngles[2]
    #     goalSpot = rightPost
    #     #print(goalAngle, "RIGHT")
    #
    # elif correctedAngles[0] < -45:
    #     goalAngle = correctedAngles[0]
    #     goalSpot = leftPost
    #
    # else:
    #     goalAngle = correctedAngles[1]
    #     goalSpot = center
    goalAngle = correctedAngles[1]
    goalSpot = center


    targetToBallAngle = math.degrees(angle2(targetVec, agent.me)) - 90
    goaloffset = abs(goalAngle *2) + abs(targetToBallAngle *2)
    totalOffset = clamp(115, 30, goaloffset)*multi
    targetLoc = findOppositeSide(agent, targetVec,goalSpot, totalOffset)

    agent.renderCalls.append(renderCall(agent.renderer.draw_line_3d,agent.me.location.data, targetLoc.data,
                                        agent.renderer.purple))

    agent.renderCalls.append(renderCall(agent.renderer.draw_line_3d, agent.ball.location.data, goalSpot,
                                        agent.renderer.red))

    if not ballLowEnough(agent):
        return timeDelayedMovement(agent, targetLoc, agent.ballDelay)
    else:
        return testMover(agent, targetLoc, agent.maxSpd)




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
    if agent.me.boostLevel <=0:
        return efficientMover(agent,target_object,agent.maxSpd)

    location = toLocal(target_object, agent.me)
    controller_state = SimpleControllerState()
    angle_to_target = math.atan2(location.data[1], location.data[0])
    _distance = distance2D(agent.me, target_object)
    #print(math.degrees(angle_to_target))

    current_vel = getVelocity(agent.me.velocity)
    steering, slide = newSteer(angle_to_target)
    #steering, slide = slideSteer(angle_to_target,_distance)

    controller_state.steer = steering
    controller_state.handbrake = slide
    currentSpd = agent.getCurrentSpd()

    if targetSpd >=2200:
        maxSpeed = 2300
        if _distance < 800:
            maxSpeed-= (abs(steering*.5))*maxSpeed

        #throttle
        if maxSpeed > current_vel:
            controller_state.throttle = 1.0
            if agent.onSurface and not slide:
                if maxSpeed > 1400 and current_vel < 2200:
                    controller_state.boost = True
        elif maxSpeed < current_vel:
            controller_state.throttle = -.1

    else:
        if currentSpd < targetSpd:
            controller_state.throttle = 1.0
            if not slide:
                controller_state.boost = True

        elif currentSpd > targetSpd:
            controller_state.boost = False
            controller_state.throttle = -1

    return controller_state

def timeDelayedMovement(agent,targetVec,delay):
    targetDistance = distance2D(agent.me.location,targetVec)
    currentSpd = agent.getCurrentSpd()
    if currentSpd == 0:
        currentSpd = 0.0001
    currentTimeToTarget = targetDistance/currentSpd
    if delay == 0:
        optimalspeed = 2200
    else:
        optimalspeed = targetDistance/delay
    if currentTimeToTarget > delay:
        if optimalspeed > 1400:
            return testMover(agent, targetVec, 2200)
        else:
            return efficientMover(agent, targetVec, 2200)

    else:
        if targetDistance <= 500:
            return efficientMover(agent,targetVec,optimalspeed)
        else:
            return efficientMover(agent, targetVec, 2200)

def efficientMover(agent,target_object,target_speed):
    controller_state = SimpleControllerState()
    location = toLocal(target_object, agent.me)
    angle_to_target = math.atan2(location.data[1], location.data[0])
    _distance = distance2D(agent.me, target_object)
    current_speed = getVelocity(agent.me.velocity)
    steerDirection, slideBool = newSteer(angle_to_target)
    if current_speed < target_speed:
        if _distance > 2500:
            if abs(math.degrees(angle_to_target)) <= clamp(5,0,_distance/1000):
                if agent.onSurface:
                    agent.setJumping(1)

    controller_state.steer = steerDirection
    controller_state.handbrake =  slideBool
    controller_state.throttle = 1
    if not slideBool:
        if _distance < 150:
            if abs(angle_to_target) >1.75:
                controller_state.throttle = 0.5


    if current_speed > target_speed:
        controller_state.throttle = -1

    return controller_state


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


def findSuitableBallPosition(agent, heightMin):
    if agent.ballPred is not None:
        for i in range(0, agent.ballPred.num_slices):
            if agent.ballPred.slices[i].physics.location.z <= heightMin:
                return agent.ballPred.slices[i]

    return None #agent.ballPred[359]

def findSoonestBallTouchable(agent):
    bestStruct = None
    quickest = 99999999
    spd = clamp(2400, 200, agent.getCurrentSpd())
    if spd == 0:
        spd = 0.00001

    if agent.ballPred is not None:
        for i in range(0, agent.ballPred.num_slices):
            if agent.ballPred.slices[i].physics.location.z <= 140:
                distance = distance2D(agent.me.location,Vector([agent.ballPred.slices[i].physics.location.x,
                                                                agent.ballPred.slices[i].physics.location.y,agent.ballPred.slices[i].physics.location.z]))
                timeEstimate = distance/spd
                if timeEstimate < quickest:
                    bestStruct = agent.ballPred.slices[i]
                    quickest = timeEstimate
        return bestStruct
    else:
        return None

def findSuitableBallPosition2(agent, heightMin,speed,origin):
    applicableStructs = []
    speed = clamp(2300,1400,speed)
    if agent.ballPred is not None:
        for i in range(0, agent.ballPred.num_slices):
            if agent.ballPred.slices[i].physics.location.z <= heightMin:
                applicableStructs.append(agent.ballPred.slices[i])

    for pred in applicableStructs:
        distance = distance2D(origin,Vector([pred.physics.location.x,pred.physics.location.y,pred.physics.location.z]))
        if distance < speed * (pred.game_seconds - agent.gameInfo.seconds_elapsed):
            return pred

    return agent.ballPred.slices[1]

def ballHeadedTowardsMyGoal(agent):
    myGoal = Vector([0, 5100 * sign(agent.team), 200])
    if (distance1D(myGoal, agent.ball.location, 1)  - distance1D(myGoal, agent.ball.location + agent.ball.velocity, 1)) > agent.deltaTime/2:
        return True
    #if distance1D(myGoal, agent.ball.location+agent.ball.velocity,1) < distance1D(myGoal,agent.ball,1):
        #return True

    return False

def openGoalOpportunity(agent):
    enemyGoal = Vector([0, 5100 * -sign(agent.team), 200])
    ballDistance = distance2D(agent.ball.location,enemyGoal)

    for e in agent.enemies:
        if distance2D(e.location,enemyGoal) < ballDistance:
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













