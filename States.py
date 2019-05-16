from Utilities import *
import time
import math
from rlbot.agents.base_agent import SimpleControllerState
import random

class baseState:
    def __init__(self, agent):
        #generic useful objects
        self.agent = agent
        self.active = True

class State:
    RESET = 0
    WAIT = 1
    INITIALIZE = 2
    RUNNING = 3




class GetBoost(baseState):
    def update(self):
        return saferBoostGrabber(self.agent)


class JumpingState(baseState):
    def __init__(self,agent, targetCode): #0 for ball, 1 for forward...
        self.agent = agent
        self.active = True
        self.targetCode = targetCode
        self.flip_obj = FlipStatus()

    def update(self):
        controller_state = SimpleControllerState()
        jump = flipHandler(self.agent, self.flip_obj)
        if jump:
            if self.targetCode == 1:
                controller_state.pitch = -1
                controller_state.steer = 0

            elif self.targetCode == 0:
                #print("How can I flip towards a ball when you don't tell me where it is?!?!?!")
                ball_local = toLocal(self.agent.ball.location, self.agent.me)
                ball_angle = math.atan2(ball_local.data[1], ball_local.data[0])
                #print(ball_angle,math.cos(ball_angle))
                controller_state.jump = True
                controller_state.yaw = math.sin(ball_angle)
                controller_state.pitch = -math.cos(ball_angle) #-abs(math.cos(ball_angle))
            elif self.targetCode == 2:
                controller_state.pitch = 0
                controller_state.steer = 0
                controller_state.yaw = 0
            if self.targetCode == 3:
                controller_state.pitch = 1
                controller_state.steer = 0

        controller_state.jump = jump
        controller_state.boost = False
        controller_state.throttle = 1
        if self.flip_obj.flipDone:
            self.active = False

        return controller_state


class gettingPhysical(baseState):
    def update(self):
        action = demoMagic(self.agent)
        if action != None:
            return action
        else:
            return saferBoostGrabber(self.agent)


class GroundShot(baseState):
    #shoot while ball grounded
    def __init__(self, agent):
        self.agent = agent
        self.active = True

    def update(self):
        return lineupShot(self.agent,3)

class Dribble(baseState):
    #shoot while ball grounded
    def __init__(self, agent):
        #generic useful objects
        self.agent = agent
        self.active = True

    def update(self):
        if not kickOffTest(self.agent):
            #return boostHungry(self.agent)
            return lineupShot(self.agent,1)
        else:
            self.active = False
            #print("Retiring groundshot")
            self.agent.activeState = Kickoff(self.agent)
            return self.agent.activeState.update()


class GroundDefend(baseState):
    def update(self):
        if not kickOffTest(self.agent):
            return defendGoal(self.agent)
        else:
            self.active = False
            #print("Retiring groundshot")
            self.agent.activeState = Kickoff(self.agent)
            return self.agent.activeState.update()


class AerialDefend(baseState):
    #defend while ball is in air
    pass




class Obstruct(baseState):
    #stay on the direct opposite side of the ball from the opposition
    def update(self):
        if not kickOffTest(self.agent):
            return turtleTime(self.agent)

        else:
            self.active = False
            #print("Retiring turtleMode")
            self.agent.activeState = Kickoff(self.agent)
            return self.agent.activeState.update()

class Kickoff(baseState):
    def __init__(self,agent):
        self.agent = agent
        self.started = False
        self.firstFlip = False
        self.secondFlip = False
        self.finalFlipDistance = 900
        self.active = True
        self.startTime = time.time()
        self.flipState = None

    def retire(self):
        self.active = False
        self.agent.activeState = None
        self.flipState = None
        #stateManager(self.agent)

    def update(self):
        spd = self.agent.getCurrentSpd()
        if self.flipState != None:
            if self.flipState.active:
                controller = self.flipState.update()
                if time.time() - self.flipState.flip_obj.flipStartedTimer <= 0.15:
                    if spd < 2200:
                        controller.boost = True
                return controller
            if self.secondFlip:
                self.retire()

        jumping = False
        ballDistance = findDistance(self.agent.me.location, self.agent.ball.location)

        if not self.started:
            if not kickOffTest(self.agent):
                self.started = True
                self.startTime = time.time()

        if self.started and time.time() - self.startTime > 2.5:
            self.retire()

        if not self.firstFlip:
            if spd > 1100:
                self.flipState = JumpingState(self.agent,1)
                self.firstFlip = True
                return self.flipState.update()

        if ballDistance > self.finalFlipDistance:
            return greedyMover(self.agent, self.agent.ball)

        else:
            self.flipState = JumpingState(self.agent,1)
            self.secondFlip = True
            return self.flipState.update()

class aerialRecovery(baseState):
    def update(self):
        if self.agent.onSurface or self.agent.me.location[2] < 100:
            self.active = False
        # self.me.rotation = Vector([car.physics.rotation.pitch, car.physics.rotation.yaw, car.physics.rotation.roll])
        controller_state = SimpleControllerState()
        if self.agent.me.rotation[0] > 0:
            controller_state.pitch = -1

        elif self.agent.me.rotation[0] < 0:
            controller_state.pitch = 1

        if self.agent.me.rotation[2] > 0:
            controller_state.roll = -1

        elif self.agent.me.rotation[2] < 0:
            controller_state.roll = 1

        # if self.agent.me.rotation[0] > self.agent.me.velocity[0]:
        #     controller_state.yaw = -1
        #
        # elif self.agent.me.rotation[0] < self.agent.me.velocity[0]:
        #     controller_state.yaw = 1

        if self.agent.me.rotation[0] > self.agent.velAngle:
            controller_state.yaw = -1

        elif self.agent.me.rotation[0] < self.agent.velAngle:
            controller_state.yaw = 1


        controller_state.throttle = 1

        return controller_state



class halfFlip(baseState):
    def __init__(self,agent):
        self.agent = agent
        self.active = True
        self.firstJump= False
        self.secondJump = False
        self.jumpStart = 0
        self.timeCreated = time.time()
        #print("half flipping!!!")


    def update(self):
        controller_state = SimpleControllerState()
        if not self.firstJump:
            controller_state.throttle = -1
            controller_state.jump = True
            controller_state.pitch = 1
            self.firstJump = True
            self.jumpStart = time.time()
            return controller_state

        elif self.firstJump and not self.secondJump:
            jumpTimer = time.time() - self.jumpStart
            controller_state.throttle = -1
            controller_state.pitch = 1
            controller_state.jump = False
            if jumpTimer < 0.12:
                controller_state.jump = True
            if jumpTimer > 0.15:
                controller_state.jump = True
                self.jumpStart = time.time()
                self.secondJump = True
            return controller_state

        elif self.firstJump and self.secondJump:
            timer = time.time() - self.jumpStart
            if timer < 0.15:
                controller_state.throttle = -1
                controller_state.pitch = 1

            else:
                controller_state.pitch = -1
                controller_state.throttle = 1
                controller_state.roll = 1

            if timer > .8:
                controller_state.roll = 0
            if timer > 1.15:
                self.active = False
            return controller_state

        else:
            print("halfFlip else conditional called in update. This should not be happening")



class Chase(baseState):
    def __init__(self, agent):
        self.agent = agent
        self.active = True

    def update(self):
        if not kickOffTest(self.agent):
            return efficientMover(self.agent,self.agent.ball,self.agent.maxSpd)
        else:
            self.active = False
            self.agent.activeState = Kickoff(self.agent)
            #print("switchign to kickoff")
            return self.agent.activeState.update()

class backMan(baseState):
    def update(self):
        return backmanDefense(self.agent)


class secondMan(baseState):
    def update(self):
        return secondManSupport(self.agent)




def alteredStateManager(agent):
    agentType = type(agent.activeState)
    if agentType == JumpingState:
        if agent.activeState.active != False:
            return
    if agentType != gettingPhysical:
        agent.activeState = gettingPhysical(agent)

def halfFlipStateManager(agent):
    if agent.activeState.active == False:
        agent.activeState = halfFlip(agent)

    else:
        if type(agent.activeState) != halfFlip:
            agent.activeState = halfFlip(agent)


def simpleStateManager(agent):
    agentType = type(agent.activeState)

    if agentType != Kickoff:
        if not kickOffTest(agent):
            myGoalLoc = center = Vector([0, 5150 * sign(agent.team), 200])

            ballDistanceFromGoal = distance2D(myGoalLoc,agent.ball)
            carDistanceFromGoal = distance2D(myGoalLoc,agent.me)

            timeTillBallReady = 9999
            agent.ballDelay = 6

            #ballStruct = findSuitableBallPosition(agent, 125)
            #ballStruct = findSoonestBallTouchable(agent)
            ballStruct =  findSuitableBallPosition2(agent, 150,agent.getCurrentSpd(),agent.me.location)
            agent.selectedBallPred = ballStruct
            goalward = ballHeadedTowardsMyGoal(agent)
            openNet = openGoalOpportunity(agent)

            if ballStruct != None:
                if ballStruct == agent.ballPred.slices[0]:
                    timeTillBallReady = 0
                else:
                    timeTillBallReady = ballStruct.game_seconds - agent.gameInfo.seconds_elapsed
                #print(timeTillBallReady)
            else:
                timeTillBallReady = 0
            agent.ballDelay = timeTillBallReady


            if agentType == JumpingState:
                if agent.activeState.active != False:
                    return

            if agentType == halfFlip:
                if agent.activeState.active != False:
                    return

            if agentType == aerialRecovery:
                if agent.activeState.active != False:
                    return

            if not agent.onSurface:
                if agent.me.location[2] > 150:
                    if agentType != aerialRecovery:
                        agent.activeState = aerialRecovery(agent)
                        return

            if ballDistanceFromGoal < 2500:
                if agentType != GroundDefend:
                    agent.activeState = GroundDefend(agent)

            elif carDistanceFromGoal > ballDistanceFromGoal+50:
                if agentType != GroundDefend:
                    agent.activeState = GroundDefend(agent)

            elif goalward:
                if agentType != GroundDefend:
                    agent.activeState = GroundDefend(agent)


            else:
                if openNet:
                    if agentType != Dribble:
                        agent.activeState = Dribble(agent)
                    return

                elif challengeDecider(agent):
                    if agentType != Dribble:
                        agent.activeState = Dribble(agent)
                    return

                # elif timeTillBallReady > 2.5:
                #     if agent.me.boostLevel <= 25:
                #         if agentType != GetBoost:
                #             agent.activeState = GetBoost(agent)
                #         return

                if agentType != Dribble:
                    agent.activeState = Dribble(agent)


        else:
            agent.activeState = Kickoff(agent)

def teamStateManager(agent):
    agentType = type(agent.activeState)

    if agentType != Kickoff:
        if not kickOffTest(agent):
            myGoalLoc = center = Vector([0, 5150 * sign(agent.team), 200])

            ballDistanceFromGoal = distance2D(myGoalLoc, agent.ball)

            carDistancesFromGoal = []
            for c in agent.allies:
                carDistancesFromGoal.append(distance2D(myGoalLoc, c.location))

            carDistanceFromGoal = distance2D(myGoalLoc, agent.me)

            cardistancesFromBall = []
            for c in agent.allies:
                cardistancesFromBall.append(distance2D(agent.ball.location, c.location))

            carDistanceFromBall = distance2D(agent.me.location,agent.ball.location)



            timeTillBallReady = 9999
            agent.ballDelay = 6
            ballStruct = findSuitableBallPosition2(agent, 150, agent.getCurrentSpd(), agent.me.location)
            agent.selectedBallPred = ballStruct


            if ballStruct != None:
                if ballStruct == agent.ballPred.slices[0]:
                    timeTillBallReady = 0
                else:
                    timeTillBallReady = ballStruct.game_seconds - agent.gameInfo.seconds_elapsed
            else:
                timeTillBallReady = 0
            agent.ballDelay = timeTillBallReady

            if agentType == JumpingState:
                if agent.activeState.active != False:
                    return

            if agentType == halfFlip:
                if agent.activeState.active != False:
                    return

            if agentType == aerialRecovery:
                if agent.activeState.active != False:
                    return

            if not agent.onSurface:
                if agent.me.location[2] > 150:
                    if agentType != aerialRecovery:
                        agent.activeState = aerialRecovery(agent)
                        return

            if len(agent.allies) < 1:
                soloStateManager(agent)
                return

            elif len(agent.allies) == 1:
                if carDistanceFromBall< cardistancesFromBall[0]:
                    if agentType != Dribble:
                        agent.activeState = Dribble(agent)
                    return

                else:
                    # if ballDistanceFromGoal >7000:
                    #     if agentType != gettingPhysical:
                    #         agent.activeState = gettingPhysical(agent)
                    #     return
                    # else:
                    if agentType != backMan:
                        agent.activeState = backMan(agent)
                    return

            else:
                if carDistanceFromBall < min(cardistancesFromBall):
                    if agentType != Dribble:
                        agent.activeState = Dribble(agent)
                    return

                # if ballDistanceFromGoal > 6500:
                #     if carDistanceFromGoal > min(carDistancesFromGoal):
                #         if agentType != gettingPhysical:
                #             agent.activeState = gettingPhysical(agent)
                #         return
                #     else:
                #         if agentType != secondMan:
                #             agent.activeState = secondMan(agent)

                else:
                    if carDistanceFromGoal <= min(carDistancesFromGoal):
                        if agentType != backMan:
                            agent.activeState = backMan(agent)
                        return
                    else:
                        if ballDistanceFromGoal >= 5000:
                            if agentType != gettingPhysical:
                                agent.activeState = gettingPhysical(agent)
                            return
                        else:
                            if agentType != backMan:
                                agent.activeState = backMan(agent)
                            return

        else:
            agent.activeState = Kickoff(agent)

def soloStateManager(agent):
    agentType = type(agent.activeState)

    if agentType != Kickoff:
        if not kickOffTest(agent):
            myGoalLoc = center = Vector([0, 5150 * sign(agent.team), 200])

            ballDistanceFromGoal = distance2D(myGoalLoc,agent.ball)
            carDistanceFromGoal = distance2D(myGoalLoc,agent.me)

            timeTillBallReady = 9999
            agent.ballDelay = 6

            #ballStruct = findSuitableBallPosition(agent, 125)
            #ballStruct = findSoonestBallTouchable(agent)
            ballStruct =  findSuitableBallPosition2(agent, 150,agent.getCurrentSpd(),agent.me.location)
            agent.selectedBallPred = ballStruct
            goalward = ballHeadedTowardsMyGoal(agent)
            openNet = openGoalOpportunity(agent)

            if ballStruct != None:
                if ballStruct == agent.ballPred.slices[0]:
                    timeTillBallReady = 0
                else:
                    timeTillBallReady = ballStruct.game_seconds - agent.gameInfo.seconds_elapsed
                #print(timeTillBallReady)
            else:
                timeTillBallReady = 0
            agent.ballDelay = timeTillBallReady


            if agentType == JumpingState:
                if agent.activeState.active != False:
                    return

            if agentType == halfFlip:
                if agent.activeState.active != False:
                    return

            if agentType == aerialRecovery:
                if agent.activeState.active != False:
                    return

            if not agent.onSurface:
                if agent.me.location[2] > 150:
                    if agentType != aerialRecovery:
                        agent.activeState = aerialRecovery(agent)
                        return


            if ballDistanceFromGoal < 2500:
                if agentType != GroundDefend:
                    agent.activeState = GroundDefend(agent)


            elif carDistanceFromGoal > ballDistanceFromGoal+50:
                if agentType != GroundDefend:
                    agent.activeState = GroundDefend(agent)

            elif goalward:
                if agentType != GroundDefend:
                    agent.activeState = GroundDefend(agent)


            else:
                #if timeTillBallReady < 2:
                if openNet:
                    if agentType != Dribble:
                        agent.activeState = Dribble(agent)
                    return

                elif challengeDecider(agent):
                    if agentType != Dribble:
                        agent.activeState = Dribble(agent)
                    return


                elif agent.me.boostLevel >= 25:
                    if agentType != Dribble:
                        agent.activeState = Dribble(agent)

                else:
                    if agentType != Dribble:
                        agent.activeState = Dribble(agent)


                # else:
                #     if agentType != GetBoost:
                #         agent.activeState = GetBoost(agent)

        else:
            agent.activeState = Kickoff(agent)

