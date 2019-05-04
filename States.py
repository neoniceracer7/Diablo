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
        return boostHungry(self.agent)


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
                controller_state.jump = True
                controller_state.yaw = math.sin(ball_angle)
                controller_state.pitch = -abs(math.cos(ball_angle))

        controller_state.jump = jump
        controller_state.boost = False
        controller_state.throttle = 1
        if self.flip_obj.flipDone:
            self.active = False

        return controller_state

class GroundShot(baseState):
    #shoot while ball grounded
    def __init__(self, agent):
        #generic useful objects
        self.agent = agent
        self.active = True
        #print("started groundshot")

    def update(self):
        if not kickOffTest(self.agent):
            #return boostHungry(self.agent)
            return lineupShot(self.agent,5)
        else:
            self.active = False
            #print("Retiring groundshot")
            self.agent.activeState = Kickoff(self.agent)
            return self.agent.activeState.update()

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


class DemolitionMode(baseState):
    #search and destroy
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
        self.finalFlipDistance = 700
        self.active = True
        self.startTime = time.time()
        self.flipState = None

    def retire(self):
        self.active = False
        self.agent.activeState = None
        self.flipState = None
        stateManager(self.agent)

    def update(self):
        spd = self.agent.getCurrentSpd()
        if self.flipState != None:
            if self.flipState.active:
                controller = self.flipState.update()
                if time.time() - self.flipState.flip_obj.flipStartedTimer <= 0.2:
                    if spd < 2200:
                        controller.boost = True
                return controller

        jumping = False
        ballDistance = findDistance(self.agent.me.location, self.agent.ball.location)

        if not self.started:
            if not kickOffTest(self.agent):
                self.started = True
                self.startTime = time.time()

        if self.started and time.time() - self.startTime > 2.5:
            self.retire()

        if not self.firstFlip:
            if spd > 1300:
                self.flipState = JumpingState(self.agent,1)
                self.firstFlip = True
                return self.flipState.update()

        if ballDistance > self.finalFlipDistance:
            return greedyMover(self.agent, self.agent.ball)

        else:
            self.flipState = JumpingState(self.agent,1)
            return self.flipState.update()



class Chase(baseState):
    def __init__(self, agent):
        self.agent = agent
        self.active = True

    def update(self):
        if not kickOffTest(self.agent):
            return efficientMover(self.agent,self.agent.ball,agent.maxSpd)
        else:
            self.active = False
            self.agent.activeState = Kickoff(self.agent)
            #print("switchign to kickoff")
            return self.agent.activeState.update()

def stateManager(agent):
    agentType = type(agent.activeState)

    if agentType != Kickoff:
        if not kickOffTest(agent):
            myGoalLoc = center = Vector([0, 5150 * sign(agent.team), 200])

            ballDistanceFromGoal = distance1D(myGoalLoc,agent.ball,1)
            carDistanceFromGoal = distance1D(myGoalLoc,agent.me,1)

            timeTillBallReady = 9999
            agent.ballDelay = 6

            #ballStruct = findSuitableBallPosition(agent, 125)
            ballStruct =  findSuitableBallPosition2(agent, 140, agent.getCurrentSpd(), agent.me.location)
            agent.selectedBallPred = ballStruct
            goalward = ballHeadedTowardsMyGoal(agent)
            openNet = openGoalOpportunity(agent)

            if ballStruct != None:
                timeTillBallReady = ballStruct.game_seconds - agent.gameInfo.seconds_elapsed
                agent.ballDelay = timeTillBallReady


            if agentType == JumpingState:
                if agent.activeState.active != False:
                    return


            if carDistanceFromGoal > ballDistanceFromGoal+20:
                if agentType != GroundDefend:
                    agent.activeState = GroundDefend(agent)

            elif goalward:
                if agentType != GroundDefend:
                    agent.activeState = GroundDefend(agent)


            else:
                if timeTillBallReady < 2:
                    if agentType != Dribble:
                        agent.activeState = Dribble(agent)

                else:

                    if agent.me.boostLevel >= 25:
                        if openNet:
                            if agentType != Dribble:
                                agent.activeState = Dribble(agent)
                                return

                        if agentType != GroundDefend:
                            agent.activeState = GroundDefend(agent)


                    else:
                        if agentType != GetBoost:
                            agent.activeState = GetBoost(agent)

        else:
            agent.activeState = Kickoff(agent)

