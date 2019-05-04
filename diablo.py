from Utilities import *
from States import *
from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.game_state_util import GameState, BallState, CarState, Physics, Vector3 as vector3, Rotator
import time

class diabloBot(BaseAgent):

    def initialize_agent(self):
        self.controller_state = SimpleControllerState()
        self.me = physicsObject()
        self.ball = physicsObject()
        self.me.team = self.team
        self.allies = []
        self.enemies = []
        self.start = 5
        self.flipStart = 0
        self.flipping = False
        self.controller = None
        self.state = efficientMover
        self.flipTimer = time.time()
        #self.activeState = Chase(self)
        self.activeState = Kickoff(self)
        self.gameInfo = None
        self.onSurface = False
        self.boosts = []
        self.fieldInfo = []
        self.positions = []
        self.deltaList = []
        self.time = 0
        self.deltaTime = 0
        self.maxSpd = 2200
        self.ballPred = []
        self.selectedBallPred = None
        self.ballDelay = 0
        self.renderCalls = []
        self.ballPredObj = None

    def getAvgDelta(self):
        if len(self.deltaList) > 0:
            return sum(self.deltaList)/len(self.deltaList)
        else:
            return 1/60


    def setJumping(self,targetType):
        self.activeState = JumpingState(self,targetType)

    def getAvgSpd(self):
        distance = 0
        cap = 0
        if len(self.positions) >=60:
            cap = 60
        else:
            cap = len(self.deltaList)

        for i in range(cap): #deltaList and positions SHOULD be equal
            if i != 0:
                distance+= distance2D(self.positions[i], self.positions[i-1])

        timeElapsed = sum(self.deltaList[:cap])
        return (distance,timeElapsed)

    def getCurrentSpd(self):
        return Vector(self.me.velocity[:2]).magnitude()

    def updateSelectedBallPrediction(self,ballStruct):
        x = physicsObject()
        x.location = Vector([ballStruct.physics.location.x, ballStruct.physics.location.y, ballStruct.physics.location.z])
        x.velocity = Vector([ballStruct.physics.velocity.x, ballStruct.physics.velocity.y, ballStruct.physics.velocity.z])
        x.rotation = Vector([ballStruct.physics.rotation.pitch, ballStruct.physics.rotation.yaw, ballStruct.physics.rotation.roll])
        x.avelocity = Vector([ballStruct.physics.angular_velocity.x, ballStruct.physics.angular_velocity.y, ballStruct.physics.angular_velocity.z])
        x.local_location = localizeVector(x.location, self.me)
        self.ballPredObj = x




    def preprocess(self, game):
        self.ballPred = self.get_ball_prediction_struct()
        self.players = [self.index]
        car = game.game_cars[self.index]
        self.me.location = Vector([car.physics.location.x, car.physics.location.y, car.physics.location.z])
        self.me.velocity = Vector([car.physics.velocity.x, car.physics.velocity.y, car.physics.velocity.z])
        self.me.rotation = Vector([car.physics.rotation.pitch, car.physics.rotation.yaw, car.physics.rotation.roll])
        self.me.avelocity = Vector([car.physics.angular_velocity.x, car.physics.angular_velocity.y, car.physics.angular_velocity.z])
        self.me.boostLevel = car.boost
        self.onSurface = car.has_wheel_contact
        self.positions.insert(0,self.me.location)
        if len(self.positions) > 200:
            self.positions = self.positions[:200]
        t = time.time()
        self.deltaTime = t-self.time
        self.time = t
        self.deltaList.insert(0,self.deltaTime)
        if len(self.deltaList) > 200:
            self.deltaList = self.deltaList[:200]


        ball = game.game_ball.physics
        self.ball.location = Vector([ball.location.x, ball.location.y, ball.location.z])
        self.ball.velocity = Vector([ball.velocity.x, ball.velocity.y, ball.velocity.z])
        self.ball.rotation = Vector([ball.rotation.pitch, ball.rotation.yaw, ball.rotation.roll])
        self.ball.avelocity = Vector([ball.angular_velocity.x, ball.angular_velocity.y, ball.angular_velocity.z])
        self.me.matrix = rotator_to_matrix(self.me)
        self.ball.local_location = localizeVector(self.ball.location,self.me)
        #print(self.ball.local_location)

        # collects info for all other cars in match, updates objects in self.players accordingly
        self.allies.clear()
        self.enemies.clear()
        for i in range(game.num_cars):
            if i != self.index:
                car = game.game_cars[i]
                _obj = physicsObject()
                _obj.index = i
                _obj.team = car.team
                _obj.location = Vector([car.physics.location.x, car.physics.location.y, car.physics.location.z])
                _obj.velocity = Vector([car.physics.velocity.x, car.physics.velocity.y, car.physics.velocity.z])
                _obj.rotation = Vector([car.physics.rotation.pitch, car.physics.rotation.yaw, car.physics.rotation.roll])
                _obj.avelocity = Vector([car.physics.angular_velocity.x, car.physics.angular_velocity.y, car.physics.angular_velocity.z])
                _obj.boostLevel = car.boost
                _obj.local_location = localizeVector(_obj,self.me)

                if car.team == self.team:
                    self.allies.append(_obj)
                else:
                    self.enemies.append(_obj)
        self.gameInfo = game.game_info
        #print(self.Game.time_delta)


        #boost info
        self.boosts.clear()
        self.fieldInfo = self.get_field_info()
        for index in range(len(self.fieldInfo.boost_pads)):
            packetBoost = game.game_boosts[index]
            fieldInfoBoost = self.fieldInfo.boost_pads[index]
            self.boosts.append(Boost_obj([fieldInfoBoost.location.x,fieldInfoBoost.location.y,fieldInfoBoost.location.z],fieldInfoBoost.is_full_boost, packetBoost.is_active))

        #print(self.getCurrentSpd())

    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        self.preprocess(packet)
        stateManager(self)
        action = self.activeState.update()

        self.renderer.begin_rendering()
        self.renderer.draw_string_2d(100, 100, 1, 1, str(type(self.activeState)), self.renderer.white())

        for each in self.renderCalls:
            each.run()
        self.renderer.end_rendering()
        self.renderCalls.clear()

        return action


