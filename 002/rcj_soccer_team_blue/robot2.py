import math
import pickle
import random
from enum import Enum
from dataclasses import dataclass
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP

@dataclass
class State:
    move_loop = 0
    state_vertical = 0
    state_horizental = 0
    state_not = 0
    relocate = 0

@dataclass
class Time:
    time_value = 30
    second = time_value
    time_stuck = second * 10
    time_not = second * 5
    time_moving = second * 2
    time_relocate = second * 5
    dande_be_dande = 0
    time_party = second * 2
    time_sos = second * 2
    one_sec = TIME_STEP / 1000.0 

@dataclass
class Robot:
    id = 0
    x = 0.0
    y = 0.0
    role = ""
    place = 0
    dist_list = []
    role_list = []
    sonar = (0,0)
    speed = 0.0
    angle = 0.0
    angle_2 = 0.0
    speed = 0.0
    last_x = 0.0
    last_y = 0.0
    relocate_X_last = 0.0
    relocate_Y_last = 0.0
    dist = 0.0
    angle_radian = 0.0
    sos = False
    error = 0.0
    previous_error = 0.0
    integral = 0.0
    relocate_sos = False

@dataclass
class Game:
    goal_blue = [0, 0.7]
    goal_yellow = [0, -0.7]
    neutral_spot = []
    wall_right = 0.0 
    wall_top = 0.0
    wall_left = 0.0
    wall_buttom = 0.0
    is_kickoff = False
    penalty_area_y_blue = 0.0
    penalty_area_x_end_blue = 0.0
    penalty_area_x_start_blue = 0.0
    penalty_area_y_yellow = 0.0
    penalty_area_x_end_yellow = 0.0
    penalty_area_x_start_yellow = 0.0
    attacker_x = 0.0
    attacker_y = 0.0
    fred_x = 0
    fred_y = 0 
    goldan_x = 0
    goldan_y = 0
    attacker_x_last = 0.0
    attacker_y_last = 0.0

@dataclass
class Ball:
    x = 0.0
    y = 0.0
    c_ball = True
    angle = 0.0
    angle_2 = 0.0
    speed = 0.0
    last_x = 0.0
    last_y = 0.0
    strength = 0.0
    be_robot_x = 0.0
    be_robot_y = 0.0
    angle_s = 0.0
    s_2 = 0.0
    next_x = 0.0
    next_y = 0.0
    positions_ball_x = []
    positions_ball_y = []

class Role(Enum):
    ATTACKER = "Attacker"
    FRED = "Fred"
    GOLDAN = "Goldan"

@dataclass
class Goldan_Relocate:
    last_x = 0
    last_y = 0
    get_out = False
    run_away_time = 0
    counting = 0
    counting_penalty = 0
    counting_penalty_state = "stop"
    mojasame = 0
    mojasame_last_x = 0
    mojasame_last_y = 0
    counting_mojasame = 0
    mojasame_last_time = 0
    mojasame_get_out = False
    counting_mojasame_state = "stop"    

class Vali(Enum):
    TOJ_STRENGTH = 150
    RUN = "run"
    STOP = "stop"

class MyRobot2(RCJSoccerRobot):
    def mapping(self):
        unpacked_2 = None
        unpacked_3 = None

        unpacked_2, unpacked_3 = self.receive_data()
        if unpacked_2 is not None:
            self.robot_data_update_2(unpacked_2)
            self.ball_data_update_2(unpacked_2)
        if unpacked_3 is not None:
            self.robot_data_update_3(unpacked_3)
            self.ball_data_update_3(unpacked_3)

        self.robot_data_update_1()
        self.ball_data_update_1()
        self.game_update()
        game.attacker_x, game.attacker_y = self.attacker_coor()
        game.fred_x, game.fred_y = self.fred_coor()
        game.goldan_x, game.goldan_y = self.goldan_coor()
        if ball_1.c_ball == False:
            ball_1.x, ball_1.y = self.send_help()

    def robot_data_update_1(self):
        r_1.id = self.player_id
        r_1.last_x = r_1.x
        r_1.last_y = r_1.y
        pos = self.get_gps_coordinates()
        r_1.x = pos[0] if self.name[0] == "B" else -pos[0]
        r_1.y = pos[1] if self.name[0] == "B" else -pos[1]
        r_1.angle_radian = self.get_compass_heading()
        r_1.angle = self.norm_ang(math.degrees(r_1.angle_radian) + 90)
        r_1.angle_2 = r_1.angle_radian
        if r_1.angle_2 < 0:
            r_1.angle_2 = 2 * math.pi + r_1.angle_2
        r_1.angle_2 = math.degrees(r_1.angle_2)
        #r_1.role
        #r_1.speed

    def robot_data_update_2(self, packet):
        r_2.id = packet['player_id']
        r_2.last_x = r_2.x
        r_2.last_y = r_2.y
        r_2.x = packet['robot_x']
        r_2.y = packet['robot_y']
        r_2.angle = packet['robot_angle']
        r_2.angle_2 = packet['robot_angle_2']
        r_1.role = packet['role_2']
        r_2.role = packet['role_1']
        r_3.role = packet['role_3']
        r_1.place = packet['place_2']
        r_2.place = packet['place_1']
        r_3.place = packet['place_3']
        #r_2.role
        #r_2.speed

    def robot_data_update_3(self, packet):
        r_3.id = packet['player_id']
        r_3.last_x = r_3.x
        r_3.last_y = r_3.y
        r_3.x = packet['robot_x']
        r_3.y = packet['robot_y']
        r_3.angle = packet['robot_angle']
        r_3.angle_2 = packet['robot_angle_2']
        #r_2.role
        #r_2.speed

    def ball_data_update_1(self):
        if self.is_new_ball_data():
            ball_data = self.get_new_ball_data()  
            ball_1.strength = ball_data["strength"]
            ball_1.be_robot_x = ball_data["direction"][0]
            ball_1.be_robot_y = ball_data["direction"][1]
            ball_1.c_ball = True
        else:
            ball_1.c_ball = False
            ball_1.strength = 0.0
            ball_1.x = 0
            ball_1.y = 0
            ball_1.next_x = 0
            ball_1.next_y = 0
        ball_1.angle = math.degrees(math.atan2(ball_1.be_robot_x,ball_1.be_robot_y)) - 90

        ball_1.last_x = ball_1.x
        ball_1.last_y = ball_1.y
        ball_1.x , ball_1.y = self.tale_in_the_air()
        ball_1.angle_s = self.norm_ang(r_1.angle - ball_1.angle)
        ball_1.angle_2 = math.atan2(ball_1.be_robot_y, ball_1.be_robot_x)
        if ball_1.angle_2 < 0:
            ball_1.angle_2 = 2 * math.pi + ball_1.angle_2
        ball_1.angle_2 = math.degrees(ball_1.angle_2)
        ball_1.s_2 = (r_1.angle_2 + ball_1.angle_2) % 360
        ball_1.next_x, ball_1.next_y = self.ball_prediction(ball_1)
        #ball_1.x
        #ball_1.y
        #ball_1.next_x
        #ball_1.next_y

        #ball_1.speed

    def ball_data_update_2(self, packet):
        ball_2.strength = packet['strength']
        ball_2.x = packet['ball_x']
        ball_2.y = packet['ball_y']
        ball_2.c_ball = packet['c_ball']
        #ball_2.next_x
        #ball_2.next_y

        #ball_2.speed

    def ball_data_update_3(self, packet):
        ball_3.strength = packet['strength']
        ball_3.x = packet['ball_x']
        ball_3.y = packet['ball_y']
        ball_3.c_ball = packet['c_ball']
        
        #ball_3.next_x
        #ball_3.next_y

        #ball_2.speed

    def send_help(self):
        if ball_2.c_ball:
            return ball_2.x, ball_2.y
        elif ball_3.c_ball:
            return ball_3.x, ball_3.y
        else:
            return 0, 0

    def tale_in_the_air(self):
        if ball_1.c_ball:
            dist = math.sqrt(ball_1.be_robot_x ** 2 + ball_1.be_robot_y ** 2)
            r = 0.11886 * dist - 0.02715
            theta = self.norm_ang(ball_1.s_2 + 90)
            theta = math.radians(theta)
            pos_x = r * math.cos(theta)
            pos_y = r * math.sin(theta)
            be_zamin_x = r_1.x - 2 * pos_x
            be_zamin_y = r_1.y - 2 * pos_y
        else:
            be_zamin_x, be_zamin_y = 0, 0
        return be_zamin_x, be_zamin_y

    def game_update(self):
        if self.is_new_data():
            data = self.get_new_data()
            game.is_kickoff = data['waiting_for_kickoff']

    def norm_ang(self, angle):
        return (angle + 360) % 360

    def receive_data(self):
        unpacked_2 = None
        unpacked_3 = None

        if self.is_new_team_data():
            received_data = self.team_receiver.getData()
            self.team_receiver.nextPacket()
            deserialized_data = pickle.loads(received_data)
            unpacked_2 = deserialized_data

        if self.is_new_team_data():
            received_data = self.team_receiver.getData()
            self.team_receiver.nextPacket()
            deserialized_data = pickle.loads(received_data)
            unpacked_3 = deserialized_data

        return unpacked_2, unpacked_3

    def send_data(self):
        data = {
            'player_id': self.player_id,
            'robot_x': r_1.x,
            'robot_y': r_1.y,
            'robot_angle': r_1.angle,
            'robot_angle_2': r_1.angle_2,
            'strength': ball_1.strength,
            'ball_x': ball_1.x,
            'ball_y': ball_1.y,
            'c_ball': ball_1.c_ball
        }
        serialized_data = pickle.dumps(data)
        self.team_emitter.send(serialized_data)

    def initialazation(self):
        global r_1, r_2, r_3, game
        global ball_1, ball_2, ball_3
        global state, time, goldan_relocate

        r_1 = Robot()
        r_2 = Robot()
        r_3 = Robot()

        ball_1 = Ball()
        ball_2 = Ball()
        ball_3 = Ball()

        game = Game()
        state = State()
        time = Time()
        goldan_relocate = Goldan_Relocate()

    def sosing(self):
        if abs(ball_1.x) > abs(r_1.x) and r_1.y >= 0.35:
            if r_1.place == 1:
                self.go_to(ball_1.x, ball_1.y)
            elif r_1.place == 2:
                self.go_to(ball_1.x - math.copysign(0.1, ball_1.x), ball_1.y)
            else:
                self.go_to(ball_1.x - math.copysign(0.2, ball_1.x), ball_1.y)
        else:
            self.go_to(0, 0.4)

    def robots_dists(self, dest):
        return [self.dist_cal([r.x, r.y], [dest[0], dest[1]]) for r in [r_1, r_2, r_3]]
  
    def dist_cal(self, org, dest):
        return math.sqrt((dest[1] - org[1])**2 + (dest[0] - org[0])**2)

    def attacker_coor(self):
        for r in [r_1, r_2, r_3]:
            if r.role == Role.ATTACKER.value:
                return r.x, r.y
        return 0, 0

    def fred_coor(self):
        for r in [r_1, r_2, r_3]:
            if r.role == Role.FRED.value:
                return r.x, r.y
        return 0, 0

    def goldan_coor(self):
        for r in [r_1, r_2, r_3]:
            if r.role == Role.GOLDAN.value:
                return r.x, r.y
        return 0, 0

    def move(self, left_speed, right_speed):
        left_speed,right_speed = right_speed,left_speed
        left_speed = min(max(left_speed, -10) , 10)
        right_speed = min(max(right_speed, -10) , 10)
        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)
    
    def go_to(self, x_maghsad, y_maghsad):
        angle_to_target = (math.degrees(math.atan2(r_1.y - y_maghsad, r_1.x - x_maghsad)) + 180) % 360
        rel_angle = (angle_to_target - r_1.angle + 360) % 360 - 180
        r_1.error = rel_angle 
        r_1.integral += r_1.error
        derivative = r_1.error - r_1.previous_error
        Kp = 0.5
        Kd = 0.04
        control_signal = Kp * r_1.error + Kd * derivative
        r_1.previous_error = r_1.error
        base_speed = 10
        self.move(base_speed - control_signal, base_speed + control_signal)

    def tom_oo_jerry(self):
        if ball_1.strength >= 150:
            self.go_to(game.goal_yellow[0], game.goal_yellow[1])
        else:
            self.go_to(ball_1.x, ball_1.y)
    
    def scan(self):
        if r_2.role == Role.FRED.value:
            fasele_1 = math.sqrt((0.2 - r_1.y)**2 + (0.5 - r_1.x)**2)
            fasele_2 = math.sqrt((0.2 - r_2.y)**2 + (0.5 - r_2.x)**2)
        else:
            fasele_1 = math.sqrt((0.2 - r_1.y)**2 + (0.5 - r_1.x)**2)
            fasele_2 = math.sqrt((0.2 - r_3.y)**2 + (0.5 - r_3.x)**2)
        if fasele_1 < fasele_2:
            self.move_loop(0.5, 0.2)
        else:
            self.move_loop(0.5, -0.2)

    def move_loop(self, loop_x, loop_y):
        self.move_loop2(loop_x, loop_y)
        if state.move_loop == 1:
            self.go_to(loop_x, loop_y)
        else:
            self.go_to(-loop_x, loop_y )
    
    def move_loop2(self, loop_2_x, loop_2_y):
        if loop_2_x - 0.01 <= r_1.x <= loop_2_x + 0.01 and loop_2_y - 0.01 <= r_1.y <= loop_2_y + 0.01:
            state.move_loop = 0
        elif -loop_2_x - 0.01 <= r_1.x <= -loop_2_x + 0.01 and loop_2_y - 0.01 <= r_1.y <= loop_2_y + 0.01:
            state.move_loop = 1

    def chap_nakon(self):
        Stuck_Dist = (abs(r_1.last_x - r_1.x + r_1.last_y - r_1.y))
        r_1.last_x = r_1.x
        r_1.last_y = r_1.y
        if game.is_kickoff == True:
            return True
        if time.dande_be_dande > 0:
            self.move(-5 , -8)
            time.dande_be_dande -= 1
        else:
            if Stuck_Dist < 0.001 and time.time_stuck > 0:
                time.time_stuck -= 1
                if time.time_stuck == 200:
                    time.dande_be_dande = 2 * time.second
            elif Stuck_Dist > 0.001:
                time.time_stuck = 12 * time.second
                return False
            if time.time_stuck == 0:
                return True
        return False

    def bermoda(self, y, cor_angle, angle_3, r_first):
        corner_l = (math.degrees(math.atan2(y - r_1.y, 0.15 - r_1.x)) + cor_angle) % 360
        corner_r = (math.degrees(math.atan2(y - r_1.y, -0.15 - r_1.x)) + cor_angle) % 360
        Ball_Angle_S_3 = (ball_1.angle_s + angle_3) % 360
        if r_first:
            if corner_r < Ball_Angle_S_3 < corner_l:
                return True
        else:
            if corner_l < Ball_Angle_S_3 < corner_r:
                return True  

    def icu(self):
        if self.bermoda(-0.7, 270, 90, True) or 340 < ball_1.s_2 or ball_1.s_2 < 20:
            self.tom_oo_jerry()
        elif 180 < ball_1.s_2 < 270:
            self.go_to(ball_1.x + 0.17, ball_1.y)
        elif 90 < ball_1.s_2 < 180:
            self.go_to(ball_1.x - 0.17, ball_1.y)
        else:
            self.go_to(ball_1.x, ball_1.y + 0.17)
    
    def sos_goldan(self):
        if time.time_sos <= 0:
            r_1.sos = True
        if (abs(game.goldan_y - ball_1.y) < 0.1 and abs(game.goldan_x) < abs(ball_1.x) and abs(game.goldan_x - ball_1.x) < 0.2 and
        ball_1.y > 0.55 and abs(ball_1.x) > 0.15 and (ball_1.c_ball or ball_2.c_ball or ball_3.c_ball)):
            time.time_sos -= 1
        else:
            time.time_sos = 2 * time.second
            r_1.sos = False

    def hor_ver(self):
        if 180 < ball_1.angle_s <= 360:
            if state.state_horizental == 1 and (ball_1.angle_s < 190 or ball_1.angle_s > 350):
                return True
            else:
                state.state_vertical = 1
                state.state_horizental = 0
                return False
        else:
            if state.state_vertical == 1 and (ball_1.angle_s > 170 or ball_1.angle_s <= 10):
                return False
            else:
                state.state_vertical = 0
                state.state_horizental = 1
                return True

    def goldan(self): 
        if (ball_1.strength >= 50 and ball_1.y < (r_1.y - 0.05) and r_1.y >= 0.2 and (ball_1.c_ball or ball_2.c_ball or ball_3.c_ball)
        and abs(ball_1.x) > (abs(r_1.x) + 0.1)):
            self.tom_oo_jerry()
        elif -0.35 < r_1.x < 0.35 and r_1.y > 0.5:
            if ball_1.c_ball == False:
                if ball_2.c_ball or ball_3.c_ball:
                    self.go_to(ball_1.x, 0.55)
                else:
                    self.move_loop(0.12, 0.55)
            else:
                state.move_loop = 0
                #true ==> hor
                if self.hor_ver():
                    self.go_to(ball_1.x, 0.55+abs(ball_1.y/6))
                else:
                    self.go_to(math.copysign(0.3, ball_1.x), ball_1.y)
        elif -0.4 < r_1.x < 0.4 and r_1.y < 0.4 and 180 < ball_1.y <= 360 and (ball_1.c_ball or ball_2.c_ball or ball_3.c_ball):
            self.icu()
        else:
            self.go_to(0, 0.55)

    def behind(self):
        if game.attacker_y < -0.5:
            self.go_to(game.attacker_x - math.copysign(0.6, game.attacker_x), game.attacker_y)
        else:
            self.go_to(game.attacker_x - math.copysign(0.3, game.attacker_x), game.attacker_y)
    
    def relocate(self):
        if abs(game.attacker_x_last - game.attacker_x) < 0.1 and abs(game.attacker_y_last - game.attacker_y) < 0.1:
            time.time_relocate -= 1
        else:
            time.time_relocate = time.second * 5
            game.attacker_x_last = game.attacker_x
            game.attacker_y_last = game.attacker_y
            return False
        if time.time_relocate <= 0:
            return True
        return False

    def un_yeki(self):
        if r_1.sos:
            self.sosing()
        elif r_1.y > 0.45 and abs(r_1.x) < 0.3:
            self.go_to(math.copysign(0.5, game.goldan_x), 0.4)
        else:
            if 0.15 < abs(game.attacker_x) and game.attacker_y < -0.4 and ball_1.y < -0.4:
                time.time_party -= 1
                if time.time_party < 0:
                    self.disco_party()
            else:
                time.time_party = time.second * 2
                if r_2.role == Role.ATTACKER.value or r_3.role == Role.ATTACKER.value:
                    if self.relocate():
                        self.go_to(0, 0.1)
                    else:
                        self.behind()
                else:
                    if ball_2.c_ball or ball_3.c_ball:
                        self.go_to(ball_1.x, ball_1.y)
                    else:
                        self.scan()

    def disco_party(self):
        if abs(game.fred_x) < abs(game.attacker_x) and r_1.y <= -0.45:
            self.go_to(game.attacker_x, -0.45)
        else:
            self.go_to(game.attacker_x, game.attacker_y)

    def be_samte_khoda(self):
        if self.bermoda(0.7, 90, -90, False) and ball_1.strength >= 50:
            if r_1.y > 0.4:
                if 0 < r_1.angle <= 180:
                    self.move(5, 5)
                else:
                    self.move(-5, -5)
            else:
                self.icu()
            return True
        return False

    def vazaief(self):
        if r_1.role == Role.GOLDAN.value:
            self.goal_keeper()
        elif r_1.role == Role.ATTACKER.value:
            self.attacker()
        else:
            self.un_yeki()

    def attacker(self):
        if r_1.sos:
            self.sosing()
        elif r_1.y > 0.45 and abs(r_1.x) < 0.3:
            self.go_to(math.copysign(0.5, game.goldan_x), 0.4)
        else:
            self.icu()

    def list_avr(self, list):
        return sum(list) / len(list)

    def abr(self, value, counter):
        return value / counter

    def ball_prediction(self, ball: Ball):
        total_x_distance = 0.0
        ball.positions_ball_x.append(ball.x)
        total_y_distance = 0.0
        ball.positions_ball_y.append(ball.y)

        cycles = 8 #6
        num_points = 4 #3 
        ball.positions_ball_x = ball.positions_ball_x[-50:]
        ball.positions_ball_y = ball.positions_ball_y[-50:]
        
        if len(ball.positions_ball_x) > num_points and abs(r_1.y - ball_1.y) < 0.2:            
            for i in range(-1, -(num_points + 1), -1):  
                dx = ball.positions_ball_x[i] - ball.positions_ball_x[i - 1]
                total_x_distance += abs(dx)
                dy = ball.positions_ball_y[i] - ball.positions_ball_y[i - 1]
                total_y_distance += abs(dy)

            average_x_speed = self.abr(total_x_distance, num_points - 1)
            average_y_speed = self.abr(total_y_distance, num_points - 1)

            next_x = ball.x + average_x_speed * cycles
            next_y = ball.y + average_y_speed * cycles
        else:
            next_x = ball.x
            next_y = ball.y
        return next_x, next_y

    def gir_nakon(self):
        if r_1.role == Role.GOLDAN.value or r_1.sos == True:
            return False
        dist1 = self.dist_cal([r_1.x, r_1.y], [r_2.x, r_2.y])
        dist2 = self.dist_cal([r_1.x, r_1.y], [r_3.x, r_3.y])
        if (dist1 < 0.15 and (((r_1.x > r_2.x or r_1.y > r_2.y) and -190 < r_1.angle - r_2.angle < -170)
        or ((r_2.x > r_1.x or r_2.y > r_1.y) and -190 < r_2.angle - r_1.angle < -170))):
            base = 3
            error = (dist1 + 0.2) * 25
            self.move(base + error, base - error)
            return True
        elif (dist1 < 0.15 and (((r_1.x > r_3.x or r_1.y > r_3.y) and -190 < r_1.angle - r_3.angle < -170)
        or ((r_3.x > r_1.x or r_3.y > r_1.y) and -190 < r_3.angle - r_1.angle < -170))):
            base = 3
            error = (dist2 + 0.2) * 25
            self.move(base + error, base - error)
            return True
        return False

    def goal_keeper(self):   
        ball_x = ball_1.x
        ball_y = ball_1.y

        '''if ball_x != 0.0:            
            ball_1.last_x = ball_x
            ball_1.last_y = ball_y
        else:
            ball_x = ball_1.last_x + (random.randrange(0,7,1) - random.randrange(0,7,1))/50
            ball_y = ball_1.last_y'''
        
        if ball_1.y > 0.2:
            if -0.25 < ball_x < 0.25 and 0 < ball_1.angle_s <= 180:
                self.go_to(ball_x, 0.55 + abs(ball_y / 6))
            else:
                self.go_to(math.copysign(0.3, ball_x), max(ball_y, 0.55))    
        else:
            self.move_loop(0.2, 0.5) 
    
    def run(self):
        self.initialazation()

        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                self.mapping()
                self.send_data()
                self.sos_goldan()

                if self.chap_nakon() == False:
                    if self.be_samte_khoda() == False:
                        self.vazaief()
                else:
                    self.move(0, 0)