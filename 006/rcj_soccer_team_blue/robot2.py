# rcj_soccer_player controller - ROBOT B2

# Feel free to import built-in libraries
import math
from time import process_time_ns  # noqa: F401

# You can also import scripts that you put into the folder with controller
from utils import *
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP


class MyRobot2(RCJSoccerRobot):
    def run(self):
        ball_pos_data = [[0,0], [0,0], [0,0], [0,0], [0,0], [0,0]]
        team_pos = [[0,0], [0,0], [0,0]]
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                last_pos = team_pos[self.player_id-1]
                data = self.get_new_data()
                heading = self.get_compass_heading()
                heading_rad = get_compass_heading_radian(heading)
                robot_pos = self.get_gps_coordinates()
                sonar_values = self.get_sonar_values() 
                team_pos[self.player_id-1] = robot_pos


                if self.is_new_ball_data():
                    ball_data = self.get_new_ball_data()
                    ball_vector = ball_data["direction"]
                    ball_strength = ball_data["strength"]
                    ball_pos = get_ball_pos([ball_vector[0], -ball_vector[1]],ball_strength,heading, robot_pos)
                    wall(ball_pos, self.team)
                    ball_pos_data = [ball_pos] + ball_pos_data[:-1]
                    self.send_data_to_team(self.player_id, ball_pos, robot_pos)
                
                while self.is_new_team_data():
                    team_data = self.get_new_team_data()
                    wall(ball_pos, self.team)
                    ball_pos_data = [team_data['ball_pos']] + ball_pos_data[:-1]
                    team_pos[team_data["robot_id"]-1] = team_data["robot_pos"]
                
                get_vel = state_manager(ball_pos_data, team_pos, heading, self.player_id, self.team, heading_rad) 

                if data['waiting_for_kickoff']:
                    get_vel = (0,0)
                
                left_speed = get_vel[0]
                right_speed = get_vel[1]

                # Set the speed to motors
                self.left_motor.setVelocity(left_speed)
                self.right_motor.setVelocity(right_speed)
