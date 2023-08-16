
# rcj_soccer_player controller - ROBOT B2

# Feel free to import built-in libraries
import math  # noqa: F401

# You can also import scripts that you put into the folder with controller
import utils
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
import time


class MyRobot1(RCJSoccerRobot):
    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                data = self.get_new_data

                while self.is_new_team_data():
                    team_data = self.get_new_team_data()  # noqa: F841
                    # Do something with team data
                
                if self.is_new_ball_data():
                    ball_data = self.get_new_ball_data()
                    # print(ball_data)
                      
                    # ball_data['direction'] = [0,0,0]
                    # print("ball new")
                    # print(  ball_data['direction'])
                   
               
                    # If the robot does not see the ball, stop motors
                    #if ball_data['direction']==[-0.6,0.7,0]or[-0.5,0.7,0]or[0.4,0.7,0]or[0.3,0.7,0]or[0.2,0.7,0]or[0.1,0.7,0]or[0,0.7,0]or[-0.1,0.7,0]or[-0.2,0.7,0]or[-0.3,0.7,0]or[-0.4,0.7,0]or[-0.5,0.7,0]or[-0.6,0.7,0]or[-0.6,0.7,0]or[-0.5,0.7,0]or[0.4,0.7,0]or[0.3,0.7,0]or[0.2,0.7,0]or[0.1,0.7,0]or[0,0.7,0]or[-0.1,0.7,0]or[-0.2,0.7,0]or[-0.3,0.7,0]or[-0.4,0.7,0]or[-0.5,0.7,0]or[-0.6,0.7,0]or[-0.6,0.7,0]or[-0.5,0.7,0]or[0.4,0.7,0]or[0.3,0.7,0]or[0.2,0.7,0]or[0.1,0.7,0]or[0,0.7,0]or[-0.1,0.7,0]or[-0.2,0.7,0]or[-0.3,0.7,0]or[-0.4,0.7,0]or[-0.5,0.7,0]or[-0.6,0.7,0]or[-0.6,0.7,0]or[-0.5,0.7,0]or[0.4,0.7,0]or[0.3,0.7,0]or[0.2,0.7,0]or[0.1,0.7,0]or[0,0.7,0]or[-0.1,0.7,0]or[-0.2,0.7,0]or[-0.3,0.7,0]or[-0.4,0.7,0]or[-0.5,0.7,0]or[-0.6,0.7,0]or[-0.6,0.7,0]or[-0.5,0.7,0]or[0.4,0.7,0]or[0.3,0.7,0]or[0.2,0.7,0]or[0.1,0.,0]or[0,0.3,0]or[-0.1,0.3,0]or[-0.2,0.3,0]or[-0.3,0.,0]or[-0.4,0.3,0]or[-0.5,0.3,0]or[-0.6,0.3,0]:    # کامل کن
                    #       

                    print('ball 0 :' + str(ball_data['direction'][0]))
                
                    # if ball_data['direction'][0] > robot_pos[0]: 
                    #     left_speed = 0
                    #     right_speed = 0
                    #     self.left_motor.setVelocity(left_speed)
                    #     self.right_motor.setVelocity(right_speed)
                    #     continue
                    
                    if ball_data['direction'][0] ==[0.35] or [0.4] or [0.5] or [0.6] or [0.7]:
                        if ball_data['direction'][1] == [0.25] or [0.2] or [0.1] or [0] or [-0.1] or [-0.2] or [-0.25]:
                            left_speed = 0
                            right_speed = 0
                        else:
                           if direction == 0:
                               left_speed = 10
                               right_speed = 10
                           else:
                               left_speed = direction * 10
                               right_speed = direction * -10
                else:
                   while self.is_new_ball_data == False:
                       ball_data['direction'] = [-0.35,0,0]
                       left_speed = 10
                       right_speed = 10
                       time.sleep(0.25)
                       left_speed = -10
                       right_speed = -10
                       time.sleep(0.25)
                   continue
 
                # Get data from compass
                heading = self.get_compass_heading()  # noqa: F841

                # Get GPS coordinates of the robot
                robot_pos = self.get_gps_coordinates()  # noqa: F841
              #  print(robot_pos)

                # Compute the speed for motors
                direction = utils.get_direction(ball_data["direction"])

                # If the robot has the ball right in front of it, go forward,
                # rotate otherwise

                if direction == 0:
                    left_speed = 10
                    right_speed = 10
                else:
                     left_speed = 10 * direction
                     right_speed = -10 * direction
                # Set the speed to motors
                self.left_motor.setVelocity(left_speed)
                self.right_motor.setVelocity(right_speed)

                # Send message to team robots
                self.send_data_to_team(self.player_id)
