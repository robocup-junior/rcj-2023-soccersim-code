# rcj_soccer_player controller - ROBOT B3

# Feel free to import built-in libraries
import math  # noqa: F401

# You can also import scripts that you put into the folder with controller
import utils
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP

class MyRobot3(RCJSoccerRobot):
    def run(self):
        ball_x_data = []
        ball_y_data = []
        # Game info
        passed_time = 0.032
        predicate_future_seconds = 5
        Goal_y = -0.85
        # Goal_mid_x = 0
        max_spd = 10
        KP_ball_near = 0.4
        Goal_mid_x = 0.1
        Goal_ball_robot_dis = 0.07
        robot_need_goal_precision = 0.1
        area_error = 40
        buffer_zone = 20
        future = True
        
        def motor(left_spd, right_spd) -> None:
            self.left_motor.setVelocity(right_spd)
            self.right_motor.setVelocity(left_spd)
            
        def stop() -> None:
            self.left_motor.setVelocity(0)
            self.right_motor.setVelocity(0)
        
        def compass_car(heading, buffer_zone) -> None:
            if heading > (360 - buffer_zone) or heading < buffer_zone:
                left_speed = 0
                right_speed = 0
            elif heading >= buffer_zone and heading < 180:
                left_speed = -10
                right_speed = 10
            elif heading >= 180 and heading <= (360 - buffer_zone):
                left_speed = 10
                right_speed = -10
            self.left_motor.setVelocity(right_speed)
            self.right_motor.setVelocity(left_speed)
        
        def target_compass_car(heading, target_theta, area_error, buffer_zone) -> None:
            if target_theta > 360 - area_error or target_theta < area_error:
                # print('area 1')
                if heading > (360 - buffer_zone) or heading < buffer_zone:
                    left_spd = 10
                    right_spd = 10
                    motor(left_spd, right_spd)
                elif heading >= buffer_zone and heading < 180:
                    left_spd = -10
                    right_spd = 10
                    motor(left_spd, right_spd)
                elif heading >= 180 and heading <= (360 - buffer_zone):
                    left_spd = 10
                    right_spd = -10
                    motor(left_spd, right_spd)
            elif target_theta >= area_error and target_theta <= 180 - area_error:
                # print('area 2')
                if heading > target_theta - buffer_zone and heading < target_theta + buffer_zone:
                    left_spd = 10
                    right_spd = 10
                    motor(left_spd, right_spd)
                elif heading >= target_theta + buffer_zone and heading < 270:
                    left_spd = -10
                    right_spd = 10
                    motor(left_spd, right_spd)
                elif heading >= 270 or heading <= target_theta - buffer_zone:
                    left_spd = 10
                    right_spd = -10
                    motor(left_spd, right_spd)
            elif target_theta > 180 - area_error and target_theta < 180 + area_error:
                # print('area 3')
                if heading > 180 - buffer_zone and heading < 180 + buffer_zone:
                    left_spd = 10
                    right_spd = 10
                    motor(left_spd, right_spd)
                elif heading > 360 - buffer_zone or heading < buffer_zone:
                    left_spd = -10
                    right_spd = -10
                    motor(left_spd, right_spd)
                elif heading >= 180 + buffer_zone and heading <= 360 - buffer_zone:
                    left_spd = -10
                    right_spd = 10
                    motor(left_spd, right_spd)
                elif heading >= buffer_zone and heading < 180 - buffer_zone:
                    left_spd = 10
                    right_spd = -10
                    motor(left_spd, right_spd)
            elif target_theta >= 180 + area_error and target_theta < 360 - area_error:
                # print('area 4')
                if heading > target_theta - buffer_zone and heading < target_theta + buffer_zone:
                    left_spd = 10
                    right_spd = 10
                    motor(left_spd, right_spd)
                elif heading >= target_theta + buffer_zone or heading < 90:
                    left_spd = -10
                    right_spd = 10
                    motor(left_spd, right_spd)
                elif heading <= target_theta - buffer_zone and heading >= 90:
                    left_spd = 10
                    right_spd = -10
                    motor(left_spd, right_spd)
            
        def predicate_future_ball(predicate_future_seconds, previous, current, passed_time) -> list:
            future_ball_data = []
            for n in range(predicate_future_seconds):
                ball_change = current - previous
                ball_spd = ball_change / passed_time
                future = current + passed_time * ball_spd
                future_ball_data.append(future) 
                # finished predict one future x
                previous = current
                current = future
            return  future_ball_data
        
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                data = self.get_new_data()  # noqa: F841

                # Get GPS coordinates of the robot
                robot_pos = self.get_gps_coordinates()  # noqa: F841
                robot_x = robot_pos[0]
                robot_y = robot_pos[1]
                
                # Get data from compass
                heading = self.get_compass_heading() / math.pi * 180  # noqa: F841
                heading -= 360
                if(heading < 0):
                    heading = -heading
                if heading >= 360:
                    heading -= 360
                
                sonar_values = self.get_sonar_values()  # noqa: F841
                
                while self.is_new_team_data():
                    team_data = self.get_new_team_data()  # noqa: F841
                    # Do something with team data

                if self.is_new_ball_data():
                    # ball info
                    ball_data = self.get_new_ball_data()
                    strength = ball_data["strength"] # s = 1 / r2
                    distance = 1/math.sqrt(strength) # r
                    # int: 0 = forward, -1 = right, 1 = left
                    direction = utils.get_direction(ball_data["direction"])
                    
                    direction_xy = ball_data["direction"]
                    direction_x = direction_xy[0]
                    direction_y = -direction_xy[1]
                    
                    ball_direction_rad = math.atan2(direction_y, direction_x)
                    ball_direction = ball_direction_rad /math.pi  * 180
                    if (ball_direction < 0):
                        ball_direction += 360
                        
                    # abs ball data
                    ball_abs_direction = ball_direction + heading
                    if ball_abs_direction >= 360:
                        ball_abs_direction -= 360
                    ball_abs_direction_rad = ball_abs_direction * math.pi / 180
                    ball_abs_x, ball_abs_y = robot_x - distance * math.sin(ball_abs_direction_rad), robot_y - distance * math.cos(ball_abs_direction_rad)
                    if future == True:
                        ball_x_data.append(ball_abs_x)
                        ball_y_data.append(ball_abs_y)
                        
                        # ball_x_data
                        if len(ball_x_data) > 2:
                            del ball_x_data[0:len(ball_x_data)-2]
                        else:
                            x_data_size = len(ball_x_data)
                        current_x = ball_x_data[x_data_size-1]
                        previous_one_x = ball_x_data[x_data_size-2]
                        
                        # ball_y_data
                        if len(ball_y_data) > 2:
                            del ball_y_data[0:len(ball_y_data)-2]
                        else:
                            y_data_size = len(ball_y_data)
                        current_y = ball_y_data[y_data_size-1]
                        previous_one_y = ball_y_data[y_data_size-2]
                        
                        # future ball xy           
                        ball_futures_x = predicate_future_ball(predicate_future_seconds, previous_one_x, current_x, passed_time)
                        ball_futures_y = predicate_future_ball(predicate_future_seconds, previous_one_y, current_y, passed_time)
                        ball_farthest_future_x = ball_futures_x[len(ball_futures_x)-1]
                        ball_farthest_future_y = ball_futures_y[len(ball_futures_y)-1]
                    
                        # predict_ball_dx = ball_farthest_future_x - current_x
                        # predict_ball_dy = ball_farthest_future_y - current_y
                        # if predict_ball_dx == 0:
                        #     predict_ball_dx = 0.000000000000000000000000000000000000000000000000000000001
                        predict_ball_robot_dx = ball_farthest_future_x - robot_x
                        predict_ball_robot_dy = ball_farthest_future_y - robot_y
                        
                        ball_future_rad = math.atan2(predict_ball_robot_dx, predict_ball_robot_dy)
                        ball_future_theta = ball_future_rad /math.pi * 180
                        ball_future_theta -= 180
                        if (ball_future_theta < 0):
                            ball_future_theta += 360
                            
                        # action
                        
                        future_ball_goal_theta = math.atan2((ball_farthest_future_x - Goal_mid_x), (ball_farthest_future_y - Goal_y)) /math.pi  * 180
                        if (future_ball_goal_theta < 0):
                            future_ball_goal_theta += 360
                        future_ball_goal_slope = (ball_farthest_future_y - Goal_y)/(ball_farthest_future_x - Goal_mid_x)       # dy/dx
                        future_robot_need_goal_y = ball_farthest_future_y + Goal_ball_robot_dis
                        future_robot_need_goal_x = (future_robot_need_goal_y - ball_farthest_future_y)/future_ball_goal_slope + ball_farthest_future_x
                        
                        future_robot_need_goal_x = max(future_robot_need_goal_x,-0.63)
                        future_robot_need_goal_x = min(future_robot_need_goal_x,0.63)
                        future_robot_need_goal_y = max(future_robot_need_goal_y,-0.6)
                        future_robot_need_goal_y = min(future_robot_need_goal_y,0.7)
                        
                        future_robot_need_goal_theta = math.atan2((robot_x - future_robot_need_goal_x), (robot_y - future_robot_need_goal_y)) /math.pi  * 180
                        if (future_robot_need_goal_theta < 0):
                            future_robot_need_goal_theta += 360
                        
                        if (robot_x >= future_robot_need_goal_x - (robot_need_goal_precision + 0.03) and 
                            robot_x <= future_robot_need_goal_x + (robot_need_goal_precision + 0.03) and 
                            robot_y >= future_robot_need_goal_y - robot_need_goal_precision and 
                            robot_y <= future_robot_need_goal_y + robot_need_goal_precision):
                            future_robot_need_goal_xy = True
                            
                            area_error = 40
                            if heading > (360 - area_error) or heading < area_error:
                                left_spd = 10
                                right_spd = 10
                                motor(left_spd, right_spd)
                            elif heading > 180 - area_error and heading < 180 + area_error:
                                left_spd = -10
                                right_spd = -10
                                motor(left_spd, right_spd)
                            elif heading >= area_error and heading <= 180 - area_error:
                                left_spd = -10
                                right_spd = 10
                                motor(left_spd, right_spd)
                            elif heading >= 180 + area_error and heading <= (360 - area_error):
                                left_spd = 10
                                right_spd = -10
                                motor(left_spd, right_spd)
                            # compass_car(heading, buffer_zone + 10)
                        else:
                            future_obot_need_goal_xy = False
                            if future_robot_need_goal_theta > 360 - area_error or future_robot_need_goal_theta < area_error:
                                # print('area 1')
                                if heading > (360 - buffer_zone) or heading < buffer_zone:
                                    left_spd = 10
                                    right_spd = 10
                                    motor(left_spd, right_spd)
                                elif heading >= buffer_zone and heading < 180:
                                    left_spd = -10
                                    right_spd = 10
                                    motor(left_spd, right_spd)
                                elif heading >= 180 and heading <= (360 - buffer_zone):
                                    left_spd = 10
                                    right_spd = -10
                                    motor(left_spd, right_spd)
                            elif future_robot_need_goal_theta >= area_error and future_robot_need_goal_theta <= 180 - area_error:
                                # print('area 2')
                                if heading > future_robot_need_goal_theta - buffer_zone and heading < future_robot_need_goal_theta + buffer_zone:
                                    left_spd = 10
                                    right_spd = 10
                                    motor(left_spd, right_spd)
                                elif heading >= future_robot_need_goal_theta + buffer_zone and heading < 270:
                                    left_spd = -10
                                    right_spd = 10
                                    motor(left_spd, right_spd)
                                elif heading >= 270 or heading <= future_robot_need_goal_theta - buffer_zone:
                                    left_spd = 10
                                    right_spd = -10
                                    motor(left_spd, right_spd)
                            elif future_robot_need_goal_theta > 180 - area_error and future_robot_need_goal_theta < 180 + area_error:
                                # print('area 3')
                                if heading > 200 - buffer_zone and heading < 200 + buffer_zone:
                                    left_spd = 10
                                    right_spd = 10
                                    motor(left_spd, right_spd)
                                elif heading > 20 - buffer_zone and heading < 20 + buffer_zone:
                                    left_spd = -10
                                    right_spd = -10
                                    motor(left_spd, right_spd)
                                elif heading >= 200 + buffer_zone or heading <= 20 - buffer_zone:
                                    left_spd = -10
                                    right_spd = 10
                                    motor(left_spd, right_spd)
                                elif heading >= 20 + buffer_zone and heading < 200 - buffer_zone:
                                    left_spd = 10
                                    right_spd = -10
                                    motor(left_spd, right_spd)
                            elif future_robot_need_goal_theta >= 180 + area_error and future_robot_need_goal_theta < 360 - area_error:
                                # print('area 4')
                                if heading > future_robot_need_goal_theta - buffer_zone and heading < future_robot_need_goal_theta + buffer_zone:
                                    left_spd = 10
                                    right_spd = 10
                                    motor(left_spd, right_spd)
                                elif heading >= future_robot_need_goal_theta + buffer_zone or heading < 90:
                                    left_spd = -10
                                    right_spd = 10
                                    motor(left_spd, right_spd)
                                elif heading <= future_robot_need_goal_theta - buffer_zone and heading >= 90:
                                    left_spd = 10
                                    right_spd = -10
                                    motor(left_spd, right_spd)
                    elif future == False:
                        ball_goal_theta = math.atan2((ball_abs_x - Goal_mid_x), (ball_abs_y - Goal_y)) /math.pi  * 180
                        if (ball_goal_theta < 0):
                            ball_goal_theta += 360
                        ball_goal_slope = (ball_abs_y - Goal_y)/(ball_abs_x - Goal_mid_x)       # dy/dx
                        robot_need_goal_y = ball_abs_y + Goal_ball_robot_dis
                        robot_need_goal_x = (robot_need_goal_y - ball_abs_y)/ball_goal_slope + ball_abs_x
                        
                        robot_need_goal_x = max(robot_need_goal_x,-0.63)
                        robot_need_goal_x = min(robot_need_goal_x,0.63)
                        robot_need_goal_y = max(robot_need_goal_y,-0.6)
                        robot_need_goal_y = min(robot_need_goal_y,0.7)
                        
                        robot_need_goal_theta = math.atan2((robot_x - robot_need_goal_x), (robot_y - robot_need_goal_y)) /math.pi  * 180
                        if (robot_need_goal_theta < 0):
                            robot_need_goal_theta += 360
                        
                        if (robot_x >= robot_need_goal_x - (robot_need_goal_precision + 0.03) and 
                            robot_x <= robot_need_goal_x + (robot_need_goal_precision + 0.03) and 
                            robot_y >= robot_need_goal_y - robot_need_goal_precision and 
                            robot_y <= robot_need_goal_y + robot_need_goal_precision):
                            robot_need_goal_xy = True
                            
                            area_error = 40
                            if heading > (360 - area_error) or heading < area_error:
                                left_spd = 10
                                right_spd = 10
                                motor(left_spd, right_spd)
                            elif heading > 180 - area_error and heading < 180 + area_error:
                                left_spd = -10
                                right_spd = -10
                                motor(left_spd, right_spd)
                            elif heading >= area_error and heading <= 180 - area_error:
                                left_spd = -10
                                right_spd = 10
                                motor(left_spd, right_spd)
                            elif heading >= 180 + area_error and heading <= (360 - area_error):
                                left_spd = 10
                                right_spd = -10
                                motor(left_spd, right_spd)
                            # compass_car(heading, buffer_zone + 10)
                        else:
                            obot_need_goal_xy = False
                            if robot_need_goal_theta > 360 - area_error or robot_need_goal_theta < area_error:
                                # print('area 1')
                                if heading > (360 - buffer_zone) or heading < buffer_zone:
                                    left_spd = 10
                                    right_spd = 10
                                    motor(left_spd, right_spd)
                                elif heading >= buffer_zone and heading < 180:
                                    left_spd = -10
                                    right_spd = 10
                                    motor(left_spd, right_spd)
                                elif heading >= 180 and heading <= (360 - buffer_zone):
                                    left_spd = 10
                                    right_spd = -10
                                    motor(left_spd, right_spd)
                            elif robot_need_goal_theta >= area_error and robot_need_goal_theta <= 180 - area_error:
                                # print('area 2')
                                if heading > robot_need_goal_theta - buffer_zone and heading < robot_need_goal_theta + buffer_zone:
                                    left_spd = 10
                                    right_spd = 10
                                    motor(left_spd, right_spd)
                                elif heading >= robot_need_goal_theta + buffer_zone and heading < 270:
                                    left_spd = -10
                                    right_spd = 10
                                    motor(left_spd, right_spd)
                                elif heading >= 270 or heading <= robot_need_goal_theta - buffer_zone:
                                    left_spd = 10
                                    right_spd = -10
                                    motor(left_spd, right_spd)
                            elif robot_need_goal_theta > 180 - area_error and robot_need_goal_theta < 180 + area_error:
                                # print('area 3')
                                if heading > 200 - buffer_zone and heading < 200 + buffer_zone:
                                    left_spd = 10
                                    right_spd = 10
                                    motor(left_spd, right_spd)
                                elif heading > 20 - buffer_zone and heading < 20 + buffer_zone:
                                    left_spd = -10
                                    right_spd = -10
                                    motor(left_spd, right_spd)
                                elif heading >= 200 + buffer_zone or heading <= 20 - buffer_zone:
                                    left_spd = -10
                                    right_spd = 10
                                    motor(left_spd, right_spd)
                                elif heading >= 20 + buffer_zone and heading < 200 - buffer_zone:
                                    left_spd = 10
                                    right_spd = -10
                                    motor(left_spd, right_spd)
                            elif robot_need_goal_theta >= 180 + area_error and robot_need_goal_theta < 360 - area_error:
                                # print('area 4')
                                if heading > robot_need_goal_theta - buffer_zone and heading < robot_need_goal_theta + buffer_zone:
                                    left_spd = 10
                                    right_spd = 10
                                    motor(left_spd, right_spd)
                                elif heading >= robot_need_goal_theta + buffer_zone or heading < 90:
                                    left_spd = -10
                                    right_spd = 10
                                    motor(left_spd, right_spd)
                                elif heading <= robot_need_goal_theta - buffer_zone and heading >= 90:
                                    left_spd = 10
                                    right_spd = -10
                                    motor(left_spd, right_spd)
                else:
                    # If the robot does not see the ball, stop motors
                    # defence (middle: x: 0.05 ~ -0.05, y: 0.1)
                    middle_dx = robot_x
                    middle_dy = robot_y - 0.13
                    middle_theta = math.atan2(middle_dx, middle_dy) /math.pi  * 180
                    if middle_theta < 0:
                        middle_theta += 360
                    
                    if (robot_x >= -0.05 and robot_x <= 0.05 and robot_y >= 0.09 and robot_y <= 0.13):
                        middle = True
                        buffer = 10
                        if heading > (360 - buffer) or heading < buffer:
                            if robot_y >= -0.15 and robot_y <= 0.15:
                                front_middle = False
                                back_middle = False
                            if robot_y < -0.15 :
                                front_middle = True
                                back_middle = False
                            if robot_y > 0.15:
                                front_middle = False
                                back_middle = True
                            if front_middle == False:
                                motor(10, 10)
                            elif back_middle == False:
                                motor(-10, -10)
                        elif heading >= buffer and heading < 180:
                            left_spd = -6
                            right_spd = 6
                            motor(left_spd, right_spd)
                        elif heading >= 180 and heading <= (360 - buffer):
                            left_spd = 6
                            right_spd = -6
                            motor(left_spd, right_spd)
                    else:
                        middle = False
                        target_compass_car(heading, middle_theta, area_error, buffer_zone)

                # Send message to team robots
                self.send_data_to_team(self.player_id)
