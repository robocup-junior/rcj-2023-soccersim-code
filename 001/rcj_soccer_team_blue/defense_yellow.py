import numpy as np
import math
import time

front_defense_mode = 0
diagonal_left_defense_mode = 1
diagonal_right_defense_mode = 2
wall_spin_defense_mode = 3
wall_side_defense_mode = 4


class DefenseYellow:
    mode = 0
    switch_mode = False
    last_ball_pos = {'x': 0.0, 'y': 0.0}
    pred_n = 40
    prev_mode = 0
    face = 1
    goalline_range = 0.07
    set_time = time.time_ns()

    # Run Yellow
    def run_program(self, robot, data, lop1, lop2, roles):
        robot_angle = data.getRobotHeading()
        robot_y, robot_x = data.getRobotPos()
        ball_y, ball_x = data.getBallPos()
        ball_angle = data.getBallAngle()
        ball_distance = data.getBallDistance()
        ball_pred_y, ball_pred_x = data.getBallPredPos()
        ball_pred_angle = data.getBallPredAngle()
        ball_pred_distance = data.getBallPredDistance()
        offense1_y, offense1_x = data.getRobotPosSearch(roles.getRoleToRobot(1))
        offense2_y, offense2_x = data.getRobotPosSearch(roles.getRoleToRobot(2))
        defense_y, defense_x = data.getRobotPosSearch(roles.getRoleToRobot(3))
        offense1_angle = data.getHeadingSearch(roles.getRoleToRobot(1))
        offense2_angle = data.getHeadingSearch(roles.getRoleToRobot(2))
        defense_angle = data.getHeadingSearch(roles.getRoleToRobot(3))
        offense1_ball_distance = data.getBallDistanceSearch(roles.getRoleToRobot(1))
        offense2_ball_distance = data.getBallDistanceSearch(roles.getRoleToRobot(2))
        defense3_ball_distance = data.getBallDistanceSearch(roles.getRoleToRobot(3))
        detect = data.getTeamDetect()

        robot_angle = (-robot_angle + 90) % 360

        robot_pos = {'x': robot_x, "y": robot_y}
        ball_pos = {'x': ball_x, "y": ball_y}

        ball_pos_diff = {'x': ball_pos['x'] - self.last_ball_pos['x'],
                         'y': ball_pos['y'] - self.last_ball_pos['y']}
        ball_dist = math.sqrt((robot_pos['x'] - ball_pos['x']) ** 2 + (robot_pos['y'] - ball_pos['y']) ** 2)

        self.pred_n = DefenseYellow.rescale(self, ball_dist, 30, 50, 0, 0.8)

        ball_pos_pred = {'x': ball_pos['x'] + ball_pos_diff['x'] * ball_dist * self.pred_n,
                         'y': ball_pos['y'] + ball_pos_diff['y'] * ball_dist * self.pred_n}

        if ball_pos_diff['x'] != 0:
            ball_pos_pred = {'x': ball_pos['x'] + ball_pos_diff['x'] * ball_dist * self.pred_n,
                             'y': ball_pos['y'] + ball_pos_diff['y'] / ball_pos_diff['x'] * (-0.54 - ball_pos['x'])}

            if ball_pos_pred['y'] > 0.63:
                # ball_pos_pred['y'] = ball_pos_pred['y'] - (ball_pos_pred['y'] - 0.63)
                ball_pos_pred['y'] = 0.63
            if ball_pos_pred['y'] < -0.63:
                # ball_pos_pred['y'] = ball_pos_pred['y'] - (ball_pos_pred['y'] + 0.63)
                ball_pos_pred['y'] = -0.63
        else:
            ball_pos_pred['y'] = ball_pos['y']
        # else:
        # ball_pos_pred['y'] = ball_pos['y']

        # print(ball_pos_pred['y'], " ", ball_pos['y'])

        self.last_ball_pos = ball_pos

        spin_speed = 6

        left_speed = 0
        right_speed = 0

        # Adjust Angle Range
        if ball_x > 0:
            angle_range = 12
        else:
            angle_range = 15

        # Mode Switching
        if abs(ball_pos_diff['y']) <= 0.0045 and ball_pos['x'] < -0.44 and (
                ball_pos['y'] > 0.35 or ball_pos['y'] < -0.35):
            Xdiag1 = -(ball_pos['y'] - 0.35) / 1.4 - 0.44
            Xdiag2 = (ball_pos['y'] + 0.35) / 1.4 - 0.44
            Ydiag1 = -1.4 * (ball_pos['x'] + 0.44) + 0.35
            Ydiag2 = 1.4 * (ball_pos['x'] + 0.44) - 0.35
        else:
            Xdiag1 = -0.44
            Xdiag2 = -0.44
            Ydiag1 = 0.35
            Ydiag2 = -0.35

        Xdiag1 = -0.44
        Xdiag2 = -0.44
        Ydiag1 = 0.35
        Ydiag2 = -0.35

        if ball_pos['x'] < -0.66 and 0.24 > ball_pos['y'] > -0.24 and robot_pos[
            'x'] < -0.65 and self.prev_mode != front_defense_mode:
            self.mode = wall_spin_defense_mode
        elif ball_pos['x'] < -0.66 and (ball_pos['y'] >= 0.24 or ball_pos['y'] <= -0.24):
            self.mode = wall_side_defense_mode
        elif (ball_pos['x'] < Xdiag1 and ball_pos['y'] > Ydiag1):
            self.mode = diagonal_left_defense_mode
        elif (ball_pos['x'] < Xdiag2 and ball_pos['y'] < Ydiag2):
            self.mode = diagonal_right_defense_mode
        else:
            self.mode = front_defense_mode

        if (
                self.mode == wall_side_defense_mode or self.mode == diagonal_left_defense_mode or self.mode == diagonal_right_defense_mode) and (
                ball_dist > 0.161 and abs(ball_pos_diff['y']) < 0.001):
            if time.time_ns() - self.set_time >= 2000000000:
                self.mode = front_defense_mode
        else:
            self.set_time = time.time_ns()

        # Penalty Area Testing
        if self.mode == 5:
            left_speed, right_speed = DefenseYellow.toPoint(self, left_speed, right_speed, -0.7, ball_pos_pred['y'],
                                                          robot_pos['x'], robot_pos['y'], robot_angle,
                                                          spin_speed, angle_range)

        # Wall Defense In Penalty Area (With Deflection)
        elif self.mode == wall_spin_defense_mode:
            angle = DefenseYellow.direction(self, robot_angle, 90, 270)
            if (robot_y > -0.21 and robot_y < 0) and (robot_x < -0.7) and (
                    (robot_angle < 180 + angle_range and robot_angle > 180 - angle_range) or (
                    robot_angle < 0 + angle_range or robot_angle > 360 - angle_range)):
                angle = DefenseYellow.direction(self, robot_angle, 0, 180)
                left_speed, right_speed = DefenseYellow.move(self, robot_angle, 10, 10, angle,
                                                           0, False)
            elif (robot_y < 0.21 and robot_y >= 0) and (robot_x < -0.7) and (
                    (robot_angle < 180 + angle_range and robot_angle > 180 - angle_range) or (
                    robot_angle < 0 + angle_range or robot_angle > 360 - angle_range)):
                angle = DefenseYellow.direction(self, robot_angle, 0, 180)
                left_speed, right_speed = DefenseYellow.move(self, robot_angle, -10, -10, angle,
                                                           0, False)
            elif robot_y > -0.21 and robot_y < 0:
                if angle == 90:
                    left_speed = 5
                    right_speed = 10
                else:
                    left_speed = -10
                    right_speed = -5
            elif robot_y < 0.21 and robot_y >= 0:
                if angle == 90:
                    left_speed = 10
                    right_speed = 5
                else:
                    left_speed = -5
                    right_speed = -10
            else:
                angle = DefenseYellow.direction(self, robot_angle, 90, 270)
                if robot_angle > angle + angle_range or robot_angle < angle - angle_range:
                    left_speed, right_speed = DefenseYellow.spin(self, robot_angle, angle, left_speed,
                                                               right_speed,
                                                               spin_speed, angle_range)
                else:
                    left_speed, right_speed = DefenseYellow.move(self, robot_angle, 10, 10, angle,
                                                               90, False)
                    left_speed, right_speed = DefenseYellow.border_x(self, robot_angle, robot_pos['x'],
                                                                   left_speed,
                                                                   right_speed, 0.718, -0.718, 0,
                                                                   return_speed=0)
                    if abs(ball_pos['y'] - robot_pos['y']) > 0.2:
                        left_speed, right_speed = DefenseYellow.border_x(self, robot_angle, robot_pos['x'],
                                                                       left_speed,
                                                                       right_speed, 0.695, -0.695, 0.05, -10)

        # Wall Defense Outside Of Penalty Area (With Going To Wall Defense Point)
        elif self.mode == wall_side_defense_mode:
            if (robot_y <= 0.23 or robot_y > 0.38) and ball_y >= 0.28:
                left_speed, right_speed = DefenseYellow.toPoint(self, left_speed, right_speed, -0.67, 0.3,
                                                              robot_pos['x'], robot_pos['y'], robot_angle,
                                                              spin_speed, angle_range)
            elif (robot_y >= -0.23 or robot_y < -0.38) and ball_y <= -0.28:
                left_speed, right_speed = DefenseYellow.toPoint(self, left_speed, right_speed, -0.67, -0.3,
                                                              robot_pos['x'], robot_pos['y'], robot_angle,
                                                              spin_speed, angle_range)
            else:
                angle = DefenseYellow.direction(self, robot_angle, 90, 270)
                if robot_angle > angle + angle_range or robot_angle < angle - angle_range:
                    left_speed, right_speed = DefenseYellow.spin(self, robot_angle, angle, left_speed,
                                                               right_speed,
                                                               spin_speed, angle_range)
                else:
                    left_speed, right_speed = DefenseYellow.move(self, robot_angle, 10, 10, angle,
                                                               90, False)
                    left_speed, right_speed = DefenseYellow.border_x(self, robot_angle, robot_pos['x'],
                                                                   left_speed,
                                                                   right_speed, 0.718, -0.718, 0, return_speed=0)
                    if abs(ball_pos['y'] - robot_pos['y']) > 0.2:
                        left_speed, right_speed = DefenseYellow.border_x(self, robot_angle, robot_pos['x'],
                                                                       left_speed,
                                                                       right_speed, 0.695, -0.695, 0.05, -10)

        # Diagonal Ball Tracking On The Robot's Left Side (Blue)
        elif self.mode == diagonal_left_defense_mode:
            if (robot_x < -0.65 or robot_x > -0.57):
                left_speed, right_speed = DefenseYellow.toPoint(self, left_speed, right_speed, -0.64, 0.28,
                                                              robot_pos['x'], robot_pos['y'], robot_angle,
                                                              spin_speed, angle_range)
            else:
                angle = DefenseYellow.direction(self, robot_angle, 120, 300)
                if robot_angle > angle + angle_range or robot_angle < angle - angle_range:
                    left_speed, right_speed = DefenseYellow.spin(self, robot_angle, angle, left_speed, right_speed,
                                                               spin_speed,
                                                               angle_range)
                else:
                    solution_x, solution_y = DefenseYellow.findPoint(self, robot_angle, robot_x, robot_y, ball_x,
                                                                   ball_pos_pred['y'],
                                                                   -0.745, 0)
                    left_speed, right_speed = DefenseYellow.chase(self, robot_angle, robot_pos['y'],
                                                                solution_y)
                    left_speed, right_speed = DefenseYellow.border_y(self, robot_angle, robot_pos['y'],
                                                                   left_speed, right_speed, -0.39, 0.39, 0.05, 10)

        # Diagonal Ball Tracking On The Robot's Right Side (Blue)
        elif self.mode == diagonal_right_defense_mode:
            if (robot_x < -0.65 or robot_x > -0.57):
                left_speed, right_speed = DefenseYellow.toPoint(self, left_speed, right_speed, -0.64, -0.28,
                                                              robot_pos['x'], robot_pos['y'], robot_angle,
                                                              spin_speed, angle_range)
            else:
                angle = DefenseYellow.direction(self, robot_angle, 60, 240)
                if robot_angle > angle + angle_range or robot_angle < angle - angle_range:
                    left_speed, right_speed = DefenseYellow.spin(self, robot_angle, angle, left_speed,
                                                               right_speed, spin_speed, angle_range)
                else:
                    solution_x, solution_y = DefenseYellow.findPoint(self, robot_angle, robot_x, robot_y, ball_x,
                                                                   ball_pos_pred['y'], -0.745, 0)
                    left_speed, right_speed = DefenseYellow.chase(self, robot_angle, robot_pos['y'],
                                                                solution_y)
                    left_speed, right_speed = DefenseYellow.border_y(self, robot_angle, robot_pos['y'],
                                                                   left_speed, right_speed, -0.39, 0.39, 0.05,
                                                                   10)

        # Uncovered Areas For Ball Tracking
        elif False and ball_pos['x'] > 0.56 and 0.35 >= ball_pos['y'] > 0:
            pass
            pass
        elif False and ball_pos['x'] > 0.56 and -0.35 <= ball_pos['y'] > 0:
            pass
            pass

        # Regular Ball Tracking In A Straight Line
        else:
            if abs(-0.56 - robot_pos['x']) > self.goalline_range:
                self.goalline_range = 0.01

                if ball_pos['x'] > 0:
                    border = 0.24
                else:
                    border = 0.30
                temp_ball_pos_y = ball_pos['y']
                if temp_ball_pos_y > border:
                    temp_ball_pos_y = border
                elif temp_ball_pos_y < -border:
                    temp_ball_pos_y = -border

                left_speed, right_speed = DefenseYellow.toPoint(self, left_speed, right_speed, -0.56, temp_ball_pos_y,
                                                              robot_pos['x'], robot_pos['y'], robot_angle,
                                                              spin_speed, angle_range)
            # stay parallel to goal
            else:
                if robot_angle > 180:
                    robot_angle -= 360
                if ball_pos['x'] > 0:
                    border = 0.24
                else:
                    border = 0.30
                self.goalline_range = 0.01

                if robot_angle < 178 and robot_angle >= 90:
                    left_speed = 10 - (robot_angle - 90) * 9.5 / 90
                    right_speed = -10 + (robot_angle - 90) * 9.5 / 90
                    self.face = 1
                elif robot_angle > -178 and robot_angle <= -90:
                    left_speed = -10 - (robot_angle + 90) * 9.5 / 90
                    right_speed = 10 + (robot_angle + 90) * 9.5 / 90
                    self.face = 1
                elif robot_angle < 90 and robot_angle > 2:
                    left_speed = -10 + (9 - robot_angle * 9.5 / 90)
                    right_speed = 10 - (9 - robot_angle * 9.5 / 90)
                    self.face = -1
                elif robot_angle > -90 and robot_angle < -2:
                    left_speed = 10 - (9 + robot_angle * 9.5 / 90)
                    right_speed = -10 + (9 + robot_angle * 9.5 / 90)
                    self.face = -1
                # follow ball  
                elif robot_pos['y'] > border:
                    left_speed = -10 * self.face
                    right_speed = -10 * self.face
                elif robot_pos['y'] < -border:
                    left_speed = 10 * self.face
                    right_speed = 10 * self.face
                else:
                    if ball_pos['x'] < -0.4:
                        if robot_pos['y'] < ball_pos['y']:
                            left_speed = 10 * self.face
                            right_speed = 10 * self.face
                        elif robot_pos['y'] > ball_pos['y']:
                            left_speed = -10 * self.face
                            right_speed = -10 * self.face
                        else:
                            left_speed = 0
                            right_speed = 0
                    else:
                        if robot_pos['y'] < ball_pos_pred['y']:
                            left_speed = 10 * self.face
                            right_speed = 10 * self.face
                        elif robot_pos['y'] > ball_pos_pred['y']:
                            left_speed = -10 * self.face
                            right_speed = -10 * self.face
                        else:
                            left_speed = 0
                            right_speed = 0

            if left_speed > 10:
                left_speed = 10
            elif left_speed < -10:
                left_speed = -10
            if right_speed > 10:
                right_speed = 10
            elif right_speed < -10:
                right_speed = -10

        self.prev_mode = self.mode

        robot.left_motor.setVelocity(left_speed)
        robot.right_motor.setVelocity(right_speed)

        # Checks if the location of the ball can be found
        if detect:
            pass
        else:
            pass

    def reset(self):
        self.mode = 0
        self.switch_mode = False
        self.last_ball_pos = {'x': 0.0, 'y': 0.0}
        self.pred_n = 40
        self.prev_mode = 0

    # Add any additional functions here
    def coords(self, robot_angle, robot_x, robot_y, dest_x, dest_y):
        angleA = (math.degrees(math.atan2(robot_x - dest_x, robot_y - dest_y)) + 270) % 360
        angleB = (math.degrees(math.atan2(robot_x - dest_x, robot_y - dest_y)) + 180) % 360
        if robot_angle - angleA >= 0:
            leftA_total = robot_angle - angleA
        else:
            leftA_total = robot_angle + 360 - angleA
        if angleA - robot_angle >= 0:
            rightA_total = angleA - robot_angle
        else:
            rightA_total = angleA + 360 - robot_angle
        if robot_angle - angleB >= 0:
            leftB_total = robot_angle - angleB
        else:
            leftB_total = robot_angle + 360 - angleB
        if angleB - robot_angle >= 0:
            rightB_total = angleB - robot_angle
        else:
            rightB_total = angleB + 360 - robot_angle
        if leftA_total < rightA_total:
            a_close = leftA_total
        else:
            a_close = rightA_total
        if leftB_total < rightB_total:
            b_close = leftB_total
        else:
            b_close = rightB_total
        if a_close < b_close:
            angle = angleA
        else:
            angle = angleB
        return angle, angleA

    def toPoint(self, left_speed, right_speed, dest_x, dest_y, robot_pos_x, robot_pos_y, robot_angle, spin_speed,
                angle_range):
        angle, absolute_angle = DefenseYellow.coords2(self, robot_angle, robot_pos_x, robot_pos_y,
                                                    dest_x,
                                                    dest_y)
        if robot_angle > angle + angle_range or robot_angle < angle - angle_range:
            left_speed, right_speed = DefenseYellow.spin(self, robot_angle, angle, left_speed,
                                                       right_speed,
                                                       spin_speed, angle_range)
        else:
            left_speed, right_speed = DefenseYellow.move(self, robot_angle, 10, 10, angle,
                                                       absolute_angle,
                                                       False)
        return left_speed, right_speed

    def spin(self, robot_angle, angle, left_speed, right_speed, spin_speed, angle_range, spin_speedL=None,
             spin_speedR=None):
        if robot_angle - angle >= 0:
            left_total = robot_angle - angle
        else:
            left_total = robot_angle + 360 - angle
        if angle - robot_angle >= 0:
            right_total = angle - robot_angle
        else:
            right_total = angle + 360 - robot_angle
        if robot_angle < angle - angle_range or robot_angle > angle + angle_range:
            if (left_total > right_total):
                if spin_speedL != None and spin_speedR != None:
                    left_speed = spin_speedL
                    right_speed = spin_speedR
                else:
                    left_speed = spin_speed
                    right_speed = -spin_speed
            else:
                if spin_speedL != None and spin_speedR != None:
                    left_speed = spin_speedL
                    right_speed = spin_speedR
                else:
                    left_speed = -spin_speed
                    right_speed = spin_speed
        # print("Spin")
        return left_speed, right_speed

    def direction(self, robot_angle, angleA, angleB):
        if robot_angle - angleA >= 0:
            leftA_total = robot_angle - angleA
        else:
            leftA_total = robot_angle + 360 - angleA
        if angleA - robot_angle >= 0:
            rightA_total = angleA - robot_angle
        else:
            rightA_total = angleA + 360 - robot_angle
        if robot_angle - angleB >= 0:
            leftB_total = robot_angle - angleB
        else:
            leftB_total = robot_angle + 360 - angleB
        if angleB - robot_angle >= 0:
            rightB_total = angleB - robot_angle
        else:
            rightB_total = angleB + 360 - robot_angle
        if leftA_total < rightA_total:
            a_close = leftA_total
        else:
            a_close = rightA_total
        if leftB_total < rightB_total:
            b_close = leftB_total
        else:
            b_close = rightB_total
        if a_close < b_close:
            angle = angleA
        else:
            angle = angleB
        # print("Direction")
        return angle

    def move(self, robot_angle, left_run, right_run, relative, absolute, straight):
        if straight == True:
            angle = DefenseYellow.direction(self, robot_angle, 0, 180)
            if angle == 180:
                left_speed = left_run
                right_speed = right_run
            else:
                left_run2 = left_run
                left_speed = -right_run
                right_speed = -left_run2
        else:
            angle = DefenseYellow.direction(self, robot_angle, relative, absolute)
            if angle == absolute:
                left_speed = left_run
                right_speed = right_run
            else:
                left_run2 = left_run
                left_speed = -right_run
                right_speed = -left_run2
        # print("Move")
        return left_speed, right_speed

    def coords2(self, robot_angle, robot_x, robot_y, dest_x, dest_y):
        angleA = (math.degrees(math.atan2(robot_x - dest_x, robot_y - dest_y))) % 360
        angleB = (math.degrees(math.atan2(robot_x - dest_x, robot_y - dest_y)) + 180) % 360
        if robot_angle - angleA >= 0:
            leftA_total = robot_angle - angleA
        else:
            leftA_total = robot_angle + 360 - angleA
        if angleA - robot_angle >= 0:
            rightA_total = angleA - robot_angle
        else:
            rightA_total = angleA + 360 - robot_angle
        if robot_angle - angleB >= 0:
            leftB_total = robot_angle - angleB
        else:
            leftB_total = robot_angle + 360 - angleB
        if angleB - robot_angle >= 0:
            rightB_total = angleB - robot_angle
        else:
            rightB_total = angleB + 360 - robot_angle
        if leftA_total < rightA_total:
            a_close = leftA_total
        else:
            a_close = rightA_total
        if leftB_total < rightB_total:
            b_close = leftB_total
        else:
            b_close = rightB_total
        if a_close < b_close:
            angle = angleA
        else:
            angle = angleB
        # print("Coordinates")
        return angle, angleA

    def toPoint2(self, left_speed, right_speed, dest_x, dest_y, robot_pos_x, robot_pos_y, robot_angle):
        spin_speed = 6
        angle_range = 15
        angle, absolute_angle = DefenseYellow.coords2(self, robot_angle, robot_pos_x, robot_pos_y, dest_x, dest_y)
        if robot_angle > angle + angle_range or robot_angle < angle - angle_range:
            left_speed, right_speed = DefenseYellow.spin(self, robot_angle, angle, left_speed,
                                                       right_speed,
                                                       spin_speed, angle_range)
        else:
            left_speed, right_speed = DefenseYellow.move(self, robot_angle, 10, 10, angle,
                                                       absolute_angle,
                                                       False)
        return left_speed, right_speed

    def chase(self, robot_angle, robot_y, ball_y):
        if ball_y < robot_y:
            left_speed, right_speed = DefenseYellow.move(self, robot_angle, -7, -7, 0, 0, True)
        elif ball_y > robot_y:
            left_speed, right_speed = DefenseYellow.move(self, robot_angle, 7, 7, 0, 0, True)
        else:
            left_speed, right_speed = DefenseYellow.move(self, robot_angle, 0, 0, 0, 0, True)
        # print("Chase")
        return left_speed, right_speed

    def border_x(self, robot_angle, robot_x, left_speed, right_speed, limit1, limit2, given_range, return_speed=10):
        if limit2 < limit1:
            limitA = limit2
            limitB = limit1
        elif limit1 < limit2:
            limitA = limit1
            limitB = limit2
        else:
            return left_speed, right_speed
        if robot_x < limitA:
            left_speed, right_speed = DefenseYellow.move(self, robot_angle, return_speed, return_speed, 0, 0, True)
        elif robot_x > limitB:
            left_speed, right_speed = DefenseYellow.move(self, robot_angle, -return_speed, -return_speed, 0, 0, True)
        elif robot_x > limitA and robot_x < limitA + given_range and left_speed < 0:
            angle = DefenseYellow.direction(self, robot_angle, 0, 180)
            if (angle == 180 and left_speed < 0 or angle == 0 and left_speed > 0):
                left_speed = 1
                right_speed = 1
                if return_speed == 0:
                    left_speed = 0
                    right_speed = 0
        elif robot_x < limitB and robot_x > limitB - given_range and left_speed > 0:
            angle = DefenseYellow.direction(self, robot_angle, 0, 180)
            if (angle == 180 and left_speed > 0 or angle == 0 and left_speed < 0):
                left_speed = -1
                right_speed = -1
                if return_speed == 0:
                    left_speed = 0
                    right_speed = 0
        return left_speed, right_speed

    def border_y(self, robot_angle, robot_y, left_speed, right_speed, limit1, limit2, given_range, return_speed=10):
        if limit2 < limit1:
            limitA = limit2
            limitB = limit1
        elif limit1 < limit2:
            limitA = limit1
            limitB = limit2
        else:
            return left_speed, right_speed
        if robot_y < limitA:
            left_speed, right_speed = DefenseYellow.move(self, robot_angle, return_speed, return_speed, 0, 0, True)
        elif robot_y > limitB:
            left_speed, right_speed = DefenseYellow.move(self, robot_angle, -return_speed, -return_speed, 0, 0, True)
        elif robot_y > limitA and robot_y < limitA + given_range and left_speed < 0:
            angle = DefenseYellow.direction(self, robot_angle, 0, 180)
            if (angle == 180 and left_speed < 0 or angle == 0 and left_speed > 0):
                left_speed = 1
                right_speed = 1
        elif robot_y < limitB and robot_y > limitB - given_range and left_speed > 0:
            angle = DefenseYellow.direction(self, robot_angle, 0, 180)
            if (angle == 180 and left_speed > 0 or angle == 0 and left_speed < 0):
                left_speed = -1
                right_speed = -1
        # print("Border Y")
        return left_speed, right_speed

    def findPoint(self, robot_angle, robot_x, robot_y, point_x, point_y, dest_x, dest_y):
        robot_x, robot_y = robot_x, -robot_y
        point_x, point_y = point_x, -point_y
        dest_x, dest_y = dest_x, -dest_y
        robot_angle = (robot_angle + 270) % 360
        robot_radian = math.radians(robot_angle)

        robot_slope = math.tan(robot_radian)
        robot_intercept = robot_y - robot_slope * robot_x
        point_angle = math.atan2(point_y, point_x - dest_x)
        point_slope = math.tan(point_angle)
        point_intercept = point_y - point_slope * point_x

        left_matrix = np.array([[-robot_slope, 1], [-point_slope, 1]])
        right_matrix = np.array([robot_intercept, point_intercept])
        solution = np.linalg.solve(left_matrix, right_matrix).tolist()
        solution_x, solution_y = solution[0], solution[1]
        # print("Find Point")
        return solution_x, -solution_y

    def rescale(self, dist, minScale, maxScale, minLimit, maxLimit):
        if dist >= maxLimit:
            return maxScale
        elif dist <= minLimit:
            return minScale
        else:
            scale = (maxScale - minScale) / (maxLimit - minLimit)
            pred = scale * dist + minScale
        return pred