from rcj_soccer_robot import TIME_STEP
import numpy as np
import math

goal_attack_mode = 0
ambush_attack_mode = 1
stationing_attack_mode = 2
double_pushing_mode = 3
midfield_defense_mode = 4
wall_defense_mode = 5
deflection_defense_mode = 6
goal_defense_mode = 7
trap_ball_main_mode = 8
trap_ball_support_mode = 9

class OffenseBlue2:
    mode = 0
    side = 1
    goal_x, goal_y = 0, -0.75
    off_angle = 25
    last_ball_x, last_ball_y = 0, 0
    last_mode = 0
    limit_x = -0.12
    timer = 0
    push_ball = False
    trap_stage = -1
    double_push_x, double_push_y = 0, 0
    wall_dest_x, wall_dest_y = 0.44, 0.7
    team = "B"

    # Run Blue
    def run_program(self, robot, data, lop1, lop2, roles, trap_stage):
        left_speed, right_speed = 0, 0

        robot_angle = data.getRobotHeading()
        robot_x, robot_y = data.getRobotPos()
        ball_x, ball_y = data.getBallPos()
        ball_angle = data.getBallAngle()
        ball_distance = data.getBallDistance()
        ball_pred_x, ball_pred_y = data.getBallPredPos()
        ball_pred_angle = data.getBallPredAngle()
        ball_pred_distance = data.getBallPredDistance()
        offense1_x, offense1_y = data.getRobotPosSearch(roles.getRoleToRobot(1))
        offense2_x, offense2_y = data.getRobotPosSearch(roles.getRoleToRobot(2))
        defense_x, defense_y = data.getRobotPosSearch(roles.getRoleToRobot(3))
        offense1_angle = data.getHeadingSearch(roles.getRoleToRobot(1))
        offense2_angle = data.getHeadingSearch(roles.getRoleToRobot(2))
        defense_angle = data.getHeadingSearch(roles.getRoleToRobot(3))
        offense1_ball_distance = data.getBallDistanceSearch(roles.getRoleToRobot(1))
        offense2_ball_distance = data.getBallDistanceSearch(roles.getRoleToRobot(2))
        defense3_ball_distance = data.getBallDistanceSearch(roles.getRoleToRobot(3))
        detect = data.getTeamDetect()

        ball_lack_progress_short = lop1["ball"]
        ball_lack_progress_long = lop2["ball"]

        spin_speed = 6
        angle_range = 15

        run_pass = False

        self.goal_x, self.goal_y = 0, -0.75

        if ball_y < -0.65:
            self.limit_x = -0.15
        else:
            self.limit_x = -0.12

        if detect and not data.robot_flipped:
            if robot_y < 0.6 and ball_y > 0.6 and ball_x > 0:
                offset = 0.15
                if defense_y < 0.62:
                    self.wall_dest_x, self.wall_dest_y = 0.44, 0.7
                elif defense_x < 0.25:
                    self.wall_dest_x, self.wall_dest_y = 0.25 + offset, 0.7
                elif defense_x > 0.4:
                    self.wall_dest_x, self.wall_dest_y = 0.4 + offset, 0.7
                else:
                    self.wall_dest_x, self.wall_dest_y = defense_x + offset, 0.7
            if self.trap_stage != -1 or (
                    offense1_y > 0.67 and offense2_y > 0.67 and ball_y > 0.7 and offense2_x < ball_x < offense1_x and ball_x > 0.2 and offense1_y < ball_y and offense2_y < ball_y and 0.01 < OffenseBlue2.getDistance(
                    self, offense1_x, offense1_y, offense2_x, offense2_y) < 0.16):
                self.mode = trap_ball_main_mode
                if offense1_y > 0.67 and offense2_y > 0.67 and ball_y > 0.7 and offense2_x < ball_x < offense1_x and ball_x > 0.2 and offense1_y < ball_y and offense2_y < ball_y and 0.01 < OffenseBlue2.getDistance(
                        self, offense1_x, offense1_y, offense2_x, offense2_y) < 0.16 and self.trap_stage == -1:
                    self.trap_stage = 1
            elif trap_stage != -1:
                self.mode = trap_ball_support_mode
            elif ball_y > 0.66 and ball_x > 0:
                self.mode = wall_defense_mode
            elif ball_y > 0.64 and -0.38 <= ball_x and (ball_x <= -0.25 or robot_y > ball_y):
                self.mode = goal_defense_mode
            elif ball_y > 0.64 and not 0.25 > ball_x > -0.25:
                self.mode = midfield_defense_mode
            elif ball_y > 0.53 and 0.25 > ball_x > -0.25 and defense_y > 0.59:
                self.mode = deflection_defense_mode
            elif ball_lack_progress_short and ball_x < self.limit_x:
                if OffenseBlue2.rangeAngle(self, offense1_angle, 125, angle_range, 35, 20) and -0.42 < ball_x < -0.12 and ball_y < -0.65 and \
                        -0.47 < offense1_x < -0.12 and offense1_y < -0.64 and robot_y < -0.3:
                    self.mode = double_pushing_mode
                else:
                    self.mode = stationing_attack_mode
            else:
                if (self.push_ball is True and ball_x >= -0.2) or (ball_x >= self.limit_x and (ball_y <= -0.22 or offense2_ball_distance < offense1_ball_distance or ball_x > -self.limit_x)):
                    self.mode = goal_attack_mode
                    if self.last_mode == ambush_attack_mode:
                        self.push_ball = True
                else:
                    self.push_ball = False
                    self.mode = ambush_attack_mode
            self.timer = 0
        else:
            self.mode = stationing_attack_mode
            self.timer += (TIME_STEP / 1000)

        # Ball Chasing and Shooting
        if self.mode == goal_attack_mode:
            if OffenseBlue2.rangePosition(self, ball_x, ball_y, 0, 0, 0.1, 0.1) and ball_distance < 0.15 and robot_y > ball_y and OffenseBlue2.getDistance(self, ball_x, ball_y, self.last_ball_x, self.last_ball_y) < math.pow(10, -5) * 5:
                self.goal_x, self.goal_y = 0.05, -0.75
            elif ball_x < -0.25 and ball_y < 0:
                self.goal_x, self.goal_y = -0.05, -0.75
            elif ball_x > 0.25 and ball_y < 0:
                self.goal_x, self.goal_y = 0.05, -0.75
            else:
                self.goal_x, self.goal_y = 0.03, -0.75

            goal_angle = (math.degrees(math.atan2(self.goal_x - ball_pred_x, self.goal_y - ball_pred_y)) + 180) % 360

            if ball_y < -0.6 and (ball_x > 0.35 or ball_x < -0.35):
                goal_angle = 65

            if (ball_x > 0.52 or ball_x < -0.52) and -0.54 < ball_y < 0.54:
                goal_angle = 0
            elif ball_x > 0.55 or ball_x < -0.55:
                goal_angle = 0

            if 0 > ball_y > -0.5 and -0.4 < ball_x < 0.4:
                goal_temp_angle = math.atan2(abs(self.goal_x - ball_pred_x), abs(self.goal_y - ball_pred_y))
                goal_temp_distance = 0.015
                ball_temp_x = ball_pred_x - math.sin(goal_temp_angle) * goal_temp_distance
                ball_temp_y = ball_pred_y + math.cos(goal_temp_angle) * goal_temp_distance
            else:
                goal_temp_angle = math.atan2(self.goal_x - ball_pred_x, self.goal_y - ball_pred_y)
                goal_temp_distance = 0.015
                ball_temp_x = ball_pred_x + math.sin(goal_temp_angle) * goal_temp_distance
                ball_temp_y = ball_pred_y + math.cos(goal_temp_angle) * goal_temp_distance

            ball_pred_angle = (math.degrees(
                math.atan2(ball_temp_x - robot_x, ball_temp_y - robot_y)) - robot_angle + 180) % 360

            ball_direction = ball_pred_angle
            if ball_direction > 180:
                ball_direction = ball_direction - 360
            abs_ba = (ball_direction + robot_angle - goal_angle + 360) % 360

            if abs_ba < self.off_angle or abs_ba > 360 - self.off_angle:
                offset = 0
            elif ball_distance < 0.1:
                offset = 62
            elif ball_distance > 1:
                offset = 1
            else:
                offset = 61 * pow(ball_distance - 1, 2) + 1

            if 180 < abs_ba < 360 - self.off_angle:
                offset *= -1

            if ball_pred_y < -0.66 and (0.55 < ball_x < 0.62 or -0.55 > ball_x > -0.62):
                offset *= 0.5
            elif (robot_x >= 0.51 or robot_x <= -0.51) and -0.63 < ball_pred_y + 0.0375 < robot_y:
                offset *= (0.63 - abs(robot_y)) / 0.9
            elif (ball_x <= -0.42 or (ball_x <= -0.35 and ball_y > -0.1)) and (robot_x <= -0.35 or robot_x >= 0.35) and 180 < abs_ba < 360 - self.off_angle:
                offset *= -1
            elif (ball_x >= 0.42 or (ball_x >= 0.35 and ball_y > -0.1)) and (robot_x <= -0.35 or robot_x >= 0.35) and self.off_angle < abs_ba < 180:
                offset *= -1

            closest_angle, closest_difference = OffenseBlue2.direction(self, ball_pred_angle, 0, 180)

            if closest_angle == 0:
                self.side = 1
            elif closest_angle == 180:
                self.side = -1

            if self.side == 1:
                move_angle = ball_direction + offset
            else:
                if ball_direction + offset >= 0:
                    move_angle = ball_direction + offset - 180
                else:
                    move_angle = 180 - abs((ball_direction + offset))

            if OffenseBlue2.rangePosition(self, ball_x, ball_y, 0.575, -0.73, 0.01, 0.01) and ball_distance < 0.07 and ball_y < robot_y:
                shoot_angle = 60
                closest_angle, closest_difference = OffenseBlue2.direction(self, robot_angle, (shoot_angle + 360) % 360,
                                                                       (shoot_angle + 360 + 180) % 360)
                if not OffenseBlue2.rangeAngle(self, robot_angle, closest_angle, angle_range):
                    spin_speed = 6
                    left_speed, right_speed = OffenseBlue2.spin(self, left_speed, right_speed, robot_angle, closest_angle,
                                                            spin_speed)
            elif -0.58 < ball_x < 0.58 and -0.675 < ball_y < 0.2:
                left_speed, right_speed = OffenseBlue2.moveCurve(self, move_angle, self.side, 45)
            elif (ball_x < -0.58 or ball_x > 0.58) and ball_y > 0:
                left_speed, right_speed = OffenseBlue2.moveCurve(self, move_angle, self.side, 90)
            else:
                left_speed, right_speed = OffenseBlue2.moveCurve(self, move_angle, self.side, 60)

        # Robot Ambushing
        elif self.mode == ambush_attack_mode:
            angle_range = 4
            shoot_angle = 0
            point_x = self.limit_x + 0.04
            if OffenseBlue2.getDistance(self, ball_x, ball_y, self.last_ball_x, self.last_ball_y) < math.pow(10, -5):
                dest_x, dest_y = point_x, ball_y
                offset = 0.0725
            else:
                dest_x, dest_y = OffenseBlue2.findAttackPoint(self, point_x, ball_x, ball_y, self.last_ball_x, self.last_ball_y)
                if dest_x is None or dest_y is None:
                    dest_x, dest_y = point_x, ball_y + 0.075
                offset = 0

            dest_y = dest_y + offset

            if dest_y > 0 and dest_y > ball_y + 0.1:
                dest_y = ball_y + 0.075
            elif (dest_y > 0.1 and ball_y < 0) or (dest_y < -0.1 and ball_y > 0):
                dest_y = 0.0725
            elif dest_y < 0 < ball_y and abs(ball_y - dest_y) > 0.15 and ball_y > self.last_ball_y:
                dest_y = 0.0725
            elif dest_y > 0 > ball_y and abs(ball_y - dest_y) > 0.15 and ball_y < self.last_ball_y:
                dest_y = 0.0725

            if ball_y > -0.57 and dest_y < -0.54:
                dest_y = -0.54
            elif dest_y > 0.4 and -0.25 < ball_y < 0.25:
                dest_y = 0.4
            elif dest_y < -0.675:
                dest_y = -0.675

            if not OffenseBlue2.rangePosition(self, robot_x, robot_y, dest_x, dest_y, 0.005, 0.005):
                robot_radius = 0.0475
                ball_radius = 0.021
                if ball_distance < 0.1 and abs(robot_x) + robot_radius > abs(ball_x) - ball_radius > abs(dest_x) and \
                        robot_y - robot_radius < ball_y + ball_radius < dest_y:
                    temp_dest_x, temp_dest_y = OffenseBlue2.findPoint(self, point_x, robot_x, robot_y, ball_x, ball_y - 0.1)
                    left_speed, right_speed = OffenseBlue2.toPoint(self, left_speed, right_speed, temp_dest_x, temp_dest_y, robot_x,
                                                               robot_y, robot_angle, spin_speed, angle_range)
                else:
                    left_speed, right_speed = OffenseBlue2.toPoint(self, left_speed, right_speed, dest_x, dest_y, robot_x,
                                                           robot_y, robot_angle, spin_speed, angle_range)
                left_speed, right_speed = OffenseBlue2.toPoint(self, left_speed, right_speed, dest_x, dest_y, robot_x,
                                                           robot_y, robot_angle, spin_speed, angle_range)
            else:
                closest_angle, closest_difference = OffenseBlue2.direction(self, robot_angle, (shoot_angle + 360) % 360, (shoot_angle + 360 + 180) % 360)
                if not OffenseBlue2.rangeAngle(self, robot_angle, closest_angle, angle_range):
                    left_speed, right_speed = OffenseBlue2.spin(self, left_speed, right_speed, robot_angle, closest_angle, spin_speed)
                else:
                    left_speed, right_speed = OffenseBlue2.moveStraight(self, robot_angle, 0, -2)

        # Robot Stationing and Tracking
        elif self.mode == stationing_attack_mode:
            if not detect:
                if ball_x > 0:
                    if ball_y > 0.6:
                        dest_x, dest_y = 0.3, 0.35
                    else:
                        dest_x, dest_y = 0.3, 0.2
                else:
                    if ball_y > 0.6:
                        if self.timer < 5:
                            dest_x, dest_y = 0, 0.35
                        else:
                            dest_x, dest_y = 0.2, 0.35
                    else:
                        if self.timer < 5:
                            dest_x, dest_y = 0, 0.2
                        else:
                            dest_x, dest_y = 0.2, 0.2
            else:
                if ball_y < 0:
                    if ball_x < 0:
                        attack_point_distance = OffenseBlue2.getDistance(self, ball_x, ball_y, -0.3, -0.3)
                        defense_point_distance = OffenseBlue2.getDistance(self, ball_x, ball_y, -0.3, 0.3)
                    else:
                        attack_point_distance = OffenseBlue2.getDistance(self, ball_x, ball_y, 0.3, -0.3)
                        defense_point_distance = OffenseBlue2.getDistance(self, ball_x, ball_y, 0.3, 0.3)
                    if attack_point_distance + 0.03 < defense_point_distance:
                        dest_x, dest_y = 0, 0
                    else:
                        dest_x, dest_y = 0, 0.1
                else:
                    dest_x, dest_y = 0, 0.2
            angle_range = 8
            if not OffenseBlue2.rangePosition(self, robot_x, robot_y, dest_x, dest_y, 0.02, 0.02):
                left_speed, right_speed = OffenseBlue2.toPoint(self, left_speed, right_speed, dest_x, dest_y, robot_x,
                                                           robot_y, robot_angle, spin_speed, angle_range)
            else:
                closest_angle, closest_difference = OffenseBlue2.direction(self, robot_angle, 0, 180)
                if not OffenseBlue2.rangeAngle(self, robot_angle, closest_angle, angle_range):
                    left_speed, right_speed = OffenseBlue2.spin(self, left_speed, right_speed, robot_angle, closest_angle, spin_speed)
                else:
                    left_speed, right_speed = OffenseBlue2.moveStraight(self, robot_angle, 0, -2)

        # Double Pushing
        elif self.mode == double_pushing_mode:
            angle_range = 8
            offset_x, offset_y = -0.03, 0.035
            self.double_push_x, self.double_push_y = offense1_x + offset_x, offense1_y + offset_y
            dest_x, dest_y = self.double_push_x, self.double_push_y
            if OffenseBlue2.rangePosition(self, robot_x, robot_y, dest_x, dest_y, 0.05, 0.05) or \
                    OffenseBlue2.rangePosition(self, robot_x, robot_y, offense1_x, offense1_y, 0.1,
                                           0.1) and robot_x < offense1_x:
                closest_angle, closest_difference = OffenseBlue2.direction(self, robot_angle, 125, 305)
                if not OffenseBlue2.rangeAngle(self, robot_angle, closest_angle, angle_range):
                    left_speed, right_speed = OffenseBlue2.spin(self, left_speed, right_speed, robot_angle, closest_angle,
                                                            spin_speed)
                else:
                    left_speed, right_speed = OffenseBlue2.moveStraight(self, robot_angle, 125, 10)
            else:
                point_angle = (math.degrees(math.atan2(dest_x - robot_x, dest_y - robot_y)) - robot_angle + 180) % 360

                push_angle = 300
                point_direction = point_angle
                if point_direction > 180:
                    point_direction = point_direction - 360
                abs_ba = (point_direction + robot_angle - push_angle + 360) % 360

                point_distance = OffenseBlue2.getDistance(self, robot_x, robot_y, dest_x, dest_y)

                if abs_ba < self.off_angle or abs_ba > 360 - self.off_angle:
                    offset = 0
                elif point_distance < 0.1:
                    offset = 62
                elif point_distance > 1:
                    offset = 1
                else:
                    offset = 61 * pow(point_distance - 1, 2) + 1

                if 180 < abs_ba < 360 - self.off_angle:
                    offset *= -1

                closest_angle, closest_difference = OffenseBlue2.direction(self, (robot_angle + 180) % 360, 0, 180)

                if self.last_mode != double_pushing_mode:
                    if closest_angle == 0:
                        self.side = 1
                    elif closest_angle == 180:
                        self.side = -1

                if self.side == 1:
                    move_angle = point_direction + offset
                else:
                    if point_direction + offset >= 0:
                        move_angle = point_direction + offset - 180
                    else:
                        move_angle = 180 - abs(point_direction + offset)

                left_speed, right_speed = OffenseBlue2.moveCurve(self, move_angle, self.side, 45)

        # Wall Defense
        elif self.mode == wall_defense_mode:
            dest_x, dest_y = self.wall_dest_x, self.wall_dest_y
            angle_range = 15
            if not OffenseBlue2.rangePosition(self, robot_x, robot_y, dest_x, dest_y, 1, 1, 0.02, 0.4, 1, 0.025):
                point_angle = (math.degrees(math.atan2(dest_x - robot_x, dest_y - robot_y)) - robot_angle + 180) % 360

                point_direction = point_angle
                if point_direction > 180:
                    point_direction = point_direction - 360

                offset = 0

                closest_angle, closest_difference = OffenseBlue2.direction(self, point_angle, 0, 180)

                if closest_angle == 0:
                    self.side = 1
                elif closest_angle == 180:
                    self.side = -1

                if self.side == 1:
                    move_angle = point_direction + offset
                else:
                    if point_direction + offset >= 0:
                        move_angle = point_direction + offset - 180
                    else:
                        move_angle = 180 - abs(point_direction + offset)

                left_speed, right_speed = OffenseBlue2.moveCurve(self, move_angle, self.side, 60)
            else:
                closest_angle, closest_difference = OffenseBlue2.direction(self, robot_angle, 0, 180)
                if not OffenseBlue2.rangeAngle(self, robot_angle, closest_angle, angle_range):
                    left_speed, right_speed = OffenseBlue2.spin(self, left_speed, right_speed, robot_angle, closest_angle,
                                                            spin_speed)
                else:
                    left_speed, right_speed = OffenseBlue2.moveStraight(self, robot_angle, 0, 10)

        # Midfield Defense
        elif self.mode == midfield_defense_mode:
            dest_x, dest_y = 0, 0.54
            angle_range = 8
            if not OffenseBlue2.rangePosition(self, robot_x, robot_y, dest_x, dest_y, 0.02, 0.02):
                left_speed, right_speed = OffenseBlue2.toPoint(self, left_speed, right_speed, dest_x, dest_y, robot_x,
                                                           robot_y, robot_angle, spin_speed, angle_range)
            else:
                closest_angle, closest_difference = OffenseBlue2.direction(self, robot_angle, 90, 270)
                if not OffenseBlue2.rangeAngle(self, robot_angle, closest_angle, angle_range):
                    left_speed, right_speed = OffenseBlue2.spin(self, left_speed, right_speed, robot_angle,
                                                            closest_angle, spin_speed)
                else:
                    left_speed, right_speed = OffenseBlue2.moveStraight(self, robot_angle, 90, -2)

        # Deflection Defense
        elif self.mode == deflection_defense_mode:
            run_pass = True
            dest_x, dest_y = 0, 0.35
            angle_range = 8
            if not OffenseBlue2.rangePosition(self, robot_x, robot_y, dest_x, dest_y, 0.02, 0.02):
                left_speed, right_speed = OffenseBlue2.toPoint(self, left_speed, right_speed, dest_x, dest_y, robot_x,
                                                           robot_y, robot_angle, spin_speed, angle_range)
            else:
                closest_angle, closest_difference = OffenseBlue2.direction(self, robot_angle, 0, 180)
                if not OffenseBlue2.rangeAngle(self, robot_angle, closest_angle, angle_range):
                    left_speed, right_speed = OffenseBlue2.spin(self, left_speed, right_speed, robot_angle,
                                                            closest_angle, spin_speed)
                else:
                    left_speed, right_speed = OffenseBlue2.moveStraight(self, robot_angle, 0, -2)

        # Goal Defense
        elif self.mode == goal_defense_mode:
            dest_x, dest_y = -0.16, 0.8
            angle_range = 8
            if not OffenseBlue2.rangePosition(self, robot_x, robot_y, dest_x, dest_y, 0.02, 0.02):
                left_speed, right_speed = OffenseBlue2.toPoint(self, left_speed, right_speed, dest_x, dest_y, robot_x,
                                                           robot_y, robot_angle, spin_speed, angle_range)
            else:
                closest_angle, closest_difference = OffenseBlue2.direction(self, robot_angle, 90, 270)
                if not OffenseBlue2.rangeAngle(self, robot_angle, closest_angle, angle_range):
                    left_speed, right_speed = OffenseBlue2.spin(self, left_speed, right_speed, robot_angle, closest_angle,
                                                            spin_speed)
                else:
                    left_speed, right_speed = OffenseBlue2.moveStraight(self, robot_angle, 90, -10)

        # Main Trap Ball
        elif self.mode == trap_ball_main_mode:
            run_pass = True

            angle_range = 2
            spin_speed = 4

            if self.trap_stage == 1:
                if ball_y < 0.7 or not offense2_x < ball_x < offense1_x or ball_distance > 0.1 or offense1_ball_distance > 0.1:
                    self.trap_stage = -1
                else:
                    shoot_angle = 40
                    closest_angle, closest_difference = OffenseBlue2.direction(self, robot_angle, (shoot_angle + 360) % 360,
                                                                           (shoot_angle + 360 + 180) % 360)
                    if not OffenseBlue2.rangeAngle(self, robot_angle, closest_angle, angle_range):
                        left_speed, right_speed = OffenseBlue2.spin(self, left_speed, right_speed, robot_angle,
                                                                closest_angle,
                                                                spin_speed)
                    else:
                        move_speed = 0
                        left_speed, right_speed = OffenseBlue2.moveStraight(self, robot_angle, 0, move_speed)

                        if OffenseBlue2.rangePosition(self, offense1_x, offense1_y, offense1_x, 0.67, 0.01, 0.01) and \
                                OffenseBlue2.rangeAngle(self, robot_angle, closest_angle, angle_range):
                            self.trap_stage = 2

            elif self.trap_stage == 2:
                if ball_y < 0.7 or ball_distance > 0.1 or offense1_ball_distance > 0.1:
                    self.trap_stage = -1
                else:
                    shoot_angle = 40
                    closest_angle, closest_difference = OffenseBlue2.direction(self, robot_angle, (shoot_angle + 360) % 360,
                                                                           (shoot_angle + 360 + 180) % 360)
                    if not OffenseBlue2.rangeAngle(self, robot_angle, closest_angle, angle_range):
                        left_speed, right_speed = OffenseBlue2.spin(self, left_speed, right_speed, robot_angle,
                                                                closest_angle,
                                                                spin_speed)
                    else:
                        move_speed = 10
                        left_speed, right_speed = OffenseBlue2.moveStraight(self, robot_angle, 0, move_speed)

                if OffenseBlue2.rangePosition(self, ball_x, ball_y, 0.56, 0.73, 0.01, 0.01):
                    self.trap_stage = 3

            elif self.trap_stage == 3:
                if ball_y < 0.51 or ball_x < 0.43:
                    self.trap_stage = -1
                else:
                    shoot_angle = 40
                    dest_x, dest_y = 0.47, 0.67
                    if not OffenseBlue2.rangePosition(self, robot_x, robot_y, dest_x, dest_y, 0.005, 0.005):
                        left_speed, right_speed = OffenseBlue2.toPoint(self, left_speed, right_speed, dest_x, dest_y,
                                                                   robot_x,
                                                                   robot_y, robot_angle, spin_speed, angle_range)
                    else:
                        closest_angle, closest_difference = OffenseBlue2.direction(self, robot_angle,
                                                                               (shoot_angle + 360) % 360,
                                                                               (shoot_angle + 360 + 180) % 360)
                        if not OffenseBlue2.rangeAngle(self, robot_angle, closest_angle, angle_range):
                            left_speed, right_speed = OffenseBlue2.spin(self, left_speed, right_speed, robot_angle,
                                                                    closest_angle,
                                                                    spin_speed)
                        else:
                            left_speed, right_speed = OffenseBlue2.moveStraight(self, robot_angle, 0, 0)

                            shoot_angle = 0
                            dest_x, dest_y = 0.5715, 0.595
                            closest_angle, closest_difference = OffenseBlue2.direction(self, offense1_angle,
                                                                                   (shoot_angle + 360) % 360,
                                                                                   (shoot_angle + 360 + 180) % 360)
                            if OffenseBlue2.rangePosition(self, offense1_x, offense1_y, dest_x, dest_y, 0.01, 0.01) and \
                                    OffenseBlue2.rangeAngle(self, offense1_angle, closest_angle, angle_range):
                                self.trap_stage = 4

            elif self.trap_stage == 4:
                if ball_y < 0.51 or ball_x < 0.43:
                    self.trap_stage = -1
                else:
                    if not OffenseBlue2.rangePosition(self, ball_x, ball_y, 0.63, 0.64, 0.01, 0.01):
                        dest_x, dest_y = ball_x, ball_y
                        point_angle = (math.degrees(
                            math.atan2(dest_x - robot_x, dest_y - robot_y)) - robot_angle + 180) % 360

                        point_direction = point_angle
                        if point_direction > 180:
                            point_direction = point_direction - 360

                        offset = 0

                        closest_angle, closest_difference = OffenseBlue2.direction(self, point_angle, 0, 180)

                        if closest_angle == 0:
                            self.side = 1
                        elif closest_angle == 180:
                            self.side = -1

                        if self.side == 1:
                            move_angle = point_direction + offset
                        else:
                            if point_direction + offset >= 0:
                                move_angle = point_direction + offset - 180
                            else:
                                move_angle = 180 - abs(point_direction + offset)

                        left_speed, right_speed = OffenseBlue2.moveCurve(self, move_angle, self.side, 45)

                        if OffenseBlue2.rangePosition(self, ball_x, ball_y, 0.58, 0.73, 0.01, 0.01) and \
                                OffenseBlue2.getDistance(
                                    self, ball_x, ball_y, self.last_ball_x, self.last_ball_y) < math.pow(10, -5):
                            self.trap_stage = 3
                        elif OffenseBlue2.rangePosition(self, ball_x, ball_y, 0.63, 0.68, 0.01, 0.01) and \
                                OffenseBlue2.getDistance(
                                    self, ball_x, ball_y, self.last_ball_x, self.last_ball_y) < math.pow(10, -5):
                            self.trap_stage = 5

                    elif OffenseBlue2.rangePosition(self, robot_x, robot_y, 0.57, 0.69, 0.01, 0.01):
                        self.trap_stage = 5

            elif self.trap_stage == 5:
                if ball_x < 0.56 or not offense1_y < ball_y < offense2_y:
                    self.trap_stage = -1
                else:
                    shoot_angle = 140
                    closest_angle, closest_difference = OffenseBlue2.direction(self, robot_angle, (shoot_angle + 360) % 360,
                                                                           (shoot_angle + 360 + 180) % 360)
                    if not OffenseBlue2.rangeAngle(self, robot_angle, closest_angle, angle_range):
                        left_speed, right_speed = OffenseBlue2.spin(self, left_speed, right_speed, robot_angle,
                                                                closest_angle,
                                                                spin_speed)
                    else:
                        move_speed = 0
                        left_speed, right_speed = OffenseBlue2.moveStraight(self, robot_angle, 0, move_speed)

                        self.trap_stage = 6

            elif self.trap_stage == 6:
                if ball_x < 0.56 or ball_distance > 0.1 or offense1_ball_distance > 0.1:
                    self.trap_stage = -1
                else:
                    shoot_angle = 140
                    closest_angle, closest_difference = OffenseBlue2.direction(self, robot_angle, (shoot_angle + 360) % 360,
                                                                           (shoot_angle + 360 + 180) % 360)
                    if not OffenseBlue2.rangeAngle(self, robot_angle, closest_angle, angle_range):
                        left_speed, right_speed = OffenseBlue2.spin(self, left_speed, right_speed, robot_angle,
                                                                closest_angle,
                                                                spin_speed)
                    else:
                        move_speed = -10
                        left_speed, right_speed = OffenseBlue2.moveStraight(self, robot_angle, 0, move_speed)

                    if OffenseBlue2.rangePosition(self, ball_x, ball_y, 0.625, -0.67, 0.01, 0.01):
                        self.trap_stage = 7

            elif self.trap_stage == 7:
                if ball_x < 0.4 or ball_y > -0.55:
                    self.trap_stage = -1
                else:
                    shoot_angle = 140
                    dest_x, dest_y = 0.57, -0.57
                    if not OffenseBlue2.rangePosition(self, robot_x, robot_y, dest_x, dest_y, 0.005, 0.005):
                        left_speed, right_speed = OffenseBlue2.toPoint(self, left_speed, right_speed, dest_x, dest_y,
                                                                   robot_x,
                                                                   robot_y, robot_angle, spin_speed, angle_range)
                    else:
                        closest_angle, closest_difference = OffenseBlue2.direction(self, robot_angle,
                                                                               (shoot_angle + 360) % 360,
                                                                               (shoot_angle + 360 + 180) % 360)
                        if not OffenseBlue2.rangeAngle(self, robot_angle, closest_angle, angle_range):
                            left_speed, right_speed = OffenseBlue2.spin(self, left_speed, right_speed, robot_angle,
                                                                    closest_angle,
                                                                    spin_speed)
                        else:
                            left_speed, right_speed = OffenseBlue2.moveStraight(self, robot_angle, 0, 0)

                            shoot_angle = 90
                            dest_x, dest_y = 0.495, -0.672
                            closest_angle, closest_difference = OffenseBlue2.direction(self, offense1_angle,
                                                                                   (shoot_angle + 360) % 360,
                                                                                   (shoot_angle + 360 + 180) % 360)
                            if OffenseBlue2.rangePosition(self, offense1_x, offense1_y, dest_x, dest_y, 0.01, 0.01) and \
                                    OffenseBlue2.rangeAngle(self, offense1_angle, closest_angle, angle_range):
                                self.trap_stage = 8

            elif self.trap_stage == 8:
                if ball_x < 0.4 or ball_y > -0.55:
                    self.trap_stage = -1
                else:
                    if not OffenseBlue2.rangePosition(self, ball_x, ball_y, 0.54, -0.73, 0.01, 0.01):
                        dest_x, dest_y = ball_x, ball_y
                        point_angle = (math.degrees(
                            math.atan2(dest_x - robot_x, dest_y - robot_y)) - robot_angle + 180) % 360

                        point_direction = point_angle
                        if point_direction > 180:
                            point_direction = point_direction - 360

                        offset = 0

                        closest_angle, closest_difference = OffenseBlue2.direction(self, point_angle, 0, 180)

                        if closest_angle == 0:
                            self.side = 1
                        elif closest_angle == 180:
                            self.side = -1

                        if self.side == 1:
                            move_angle = point_direction + offset
                        else:
                            if point_direction + offset >= 0:
                                move_angle = point_direction + offset - 180
                            else:
                                move_angle = 180 - abs(point_direction + offset)

                        left_speed, right_speed = OffenseBlue2.moveCurve(self, move_angle, self.side, 45)

                        if OffenseBlue2.rangePosition(self, ball_x, ball_y, 0.63, -0.68, 0.01,
                                                  0.01) and OffenseBlue2.getDistance(
                                self, ball_x, ball_y, self.last_ball_x, self.last_ball_y) < math.pow(10, -5):
                            self.trap_stage = 7
                        elif OffenseBlue2.rangePosition(self, ball_x, ball_y, 0.58, -0.73, 0.01, 0.01) and \
                                OffenseBlue2.getDistance(
                                    self, ball_x, ball_y, self.last_ball_x, self.last_ball_y) < math.pow(10,
                                                                                                         -5) and ball_distance < 0.05:
                            self.trap_stage = 9

                    elif OffenseBlue2.rangePosition(self, robot_x, robot_y, 0.59, -0.67, 0.01, 0.01):
                        self.trap_stage = 9

            elif self.trap_stage == 9:
                if ball_y > -0.69 or not offense2_x > ball_x > offense1_x:
                    self.trap_stage = -1
                else:
                    shoot_angle = 40
                    closest_angle, closest_difference = OffenseBlue2.direction(self, robot_angle, (shoot_angle + 360) % 360,
                                                                           (shoot_angle + 360 + 180) % 360)
                    if not OffenseBlue2.rangeAngle(self, robot_angle, closest_angle, angle_range):
                        left_speed, right_speed = OffenseBlue2.spin(self, left_speed, right_speed, robot_angle,
                                                                closest_angle,
                                                                spin_speed)
                    else:
                        move_speed = 0
                        left_speed, right_speed = OffenseBlue2.moveStraight(self, robot_angle, 0, move_speed)

                        self.trap_stage = 10

            elif self.trap_stage == 10:
                if ball_x < 0.1 or not -0.77 < ball_y < -0.68 or ball_distance > 0.1 or offense1_ball_distance > 0.1:
                    self.trap_stage = -1
                else:
                    shoot_angle = 40
                    closest_angle, closest_difference = OffenseBlue2.direction(self, robot_angle, (shoot_angle + 360) % 360,
                                                                           (shoot_angle + 360 + 180) % 360)
                    if not OffenseBlue2.rangeAngle(self, robot_angle, closest_angle, angle_range):
                        left_speed, right_speed = OffenseBlue2.spin(self, left_speed, right_speed, robot_angle,
                                                                closest_angle,
                                                                spin_speed)
                    else:
                        move_speed = -10
                        left_speed, right_speed = OffenseBlue2.moveStraight(self, robot_angle, 0, move_speed)

        # Support Trap Ball
        elif self.mode == trap_ball_support_mode:
            run_pass = True

            angle_range = 2
            spin_speed = 4

            robot_radius = 0.0375
            ball_radius = 0.021

            if trap_stage == 1:
                shoot_angle = 90
                dest_x, dest_y = robot_x, 0.672
                if not OffenseBlue2.rangePosition(self, robot_x, robot_y, dest_x, dest_y, 0.01, 0.01):
                    left_speed, right_speed = OffenseBlue2.toPoint(self, left_speed, right_speed, dest_x, dest_y,
                                                               robot_x,
                                                               robot_y, robot_angle, spin_speed, angle_range)
                else:
                    closest_angle, closest_difference = OffenseBlue2.direction(self, robot_angle,
                                                                           (shoot_angle + 360) % 360,
                                                                           (shoot_angle + 360 + 180) % 360)
                    if not OffenseBlue2.rangeAngle(self, robot_angle, closest_angle, angle_range):
                        left_speed, right_speed = OffenseBlue2.spin(self, left_speed, right_speed, robot_angle,
                                                                closest_angle,
                                                                spin_speed)
                    else:
                        left_speed, right_speed = OffenseBlue2.moveStraight(self, robot_angle, 0, 0)

            elif trap_stage == 2:
                shoot_angle = 90
                dest_x, dest_y = ball_x - robot_radius - ball_radius + 0.008, 0.672
                if not OffenseBlue2.rangePosition(self, robot_x, robot_y, dest_x, dest_y, 0.005, 0.005):
                    left_speed, right_speed = OffenseBlue2.toPoint(self, left_speed, right_speed, dest_x, dest_y, robot_x,
                                                               robot_y, robot_angle, spin_speed, angle_range)
                else:
                    closest_angle, closest_difference = OffenseBlue2.direction(self, robot_angle, (shoot_angle + 360) % 360,
                                                                           (shoot_angle + 360 + 180) % 360)
                    if not OffenseBlue2.rangeAngle(self, robot_angle, closest_angle, angle_range):
                        left_speed, right_speed = OffenseBlue2.spin(self, left_speed, right_speed, robot_angle,
                                                                closest_angle, spin_speed)
                    else:
                        left_speed, right_speed = OffenseBlue2.moveStraight(self, robot_angle, 0, 0)

            elif trap_stage == 3:
                if OffenseBlue2.rangePosition(self, offense1_x, offense1_y, -0.47, 0.67, 0.005, 0.005):
                    shoot_angle = 0
                    dest_x, dest_y = -0.5715, 0.595
                    if not OffenseBlue2.rangePosition(self, robot_x, robot_y, dest_x, dest_y, 0.005, 0.005):
                        left_speed, right_speed = OffenseBlue2.toPoint(self, left_speed, right_speed, dest_x, dest_y,
                                                                   robot_x,
                                                                   robot_y, robot_angle, spin_speed, angle_range)
                    else:
                        closest_angle, closest_difference = OffenseBlue2.direction(self, robot_angle,
                                                                               (shoot_angle + 360) % 360,
                                                                               (shoot_angle + 360 + 180) % 360)
                        if not OffenseBlue2.rangeAngle(self, robot_angle, closest_angle, angle_range):
                            left_speed, right_speed = OffenseBlue2.spin(self, left_speed, right_speed, robot_angle,
                                                                    closest_angle,
                                                                    spin_speed)
                        else:
                            left_speed, right_speed = OffenseBlue2.moveStraight(self, robot_angle, 0, 0)

            elif trap_stage == 4:
                shoot_angle = 0
                dest_x, dest_y = -0.5715, 0.595
                if not OffenseBlue2.rangePosition(self, robot_x, robot_y, dest_x, dest_y, 0.005, 0.005):
                    left_speed, right_speed = OffenseBlue2.toPoint(self, left_speed, right_speed, dest_x, dest_y,
                                                               robot_x,
                                                               robot_y, robot_angle, spin_speed, angle_range)
                else:
                    closest_angle, closest_difference = OffenseBlue2.direction(self, robot_angle,
                                                                           (shoot_angle + 360) % 360,
                                                                           (shoot_angle + 360 + 180) % 360)
                    if not OffenseBlue2.rangeAngle(self, robot_angle, closest_angle, angle_range):
                        left_speed, right_speed = OffenseBlue2.spin(self, left_speed, right_speed, robot_angle,
                                                                closest_angle,
                                                                spin_speed)
                    else:
                        left_speed, right_speed = OffenseBlue2.moveStraight(self, robot_angle, 0, -2)

            elif trap_stage == 5:
                shoot_angle = 0
                dest_x, dest_y = -0.5715, 0.57
                if not OffenseBlue2.rangePosition(self, robot_x, robot_y, dest_x, dest_y, 0.005, 0.005):
                    left_speed, right_speed = OffenseBlue2.toPoint(self, left_speed, right_speed, dest_x, dest_y,
                                                               robot_x,
                                                               robot_y, robot_angle, spin_speed, angle_range)
                else:
                    closest_angle, closest_difference = OffenseBlue2.direction(self, robot_angle,
                                                                           (shoot_angle + 360) % 360,
                                                                           (shoot_angle + 360 + 180) % 360)
                    if not OffenseBlue2.rangeAngle(self, robot_angle, closest_angle, angle_range):
                        left_speed, right_speed = OffenseBlue2.spin(self, left_speed, right_speed, robot_angle,
                                                                closest_angle,
                                                                spin_speed)
                    else:
                        left_speed, right_speed = OffenseBlue2.moveStraight(self, robot_angle, 0, 0)

            elif trap_stage == 6:
                shoot_angle = 0
                dest_x, dest_y = -0.5715, ball_y - robot_radius - ball_radius + 0.008
                if not OffenseBlue2.rangePosition(self, robot_x, robot_y, dest_x, dest_y, 0.005, 0.005):
                    left_speed, right_speed = OffenseBlue2.toPoint(self, left_speed, right_speed, dest_x, dest_y, robot_x,
                                                               robot_y, robot_angle, spin_speed, angle_range)
                else:
                    closest_angle, closest_difference = OffenseBlue2.direction(self, robot_angle, (shoot_angle + 360) % 360,
                                                                           (shoot_angle + 360 + 180) % 360)
                    if not OffenseBlue2.rangeAngle(self, robot_angle, closest_angle, angle_range):
                        left_speed, right_speed = OffenseBlue2.spin(self, left_speed, right_speed, robot_angle,
                                                                closest_angle, spin_speed)
                    else:
                        left_speed, right_speed = OffenseBlue2.moveStraight(self, robot_angle, 0, 0)

            elif trap_stage == 7:
                if OffenseBlue2.rangePosition(self, offense1_x, offense1_y, -0.57, -0.57, 0.005, 0.005):
                    shoot_angle = 90
                    dest_x, dest_y = -0.495, -0.672
                    if not OffenseBlue2.rangePosition(self, robot_x, robot_y, dest_x, dest_y, 0.005, 0.005):
                        left_speed, right_speed = OffenseBlue2.toPoint(self, left_speed, right_speed, dest_x, dest_y,
                                                                   robot_x,
                                                                   robot_y, robot_angle, spin_speed, angle_range)
                    else:
                        closest_angle, closest_difference = OffenseBlue2.direction(self, robot_angle,
                                                                               (shoot_angle + 360) % 360,
                                                                               (shoot_angle + 360 + 180) % 360)
                        if not OffenseBlue2.rangeAngle(self, robot_angle, closest_angle, angle_range):
                            left_speed, right_speed = OffenseBlue2.spin(self, left_speed, right_speed, robot_angle,
                                                                    closest_angle,
                                                                    spin_speed)
                        else:
                            left_speed, right_speed = OffenseBlue2.moveStraight(self, robot_angle, 0, 0)

            elif trap_stage == 8:
                shoot_angle = 90
                dest_x, dest_y = -0.495, -0.672
                if not OffenseBlue2.rangePosition(self, robot_x, robot_y, dest_x, dest_y, 0.005, 0.005):
                    left_speed, right_speed = OffenseBlue2.toPoint(self, left_speed, right_speed, dest_x, dest_y,
                                                               robot_x,
                                                               robot_y, robot_angle, spin_speed, angle_range)
                else:
                    closest_angle, closest_difference = OffenseBlue2.direction(self, robot_angle,
                                                                           (shoot_angle + 360) % 360,
                                                                           (shoot_angle + 360 + 180) % 360)
                    if not OffenseBlue2.rangeAngle(self, robot_angle, closest_angle, angle_range):
                        left_speed, right_speed = OffenseBlue2.spin(self, left_speed, right_speed, robot_angle,
                                                                closest_angle,
                                                                spin_speed)
                    else:
                        left_speed, right_speed = OffenseBlue2.moveStraight(self, robot_angle, 90, -2)

            elif trap_stage == 9:
                shoot_angle = 90
                dest_x, dest_y = -0.495, -0.672
                if not OffenseBlue2.rangePosition(self, robot_x, robot_y, dest_x, dest_y, 0.005, 0.005):
                    left_speed, right_speed = OffenseBlue2.toPoint(self, left_speed, right_speed, dest_x, dest_y, robot_x,
                                                               robot_y, robot_angle, spin_speed, angle_range)
                else:
                    closest_angle, closest_difference = OffenseBlue2.direction(self, robot_angle, (shoot_angle + 360) % 360,
                                                                           (shoot_angle + 360 + 180) % 360)
                    if not OffenseBlue2.rangeAngle(self, robot_angle, closest_angle, angle_range):
                        left_speed, right_speed = OffenseBlue2.spin(self, left_speed, right_speed, robot_angle,
                                                                closest_angle,
                                                                spin_speed)
                    else:
                        left_speed, right_speed = OffenseBlue2.moveStraight(self, robot_angle, 0, 0)

            elif trap_stage == 10:
                shoot_angle = 90
                dest_x, dest_y = ball_x + robot_radius + ball_radius - 0.008, -0.672
                if not OffenseBlue2.rangePosition(self, robot_x, robot_y, dest_x, dest_y, 0.005, 0.005):
                    left_speed, right_speed = OffenseBlue2.toPoint(self, left_speed, right_speed, dest_x, dest_y, robot_x,
                                                               robot_y, robot_angle, spin_speed, angle_range)
                else:
                    closest_angle, closest_difference = OffenseBlue2.direction(self, robot_angle, (shoot_angle + 360) % 360,
                                                                           (shoot_angle + 360 + 180) % 360)
                    if not OffenseBlue2.rangeAngle(self, robot_angle, closest_angle, angle_range):
                        left_speed, right_speed = OffenseBlue2.spin(self, left_speed, right_speed, robot_angle,
                                                                closest_angle, spin_speed)
                    else:
                        left_speed, right_speed = OffenseBlue2.moveStraight(self, robot_angle, 0, 0)

        distance_range = 0.1
        temp_pred_x, temp_pred_y = 2 * ball_x - self.last_ball_x, 2 * ball_y - self.last_ball_y
        temp_pred_distance = OffenseBlue2.getDistance(self, robot_x, robot_y, temp_pred_x, temp_pred_y)
        if not run_pass and ball_y > 0 and -0.23 < robot_x < 0.23 and robot_y > 0 and (
                (ball_distance < distance_range and ball_y > robot_y) or (
                temp_pred_distance < distance_range and temp_pred_y > robot_y) or (
                        OffenseBlue2.getDistance(self, robot_x, robot_y, self.last_ball_x,
                                             self.last_ball_y) < distance_range and self.last_ball_y > robot_y)):
            left_speed, right_speed = 0, 0

        self.last_ball_x, self.last_ball_y = ball_x, ball_y

        self.last_mode = self.mode

        robot.left_motor.setVelocity(left_speed)
        robot.right_motor.setVelocity(right_speed)

    def reset(self):
        self.mode = 0
        self.side = 1
        self.goal_x, self.goal_y = 0, -0.75
        self.off_angle = 25
        self.last_ball_x, self.last_ball_y = 0, 0
        self.last_mode = 0
        self.limit_x = -0.12
        self.timer = 0
        self.push_ball = False
        self.trap_stage = -1
        self.double_push_x, self.double_push_y = 0, 0
        self.wall_dest_x, self.wall_dest_y = 0.44, 0.7
        self.team = "B"

    # Add any additional functions here
    def moveCurve(self, dir, side, ang=65):
        # Side (+: front, -: back)
        left_speed = side * -10 * (ang + dir * side) / ang
        right_speed = side * -10 * (ang - dir * side) / ang

        if left_speed > 10:
            left_speed = 10
        elif left_speed < -10:
            left_speed = -10
        if right_speed > 10:
            right_speed = 10
        elif right_speed < -10:
            right_speed = -10

        return left_speed, right_speed

    def direction(self, robot_angle, angle_a, angle_b):
        angle_a_left = (robot_angle - angle_a + 360) % 360
        angle_a_right = (angle_a - robot_angle + 360) % 360
        angle_b_left = (robot_angle - angle_b + 360) % 360
        angle_b_right = (angle_b - robot_angle + 360) % 360

        angle_smallest = min([angle_a_left, angle_a_right, angle_b_left, angle_b_right])

        if angle_smallest == angle_a_left:
            return angle_a, -angle_a_left
        elif angle_smallest == angle_a_right:
            return angle_a, angle_a_right
        elif angle_smallest == angle_b_left:
            return angle_b, -angle_b_left
        else:
            return angle_b, -angle_b_right

    def toPoint(self, left_speed, right_speed, dest_x, dest_y, robot_x, robot_y, robot_angle, spin_speed,
                angle_range):
        closest_angle, absolute_angle = OffenseBlue2.coords(self, robot_angle, robot_x, robot_y, dest_x, dest_y)
        if not OffenseBlue2.rangeAngle(self, robot_angle, closest_angle, angle_range):
            left_speed, right_speed = OffenseBlue2.spin(self, left_speed, right_speed, robot_angle, closest_angle,
                                                    spin_speed)
        else:
            if self.team == "B":
                left_speed, right_speed = OffenseBlue2.moveStraight(self, robot_angle, absolute_angle, -10)
            else:
                left_speed, right_speed = OffenseBlue2.moveStraight(self, robot_angle, absolute_angle, 10)
        return left_speed, right_speed

    def coords(self, robot_angle, robot_x, robot_y, dest_x, dest_y):
        absolute_angle = (math.degrees(math.atan2(robot_x - dest_x, robot_y - dest_y))) % 360
        closest_angle, closest_difference = OffenseBlue2.direction(self, robot_angle, absolute_angle,
                                                               (absolute_angle + 180) % 360)
        return closest_angle, absolute_angle

    def spin(self, left_speed, right_speed, robot_angle, angle, spin_speed):
        closest_angle, closest_difference = OffenseBlue2.direction(self, robot_angle, angle, (angle + 180) % 360)
        # The closest difference is relative to the robot
        if closest_difference > 0:
            left_speed = -spin_speed
            right_speed = spin_speed
        elif closest_difference < 0:
            left_speed = spin_speed
            right_speed = -spin_speed
        return left_speed, right_speed

    def moveStraight(self, robot_angle, target_angle, move_speed):
        closest_angle, closest_difference = OffenseBlue2.direction(self, target_angle, robot_angle,
                                                               (robot_angle + 180) % 360)
        if closest_angle == robot_angle:
            left_speed = move_speed
            right_speed = move_speed
        else:
            left_speed = -move_speed
            right_speed = -move_speed
        return left_speed, right_speed

    def rangePosition(self, pos_x, pos_y, dest_x, dest_y, range_x=0.2, range_y=0.2, range_x_up=None, range_x_down=None,
                      range_y_up=None, range_y_down=None):
        if range_x_up is None:
            range_x_up = range_x
        if range_x_down is None:
            range_x_down = range_x
        if range_y_up is None:
            range_y_up = range_y
        if range_y_down is None:
            range_y_down = range_y
        if pos_x > dest_x + range_x_up or pos_x < dest_x - range_x_down or pos_y > dest_y + range_y_up or pos_y < dest_y - range_y_down:
            return False
        return True

    def rangeAngle(self, angle, target_angle, angle_range=15, angle_left_range=None, angle_right_range=None,
                   both_sides=True):
        if angle_left_range is None:
            angle_left_range = angle_range
        if angle_right_range is None:
            angle_right_range = angle_range
        if both_sides:
            closest_angle, closest_difference = OffenseBlue2.direction(self, angle, target_angle,
                                                                   (target_angle + 180) % 360)
        else:
            closest_angle, closest_difference = OffenseBlue2.direction(self, angle, target_angle, target_angle)
        if angle_right_range > closest_difference > -angle_left_range:
            return True
        return False

    def linearRescale(self, raw_value, min_old_value, max_old_value, min_new_value, max_new_value):
        if raw_value >= max_old_value:
            return max_new_value
        elif raw_value <= min_old_value:
            return min_new_value
        else:
            slope = (max_new_value - min_new_value) / (max_old_value - min_old_value)
            intercept = slope * min_old_value - min_new_value
            new_value = slope * raw_value + intercept
        return new_value

    def findPoint(self, input_value, point_a_x, point_a_y, point_b_x, point_b_y, find_y=True):
        point_a_x, point_a_y = -point_a_x, point_a_y
        point_b_x, point_b_y = -point_b_x, point_b_y
        input_value = -input_value

        point_angle = math.atan2(point_b_y - point_a_y, point_b_x - point_a_x)
        point_slope = math.tan(point_angle)
        point_intercept = point_a_y - point_slope * point_a_x

        if not find_y:
            return -(input_value - point_intercept) / point_slope, input_value
        return -input_value, point_slope * input_value + point_intercept

    def findAttackPoint(self, point_x, ball_x, ball_y, last_ball_x, last_ball_y, offset_front=0.0,
                        offset_back=0.0):
        ball_radius = 0.021
        robot_short_radius = 0.0375  # 0.0383
        robot_long_radius = 0.0475

        main_x, main_y = OffenseBlue2.findPoint(self, point_x, ball_x, ball_y, last_ball_x, last_ball_y)
        main_x, main_y = main_y, -main_x

        point_y = -point_x
        ball_x, ball_y = ball_y, -ball_x
        last_ball_x, last_ball_y = last_ball_y, -last_ball_x

        theta = math.atan2(last_ball_y - ball_y, last_ball_x - ball_x)
        abs_theta = abs(theta)

        if theta == 0:
            return None, None

        # Finding first gap length
        gap_1 = ball_radius / math.sin(abs_theta)
        length_1 = gap_1 * math.tan(abs_theta)

        # Finding second gap length
        slope_original = math.tan(theta)
        intercept_original = (ball_y - slope_original * ball_x)
        if point_y > ball_y:
            intercept_original += length_1
        else:
            intercept_original -= length_1
        slope_perpendicular = -1 / slope_original
        intercept_perpendicular = (ball_y - slope_perpendicular * ball_x)

        left_matrix = np.array([[-slope_original, 1], [-slope_perpendicular, 1]])
        right_matrix = np.array([intercept_original, intercept_perpendicular])
        solution = np.linalg.solve(left_matrix, right_matrix).tolist()
        solution_x, solution_y = solution[0], solution[1]

        temp_length = math.sqrt(math.pow((main_x + gap_1) - solution_x, 2) + math.pow(main_y - solution_y, 2))
        length_2 = temp_length * math.sin(abs_theta)

        if self.team == "B":
            if ball_x > main_x:
                if length_2 >= robot_long_radius:
                    gap_2 = robot_long_radius / math.tan(abs_theta)
                    dest_x, dest_y = main_x + gap_1 + gap_2 + robot_short_radius + offset_back, point_y
                else:
                    dest_x, dest_y = ball_x + ball_radius + robot_short_radius + offset_back, point_y
            else:
                dest_x, dest_y = main_x + ball_radius + robot_short_radius + offset_front, point_y
        else:
            if ball_x < main_x:
                if length_2 >= robot_long_radius:
                    gap_2 = robot_long_radius / math.tan(abs_theta)
                    dest_x, dest_y = main_x - gap_1 - gap_2 - robot_short_radius - offset_back, point_y
                else:
                    dest_x, dest_y = ball_x - ball_radius - robot_short_radius - offset_back, point_y
            else:
                dest_x, dest_y = main_x - ball_radius - robot_short_radius - offset_front, point_y

        return -dest_y, dest_x

    def getDistance(self, point_a_x, point_a_y, point_b_x, point_b_y):
        return math.sqrt(math.pow(point_a_x - point_b_x, 2) + math.pow(point_a_y - point_b_y, 2))

