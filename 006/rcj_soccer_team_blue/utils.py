from math import *

def get_direction(x):
    if x <= 15 or x >= 365:
        return 0
    if 165 <= x <= 195:
        return 2
    if x > 270:
        return -1
    if x < 90:
        return 1
    return 3 if x < 180 else -3

def state_manager(ball_pos, team_pos, heading, robot_id, team, robot_angle):
    dists = get_distences_from_ball(team_pos, ball_pos[0])
    if robot_id != 1:
        #if ball_pos[0][1] > team_pos[robot_id-1][1] + 0.1:
        #    return goal_prevention(team_pos[robot_id-1], ball_pos[0], heading, robot_angle)
        if team_pos[robot_id-1][1] < -0.55 and ball_pos[0][1] < -0.6 and abs(ball_pos[0][1] - ball_pos[1][1]) < 0.03 and (abs(ball_pos[0][0]-0.2) > abs(ball_pos[1][0]-0.2) or abs(ball_pos[0][0]+0.2) > abs(ball_pos[1][0]+0.2)) and 0.2 > team_pos[robot_id-1][0] > -0.2 and team == 'B':
            return shoot_in_offense(team_pos[robot_id-1], heading, ball_pos ,team)
        if team_pos[robot_id-1][1] > 0.55 and ball_pos[0][1] > 0.6 and abs(ball_pos[0][1] - ball_pos[1][1]) < 0.03 and (abs(ball_pos[0][0]-0.2) > abs(ball_pos[1][0]-0.2) or abs(ball_pos[0][0]+0.2) > abs(ball_pos[1][0]+0.2)) and 0.2 > team_pos[robot_id-1][0] > -0.2 and team == 'Y':
            return shoot_in_offense(team_pos[robot_id-1], heading, ball_pos ,team)
        if min(dists[1:]) == dists[robot_id-1] or (dists[robot_id-1] <= 0.3 and abs(dists[1] - dists[2]) > 0.3):
            return atacker(ball_pos, team_pos[robot_id-1], robot_angle, team)
        return defender(team_pos[robot_id-1], team, ball_pos[0], robot_angle, team_pos)
    target_pos = 0.57 if team == 'B' else -0.57
    return goalkeeper(team_pos[robot_id-1], heading, ball_pos[0], target_pos, team)

def defender(robot_pos, team, ball_pos, robot_angle, team_pos):
    angle = get_angles([0.07 if ball_pos[0] < 0 else -0.07, ball_pos[1]  + 0.07 if team == 'B' else ball_pos[1] - 0.07], robot_pos, robot_angle)
    direction = get_direction(angle)
    if( ball_pos[1] > 0.675 and team == 'B') or (ball_pos[1] < -0.675 and team == 'Y'):
        return help_goalkeeper(team_pos, robot_angle, robot_pos, ball_pos)
    if direction == 0:
        return (10, 10)
    if direction == 2:
        return (-10, -10)
    return ((10, -5) if direction == -1 else (-5, 10)) if direction in [-1,1] else ((-10, 5) if direction == -3 else (5, -10))

def atacker(ball_pos, robot_pos, robot_angle, team):
    ball_angle = get_angles([ball_pos[0][0], ball_pos[0][1]], robot_pos, robot_angle)
    direction = get_direction(ball_angle)
    distance = get_dist_f_ball(ball_pos[0], robot_pos)
    if (((-0.2 < robot_pos[0] < 0.2 and robot_pos[1] > 0.5 and ball_pos[0][1] > robot_pos[1]) or (0.2 > ball_pos[0][0] > -0.2 and ball_pos[0][1] > 0.625)) and team == 'B') or (((-0.2 < robot_pos[0] < 0.2 and robot_pos[1] < -0.5 and ball_pos[0][1] > robot_pos[1]) or (0.2 > ball_pos[0][0] > -0.2 and ball_pos[0][1] < -0.625)) and team == 'Y'):
        direction = get_direction(get_angles([0,0], robot_pos, robot_angle))
        distance = 5
    elif (robot_pos[1] > 0.665 or (robot_pos[1] > 0.65 and (robot_pos[0] > 0.575 or robot_pos[0] < -0.575))) and (abs(robot_pos[0]) - abs(ball_pos[0][0]) > 0 and ((robot_pos[0] > 0 and ball_pos[0][0] > 0) or (robot_pos[0] < 0 and ball_pos[0][0] < 0))) and team == 'B':
        return (-5,-10) if robot_pos[0] > 0 else (-10,-5)
    elif (robot_pos[1] < -0.665 or (robot_pos[1] < -0.65 and (robot_pos[0] > 0.575 or robot_pos[0] < -0.575))) and (abs(robot_pos[0]) - abs(ball_pos[0][0]) > 0 and ((robot_pos[0] > 0 and ball_pos[0][0] > 0) or (robot_pos[0] < 0 and ball_pos[0][0] < 0))) and team == 'Y':
        return (-5,-10) if robot_pos[0] > 0 else (-10,-5)
    elif distance < 0.07:
        if 50 < ball_angle < 130 or 230 < ball_angle < 310:
            return (-10,10) if ball_angle < 90 or 230 < ball_angle < 270 else (10,-10)
        angle = get_angles([0, -0.75] if team == 'B' else [0,0.75], robot_pos, robot_angle)
        vel = (180 - (angle - 180)) / 18 if angle > 180 else angle / 18
        if 90 < ball_angle < 270:
            return (-10, -vel) if ((ball_pos[0][0] > robot_pos[0] and ball_pos[0][1] < robot_pos[1]) or (ball_pos[0][1] > robot_pos[1] and ball_pos[0][0] < robot_pos[0]) or (robot_pos[0] > 0.575)) and not robot_pos[0] < -0.575 else (-vel, -10)
        return (10, vel) if ((ball_pos[0][0] > robot_pos[0] and ball_pos[0][1] < robot_pos[1]) or (ball_pos[0][1] > robot_pos[1] and ball_pos[0][0] < robot_pos[0]) or (robot_pos[0] > 0.575)) and not robot_pos[0] < -0.575 else (vel, 10)
    if direction == 0:
        return (10, 10)
    if direction == 2:
        return (-10, -10)
    if distance < 0.15:
        return ((10, 5) if direction == -1 else (5, 10)) if direction in [-1,1] else ((-10, -5) if direction == -3 else (-5, -10))
    return ((10, -7) if direction == -1 else (-7, 10)) if direction in [-1,1] else ((-10, 3) if direction == -3 else (3, -10))

def shoot_in_offense(robot_pos, heading, ball_pos ,team):
    rad = get_compass_heading_radian(heading)
    deg = (rad/pi*180+360) % 360 
    if (-0.625 < robot_pos[1] and team == 'B') or (0.625 > robot_pos[1] and team == 'Y'):
        return (10, 10) if ((deg + 90) % 360 < 180 if team == 'B' else (deg + 90) % 360 > 180) else (-10,-10) 
    if get_dist_f_ball(ball_pos[0], robot_pos) < 0.2:
        return (9,9) if ((rad/pi*180+90) % 360 < 180 if team == 'B' else (rad/pi*180+90) % 360 > 180) else (-9,-9)
    if ball_pos[0][0] > 0:
        return (10,-10) if (315 > deg > 225 or 135 > deg > 45 if team == 'B' else not 315 > deg > 225 and 135 > deg > 45) else (-10,10)
    return (-10,10) if (315 > deg > 225 or 135 > deg > 45 if team == 'B' else not 315 > deg > 225 and 135 > deg > 45) else (10,-10)

def goal_prevention(robot_pos, ball_pos, heading, robot_angle):
    if ball_pos[0] < robot_pos[0]:
        b = [ball_pos[0] + 0.08, ball_pos[1]]
    else:
        b = [ball_pos[0] - 0.07, ball_pos[1]]
    angle = get_angles(b, robot_pos, robot_angle)
    direction = get_direction(angle)
    if direction == 0:
        return (10,10)
    if direction == 2:
        return (-10,-10)
    return ((10,-10) if direction == -1 else (-10,10)) if direction in [-1,1] else ((10,-10) if direction == 3 else (-10,10))

def help_goalkeeper(team_pos, robot_angle, robot_pos, ball_pos):
    angle = get_angles([team_pos[0][0] + 0.05 if ball_pos[0] < 0 else team_pos[0][0] - 0.05, team_pos[0][1]], robot_pos, robot_angle)
    direction = get_direction(angle)
    if direction == 0:
        return (10,10)
    return (10,-10) if direction == -1 else (-10,10)

def get_future_y(x):
    try: 
        return [0, (x[0][0] / (x[1][0] - x[0][0])) * (x[1][1] - x[0][1]) + x[0][1]]
    except Exception:
        return [0,0]

def get_future_ball_positions(x):
    return [((x[0][0] - x[1][0])*i + x[0][0], (x[0][1] - x[1][1])*i + x[0][1]) for i in range(1, 11)]

def ball_prediction(ball_data, robot_pos):
    future_ball_pos = get_future_ball_positions(ball_data)
    return future_ball_pos[10]

#def ball_prediction(ball_data, robot_pos):
#    future_ball_pos = get_future_ball_positions(ball_data)
#    for i, (x, y) in enumerate(future_ball_pos):
#        dist = get_dist_f_ball(robot_pos, [x, y])
#        if (i+1) * 0.005 > dist:
#            return (x, y)
#    return -1

def ball_angle(ball_vector):
    deg = (1 - ball_vector[0]) * 90
    if ball_vector[1] >= 0:
        deg = -deg
    return deg

def c_add(a, b):
    return [a[0]+b[0], a[1]+b[1]]

def c_mul(a, b):
    return [a[0]*b[0] - a[1]*b[1], a[0]*b[1]+a[1]*b[0]]

def get_ball_pos(bv, s, h, r):
    return (lambda x: [x[0]+r[0],-x[1]+r[1]])(c_mul([0, -1], c_mul(h, [bv[0] / (s ** 0.5), bv[1] / (s ** 0.5)])))

def get_dist_f_ball(a, b):
    return sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

def get_distences_from_ball(a, b):
    return [get_dist_f_ball(i, b) for i in a]

def rotate_to_position(heading, orientation, robot_pos,name):
    if name == 'B':
        if 0.87 <= heading[0]:
            return (10,10) if orientation == 'S' else (-10,-10)
        return (10,-10) if robot_pos[0] < 0 else (-10,10)
    if -0.87 >= heading[0]:
        return (10,10) if orientation == 'N' else (-10,-10)
    return (10,-10) if robot_pos[0] < 0 else (-10,10)

def goalkeeper_(robot_pos,robot_angle,ball_pos,target_pos,name):
    if ball_pos[1] >= target_pos:
        target_pos = 0.65
    angle = get_angles([ball_pos[0], target_pos], robot_pos, robot_angle)
    direction = get_direction(angle)
    if direction == 0:
        return (10, 10)
    if direction == 2:
        return (-10, -10)
    return ((10, -10) if direction == -1 else (-10, 10)) if direction in [-1,1] else ((-10, 10) if direction == 3 else (10, -10))

def goalkeeper(rob_pos,heading,ball_pos,target_pos,name):
    if name == 'B':
        if ball_pos[1] >= target_pos + 0.3:
            target_pos= 0.72
            #return defender(rob_pos, name, ball_pos, get_compass_heading_radian(heading))
        if rob_pos[1] < target_pos - 0.04:
            return rotate_to_position(heading, 'S', rob_pos,name)
        elif rob_pos[1] >= target_pos + 0.04:
            return rotate_to_position(heading, 'N', rob_pos,name)
        else:
            if -0.15 < heading[0] < 0.15:
                return (-10,-10) if ball_pos[0] < rob_pos[0] else (10,10)
            return (10,-10) if heading[0] < 0 else (-10,10)
    if ball_pos[1] <= target_pos - 0.3:
        target_pos = 0.72
    if rob_pos[1] < target_pos - 0.04:
        return rotate_to_position(heading, 'S', rob_pos,name)
    elif rob_pos[1] >= target_pos + 0.04:
        return rotate_to_position(heading, 'N', rob_pos,name)
    else:
        if -0.15 < heading[0] < 0.15:
            return (10,10) if ball_pos[0] > rob_pos[0] else (-10,-10)
        return (10,-10) if heading[0] < 0 else (-10,10)

def get_compass_heading_radian(x):
    rad = atan2(x[0], x[1]) + (pi / 2)
    if rad < -pi:
        rad = rad + (2 * pi)
    return rad

def get_angles(ball_pos, robot_pos, robot_angle):
    angle = atan2(ball_pos[0] - robot_pos[0], ball_pos[1] - robot_pos[1])
    if angle < 0:
        angle = 2 * pi + angle
    if robot_angle < 0:
        robot_angle = 2 * pi + robot_angle
    robot_ball_angle = (degrees(angle + robot_angle)-180) % 360
    if robot_ball_angle<0: robot_ball_angle+=360
    return robot_ball_angle

def wall(ball_pos, team):
    if ball_pos[1] > 0.695:
        ball_pos[1] = 0.695
    elif ball_pos[1] < -0.695:
        ball_pos = [ball_pos[0] + 0.03 if ball_pos[0] > 0 else ball_pos[0] - 0.03, -0.695]
    if ball_pos[0] > 0.582:
        ball_pos = [0.582, ball_pos[1] + 0.03 if team == 'B' else ball_pos[1] - 0.03]
    elif ball_pos[0] < -0.582:
        ball_pos = [-0.582, ball_pos[1] + 0.03 if team == 'B' else ball_pos[1] - 0.03]
    if ball_pos[1] > 0.695:
        ball_pos[1] = 0.695