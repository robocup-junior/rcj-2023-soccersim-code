import math
from rcj_soccer_robot import TIME_STEP

class Data:
    team = ""
    old_coord = False
    robot_id = 0
    robot_x = 0
    robot_y = 0
    robot_heading = 0
    robot_flipped = False
    ball_x = 0
    ball_y = 0
    ball_angle = 0
    ball_distance = 0
    ball_detect = False
    last_robot_x = 0
    last_robot_y = 0
    last_robot_heading = 0
    last_ball_x = 0
    last_ball_y = 0
    last_ball_distance = 0
    last_detect = False
    last_flipped = False
    ball_pred_x = 0
    ball_pred_y = 0
    ball_pred_angle = 0
    ball_pred_distance = 0
    pred_last_ball_x = 0
    pred_last_ball_y = 0
    pred_n = 0
    teammate1_id = 0
    teammate1_x = 0
    teammate1_y = 0
    teammate1_heading = 0
    teammate1_ball_x = 0
    teammate1_ball_y = 0
    teammate1_strength = 0
    teammate1_detect = False
    teammate1_flipped = False
    teammate1_ball_distance = 0
    teammate2_id = 0
    teammate2_x = 0
    teammate2_y = 0
    teammate2_heading = 0
    teammate2_ball_x = 0
    teammate2_ball_y = 0
    teammate2_strength = 0
    teammate2_detect = False
    teammate2_flipped = False
    teammate2_ball_distance = 0

    def __init__(self, name, pred_n=40, old_coord=False):
        if "B" in name:
            self.team = "B"
        else:
            self.team = "Y"

        if "1" in name:
            self.robot_id = 1
            self.teammate1_id = 2
            self.teammate2_id = 3
        elif "2" in name:
            self.robot_id = 2
            self.teammate1_id = 1
            self.teammate2_id = 3
        else:
            self.robot_id = 3
            self.teammate1_id = 1
            self.teammate2_id = 2

        self.pred_n = pred_n

        if old_coord:
            self.old_coord = True

    def updatePosition(self, robot_pos):
        self.robot_x = robot_pos[0]
        self.robot_y = robot_pos[1]
        return True

    def updateHeading(self, heading):
        if heading == -1:
            self.robot_flipped = True
            self.ball_detect = False
            return False
        else:
            self.robot_flipped = False
            if self.team == "B":
                self.robot_heading = (-math.degrees(heading) + 180 + 360) % 360
            else:
                self.robot_heading = (-math.degrees(heading) + 360) % 360
            return True

    def updateBallAngle(self, ball_vector):
        ball_radian = math.atan2(ball_vector[0], ball_vector[1]) + (math.pi / 2)
        if ball_radian < -math.pi:
            ball_radian = ball_radian + (2 * math.pi)
        self.ball_angle = (math.degrees(ball_radian) + 360) % 360
        return True

    def updateBallDistance(self, strength):
        self.ball_distance = math.pow(1/strength, 0.5)
        return True

    def updateBallCoords(self):
        if self.team == "B":
            heading_radian = math.radians((self.robot_heading + self.ball_angle) % 360)
            x_distance = math.sin(heading_radian) * self.ball_distance
            y_distance = math.cos(heading_radian) * self.ball_distance
            ball_x = self.robot_x - x_distance
            ball_y = self.robot_y - y_distance
        else:
            heading_radian = math.radians((self.robot_heading + self.ball_angle) % 360)
            x_distance = math.sin(heading_radian) * self.ball_distance
            y_distance = math.cos(heading_radian) * self.ball_distance
            ball_x = self.robot_x + x_distance
            ball_y = self.robot_y + y_distance

        self.ball_x = ball_x
        self.ball_y = ball_y
        return True

    def updateBallPredCoords(self):
        ball_pos_diff = {'x': self.ball_x - self.pred_last_ball_x,
                         'y': self.ball_y - self.pred_last_ball_y}
        ball_dist = math.sqrt((self.robot_x - self.ball_x) ** 2 + (self.robot_y - self.ball_y) ** 2)
        ball_pos_pred = {'x': self.ball_x + ball_pos_diff['x'] * ball_dist * self.pred_n,
                         'y': self.ball_y + ball_pos_diff['y'] * ball_dist * self.pred_n}
        self.pred_last_ball_x, self.pred_last_ball_y = self.ball_x, self.ball_y
        if ball_pos_pred['x'] > 0.625:
            ball_pos_pred['x'] = 0.625 - abs(0.625 - ball_pos_pred['x'])
        elif ball_pos_pred['x'] < -0.625:
            ball_pos_pred['x'] = -0.625 + abs(-0.625 - ball_pos_pred['x'])
        if ball_pos_pred['y'] > 0.725:
            ball_pos_pred['y'] = 0.725 - abs(0.725 - ball_pos_pred['y'])
        elif ball_pos_pred['y'] < -0.725:
            ball_pos_pred['y'] = -0.725 + abs(-0.725 - ball_pos_pred['y'])
        self.ball_pred_x, self.ball_pred_y = ball_pos_pred['x'], ball_pos_pred['y']
        if self.ball_pred_x < -0.63:
            self.ball_pred_x = -0.63
        elif self.ball_pred_x > 0.63:
            self.ball_pred_x = 0.63
        if self.ball_pred_y < -0.74:
            self.ball_pred_y = -0.74
        elif self.ball_pred_y > 0.74:
            self.ball_pred_y = 0.74
        return True

    def updateBallPredDistance(self):
        self.ball_pred_distance = math.sqrt(math.pow(self.ball_pred_x - self.robot_x, 2) + math.pow(self.ball_pred_y - self.robot_y, 2))
        return True

    def updateBallPredAngle(self):
        if self.team == "B":
            self.ball_pred_angle = (math.degrees(math.atan2(self.ball_pred_x - self.robot_x, self.ball_pred_y - self.robot_y)) - self.robot_heading + 180) % 360
        else:
            self.ball_pred_angle = (math.degrees(math.atan2(self.ball_pred_x - self.robot_x, self.ball_pred_y - self.robot_y)) - self.robot_heading + 360) % 360
        return True

    def updatePredN(self, pred_n):
        self.pred_n = pred_n
        return True

    def updateLastValues(self):
        self.last_robot_x = self.robot_x
        self.last_robot_y = self.robot_y
        self.last_robot_heading = self.robot_heading
        self.last_ball_x = self.ball_x
        self.last_ball_y = self.ball_y
        self.last_ball_distance = self.ball_distance
        self.last_detect = self.ball_detect
        self.last_flipped = self.robot_flipped
        return True

    def receiveData(self, team_data):
        if team_data:
            for packet in team_data:
                # ID Meaning
                # single-digit: robot location (1, 2, 3),
                # double-digit: ball location (10, 20, 30),
                id = packet["id"]
                x = packet["x"]
                y = packet["y"]
                value = packet["value"]
                if id == self.teammate1_id:
                    self.teammate1_x = x
                    self.teammate1_y = y
                    if value == -1:
                        self.teammate1_flipped = True
                    else:
                        self.teammate1_heading = value
                        self.teammate1_flipped = False
                    self.teammate1_ball_distance = math.sqrt(math.pow(self.teammate1_x - self.ball_x, 2) + math.pow(self.teammate1_y - self.ball_y, 2))
                elif id == self.teammate2_id:
                    self.teammate2_x = x
                    self.teammate2_y = y
                    if value == -1:
                        self.teammate2_flipped = True
                    else:
                        self.teammate2_heading = value
                        self.teammate2_flipped = False
                    self.teammate2_ball_distance = math.sqrt(math.pow(self.teammate2_x - self.ball_x, 2) + math.pow(self.teammate2_y - self.ball_y, 2))
                elif id == self.teammate1_id * 10:
                    if value >= 0:
                        self.teammate1_detect = True
                        self.teammate1_ball_x = x
                        self.teammate1_ball_y = y
                        self.teammate1_strength = value
                    else:
                        self.teammate1_detect = False
                elif id == self.teammate2_id * 10:
                    if value >= 0:
                        self.teammate2_detect = True
                        self.teammate2_ball_x = x
                        self.teammate2_ball_y = y
                        self.teammate2_strength = value
                    else:
                        self.teammate2_detect = False
            return True
        return False

    def receiveActive(self):
        if self.teammate1_detect and self.teammate2_detect:
            if self.teammate1_strength > self.teammate2_strength:
                return self.teammate1_id
            else:
                return self.teammate2_id
        elif self.teammate1_detect:
            return self.teammate1_id
        elif self.teammate2_detect:
            return self.teammate2_id
        else:
            return None

    def receiveBallCoords(self):
        active = self.receiveActive()
        if active:
            if active == self.teammate1_id:
                self.ball_x = self.teammate1_ball_x
                self.ball_y = self.teammate1_ball_y
            elif active == self.teammate2_id:
                self.ball_x = self.teammate2_ball_x
                self.ball_y = self.teammate2_ball_y
            else:
                return False
            return True
        else:
            return False

    def receiveBallDistance(self):
        active = self.receiveActive()
        if active:
            if active == self.teammate1_id:
                x = self.teammate1_ball_x
                y = self.teammate1_ball_y
            elif active == self.teammate2_id:
                x = self.teammate2_ball_x
                y = self.teammate2_ball_y
            else:
                return False
            self.ball_distance = math.sqrt(math.pow(x - self.robot_x, 2) + math.pow(y - self.robot_y, 2))
            return True
        else:
            return False

    def receiveBallAngle(self):
        active = self.receiveActive()
        if active:
            if active == self.teammate1_id:
                x = self.teammate1_ball_x
                y = self.teammate1_ball_y
            elif active == self.teammate2_id:
                x = self.teammate2_ball_x
                y = self.teammate2_ball_y
            else:
                return False
            if self.team == "B":
                self.ball_angle = (math.degrees(math.atan2(x - self.robot_x, y - self.robot_y)) - self.robot_heading + 180) % 360
            else:
                self.ball_angle = (math.degrees(math.atan2(x - self.robot_x, y - self.robot_y)) - self.robot_heading + 360) % 360
            return True
        else:
            return False

    def getTeam(self):
        return self.team

    def getRobotID(self):
        return self.robot_id

    def getRobotPos(self):
        if self.old_coord:
            return self.robot_y, self.robot_x
        return self.robot_x, self.robot_y

    def getRobotHeading(self):
        return self.robot_heading

    def getRobotFlipped(self):
        return self.robot_flipped

    def getBallPos(self):
        if self.old_coord:
            return self.ball_y, self.ball_x
        return self.ball_x, self.ball_y

    def getBallAngle(self):
        return self.ball_angle

    def getBallDistance(self):
        return self.ball_distance

    def getBallPredPos(self):
        if self.old_coord:
            return self.ball_pred_y, self.ball_pred_x
        return self.ball_pred_x, self.ball_pred_y

    def getBallPredAngle(self):
        return self.ball_pred_angle

    def getBallPredDistance(self):
        return self.ball_pred_distance

    def getLastRobotPos(self):
        if self.old_coord:
            return self.last_robot_y, self.last_robot_x
        return self.last_robot_x, self.last_robot_y

    def getLastRobotHeading(self):
        return self.last_robot_heading

    def getLastBallPos(self):
        if self.old_coord:
            return self.last_ball_y, self.last_ball_x
        return self.last_ball_x, self.last_ball_y

    def getLastDetect(self):
        return self.last_detect

    def getLastFlipped(self):
        return self.last_flipped

    def getLastBallDistance(self):
        return self.last_ball_distance

    def getBallDetect(self):
        return self.ball_detect

    def getTeammate1ID(self):
        return self.teammate1_id

    def getTeammate1Pos(self):
        if self.old_coord:
            return self.teammate1_y, self.teammate1_x
        return self.teammate1_x, self.teammate1_y

    def getTeammate1Heading(self):
        return self.teammate1_heading

    def getTeammate1Detect(self):
        return self.teammate1_detect

    def getTeammate1Flipped(self):
        return self.teammate1_flipped

    def getTeammate1BallDistance(self):
        return self.teammate1_ball_distance

    def getTeammate2ID(self):
        return self.teammate2_id

    def getTeammate2Pos(self):
        if self.old_coord:
            return self.teammate2_y, self.teammate2_x
        return self.teammate2_x, self.teammate2_y

    def getTeammate2Heading(self):
        return self.teammate2_heading

    def getTeammate2Detect(self):
        return self.teammate2_detect

    def getTeammate2Flipped(self):
        return self.teammate2_flipped

    def getTeammate2BallDistance(self):
        return self.teammate2_ball_distance

    def getTeamDetect(self):
        return self.ball_detect or self.teammate1_detect or self.teammate2_detect

    def getIndividualDetect(self):
        return self.ball_detect, self.teammate1_detect, self.teammate2_detect

    def getActiveDetect(self):
        if self.ball_detect:
            return self.robot_id
        return self.receiveActive()
    
    def getRobotPosSearch(self, id, last=False):
        if id == self.teammate1_id:
            if self.old_coord:
                return self.teammate1_y, self.teammate1_x
            return self.teammate1_x, self.teammate1_y
        elif id == self.teammate2_id:
            if self.old_coord:
                return self.teammate2_y, self.teammate2_x
            return self.teammate2_x, self.teammate2_y
        elif id == self.robot_id:
            if last:
                if self.old_coord:
                    return self.last_robot_y, self.last_robot_x
                return self.last_robot_x, self.last_robot_y
            else:
                if self.old_coord:
                    return self.robot_y, self.robot_x
                return self.robot_x, self.robot_y
        return None

    def getHeadingSearch(self, id, last=False):
        if id == self.teammate1_id:
            return self.teammate1_heading
        elif id == self.teammate2_id:
            return self.teammate2_heading
        elif id == self.robot_id:
            if last:
                return self.last_robot_heading
            else:
                return self.robot_heading
        return None

    def getDetectSearch(self, id, last=False):
        if id == self.teammate1_id:
            return self.teammate1_detect
        elif id == self.teammate2_id:
            return self.teammate2_detect
        elif id == self.robot_id:
            if last:
                return self.last_detect
            else:
                return self.ball_detect
        return None

    def getFlippedSearch(self, id, last=False):
        if id == self.teammate1_id:
            return self.teammate1_flipped
        elif id == self.teammate2_id:
            return self.teammate2_flipped
        elif id == self.robot_id:
            if last:
                return self.last_flipped
            else:
                return self.robot_flipped
        return None
    
    def getBallPosSearch(self, id, last=False):
        if id == self.teammate1_id:
            if self.old_coord:
                return self.teammate1_ball_y, self.teammate1_ball_x
            return self.teammate1_ball_x, self.teammate1_ball_y
        elif id == self.teammate2_id:
            if self.old_coord:
                return self.teammate2_ball_y, self.teammate2_ball_x
            return self.teammate2_x, self.teammate2_y
        elif id == self.robot_id:
            if last:
                if self.old_coord:
                    return self.last_ball_y, self.last_ball_x
                return self.last_ball_x, self.last_ball_y
            else:
                if self.old_coord:
                    return self.ball_y, self.ball_x
                return self.ball_x, self.ball_y
        return None

    def getBallDistanceSearch(self, id, last=False):
        if id == self.teammate1_id:
            return self.teammate1_ball_distance
        elif id == self.teammate2_id:
            return self.teammate2_ball_distance
        elif id == self.robot_id:
            if last:
                return self.last_ball_distance
            else:
                return self.ball_distance
        return None

    def getSearch(self, id, last=False):
        search_robot_x, search_robot_y = self.getRobotPosSearch(id, last)
        search_ball_x, search_ball_y = self.getBallPosSearch(id, last)
        search_heading = self.getHeadingSearch(id, last)
        search_ball_distance = self.getBallDistanceSearch(id, last)
        search_detect = self.getDetectSearch(id, last)
        search_flipped = self.getFlippedSearch(id, last)
        return {"robot_x": search_robot_x,
                "robot_y": search_robot_y,
                "heading": search_heading,
                "ball_x": search_ball_x,
                "ball_y": search_ball_y,
                "ball_distance": search_ball_distance,
                "detect": search_detect,
                "flipped": search_flipped
                }

    def getStandardSearch(self, id, last=False):
        if self.old_coord:
            search_robot_y, search_robot_x = self.getRobotPosSearch(id, last)
            search_ball_y, search_ball_x = self.getBallPosSearch(id, last)
        else:
            search_robot_x, search_robot_y = self.getRobotPosSearch(id, last)
            search_ball_x, search_ball_y = self.getBallPosSearch(id, last)
        search_heading = self.getHeadingSearch(id, last)
        search_ball_distance = self.getBallDistanceSearch(id, last)
        search_detect = self.getDetectSearch(id, last)
        search_flipped = self.getFlippedSearch(id, last)
        return {"robot_x": search_robot_x,
                "robot_y": search_robot_y,
                "heading": search_heading,
                "ball_x": search_ball_x,
                "ball_y": search_ball_y,
                "ball_distance": search_ball_distance,
                "detect": search_detect,
                "flipped": search_flipped
                }

    def getAll(self):
        if self.old_coord:
            return {"team": self.team,
                    "robot_heading": self.robot_heading,
                    "robot_id": self.robot_id,
                    "robot_pos": {"x": self.robot_y, "y": self.robot_x},
                    "robot_flipped": self.robot_flipped,
                    "ball_pos": {"x": self.ball_y, "y": self.ball_x},
                    "ball_angle": self.ball_angle,
                    "ball_distance": self.ball_distance,
                    "ball_pred_pos": {"x": self.ball_pred_y, "y": self.ball_pred_x},
                    "ball_pred_angle": self.ball_pred_angle,
                    "ball_pred_distance": self.ball_pred_distance,
                    "ball_detect": self.ball_detect,
                    "last_robot_pos": {"x": self.last_robot_y, "y": self.last_robot_x},
                    "last_heading": self.last_robot_heading,
                    "last_detect": self.last_detect,
                    "last_flipped": self.last_flipped,
                    "last_ball_pos": {"x": self.last_ball_y, "y": self.last_ball_x},
                    "last_ball_distance": self.last_ball_distance,
                    "teammate1_id": self.teammate1_id,
                    "teammate1_pos": {"x": self.teammate1_y, "y": self.teammate1_x},
                    "teammate1_heading": self.teammate1_heading,
                    "teammate1_detect": self.teammate1_detect,
                    "teammate1_flipped": self.teammate1_flipped,
                    "teammate1_ball_pos": {"x": self.teammate1_ball_y, "y": self.teammate1_ball_x},
                    "teammate1_ball_distance": self.teammate1_ball_distance,
                    "teammate2_id": self.teammate2_id,
                    "teammate2_pos": {"x": self.teammate2_y, "y": self.teammate2_x},
                    "teammate2_heading": self.teammate2_heading,
                    "teammate2_detect": self.teammate2_detect,
                    "teammate2_flipped": self.teammate2_flipped,
                    "teammate2_ball_pos": {"x": self.teammate2_ball_y, "y": self.teammate2_ball_x},
                    "teammate2_ball_distance": self.teammate2_ball_distance,
                    "team_detect": self.getTeamDetect()
                    }
        return {"team": self.team,
                "robot_heading": self.robot_heading,
                "robot_id": self.robot_id,
                "robot_pos": {"x": self.robot_x, "y": self.robot_y},
                "robot_flipped": self.robot_flipped,
                "ball_pos": {"x": self.ball_x, "y": self.ball_y},
                "ball_angle": self.ball_angle,
                "ball_distance": self.ball_distance,
                "ball_pred_pos": {"x": self.ball_pred_x, "y": self.ball_pred_y},
                "ball_pred_angle": self.ball_pred_angle,
                "ball_pred_distance": self.ball_pred_distance,
                "ball_detect": self.ball_detect,
                "last_robot_pos": {"x": self.last_robot_x, "y": self.last_robot_y},
                "last_heading": self.last_robot_heading,
                "last_detect": self.last_detect,
                "last_flipped": self.last_flipped,
                "last_ball_pos": {"x": self.last_ball_x, "y": self.last_ball_y},
                "last_ball_distance": self.last_ball_distance,
                "teammate1_id": self.teammate1_id,
                "teammate1_pos": {"x": self.teammate1_x, "y": self.teammate1_y},
                "teammate1_heading": self.teammate1_heading,
                "teammate1_detect": self.teammate1_detect,
                "teammate1_flipped": self.teammate1_flipped,
                "teammate1_ball_pos": {"x": self.teammate1_ball_x, "y": self.teammate1_ball_y},
                "teammate1_ball_distance": self.teammate1_ball_distance,
                "teammate2_id": self.teammate2_id,
                "teammate2_pos": {"x": self.teammate2_x, "y": self.teammate2_y},
                "teammate2_heading": self.teammate2_heading,
                "teammate2_detect": self.teammate2_detect,
                "teammate2_flipped": self.teammate2_flipped,
                "teammate2_ball_pos": {"x": self.teammate2_ball_x, "y": self.teammate2_ball_y},
                "teammate2_ball_distance": self.teammate2_ball_distance,
                "team_detect": self.getTeamDetect()
                }

    def convert(self, x, y, coord_system):
        # Coordinate System: Old = 0, New = 1
        if (coord_system == 1 and self.old_coord) or (coord_system == 0 and not self.old_coord):
            return y, x
        return x, y

    def process(self, robot_pos, heading, team_data, ball_vector=None, strength=None):
        self.updateLastValues()
        self.updatePosition(robot_pos)
        self.updateHeading(heading)
        receive = self.receiveData(team_data)
        if ball_vector and strength:
            self.updateBallAngle(ball_vector)
            self.updateBallDistance(strength)
            self.updateBallCoords()
            self.updateBallPredCoords()
            self.updateBallPredDistance()
            self.updateBallPredAngle()
            self.ball_detect = True
        else:
            self.ball_detect = False
            if receive and self.receiveActive():
                self.receiveBallCoords()
                self.receiveBallDistance()
                self.receiveBallAngle()
                self.updateBallPredCoords()
                self.updateBallPredDistance()
                self.updateBallPredAngle()
            else:
                return False
        return True