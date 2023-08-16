import math

from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
from data import Data
from offense1_blue import OffenseBlue1
from offense2_blue import OffenseBlue2
from defense_blue import DefenseBlue
from offense1_yellow import OffenseYellow1
from offense2_yellow import OffenseYellow2
from defense_yellow import DefenseYellow
from progress import Progress
from roles import Roles

class MyRobot(RCJSoccerRobot):
    def run(self):
        if "B" in self.name:
            offense1 = OffenseBlue1()
            offense2 = OffenseBlue2()
            defense = DefenseBlue()
        else:
            offense1 = OffenseYellow1()
            offense2 = OffenseYellow2()
            defense = DefenseYellow()
        data = Data(self.name, 80)
        progress1 = Progress(self.name, math.ceil(15 / (TIME_STEP / 1000.0)), math.ceil(10 / (TIME_STEP / 1000.0)), 0.5,
                            math.ceil(10 / (TIME_STEP / 1000.0)), math.ceil(2 / (TIME_STEP / 1000.0)), 0.5 * 0.25)
        progress2 = Progress(self.name, math.ceil(15 / (TIME_STEP / 1000.0)), math.ceil(10 / (TIME_STEP / 1000.0)), 0.5,
                            math.ceil(10 / (TIME_STEP / 1000.0)), math.ceil(2 / (TIME_STEP / 1000.0)), 0.5 * 0.2)
        roles = Roles(self.name)
        last_role = 0
        while self.robot.step(TIME_STEP) != -1:
            try:
                if self.is_new_data():
                    team_data = []
                    while self.is_new_team_data():
                        team_data.append(self.get_new_team_data())

                    if self.is_new_ball_data() and not self.get_robot_flipped():
                        ball_data = self.get_new_ball_data()
                        data.process(self.get_gps_coordinates(), self.get_compass_heading(), team_data, ball_data["direction"], ball_data["strength"])
                        ball_x, ball_y = data.getBallPos()
                        ball_x, ball_y = data.convert(ball_x, ball_y, 1)
                        self.send_data_to_team(self.player_id * 10, ball_x, ball_y, ball_data["strength"])
                    elif self.is_new_ball_data() and self.get_robot_flipped():
                        self.get_new_ball_data()
                        data.process(self.get_gps_coordinates(), -1, team_data)
                        self.send_data_to_team(self.player_id * 10, 0, 0)
                    else:
                        data.process(self.get_gps_coordinates(), self.get_compass_heading(), team_data)
                        self.send_data_to_team(self.player_id * 10, 0, 0)

                    progress1.update(data.getAll())
                    progress2.update(data.getAll())

                    if self.player_id != 3:
                        for packet in team_data:
                            if packet["id"] == -1:
                                roles.receiveRoles(packet["x"], packet["y"], packet["value"])

                    MyRobot.resetProgram(self, last_role, roles, offense1, offense2, defense)
                    last_role = roles.getRoles()[self.player_id]

                    trap_stage = MyRobot.updateTrapStage(self, team_data, roles, offense1, offense2)

                    MyRobot.runProgram(self, self, data, progress1, progress2, roles, offense1, offense2, defense, trap_stage)

                    trap_stage = MyRobot.updateTrapStage(self, team_data, roles, offense1, offense2)

                    if self.player_id == 3:
                        roles.update(data, trap_stage)
                        self.send_data_to_team(-1, roles.getRobotToRole(1), roles.getRobotToRole(2),
                                               roles.getRoleToRobot(3))

                    role = roles.getRoles()[self.player_id]
                    if role == 1:
                        self.send_data_to_team(self.player_id * 100, offense1.trap_stage, 0, 0)
                    elif role == 2:
                        self.send_data_to_team(self.player_id * 100, offense2.trap_stage, 0, 0)

                    robot_x, robot_y = data.getRobotPos()
                    robot_x, robot_y = data.convert(robot_x, robot_y, 1)
                    robot_angle = data.getRobotHeading()

                    if self.get_robot_flipped():
                        self.send_data_to_team(self.player_id, robot_x, robot_y, -1)
                    else:
                        self.send_data_to_team(self.player_id, robot_x, robot_y, robot_angle)
            except:
                pass

    def runProgram(self, robot, data, progress1, progress2, roles, offense1, offense2, defense, trap_stage):
        role = roles.getRoles()[self.player_id]
        if role == 1:
            offense1.run_program(robot, data, progress1.getProgress(), progress2.getProgress(), roles, trap_stage)
        elif role == 2:
            offense2.run_program(robot, data, progress1.getProgress(), progress2.getProgress(), roles, trap_stage)
        else:
            defense.run_program(robot, data, progress1.getProgress(), progress2.getProgress(), roles)

    def resetProgram(self, last_role, roles, offense1, offense2, defense):
        role = roles.getRoles()[self.player_id]
        if role != last_role:
            if role == 1:
                offense1.reset()
            elif role == 2:
                offense2.reset()
            else:
                defense.reset()

    def updateTrapStage(self, team_data, roles, offense1, offense2):
        trap_stage = -1
        for packet in team_data:
            if (packet["id"] == 100 or packet["id"] == 200 or packet["id"] == 300) and packet["x"] != -1:
                return packet['x']
        role = roles.getRoles()[self.player_id]
        if role == 1 and offense1.trap_stage != -1:
            return offense1.trap_stage
        elif role == 2 and offense2.trap_stage != -1:
            return offense2.trap_stage
        return trap_stage


