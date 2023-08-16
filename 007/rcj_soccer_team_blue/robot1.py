# rcj_soccer_player controller - ROBOT B1

# Feel free to import built-in libraries
import math  # noqa: F401

import time

# You can also import scripts that you put into the folder with controller
import utils
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP


class MyRobot1(RCJSoccerRobot):
    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                data = self.get_new_data()  # noqa: F841

                while self.is_new_team_data():
                    team_data = self.get_new_team_data()  # noqa: F841
                    # Do something with team data

                # Get data from compass
                heading = self.get_compass_heading()  # noqa: F841
                heading = int(self.get_compass_heading() / math.pi * 180)  # noqa: F841
                if(heading < 0):
                    heading += 360
                    
                # Get GPS coordinates of the robot
                robot_pos = self.get_gps_coordinates()  # noqa: F841
                robot_pos[0] = round(robot_pos[0] , 5)
                robot_pos[1] = round(robot_pos[1] , 5)

                # Get data from sonars
                sonar_values = self.get_sonar_values()  # noqa: F841
                
                deltax = 0.68 - robot_pos[1]
                if ((0.68 - robot_pos[1]) == 0):
                    deltax = -0.384
                    
                angle = math.atan(robot_pos[0] / deltax) / math.pi * 180
                if(angle <= 0):
                    angle += 360
                    
                heading123 = angle
                headingA = angle + 2
                headingB = angle - 2
                heading1 = angle + 8
                heading2 = angle - 8
                headingC = angle + 10
                headingD = angle - 10
                angleplus = angle + 180
                angleminus = angle - 180
                
                if (robot_pos[1] < 0.5):
                    gap = False
                    rotate = False
                    if (gap == False):
                        #When y coordination < 0.68 (larger than defence area)
                        if (robot_pos[1] <= 0.55): #y= pos[1]
                            #Define defence and moving as False and won't do defence operation
                            defence = False
                            #Heading123 = angle   
                            #If heading <= angle + 2 and heading >= angle - 2
                            if ((heading <= headingA) and (heading >= headingB)):
                                # print("straightback")
                                #Move backward
                                left_speed = -10
                                right_speed = -10                       
                            #If heading > angle + 2 and heading < angle + 8
                            elif ((heading > headingA) and (heading < heading1)):
                                #Left motor is faster
                                # print("LM+")
                                left_speed = -10
                                right_speed = -9.5
                            #If heading < angle + 2 and heading > angle + 8
                            elif ((heading < headingB) and (heading > heading2)):
                                #Right motor is faster
                                # print("RM+")
                                left_speed = -9.5
                                right_speed = -10
                                
                            #If on right side  
                            elif (robot_pos[0] < -0.03 ):
                                #If heading on left but angle > 270   
                                if (heading < angleminus):
                                    # print("tk1")
                                    #Rotate clockrise
                                    left_speed = -10
                                    right_speed = 10
                                #Angle >= 270                
                                elif (heading123 >= 270):
                                    #
                                    if (heading123 < 350):
                                        #If heading > angle + 10
                                        if (heading > headingC):
                                            # print("tk2")
                                            left_speed = -10
                                            right_speed = 10
                                        #If heading <= angle + 10 and heading >= angle + 2
                                        elif (heading <= headingC) and (heading >= headingA):
                                            # print("tk3")
                                            left_speed = -1
                                            right_speed = 1
                                        #If heading < angle - 10
                                        elif (heading < headingD):
                                            # print("tk4")
                                            left_speed = 10
                                            right_speed = -10
                                        #If heading >= angle - 10  and heading <= angle - 2
                                        elif (heading >= headingD) and (heading <= headingB):
                                            # print("tk5")
                                            left_speed = 1
                                            right_speed = -1
                                    
                           #If on left side
                            elif (robot_pos[0] > 0.03 ):
                                if (heading > angleplus):
                                    # print("tk6")
                                    left_speed = 10
                                    right_speed = -10                      
                                elif (heading123 <= 90):
                                    if (heading123 > 10):
                                        #If heading < angle - 10  
                                        if (heading < headingD):
                                            # print("tk7")
                                            left_speed = 10
                                            right_speed = -10
                                        #If heading >= angle - 10  and heading <= angle - 2
                                        elif (heading >= headingD) and (heading <= headingB):
                                            # print("tk8")
                                            left_speed = 1
                                            right_speed = -1
                                        if (heading > headingC):
                                            # print("tk9")
                                            left_speed = -10
                                            right_speed = 10
                                        #If heading <= angle + 10 and heading >= angle + 2
                                        elif (heading <= headingC) and (heading >= headingA):
                                            # print("tk10")
                                            left_speed = -1
                                            right_speed = 1
                                    elif (heading123 <= 10):
                                        if (heading < 10):  
                                            # print("tk11")     
                                            left_speed = -1
                                            right_speed = 1
                                        elif (heading >= 10):
                                            # print("tk12")
                                            left_speed = -10
                                            right_speed = 10
                            #If on middle (buffer zone)
                            else:
                                # print("-10")
                                left_speed = -10
                                right_speed = -10 
                                                          
                            #Move backward and defence (Useless)    
                            #Do compass car before moving backward to defence area                  
                            # if (heading <= 1 or heading >= 359):
                                # left_speed = -10 
                                # right_speed = -10
                            # elif (heading <= 180):
                                # if (heading >= 10):
                                    # left_speed = -5
                                    # right_speed = 5
                                # elif (heading < 10 and heading > 1):
                                    # left_speed = -1
                                    # right_speed = 1
                            # elif (heading > 180):
                                # if (heading <= 350):
                                    # left_speed = 5
                                    # right_speed = -5
                                # elif (heading > 350 and heading < 359):
                                    # left_speed = 1
                                    # right_speed = -1
#_______________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________
                                                                 
                
                elif (robot_pos[1] >= 0.68):
                    rotate = True  
                    if (gap == True and rotate == True):
                        if (self.is_new_ball_data() == True):
                            #After arriving defence, turn clockrise 90 (do compass car)
                            if (heading <= 91 and heading >= 89):    #Left right motor rotate
                                # print("BB1")
                                defence = True
                                left_speed = 0
                                right_speed = 0
                            elif (heading >= 80 and heading < 89):
                                # print("BB2")
                                left_speed = 1
                                right_speed = -1
                            elif (heading < 80 or heading >= 270):
                                # print("enter")
                                left_speed = 10
                                right_speed = -10 
                            elif (heading > 91 and heading <= 100):   
                                # print("BB3")                 
                                left_speed = -1
                                right_speed = 1
                            elif (heading > 100 or heading < 270):
                                # print("BB4")
                                left_speed = -10
                                right_speed = 10
                        elif (self.is_new_ball_data() == False and robot_pos[0] <= 0.08 and robot_pos[0] >= -0.08):
                            if (heading <= 1 or heading >= 359):
                                defence = True
                                left_speed = 8
                                right_speed = 8
                                # print("Python")
                            elif (heading <= 180):
                                if (heading >= 10):
                                    # print("noway1")
                                    left_speed = -10
                                    right_speed = 10
                                elif (heading < 10 and heading > 1):
                                    # print("noway2")
                                    left_speed = -1
                                    right_speed = 1
                            elif (heading > 180):
                                if (heading <= 350):
                                    # print("noway3")
                                    left_speed = 10
                                    right_speed = -10
                                elif (heading > 350 and heading < 359):
                                    # print("noway4")
                                    left_speed = 1
                                    right_speed = -1
                elif (robot_pos[1] >= 0.52 and robot_pos[1] < 0.68 and self.is_new_ball_data() == False and defence == True and (heading <= 10 or heading >= 350)):
                    left_speed = 8
                    right_speed = 8  
                    # print("Python123")
                elif (robot_pos[1] < 0.53):
                    # print("BB6")
                    les = True
                    rotate = True
                    gap = True
                    if (robot_pos[1] <= 0.529 and les == True):
                        # print("-9")
                        left_speed = -8
                        right_speed = -8
                            
                self.left_motor.setVelocity(left_speed)
                self.right_motor.setVelocity(right_speed)
                
                # print("    Compass : " , heading , "    Angle :" , round(angle , 5) , "        RoboPos :" , robot_pos , "    Rotate :" , rotate , "    Gap :" , gap , "    Defence :" , defence)
                
                #If can see ball, keep receiving ALL data
                if self.is_new_ball_data():
                   ball_data = self.get_new_ball_data()
                   #rotateback = False
                else:
                    #If can't see ball, stop receiving data related to ball
                    ## print('    No ball data')
                    if (defence == False):
                        #If defence = False , won't do program below
                        continue
                    else:
                        if (robot_pos[1] <= 0.68 and (robot_pos[0] >= 0.08 or robot_pos[0] <= -0.08)):
                            #If heading <= angle + 2 and heading >= angle - 2
                            if ((heading <= headingA) and (heading >= headingB)):
                                # print("1")
                                #Move backward
                                left_speed = -10
                                right_speed = -10                       
                            #If heading > angle + 2 and heading < angle + 8
                            elif ((heading > headingA) and (heading < heading1)):
                                # print("1.1")
                                #Left motor is faster
                                left_speed = -10
                                right_speed = -9.5
                            #If heading < angle + 2 and heading > angle + 8
                            elif ((heading < headingB) and (heading > heading2)):
                                # print("1.2")
                                #Right motor is faster
                                left_speed = -9.5
                                right_speed = -10
                                
                            #If on right side  
                            elif (robot_pos[0] < -0.03 ):
                                #If heading on left but angle > 270   
                                if (heading < angleminus):
                                    # print("1.3")
                                    #Rotate clockrise
                                    left_speed = -10
                                    right_speed = 10
                                #Angle >= 270                
                                elif (heading123 >= 270):
                                    #
                                    if (heading123 < 350):
                                        #If heading > angle + 10
                                        if (heading > headingC):
                                            # print("001")
                                            left_speed = -10
                                            right_speed = 10
                                        #If heading <= angle + 10 and heading >= angle + 2
                                        elif (heading <= headingC) and (heading >= headingA):
                                            # print("0015")
                                            left_speed = -1
                                            right_speed = 1
                                        #If heading < angle - 10
                                        elif (heading < headingD):
                                            # print("002")
                                            left_speed = 10
                                            right_speed = -10
                                        #If heading >= angle - 10  and heading <= angle - 2
                                        elif (heading >= headingD) and (heading <= headingB):
                                            # print("0025")
                                            left_speed = 1
                                            right_speed = -1
                                    
                           #If on left side
                            elif (robot_pos[0] > 0.03 ):
                                if (heading > angleplus):
                                    # print("003")
                                    left_speed = 10
                                    right_speed = -10                      
                                elif (heading123 <= 90):
                                    if (heading123 > 10):
                                        #If heading < angle - 10  
                                        if (heading < headingD):
                                            # print("004")
                                            left_speed = 10
                                            right_speed = -10
                                        #If heading >= angle - 10  and heading <= angle - 2
                                        elif (heading >= headingD) and (heading <= headingB):
                                            # print("005")
                                            left_speed = 1
                                            right_speed = -1
                                        if (heading > headingC):
                                            # print("006")
                                            left_speed = -10
                                            right_speed = 10
                                        #If heading <= angle + 10 and heading >= angle + 2
                                        elif (heading <= headingC) and (heading >= headingA):
                                            # print("007")
                                            left_speed = -1
                                            right_speed = 1
                                    elif (heading123 <= 10):
                                        if (heading < 10):  
                                            # print("008")     
                                            left_speed = -1
                                            right_speed = 1
                                        elif (heading >= 10):
                                            # print("009")
                                            left_speed = -10
                                            right_speed = 10
                                
                        elif (robot_pos[1] <= 0.683 and robot_pos[0] <= 0.08 and robot_pos[0] >= -0.08 and rotate == True and defence == False):
                            # print("this?")
                            left_speed = -10
                            right_speed = -10
                            
                        # elif (robot_pos[1] >= 0.52 and robot_pos[0] <= 0.08 and robot_pos[0] >= -0.08 and rotate == True and defence == True):     
                             # if (heading <= 1 or heading >= 359):
                                 # defence = True
                                 # left_speed = 8
                                 # right_speed = 8
                                 # # print("Python")
                             # elif (heading <= 180):
                                 # # print("rein")
                                 # if (heading >= 10):
                                     # left_speed = -5
                                     # right_speed = 5
                                 # elif (heading < 10 and heading > 1):
                                     # # print("genji")
                                     # left_speed = -1
                                     # right_speed = 1
                             # elif (heading > 180):
                                 # if (heading <= 350):
                                     # # print("mercy")
                                     # left_speed = 5
                                     # right_speed = -5
                                 # elif (heading > 350 and heading < 359):
                                     # # print("moira")
                                     # left_speed = 1
                                     # right_speed = -1
                        elif (robot_pos[1] < 0.52 and defence == True):
                            gap = True
                            # print("-8")
                            left_speed = -8
                            right_speed = -8
                             
                        #Inside dragon door
                        elif (robot_pos[1] >= 0.693):
                            if (heading <= 1 or heading >= 359):
                                # print("doom")
                                left_speed = 8
                                right_speed = 8
                            elif (heading <= 180):
                                if (heading >= 10):
                                    # print("zyira")
                                    left_speed = -10
                                    right_speed = 10
                                elif (heading < 10 and heading > 1):
                                    # print("ball")
                                    left_speed = -1
                                    right_speed = 1
                            elif (heading > 180):
                                if (heading <= 350):
                                    # print("sigma")
                                    left_speed = 10
                                    right_speed = -10
                                elif (heading > 350 and heading < 359):
                                    # print("orisa")
                                    left_speed = 1
                                    right_speed = -1 
                            
                        else:
                            if (heading >= 85 and heading <= 95):
                                if (robot_pos[0] >= 0.08):
                                    # print("ramattra")
                                    left_speed = -8
                                    right_speed = -8
                                elif (robot_pos[0] <= -0.08):
                                    # print("junker")
                                    left_speed = 8
                                    right_speed = 8   
                            elif (heading >= 265 and heading <= 275):
                                if (robot_pos[0] >= 0.08):
                                    # print("winston")
                                    left_speed = 8
                                    right_speed = 8
                                elif (robot_pos[0] <= -0.08):
                                    # print("dvas")
                                    left_speed = -8
                                    right_speed = -8   
                                    
                        self.left_motor.setVelocity(left_speed)
                        self.right_motor.setVelocity(right_speed)
                        continue
                                
                        
                
                # ball info
                strength = ball_data["strength"] # s = 1 / r2
                distance = math.sqrt(1 / strength)
                # int: 0 = forward, -1 = right, 1 = left
                direction = utils.get_direction(ball_data["direction"])
                
                direction_xy = ball_data["direction"]
                direction_x = direction_xy[0]
                direction_y = -direction_xy[1]                
                ball_direction_rad = math.atan2(direction_y, direction_x)
                
                heading_ball = int(self.get_compass_heading() / math.pi * 180)  # noqa: F841
                heading_ball -= 360
                if(heading_ball < 0):
                    heading_ball = -heading_ball
                elif heading_ball >= 360:
                    heading_ball -= 360
                    
                ball_direction = ball_direction_rad /math.pi  * 180
                if (ball_direction < 0):
                    ball_direction += 360
                ball_abs_direction = ball_direction + heading_ball
                if ball_abs_direction >= 360:
                    ball_abs_direction -= 360
                ball_abs_direction_rad = ball_abs_direction * math.pi / 180
                ball_abs_x, ball_abs_y = robot_pos[0] - distance * math.sin(ball_abs_direction_rad), robot_pos[1] - distance * math.cos(ball_abs_direction_rad)
                              
                #ball_abs_x, ball_abs_y = robot_pos[0] - distance * math.cos(ball_direction_rad), robot_pos[1] - distance * math.sin(ball_direction_rad)
                
                if (defence == False):
                    if (strength >= 2 and robot_pos[1] < 0.68 and rotate == True):
                        # print("backspace")
                        left_speed = -9
                        right_speed = -9
                
                #If defence area                
                elif (defence == True):
                    #If can see ball within half of field
                    if (strength >= 2 and robot_pos[1] >= 0.68):
                        if (heading <= 91 and heading >= 89):
                            # print("1")
                            if (robot_pos[0] >= -0.4 and robot_pos[0] <= 0.4): 
                            #defence ball
                                if (ball_direction >= 180):
                                    #In Q4
                                    if (ball_direction <= 270):
                                        left_speed = -9
                                        right_speed = -9
                                        # print("degg1")
                                    #In Q1
                                    elif (ball_direction > 270):
                                        left_speed = 9
                                        right_speed = 9
                                        # print("degg2")
                                elif (ball_direction < 180):
                                    #In Q3
                                    if (ball_direction >= 90):
                                        left_speed = -(9 + math.sin(ball_direction + 45))
                                        right_speed = -(9 + math.cos(ball_direction + 45))
                                        # print("degg3")
                                        #In Q2
                                    elif (ball_direction < 90):
                                        left_speed = (9 + math.cos(ball_direction + 45))
                                        right_speed = (9 + math.sin(ball_direction + 45))
                                        # print("degg4")
                            elif (robot_pos[0] > 0.4):
                                left_speed = -10
                                right_speed = -10
                                # print("right")
                            elif (robot_pos[0] < -0.4):
                                left_speed = 10
                                right_speed = 10
                                # print("left")
                                
                                
                        elif (heading >= 80 and heading < 89 and robot_pos[0] >= 0.08 and robot_pos[0] <= -0.08):
                            # print("turn1")
                            left_speed = 1
                            right_speed = -1
                        elif (heading < 80 or heading >= 270 and robot_pos[0] >= 0.08 and robot_pos[0] <= -0.08):
                            # print("turn2")
                            left_speed = 10
                            right_speed = -10 
                        elif (heading > 91 and heading <= 100 and robot_pos[0] >= 0.08 and robot_pos[0] <= -0.08):  
                            # print("turn3")                  
                            left_speed = -1
                            right_speed = 1
                        elif (heading > 100 or heading < 270 and robot_pos[0] >= 0.08 and robot_pos[0] <= -0.08):
                            # print("turn4")
                            left_speed = -10
                            right_speed = 10
                            
                    elif (strength >= 2 and robot_pos[1] < 0.65 and robot_pos[0] >= -0.4 and robot_pos[0] <= 0.4): 
                        # if (ball_direction >= 180):
                            # #In Q4
                            # if (ball_direction <= 270):
                                
                            # #In Q1
                            # elif (ball_direction > 270):
                                
                        # elif (ball_direction < 180):
                            #In Q3
                            if (heading >= 80 and heading <= 100):
                                if (ball_direction >= 90):
                                    # print("goingback-")
                                    left_speed = -(8.5 + 1.5 * math.sin(ball_direction + 45))
                                    right_speed = -(8.5 + 1.5 * math.cos(ball_direction + 45))
                                    #In Q2
                                elif (ball_direction < 90):
                                    # print("goingback+")                  
                                    left_speed = (8.5 + 1.5 * math.sin(ball_direction + 45))
                                    right_speed = (8.5 + 1.5 * math.cos(ball_direction + 45))
                            # elif (heading <= 10 or heading >= 350):
                                # # print("gone")
                                # left_speed = -8
                                # right_speed = -8
                                    
                    elif (strength >= 2 and robot_pos[1] >= 0.50 and robot_pos[1] < 0.68 and robot_pos[0] >= -0.4 and robot_pos[0] <= 0.4):
                        # up = True
                        # if (robot_pos[0] >= 0.08):
                            # # print("dde1")
                            # left_speed = -6
                            # right_speed = -8
                            # if (robot_pos[1] >= 0.55):
                                # # print("dde2")
                                # down = True
                                # left_speed = -8
                                # right_speed = -6
                        # elif (robot_pos[0] <= 0.08):
                            # # print("dde3")
                            # left_speed = 6
                            # right_speed = 8
                            # if (robot_pos[1] >= 0.55):
                                # # print("dde4")
                                # down = True
                                # left_speed = 8
                                # right_speed = 6
                        
                        if (robot_pos[1] >= 0.60):
                            if (heading >= 85 and heading <= 95):
                                if (ball_direction >= 180):
                                    #In Q4
                                    if (ball_direction <= 270):
                                        left_speed = -10
                                        right_speed = -10
                                        # print("defence1")
                                    #In Q1
                                    elif (ball_direction > 270):
                                        left_speed = 10
                                        right_speed = 10
                                        # print("defence2")
                                elif (ball_direction < 180):
                                    #In Q3
                                    if (ball_direction >= 90):
                                        left_speed = -9        
                                        right_speed = -9
                                        # print("defence3")
                                        #In Q2
                                    elif (ball_direction < 90):
                                        left_speed = 9      
                                        right_speed = 9
                                        # print("defence4") 
                            
                            elif (heading <= 85 and heading >= 95):
                                if (ball_direction >= 180):
                                    #In Q4
                                    if (ball_direction <= 270):
                                        left_speed = -10
                                        right_speed = -10
                                        # print("defence1.1")
                                    #In Q1
                                    elif (ball_direction > 270):
                                        left_speed = 10
                                        right_speed = 10
                                        # print("defence2.1")
                                elif (ball_direction < 180):
                                    #In Q3
                                    if (ball_direction >= 90):
                                        left_speed = - (9 + math.cos(ball_direction + 45))        
                                        right_speed = - (9 + math.sin(ball_direction + 45))
                                        # print("defence3.1")
                                        #In Q2
                                    elif (ball_direction < 90):
                                        left_speed = (9 + math.cos(ball_direction + 45))        
                                        right_speed = (9 + math.sin(ball_direction + 45))
                                        # print("defence4.1")   
                                       
                    elif (strength >= 2 and robot_pos[0] < -0.4 or robot_pos[0] > 0.4):
                        if (robot_pos[1] > 0.65 and robot_pos[1] < 0.68 and robot_pos[0] > 0.4):
                            # print("777")
                            left_speed = -9
                            right_speed = -9.5
                        elif (robot_pos[1] > 0.65 and robot_pos[1] < 0.68 and robot_pos[0] < -0.4):
                            # print("7771")
                            left_speed = 9
                            right_speed = 9.5
                        elif (robot_pos[1] <= 0.65 and robot_pos[0] > 0.4):
                            # print("7772")
                            left_speed = -9
                            right_speed = -10
                        elif (robot_pos[1] <= 0.65 and robot_pos[0] < -0.4):
                            # print("7773")
                            left_speed = 9
                            right_speed = 10
                            
                            
                            
                    elif (strength < 2 and robot_pos[1] < 0.71):
                        left_speed = -5
                        right_speed = -5
                        # print("nothing")
                    
                    # else:
                        # if (heading <= 91 and heading >= 89):    #Left right motor rotate
                        # print("break1")
                        # left_speed = 0
                        # right_speed = 0
                        # elif (heading >= 80 and heading < 89):
                            # # print("break2")
                            # left_speed = 1
                            # right_speed = -1
                        # elif (heading < 80 or heading >= 270):
                            # # print("break3")
                            # left_speed = 10
                            # right_speed = -10 
                        # elif (heading > 91 and heading <= 100):   
                            # # print("break4")                 
                            # left_speed = -1
                            # right_speed = 1
                        # elif (heading > 100 or heading < 270):
                            # # print("break5")
                            # left_speed = -10
                            # right_speed = 10
                               
                # Set the speed to motors
                self.left_motor.setVelocity(left_speed)
                self.right_motor.setVelocity(right_speed)
                
                # print("    BallStren :" , round(ball_data["strength"] , 5) , "     Dir :" , round(ball_direction , 5) , "    Radian :" , round(ball_direction_rad , 5) , "    X of ball :" , ball_abs_x , "    Y of ball :" , ball_abs_y)
                # who = 1
                # d1 = ball_abs_x
                # d2 = ball_abs_y
                # message_format = "iff"
                # message = [who,d1,d2]
                # x & y delay for about 1 period (nearly correct)
                # # print(robot_x, robot_y)
                    
                # self.send_data_to_team(message_format, message)
                
                
                # Send message to team robots
                #self.send_data_to_team(self.player_id)
                
                