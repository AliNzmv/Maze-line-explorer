"""my_controller controller."""
"""This is  project of Control Digital Systems Course at Amirkabir University of Tehran
In this project we managed to plant a controller for a e-puck robot to search maze and discover
the lines and also relations of the map using a tree graph algorithm"""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import sys
import os
import threading
import time
import numpy as np
#=======================================================Define Constants Variables================================================
# create the Robot instance.
robot = Robot()
current_direction = 0
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
MAX_SPEED = 6.28



line_nodes = list()

# setting motors starting characteristic
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')


leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

leftMotor.setVelocity(0)
rightMotor.setVelocity(0)
# set up the distance and ground sensors
ps = []
gs = []
psNames = ["ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"]
gsNames = ['gs0', 'gs1', 'gs2']
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timestep)
    
for i in range(3):
    gs.append(robot.getDevice(gsNames[i]))
    gs[i].enable(timestep)

# set up the encoder sensors    
encoder = []
encoderNames = ['left wheel sensor', 'right wheel sensor']
for i in range(2):
    encoder.append(robot.getDevice(encoderNames[i]))
    encoder[i].enable(timestep)



#time for going near all = 0.35* one_block_ms
sensor_detect_delay_rate = 0.35

#define english alphabets as name for each node
alphabet = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm',
            'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z',
            'ab', 'ac', 'ad', 'ae', 'af', 'ag', 'ah', 'ai', 'aj', 'ak',
            'al', 'am', 'an', 'ao', 'ap', 'aq', 'ar', 'as', 'at', 'au',
            'av', 'aw', 'ax', 'ay', 'az', 'ba', 'bc', 'bd', 'be', 'bf',
            'bg', 'bh', 'bi', 'bj', 'bk', 'bl', 'bm', 'bn', 'bo', 'bp',
            'bq', 'br', 'bs', 'bt', 'bu', 'bv', 'bw', 'bx', 'by', 'bz',
            'cab', 'cac', 'cad', 'cae', 'caf', 'cag', 'cah', 'cai', 'caj',
            'cak', 'cal', 'cam', 'can', 'cao', 'cap', 'caq', 'car', 'cas',
            'cat', 'cau', 'cav', 'caw', 'cax', 'cay', 'caz', 'cba', 'cbc',
            'cbd', 'cbe', 'cbf', 'cbg', 'cbh', 'cbi', 'cbj', 'cbk', 'cbl',
            'cbm', 'cbn', 'cbo', 'cbp', 'cbq', 'cbr', 'cbs', 'cbt', 'cbu',
            'cbv', 'cbw', 'cbx', 'cby', 'cbz']
alphabet_counter = 0
#Save the coordinates which robot has been there
coordinat_list = list()

num_of_line = 0
#x max and min we have explored
x_max = 0
x_min = 0
#In which activity the line has been seen
situation = None
#Check if the line is new
new_line = True
#--Flags
#this flag gets True when ever the line detector function detect line
# only the line detector function has the ability to write on this flag and the controller only can read
line_flag = False
#this flag gets False when ever the controller exit line following, we made this flag to avoiding resourse hazards while two synchron functions are writing on the same flag
# only the controller function has the ability to write on this flag and the line detector only can read
line_flag_write = True

#this flag gets True when all lines have followed
finish_flag = False
#define a function for time delay which depend on robot timestep
def delay(ms):
    initTime = robot.getTime()      # Store starting time (in seconds)
    while robot.step(timestep) != -1:
        if (robot.getTime() - initTime) * 1000.0 > ms: # If time elapsed (converted into ms) is greater than value passed in
            break

#================================================Tree Node Class===================================================
#define a class for our graph
class TreeNode :
    def __init__(self, data, coordinate):
        self.data = data
        self.blocked = 0
        self.coordinate = coordinate
        self.children = list()
        self.wall = list()
        self.parents = None

    #Add child for each node
    def add_child(self, child):
        child.parents = self
        self.children.append(child)
    
    #Function for printing the form of our tree
    def show_tree(self) :
        spaces = "  " * self.get_level() * 3
        prefix = spaces + "|__" if self.parents else ""
        print(f"{prefix} {self.data}: {self.coordinate}")
        if self.children :
            for child in self.children :
                child.show_tree()
    # Get level of Node
    def get_level(self) :
        level = 0
        p = self.parents
        while p :
            level += 1
            p = p.parents
        return level


#======================================================PID class====================================================
#defining PID class
class PID_Controller():
    def __init__(self, kp=1.15, ki=0.001, kd=0.15):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.error = 0
        self.pre_error = 0
        self.integrate = 0
        self.center = 0
   #function to calcute the control signal based on the error input     
    def calculate(self, val):
        self.error = self.center - val
        self.integrate += self.error
        d = self.error - self.pre_error
        
        res = (self.kp * self.error) + (self.ki * self.integrate) + (self.kd * d)
        self.pre_error = self.error
        
        return res
    
#defining pid as an object of PID_Controller class
pid = PID_Controller()


#function to read encoder sensor values
def read_encoder():
    encoderValues = []
    for i in range(2):
        encoderValues.append(encoder[i].getValue())
    return encoderValues
#======================================================Movement Func====================================================
# A function used for caliberating our robot in maze
def calibr(error,direction):
    if direction == 'f':
        psValues = read_sensors()
        while abs(psValues[7] - psValues[0]) > error:   
            psValues = read_sensors()
            k = (psValues[7] - psValues[0])/1000
            leftMotor.setVelocity(-k * MAX_SPEED)
            rightMotor.setVelocity(k * MAX_SPEED)
            delay(1)
    elif direction == 'b':
        psValues = read_sensors()
        while abs(psValues[3] - psValues[4]) > error:   
            psValues = read_sensors()
            k = (psValues[3] - psValues[4])/1000
            leftMotor.setVelocity(-k * MAX_SPEED)
            rightMotor.setVelocity(k * MAX_SPEED)
            delay(1)
    return print("calibr done")

#definig PID Coefficients for our Rotation task
Rotation = PID_Controller()
Rotation.kp = 1.5
Rotation.ki = 0.04
Rotation.kd = 0.3

#definig PID Coefficients for our Movement task
Forward = PID_Controller()
Forward.kp = 2
Forward.ki = 0.01
Forward.kd = 0.2

#Function for our movement tasks. The inputs are the length of blocks to move, the degrees needed to shift, and the direction of rotation.
def move_func(block_rate = 0 ,rotation = 0 ,direction = 'clk'):

    encoderValues = read_encoder()

    temp_l = encoderValues[0]
    temp_r = encoderValues[1]
    
    
    #change degree to radian
    rad = (rotation * np.pi/180)*(58.05/41)
    
    
    
    if rotation != 0:
        #clockwise rotation
        if direction == 'clk':
    
            while abs(rad - (encoderValues[0] - temp_l)) > 0.004 :
                start = time.time()
                k = -rad + (encoderValues[0] - temp_l)
                   #error given to PID to calculate the appropiate control signal
                V = Rotation.calculate(k)
                leftMotor.setVelocity(V)
                rightMotor.setVelocity(-V)
    
                end = time.time()
    
                dt = end - start
                delay(1-dt)
                encoderValues = read_encoder()
    
            leftMotor.setVelocity(0*MAX_SPEED)
            rightMotor.setVelocity(0*MAX_SPEED)
            Rotation.integrate = 0
    
        else:
            #counter clockwise rotation
            while abs(rad-(encoderValues[1] - temp_r) ) > 0.004 :
                start = time.time()
                k = -(rad - (encoderValues[1] - temp_r)) 
                   #error given to PID to calculate the appropiate control signal
                V = Rotation.calculate(k)
                leftMotor.setVelocity(-V)
                rightMotor.setVelocity(V)
    
                end = time.time()
    
                dt = end - start
                delay(1-dt)
                encoderValues = read_encoder()
    
            leftMotor.setVelocity(0*MAX_SPEED)
            rightMotor.setVelocity(0*MAX_SPEED)
            Rotation.integrate = 0


    encoderValues = read_encoder()

    temp_l1 = encoderValues[0]
   # The ratio of block length to the wheels' radius
    ratio = 0.25593/0.0205
    
    # movement ( forward and backward)
    while abs(ratio*block_rate- (encoderValues[0] - temp_l1)) > 0.01:
        start = time.time()
        
        k = (- (ratio*block_rate - (encoderValues[0] - temp_l1)))/10
        #error given to PID to calculate the appropiate control signal
        V = Forward.calculate(k)
        leftMotor.setVelocity(V)
        rightMotor.setVelocity(V)

        end = time.time()

        dt = end - start
        delay(1-dt)
        encoderValues = read_encoder()

    leftMotor.setVelocity(0*MAX_SPEED)
    rightMotor.setVelocity(0*MAX_SPEED)
    #At the end of the block , we set the summation of errors to 0 
    Forward.integrate = 0
#======================================================Movement Func====================================================
#====================================================get ground sensors value===============================================
# Function to read ground sensor values
def get_gs_values():
        global gs
        gsValues = []
        for i in range(3):
            gsValues.append(gs[i].getValue())
        return gsValues

#Main function for line following process
def line_follower():
    global line_flag
    max_speed = 6.28 * 0.45
    time_step = 32

    gs = []
    gsNames = ['gs0', 'gs1', 'gs2']
    for i in range(3):
        gs.append(robot.getDevice(gsNames[i]))
        gs[i].enable(time_step)


    leftMotor.setVelocity(0.0)
    rightMotor.setVelocity(0.0)
# Delay so the motor and sensors would set up correctly
    robot.step(50)

    #Function for reading ground sensors' value
    def get_values():
        gsValues = get_gs_values()
        if(gsValues[0] > 750 and gsValues[1] > 750 and gsValues[2] > 750):
            return None # line is finished or lost
        return (gsValues[0] - gsValues[2]) / 200
    #set up motor speed
    def set_motors(l, r):
        leftMotor.setVelocity(l)
        rightMotor.setVelocity(r)
    
    # Main function for line following behavior 
    def follower():
        global line_flag
        while True:
            diff = get_values()
            if diff is None: #The sensors; input define there is no line
                line_flag = False
                break
            correction = pid.calculate(diff)
            
            l = max_speed - correction
            r = max_speed + correction
            
            set_motors(l, r)
            robot.step(time_step)
        
        set_motors(0.0, 0.0)
    follower()
    
#=========================================================Required Functions=======================================
    
#line detector
#change the flag to True if it's detect line
#We used a falg for sending signal to controller and another flag to read signal from controller
def line_detector():
    global line_flag, line_flag_write, finish_flag
    
    while not finish_flag:
        gsValues = get_gs_values()
        # The length of the interval for which the black line is defined
        if((310<gsValues[0] < 400) or (310<gsValues[1] < 400)  or (310<gsValues[2] < 400)):
                line_flag = True 
                continue
        #check for reading the new flag value from controller
        if line_flag_write != True:
            line_flag = line_flag_write
            line_flag_write = True

#========================================================make new node========================================

#Function to create a new tree node, add it as a child to the parent node, and update the coordinate list
def make_node(name, coordinate, parent) :
    global coordinat_list
    new_node = TreeNode(name, coordinate)
    parent.add_child(new_node)
    coordinat_list.append(new_node.coordinate)
    print(f"Go to:{new_node.coordinate}")
    return new_node



#======================================================driver and motor management================================================

#back_to_nul = should robot turn to the null direction?
def driver_controller(direction, block, back_to_null = True):
    global current_direction
    if direction == 'l':
        #move to left direction
        current_direction -= 90
        move_func(1*block,90,"cclk")
        if back_to_null:
             #set the head of the robot to the North
            current_direction += 90
            move_func(0,90)

    elif direction == 'r':
        #move to right direction
        current_direction += 90
        move_func(1*block,90)
        if back_to_null:
            #set the head of the robot to the North
            current_direction -= 90
            move_func(0,90,"cclk")

    elif direction == 'f':
        #move to forward direction
        move_func(1*block)
    elif direction == 'b':
        #move to forward direction
        move_func(-1*block)
    elif direction == 'cwt':
        #Clockwise Rotaion
        current_direction += 90
        move_func(0,90)
    elif direction == 'ucwt':
        #Counter clockwise Rotaion
        current_direction -= 90
        move_func(0,90,"cclk")

#======================================================movement management between blocks====================================
def movement_control(stay_on_node, going_to, is_new):
    global alphabet_counter, x_max, x_min
    global alphabet

    #Check if the destination shoul known as a new node
    if is_new :
        # Get the current coordinates from the node we are staying on
        from_coor = stay_on_node.coordinate
        # Get the current coordinates from the node we are staying on
        alphabet_counter += 1

        if going_to == 'l':
            new_coordinate = [from_coor[0], from_coor[1] - 1] # Moving left
        elif going_to == 'f':
            new_coordinate = [from_coor[0] + 1, from_coor[1]] # Moving forward
        elif going_to == 'r':
            new_coordinate = [from_coor[0], from_coor[1] + 1] # Moving right
        elif going_to == 'b':
            new_coordinate = [from_coor[0]-1, from_coor[1]] # Moving backward

        # Update the maximum and minimum y-coordinate boundaries
        if new_coordinate[1]> x_max:
            x_max = new_coordinate[1]
        if new_coordinate[1]< x_min:
            x_min = new_coordinate[1]

        # Create a new node with the calculated coordinates and the current node as its parent
        new_node = make_node(alphabet[alphabet_counter], new_coordinate, stay_on_node)

        # move in the correct direction with sensor delay adjustment then return the new node
        if going_to =='b':
            driver_controller('b', 1-sensor_detect_delay_rate)
        else:
            driver_controller('f', 1-sensor_detect_delay_rate)
        return new_node
    else:
        driver_controller(going_to, 1)
        return stay_on_node.parents
#================================================path finder=================================================
#Specifies the direction of the destination relative to the origin
def path_finder(fl, sl):
    if (fl[0]-sl[0]) >0:
        return 'b'
    elif (fl[0] - sl[0]) <0:
        return 'f'
    elif (fl[1] - sl[1]) <0:
        return 'r'
    elif (fl[1] - sl[1]) >0:
        return 'l'
    
#==================================================read distance sensors=================================================
def read_sensors():
    global ps
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())
    return psValues
#====================================================return to home================================================
def back_to_home(node):
    global finish_flag, root, line_nodes

     # Loop until the node has no parent or it is the root node
    while node.parents:
        # Find the direction to move from the current node to its parent
        direction = path_finder(node.coordinate, node.parents.coordinate)
        # Move to the parent node without creating a new node
        node = movement_control(node, direction, False)
    finish_flag = True
    show_tree()
    print("=====================")
    print("Line coordinats: ")
    for line_node in line_nodes:
        print(line_node.coordinate)
    

    
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
    #give time to motors to make velocity 0
    delay(50)
    #terminate the running
    os._exit(0)
#=================================================controller=======================================================
#Main Module
#after each move robot check if there is a wall or not and then decide the next move
def controller(base_node):
    global coordinat_list, current_direction, line_flag_write, line_nodes, new_line , situation, x_max, x_min

    #exit the while if you done all tasks in the current node
    finish_node = False
    current_node = base_node
    current_coordinate = current_node.coordinate

    #check if the all directions have checked
    check_forward = True
    check_backward = True
    check_right = True
    check_left = True

    #where is the line when ever robot exit the line following 
    line_direction = 'f'

    #is y-axis of robot calibrated
    is_y_calibrated = False
    print("Line: ",line_flag)
    #if all the lines have been seen and the overall task is finished
    if len(line_nodes) == 4:
        back_to_home(current_node)
        os._exit(0)
    
    # Main loop to control the robot's movement
    while (not line_flag) and (not finish_node):
        print("movement")

        # If the current y-coordinate matches the last node's y-coordinate when we are looking for another head of line
        try:
            #if the line number is odd
            if len(line_nodes)%2 == 1:
                check_distance = 0
                # If the current y-coordinate matches the last node's y-coordinate
                if current_coordinate[0] == line_nodes[-1].coordinate[0]:
                    print("trigger point")
                    #check back
                    while (robot.step(timestep) != -1) and (check_distance<0.75):
                        driver_controller('b', 0.15, False)
                        check_distance += 0.15
                        gs_values = get_gs_values()
                         # Check for obstacles using ground sensors
                        if(gs_values[0] < 400 or gs_values[1] <400  or gs_values[2] < 400):
                            situation = 'ob'
                            break
                    if check_distance == 0.75:  # If the check distance is 0.75 of block, move forward
                        driver_controller('f', 0.75, False)
                    else:
                        break
                    #check right
                    check_distance = 0
                    while (robot.step(timestep) != -1) and (check_distance<0.75):
                        driver_controller('cwt', 0)
                        driver_controller('f', 0.15, False)
                        check_distance += 0.15
                        gs_values = get_gs_values()
                         # Check for obstacles using ground sensors
                        if(gs_values[0] < 400 or gs_values[1] <400  or gs_values[2] < 400):
                            situation = 'of'
                            break
                    if check_distance == 0.75:  # If the check distance is 0.75 of block, move backward
                        driver_controller('ucwt', 0)
                        driver_controller('b', 0.75, False)
                    else:
                        break
                    #check left
                    check_distance = 0
                    while (robot.step(timestep) != -1) and (check_distance<0.75):
                        driver_controller('ucwt', 0)
                        driver_controller('f', 0.15, False)
                        check_distance += 0.15
                        gs_values = get_gs_values()
                        # Check for obstacles using ground sensors
                        if(gs_values[0] < 400 or gs_values[1] <400  or gs_values[2] < 400):
                            situation = 'of'
                            break
                    if check_distance == 0.75: # If the check distance is 0.75 of block, move backward
                        driver_controller('cwt', 0)
                        driver_controller('b', 0.75, False)
                    else:
                        break
                    #check forward
                    check_distance = 0
                    while (robot.step(timestep) != -1) and (check_distance<0.75):
                        driver_controller('f', 0.15, False)
                        check_distance += 0.15
                        gs_values = get_gs_values()
                        # Check for obstacles using ground sensors
                        if(gs_values[0] < 400 or gs_values[1] <400  or gs_values[2] < 400):
                            situation = 'fb'
                            break
                    if check_distance == 0.75:  # If the check distance is 0.75 of block, move backward
                        driver_controller('b', 0.75, False)
                    else:
                        break
        except:
            pass
        #Change the priorities of direction exploring consider the current coordinate and the coordinate which the line has been seen in it by y axis
        try:
            #Check if we are looking for another head of line
            if len(line_nodes)%2 == 1:
                #If we are in the back of the other head first check forward
                if (current_coordinate[0] < line_nodes[-1].coordinate[0]):
                    check_forward = False
                    #Forward direction   
                    if ([current_coordinate[0] + 1, current_coordinate[1]] not in coordinat_list):
                        driver_controller('f', sensor_detect_delay_rate)
                        if line_flag:
                            situation = 'cf'
                            break
                        psValues = read_sensors()
                    
                        # Check if there is no obstacle in the forward direction
                        if (psValues[0] < 90) or (psValues[7] < 90) :
                            current_node = movement_control(current_node, 'f', True)
                            if line_flag:
                                situation = 'of'
                            current_node = controller(current_node)
                            raise Exception()
                            

                        #Calibration robot to center
                        elif (psValues[0] < 150) or (psValues[7] < 150):
                            while((psValues[0] < 150) or (psValues[7] < 150)):
                                psValues = read_sensors()
                                
                                move_func(0.01)
                            calibr(3,"f")
                            driver_controller('b',sensor_detect_delay_rate)
                        else:
                            driver_controller('b', sensor_detect_delay_rate)
                            current_node.wall.append('f')  
                # #If we are in the front of the other head first check backward
                elif (current_coordinate[0] > line_nodes[-1].coordinate[0]):
                    check_backward = False
                    #Back direction
                    if ([current_coordinate[0] - 1, current_coordinate[1]] not in coordinat_list):
                        driver_controller('b', sensor_detect_delay_rate)
                        if line_flag:
                            situation = 'cb'
                            break
                        psValues = read_sensors()
                        # Check if there is no obstacle in the backward direction
                        if (psValues[3] < 90) or (psValues[4] < 90) :
                            current_node = movement_control(current_node, 'b', True)
                            if line_flag:
                                situation = 'ob'
                            current_node = controller(current_node)
                            raise Exception()
                        
                        #Calibration robot to center
                        elif (psValues[3] < 150) or (psValues[4] < 150):
                            while((psValues[3] < 150) or (psValues[4] < 150)):
                                psValues = read_sensors()
                                
                                #driver(-0.1, -0.1, 1)
                                move_func(-0.01)
                            calibr(3,"b")
                            driver_controller('f', sensor_detect_delay_rate)
                        else:
                            driver_controller('f', sensor_detect_delay_rate)
                            current_node.wall.append('l')

        except:
            pass 
        current_coordinate = current_node.coordinate
        # Change the priorities of direction exploring consider the current coordinate and the coordinate which the line has been seen in it by x axis
        try:
            if len(line_nodes)%2 == 1:
                if current_coordinate[1] > x_min:
                    check_right = False
                    #Right direction
                    #Check if the destination is new or not
                    if ([current_coordinate[0], current_coordinate[1] + 1] not in coordinat_list):
                        driver_controller('r', sensor_detect_delay_rate, back_to_null=False)
                        if line_flag:
                            situation = 'cr'
                            break
                        psValues = read_sensors()
                        calibr(3,'f')
                        # Check if there is no obstacle in the right direction
                        if (psValues[0] < 90) or (psValues[7] < 90) :
                            current_node = movement_control(current_node, 'r', True)
                            driver_controller('ucwt', 0)
                            if line_flag:
                                situation = 'or'
                            current_node = controller(current_node)
                            raise Exception()

                        #Calibration robot to center
                        elif (psValues[0] < 150) or (psValues[7] < 150):
                            while((psValues[0] < 150) or (psValues[7] < 150)):
                                psValues = read_sensors()
                                move_func(0.01)
                            calibr(3,"f")
                            driver_controller('b', sensor_detect_delay_rate)
                            driver_controller('ucwt', 0)
                        else:
                            driver_controller('b', sensor_detect_delay_rate)
                            driver_controller('ucwt', 0)
                            current_node.wall.append('l')
                # If the current y-coordinate is less than x_max
                elif current_coordinate[1] < x_max:
                    check_left = False
                    #Left direction
                    # Check if the destination is new or not
                    if ([current_coordinate[0], current_coordinate[1] - 1] not in coordinat_list):
                        driver_controller('l', sensor_detect_delay_rate, back_to_null=False)
                        if line_flag:
                            situation = 'cl'
                            break
                        psValues = read_sensors()
                        calibr(3,'f')
                        
                        # Check if there is no obstacle in the left direction
                        if (psValues[0] < 90) or (psValues[7] < 90) :
                            current_node = movement_control(current_node, 'l', True)
                            driver_controller('cwt', 0)
                            if line_flag:
                                situation = 'ol'
                            current_node = controller(current_node)
                            raise Exception()

                        #Calibration robot to center
                        elif (psValues[0] < 150) or (psValues[7] < 150):
                            while((psValues[0] < 150) or (psValues[7] < 150)):
                                psValues = read_sensors()
                                
                                #driver(0.1, 0.1, 1)
                                move_func(0.01)
                            calibr(3,'f')
                            driver_controller('b', sensor_detect_delay_rate)
                            driver_controller('cwt', 0)
                        else:
                            driver_controller('b', sensor_detect_delay_rate)
                            driver_controller('cwt', 0)
                            current_node.wall.append('l')
        except:
            pass
        current_coordinate = current_node.coordinate
        #check if we explore right direction or not
        if check_right:
            #Right direction
            #Check if the destination is new or not
            if ([current_coordinate[0], current_coordinate[1] + 1] not in coordinat_list):
                        driver_controller('r', sensor_detect_delay_rate, back_to_null=False)
                        if line_flag:
                            situation = 'cr'
                            break
                        psValues = read_sensors()
                        calibr(3,'f')
                        
                        # Check if there is no obstacle in the right direction
                        if (psValues[0] < 90) or (psValues[7] < 90) :
                            current_node = movement_control(current_node, 'r', True)
                            driver_controller('ucwt', 0)
                            if line_flag:
                                situation = 'or'
                            current_node = controller(current_node)

                        #Calibration robot to center
                        elif (psValues[0] < 150) or (psValues[7] < 150):
                            while((psValues[0] < 150) or (psValues[7] < 150)):
                                psValues = read_sensors()
                                move_func(0.01)
                            calibr(3,"f")
                            driver_controller('b', sensor_detect_delay_rate)
                            driver_controller('ucwt', 0)
                        else:
                            driver_controller('b', sensor_detect_delay_rate)
                            driver_controller('ucwt', 0)
                            current_node.wall.append('l')
        #check if we explore forward direction or not while we are not watching for another head of line
        if check_forward and (len(line_nodes)%2 != 1):
            #Forward direction   
            if ([current_coordinate[0] + 1, current_coordinate[1]] not in coordinat_list):
                driver_controller('f', sensor_detect_delay_rate)
                if line_flag:
                    situation = 'cf'
                    break
                psValues = read_sensors()
                calibr(3,'f')
                
                # Check if there is no obstacle in the forward direction
                if (psValues[0] < 90) or (psValues[7] < 90) :
                    current_node = movement_control(current_node, 'f', True)
                    if line_flag:
                        situation = 'of'
                    current_node = controller(current_node)

                #Calibration robot to center
                elif (psValues[0] < 150) or (psValues[7] < 150):
                    while((psValues[0] < 150) or (psValues[7] < 150)):
                        psValues = read_sensors()
                        move_func(0.01)
                    calibr(3,'f')
                    driver_controller('b',sensor_detect_delay_rate)
                else:
                    driver_controller('b', sensor_detect_delay_rate)
                    current_node.wall.append('f')  

        #check if we explore right direction or not
        if check_left:
            #Left direction
            if ([current_coordinate[0], current_coordinate[1] - 1] not in coordinat_list):
                driver_controller('l', sensor_detect_delay_rate, back_to_null=False)
                if line_flag:
                    situation = 'cl'
                    break
                psValues = read_sensors()
                calibr(3,'f')
                
                # Check if there is no obstacle in the left direction
                if (psValues[0] < 90) or (psValues[7] < 90) :
                    current_node = movement_control(current_node, 'l', True)
                    driver_controller('cwt', 0)
                    if line_flag:
                        situation = 'ol'
                    current_node = controller(current_node)

                #Calibration robot to center
                elif (psValues[0] < 150) or (psValues[7] < 150):
                    while((psValues[0] < 150) or (psValues[7] < 150)):
                        psValues = read_sensors()
                        
                        #driver(0.1, 0.1, 1)
                        move_func(0.01)
                    calibr(3,'f')
                    driver_controller('b', sensor_detect_delay_rate)
                    driver_controller('cwt', 0)
                else:
                    driver_controller('b', sensor_detect_delay_rate)
                    driver_controller('cwt', 0)
                    current_node.wall.append('l')
        #check forward if we are lokking for the lina after x axis exploring
        if check_forward and (len(line_nodes)%2 == 1):
            #Forward direction   
            if ([current_coordinate[0] + 1, current_coordinate[1]] not in coordinat_list):
                driver_controller('f', sensor_detect_delay_rate)
                if line_flag:
                    situation = 'cf'
                    break
                psValues = read_sensors()
                calibr(3,'f')
            
                  # Check if there is no obstacle in the forward direction
                if (psValues[0] < 90) or (psValues[7] < 90) :
                    current_node = movement_control(current_node, 'f', True)
                    if line_flag:
                        situation = 'of'
                    current_node = controller(current_node)

                #Calibration robot to center
                elif (psValues[0] < 150) or (psValues[7] < 150):
                    while((psValues[0] < 150) or (psValues[7] < 150)):
                        psValues = read_sensors()
                        move_func(0.01)
                    calibr(3,'f')
                    driver_controller('b',sensor_detect_delay_rate)
                else:
                    driver_controller('b', sensor_detect_delay_rate)
                    current_node.wall.append('f') 
        #check if we explore backward direction or not
        if check_backward:
            #Back direction
            if ([current_coordinate[0] - 1, current_coordinate[1]] not in coordinat_list):
                driver_controller('b', sensor_detect_delay_rate)
                if line_flag:
                    situation = 'cb'
                    break
                psValues = read_sensors()
                calibr(3,'b')

                 # Check if there is no obstacle in the backward direction
                if (psValues[3] < 90) or (psValues[4] < 90) :
                    current_node = movement_control(current_node, 'b', True)
                    if line_flag:
                        situation = 'ob'
                    current_node = controller(current_node)
                
                #Calibration robot to center
                elif (psValues[3] < 150) or (psValues[4] < 150):
                    while((psValues[3] < 150) or (psValues[4] < 150)):
                        psValues = read_sensors()
                        
                        #driver(-0.1, -0.1, 1)
                        move_func(-0.01)
                    calibr(3,"b")
                    driver_controller('f', sensor_detect_delay_rate)
                else:
                    driver_controller('f', sensor_detect_delay_rate)
                    current_node.wall.append('l')


        #It gives error when we are at the origin block of robot
        try:
            # At last, the robot will go back to the parent of the current block
            direction = path_finder(current_node.coordinate, current_node.parents.coordinate)
            finish_node = True
            print("finish node:", finish_node)
            current_node = movement_control(current_node, direction, False)
        except: 
            pass

    #Check if the line flags get activated 
    if line_flag:
        #Check the activity while the line has been seen
        print('situation:', situation)
        if situation == 'cf':
            current_coordinate[0] += 1
        elif situation == 'cb':
            current_coordinate[0] -= 1
        # Check if line_nodes is not empty
        if len(line_nodes) != 0:
            for node in line_nodes:
                
                # If the x-coordinate of the node matches the current x-coordinate    
                if node.coordinate[0] == current_coordinate[0]:
                    print("run time")
                    line_nodes.append(current_node)
                    if situation == 'cr':
                        # Move backward for a short distance to check for obstacles    
                        while robot.step(timestep) != -1:
                            driver_controller('b', 0.05, False)
                            gs_values = get_gs_values()
                            if(gs_values[0] < 400 or gs_values[1] <400  or gs_values[2] < 400):
                                break
                        # Rotate the robot depending on the current coordinate
                        if current_coordinate[0] > 0:
                            driver_controller('ucwt', 0) 
                        elif current_coordinate[0] < 0:
                            driver_controller('cwt', 0)
                    elif situation == 'cl':
                        # Move backward for a short distance to check for obstacles
                        while robot.step(timestep) != -1:
                            driver_controller('b', 0.05, False)
                            gs_values = get_gs_values()
                            if(gs_values[0] < 400 or gs_values[1] <400  or gs_values[2] < 400):
                                break
                        driver_controller('cwt', 0) 
                    #mae robot direction matching the line direction
                    elif situation == 'cb':
                        driver_controller('cwt', 0)
                        driver_controller('cwt', 0)
                    elif situation == 'ob':
                        driver_controller('cwt', 0)
                        driver_controller('cwt', 0)

                    # Follow the line after adjustments    
                    line_follower()

                    # Update the current direction and reset line_flag_write
                    current_direction += 180
                    line_flag_write = False
                    driver_controller('f', 0.6)

                    # Adjust robot direction based on current direction
                    if current_direction%360 != 0:
                        line_direction = 'f'
                        driver_controller('ucwt', 0) 
                        driver_controller('ucwt', 0) 
                    else:
                        line_direction = 'b'




                    # After line calibration
                    #Check right
                    distance = 0
                    driver_controller('cwt', 0)
                    psValues = read_sensors()
                    while(((psValues[0] < 150) or (psValues[7] < 150)) and (distance < 0.75)):
                        #caliberate process
                        
                        move_func(0.15)
                        psValues = read_sensors()
                        distance += 0.15
                    if distance == 0.75:
                        driver_controller('b',0.75)
                        driver_controller('ucwt', 0)
                    else:
                        calibr(3,'f')
                        driver_controller('b',sensor_detect_delay_rate)
                        driver_controller('ucwt', 0)
                    #check left
                    distance = 0
                    driver_controller('ucwt', 0)
                    psValues = read_sensors()
                    while(((psValues[0] < 150) or (psValues[7] < 150)) and (distance < 0.75)):
                        #caliberate process
                        move_func(0.15)
                        psValues = read_sensors()
                        distance += 0.15
                    if distance == 0.75:
                        driver_controller('b',0.75)
                        driver_controller('ucwt', 0)
                    else:
                        calibr(3,'f')
                        driver_controller('b',sensor_detect_delay_rate)
                        driver_controller('cwt', 0)

                    #check forward
                    if line_direction == 'f':
                        distance = 0
                        psValues = read_sensors()
                        while(((psValues[3] < 150) or (psValues[4] < 150)) and (distance < 0.75)):
                            move_func(-0.15)
                            psValues = read_sensors()
                            distance += 0.15
                        if distance == 0.75:
                            driver_controller('f',0.75)
                            is_y_calibrated = False
                        else:
                            calibr(3,'b')
                            driver_controller('f',sensor_detect_delay_rate)
                            is_y_calibrated = True

                     #check back   
                    # After line calibration
                    elif line_direction == 'b':
                        distance = 0
                        psValues = read_sensors()
                        while(((psValues[0] < 150) or (psValues[7] < 150)) and (distance < 0.75)):
                            move_func(0.15)
                            psValues = read_sensors()
                            distance += 0.15
                        if distance == 0.75:
                            driver_controller('b',0.75)
                            is_y_calibrated = False
                        else:
                            calibr(3,'f')
                            driver_controller('b',sensor_detect_delay_rate)
                            is_y_calibrated = True

                    #We are going to the parents node of the head of line after following 
                    current_node = node.parents

                    #che if the y axis is not calibrated through the last calibration, calibre it with the parents node
                    if not is_y_calibrated:
                        direction = path_finder(current_node.coordinate, current_node.parents.coordinate)
                        b = movement_control(current_node, direction, False)
                        if line_direction == 'f':
                            distance = 0
                            psValues = read_sensors()
                            while(((psValues[0] < 150) or (psValues[7] < 150)) and (distance < 0.75)):
                                move_func(0.15)
                                psValues = read_sensors()
                                distance += 0.15
                            if distance == 0.75:
                                driver_controller('b',0.75)
                            else:
                                calibr(3,'f')
                                driver_controller('b',sensor_detect_delay_rate)

                        
                        # After line calibration
                        elif line_direction == 'b':
                            distance = 0
                            psValues = read_sensors()
                            while(((psValues[3] < 150) or (psValues[4] < 150)) and (distance < 0.75)):
                                move_func(-0.15)
                                psValues = read_sensors()
                                distance += 0.15
                            if distance == 0.75:
                                driver_controller('f',0.75)
                            else:
                                calibr(3,'b')
                                driver_controller('f',sensor_detect_delay_rate)
                    #current node based on y calibration
                    if is_y_calibrated:
                        current_node = node.parents
                    else:
                        current_node = node.parents.parents
                    new_line = False
                    current_node = controller(current_node)
                    break
                new_line = True
        
        
                
        #If we are at the first head of new line
        print('newline:', new_line)
        if new_line:
            if situation == 'cr':
                driver_controller('b', sensor_detect_delay_rate)
                driver_controller('ucwt', 0) 
            elif situation == 'cl':
                driver_controller('b', sensor_detect_delay_rate)
                driver_controller('ucwt', 0) 
            elif situation == 'cf':
                driver_controller('b', sensor_detect_delay_rate)
            elif situation == 'cb':
                driver_controller('f', sensor_detect_delay_rate)
                
            line_nodes.append(current_node) 
            
            
            direction = path_finder(current_node.coordinate, current_node.parents.coordinate)
            current_node = movement_control(current_node, direction, False)
            line_flag_write = False
            #make situation None again
            situation = None
            
        
        
    return current_node
                
#show tree graph
def show_tree():
    global root
    print('=================================')
    print("Map Root:")
  
    root.show_tree()
    
    
    
#===================================================Start Operation==================================================
#====================================================================================================================
# define root and current root for initializing
root = TreeNode(alphabet[alphabet_counter], [0, 0])
current_node = root


c = 1

#making synchron threads
coordinat_list.append(current_node.coordinate)
move_thread = threading.Thread(target=controller, args=(current_node,))
line_thread = threading.Thread(target=line_detector)
while robot.step(timestep) != -1: 
    while c == 1:
        move_func(0,90)
        c += c
    move_thread.start()
    line_thread.start()
    move_thread.join()
    line_thread.join()
    current_node = controller(current_node)
    
   
    
    
    if current_node == root:
        break
    pass





        