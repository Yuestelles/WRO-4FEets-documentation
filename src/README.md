import ioty.monitor
import machine
import stepper_wheels
import mpu6050
from ioty import pin
from machine import UART


i2c0 = None
sw_controller = sw_motor0 = sw_motor1 = sw_motor2 = sw_motor3 = None
mpu6050_device = None

speed = None
dir2 = None
correction = None
error = None
uart1 = None
code = None

#
# When Started, run the following code
#
i2c0 = machine.I2C(0, freq=100000)
sw_controller = stepper_wheels.Controller(i2c0, 85)
sw_motor0 = sw_controller.get_motor(0)
sw_motor1 = sw_controller.get_motor(1)
sw_motor2 = sw_controller.get_motor(2)
sw_motor3 = sw_controller.get_motor(3)
mpu6050_device = mpu6050.MPU6050(i2c0, 104)
mpu6050_device.calibrate_gyro()
pin.digital_write(2, 1)
pin.servo_write_deg(22, 88)


# Import the necessary libraries
import time
import math

# Create the sensors and motors objects

#
# These functions makes adjustments to the sensor readings
#

# The negative changes the direction so that CCW is positive,
# and the +90 makes it so that the north is 90 deg (math convention)
def get_gyro():
    return -mpu6050_device.angle_z() + 90

# The gyro angle can be greater than 360 degrees, but it is sometimes useful
# to limit the angle to -180 to 180.
# This function helps us do that, by adding or subtracting 360 degrees if it
# is outside of the desired range.
def limit_angle(angle):
    while angle > 60:
        angle -= 360
    while angle < -60:
        angle += 360
    return angle

# These laser functions returns the distance with some additions so
# that the distance is to the center of the robot and not the sensor
def get_front_laser():
    m = 8.398336 #THIS WORKS AHHHH front
    a = 849693
    
    x = float(pin.analog_read(32))  # Read the analog value within the loop
    if x == 0:  # Check for division by zero
        return 400
    elif x <= 45000:
        return (a / x) + m + 13
    else:
        return 0

def get_rear_laser():
    m = 8.398336 #THIS WORKS AHHHH front
    a = 849693
    
    x = float(pin.analog_read(34))  # Read the analog value within the loop
    if x == 0:  # Check for division by zero
        return 400
    elif x <= 45000:
        return (a / x) + m + 13
    else:
        return 0

def get_left_laser():
    m = -1.554024 #THIS WORKS AHHHH side
    a = 489325.7

    x = float(pin.analog_read(33))  # Read the analog value within the loop
    if x == 0:  # Check for division by zero
        return 400
    elif x <= 45000:
        return (a / x) + m + 8
    else:
        return 0

def get_right_laser():
    m = -1.554024 #THIS WORKS AHHHH side
    a = 489325.7

    x = float(pin.analog_read(35))  # Read the analog value within the loop
    if x == 0:  # Check for division by zero
        return 400
    elif x <= 45000:
        return (a / x) + m + 8
    else:
        return 0
#    
# These functions are to control the movement and steering
#

# Controls the front wheel (steering)
def steer(wheel_angle):
    pin.servo_write_deg(22, 88 + wheel_angle)
 
# Controls the steering and back wheel (drive wheel)
def drive(wheel_angle, speed):
    steer(wheel_angle)
    sw_motor0.run(500)

# Proportional control to steering the robot towards the
# given "direction". The current gyro angle needs to be supplied
def drive_heading(direction, speed, gyro_angle):
    error = direction - gyro_angle
    correction = -2 * error
    correction = min(max(correction, -45), 45)
    drive(correction, speed)


#
# These functions are used to determine robot position using laser sensors
#

# Determine the equation of the laser line. Returns m and c.
# Must be supplied with the gyro angle and estimated position
def get_laser_line_eqn(gyro_angle, pos):
    m = math.tan(math.radians(gyro_angle))
    x = pos[0]
    y = pos[1]
    c = y - m * x
    return m, c

# Check if laser intercepts the top wall
# Top wall eqn: y = 300
# Combined with laser line eqn: 
#   300 = m*x + c
#   m*x = 300 - c
#   x = (300 - c) / m
def intercept_top(angle, m, c):
    # A line stretches to infinity, both forward and back, but the laser only goes forward.
    # If the laser is facing away from the wall (angle between 0 to -180), it cannot intercept the wall.
    if -60 <= limit_angle(angle) <= 0:
        return False
    
    if m == 0: # Prevents a divide by zero error in the next line
        return False
    x = (300 - c) / m
    # Return true if intercept point is at least 5cm from edge
    # The 5cm gives us a little buffer as the estimated position may be 
    # inaccurate, and we may intercept the left/right wall instead
    if 5 < x < 295:
        return True
    return False

# Check if laser intercepts the bottom wall
# Bottom wall eqn: y = 0
# Combined with laser line eqn: 
def intercept_bottom(angle, m, c):
    if 0 <= limit_angle(angle) <= 60:
        return False
    
    if m == 0:
        return False
    x = (0 - c) / m

    if 5 < x < 295:
        return True
    return False

# Check if laser intercepts the left wall
# Top wall eqn: x = 0
# Combined with laser line eqn: 
#   y = m*0 + c
#   y = c
def intercept_left(angle, m, c):
    if -60 <= limit_angle(angle) <= 60:
        return False
    
    y = c
    if 5 < y < 295:
        return True
    return False

# Check if laser intercepts the right wall
# Top wall eqn: x = 300
# Combined with laser line eqn: 
#   y = m*300 + c
def intercept_right(angle, m, c):
    if limit_angle(angle) >= 60 or limit_angle(angle) <= -60:
        return False

    y = m * 300 + c
    if 5 < y < 295:
        return True
    return False


#
# These functions are used for estimating robot position
#

# This converts motor degrees to distance in cm
prev_motor_degrees = 0
def get_distance_travelled():
    global prev_motor_degrees
    
    current_degrees = sw_motor0.steps()
    cm = (current_degrees - prev_motor_degrees) / 360 * 17.6
    prev_motor_degrees = current_degrees
    
    return cm

# Estimate position
def get_position_estimate(prev_pos, avg_gyro_angle, distance_travelled):
    dx = distance_travelled * math.cos(math.radians(avg_gyro_angle))
    dy = distance_travelled * math.sin(math.radians(avg_gyro_angle))

    x = prev_pos[0] + dx
    y = prev_pos[1] + dy
    return x, y


#
# These functions calculate position from laser measurements
#

def calculate_xy(angle, est_pos, distance):
    m, c = get_laser_line_eqn(angle, est_pos)
    
    # Initialize x and y position with None
    x = None
    y = None

    if intercept_top(angle, m, c):
        # Top wall is 300cm from origin, so our position is 300 - y_distance_to_wall
        y = 300 - distance * math.sin(math.radians(angle))
    elif intercept_bottom(angle, m, c):
        # Bottom wall is at origin, but measuring downwards will give us a negative
        # value, so we invert in with a "-" at the front.
        y = -distance * math.sin(math.radians(angle))
    elif intercept_left(angle, m, c):
        # Left wall is at origin, but measuring towards the left will give us a negative
        # value, so we invert in with a "-" at the front.
        x = -distance * math.cos(math.radians(angle))
    elif intercept_right(angle, m, c):
        # Right wall is 300cm from origin, so our position is 300 - x_distance_to_wall
        # value, so we invert in with a "-" at the front.
        x = 300 - distance * math.cos(math.radians(angle))
        
    return x, y

# Calculate robot's position from sensors. 
# We need the estimated position to determine which wall each laser intercepts
def calculate_position(gyro_angle, est_pos):
    # Since we have multiple sensors, we may get more than one value each for x/y.
    # So we create a list here to hold all the values
    all_x = []
    all_y = []

    # Create a list of all sensor angles and distances
    sensors = [
        [gyro_angle, get_front_laser()],
        [gyro_angle + 90, get_left_laser()],
        [gyro_angle + 180, get_rear_laser()],
        [gyro_angle + 270, get_right_laser()],
    ]

    # Discard if position differs too much from estimate,
    # this helps to eliminate outliers.
    for sensor in sensors:
        x, y = calculate_xy(sensor[0], est_pos, sensor[1])
        if x != None and abs(x - est_pos[0]) < 5:
            all_x.append(x)
        if y != None and abs(y - est_pos[1]) < 5:
            all_y.append(y)

    # Calculate the average x and y position
    # First we count how many x and y values we have...
    x_count = len(all_x)
    y_count = len(all_y)
    
    # ...next we initialize x and y with None
    x = None
    y = None
    
    # ...finally we divide the sum of x/y with their count, but
    # only if the count is > 0 (...to prevent a divide by zero)
    if x_count > 0:
        x = sum(all_x) / x_count
    if y_count > 0:
        y = sum(all_y) / y_count
        
    return x, y
    
# This code was in the "while True" loop in the earlier example.
# Here we put it in it's own function.
def get_position(gyro_angle):
    global prev_pos, prev_gyro_angle

    # To estimate our position, we need to know the previous position, the direction
    # the robot is facing (...we use average of previous and current gyro angle), and
    # the distance travelled (...calculated from motor rotation).
    distance_travelled = get_distance_travelled()
    avg_gyro_angle = (gyro_angle + prev_gyro_angle) / 2
    est_pos = get_position_estimate(prev_pos, avg_gyro_angle, distance_travelled)

    # Calculate the position using our estimated position and sensors readings
    measured_pos = calculate_position(gyro_angle, est_pos)

    # Decide if we'll use the measured or estimated x/y values.
    # If measured value is None, we'll use estimate.
    # Else if the measured value is within 5cm of the estimate, we'll use the
    # measured value
    x, y = est_pos
    if measured_pos[0] != None and abs(measured_pos[0] - est_pos[0]) < 5:
        x = measured_pos[0]

    if measured_pos[1] != None and abs(measured_pos[1] - est_pos[1]) < 5:
        y = measured_pos[1]

    pos = [x, y]
    
    # Update prev values with the current values
    prev_pos = pos
    prev_gyro_angle = gyro_angle
    
    return pos

#
# These functions are for navigating along a path
#

# Vector dot product. This is used in the drive_path function.
def dot(v1, v2):
    return v1[0]*v2[0] + v1[1]*v2[1]

# Drive from start position to end position
def drive_path(start, end):
    path_vec = [end[0] - start[0], end[1] - start[1]]
    path_length = math.sqrt(path_vec[0]**2 + path_vec[1]**2)
    path_direction = math.degrees(math.atan2(path_vec[1], path_vec[0]))
    unit_path_vec = [path_vec[0] / path_length, path_vec[1] / path_length]
    unit_perpen_vec = [unit_path_vec[1], -unit_path_vec[0]]
    
    gyro_angle = get_gyro()
    while path_direction - gyro_angle > 60:
        path_direction -= 360
    while path_direction - gyro_angle < -60:
        path_direction += 360

    while True:
        gyro_angle = get_gyro()
        pos = get_position(gyro_angle)
        print(pos)

        pos_vec = [pos[0] - start[0], pos[1] - start[1]]
        distance_travelled = dot(pos_vec, unit_path_vec)
        error = dot(pos_vec, unit_perpen_vec)
        if distance_travelled >= path_length:
            break
        correction = error * 4
        correction = min(max(correction, -60), 60)
        drive_heading(path_direction+correction, 60, gyro_angle)


#
# Main code starts here
#

# Initialize the previous position based on the starting measured position
prev_pos = [get_left_laser(), get_rear_laser()]
prev_gyro_angle = 90 # starting angle is always 90

# while True:
#     matches = uart1.readline()
#     if matches:    # Check if code is not None and not an empty byte object
#         matches = matches.decode()  # Decode the byte object to a string
#         matches = int(matches)      # Convert the string to an integer
#         print(matches)     # Print the integer value
#                     path = 0
#             last_node = len(nodes) - 1

#         if matches == 1:
#             nodes = [
#                 [50, 50],
#                 [50, 250],
#                 [250, 250],
#                 [250, 50],
#             ]
#         else if matches == 2:
#             nodes = [
#                 [50, 50],
#                 [50, 250],
#                 [250, 250],
#                 [250, 50],
#             ]
#         else:
#             continue


# These are the corners of the lines
# nodes = [
#     [50, 50],
#     [50, 250],
#     [250, 250],
#     [250, 50],
# ]
nodes = [
    [50, 160],
    [160, 250],
#     [250, 250],
#     [250, 50],
]
path = 0
last_node = len(nodes) - 1

while True:
    start_node = nodes[path]
    # If the starting node is the last one, then the ending node must be the first
    if path == last_node:
        end_node = nodes[0]
    else:
        end_node = nodes[path+1]

    # Drive from start node to end node
    drive_path(start_node, end_node)
    
    # We've reached the end of the path, so move on to the next one.
    path += 1
    if path > last_node: # If at the last node, we'll start again at the first
        path = 0

# Initialize UART with the given parameters
# uart1 = machine.UART(1, baudrate=9600, tx=16, rx=17)
