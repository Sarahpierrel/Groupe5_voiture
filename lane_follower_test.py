import cv2
import numpy as np
import math
import serial

_SHOW_IMAGE = False #write True to see all frames

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

class HandCodedLaneFollower(object):

    def __init__(self, car=None):
        self.car = car
        self.curr_steering_angle = 90 # 90 means forward
        self.speed = self.car._LittleBerryCar__INITIAL_SPEED
        self.speedTurning = self.car._LittleBerryCar__SPEED_TURNING

    def follow_lane(self, frame):       
        lane_lines, frame = detect_lane(frame)
        final_frame = self.steer(frame, lane_lines)
        return final_frame

    def steer(self, frame, lane_lines):
        if len(lane_lines) == 0: #no lane lines detected
            return frame

        new_steering_angle = compute_steering_angle(frame, lane_lines)
        self.curr_steering_angle = stabilize_steering_angle(self.curr_steering_angle, new_steering_angle, len(lane_lines))

        if self.car is not None:
            print(self.curr_steering_angle)

            #direction sent to arduino
            print(self.curr_steering_angle)
            if self.curr_steering_angle < 80: # threshold set based on several tests
                ser.write('L'.encode()) #turn left
            
            elif ( self.curr_steering_angle > 100 ):
                ser.write('R'.encode()) #turn right
        
            elif  ( 80 <= self.curr_steering_angle <= 100 ):
                ser.write('F'.encode()) #go forward    
            
        curr_heading_image = display_heading_line(frame, self.curr_steering_angle)
        show_image("heading", curr_heading_image)

        return curr_heading_image


############################
# Frame processing steps
############################
def detect_lane(frame):

    edges = detect_edges(frame)
    show_image('edges', edges)

    cropped_edges = region_of_interest(edges)
    show_image('edges cropped', cropped_edges)

    line_segments = detect_line_segments(cropped_edges)
    line_segment_image = display_lines(frame, line_segments)
    show_image("line segments", line_segment_image)

    lane_lines = average_slope_intercept(frame, line_segments)
    lane_lines_image = display_lines(frame, lane_lines)
    show_image("lane lines", lane_lines_image)

    return lane_lines, lane_lines_image

# Detect line edges   
# First version
def detect_edges( image ) :
    #Transform the image into hsv image
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    show_image("hsv", hsv)
    
    # Define range of white color in HSV
    lower_white = np.array([0, 0, 0])
    upper_white = np.array([179, 43, 255])
    
    # Threshold the HSV image
    mask = cv2.inRange(hsv, lower_white, upper_white)
    show_image("mask", mask)
    
    # Remove noise with opening morphological operation
    kernel_erode = np.ones((2,2), np.uint8)
    eroded_mask = cv2.erode(mask, kernel_erode, iterations=1)
    kernel_dilate = np.ones((3,3),np.uint8)
    dilated_mask = cv2.dilate(eroded_mask, kernel_dilate, iterations=1)
    show_image("mask_200", dilated_mask)
    
    # Detect edges with Canny
    edges = cv2.Canny(mask, 200, 400)
    
    return edges

# # Second version
# def detect_edges(image):
#     #transforming into grayscale image
#     grayscale_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)    
    
#     #Gaussian filter
#     blurred_image = cv2.GaussianBlur(grayscale_image, (5,5), 0) # GaussianBlur(Source, KernelSize, Dimensions)
    
#     #detect edges with Canny filter
#     edges = cv2.Canny(blurred_image, 50, 150)   # --> Canny(image, low_threshold, high_threshold)
    
#     #opening mophological operation to suppress noise
#     kernel_erode = np.ones((2,2), np.uint8)
#     eroded_mask = cv2.erode(edges, kernel_erode, iterations=1)
#     kernel_dilate = np.ones((2,2),np.uint8)
#     dilated_mask = cv2.dilate(eroded_mask, kernel_dilate, iterations=1)
    
#     show_image("mask_200", dilated_mask)
    
#     return edges

# Only take into account the bottom half of the image for the next steps
def region_of_interest(canny):
    height, width = canny.shape
    mask = np.zeros_like(canny)
    
    polygon = np.array([[
        (0, height * 1/2),
        (width, height * 1/2),
        (width, height),
        (0, height),
    ]], np.int32)

    #define the mask
    cv2.fillPoly(mask, polygon, 255)
    show_image("mask", mask)
    
    #add the mask onto the detected edges
    masked_image = cv2.bitwise_and(canny, mask)
    
    return masked_image

# Detect line segments with Hough
def detect_line_segments(cropped_canny):
    rho = 2  # precision in pixel
    angle = np.pi / 180  # degree in radian, i.e. 1 degree
    min_threshold = 100  # minimal of votes
    line_length = 40 # min length of detected line
    max_line_gap = 5 # max distance between two segments to merge them into one
    line_segments = cv2.HoughLinesP(cropped_canny, rho, angle, min_threshold, np.array([]), minLineLength=line_length,
                                    maxLineGap=max_line_gap)

    filtered_line_segments = []
    if line_segments is not None:
        for line_segment in line_segments:
            x1,y1,x2,y2 = line_segment[0]
            length = ((x2-x1)**2 + (y2-y1)**2)**0.5
            if length>50:
                filtered_line_segments.append(line_segment)
                
    return np.array(filtered_line_segments)

# Define zero or two lane lines
def average_slope_intercept(image, lines):
    lane_lines = []
    if lines is None:
        return lane_lines
        
    left_fit = [] # Coordinates from lines on left-side of lane
    right_fit = [] # Coordinates from lines on right-side of lane
    
    height, width, _ = image.shape
    boundary = 1/2
    left_region_boundary = width*(1-boundary)
    right_region_boundary = width*boundary
    
    for line in lines:
        x1, y1, x2, y2 = line.reshape(4)
        parameters = np.polyfit((x1, x2), (y1,y2), 1)
        slope = parameters[0]
        intercept = parameters[1]
        if slope < 0: 
            if x1 < left_region_boundary and x2 < left_region_boundary:
                left_fit.append((slope, intercept))
        elif slope > 0:
            if x1 > right_region_boundary and x2 > right_region_boundary:
                right_fit.append((slope, intercept))
    
    # Only the right line was detected        
    if len(left_fit) == 0 and len(right_fit) > 0:
        right_fit_avg = np.average(right_fit, axis=0)
        # Create left line from right line with opposite parameters
        left_slope = -right_fit_avg[0]
        left_intercept = -right_fit_avg[1]
        left_fit_avg = (left_slope, left_intercept)
        lane_lines.append(make_points(image, left_fit_avg))
        
    # Only the left line was detected
    elif len(left_fit) > 0 and len(right_fit) == 0:
        left_fit_avg = np.average(left_fit, axis=0)
        # Create right line from left line with opposite parameters
        right_slope = -left_fit_avg[0]
        right_intercept = -left_fit_avg[1]
        right_fit_avg = (right_slope, right_intercept)
        lane_lines.append(make_points(image, right_fit_avg))
        
    else:
        left_fit_avg = np.average(left_fit, axis=0)
        if len(left_fit) > 0:
            lane_lines.append(make_points(image,left_fit_avg))
        
        right_fit_avg = np.average(right_fit, axis=0)    
        if len(right_fit) > 0:
            lane_lines.append(make_points(image,right_fit_avg))
       
    return lane_lines


def compute_steering_angle(frame, lane_lines):
    """ Find the steering angle based on lane line coordinate
        We assume that camera is calibrated to point to dead center
    """
    if len(lane_lines) == 0:
        return -90

    height, width, _ = frame.shape
    if len(lane_lines) == 1:
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
    else:
        print(lane_lines[0][0])
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        camera_mid_offset_percent = 0.02 # 0.0 means car pointing to center, -0.03: car is centered to left, +0.03 means car pointing to right
        mid = int(width / 2 * (1 + camera_mid_offset_percent))
        x_offset = (left_x2 + right_x2) / 2 - mid
        
    # _, _, left_x2, _ = lane_lines[0][0]
    # _, _, right_x2, _ = lane_lines[1][0]
    # camera_mid_offset_percent = 0.02 # 0.0 means car pointing to center, -0.03: car is centered to left, +0.03 means car pointing to right
    # mid = int(width / 2 * (1 + camera_mid_offset_percent))
    # x_offset = (left_x2 + right_x2) / 2 - mid

    # find the steering angle, which is angle between navigation direction to end of center line
    y_offset = int(height / 2)

    angle_to_mid_radian = math.atan(x_offset / y_offset)  # angle (in radian) to center vertical line
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # angle (in degrees) to center vertical line
    steering_angle = angle_to_mid_deg + 90  # this is the steering angle needed by picar front wheel

    return steering_angle

def stabilize_steering_angle(curr_steering_angle, new_steering_angle, num_of_lane_lines, max_angle_deviation_two_lines=5, max_angle_deviation_one_lane=1):
    """
    Using last steering angle to stabilize the steering angle
    This can be improved to use last N angles, etc
    if new angle is too different from current angle, only turn by max_angle_deviation degrees
    """
    if num_of_lane_lines == 2 :
        # if both lane lines detected, then we can deviate more
        max_angle_deviation = max_angle_deviation_two_lines
    else :
        # if only one lane detected, don't deviate too much
        max_angle_deviation = max_angle_deviation_one_lane
    
    angle_deviation = new_steering_angle - curr_steering_angle
    if abs(angle_deviation) > max_angle_deviation:
        stabilized_steering_angle = int(curr_steering_angle
                                        + max_angle_deviation * angle_deviation / abs(angle_deviation))
    else:
        stabilized_steering_angle = new_steering_angle
    return stabilized_steering_angle


############################
# Utility Functions
############################

# Define where the lines are on the frame
def make_points(frame, line):
    height, width, _ = frame.shape        
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

    # bound the coordinates within the frame
    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
    return [[x1, y1, x2, y2]]

# Draw the lines onto the frame
def display_lines(frame, lines, line_color=(255,0,0), line_width=10):
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image

# Displayed lines with the displayed steering angle
def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5, ):
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape

    # figure out the heading line from steering angle
    # heading line (x1,y1) is always center bottom of the screen
    # (x2, y2) requires a bit of trigonometry

    steering_angle_radian = steering_angle / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image

# Show the images only if _SHOW_IMAGE is True
def show_image(title, frame, show=_SHOW_IMAGE):
    if show:
        cv2.imshow(title, frame)