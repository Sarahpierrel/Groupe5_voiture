import cv2
import numpy as np
import matplotlib.pyplot as plt
import logging
import math

plt.close('all')

#Celui qui marche sur la salle
#Canny
def detect_edges( image ) :
    #Transform the image into hsv image
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    cv2.imshow("hsv", hsv)
    
    # Define range of white color in HSV (House test)
    lower_white = np.array([0, 0, 0])
    upper_white = np.array([179, 43, 255])
    # Threshold the HSV image
    mask = cv2.inRange(hsv, lower_white, upper_white)
    cv2.imshow("mask", mask)
    
    # Remove noise
    kernel_erode = np.ones((2,2), np.uint8)
    eroded_mask = cv2.erode(mask, kernel_erode, iterations=1)
    kernel_dilate = np.ones((3,3),np.uint8)
    dilated_mask = cv2.dilate(eroded_mask, kernel_dilate, iterations=1)
    cv2.imshow("mask_200", dilated_mask)
    
    # Detect edges with Canny
    edges = cv2.Canny(mask, 200, 400)
    # cv2.imwrite( "test50-400.png", edges )
    return edges

#Celui qui marche sur des vraies images de route
# def detect_edges( image ) :
#     # Pre-treatment of the image so the lines come out more
#     gray_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#     blurred_img = cv2.GaussianBlur(gray_img, (5, 5), 0)
#     exposure_corrected_img = cv2.convertScaleAbs(blurred_img, alpha=1.2, beta=10)
#     equalized_img = cv2.equalizeHist(exposure_corrected_img)
#     cv2.imshow("equalized", equalized_img)
    
#     kernel_erode = np.ones((3,3), np.uint8)
#     eroded_mask = cv2.erode(equalized_img, kernel_erode, iterations=1)
#     kernel_dilate = np.ones((3,3),np.uint8)
#     dilated_mask = cv2.dilate(eroded_mask, kernel_dilate, iterations=1)
#     cv2.imshow("mask_200", dilated_mask)
   
#     # Detect edges with Canny
#     edges = cv2.Canny(equalized_img, 200, 400)
#     # cv2.imwrite( "test50-400.png", edges )
#     return edges    
    
    # # Adaptive threshold
    # adaptive_threshold = cv2.adaptiveThreshold(equalized_img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
    
    # # Appliquer le seuillage de Canny
    # edges = cv2.Canny(adaptive_threshold, 200, 400)
    
    # kernel = np.ones((5,5),np.uint8)
    # edges = cv2.dilate(edges, kernel, iterations=1)
    # edges = cv2.erode(edges, kernel, iterations=1)
    
    # return edges

# img = cv2.imread('image.jpg', cv2.IMREAD_COLOR)
# #img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# #img = cv2.equalizeHist(img)
# #img = cv2.resize(img, (640, 640))
# cv2.imshow("og", img)
# # hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
# # cv2.imshow("hsv0", hsv[:,:,0])
# # cv2.imshow("hsv1", hsv[:,:,1])
# # cv2.imshow("hsv2", hsv[:,:,2])
# canny = detect_edges(img)
# cv2.imshow("canny", canny)

def region_of_interest(canny):
    height, width = canny.shape

    #Only keep the bottom half of the image as canny, set the rest to 0 (black)    
    top_half = np.full((height//2, width), 0, dtype=np.uint8)
    bottom_half = canny[height//2:, :]
    result = np.concatenate((top_half, bottom_half), axis=0)
    return result

def length_of_line_segment(line):
    x1, y1, x2, y2 = line
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def detect_line_segments(cropped_canny):
    # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
    rho = 1  # precision in pixel, i.e. 1 pixel
    angle = np.pi / 180  # degree in radian, i.e. 1 degree
    min_threshold = 15  # minimal of votes
    line_segments = cv2.HoughLinesP(cropped_canny, rho, angle, min_threshold, np.array([]), minLineLength=8,
                                    maxLineGap=4)

    if line_segments is not None: #ie if lines are detected
        for line_segment in line_segments:
            logging.debug('detected line_segment:')
            logging.debug("%s of length %s" % (line_segment, length_of_line_segment(line_segment[0])))

    return line_segments

def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

    # bound the coordinates within the frame
    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
    return [[x1, y1, x2, y2]]


def average_slope_intercept(frame, line_segments):
    """
    This function combines line segments into one or two lane lines
    If all line slopes are < 0: then we only have detected left lane
    If all line slopes are > 0: then we only have detected right lane
    """
    lane_lines = []
    if line_segments is None:
        logging.info('No line_segment segments detected')
        return lane_lines

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []

    boundary = 1/2 #mis à 1/3 initialement
    left_region_boundary = width * (1 - boundary)  # left lane line segment should be on left 2/3 of the screen
    right_region_boundary = width * boundary # right lane line segment should be on right 1/3 of the screen

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                logging.info('skipping vertical line segment (slope=inf): %s' % line_segment)
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            elif slope > 0:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    # Only the right line was detected
    if len(left_fit) == 0 and len(right_fit) > 0: 
        right_fit_average = np.average(right_fit, axis=0)
        # Create left line from right line with opposite parameters
        left_slope = -right_fit_average[0]
        left_intercept = -right_fit_average[1]
        left_fit_average = (left_slope, left_intercept)
        lane_lines.append(make_points(frame, left_fit_average))
        
    #Only the left line was detected
    elif len(left_fit) > 0 and len(right_fit) == 0:
        left_fit_average = np.average(left_fit, axis=0)
        # Create right line from left line with opposite parameters
        right_slope = -left_fit_average[0]
        right_intercept = -left_fit_average[1]
        right_fit_average = (right_slope, right_intercept)
        lane_lines.append(make_points(frame, right_fit_average))
    
    # Two lines were detected
    else:
        left_fit_average = np.average(left_fit, axis=0)
        if len(left_fit) > 0:
            lane_lines.append(make_points(frame, left_fit_average))
    
        right_fit_average = np.average(right_fit, axis=0)
        if len(right_fit) > 0:
            lane_lines.append(make_points(frame, right_fit_average))

    logging.debug('lane lines: %s' % lane_lines)  # [[[316, 720, 484, 432]], [[1009, 720, 718, 432]]]

    return lane_lines

def compute_steering_angle(frame, lane_lines):
    """ Find the steering angle based on lane line coordinate
        We assume that camera is calibrated to point to dead center
    """
    if len(lane_lines) == 0:
        logging.info('No lane lines detected, do nothing')
        return -90

    height, width, _ = frame.shape
    if len(lane_lines) == 1:
        logging.debug('Only detected one lane line, just follow it. %s' % lane_lines[0])
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
    else:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        camera_mid_offset_percent = 0.02 # 0.0 means car pointing to center, -0.03: car is centered to left, +0.03 means car pointing to right
        mid = int(width / 2 * (1 + camera_mid_offset_percent))
        x_offset = (left_x2 + right_x2) / 2 - mid

    # find the steering angle, which is angle between navigation direction to end of center line
    y_offset = int(height / 2)

    angle_to_mid_radian = math.atan(x_offset / y_offset)  # angle (in radian) to center vertical line
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # angle (in degrees) to center vertical line
    steering_angle = angle_to_mid_deg + 90  # this is the steering angle needed by picar front wheel

    logging.debug('new steering angle: %s' % steering_angle)
    return steering_angle

############################
# Utility Functions
############################
def display_lines(frame, lines, line_color=(0, 255, 0), line_width=10):
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image

def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5, ):
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape

    # figure out the heading line from steering angle
    # heading line (x1,y1) is always center bottom of the screen
    # (x2, y2) requires a bit of trigonometry

    # Note: the steering angle of:
    # 0-89 degree: turn left
    # 90 degree: going straight
    # 91-180 degree: turn right 
    steering_angle_radian = steering_angle / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image

def code_entier( img_path ):
    img = cv2.imread(img_path)
    cv2.imshow("og", img)
    canny = detect_edges(img)
    cv2.imshow("canny", canny)
    region = region_of_interest(canny)
    line_segments = detect_line_segments(region)
    lane_lines = average_slope_intercept(img, line_segments)
    steering_angle = compute_steering_angle(img, lane_lines)
    display = display_lines(img, lane_lines)
    cv2.imshow("display",display)
    display_final = display_heading_line(img, steering_angle)
    cv2.imshow("display final",display_final)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

code_entier("IMG_2029.jpg")
