import cv2
import numpy as np
import matplotlib.pyplot as plt


def canny(image):
    # canny method for edge detection -> derivateves in all directions (getting gradient)
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    # gaussian blur - reducing noise
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    # canny method for edge detection -> derivateves in all directions (getting gradient)
    canny = cv2.Canny(blur, 50, 150)
    return canny


def region_of_interest(image):
    height = image.shape[0]
    # array of polygons
    polygons = np.array([[(100, height - 20), (700, height - 20), (370, 275)]])
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, polygons, 255)
    # bitwise & of each homologous pixel in both arrays -> showing only the region of interest
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image


def display_lines(image, lines):
    line_image = np.zeros_like(image)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 10)
    return line_image


def make_coordinates(image, line_parameters):
    slope, intercept = line_parameters
    y1 = image.shape[0]
    y2 = int(y1 * (3/5))
    x1 = int((y1 - intercept)/slope)
    x2 = int((y2 - intercept)/slope)
    return np.array([x1, y1, x2, y2])


def average_slope_intercept(image, lines):
    left_fit = []
    right_fit = []
    if lines is None:
        return None
    for line in lines:
        x1, y1, x2, y2 = line.reshape(4)
        # Finding the slope and the y-interception of a line
        # 1 arg = the x-coordinates of two points
        # 2 arg = the y-coordinates of two points
        # 3 arg = the degree of polynomial
        parameters = np.polyfit((x1, x2), (y1, y2), 1)
        slope = parameters[0]
        intercept = parameters[1]
        if slope < 0:
            global left_line
            left_fit.append((slope, intercept))
            left_fit_average = np.average(left_fit, axis=0)
            left_line = make_coordinates(image, left_fit_average)
        else:
            right_fit.append((slope, intercept))
    right_fit_average = np.average(right_fit, axis=0)
    right_line = make_coordinates(image, right_fit_average)
    if left_line is not None:
        return np.array([left_line, right_line])
    else:
        return np.array([right_line])


def find_lanes(image):
    # takes rgb image as argument - 3 channels
    lane_image = np.copy(image)
    # canny filter
    canny_image = canny(lane_image)
    # region of interest
    cropped_image = region_of_interest(canny_image)
    # detecting straight lines (image, theta, row, threshold, placeholder)
    # theta, row ... resolution of hough space - size of bins
    # threshold ... number of intersections needed to detect a line
    lines = cv2.HoughLinesP(cropped_image, 2, np.pi / 180,
                            100, np.array([]), minLineLength=40, maxLineGap=5)
    # averaged_lines = average_slope_intercept(lane_image, lines)
    # print(f'lines: {lines}')
    # print(f'averaged lines: {averaged_lines}')
    line_image = display_lines(lane_image, lines)
    overlay_image = cv2.addWeighted(lane_image, 0.8, line_image, 1, 1)
    # plt.imshow(canny_image)
    # plt.show()
    # cv2.imshow('result', overlay_image)
    # cv2.waitKey(0)
    return overlay_image
