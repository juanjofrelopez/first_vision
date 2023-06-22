import rospy
import time
import math
import sys

import pyrealsense2 as rs
import numpy as np
import cv2
from cv_bridge import CvBridge

# Global filter Variables
poly_buffer_size = np.int16(10)
poly_buffer_indx = np.int16(0)
poly_buffer = np.empty((poly_buffer_size , 4 , 2), dtype=np.int16)

def filter_depth(depth_values,clipping_distance):
    # # Processing            
    # Remove background
    depth_values = (depth_values < clipping_distance) * depth_values
    # Convert depth to rgb colormap
    # depth_image_rgb = cv2.cvtColor(depth_image, cv2.COLOR_BGR2RGB)
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_values), cv2.COLORMAP_JET)
    # Binarizar la imagen
    thresh = 255
    max_value = 255
    _,thresholded_image = cv2.threshold(cv2.convertScaleAbs(depth_values),thresh,max_value,cv2.THRESH_TRUNC)
    # Denoise
    denoise = cv2.medianBlur(thresholded_image,ksize = 7)
    # Morph close (dilation erotion)
    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS,(21,21))
    close = cv2.morphologyEx(denoise, cv2.MORPH_CLOSE, kernel)
    return close,depth_colormap

def simplify_contour(contour, n_corners=4):
    '''
    Binary searches best `epsilon` value to force contour 
        approximation contain exactly `n_corners` points.

    :param contour: OpenCV2 contour.
    :param n_corners: Number of corners (points) the contour must contain.

    :returns: Simplified contour in successful case. Otherwise returns initial contour.
    '''
    n_iter, max_iter = 0, 100
    lb, ub = 0., 1.

    while True:
        n_iter += 1
        if n_iter > max_iter:
            return contour

        k = (lb + ub)/2.
        eps = k*cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, eps, True)

        if len(approx) > n_corners:
            lb = (lb + ub)/2.
        elif len(approx) < n_corners:
            ub = (lb + ub)/2.
        else:
            return approx

def filter_poly_lines(contour):
    global poly_buffer_indx
    global poly_buffer
    global poly_buffer_size

    p1=[contour[0][0][0], contour[0][0][1]]
    p2=[contour[1][0][0], contour[1][0][1]]
    p3=[contour[2][0][0], contour[2][0][1]]
    p4=[contour[3][0][0], contour[3][0][1]]

    filtered_output = np.empty((4,2), dtype=np.int16)
    # Apend box to buffer
    poly_buffer[poly_buffer_indx] = [p1,p2,p3,p4]
    poly_buffer_indx = (poly_buffer_indx+1)%poly_buffer_size
    # iterate
    d,r,c = poly_buffer.shape # depth, row, columns
    for i in range(r):
        for j in range(c):
            temporal = np.empty(d, dtype=np.int16)
            for k in range(d):
                temporal[k] = (poly_buffer[k][i][j])
            # get median
            filtered_output[i][j] = np.median(temporal)

    p1=(filtered_output[0][0], filtered_output[0][1])
    p2=(filtered_output[1][0], filtered_output[1][1])
    p3=(filtered_output[2][0], filtered_output[2][1])
    p4=(filtered_output[3][0], filtered_output[3][1])

    return p1,p2,p3,p4

def poly_contour(depth_colormap,contour):    
    approximated_contour = simplify_contour(contour)
    p1,p2,p3,p4 = filter_poly_lines(approximated_contour)
    # if the line is vertical, draw it
    # BUG: ojo con las slopes porque pueden dar overflow
    slope1 = abs((p2[1] - p1[1]) / (p2[0] - p1[0] + 1e-6))
    slope2 = abs((p4[1] - p3[1]) / (p4[0] - p3[0] + 1e-6))
    if slope1 > 5 and slope2 > 5:    
        cv2.line(depth_colormap,p1,p2,(0,255,0) ,3,cv2.LINE_AA)
        cv2.line(depth_colormap,p3,p4,(255,128,0), 3,cv2.LINE_AA)
    return depth_colormap

def find_draw_contours(close,depth_colormap):
    contours, hierarchy = cv2.findContours(close,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for contour in contours:
        area = cv2.contourArea(contour)
        if not(area < 150000 or area > 200000):
            #depth_colormap = box_contour(depth_colormap,contour)
            depth_colormap = poly_contour(depth_colormap,contour)
            #depth_colormap = line_contour(depth_colormap,contour)
    return depth_colormap

def get_ground_distance(depth_values):
    # ojo que depth values esta en dimensiones (480,640) !!!
    row_indxs = list(range(15,70))
    col_indxs = list(range(15,465))
    left_snippet = depth_values[col_indxs][:,row_indxs]
    
    row_indxs = list(range(570,625))
    col_indxs = list(range(15,465))
    right_snippet = depth_values[col_indxs][:,row_indxs]

    ground = np.concatenate((left_snippet,right_snippet),axis=1)
    ground_distance = np.median(ground)
    return ground_distance

def get_panel_distance(depth_values):
    row_indxs = list(range(292,348))
    col_indxs = list(range(15,465))
    panel_snippet = depth_values[col_indxs][:,row_indxs]
    panel_distance = np.median(panel_snippet)
    return panel_distance

def get_clipping_distance(panel_distance,ground_distance):
    ratio = ground_distance - panel_distance
    if(ratio < 1000 or ratio > 3000 or panel_distance < 3000):
        #print('ratio feo')
        return 5000
    #print('ratio lindo')
    clipping_distance = (panel_distance + ground_distance)/2.0
    return clipping_distance

def process_depth(depth_image):
    depth_values = np.array(depth_image, dtype=np.int32)
    
    # Get the median distance to the solar panel (center area)
    panel_distance = get_panel_distance(depth_values)
    
    # Get the median distance to ground (left and right area)
    ground_distance = get_ground_distance(depth_values)
    
    # Get clipping distance
    clipping_distance = get_clipping_distance(panel_distance,ground_distance)
    
    # Filter the depth values
    close,depth_colormap = filter_depth(depth_values,clipping_distance)
    
    # Find and draw contours
    depth_colormap = find_draw_contours(close,depth_colormap)
    
    #depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_values), cv2.COLORMAP_JET)
    return depth_colormap