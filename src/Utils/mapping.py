# -*- coding: utf-8 -*-
"""
Created on Mon Mar 21 19:30:33 2022

@author: Bhargav Kumar
"""

import numpy as np

def get_line(test_pt, pt1, pt2):
    x1, y1 = pt1
    x2, y2 = pt2
    x, y = test_pt
    if x2 != x1:
        slope = (y2-y1)/(x2-x1)
        fx = y1 + slope*(x-x1)
        return fx
    
def is_ptinhex(test_pt, clearance, center=(200, 100), width=70):
    width = width + 2*clearance
    half_width = width/2
    hex_angl = np.deg2rad(30)
    rad_hex = half_width/np.cos(hex_angl)
    X, Y = center
    x, y = test_pt
    hex_points = np.array([(X, np.ceil(Y+rad_hex)), (X+half_width, np.ceil(Y+rad_hex*np.sin(hex_angl))), (X+half_width, np.ceil(Y-rad_hex*np.sin(hex_angl))), 
                           (X, np.ceil(Y-rad_hex)), (X-half_width, np.ceil(Y-rad_hex*np.sin(hex_angl))), (X-half_width, np.ceil(Y+rad_hex*np.sin(hex_angl)))])
    utriangle_pts = np.array([hex_points[-1], hex_points[0], hex_points[1]])
    rectangle_pts = np.array([hex_points[-2], hex_points[-1], hex_points[1], hex_points[2]])
    ltriangle_pts = np.array([hex_points[-2], hex_points[3], hex_points[2]])
    truth_values = list()
    if min(hex_points[:, 0]) <= x <= max(hex_points[:, 0]):
        if min(utriangle_pts[:, 1]) < y <= max(utriangle_pts[:, 1]):
            fx1 = get_line(test_pt, utriangle_pts[0], utriangle_pts[1])
            fx2 = get_line(test_pt, utriangle_pts[1], utriangle_pts[2])
            if (fx1 >= y) and (fx2 >= y):
                truth_values.append(True)
            else:
                truth_values.append(False)
        elif min(ltriangle_pts[:, 1]) <= y < max(ltriangle_pts[:, 1]):
            fx1 = get_line(test_pt, ltriangle_pts[0], ltriangle_pts[1])
            fx2 = get_line(test_pt, ltriangle_pts[1], ltriangle_pts[2])
            if (fx1 <= y) and (fx2 <= y):
                truth_values.append(True)
            else:
                truth_values.append(False)
        elif min(rectangle_pts[:, 1]) <= y <= max(rectangle_pts[:, 1]):
            truth_values.append(True)
        else:
            truth_values.append(False)
        return any(truth_values)
    else:
        return False

def is_ptinkite(test_pt, clearance, pt1=(36, 185), pt2=(115, 210), pt3=(80, 180), pt4=(105, 100)):
    pt1 = (pt1[0]-clearance, pt1[1])
    pt2 = (pt2[0]+clearance, pt2[1]+clearance)
    pt3 = (pt3[0]+clearance, pt3[1])
    pt4 = (pt4[0]+clearance, pt4[1]-clearance)
    vertices = np.array([pt1, pt2, pt3, pt4])
    x, y = test_pt
    truth_values = list()
    if min(vertices[:, 0]) <= x <= max(vertices[:, 0]):
        fx_AC = get_line(test_pt, vertices[0], vertices[2])
        if fx_AC < y <= max(vertices[:, 1]):
            fx_AB = get_line(test_pt, vertices[0], vertices[1])
            fx_BC = get_line(test_pt, vertices[1], vertices[2])
            if (y <= fx_AB) and (y >= fx_BC):
                truth_values.append(True)
            else:
                truth_values.append(False)
        elif min(vertices[:, 1]) <= y <= fx_AC:
            fx_CD = get_line(test_pt, vertices[2], vertices[3])
            fx_DA = get_line(test_pt, vertices[3], vertices[0])
            if (fx_DA <= y) and (fx_CD >= y):
                truth_values.append(True)
            else:
                truth_values.append(False)
        else:
            truth_values.append(False)
        return any(truth_values)
    else:
        return False

def is_ptincircle(test_pt, clearance, center=(300, 185), r=40):
    X, Y = center
    x, y = test_pt
    return (x-X)**2 + (y-Y)**2 <= (r+clearance)**2

def is_ObstacleSpace(u, v, clearance):
    psn = (u, v)
    if (0 + clearance <= u <= 399 - clearance) and (0 + clearance <= v <= 249 - clearance) and \
        (is_ptinkite(psn, clearance) == False) and (is_ptinhex(psn, clearance) == False) and \
        (is_ptincircle(psn, clearance) == False):
        return False
    else:
        return True