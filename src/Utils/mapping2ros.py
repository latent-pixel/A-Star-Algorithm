import numpy as np
import cv2


def is_PtInCircle(test_pt, clearance, center, r):
    X, Y = center
    x, y = test_pt
    return (x-X)**2 + (y-Y)**2 <= (r+clearance)**2

def is_PtInRectangle(test_pt, clearance, pt1, pt2, pt3, pt4):
    x, y = test_pt
    x_lst = [pt1[0]-clearance, pt2[0]+clearance, pt3[0]+clearance, pt4[0]-clearance]
    y_lst = [pt1[1]+clearance, pt2[1]+clearance, pt3[1]-clearance, pt4[1]-clearance]
    max_x, min_x = max(x_lst), min(x_lst)
    max_y, min_y = max(y_lst), min(y_lst)
    if min_x <= x <= max_x and min_y <= y <= max_y:
        return True
    else:
        return False

def is_ObstacleSpace(u, v, clearance):
    psn = (u, v)
    if (0 <= u <= 1000) and (0 <= v <= 1000) and \
        (is_PtInCircle(psn, clearance, (200, 200), 100) == False) and (is_PtInCircle(psn, clearance, (200, 800), 100) == False) and \
        (is_PtInRectangle(psn, clearance, (25, 575), (175, 575), (175, 425), (25, 425)) == False) and \
        (is_PtInRectangle(psn, clearance, (375, 575), (625, 575), (625, 425), (375, 425)) == False) and \
        (is_PtInRectangle(psn, clearance, (725, 400), (875, 400), (875, 200), (725, 200)) == False):
        return False
    else:
        return True
    
# Creates the obstacle space visualization
def get_playground(playground, clearance = 0):
    breadth, length, _ = playground.shape
    for b in range(breadth):
        for l in range(length):
            psn = (l, b)
            if not ((is_PtInCircle(psn, clearance, (200, 200), 100) == False) and (is_PtInCircle(psn, clearance, (200, 800), 100) == False) and \
                (is_PtInRectangle(psn, clearance, (25, 575), (175, 575), (175, 425), (25, 425)) == False) and \
                (is_PtInRectangle(psn, clearance, (375, 575), (625, 575), (625, 425), (375, 425)) == False) and \
                (is_PtInRectangle(psn, clearance, (725, 400), (875, 400), (875, 200), (725, 200)) == False)):
                playground[breadth - b, l] = [0, 0, 255]
    return playground