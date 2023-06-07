import numpy as np
import matplotlib.pyplot as plt
import queue
import cv2
import copy
import argparse
import rospy
from geometry_msgs.msg import Point, Twist, PoseStamped
import time
from Utils.node import *
from Utils.mapping2ros import *
from Utils.actions import *


# Rounds the number to the closest half value
def customRoundingInt(num, my_int=5):
    rounded = int(my_int*round(num/my_int))
    return rounded

# Computes the "distance" of a given state from the goal state
def get_HeuristicCost(state, goal_state):
    if state is not None:
        hcost = np.sqrt((state[0]-goal_state[0])**2 + (state[1]-goal_state[1])**2)
    else:
        hcost = 0.
    return hcost

# Returns true if the cost of the current node is less than the previously held value
def is_visited(node, goal_state, visited_distances):
    crnt_state = node.get_state()
    x, y, theta = crnt_state[0], crnt_state[1], crnt_state[2]
    x, y = customRoundingInt(x), customRoundingInt(y)
    if node.get_cost() + get_HeuristicCost(crnt_state, goal_state) < visited_distances[x, y]:
        return True
    else:
        return False
    
# Checks if the goal is reached
def is_GoalReached(node, goal_state, threshold_dist):
    crnt_state = node.get_state()
    distance = np.sqrt((crnt_state[0]-goal_state[0])**2 + (crnt_state[1]-goal_state[1])**2)
    if distance <= threshold_dist:
        return True
    else:
        return False

# Flips the matrix upside down to facilitate plotting
def ImagetoPlotCoords(state, plground):
    height, _ , _ = plground.shape
    U = state[0]
    V = height - 1 - state[1] 
    return (int(U), int(V))

# Plots the given visited node on the playground
def plot_visited(plground, node, color=(0, 255, 0)):
    parent = node.get_parent() 
    crnt_state = node.get_state()
    if parent is not None:
        parent_state = parent.get_state()
        u, v = ImagetoPlotCoords(parent_state, plground) 
        u_new, v_new = ImagetoPlotCoords(crnt_state, plground) 
        cv2.line(plground, (u, v), (u_new, v_new), color, 1)
    else:
        u, v = u, v = ImagetoPlotCoords(crnt_state, plground) 
        plground[u, v, :] = color
    return plground


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-s', '--StartState', default="15,15,0", type=str, help="Starting state (x, y, angle), enter values like 20,10,0!")
    parser.add_argument('-g','--GoalState', default="950,950,0", type=str, help="Goal state (x, y, angle), enter values like 20,10,0!")
    parser.add_argument('-z', '--RPM_LR', default="50,100", type=str, help="Wheel velocities (rpm values) of the left and right wheels, respectively.")
    parser.add_argument('-c', '--Clearance', default=5, type=int, help="Clearance between the robot and the obstacles.")
    parser.add_argument('-r', '--RobotRadius', default=10, type=int, help="Radius of the robot.")
    parser.add_argument('-sp', '--SavePath', default="", help="Output file save path.")
    
    args = parser.parse_args()
    start_state = args.StartState
    start_state = [int(i) for i in start_state.split(",")]
    goal_state = args.GoalState
    goal_state = [int(i) for i in goal_state.split(",")]
    wheel_vels = args.RPM_LR
    wheel_vel_l, wheel_vel_r = [float(i) for i in wheel_vels.split(",")]
    clearance = args.Clearance
    robot_radius = args.RobotRadius
    
    my_queue = queue.PriorityQueue()
    start_node = Node(start_state, None, None, 0.)
    my_queue.put((start_node.get_cost(), start_node))
    node_distances = np.ones((1000, 1000))*np.inf
    threshold_dist = 1.5
    clearance += robot_radius
    playground = np.zeros([1000, 1000, 3], dtype=np.uint8)
    playground = get_playground(playground)

    goal_flag = False
    all_neighbour_plots = list()
    while my_queue.empty() == False:
        current_node = my_queue.get()[1]
        # print(current_node.get_state())
        if is_GoalReached(current_node, goal_state, threshold_dist):
            goal_flag = True
            complete_path = current_node.get_path()[1]
            break
        else:
            neighbors, neighbor_plots = get_neighbors(current_node, clearance, ang_vels=(wheel_vel_l, wheel_vel_r))
            all_neighbour_plots.append(neighbor_plots)
            for neighbor in neighbors:
                nbr_state = neighbor.get_state()
                if is_visited(neighbor, goal_state, node_distances): 
                    i = customRoundingInt(nbr_state[0])
                    j = customRoundingInt(nbr_state[1])
                    net_cost = neighbor.get_cost() + 4*get_HeuristicCost(nbr_state, goal_state)
                    node_distances[int(i), int(j)] = net_cost
                    my_queue.put((net_cost, neighbor))
                    
    if goal_flag == False:
        print("Could not find a path!!!")
    
    for neighbor_plots in all_neighbour_plots:
        for j in range(len(neighbor_plots)):
            pt_set1 = neighbor_plots[j][0]
            pt_set2 = neighbor_plots[j][1]
            for i in range(len(pt_set1)):
                pt1 = pt_set1[i]
                pt2 = pt_set2[i]
                pt1 = ImagetoPlotCoords(pt1, playground)
                pt2 = ImagetoPlotCoords(pt2, playground)
                playground = cv2.line(playground, pt1, pt2, (0, 255, 0), 1)

    finals = list()
    for my_node in complete_path:
        playground = plot_visited(playground, my_node, (255, 0, 0))
        grounds = copy.deepcopy(playground)
        for i in range(15):
            finals.append(grounds)
            
    for frm in finals:
        cv2.imshow('finding path', frm)
        if cv2.waitKey(1) == ord('q'):
            print("Saving video.......")
            break
        
    return complete_path

# move robot function
def move_robot(pub_vel, dvx, dvy, dw):
    r = rospy.Rate(100)
    vel_value = Twist()
    velocity = np.sqrt(dvx * dvx + dvy * dvy) / 100.0
    endTime = rospy.Time.now() + rospy.Duration(1)
    
    while rospy.Time.now() < endTime:
        vel_value.linear.x = velocity
        vel_value.angular.z = dw
        pub_vel.publish(vel_value)
        r.sleep()

if __name__ == "__main__":
    
    path = main()
    
    rospy.init_node('turtlebot_astar')
    pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    time.sleep(2)
    action_set = list()
    for elem in path:
        print(elem.get_action())
        dvx, dvy, dw = elem.get_action()
        move_robot(pub_vel, dvx, dvy, dw)
    move_robot(pub_vel, 0., 0., 0.)
    cv2.waitKey(0)
    cv2.destroyAllWindows()