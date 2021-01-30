#!/usr/bin/env python

import std_msgs.msg
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
import math
import sys
from matplotlib import image
from skimage.draw import random_shapes

import numpy as np
import matplotlib.pyplot as plt





np.set_printoptions(threshold=sys.maxsize)

def distance(x1,x2,y1,y2):
        dist = math.sqrt((x2-x1)**2 + (y2-y1)**2)
        return dist


def WorldG():
    image2, _ = random_shapes((128, 128),shape='circle',min_shapes=10,max_shapes=15, min_size=20,max_size=25,
                          intensity_range=((100, 254),))
    

    image2=np.where(image2==255,0,image2)
    x_1=np.random.randint(0,120)
    y_1=np.random.randint(0,120)
    x_2=np.random.randint(0,120)
    y_2=np.random.randint(0,120)
    while distance(x_1,x_2,y_1,y_2)<50:
        x_1=np.random.randint(0,120)
        y_1=np.random.randint(0,120)
        x_2=np.random.randint(0,120)
        y_2=np.random.randint(0,120)

    cv2.circle(image2,(x_1,y_1),5,(255,255,255),thickness=cv2.FILLED)
    cv2.circle(image2,(x_2,y_2),5,(255,255,255),thickness=cv2.FILLED)
    cv2.imwrite('test.png',image2)

    #plt.imshow(image2)
    #plt.title('my picture1')
    #plt.show()
    return image2



def Marker1(image2):
    grey = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)
    res = image2
    for x in range (128):
        for y in range (128):
            if grey[x][y] != 255.0:
                grey[x][y] = 0

    grey = cv2.medianBlur(grey, 3)
    rows = grey.shape[0]
    circles = cv2.HoughCircles(grey, cv2.HOUGH_GRADIENT, 1,rows/1000,param1=80, param2=5,minRadius=4, maxRadius=6)

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            center = (i[0], i[1])
            # circle center
            cv2.circle(res, center, 1, (0, 100, 100), 1)                        # circle outline
            radius = i[2]
            cv2.circle(res, center, radius, (255, 0, 255), 1)

    plt.imshow(res)
    plt.title('my picture1')
    plt.show()
    return circles

def Marker(image2):
    grey = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)
    res = image2

    for x in range (128):
        for y in range (128):
            if grey[x][y] != 255.0:
                grey[x][y] = 0

    grey = cv2.medianBlur(grey, 3)
    rows = grey.shape[0]
    circles = cv2.HoughCircles(grey, cv2.HOUGH_GRADIENT, 1,rows/10000,param1=80, param2=5,minRadius=4, maxRadius=6)

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            center = (i[0], i[1])
            cv2.circle(res, center, 1, (0, 100, 100), 1)                        
            radius = i[2]
            cv2.circle(res, center, radius, (255, 0, 255), 1)

    #plt.imshow(res)
    #plt.title('my picture3')
    #plt.show()
    print(circles)
    #cv2.show(res)
    
    return circles


def Check1(starty,endy):
    print("check1")

    if starty is None:
        print("route doesnt exist")
        return True
    elif endy is None:
        print("route doesnt exist")
        return True
    
    return None






def MazeGen(gray):
    for x in range (128):
        for y in range (128):
            if gray[x][y] > 0.0 and gray[x][y] < 255.0:
                gray[x][y] = 1
            elif gray[x][y] == 255.0:
                gray[x][y] = 0
    return gray



class Node():
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0
    def __eq__(self, other):
        return self.position == other.position

def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0:

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_list.append(child)


def printpath(start,end,gray,image2):
    path = astar(gray, start, end)
    print(path)
    out_arr = np.asarray(path)
    print(out_arr)
    m,n=out_arr.shape
    if path is not None:
        for i in range (m-1):
          startx1 = path[i][0]
          starty1 = path[i][1]
          window_name = 'Image'
          end1 = path[i+1][0]
          end2 = path[i+1][1]
          start = (startx1,starty1)
          end = (end1,end2)
          color = (255, 255, 255) 
          thickness = 1
          image2 = cv2.line(image2, start, end, color, thickness)
    return image2



def final():

    image2 = WorldG()
    res = image2
    print("hoi")
    gray = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)
    print("hoi")
    circles = Marker(res)
    
    startx = circles[0,0,0]
    print("hoi1")
    starty = circles[0,0,1]
    endx = circles[0,1,0]
    endy = circles[0,1,1]
    check=Check1(starty,endy)
    print("hoi1")

    if check is True:
        print("No Path")
        exit()
    
    start = (startx,starty)
    end = (endx,endy)
    print("hoi2")
    gray=MazeGen(gray)
    print("hoi3")
    image1=printpath(start,end,gray,image2)
    plt.imshow(image1)
    plt.title('my picture3')
    plt.show()
    cv2.imshow(image1)
    cv2.waitKey(0)
    print("hoii")
    return image1

def publish_message():


    pub = rospy.Publisher('output', Image, queue_size=10)
    print("ho")
  
    frame = final()
    print("ho")

    br = CvBridge()
    pub.publish(br.cv2_to_imgmsg(frame))
    plt.imshow(frame)
    plt.title('my picture1')
    plt.show()           

      

if __name__=="__main__":
    try:
        publish_message()
    except:rospy.ROSInterruptException
    pass