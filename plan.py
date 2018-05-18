# import libraries
import cv2
import math
import os
import sys
import heapq as hq
import argparse
import graph
import pprint
import numpy as np

# define weight based on grayvalues
def greyValue(img, y, x):
    return (8 * (img[y][x][0])/ (255.))

# append list to form complete path
def extractHistory(histDict, lastKey):
    # initialize path
    keyHist = []

    # keep appending until last node
    while len(lastKey) != 0:
        #print("lastKey: %s , prevKey: %s" % (lastKey, histDict[lastKey]))
        keyHist.append(lastKey)
        lastKey = histDict[lastKey]
    return keyHist

# plan path algorithm
def plan_path(img, sx, sy, gx, gy):
    '''initialize image and node'''
    #find intersection point closest to given start point, start from there
    (rows, cols, channels) = img.shape
    #initialize list to contain waypoints
    pointsList = []
    #create startnode and endnode based on input
    startNode = str(sx) + "," + str(sy)
    endNode = str(gx) + "," + str(gy)

    #build graph from .png
    roadGraph = graph.Graph()

    #create nodes and edges based on pixel values on image
    for i in range(rows):
        for j in range(cols):
            #grayscale image returns only tuple of rows and columns
            if(img[i][j][0] != 255):
                #print("found valid node at %s" % str(j) + ',' +str(i))
                newNode = graph.Node(str(j) + "," + str(i))
                for u in [-1,0,1]:
                    for v in [-1,0,1]:
                        if( i + u < rows and i + u > -1 \
                            and j + v < cols and j + v > -1 and \
                            not (u == 0 and v == 0)):
                            if (img[i+u][j+v][0] != 255):
                                dist = math.sqrt(u**2 + v**2)
                                #add edge in name, weight
                                newNode.addEdgeManual(str(j+v) +","+ str(i+u)\
                                        , dist + greyValue(img,i+u,j+v))
                roadGraph.addNode(newNode)

    '''implement Dijkstra's algorithm'''
    # initialize diciontaries to hold cost and previous nodes
    costDict = {}
    pred = {}

    # initialize initial cost
    for i in roadGraph.nodes:
        costDict[i] = 10000000
        pred[i] = ""
    costDict[startNode] = 0

    # initialize start node and end node
    startNode = roadGraph.getNode(startNode)
    endNode = roadGraph.getNode(endNode)

    # use heapq function to implement Dijkestra
    q = []
    #push value into heap
    hq.heappush(q, (0 , startNode))

    while len(q) > 0:
        # pop and return smallest value
        currCost, node = hq.heappop(q)

        # if there are no nodes, no path
        if node == None or endNode == None:
            print("no path found")
            quit()

        # if it is end node, record the whole path
        elif node.name == endNode.name:
            pointsList = extractHistory(pred, node.name)
            return pointsList[::-1], costDict

        # if it is nodes in the middle, record cost and pushed into heap
        else:
            for nnode in node.edges:
                #print("from node %s" % node.name)
                #print("currently considering node: %s" %nnode)
                costNext = currCost + node.edges[nnode]
                if costNext < costDict[nnode]:
                    costDict[nnode] = costNext
                    pred[nnode] = node.name
                    if roadGraph.getNode(nnode) == None:
                        print("from node %s" % node.name)
                        print("currently considering node: %s" %nnode)

                    hq.heappush(q, (costNext, roadGraph.getNode(nnode)))

    # Return a list of tuples, where each tuple is a coordinate pair
    return ([] , costDict)

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("img", help="Greyscale PNG image file serving as the map")
    parser.add_argument("sx", help="Starting position in x", type=int)
    parser.add_argument("sy", help="Starting position in y", type=int)
    parser.add_argument("gx", help="Goal position in x", type=int)
    parser.add_argument("gy", help="Goal position in y", type=int)
    args = parser.parse_args()

    pprint.pprint(args)

    img = args.img
    sx = args.sx
    sy = args.sy
    gx = args.gx
    gy = args.gy

    img = cv2.imread(img)
    # Show where the center of the red object is
    pts, costDict = plan_path(img, sx, sy, gx, gy)
    #print pts
    for i in range(len(pts)):
        pts[i] = pts[i].split(",")


    cv2.circle(img, (sx, sy), 10, (255,0,0))
    cv2.circle(img, (gx, gy), 10, (0,255,0))
    cv2.polylines(img, np.array([pts], dtype=np.int32), False, (0,0,255))
    cv2.imshow("My First Path", img)
    # Timeout after 5 seconds
    cv2.waitKey(500000)
