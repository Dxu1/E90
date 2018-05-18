import plan
#import pts_alg as alg
import alg
import graph
import argparse
import pprint
import cv2
import numpy as np
import test_pts as alg2

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
    pts, costDict = plan.plan_path(img, sx, sy, gx, gy)
    #print pts
    for i in range(len(pts)):
        pts[i] = pts[i].split(",")
    for i in range(len(pts)):
        pts[i][0] = int(pts[i][0])
        pts[i][1] = int(pts[i][1])
    newpts = alg.reducePoints(pts)
    newerpts = alg.mergePoints(newpts)
    #print pts
    #print newpts
    print newerpts

    cv2.circle(img, (sx, sy), 10, (255,0,0))
    cv2.circle(img, (gx, gy), 10, (0,255,0))
    cv2.polylines(img, np.array([newerpts], dtype=np.int32), False, (0,0,255))
    cv2.imshow("Path", img)
    # Timeout after 5 seconds
    cv2.waitKey(500000)

