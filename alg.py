import math

def reducePoints(points):
    newList = []
    newList.append(points[0])

    dx = points[1][0] - points[0][0]
    dy = points[1][1] - points[0][1]


    for i in range(1,len(points)-1):
	newdx = points[i+1][0] - points[i][0]
	newdy = points[i+1][1] - points[i][1]
	if not (newdx == dx and newdy == dy):
		newList.append(points[i])
		dx = newdx
		dy = newdy
			
    newList.append(points[-1])

    return newList


def mergePoints(points):
    newpts = []
    newpts.append(points[0])

    i = 0
    j = 0
    while i < len(points)-1:
        if dist(points[j],points[i+1]) > 18:
            newpts.append(points[i+1])
	    j = i
        i += 1

    #newpts.append(points[-1])

    return newpts


def avg(pt1 ,pt2):
	return [(pt1[0] + pt2[0])/2 ,(pt1[1] + pt2[1])/2 ]

def dist(pt1, pt2):
	return math.sqrt((pt1[0]-pt2[0])**2 + (pt1[1]-pt2[1])**2)

if __name__ == "__main__":
	print points
	print reducePoints(points)
	print mergePoints(points)
	print mergePoints(reducePoints(points))
