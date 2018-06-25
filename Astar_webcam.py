import cv2
import numpy
from heapq import *

###Astar part

def heuristic(a, b):
    return (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2

def astar(array, start, goal):

    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]

    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []

    heappush(oheap, (fscore[start], start))
    
    while oheap:

        current = heappop(oheap)[1]

        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j            
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:                
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue
                
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
                
            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heappush(oheap, (fscore[neighbor], neighbor))
                
    return False
####

####cam part

def show_webcam(mirror=False):
	cam = cv2.VideoCapture(0)

	while True:
		ret_val, img = cam.read()
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		shrink = cv2.resize(gray, None, fx=0.3, fy=0.3, interpolation=cv2.INTER_AREA)
		ret,thresh_img = cv2.threshold(shrink,90,1,cv2.THRESH_BINARY_INV)
		ret,thresh_img2 = cv2.threshold(shrink,90,255,cv2.THRESH_BINARY)

		sequance=(astar(thresh_img, (126,96), (18,96)))
		# print sequance
        
		copy_img = numpy.zeros((len(thresh_img),len(thresh_img[3]),3),numpy.float64)
		# print len(shrink[2])
		for k in sequance:
		  	copy_img[k[0],k[1]]=[0,0,255]

		if mirror:
			# cv2.imshow('img', img)
			# cv2.imshow('map for turtle',thresh_img)
			cv2.imshow('camera',shrink)
			cv2.imshow('binary',thresh_img2)
			cv2.imshow('Astar route',copy_img)


		if cv2.waitKey(1) == 27: 
			break
	cv2.destroyAllWindows()

def main():
    show_webcam(mirror=True)


if __name__ == '__main__':
    main()
