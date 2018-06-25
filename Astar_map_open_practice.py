import numpy
import cv2
from heapq import *


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


turtlemap = cv2.imread('111.png')
gray=cv2.cvtColor(turtlemap, cv2.COLOR_BGR2GRAY)
shrink=cv2.resize(gray, None, fx=0.36, fy=0.35, interpolation=cv2.INTER_AREA)
nmap=cv2.threshold(shrink,200,1,cv2.THRESH_BINARY_INV)

# print len(nmap[1])
# print(astar(nmap, (0,3), (10,13)))
sequance=(astar(nmap[1], (3,16), (21,16)))
sequance.append((3,16))
sequance.reverse()

# print sequance[1][1]

copy_img = numpy.zeros((len(nmap[1]),len(nmap[1][3]),3),numpy.float64)

for y in range(len(nmap[1])):
    for x in range(len(nmap[1][3])):
        if nmap[1][y,x]==0:
            copy_img[y,x]=[255,255,255]
        elif nmap[1][y,x]==1:
            copy_img[y,x]=[0,0,0]

for k in sequance:
    copy_img[k[0],k[1]]=[0,0,255]

cv2.imshow('hey',copy_img)
cv2.waitKey(0)