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


# turtlemap = cv2.imread('map1.png')
# gray=cv2.cvtColor(turtlemap, cv2.COLOR_BGR2GRAY)
# shrink=cv2.resize(gray, None, fx=0.36, fy=0.35, interpolation=cv2.INTER_AREA)

img = cv2.imread('map1.png',0)

img_buffer1 = cv2.imread('map1.png',0)
img_buffer2 = cv2.imread('map1.png',0)
img_buffer3 = cv2.imread('map1.png',0)
img_buffer4 = cv2.imread('map1.png',0)
img_buffer5 = cv2.imread('map1.png',0)
img_buffer6 = cv2.imread('map1.png',0)
img_buffer7 = cv2.imread('map1.png',0)
img_buffer8 = cv2.imread('map1.png',0)
img_buffer9 = cv2.imread('map1.png',0)
img_buffer10 = cv2.imread('map1.png',0)
img_buffer11 = cv2.imread('map1.png',0)

buffer=[img_buffer1,img_buffer2,img_buffer3,img_buffer4,img_buffer5,img_buffer6,img_buffer7,img_buffer8,img_buffer9,img_buffer10,img_buffer11]

for k in range(10):
    for y in range(1,len(img)-1):
        for x in range(1,len(img[y])-1):
            if buffer[k][y,x]==255 and (buffer[k][y,x+1]==0 or buffer[k][y+1,x+1]==0 or buffer[k][y+1,x]==0 or buffer[k][y+1,x-1]==0 or buffer[k][y,x-1]==0 or buffer[k][y-1,x-1]==0 or buffer[k][y-1,x]==0 or buffer[k][y-1,x+1]==0):
                buffer[k+1][y,x]=0


nmap=cv2.threshold(img_buffer11,200,1,cv2.THRESH_BINARY_INV)

# print len(nmap[1])
# print(astar(nmap, (0,3), (10,13)))
sequance=(astar(nmap[1], (126,96), (18,96)))
sequance.append((126,96))
sequance.reverse()

print sequance

direction=[]
direction_letter=[]

for k in range(len(sequance)-1):
    if sequance[k+1][1]-sequance[k][1]==1 and sequance[k+1][0]-sequance[k][0]==0:
        # print("right")
        direction.append(-2)
        direction_letter.append('right')
    elif sequance[k+1][1]-sequance[k][1]==1 and sequance[k+1][0]-sequance[k][0]==-1:
        # print("up right")
        direction.append(-1)
        direction_letter.append('up right')
    elif sequance[k+1][1]-sequance[k][1]==0 and sequance[k+1][0]-sequance[k][0]==-1:
        # print("up")
        direction.append(0)
        direction_letter.append('up')
    elif sequance[k+1][1]-sequance[k][1]==-1 and sequance[k+1][0]-sequance[k][0]==-1:
        # print("up left")
        direction.append(1)
        direction_letter.append('up left')
    elif sequance[k+1][1]-sequance[k][1]==-1 and sequance[k+1][0]-sequance[k][0]==0:
        # print("left")
        direction.append(2)
        direction_letter.append('left')
    elif sequance[k+1][1]-sequance[k][1]==-1 and sequance[k+1][0]-sequance[k][0]==1:
        # print("down left")
        direction.append(3)
        direction_letter.append('down left')
    elif sequance[k+1][1]-sequance[k][1]==0 and sequance[k+1][0]-sequance[k][0]==1:
        # print("down")
        direction.append(4)
        direction_letter.append('down')
    elif sequance[k+1][1]-sequance[k][1]==1 and sequance[k+1][0]-sequance[k][0]==1:
        # print("down right")
        direction.append(-3)
        direction_letter.append('down right')

direction.append(0)


print direction_letter

direction2=[]
direction2_letter=[]
count=1
for k in range(len(direction)-1):

    if k==len(direction)-2:
        direction2.append([direction[k],count])
    elif direction[k]==direction[k+1]:
        count=count+1
    else:
        direction2.append([direction[k],count])
        count=1

for a in range(len(direction2)):
    if direction2[a][0]==0:
        direction2_letter.append(['up',direction2[a][1]])
    if direction2[a][0]==1:
        direction2_letter.append(['up left',direction2[a][1]])
    if direction2[a][0]==2:
        direction2_letter.append(['left',direction2[a][1]])
    if direction2[a][0]==3:
        direction2_letter.append(['down left',direction2[a][1]])
    if direction2[a][0]==4:
        direction2_letter.append(['down',direction2[a][1]])
    if direction2[a][0]==-3:
        direction2_letter.append(['down right',direction2[a][1]])
    if direction2[a][0]==-2:
        direction2_letter.append(['right',direction2[a][1]])
    if direction2[a][0]==-1:
        direction2_letter.append(['up right',direction2[a][1]])
 
# print direction2
print direction2_letter

direction3=[]
direction3_letter=[]
tilt=0
direction3.append([direction2[0][0],direction2[0][1],tilt])
for i in range(1,len(direction2)):
    tilt= tilt +(direction2[i][0]-direction2[i-1][0])
    direction3.append([(direction2[i][0]-direction2[i-1][0]),direction2[i][1],tilt])

for a in range(len(direction3)):
    if direction3[a][0]==0:
        direction3_letter.append(['up',direction3[a][1],direction3[a][2]%2])
    if direction3[a][0]==1:
        direction3_letter.append(['up left',direction3[a][1],direction3[a][2]%2])
    if direction3[a][0]==2:
        direction3_letter.append(['left',direction3[a][1],direction3[a][2]%2])
    if direction3[a][0]==3:
        direction3_letter.append(['down left',direction3[a][1],direction3[a][2]%2])
    if direction3[a][0]==4:
        direction3_letter.append(['down',direction3[a][1],direction3[a][2]%2])
    if direction3[a][0]==-3:
        direction3_letter.append(['down right',direction3[a][1],direction3[a][2]%2])
    if direction3[a][0]==-2:
        direction3_letter.append(['right',direction3[a][1],direction3[a][2]%2])
    if direction3[a][0]==-1:
        direction3_letter.append(['up right',direction3[a][1],direction3[a][2]%2])

print direction3_letter
# print direction3

copy_img = numpy.zeros((len(nmap[1]),len(nmap[1][3]),3),numpy.float64)

for y in range(len(nmap[1])):
    for x in range(len(nmap[1][3])):
        if nmap[1][y,x]==0:
            copy_img[y,x]=[255,255,255]
        elif nmap[1][y,x]==1:
            copy_img[y,x]=[0,0,0]

for k in sequance:
    copy_img[k[0],k[1]]=[0,0,255]

cv2.imshow('Astar_with_buffer',copy_img)
cv2.waitKey(0)