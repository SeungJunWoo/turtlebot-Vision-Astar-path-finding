# EUISOO A* algorithm Example code

import numpy
from heapq import *
import cv2


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


img = cv2.imread('map6.png',0)

img_buffer1 = cv2.imread('map6.png',0)
img_buffer2 = cv2.imread('map6.png',0)
img_buffer3 = cv2.imread('map6.png',0)
img_buffer4 = cv2.imread('map6.png',0)
img_buffer5 = cv2.imread('map6.png',0)
img_buffer6 = cv2.imread('map6.png',0)
img_buffer7 = cv2.imread('map6.png',0)
img_buffer8 = cv2.imread('map6.png',0)
img_buffer9 = cv2.imread('map6.png',0)
img_buffer10 = cv2.imread('map6.png',0)
img_buffer11 = cv2.imread('map6.png',0)
img_buffer12 = cv2.imread('map6.png',0)
img_buffer13 = cv2.imread('map6.png',0)
img_buffer14 = cv2.imread('map6.png',0)
img_buffer15 = cv2.imread('map6.png',0)
img_buffer16 = cv2.imread('map6.png',0)
img_buffer17 = cv2.imread('map6.png',0)
img_buffer18 = cv2.imread('map6.png',0)
img_buffer19 = cv2.imread('map6.png',0)
img_buffer20 = cv2.imread('map6.png',0)
img_buffer21 = cv2.imread('map6.png',0)
img_buffer22 = cv2.imread('map6.png',0)
img_buffer23 = cv2.imread('map6.png',0)
img_buffer24 = cv2.imread('map6.png',0)
img_buffer25 = cv2.imread('map6.png',0)
img_buffer26 = cv2.imread('map6.png',0)
img_buffer27 = cv2.imread('map6.png',0)

buffer=[img_buffer1,img_buffer2,img_buffer3,img_buffer4,img_buffer5,img_buffer6,img_buffer7,img_buffer8,img_buffer9,img_buffer10,img_buffer11,img_buffer12,img_buffer13,img_buffer14,img_buffer15,img_buffer16,img_buffer17,img_buffer18,img_buffer19,img_buffer20,img_buffer21,img_buffer22,img_buffer23,img_buffer24,img_buffer25,img_buffer26,img_buffer27]

for k in range(16):
    for y in range(1,len(img)-1):
        for x in range(1,len(img[y])-1):
            if buffer[k][y,x]==255 and (buffer[k][y,x+1]==0 or buffer[k][y+1,x+1]==0 or buffer[k][y+1,x]==0 or buffer[k][y+1,x-1]==0 or buffer[k][y,x-1]==0 or buffer[k][y-1,x-1]==0 or buffer[k][y-1,x]==0 or buffer[k][y-1,x+1]==0):
                buffer[k+1][y,x]=0


nmap=cv2.threshold(img_buffer17,200,1,cv2.THRESH_BINARY_INV)

# print len(nmap[1])
# print(astar(nmap, (0,3), (10,13)))
sequance=(astar(nmap[1], (126,96), (18,96)))
sequance.append((126,96))
sequance.reverse()

cv2.imshow('buffer',img_buffer17)
cv2.waitKey(0)

# print sequance

direction=[]

for k in range(len(sequance)-1):
    if sequance[k+1][1]-sequance[k][1]==1 and sequance[k+1][0]-sequance[k][0]==0:
        # print("right")170
        direction.append(-2)
    elif sequance[k+1][1]-sequance[k][1]==1 and sequance[k+1][0]-sequance[k][0]==-1:
        # print("up right")
        direction.append(-1)
    elif sequance[k+1][1]-sequance[k][1]==0 and sequance[k+1][0]-sequance[k][0]==-1:
        # print("up")
        direction.append(0)
    elif sequance[k+1][1]-sequance[k][1]==-1 and sequance[k+1][0]-sequance[k][0]==-1:
        # print("up left")
        direction.append(1)
    elif sequance[k+1][1]-sequance[k][1]==-1 and sequance[k+1][0]-sequance[k][0]==0:
        # print("left")
        direction.append(2)
    elif sequance[k+1][1]-sequance[k][1]==-1 and sequance[k+1][0]-sequance[k][0]==1:
        # print("down left")
        direction.append(3)
    elif sequance[k+1][1]-sequance[k][1]==0 and sequance[k+1][0]-sequance[k][0]==1:
        # print("down")
        direction.append(4)
    elif sequance[k+1][1]-sequance[k][1]==1 and sequance[k+1][0]-sequance[k][0]==1:
        # print("down right")
        direction.append(-3)

direction.append(0)
# direction.append(0)

# print direction

direction2=[]
count=1
for k in range(len(direction)-1):

    if k==len(direction)-2:
        direction2.append([direction[k],count])
    elif direction[k]==direction[k+1]:
        count=count+1
    else:
        direction2.append([direction[k],count])
        count=1
# print direction2

direction3=[]
tilt=0
direction3.append([direction2[0][0],direction2[0][1],tilt])
for i in range(1,len(direction2)):
    tilt= tilt +(direction2[i][0]-direction2[i-1][0])
    direction3.append([(direction2[i][0]-direction2[i-1][0]),direction2[i][1],tilt])

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

# cam = cv2.VideoCapture(0)
# camera_img = cam.read()

cv2.imshow('Astar with buffer',copy_img)
# cv2.imshow('camera',camera_img)
cv2.waitKey(0)

import rospy
from geometry_msgs.msg import Twist
from math import radians
from nav_msgs.msg import Odometry

class Turtle():
    def __init__(self):
        rospy.init_node('turtle', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        r = rospy.Rate(5);

        move_cmd = Twist()
        move_cmd.linear.x = 0.05185

        move_cmd45 =Twist()
        move_cmd45.linear.x = 0.05185*1.414

        turn_cmd = Twist()
        turn_cmd.linear.x = 0.05
        turn_cmd.angular.z = radians(15)

        turn_cmd2 = Twist()
        turn_cmd2.linear.x = 0.05
        turn_cmd2.angular.z = radians(-15)

  

        navi=Odometry()

        
        for steps in range(len(direction3)):
            print 'step %d'%(steps+1)
            
            if direction3[steps][0]>=0:
                for x in range(0,15*abs(direction3[steps][0])):
                    # print abs(direction3[steps][0])
                    self.cmd_vel.publish(turn_cmd)
                    # print navi.angular.z
                    r.sleep() 
                        
            else:
                for x in range(0,15*abs(direction3[steps][0])):
                    self.cmd_vel.publish(turn_cmd2)
                    # print navi.angular.z
                    r.sleep()    

            if direction3[steps][2]%2==0:
                for x in range(0,1*abs(direction3[steps][1])-15):
                    self.cmd_vel.publish(move_cmd)
                    r.sleep()
            
            else:
                for x in range(0,1*abs(direction3[steps][1])-15):
                    self.cmd_vel.publish(move_cmd45)
                    r.sleep()

    def shutdown(self):
        rospy.loginfo("Stop")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        Turtle()
    except:
        rospy.loginfo("node terminated.")