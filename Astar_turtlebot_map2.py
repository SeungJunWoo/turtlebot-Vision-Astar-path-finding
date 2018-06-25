# EUISOO A* algorithm Example code

import numpy
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


nmap = numpy.array([
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,0,1,1,1,1,1,1,1,1,1,1,1,1],
    [0,0,1,1,1,1,1,1,1,1,1,1,1,1],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,1,1,1,1,1,1,1,1],
    [0,0,0,0,0,0,1,1,1,1,1,1,1,1],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0]])


sequance=(astar(nmap, (0,8), (10,8)))
sequance.append((0,8))
sequance.reverse()

# print(sequance)


direction=[]

for k in range(len(sequance)-1):
    if sequance[k+1][1]-sequance[k][1]==1 and sequance[k+1][0]-sequance[k][0]==0:
        # print("right")
        direction.append(-2)
    elif sequance[k+1][1]-sequance[k][1]==1 and sequance[k+1][0]-sequance[k][0]==1:
        # print("up right")
        direction.append(-1)
    elif sequance[k+1][1]-sequance[k][1]==0 and sequance[k+1][0]-sequance[k][0]==1:
        # print("up")
        direction.append(0)
    elif sequance[k+1][1]-sequance[k][1]==-1 and sequance[k+1][0]-sequance[k][0]==1:
        # print("up left")
        direction.append(1)
    elif sequance[k+1][1]-sequance[k][1]==-1 and sequance[k+1][0]-sequance[k][0]==0:
        # print("left")
        direction.append(2)
    elif sequance[k+1][1]-sequance[k][1]==-1 and sequance[k+1][0]-sequance[k][0]==-1:
        # print("down left")
        direction.append(3)
    elif sequance[k+1][1]-sequance[k][1]==0 and sequance[k+1][0]-sequance[k][0]==-1:
        # print("down")
        direction.append(4)
    elif sequance[k+1][1]-sequance[k][1]==1 and sequance[k+1][0]-sequance[k][0]==-1:
        # print("down right")
        direction.append(-3)

direction.append(0)
direction.append(0)

#print direction
direction2=[]
count=1
for k in range(len(direction)-1):
    if direction[k]==direction[k+1]:
        count=count+1
    else:
        direction2.append([direction[k],count])
        count=1
# print direction2
# print len(direction2)

direction3=[]
direction3.append([direction2[0][0],direction2[0][1]])
for i in range(1,len(direction2)):
    direction3.append([(direction2[i][0]-direction2[i-1][0]),direction2[i][1]])

print direction3

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
        move_cmd.linear.x = 0.2

        turn_cmd = Twist()
        turn_cmd.linear.x = 0
        turn_cmd.angular.z = radians(15)

        turn_cmd2 = Twist()
        turn_cmd2.linear.x = 0
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

            for x in range(0,10*abs(direction3[i][1])):
                self.cmd_vel.publish(move_cmd)
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