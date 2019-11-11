"""
CBF-RRT:

Guang Yang
"""

import matplotlib.pyplot as plt
import numpy as np
import random
import math
import copy
import time
from simulation_closedform_linsys import CBF_RRT

show_animation = True


class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList,
                 randArea, expandDis=0.45, goalSampleRate=5,u_ref_nominal=2.0):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Ramdom Samping Area [min,max]
        """
        self.start = Node(start[0], start[1])
        self.goal = goal
        self.end = Node(goal[0], goal[1])
        self.minrand = randArea[0]
        self.maxrand = randArea[1]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.obstacleList = obstacleList
        self.u_ref_nominal = u_ref_nominal

    def Planning(self, animation=False):
        counter = 0
        """
        Pathplanning

        animation: flag for animation on or off
        """

        self.nodeList = [self.start]
        while True:

            random_index = random.randint(0,len(self.nodeList)-1)

            # expand tree
            startNode = self.nodeList[random_index]
            desired_theta = math.atan2(self.goal[1] - startNode.y, self.goal[0] - startNode.x)

            sampled_theta = random.gauss(desired_theta, 0.2)

            # Update u_ref based on goal direction
            u_ref = [self.u_ref_nominal*math.cos(sampled_theta),self.u_ref_nominal*math.sin(sampled_theta)]

            #print(startNode.x,startNode.y)

            #Simulate Trajectory using CBF controller instead of Fixed length step Increment
            try:
                #planning_obj = Safe_Traj([startNode.x, startNode.y, startNode.theta], self.obstacleList)
                #x_simulated = planning_obj.integrate()
                #initial_state = np.array([[1.0],[1.0]])
                #obstacle_list = [[2.9,2.6,0.5]]
                cbf_rrt_simulation = CBF_RRT(np.array([[startNode.x],[startNode.y]]), self.obstacleList)
                x_simulated, u_simulated= cbf_rrt_simulation.motion_planning(u_ref)
                feasible = True


            except:
                feasible = False

            if feasible == True:
                newNode = copy.deepcopy(startNode) #initialize node structure
                newNode.xt = x_simulated[0][0:]
                newNode.yt = x_simulated[1][0:]

                newNode.parent = random_index
                newNode.x = x_simulated[0][-1]
                newNode.y = x_simulated[1][-1]

                newNode.t = 0 #needs to be parent node time + sim time\


                self.nodeList.append(newNode)


                # check goal
                dx = newNode.x - self.end.x
                dy = newNode.y - self.end.y
                d = math.sqrt(dx * dx + dy * dy)
                if d <= self.expandDis:
                    print("Goal!!")

                    if animation:
                        self.DrawGraph()

                    #path_x = [self.end.x]
                    #path_y = [self.end.y]


                    path_x = []
                    path_y = []


                    lastIndex = len(self.nodeList)-1
                    while self.nodeList[lastIndex].parent is not None:
                        node = self.nodeList[lastIndex]
                        #path_x.extend(node.xt)
                        #path_y.extend(node.yt)
                        #lastIndex = node.parent

                        path_x[:0]=node.xt
                        path_y[:0]=node.yt
                        lastIndex = node.parent


                    return path_x, path_y



    def DrawGraph(self):  # pragma: no cover
        """
        Draw Graph
        """

        plt.clf()

        #Plot added new Path
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot(node.xt, node.yt, "-g")

        #Plot Obstacles
        for (x, y, size) in self.obstacleList:
            self.PlotCircle(x, y, size)


        plt.plot(self.start.x, self.start.y, "xb",markersize=15,label="Initial State")
        plt.plot(self.end.x, self.end.y, "^r",markersize=15, label="Goal State")
        plt.xlabel("$x_1$",fontsize=20)
        plt.ylabel("$x_2$",fontsize=20)
        plt.title("CBF RRT Path",fontsize=25)
        plt.xticks(size = 20)
        plt.yticks(size = 20)
        plt.axis([-1.5, 8, -1.5, 8])
        plt.grid(True)
        plt.legend( loc='top left', borderaxespad=0.)
        plt.pause(0.5)
        plt.rcParams.update({'font.size': 30})

    def PlotCircle(self, x, y, size):
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(math.radians(d)) for d in deg]
        yl = [y + size * math.sin(math.radians(d)) for d in deg]
        plt.plot(xl, yl, "-r",markersize=100)


class Node():
    """
    RRT Node
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


def main():
    print("start " + __file__)
    show_animation = True
    # ====Search Path with RRT====
    obstacleList = [[-0.0,1.0,0.50],[1.0,0.5,0.50],[3,3,0.5],[3,2.0,0.5],[2,5,0.5]]  # [x,y,size]
    final_goal = [5.0,5.0]
    # Set Initial parameters
    rrt = RRT(start=[-0.5, -0.5], goal=final_goal,
              randArea=[-2, 15], obstacleList=obstacleList)
    path_x, path_y = rrt.Planning(animation=False)
    # Draw final path
    if show_animation:  
        rrt.DrawGraph()
        plt.plot(path_x, path_y, 'b-', ms=12,label='Path')
        plt.grid(True)
        plt.show()

if __name__ == '__main__':
    start_time = time.time()
    main()
    end_time = time.time()
    print("Total runtime:",end_time-start_time)
