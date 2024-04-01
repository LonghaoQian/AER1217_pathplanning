"""
### Copyright (C) 2024  Longhao Qian

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
"""

import numpy as np
import rrt_utils


class RRTPlanner:
    def __init__(self, map, start_pos, goal_pos, maxItrs, stepSize, goalTolerance, collisionTolerance):
        self.map = map
        self.start_pos = start_pos
        self.goal_pos = goal_pos
        self.maxItrs = maxItrs
        self.stepSize = stepSize
        self.goalTolerance = goalTolerance
        self.obstacleList = list()
        self.nodeList = [self.start_pos]
        self.pathFound = False
        self.minDist = float('inf')
        self.nearestNodeIdx = -1
        self.collisionTolerance = collisionTolerance

    def Reset(self):
        self.nodeList = [self.start_pos]
        self.pathFound = False
        self.minDist = float('inf')
        self.nearestNodeIdx = -1

    def ResetObstacles(self):
        self.obstacleList = list()

    def AddObstacles(self, obstacle):
        self.obstacleList.append(obstacle)

    def AddChild(self, pos_x, pos_y, parent):
        self.nodeList.append(rrt_utils.TreeNode(pos_x, pos_y, parent))

    def GetSteerCoorindate(self, nodeIdx, point):
        offset = self.stepSize * rrt_utils.FindDirection(self.nodeList[nodeIdx].pox_x, self.nodeList[nodeIdx].pox_y, point[0], point[1])
        return np.array([self.nodeList[nodeIdx].pox_x + offset[0], self.nodeList[nodeIdx].pox_y + offset[1]])

    def MakeSample(self):
        return map.MakeSample()

    def FindNearest(self, rootIdx, point):
        if rootIdx < 0 or rootIdx >= len(self.nodeList):
            return

        dist = self.nodeList[rootIdx].GetDistancePoint(point[0], point[1])
        if dist < self.minDist:
            self.nearestNodeIdx = rootIdx
            self.minDist = dist

        for child in self.nodeList[rootIdx]:
            self.FindNearest(child)

    def UpdateOneStep(self):
        # reset nearest value
        self.nearestDist = 1000000
        # do sample
        newPoint = self.MakeSample()
        # find the nearest node in the tree
        self.minDist = float('inf')
        self.nearestNodeIdx = -1
        self.FindNearest(0, newPoint)
        if self.nearestNodeIdx == -1:
            print("can not find nearest point!")
            return
        # do the steering
        next = self.GetSteerCoorindate(self.nearestNodeIdx, newPoint)
        # check obstacle
        noCollision = True
        for ob in self.obstacleList:
            # self.collisionTolerance
            if ob.DetectLineCollision(next[0], next[1], self.nodeList[self.nearestNodeIdx].pox_x, self.nodeList[self.nearestNodeIdx].pox_y, self.collisionTolerance):
                noCollision = False
                break

        if noCollision:
            self.AddChild(next[0], next[1], self.nearestNodeIdx)
            self.nodeList[self.nearestNodeIdx].children.append(len(self.nodeList))

    def CheckGoal(self, currNode, goalNode):
        return currNode.GetDistanceNode(goalNode) < self.goalTolerance

    def Calculate(self):
        # iterate until whether the maximum itr reached
        for _ in range(self.maxItrs):
            self.UpdateOneStep()
            if self.CheckGoal():
                self.pathFound = True
                self.nodeList.append(self.goal_pos)
                break
        
        if self.pathFound:
            print("can not find path to goal!")
        else:
            print("path found!")
