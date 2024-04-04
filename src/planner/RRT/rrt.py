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
from .rrt_utils import *
class RRTPlanner:
    def __init__(self, aera_map, start_pos, goal_pos, maxItrs, stepSize, goalTolerance, collisionTolerance):
        self.aera_map = aera_map
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
        self.nodeList.append(TreeNode(pos_x, pos_y, parent))

    def GetSteerCoorindate(self, nodeIdx, point):
        offset = self.stepSize * FindDirection(self.nodeList[nodeIdx].pos_x, self.nodeList[nodeIdx].pos_y, point[0], point[1])
        return np.array([self.nodeList[nodeIdx].pos_x + offset[0], self.nodeList[nodeIdx].pos_y + offset[1]])

    def MakeSample(self):
        return self.aera_map.MakeSample()

    def FindNearest(self, rootIdx, point):
        if rootIdx < 0 or rootIdx >= len(self.nodeList):
            return

        dist = self.nodeList[rootIdx].GetDistancePoint(point[0], point[1])
        if dist < self.minDist:
            self.nearestNodeIdx = rootIdx
            self.minDist = dist

        for child in self.nodeList[rootIdx].children:
            self.FindNearest(child, point)

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
        for ob in self.obstacleList:
            # self.collisionTolerance
            if ob.DetectLineCollision(next[0], next[1], self.nodeList[self.nearestNodeIdx].pos_x, self.nodeList[self.nearestNodeIdx].pos_y, self.collisionTolerance):
                return False

        self.AddChild(next[0], next[1], self.nearestNodeIdx)
        self.nodeList[self.nearestNodeIdx].children.append(len(self.nodeList))
        return True

    def CheckGoal(self, currNode):
        return currNode.GetDistanceNode(self.goal_pos) < self.goalTolerance

    def Calculate(self):
        # iterate until whether the maximum itr reached
        for _ in range(self.maxItrs):
            self.UpdateOneStep()
            if self.CheckGoal(self.nodeList[-1]):
                parentIdx = len(self.nodeList) - 1
                self.pathFound = True
                self.nodeList.append(self.goal_pos)
                self.nodeList[-1].parent = parentIdx
                break

        if self.pathFound:
            print("can not find path to goal!")
        else:
            print("path found!")

    def FormPath(self):
        # check the last node
        if not self.CheckGoal(self.nodeList[-1]):
            print("not a valid end point!")
            return list()
        parentIdx =self.nodeList[-1].parent
        # add the last element
        tmp = [len(self.nodeList) - 1]
        while parentIdx >= 0:
            tmp.append(parentIdx)
            parentIdx = self.nodeList[parentIdx].parent

        # flip
        res = list()
        n = len(tmp)
        for i in range(n):
            res.append(tmp[n - i - 1])

        return res

    def FormPath(self):
        # check the last node
        if not self.CheckGoal(self.nodeList[-1]):
            print("not a valid end point!")
            return list()
        parentIdx =self.nodeList[-1].parent
        # add the last element
        tmp = [len(self.nodeList) - 1]
        while parentIdx >= 0:
            tmp.append(parentIdx)
            parentIdx = self.nodeList[parentIdx].parent

        # flip
        res = list()
        n = len(tmp)
        for i in range(n):
            res.append(tmp[n - i - 1])

        return res