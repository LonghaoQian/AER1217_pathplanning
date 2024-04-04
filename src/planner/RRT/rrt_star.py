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
class RRTStarPlanner:
    def __init__(self, aera_map, start_pos, goal_pos, maxItrs, stepSize, rewireRadius, goalTolerance, collisionTolerance):
        self.aera_map = aera_map
        self.start_pos = start_pos
        self.goal_pos = goal_pos
        self.maxItrs = maxItrs
        self.stepSize = stepSize
        self.rewireRadius = rewireRadius
        self.goalTolerance = goalTolerance
        self.obstacleList = list()
        self.nodeList = [self.start_pos]
        self.pathFound = False
        self.minDist = float('inf')
        self.nearestNodeIdx = -1
        self.collisionTolerance = collisionTolerance
        self.neighborIdxList = list()

    def Reset(self):
        self.nodeList = [self.start_pos]
        self.pathFound = False
        self.minDist = float('inf')
        self.nearestNodeIdx = -1

    def ResetObstacles(self):
        self.obstacleList = list()

    def AddObstacles(self, obstacle):
        self.obstacleList.append(obstacle)

    def AddChild(self, pos_x, pos_y, parent, cost):
        self.nodeList.append(TreeNode(pos_x, pos_y, parent, cost))

    def GetSteerCoorindate(self, nodeIdx, point):
        offset = self.stepSize * FindDirection(self.nodeList[nodeIdx].pos_x, self.nodeList[nodeIdx].pos_y, point[0], point[1])
        return np.array([self.nodeList[nodeIdx].pos_x + offset[0], self.nodeList[nodeIdx].pos_y + offset[1]])

    def MakeSample(self):
        return self.aera_map.MakeSample()

    def CheckGoal(self, currNode):
        return currNode.GetDistanceNode(self.goal_pos) < self.goalTolerance

    def FindNearest(self, rootIdx, point):
        if rootIdx < 0 or rootIdx >= len(self.nodeList):
            return

        dist = self.nodeList[rootIdx].GetDistancePoint(point[0], point[1])
        if dist < self.minDist:
            self.nearestNodeIdx = rootIdx
            self.minDist = dist

        for child in self.nodeList[rootIdx].children:
            self.FindNearest(child, point)


    def CheckCollision(self, currPoint, node):
        # check obstacle
        for ob in self.obstacleList:
            # self.collisionTolerance
            if ob.DetectLineCollision(currPoint[0], currPoint[1], node.pos_x, node.pos_y, self.collisionTolerance):
                return False
        return True

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

    def FindNeighbors(self, currentPoint):
        # find the neighbour with lowest cost
        self.neighborIdxList.clear()
        n = len(self.nodeList)
        for i in range(n):
            dist = self.nodeList[i].GetDistancePoint(currentPoint[0], currentPoint[1])
            if dist <= self.rewireRadius and self.CheckCollision(currentPoint, self.nodeList[i]):
                # store the idx and cost
                self.neighborIdxList.append([i, dist+self.nodeList[i].cost])

    def RewireParent(self, currentPoint):
        if self.nearestNodeIdx == -1:
            print("invalid nearest point!")
            return
        # get the current cost
        currentCost = self.nodeList[self.nearestNodeIdx].cost + self.nodeList[self.nearestNodeIdx].GetDistancePoint(currentPoint[0], currentPoint[1])
        newParent = self.nearestNodeIdx
        # iterate through all neighbors to find a node with lower cost
        for idx in self.neighborIdxList:
            if idx[1] < currentCost:
                newParent = idx[0]
                currentCost = idx[1]
        return newParent, currentCost


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
        if not self.CheckCollision(next, self.nodeList[self.nearestNodeIdx]):
            return False

        # get neighbors
        self.FindNeighbors(next)
        # rewire parent
        newParent, newCost = self.RewireParent(next)
        
        # add child
        self.AddChild(next[0], next[1], newParent, newCost)
        self.nodeList[newParent].children.append(len(self.nodeList))
        return True
