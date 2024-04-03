import numpy as np


# find the direction of the line given starting and ending points
def FindDirection(starting_x, starting_y, ending_x, ending_y):
    start = np.array([starting_x, starting_y])
    end = np.array([ending_x, ending_y])
    d = end - start
    return d / np.linalg.norm(d)


# use linked list as the node data structure.
class TreeNode:
    def __init__(self, pos_x, pos_y, parent=-1):
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.children = list()  # initialized the children nodes as an empty list
        self.parent = parent  # invalid idx

    def GetDistancePoint(self, other_x, other_y):
        return np.sqrt((self.pos_x - other_x)**2 + (self.pos_y - other_y)**2)

    def GetDistanceNode(self, otherNode):
        return self.GetDistancePoint(otherNode.pos_x, otherNode.pos_y)


# for simplicity, use square map
class SquareMap:
    def __init__(self, bottomleft_x, bottomleft_y, topRight_x, topRight_y):
        self.bottomleft_x = bottomleft_x
        self.bottomleft_y = bottomleft_y
        self.topRight_x = topRight_x
        self.topRight_y = topRight_y

    def MakeSample(self):
        x = np.random.uniform(self.bottomleft_x, self.topRight_x)
        y = np.random.uniform(self.bottomleft_y, self.topRight_y)
        return np.array([x, y])


# use circle to define obstacles
class Obstacles:
    def __init__(self, center_x, center_y, radius):
        self.center_x = center_x
        self.center_y = center_y
        self.radius = radius

    def DetectLineCollision(self, starting_x, starting_y, ending_x, ending_y, tolerance):
        n0 = FindDirection(starting_x, starting_y, ending_x, ending_y)
        p = np.array([self.center_x - starting_x, self.center_y - starting_y])
        dist = p - np.inner(p, n0) * n0
        return np.linalg.norm(dist) <= self.radius + tolerance

    def DetectPointCollision(self, pos_x, pos_y, tolerance):
        return np.sqrt((self.center_x - pos_x)**2 + (self.center_y - pos_y)**2) <= self.radius + tolerance


def GenerateMapBorder(squareMap):
    x = [squareMap.bottomleft_x, squareMap.topRight_x, squareMap.topRight_x, squareMap.bottomleft_x, squareMap.bottomleft_x]
    y = [squareMap.bottomleft_y, squareMap.bottomleft_y, squareMap.topRight_y, squareMap.topRight_y, squareMap.bottomleft_y]
    return x, y

def GenerateCircles(obstacle, n=30):
    theta = np.linspace(0, 2*np.pi, n)
    x = list()
    y = list()
    for t in theta:
        x.append(obstacle.center_x + np.cos(t)*obstacle.radius)
        y.append(obstacle.center_y + np.sin(t)*obstacle.radius)
    return x, y
