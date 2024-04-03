import planner as pl
import matplotlib.pyplot as plt


if __name__ == "__main__":
    squareMap = pl.SquareMap(0, 0, 100, 50)
    # starting node
    start = pl.TreeNode(10, 10)
    goal = pl.TreeNode(73, 40)
    maxItr = 10000
    stepSize = 3
    rewireRadius = 15
    goalTolerance = 5
    collisionTolerance = 1
    planner = pl.RRTStarPlanner(squareMap, start, goal, maxItr, stepSize, rewireRadius, goalTolerance , collisionTolerance)
    planner.AddObstacles(pl.Obstacles(40, 18, 10))
    planner.AddObstacles(pl.Obstacles(60, 38, 5))
    x_m, y_m = pl.GenerateMapBorder(squareMap)
    ob_coordinate = list()
    for ob in planner.obstacleList:
        x, y = pl.GenerateCircles(ob)
        ob_coordinate.append([x, y])
    
    plt.ion()  # turning interactive mode on
    fig, ax = plt.subplots()
    ax.axis('equal')
    ax.plot(x_m, y_m, color = 'k', linewidth=2.0)
    # calling pause function in interactive mode to wait for 
    # drawing to complete
    for X in ob_coordinate:
        ax.plot(X[0], X[1], color = 'r', linewidth=1.0)
        fig.canvas.draw()
        fig.canvas.flush_events()

    # start location
    ax.scatter(planner.start_pos.pos_x, planner.start_pos.pos_y, c='g')
    # end location
    ax.scatter(planner.goal_pos.pos_x, planner.goal_pos.pos_y, c='b')
    fig.canvas.draw()
    fig.canvas.flush_events()

    for _ in range(1000):
        if planner.UpdateOneStep():
            parentIdx = planner.nodeList[-1].parent
            x = [planner.nodeList[-1].pos_x, planner.nodeList[parentIdx].pos_x]
            y = [planner.nodeList[-1].pos_y, planner.nodeList[parentIdx].pos_y]
            ax.plot(x, y, color = 'b', linewidth=0.5)
            fig.canvas.draw()
            fig.canvas.flush_events()
            plt.pause(0.05)
            if planner.CheckGoal(planner.nodeList[-1]):
                planner.pathFound = True
                parentIdx = len(planner.nodeList) - 1
                planner.nodeList.append(planner.goal_pos)
                planner.nodeList[-1].parent = parentIdx
                break

    print(len(planner.nodeList))
    # turn off interactive mode and keep figure open
    plt.ioff()
    if planner.pathFound:
        # plot the path
        res = planner.FormPath()
        print(res)
        x = list()
        y = list()
        for nodeIdx in res:
            x.append(planner.nodeList[nodeIdx].pos_x)
            y.append(planner.nodeList[nodeIdx].pos_y)
        ax.plot(x, y, color = 'r', linewidth=1)
        fig.canvas.draw()
        fig.canvas.flush_events()

    else:
        print("can not find path!")

    plt.show()