from pyamaze import maze,agent,COLOR,textLabel
from queue import PriorityQueue
import random
import math
def h(cell1, cell2):
    x1, y1 = cell1
    x2, y2 = cell2
    return (abs(x1 - x2) + abs(y1 - y2))
def h2(cell1, cell2):
    x1, y1 = cell1
    x2, y2 = cell2
    return ((abs(x1 - x2) + abs(y1 - y2))+math.sqrt((x1-x2)**2+(y1-y2)**2))/2
    
def aStar(m,goal,start=None):
    if start is None:
        start=(m.rows,m.cols)
    open = PriorityQueue()
    open.put((h(start, goal), h(start, goal), start))
    aPath = {}
    g_score = {row: float("inf") for row in m.grid}
    g_score[start] = 0
    f_score = {row: float("inf") for row in m.grid}
    f_score[start] = h(start, goal)
    searchPath=[start]
   
    while not open.empty() :
        currCell = open.get()[2]
        searchPath.append(currCell)
        if currCell == goal:
            break        
        for d in 'ESNW':
            if m.maze_map[currCell][d]==True:
                if d=='E':
                    childCell=(currCell[0],currCell[1]+1)
                elif d=='W':
                    childCell=(currCell[0],currCell[1]-1)
                elif d=='N':
                    childCell=(currCell[0]-1,currCell[1])
                elif d=='S':
                    childCell=(currCell[0]+1,currCell[1])

                temp_g_score = g_score[currCell] + 1
                temp_f_score = temp_g_score + h(childCell, goal)

                if temp_f_score < f_score[childCell]:   
                    aPath[childCell] = currCell
                    g_score[childCell] = temp_g_score
                    f_score[childCell] = temp_g_score + h(childCell, goal)
                    open.put((f_score[childCell], h(childCell, goal), childCell))
            


    fwdPath={}
    cell=goal
    if cell==(1,1):
        while cell!=start:
            fwdPath[aPath[cell]]=cell
            cell=aPath[cell]
        return fwdPath,True
    else:
        while cell!=start:
            fwdPath[aPath[cell]]=cell
            cell=aPath[cell]
        return fwdPath,False

def aStar2(m,goal,start=None):
    if start is None:
        start=(m.rows,m.cols)
    open = PriorityQueue()
    open.put((h2(start, goal), h2(start, goal), start))
    aPath = {}
    g_score = {row: float("inf") for row in m.grid}
    g_score[start] = 0
    f_score = {row: float("inf") for row in m.grid}
    f_score[start] = h2(start, goal)
    searchPath=[start]
   
    while not open.empty() :
        currCell = open.get()[2]
        searchPath.append(currCell)
        if currCell == goal:
            break        
        for d in 'ESNW':
            if m.maze_map[currCell][d]==True:
                if d=='E':
                    childCell=(currCell[0],currCell[1]+1)
                elif d=='W':
                    childCell=(currCell[0],currCell[1]-1)
                elif d=='N':
                    childCell=(currCell[0]-1,currCell[1])
                elif d=='S':
                    childCell=(currCell[0]+1,currCell[1])

                temp_g_score = g_score[currCell] + 1
                temp_f_score = temp_g_score + h2(childCell, goal)

                if temp_f_score < f_score[childCell]:   
                    aPath[childCell] = currCell
                    g_score[childCell] = temp_g_score
                    f_score[childCell] = temp_g_score + h2(childCell, goal)
                    open.put((f_score[childCell], h2(childCell, goal), childCell))
            


    fwdPath={}
    cell=goal
    if cell==(1,1):
        while cell!=start:
            fwdPath[aPath[cell]]=cell
            cell=aPath[cell]
        return fwdPath,True
    else:
        while cell!=start:
            fwdPath[aPath[cell]]=cell
            cell=aPath[cell]
        return fwdPath,False

def stochastic_A_star(m):
    #random points
    check_points=[(random.randint(1,m.rows),random.randint(1,m.cols))for _ in range(1,40)]
    check_points.append((1,1))
    start=(m.rows,m.cols)
    temp_goal_set=[(i,h2(i,start)) for i in check_points]
    road=[]
    
    flag=False
    
    while  len(temp_goal_set) and flag!=True>0:
        fwdpath={}
        start=(m.rows,m.cols)
        temp_goal_set.sort(key=lambda x:x[1])
        z=temp_goal_set.pop(0)
        z=z[0]
        path,flag =aStar2(m,goal=z,start=start)
        fwdpath.update(path)
        start=z
        path,flag=aStar2(m,goal=(1,1),start=start)
        fwdpath.update(path)
        road.append([fwdpath,len(fwdpath)])
    road.sort(key=lambda x:x[1])
    return road[0][0]
        

    


if __name__=='__main__':
 


    # myMaze=maze(20,20)
    # myMaze.CreateMaze(loopPercent=80,theme='blue',saveMaze=True)
    # myMaze.run()
    savemaze=maze(20,20)
    savemaze.CreateMaze(theme='blue',loadMaze='/Users/rudra_sarkar/Documents/jupyter practice projects/maze--2023-01-14--00-30-10.csv')
    # savemaze.CreateMaze(loopPercent=400,theme='blue')
    goal=(1,1)
    fwdPath=stochastic_A_star(savemaze)
    print(fwdPath.get(goal))
    print(fwdPath)
    # path2,_=aStar(savemaze,start=(20,20),goal=(1,1))
 

   

    
    a=agent(savemaze,20,20,footprints=True,color=COLOR.yellow,filled=True,shape='arrow')
    # b=agent(savemaze,20,20,footprints=True,color=COLOR.green,filled=True)
    
    savemaze.tracePath({a:fwdPath},delay=100)
    # savemaze.tracePath({b:path2},delay=100)
 

    l=textLabel(savemaze,'A Star Stoch Path Length',len(fwdPath)+1)
    # l=textLabel(savemaze,'A Star Path Length',len(path2)+1)
   
    

    savemaze.run()
    
  





