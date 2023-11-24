from astarmain import AStar
from searchutils import Plotting
from dstar import DStar
from rrtstar import RrtStar
from datetime import datetime
from rrtmain import Rrt


def astarrun(s_start,s_goal):
    print("--------A Star--------")
    start = datetime.now().timestamp()*1000
    print("start time = ",start)
    astar = AStar(s_start, s_goal, "euclidean",1)
    astar.searching()
    end = datetime.now().timestamp()*1000
    print("end time = ",end)
    print("time taken = ",end-start)
    print("----------------------")

def dstarrun( s_start,s_goal):
    print("--------D Star--------")
    start = datetime.now().timestamp()*1000
    print("start time = ",start)
    dstar = DStar(s_start, s_goal)
    dstar.run(s_start, s_goal)
    end = datetime.now().timestamp()*1000
    print("end time = ",end)
    print("time taken = ",end-start)
    print("----------------------")

def rrtstarrun(s_start,s_goal,num_iter):
    print("--------rrt Star with ",num_iter , " iterations--------")
    start = datetime.now().timestamp()*1000
    print("start time = ",start)
    rrt_star = RrtStar(s_start, s_goal, 1, 0.10, 20, num_iter,0)
    rrt_star.planning()
    end = datetime.now().timestamp()*1000
    print("end time = ",end)
    print("time taken = ",end-start)
    print("----------------------")

def rrtrun(s_start,s_goal,num_iter):
    print("--------rrt Star with ",num_iter , " iterations--------")
    start = datetime.now().timestamp()*1000
    print("start time = ",start)
    rrt = Rrt(s_start, s_goal, 0.5, 0.05, num_iter)
    path = rrt.planning()

    if not path:
        print("No Path Found!")
    
    end = datetime.now().timestamp()*1000
    print("end time = ",end)
    print("time taken = ",end-start)
    print("----------------------")




s_start = (18, 8)  # Starting node
s_goal = (37, 18) 
astarrun(s_start,s_goal)
dstarrun(s_start,s_goal)
rrtstarrun(s_start,s_goal,10)
rrtrun(s_start,s_goal,1000)
