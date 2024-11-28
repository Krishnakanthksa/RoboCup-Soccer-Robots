import time
from SimControl.Coms.Action import Action
from SimControl.Network.Receiver import *
from SimControl.Network.Sender import *
from SimControl.Model.world import World as wm
from SimControl.Coms.grSimAction import grSim_Action
from SimControl.RobotBehaviour import *
import math
import numpy as np
from SimControl.Formation.relative_position import *

import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression



def predict_trajectory(history: list, num_samples, isPostive:bool, feild_size ):

    feild_x, feild_y =feild_size
            # PARAMETERS
    #GOALIE_LINE = -FIELD_LENGTH/2 + 200 #mm #On the other side of the field if we are the other team
    if isPostive == True:
        goal_pos = feild_x/2 - 200
    else:
        goal_pos = -feild_x/2 + 200

    goal_width = 2000
    goal_line = goal_pos # the goal line is the x coordinates of the goals line 
    num_samples = 5
    #print(ball_pos)
    ball_pos_x = []
    ball_pos_y = []
    for ball_pos in history:
        ball_pos_x.append(ball_pos[0])
        ball_pos_y.append(ball_pos[1])
    if len(ball_pos) <= num_samples:
        # use all available ball positions
        last_ball_positions_x = ball_pos_x
        last_ball_positions_y = ball_pos_y
    
        # print(ball_pos_x)
        # print(ball_pos_y)
    else:
        # use last num_samples ball positions
        last_ball_positions_x = ball_pos_x[-num_samples:]
        last_ball_positions_y = ball_pos_y[-num_samples:]
    
    # print(last_ball_positions_x)1
    # print(last_ball_positions_y)
    if True:
        model = LinearRegression()

        model.fit(np.array(last_ball_positions_x).reshape(-1, 1), last_ball_positions_y)

        # Generate trajectory points
        x_values = np.linspace(-feild_x/2, feild_x/2, 20) 

        current_ball_position_x = ball_pos_x[-1]  # Current ball position

        trajectory_y_at_goal_line = model.predict(np.array([goal_line]).reshape(-1, 1))

        intersect_line = -goal_width / 2 <= trajectory_y_at_goal_line <= goal_width/ 2

        # Calculate intersection point if exists

        golie_pos = None

        # defult goal position is the fomation golie position.
    
        golie_pos = (goal_line, round(trajectory_y_at_goal_line.item()))
       
    return golie_pos, intersect_line
    
