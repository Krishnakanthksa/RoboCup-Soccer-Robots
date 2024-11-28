import math
from SimControl.Model.transform_cords import *

class RobotMovement():
    
    @classmethod
    def velocity_to_target(cls,robot_pos: tuple[float, float, float],
                           target: tuple[float,float], 
                           turning_target:tuple[float, float] = None 
                           , stop_threshold = 150) -> tuple[float, float, float]: 
        '''
        Gets the velocity required for the robot go to position and trun to target
        '''
        if robot_pos is None:
            print("Robot pos is none")
            pass
        
        transTarget = world2robot(robot_pos, target)
        vx, vy = cls.go_To_Target(transTarget, stop_threshold = stop_threshold)
        if turning_target is None:
            w = 0
        else:
            transTurningTarget = world2robot(robot_pos, turning_target)
            w = cls.turn_to_target(transTurningTarget)
            
        return vx, vy, w
    
    @classmethod
    def goShootVelcoity(cls, robot_pos:tuple[float, float,float], target: tuple[float, float]):
        shooting_position = cls.shooting_pos(target)
        vx, vy, w = cls.velocity_to_target(robot_pos, shooting_position, target)
        return vx, vy, w
    
    @staticmethod
    def turn_to_target(target:tuple[float,float] =None, epsilon: float=0.15, speed: float = 0.1, robotOmega = None):
        if target is None :
            omega = -speed*np.sign(robotOmega)
            return omega
        orientation_to_ball = np.arctan2(target[0], target[1])-np.pi/2

        if abs(orientation_to_ball) < epsilon:
            # to avoid jitter
            omega = 0
        elif abs(orientation_to_ball) > epsilon and abs(orientation_to_ball) < 4*epsilon:
            omega = -speed*np.sign(orientation_to_ball) * 0.5
        else:
            omega = -speed*np.sign(orientation_to_ball)
        return omega 
    
    @staticmethod
    def go_To_Target(target_pos: tuple[float,float], speed: int=1, stop_threshold:float=150):
        if target_pos is None:
            return 0,0
        distance = math.sqrt(target_pos[0]**2 + target_pos[1]**2)

        # print(distance)
        if distance > stop_threshold:
            vy:float = (target_pos[1] / distance) * speed
            vx:float = (target_pos[0] / distance) * speed
            return vx,vy
        else: 
            return 0,0

    @staticmethod
    def shooting_pos(ball_pos:tuple[float,float],shootingTarget: tuple[float,float], robot_offset = 500):
        # Calculate direction vector from ball to target
        direction = np.array(shootingTarget) - np.array(ball_pos)
        
        # Normalize direction vector
        direction = direction.astype(float)  # Ensure direction vector is float
        direction = np.linalg.norm(direction)
        
        # Calculate robot position slightly behind the ball
        robot_position = np.array(ball_pos) - robot_offset * direction
        
        return robot_position
    
    
   

    @staticmethod
    def calculate_target_position(target, ball, robot_offset):
        '''
            This function returns the target position for a robot. It needs this
            to aim and shoot a ball.
        '''
        # Calculate direction vector from ball to target
        direction = np.array(target) - np.array(ball)
        
        # Normalize direction vector
        direction = direction.astype(float)  # Ensure direction vector is float
        direction = np.linalg.norm(direction)
        
        # Calculate robot position slightly behind the ball
        robot_position = np.array(ball) - robot_offset * direction
        
        return robot_position
    


if __name__ == "__main__":
        
    from TeamControl.Model.world import World as wm
    from TeamControl.Network.Receiver import grSimVision,vision
    from TeamControl.Network.Sender import grSimSender,robotSender
    ...
