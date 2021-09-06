#!/usr/bin/env python
import pygame
import random
import numpy as np


pygame.init()

## parameters can be changed (vehicle and obstacle)
start_x = 0
start_y = 0
end_x = 800
end_y = 600
radius_vehicle = 10
radius_obstacle = 15
velocity_vehicle = 1
velocity_obstacle = 1
no_of_action_space = 50
time = 1
time_future = 0.5
safety_margin = 5
number_obstacles = 25

## parameters of pygame
width = 800
height = 600
screen = pygame.display.set_mode((width,height))

# class to control the vehicle 
class vehicle:
    def __init__(self):
        self.set_start_end()
        self.curr_x = self.start_x
        self.curr_y = self.start_y
        self.future_x = 0
        self.future_y = 0
        self.angle = 0
        self.reach_target = False

    # sets the start and end position of the vehicle
    def set_start_end(self):
        self.start_x = start_x
        self.start_y = start_y
        self.end_x = end_x
        self.end_y = end_y

    # all the future values of the vehicle are calculated
    def future_pos_veh(self,vel,time):
        if (self.goal_reached(self.curr_x,self.curr_y,self.end_x,self.end_y)):
            self.reach_target = True

        if not self.reach_target:
            self.angle = np.linspace(0,2*np.pi,no_of_action_space)
            self.future_x = self.curr_x + vel*time*np.cos(self.angle)
            self.future_y = self.curr_y + vel*time*np.sin(self.angle)
            
        if self.reach_target:
            print('TARGET REACHED')

    # function to check if goal is reached (the min distance to reach is 6, lower than this the target misses)
    def goal_reached(self,curr_x,curr_y,end_x,end_y):
        goal_reached = False
        if np.sqrt((curr_x - end_x)**2 + (curr_y - end_y)**2)<6:
            goal_reached = True
        return goal_reached        

# class to control the obstacles
class obstacle:
    def __init__(self):
        self.x = random.randint(0,width)
        self.y = random.randint(0,height)
        self.angle_rand = random.randint(0,360)
        self.x_boundary = width
        self.y_boundary = height
        self.x_curr = self.x
        self.y_curr = self.y

    # calculates the current position of the obstacle
    def current_cord(self,vel,time):
        bounce = False
        if self.x_curr>self.x_boundary:
            self.x_curr = self.x_boundary
            bounce = True
        elif self.x_curr<0:
            self.x_curr = 0
            bounce = True
        elif self.y_curr>self.y_boundary:
            self.y_curr = self.y_boundary
            bounce = True
        elif self.y_curr<0:
            self.y_curr = 0    
            bounce = True
        
        if (bounce):
            self.angle_rand = random.randint(0,360)
       
        self.x_curr = self.x_curr + vel*time*np.cos(np.radians(self.angle_rand))
        self.y_curr = self.y_curr + vel*time*np.sin(np.radians(self.angle_rand))
   
    # calculates the future position of the obstacle    
    def future_pos_obs(self,vel,time):
        x_fut_pos = self.x_curr + vel*time*np.cos(np.radians(self.angle_rand))
        y_fut_pos = self.y_curr + vel*time*np.sin(np.radians(self.angle_rand))
        
        return x_fut_pos,y_fut_pos
  
# function which is uses pygame to draw circle for obstacles and vehicle       
def draw_circle(x,y,color,radius):
    pygame.draw.circle(screen,color,(x,y),radius)

# function calculates the distance between the chosen path and the obstacle
def distance(veh_x,veh_y,obs_x,obs_y):
    return np.sqrt((veh_x - obs_x)**2 + (veh_y - obs_y)**2)

# function return the distance between the chosen point and the target
def distance_goal(point,veh):
    return np.sqrt((point[0] - veh.end_x)**2 + (point[1] - veh.end_y)**2)

# Function to find the path with least cost, the cost is estimated by the euclidean distance between target and current position  
def check_paths(veh1,obs_future_pos_x,obs_future_pos_y):
    veh1.future_pos_veh(velocity_vehicle,time+time_future)
    path = main_check_loop(veh1.future_x,veh1.future_y,obs_future_pos_x,obs_future_pos_y)
    goal_error = np.argmin([distance_goal(point,veh1) for point in path])
    veh1.curr_x = path[goal_error][0]
    veh1.curr_y = path[goal_error][1]
    return None      

# Function based on the vehicle obstacle but using positions instead of velocities (modified)   
# it checks if the path is obstacle free or not
def main_check_loop(vehicle_list_x,vehicle_list_y,obstacle_list_x,obstacle_list_y):
    free_path = []
    for veh in range(len(vehicle_list_x)):
        path = True
        for obs in range(len(obstacle_list_x)):
            error = distance(vehicle_list_x[veh],vehicle_list_y[veh],obstacle_list_x[obs],obstacle_list_y[obs])
            if error < radius_obstacle + radius_vehicle + safety_margin:
                path = False

        if not path:
            continue

        free_path.append([vehicle_list_x[veh],vehicle_list_y[veh]])

    return free_path


# main pygame loop which calls the obstacles and the vehicle
# the obstacle loop can be optimized but idk how!
def main():
    running = True
    obstacle_list = [obstacle() for i in range(number_obstacles)]
    veh1 = vehicle()
    
    while running:
        obs_future_pos_x = []
        obs_future_pos_y = []

        screen.fill((255,255,255))
        
        for event in pygame.event.get():
            if event == pygame.QUIT:
                running = False

     
        for obs in obstacle_list:
            obs.current_cord(velocity_obstacle,time) 
            draw_circle(obs.x_curr,obs.y_curr,'red',radius_obstacle)
            x_temp,y_temp = obs.future_pos_obs(velocity_obstacle,time+time_future)
            obs_future_pos_x.append(x_temp)
            obs_future_pos_y.append(y_temp)
          

        check_paths(veh1,obs_future_pos_x,obs_future_pos_y)

        draw_circle(veh1.curr_x,veh1.curr_y,'blue',radius_vehicle)

        if veh1.reach_target:
            break

        pygame.display.update()

       
        

if __name__ == '__main__':
    # try:
    #     print("STARTED")
    #     main()
    # except:
    #     print("OOPS! Error with the program!")
    main()