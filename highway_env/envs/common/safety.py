# use optimizedDP for BRT computation for vehicle-vehicle systems 
# also have the worst case 
# * modified BRT that does not do worst case but bounds the disturbance (the action of the other vehicle by their predictive motion instead of worst case)
# practical BRT is the union of all the vehicle-vehicle BRTs for each additional vehicle in the system 
# TODO: Pedestrian-Vehicle BRT
#compute the backwards reachable tube if not computed or call the backwards reachable tube
# color the vehicles yellow if violating worst case BRT and orange if violating predictive BRT
# include a parameter that you can toggle on to record the time it takes to compute the BRT
from abc import ABC
import imp
import numpy as np
from odp.Grid import Grid
from odp.Shapes import *
import pickle
import matplotlib.pyplot as plt

# Specify the  file that includes dynamic systems
from highway_env.vehicle.hji_dynamics import HJIVehicle
# Plot options
from odp.Plots import PlotOptions
from odp.Plots import plot_isosurface, plot_valuefunction
from odp.Plots.plotting_utilities import pre_plot
from odp.compute_trajectory import spa_deriv
# Solver core
from odp.solver import HJSolver, computeSpatDerivArray
from highway_env.envs.common.abstract import AbstractEnv
import math
import os

# def define_rel_coord(ego_vehicle, other_vehicle):
#                             # sin, cos 
#     ego_vehicle_heading = np.arctan2(ego_vehicle[5], ego_vehicle[4])
#     other_vehicle_heading = np.arctan2(other_vehicle[5], other_vehicle[4])
#     ego_vehicle_speed = np.sqrt(ego_vehicle[3]**2+ego_vehicle[2]**2)
#     other_vehicle_speed = np.sqrt(other_vehicle[3]**2+other_vehicle[2]**2)
#     x_rel = ego_vehicle[0] - other_vehicle[0]
#     y_rel = ego_vehicle[1] - other_vehicle[1]
#     heading_rel = (other_vehicle_heading - ego_vehicle_heading) % 360 
#     return [x_rel, y_rel, heading_rel, ego_vehicle_speed, other_vehicle_speed]

def define_rel_coord(ego_vehicle, other_vehicle):
                            # sin, cos 
    ego_vehicle_heading = np.arctan2(ego_vehicle[5], ego_vehicle[4])
    other_vehicle_heading = np.arctan2(other_vehicle[5], other_vehicle[4])
    x_rel = other_vehicle[0] - ego_vehicle[0]
    y_rel = other_vehicle[1] - ego_vehicle[1]
    heading_rel = (other_vehicle_heading - ego_vehicle_heading) % 360 
    return [x_rel, y_rel, heading_rel]

class BRTCalculator(ABC):
    def __init__(self, step, env: AbstractEnv, obs, conservative = True) -> None:
        plotting_dict = {}

        config_boundaries = env.config["observation"]["features_range"]
        
        grid_min = np.array([config_boundaries["x"][0], 
                             config_boundaries["y"][0], 
                           -np.pi]) 
        grid_max = np.array([config_boundaries["x"][1],
                            config_boundaries["y"][1], 
                            np.pi])
        
        dims = np.array(3)

        #print(grid_min)
        N = np.array([40, 40, 40])
        pd = [2]
        g = Grid(grid_min, grid_max, dims, N, pd)
        
        self.obs_type = env.config["observation"]["type"]
        self.absolute = env.config["observation"]["absolute"]
        self.conservative = conservative
        self.last_obs = {}
        self.dt = 1/env.config["simulation_frequency"]

        # Failure set
        # the failure set should be a joint-collision shape around the other agent 
        ego_info, vehicles_info, relative_state_info = self.unpack_obs(obs)
        plotting_dict["ego_info"] = ego_info
        plotting_dict["vehicles_info"] = vehicles_info

        
        for i, rel_to_a_vehicle in enumerate(relative_state_info):
            length = 5.0
            width = 5.0
            collision_radius = 2*math.sqrt((length / 2)**2 + (width / 2)**2) # try setting a cylinder around the loation, to visualize l(x)
            l_x = CylinderShape(g, [2], rel_to_a_vehicle[:3], collision_radius) # radius in meters, vehicles are 5x2, ignoring vx, vy, heading
            plotting_dict["rel_l_x"] = (rel_to_a_vehicle, collision_radius, l_x)
            plotting_dict[f"vehicle_{i}"] = vehicles_info[i]

            lookback_length = 2.0
            t_step = 0.05

            small_delta = 1e-5
            self.sample_system = HJIVehicle(conservative)
    
            tau = np.arange(start=0, stop=lookback_length + small_delta, step=t_step)
            #po = PlotOptions(do_plot=True, plot_type="value", plotDims=[0,1],
            #          slicesCut=[19, 30])
            self.po = PlotOptions(do_plot=True, plot_type="value", plotDims=[0,1], slicesCut=[39,39,39],
                            save_fig=True, filename=f"plots/2D_4_valuefunction_step_{step}_other_agent_{i}", interactive_html=True)

            compMethods = { "TargetSetMode": "minVWithV0"}
            self.result = HJSolver(self.sample_system, g,l_x, tau, compMethods, self.po, saveAllTimeSteps=False)
            #np.save('converged_brt.npy', result)

            #result = np.load('converged_brt.npy')
            #plot_valuefunction(g, result, po)
            #plot_isosurface(g, self.result, self.po)

            #last_time_step_result = self.result[..., 0]

            #self.BRT_converged = last_time_step_result
            self.V = self.result
            self.g = g
        
        self.plot_brt(plotting_dict)


    def unpack_obs(self, obs):
        ego_info = []
        vehicles_info = []
        relative_state_info = []
        for i in range(len(obs)):
            if i == 0: 
                            # x, y, vx, vy, cos_h, sin_h 
                ego_info = [obs[i][1], obs[i][2], obs[i][3], obs[i][4], obs[i][5], obs[i][6]]
            elif obs[i][0] >= 1:
                            # x, y, vx, vy, cos_h, sin_h 
                relative_state_info.append(define_rel_coord(ego_info, [obs[i][1], obs[i][2], obs[i][3], obs[i][4], obs[i][5], obs[i][6]]))
                vehicles_info.append([obs[i][1], obs[i][2], obs[i][3], obs[i][4], obs[i][5], obs[i][6]])
        return ego_info, vehicles_info, relative_state_info
    
    

    def plot_brt(self, plotting_dict):
        ego_vehicle_x = [plotting_dict["ego_info"][0] + delta_radius for delta_radius in [-2.5, -2.5, 2.5, 2.5, -2.5]]
        ego_vehicle_y = [plotting_dict["ego_info"][1] + delta_radius for delta_radius in [-2.5, 2.5, 2.5, -2.5, -2.5]]

        # Plot the shape
        plt.plot(ego_vehicle_x, ego_vehicle_y, 'b-')  # Plot the perimeter
        plt.fill(ego_vehicle_x, ego_vehicle_y, 'lightblue')  # Fill the shape with color (optional)

        for i, vehicle in enumerate(plotting_dict["vehicles_info"]):
            oth_vehicle_x = [plotting_dict["vehicles_info"][i][0] + delta_radius for delta_radius in [-2.5, -2.5, 2.5, 2.5, -2.5]]
            oth_vehicle_y = [plotting_dict["vehicles_info"][i][1] + delta_radius for delta_radius in [-2.5, 2.5, 2.5, -2.5, -2.5]]

            plt.plot(oth_vehicle_x, oth_vehicle_y, 'r-')  # Plot the perimeter
            plt.fill(oth_vehicle_x, oth_vehicle_y, 'maroon')  # Fill the shape with color (optional)

            collision_radius = plotting_dict["rel_l_x"][1]

            # Generate points on the perimeter of the circle
            theta = np.linspace(0, 2*np.pi, 100)  # Angle values from 0 to 2*pi
            l_x = plotting_dict["vehicles_info"][i][0] + collision_radius * np.cos(theta)
            l_y = plotting_dict["vehicles_info"][i][1] + collision_radius * np.sin(theta)

            # Plot the circle
            plt.plot(l_x, l_y, 'violet')  # Plot the perimeter
            
            #spat_deriv_vector_x = computeSpatDerivArray(self.g, self.V, deriv_dim=1, accuracy="low")
            #spat_deriv_vector_y = computeSpatDerivArray(self.g, self.V, deriv_dim=2, accuracy="low")
            #spat_deriv_vector_heading = computeSpatDerivArray(self.g, self.V, deriv_dim=3, accuracy="low")

            xV = []
            yV = []
            for x in range(0, 100):
                for y in range(0, 100):
                    for angle in np.linspace(-np.pi, np.pi, 100):
                        state = (x, y, angle)
                        g_state = self.g.get_index(state)
                        #spat_deriv_vector = (spat_deriv_vector_x[g_state], spat_deriv_vector_y[g_state], spat_deriv_vector_heading[g_state])
                        #a_term = spat_deriv_vector[0] * state[1] - spat_deriv_vector[1] * state[0] - spat_deriv_vector[2]
                        #print(f"a_term = {a_term}")
                        #v_value = self.V[g_state]

                        #if v_value <= 0:
                        #    print(f"v_value <= 0, at state {state}")
                        #else:
                        #    pass
                        #print(f"self.g.get_index(state = ({x}, {y}, {angle})) = {self.g.get_index(state)}")
                        #spat_deriv_vector = spa_deriv(self.g.get_index(state), self.V, self.g, periodic_dims=[2])
                        
                        #a_term = spat_deriv_vector[0] * state[1] - spat_deriv_vector[1] * state[0] - spat_deriv_vector[2]
                        #print(f"state = {state}, v_value= {v_value}")

            xV.append(xV[0])
            yV.append(yV[0])
            plt.plot(xV, yV, 'yellow')
                 

            #plt.plot()

        # Set aspect ratio to equal
        #plt.gca().set_aspect('equal', adjustable='box')

        # Show plot
        plt.grid(True)
        #plt.show()
        plt.savefig('rectangle_plot.png')
        assert(False) 
        

    def check_safety_violation(self, obs):
        # Return 1 for violation and 0 for no violation, -1 for invalid observation type
        #at this time, support for Kinematics observation only (order to implement: OccupancyGrid, TTC, Grayscale image )
        if self.obs_type != "Kinematics":
            return -1
        
        x_derivative = computeSpatDerivArray(self.g, self.V, deriv_dim=1, accuracy="low")
        y_derivative = computeSpatDerivArray(self.g, self.V, deriv_dim=2, accuracy="low")
        heading_derivative = computeSpatDerivArray(self.g, self.V, deriv_dim=3, accuracy="low")
        v_r_derivative = computeSpatDerivArray(self.g, self.V, deriv_dim=4, accuracy="low")
        v_h_derivative = computeSpatDerivArray(self.g, self.V, deriv_dim=5, accuracy="low")

        ego_info, vehicles_info, relative_state_info = self.unpack(obs)

        safety_violation = 0 
        for i in range(len(vehicles_info)):
            if self.V[relative_state_info[i][0]][relative_state_info[i][1]][relative_state_info[i][2]][relative_state_info[i][3]][relative_state_info[i][4]] <= 0:
                # if not self.conservative and self.last_obs != {}:
                #    old_action = [self.last_obs[],]
                #else:
                safety_violation = 1
        
        return safety_violation

    def determine_safe_action(self, obs):
        # derivatives for each state
        x_derivative = computeSpatDerivArray(self.g, self.V, deriv_dim=1, accuracy="low")
        y_derivative = computeSpatDerivArray(self.g, self.V, deriv_dim=2, accuracy="low")
        heading_derivative = computeSpatDerivArray(self.g, self.V, deriv_dim=3, accuracy="low")
        v_r_derivative = computeSpatDerivArray(self.g, self.V, deriv_dim=4, accuracy="low")
        v_h_derivative = computeSpatDerivArray(self.g, self.V, deriv_dim=5, accuracy="low")

        if self.obs_type != "Kinematics":
            return -1
        
        ego_info, vehicles_info, relative_state_info = self.unpack(obs)

        opt_ctl = []
        for i in range(len(vehicles_info)):
            spat_deriv_vector = [x_derivative[tuple(relative_state_info[i])], y_derivative[tuple(relative_state_info[i])],
                            heading_derivative[tuple(relative_state_info[i])], v_r_derivative[tuple(relative_state_info[i])], v_h_derivative[tuple(relative_state_info[i])]]
            opt_ctl.append(self.sample_relative_system.optCtrl_inPython(spat_deriv_vector))

        return np.mean(opt_ctl, axis=0) 
