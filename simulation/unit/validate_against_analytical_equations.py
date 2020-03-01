# Validate simulation dynamics on a pure sagittal motion 
# by comparing it with the analytical equation.

import numpy as np
import meshcat
import matplotlib.pyplot as plt

from claptrap_simu.core.claptrap import Claptrap, CLAPTRAP_STATE_SUFFIXES
from claptrap_simu.core.sagittal_dynamics import ClaptrapSagittalDynamics
from claptrap_simu.log_handling.logger import Logger

# PD controller parameters
Kp = 200
Kd = 0.1

def claptrap_pd(t, q, v, q_ref, v_ref, a_ref):
    pitch = q[4, 0]
    dpitch = v[4, 0]
    
    u = - Kp * (pitch + Kd * dpitch )
    return -np.matrix(u) 

def analytical_pd(t, x):
    theta = x[0]
    dtheta = x[2]
    return  - Kp * (theta + Kd * dtheta )
    
if __name__ == "__main__":
    
    # Parameters
    simulation_duration = 2.0
    # ~ simulation_duration = 0.5
    dt = 0.010
    
    # Create the viewer.
    viewer = meshcat.visualizer.Visualizer("tcp://127.0.0.1:6000")
    # Initialize claptrap object
    claptrap = Claptrap(claptrap_pd, meshcat_viewer=viewer)
    claptrap.q[4, 0] = 0.1 # Pitch
    claptrap.solver.set_initial_value(np.concatenate((claptrap.q, claptrap.v)))
    
    # Initialize analytical simulation
    x0 = np.array([0.1, 0.0, 0.0, 0.0, 0.0])
    analytical_simulator = ClaptrapSagittalDynamics()
    
    # Run simulation with claptrap object.
    time_claptrap = [0.0]
    # Save x in same format as analytical simulation
    x_claptrap = [np.array([claptrap.q[4, 0], claptrap.q[5, 0], claptrap.v[4, 0], claptrap.v[5, 0], claptrap.q[0, 0]])]
    t = dt
    while claptrap.solver.successful() and t < simulation_duration:
        claptrap.integrate(t)
        time_claptrap.append(t)
        x_claptrap.append(np.array([claptrap.q[4, 0], claptrap.q[5, 0], claptrap.v[4, 0], claptrap.v[5, 0], claptrap.q[0, 0]]))
        t += dt
    
    # Run analytical simulation
    time_analytical, x_analytical = analytical_simulator.run_nonlinear_simulation(x0, dt, simulation_duration, motor_control_law = analytical_pd)
    
    # Compare results
    assert (time_claptrap == time_analytical)
    assert np.allclose(np.array(x_claptrap), np.array(x_analytical), 1e-5)
    print("Unit test passed, claptrap simulation matches analytical equations for the sagittal motion")
