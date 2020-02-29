# Run a simulation of claptrap
from logger import Logger
from claptrap import Claptrap, CLAPTRAP_STATE_SUFFIXES
import sys
import pinocchio as se3
import numpy as np
import meshcat

# Parameters
# ~ simulation_length = 20.0
# ~ simulation_length = 5.0
simulation_length = 1.0
dt = 0.010

integral = 0
old_t = 0


Kp = 200
Kd = 0.1

# Dummy controller: simpy return zero torque
def zero_torque_controller(t, q, v, q_ref, v_ref, a_ref):
    global integral, old_t
    # PD controller on base pitch
    pitch = q[4, 0]
    dpitch = v[4, 0]
    
    integral += pitch * (t - old_t)
    old_t = t
    Kp = 200
    Kd = 0.1
    u = - Kp * (pitch + Kd * dpitch )
    return -np.matrix(u) 


# Create the viewer.
viewer = meshcat.visualizer.Visualizer("tcp://127.0.0.1:6000")
# Initialize claptrap object
claptrap = Claptrap(zero_torque_controller, meshcat_viewer=viewer)
claptrap.q[4, 0] = 0.1 # Pitch
claptrap.solver.set_initial_value(np.concatenate((claptrap.q, claptrap.v)))

# Initialize logger
logged_values = ["ZeroTorque." +  s for s in CLAPTRAP_STATE_SUFFIXES] + ["Analytical" + s for s in ["theta", "dtheta", "phi", "dphi", "x", "energy"]]

logger = Logger(logged_values)


def get_H(theta):
    g = 9.81
    m = 3
    mw = 1
    l = 0.5
    r = 0.15
    I = 1.0
    J = 0.1
    H = np.matrix([[2 * m * l * r * np.cos(theta) + m * l**2 + (m + mw) * r**2 + I + J, m * r * l * np.cos(theta) + (m + mw) * r**2 + J], 
                   [m * r * l * np.cos(theta) + (m + mw) * r**2 + J, (m + mw) * r**2 + J]])
    return H

def get_energy(x):
    theta = x[0]
    phi = x[1]
    dtheta = x[2]
    dphi = x[3]
    
    g = 9.81
    m = 3
    mw = 1
    l = 0.5
    r = 0.15
    I = 1.0
    J = 0.1
    
    H = get_H(theta)
    dx = np.matrix([[dtheta, dphi]]).T
    E = 0.5 * dx.T * H * dx + m * l * g * np.cos(theta) + m * g * r   
    
    return E[0, 0]
    
def theoretical_dynamics(t, x):
    theta = x[0]
    phi = x[1]
    dtheta = x[2]
    dphi = x[3]
    
    g = 9.81
    m = 3
    mw = 1
    l = 0.5
    r = 0.15
    I = 1.0
    J = 0.1
    H = get_H(theta)
    
    nle = np.matrix([[(m * g * l + m * l * r * dtheta**2)* np.sin(theta)], [m * l * r * dtheta**2 * np.sin(theta)]])
    
    
    tau = np.matrix([[0.0, - Kp * (theta + Kd * dtheta)]]).T 
    ddq = np.linalg.solve(H, nle - tau)
    
    # ~ dx = np.array([dtheta, dphi, ddq[0,0], ddq[1, 0], -r * (dtheta + dphi)])
    dx = np.array([dtheta, dphi, ddq[0,0], ddq[1, 0], r * (dtheta + dphi)])
    return dx
    
import scipy.integrate

solver = scipy.integrate.ode(theoretical_dynamics)
solver.set_integrator('dopri5')
solver.set_initial_value(np.array([0.1, 0.0, 0.0, 0.0, 0.0]))

n_iter = 0
t = dt
success = True
E0 = None
while success and t < simulation_length:
    success = claptrap.integrate(t)
    
    # Display
    claptrap.robot.display(claptrap.q)
    
    # Log
    claptrap.log_state(logger, "ZeroTorque.")
    
    solver.integrate(t)
    logger.set("Analyticaltheta", solver.y[0])
    logger.set("Analyticalphi", solver.y[1])
    logger.set("Analyticaldtheta", solver.y[2])
    logger.set("Analyticaldphi", solver.y[3])
    logger.set("Analyticalx", solver.y[4])
    if E0 is None:
        E0 = get_energy(solver.y)
    logger.set("Analyticalenergy", get_energy(solver.y) - E0)
    
    
    logger.set("time", t)
    t += dt
    logger.new_line()
    sys.stdout.write('Running simulation: {:0.1f}/{}s\r'.format(t, simulation_length))
    sys.stdout.flush()

logger.save('/tmp/claptrap_simulation.csv')
