# Explicit analytical equations of the dynamics of a Segway (i.e. a simple Claptrap, with no upper body)
# in the sagittal plane.
# The non-linear version is meant to be used as simulation validation, while a linearized version is made availble
# for study and gain tuning (for instance, using LQR).
# For more information on the equations displayed here, have a look at ControlDocumentation.pdf

import numpy as np
import scipy.integrate
import pkg_resources

import pinocchio as pnc


class ClaptrapSagittalDynamics():
    '''  Exact and linearized dynamics of a Segway in the sagittal plane
    '''
    def __init__(self, urdf_path=None):
        ''' Constructor
        '''
        # Get parameters from the URDF.
        model = pnc.buildModelFromUrdf(urdf_path)
        self.g = 9.81 # Gravity
        self.m = model.inertias[model.getJointId("BodyJoint")].mass # Mass of the body
        self.mw = model.inertias[model.getJointId("WheelJoint")].mass # Mass of the wheel
        self.I =  model.inertias[model.getJointId("BodyJoint")].inertia[1,1] # Body inertia at CoM arong Y
        self.J =  model.inertias[model.getJointId("WheelJoint")].inertia[1,1] # Wheel inertia at CoM around Y

        self.l = model.inertias[model.getJointId("BodyJoint")].lever[2, 0] # Distance of CoM of body from wheel axis
        self.r = model.jointPlacements[model.getJointId("BodyJoint")].translation[2, 0] # Radius of the wheel
        
        # Define some constants that appear often in the inertia matrix
        self.mlr= self.m * self.l * self.r
        self.Mr2 = (self.m + self.mw) * self.r**2
        
        # Motor control law
        self.motor_control_law = None
    
    def compute_inertia_matrix(self, theta):
        ''' Compute inertia matrix from current base angle
        '''
        H = np.matrix([[2 * self.mlr * np.cos(theta) + self.m * self.l**2 + self.Mr2 + self.I + self.J,  self.mlr * np.cos(theta) + self.Mr2 + self.J], 
                       [self.mlr * np.cos(theta) + self.Mr2 + self.J, self.Mr2 + self.J]])
        return H
    
    def nonlinear_dynamics(self, t, x):
        ''' Nonlinear dynamics of claptrap, meant to be used by an ODE solver.
        @param t Current time
        @param x Current state, (theta, phi, dtheta, dphi, x). Theta is the base angle wrt the world, phi the relative
                 wheel displacement (i.e. encoder reading).
        @param Callback torque function
        '''
        
        theta = x[0]
        phi = x[1]
        dtheta = x[2]
        dphi = x[3]
        
        # Compute inertia matrix
        H = self.compute_inertia_matrix(theta)
        # Comptue nonlinear effects
        nle = np.matrix([[(self.m * self.l * self.g  + self.mlr * dtheta**2)* np.sin(theta)], 
                         [self.mlr * dtheta**2 * np.sin(theta)]])
        
        tau = np.zeros((2, 1))
        if self.motor_control_law is not None:
            tau[1, 0] = self.motor_control_law(t, x)
        
        # Solve for acceleration
        ddq = np.linalg.solve(H, nle - tau)
        
        dx = np.array([dtheta, dphi, ddq[0,0], ddq[1, 0], self.r * (dtheta + dphi)])
        return dx
    
    def run_nonlinear_simulation(self, x0, dt, simulation_duration, motor_control_law = None):
        ''' Run a simulation with the nonlinear equation.
        @param x0 Initial point.
        @param dt Sampling period of output.
        @param simulation_duration Duration to simulate
        @param motor_control_law Optional, callback with signature motor_control_law(t, x) -> float
        '''
        self.motor_control_law = motor_control_law

        solver = scipy.integrate.ode(self.nonlinear_dynamics)
        solver.set_integrator('dopri5')
        solver.set_initial_value(x0)
        
        time = [0.0]
        x = [x0]
        t = dt
        while solver.successful() and t < simulation_duration:
            # Integrate for dt
            solver.integrate(t)
            # Save result
            time.append(t)
            x.append(solver.y)
            t += dt
        
        return time, x
        

