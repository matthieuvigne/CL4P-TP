# Claptrap class
# This class runs a simulation of Claptrap using pinocchio, given a controller as input.

# A controller is defined by the following signature:
# def controller(t, q, v, q_ref, v_ref, a_ref):
#   return tau


import pinocchio as pnc
import meshcat
import numpy as np
import scipy.integrate

# Suffix used when logging the state
CLAPTRAP_STATE_SUFFIXES = ["roll", "pitch", "yaw"] + \
                          ["baseX", "baseY", "baseZ"] + \
                          ["omegaX", "omegaY", "omegaZ"] + \
                          ["wheelVelocity", "energy", "wheelTorque"]

def skew_symmetric(v):
    return np.matrix([[0, -v[2,0], v[1,0]], [v[2,0], 0, -v[0, 0]], [-v[1, 0], v[0, 0], 0]])

class Claptrap():
    def __init__(self, controller, initial_state = None, meshcat_viewer = None, meshcat_name = "claptrap", robot_color = None):
        '''
            Init the robot object.
            @param controller Controller callback.
            @param initial_state Optional, initial state as tuple (q0, v0) to start the simulation (position and velocity)
            @param meshcat_viewer Optional, meshcat viewer. If set, this viewer can be used to display the robots during simulation.
            @param meshcat_name Optional, if meshcat_viewer is set, name of the robot in the viewer.
            @param robot_color Optional, if meshcat_viewer is set, color of the robot in the viewer.
        '''
        self.meshcat_viewer = meshcat_viewer 
        self.controller = controller
        
        # TODO: fix import of URDF !
        import os
        import claptrap_simu
        urdf_path = os.path.join(os.path.dirname(claptrap_simu.__file__),'../data/claptrap.urdf')
        
        self.robot = pnc.RobotWrapper.BuildFromURDF("data/claptrap.urdf", ["data/"], root_joint=None)
        
        # Compute wheel radius vector.
        self.wheel_radius = self.robot.model.jointPlacements[self.robot.model.getJointId("BodyJoint")].translation[2, 0]
        
        # Current robot state
        if initial_state is not None:
            self.q = initial_state[0]
            self.v = initial_state[1]
        else:
            self.q = self.robot.q0
            self.v = np.matrix(np.zeros((self.robot.nv, 1)))
            
        # Initialize viewer if needed.
        if self.meshcat_viewer is not None:
            self.robot.initMeshcatDisplay(self.meshcat_viewer, meshcat_name, robot_color)
            self.robot.display(self.robot.q0)
        
        # Create integrator.
        self.solver = scipy.integrate.ode(self.dynamics)
        self.solver.set_integrator('dopri5')
        self.solver.set_initial_value(np.concatenate((self.q, self.v)))
    
    
    def integrate(self, t):
        ''' Integrate simulation up to time t'''
        # Run integration
        self.solver.integrate(t)
        # Extract data from solver
        self.q = np.matrix(self.solver.y[:self.robot.nq])
        # Renormalize quaternion to prevent propagation of rounding errors due to integration.
        # ~ self.q[3:7, 0] /= np.linalg.norm(self.q[3:7, 0])
        self.v = np.matrix(self.solver.y[self.robot.nq:])
        self.tau = self.controller(t, self.q, self.v, None, None, None)
        return self.solver.successful()
    
    
    def dynamics(self, t, x):
        ''' Forward dynamics of the robot, to integrate
        '''
        # Split input as (q, v) pair
        q = np.matrix(x[:self.robot.nq]).T
        v = np.matrix(x[self.robot.nq:]).T
        # Renormalize quaternion to prevent propagation of rounding errors due to integration.
        # ~ q[3:7, 0] /= np.linalg.norm(q[3:7, 0])
        
        # Run forward dynamic computation
        # Compute H and g + coloriolis effects
        H =  self.robot.mass(q)
        g = self.robot.nle(q, v)
        
        # Compute contact jacobian and derivative (drift).
        # Since q = (x y gamma beta alpha theta) where (alpha beta gamma) are RPY angles of the base,
        # the contact implies that in the YawLink frame (after rotation by gamma), we have vx = R (dalpha + dtheta)
        # and vy = 0
        gamma = q[2, 0]
        J = np.matrix([[np.cos(gamma), np.sin(gamma), 0, 0, -self.wheel_radius, -self.wheel_radius],
                       [-np.sin(gamma), np.cos(gamma), 0, 0, 0, 0]])
        dJ = np.matrix([[-np.sin(gamma), np.cos(gamma), 0, 0, 0, 0],
                       [-np.cos(gamma), -np.sin(gamma), 0, 0, 0, 0]]) * v[2, 0]
        drift = -  dJ * v
        
        # Compute controller torque
        torque = np.zeros((self.robot.model.nv, 1))

        torque[5, 0] = self.controller(t, q, v, None, None, None)
        # Write full equation
        A = np.block([[H, J.T], [J, np.zeros((2, 2))]]) 
        b = np.concatenate((torque - g, drift))
        # Find dv, we don't care about the contact forces for now.
        dv = np.linalg.solve(A, b)[:-2]
        
        return np.concatenate((v, dv))
    
    def log_state(self, logger, prefix):
        '''Log current state: the values logged are defined in CLAPTRAP_STATE_SUFFIXES
        @param logger Logger object
        @param prefix Prefix to add before each suffix.
        '''
        logger.set(prefix + "roll", self.q[3, 0])
        logger.set(prefix + "pitch", self.q[4,0])
        logger.set(prefix + "yaw", self.q[2, 0])
        # TODO
        # ~ logger.set_vector(prefix + "omega", self.v[3:6, 0])
        logger.set(prefix + "wheelVelocity", self.v[self.robot.model.joints[self.robot.model.getJointId("WheelJoint")].idx_v, 0])
        
        pnc.computeAllTerms(self.robot.model, self.robot.data, self.q, self.v)
        energy = self.robot.data.kinetic_energy + self.robot.data.potential_energy
        logger.set(prefix + "energy", energy)
        
        logger.set(prefix + "wheelTorque", self.tau[0, 0])
        
        w_M_base = self.robot.framePosition(self.q, self.robot.model.getFrameId("Body"), False)
        logger.set_vector(prefix + "base", w_M_base.translation)
        
