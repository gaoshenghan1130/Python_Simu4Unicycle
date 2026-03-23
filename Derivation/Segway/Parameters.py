class Parameters:
    def __init__(self):
        # All in SI units
        self.m = 1 # pendulum mass
        self.m_w = 1.5 # wheel mass
        self.h = 0.1 # pendulum length(distance between the center of mass of the wheel and the center of mass of the pendulum)
        self.R = 0.25 # radius of the wheel
        self.I = 0.0 # moment of inertia of the wheel, not used now because currently it is calculated with equivalent point mass
        self.g = 9.81

        self.w_b = 0.1 # width of the pendulum body
        self.h_b = 0.05 # height of the pendulum body


        ### This part will be override by the run_simulation() function always ######
        self.v_desired = 1.0 # desired velocity of the segway
        self.gamma_desired = 0.0 # desired angle of the pendulum, 0 means upright position
        self.s_desired = 0.0 # desired position of the segway,
        ##################################################################################

        ######################## This part is used for damping #######################
        # for motor damping
        self.B = 0.306 #slop of damping
        self.B_0 = 0 # damping at zero velocity, positive and negative values will be auto adjusted to make always oppose the motion(but is set to 0 now because it will cause occillations that can't be handled by the PD controller)

        self.K_tandamp = 10.0  # for smoothing the damping torque, only used in motorDamp_Smooth Model
        ############################################################################

        # for PD controller
        self.K_gamma = 3.0
        self.K_dgamma = 0.8
        self.K_velocity = 4.0
        self.K_position = 2

        # for PID controller, used only when declared using PID in run_simulation()
        self.K_vi = 0.1
        self.K_pi = 0.05
        self.posDeadZone = 0.005
        self.clip_integral = 1.0


params = Parameters()  # Light singleton pattern

__all__ = ["params"]
