from GetDynamics import GetDynamics
import numpy as np


class UnicycleModel:
    def __init__(self):
        self.M, self.f = GetDynamics()
    def get_equations_of_motion(self, t, state, u_input, params):
        # Input in the order of state = [q1, q2, q3, q4, q5, u1, u2, u3, u4, u5] u_i are derivatives of q_i
        # u_input = ['T_W', 'F_L'] 
        # Params = {R = , h = , m = , m_w = , m_p = , m_l =, g = } (should be dictionary)

        q_u = list(state)
        controls = [u_input['T_W'], u_input['F_L']]
        phys_params = [
            params['R'], params['h'], params['m'], 
            params['m_w'], params['m_l'], params['g']
        ]

        args = q_u + controls + phys_params # To combine all the params together for function evaluation

        # print(f"Evaluating M and f at t={t:.2f}, state={state}, controls={u_input}, params={params}")

        M_eval = self.M(*args)
        f_eval = self.f(*args)

        return np.linalg.solve(M_eval, f_eval).flatten()
