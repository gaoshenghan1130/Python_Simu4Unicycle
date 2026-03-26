from Models.GetDynamics import GetDynamics
import numpy as np
from Config import MODEL_T, model_bin_Dir
import os

### Model Loader, will recalculate the algrithms in  and save them in BinModel


class UnicycleModel:
    def __init__(self, model_id: MODEL_T):
        self.M = None
        self.f = None
        self.model_id = model_id
        self.bin_dir = "Models/BinModels"
        os.makedirs(self.bin_dir, exist_ok=True)
        self.bin_file = os.path.join(self.bin_dir, f"{self.model_id.name}.pkl")


    def load_dynamics(self, recalculate=False):
        self.M, self.f = GetDynamics(self.model_id, refresh=recalculate)

    def isLoaded(self):
        return os.path.exists(self.bin_file)

    def get_equations_of_motion(self, t, state, u_input, params):
        # Input in the order of state = [q1, q2, q3, q4, q5, u1, u2, u3, u4, u5] u_i are derivatives of q_i
        # u_input = ['T_W', 'F_L']
        # Params = {R = , h = , m = , m_w = , m_p = , m_l =, g = } (should be dictionary)

        q_u = list(state)
        controls = [u_input["T_W"], u_input["F_L"]]
        phys_params = [
            params["R"],
            params["h"],
            params["m"],
            params["m_w"],
            params["m_l"],
            params["g"],
        ]

        args = (
            q_u + controls + phys_params
        )  # To combine all the params together for function evaluation

        # print(f"Evaluating M and f at t={t:.2f}, state={state}, controls={u_input}, params={params}")
        if self.M is None or self.f is None:
            raise ValueError(
                "Mass matrix and forcing function must be loaded before getting equations of motion. Please call load_dynamics() first."
            )

        M_eval = self.M(*args)
        f_eval = self.f(*args)

        return np.linalg.solve(
            M_eval, f_eval
        ).flatten()  # inversion of M to get u' = M^-1 * f
