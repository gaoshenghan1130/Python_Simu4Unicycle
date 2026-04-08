from sympy.physics.mechanics import (
    ReferenceFrame,
    Point,
    dynamicsymbols,
    KanesMethod,
    Particle,
    RigidBody,
)
from sympy import init_printing, simplify, symbols
from IPython.display import display
from Config import MODEL_T, model_bin_Dir
from sympy import lambdify
import numpy as np
import dill
from Models.Algs.Derivation import Derivation
from typing_extensions import override
from Models.Register import register_model
import os


@register_model(MODEL_T.KANE)
class KaneDerivation(Derivation):
    @override
    def __init__(self):
        self.name = "Kane's Method, derived with sympy.physics.mechanics.KanesMethod"
        self.filename = MODEL_T.KANE.name  # used to save in the BinModel
        self.id = MODEL_T.KANE
        self.mass_matrix = None
        self.forcing = None
        for i in range(1, 5):
            setattr(self, f"q{i}", None)
            setattr(self, f"u{i}", None)

        self.fr = None
        self.frstar = None

    @override
    def store_dynamics(self):
        """Store current M and f in a binary file."""
        if self.mass_matrix is None or self.forcing is None:
            raise ValueError(
                "Mass matrix and forcing must be derived before storing dynamics."
            )

        args = [
            self.q1,
            self.q2,
            self.q3,
            self.q4,
            self.q5,
            self.u1,
            self.u2,
            self.u3,
            self.u4,
            self.u5,
            dynamicsymbols("T_W"),
            dynamicsymbols("F_L"),
            symbols("R"),
            symbols("h"),
            symbols("m"),
            symbols("m_w"),
            symbols("m_l"),
            symbols("g"),
        ]

        mass_matrix_func = lambdify(args, self.mass_matrix, "numpy")
        forcing_func = lambdify(args, self.forcing, "numpy")

        os.makedirs(model_bin_Dir, exist_ok=True)
        filepath = os.path.join(model_bin_Dir, f"{self.filename}.pkl")

        with open(filepath, "wb") as f:
            dill.dump({"M": mass_matrix_func, "f": forcing_func}, f)

    @override
    def derive(self, get_forcing=False, display_result=False):
        """
        Generate M and f for the system dynamics in the form of M(q) * u' = f(q, u, t)
        """
        R, h, m, m_w, m_l, g = symbols("R h m m_w m_l g")

        # Generalized coordinates and speeds
        # q1 = psi,
        # q2 = theta,
        # q3 = phi,
        # q4 = rho,
        # q5 = gamma
        self.q1, self.q2, self.q3, self.q4, self.q5 = [
            dynamicsymbols(r"\psi"),
            dynamicsymbols(r"\theta"),
            dynamicsymbols(r"\phi"),
            dynamicsymbols("r"),
            dynamicsymbols(r"\gamma"),
        ]  # psi, theta, phi, rho, gamma
        self.u1, self.u2, self.u3, self.u4, self.u5 = [
            dynamicsymbols(r"\dot{\psi}"),
            dynamicsymbols(r"\dot{\theta}"),
            dynamicsymbols(r"\dot{\phi}"),
            dynamicsymbols(r"\dot{r}"),
            dynamicsymbols(r"\dot{\gamma}"),
        ]  # corresponding generalized speeds

        kde = {self.q1.diff(): self.u1, self.q2.diff(): self.u2, self.q3.diff(): self.u3, self.q4.diff(): self.u4, self.q5.diff(): self.u5}  # type: ignore

        E0 = ReferenceFrame("E0")
        E1 = E0.orientnew("E1", "Axis", [self.q1, E0.z])  # rotate around z-axis by psi
        E2 = E1.orientnew(
            "E2", "Axis", [self.q2, E1.x]
        )  # rotate around x-axis by theta
        E3 = E2.orientnew(
            "E3", "Axis", [self.q5, E2.y]
        )  # rotate around y-axis by gamma
        EW = E3.orientnew(
            "EW", "Axis", [self.q3, E2.y]
        )  # rotate around y-axis by phi, this is the frame attached to the wheel

        O = Point("O")  # origin
        O.set_vel(E0, 0)  # origin is fixed

        ################################### Wheel center kinematics ###################################

        W = Point("W")  # wheel center
        W.set_pos(O, R * E2.z)  # type: ignore     Initial position

        w_total = (
            E2.ang_vel_in(E0) + self.u3 * E2.y
        )  # Total angular velocity fo the wheel

        r_cw = R * E2.z
        v_w_vector = w_total.cross(r_cw)

        W.set_vel(E0, v_w_vector)  # type: ignore

        ################################## Pendulum kinematics ########################################

        P = W.locatenew("P", h * E3.z)  # create Pendulum relative to the wheel
        P.v2pt_theory(W, E0, E3)

        ################################## Linear motor kinematics ########################################

        L = W.locatenew("L", (self.q4) * E2.y)
        # L.set_vel(E2, u4 * E2.y) # relative velocity in E2 frame, which is the velocity of the linear motor relative to the wheel center
        # L.v2pt_theory(W, E0, E2)

        L.set_vel(E0, W.vel(E0) + self.u4 * E2.y + E2.ang_vel_in(E0).cross(L.pos_from(W)))  # type: ignore     Absolute velocity of the linear motor, which is the velocity of the wheel center plus the relative velocity in E2 frame plus the contribution from the rotation of the E2 frame

        ############################# Define particles and forces ########################################
        body_p = Particle("body_p", P, m)
        body_w = Particle("body_w", W, m_w)
        body_l = Particle("body_l", L, m_l)

        #### Forces

        T_W = dynamicsymbols("T_W")  # torque applied to the wheel
        F_L = dynamicsymbols("F_L")  # force applied to the linear motor

        forces = [
            (P, -m * g * E0.z),
            (W, -m_w * g * E0.z),
            (L, -m_l * g * E0.z),
            ## Wheel torque
            (E3, -T_W * E2.y),  # type: ignore  torque applied to the wheel, acting on the pendulum reversely
            (EW, T_W * E2.y),  # type: ignore  torque applied to the wheel
            ## Linear motor force
            (L, -F_L * E2.y),  # type: ignore
            (W, F_L * E2.y),  # type: ignore
        ]

        ########################### Summarize the system and derive equations of motion ############################

        particles = [body_p, body_w, body_l]
        q_list = [self.q1, self.q2, self.q3, self.q4, self.q5]
        u_list = [self.u1, self.u2, self.u3, self.u4, self.u5]

        kd_eqs = [self.u1 - self.q1.diff(), self.u2 - self.q2.diff(), self.u3 - self.q3.diff(), self.u4 - self.q4.diff(), self.u5 - self.q5.diff()]  # type: ignore

        print("Deriving dynamics using Kane's method...")

        kane = KanesMethod(
            E0, q_ind=q_list, u_ind=u_list, kd_eqs=kd_eqs
        )  # on interial frame E0

        if get_forcing:
            (self.fr, self.frstar) = kane.kanes_equations(
                particles, forces
            )  # Calculate the generalized active forces and inertia forces
        else:
            kane.kanes_equations(particles, forces) 

        self.mass_matrix = kane.mass_matrix_full
        self.forcing = kane.forcing_full

        if display_result:
            init_printing(use_latex="mathjax")
            display(simplify(self.mass_matrix))
            display(simplify(self.forcing))

        ## M(q) * u' = f(q, u, t)

        ## No return here as we will store the results in the binary files
        # return self.mass_matrix, self.forcing
