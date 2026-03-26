from Config import MODEL_T
from Models.Register import register_model


@register_model(MODEL_T.EMPTY)
class Derivation:
    def __init__(self):
        self.name = "Kane's Method, derived with sympy.physics.mechanics.KanesMethod"
        self.filename = MODEL_T.EMPTY
        self.id = MODEL_T.EMPTY

    def store_dynamics(self):
        """Store current M and f in a binary file."""
        raise NotImplementedError(
            "Should not use template directly, please use children from this class"
        )

    def derive(self, get_forcing=False, display_result=False):
        raise NotImplementedError(
            "Should not use template directly, please use children from this class"
        )
