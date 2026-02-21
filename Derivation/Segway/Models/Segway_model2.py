from Models.Segway_model1 import Model1
from typing_extensions import deprecated
from Factories import register_model
from Config import MODEL_T

@deprecated("Model2 trys to correct Model1, but it is still not fully accurate. Please use Model instead.")
@register_model(MODEL_T.MODEL2)
class Model2(Model1):
    def __init__(self):
        super().__init__()

        # To linearize around gamma = pi, we can simply flip the sign of the gravity term in A matrix
        self.A[1, 2] *= -1
        self.A[3, 2] *= -1
