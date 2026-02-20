from Segway_model1 import Model1
from typing_extensions import deprecated

@deprecated("Model2 trys to correct Model1, but it is still not fully accurate. Please use Model instead.")
class Model2(Model1):
    def __init__(self):
        super().__init__()

        # To linearize around gamma = pi, we can simply flip the sign of the gravity term in A matrix
        self.A[1, 2] *= -1
        self.A[3, 2] *= -1


# Segway_model2 = Model2()  # Light singleton pattern, deprecated, please use Model instead.