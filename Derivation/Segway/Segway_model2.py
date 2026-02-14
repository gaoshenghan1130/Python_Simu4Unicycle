from Segway_model1 import Model1

class Model2(Model1):
    def __init__(self):
        super().__init__()

        # To linearize around gamma = pi, we can simply flip the sign of the gravity term in A matrix
        self.A[1, 2] *= -1
        self.A[3, 2] *= -1

Segway_model2 = Model2()  # Light singleton pattern