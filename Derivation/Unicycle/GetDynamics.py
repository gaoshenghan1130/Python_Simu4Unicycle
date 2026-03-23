import dill

def GetDynamics():

    with open("system_dynamics.pkl", "rb") as f:
        dynamics = dill.load(f)

        return dynamics['M'], dynamics['f']