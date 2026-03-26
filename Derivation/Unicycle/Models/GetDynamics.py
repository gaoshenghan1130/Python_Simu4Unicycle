import dill
from Config import MODEL_T
from Algs import Derivation

model_id_to_class = {}

def register_model(model_type: MODEL_T):
    def decorator(cls):
            model_id_to_class[model_type] = cls
    return decorator



def GetDynamics(model_id : MODEL_T, refresh=False):
    if refresh:
        cls = model_id_to_class.get(model_id)
        if cls is None:
            raise ValueError(f"No model class registered for model_id: {model_id}, please use @register_model decorator to register the model class first. And remember to import the model class in Models/__init__.py")
        model_instance : Derivation = cls() # must be a subclass of Derivation, and should not have any required parameters in the constructor
        if model_instance.id == MODEL_T.EMPTY:
            raise ValueError(f"The model class {cls.__name__} does not have a valid id, or you are using the empty template for calculation, please set the id attribute in the model class to a valid MODEL_T value.")
    
        model_instance.derive()
        model_instance.store_dynamics() # derive and store the dynamics in Models/BinModel/
         
        
    # then load the dynamics from the binary file
    with open(f"/BinModels/{model_id.name}.pkl", "rb") as f:
        dynamics = dill.load(f)
        return dynamics['M'], dynamics['f']