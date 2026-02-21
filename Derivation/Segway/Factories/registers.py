from Config import CONTROL_STRATEGY, MODEL_T
from typing import Dict, Type, TYPE_CHECKING

if TYPE_CHECKING:
    from Controllers import Controller
    from Models import Model

_MODEL_REGISTRY: Dict[MODEL_T, Type] = {}

# use this decorator to register the model class to the factory, so that we can create the model instance by the factory later. The model class must inherit from Model base class, and should not have any required parameters in the constructor.
def register_model(model_type: MODEL_T):
    def decorator(cls):
        _MODEL_REGISTRY[model_type] = cls
        return cls
    return decorator

_CONTROLLER_REGISTRY: Dict[CONTROL_STRATEGY, Type] = {}

# use this decorator to register the controller class to the factory, so that we can create the controller instance by the factory later. The controller class must inherit from Controller base class, and should not have any required parameters in the constructor.
def register_controller(controller_type: CONTROL_STRATEGY):
    def decorator(cls):
        _CONTROLLER_REGISTRY[controller_type] = cls
        return cls
    return decorator