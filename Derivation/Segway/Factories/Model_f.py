from Config import MODEL_T
from typing import TYPE_CHECKING
from Factories.registers import _MODEL_REGISTRY

if TYPE_CHECKING:
    from Models.Segway_model import Model


class Model_Factory:
    @staticmethod
    def create_model(model_type: MODEL_T) -> "Model":
        model_class = _MODEL_REGISTRY.get(model_type)
        if model_class is None:
            raise ValueError(f"No model registered for type: {model_type}, please use @register_model decorater to register the model class first. And remember to import the model class in Models/__init__.py")
        
        # Model should not have any required parameters in the constructor
        return model_class()
    

