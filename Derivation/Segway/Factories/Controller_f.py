from Config import CONTROL_STRATEGY, CONTROL_MODE
from typing import TYPE_CHECKING
from Factories.registers import _CONTROLLER_REGISTRY
if TYPE_CHECKING:
    from Controllers import Controller

class Controller_Factory:
    @staticmethod
    def create_controller(controller_type: CONTROL_STRATEGY, control_mode: CONTROL_MODE) -> "Controller":
        controller_class = _CONTROLLER_REGISTRY.get(controller_type)
        if controller_class is None:
            raise ValueError(f"No controller registered for type: {controller_type}, please use @register_controller decorater to register the controller class first.")
        
        # Controller should not have any required parameters in the constructor
        return controller_class(control_mode)
    

