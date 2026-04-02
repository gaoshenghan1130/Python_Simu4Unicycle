from Config import CONTROL_T

control_id_to_class = {}

def register_control(control_type: CONTROL_T):
    def decorator(cls):
        control_id_to_class[control_type] = cls
        return cls
    return decorator