from Config import MODEL_T

model_id_to_class = {}

def register_model(model_type: MODEL_T):
    def decorator(cls):
        model_id_to_class[model_type] = cls
        return cls
    return decorator