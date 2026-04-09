import torch
import copy
import math
import random
from typing import List
from IPython import display  # 必须导入

class ParamTuner:
    def __init__(self, param_dict, simulation, penaltyFunc):
        self.param_dict = param_dict
        self.simulation = simulation
        self.penaltyFunc = penaltyFunc

    def perturb(self, params, scale=0.05):
        new_params = copy.deepcopy(params)
        for k in new_params:
            new_params[k] += torch.randn(1).item() * scale * (abs(params[k]) + 1e-6)
        return new_params

    def _installParams(self, p_dict):
        from Parameters import params as global_params
        global_params.updateParams(p_dict)

    def acceptance(self, old_score, new_score, temperature):
        if new_score < old_score:
            return True
        else:
            delta = new_score - old_score
            if temperature < 1e-10: return False 
            accept_prob = math.exp(-delta / temperature)
            return random.random() < accept_prob

    def tune(self, resultStorage: List[List[float]], max_iter=200, T=0.1, cooling=0.99, plotter=None):
        current_params = copy.deepcopy(self.param_dict)
        self._installParams(current_params)

        print("initial params:", current_params)
        
        resultStorage.clear()
        self.simulation(resultStorage)
        current_score = self.penaltyFunc(resultStorage)

        best_params = copy.deepcopy(current_params)
        best_score = current_score
        current_T = T

        for i in range(max_iter):
            candidate = self.perturb(current_params)

            self._installParams(candidate)
            resultStorage.clear()
            self.simulation(resultStorage)
            new_score = self.penaltyFunc(resultStorage)

            if self.acceptance(current_score, new_score, current_T):
                current_params = copy.deepcopy(candidate)
                current_score = new_score
                
                if current_score < best_score:
                    best_score = current_score
                    best_params = copy.deepcopy(current_params)

                print(f"Iteration {i+1}/{max_iter}, Current Score: {current_score:.6f}, Best Score: {best_score:.6f}")
            else:
                pass

            

                

            current_T *= cooling
        
        if plotter is not None:
                    display.clear_output(wait=True)
                    
                    print(f"Best Score: {best_score:.6f}")
                    print(f"Best Parameters: {best_params}")

                    plotter.update(resultStorage)

        self.param_dict = best_params
        self._installParams(best_params)
        return best_params