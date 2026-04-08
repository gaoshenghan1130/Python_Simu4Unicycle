import torch
import copy
import math
import random
from typing import List

class ParamTuner:
    def __init__(self, param_dict, simulation, penaltyFunc):
        self.param_dict = param_dict
        self.simulation = simulation
        self.penaltyFunc = penaltyFunc

    def perturb(self, params, scale=0.05):
        new_params = copy.deepcopy(params)
        for k in new_params:
            new_params[k] += torch.randn(1).item() * scale * (abs(params[k]) + 1e-6)  # in case 0 is met

            #print(f"Perturbed {k}: {params[k]:.4f} -> {new_params[k]:.4f}")
        return new_params

    def _installParams(self, p_dict):
        from Parameters import params as global_params
        global_params.updateParams(p_dict)

    def acceptance(self, old_score, new_score, temperature):
        if new_score < old_score:
            return True
        else:
            # exp(-(delta) / T)
            delta = new_score - old_score
            accept_prob = math.exp(-delta / temperature)
            return random.random() < accept_prob

    def tune(self, resultStorage: List[List[float]], max_iter=200, T=1.0, cooling=0.99, plotter=None):
        current_params = copy.deepcopy(self.param_dict)
        self._installParams(current_params)
        
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

                #print(f"Better or Accepted new score: {new_score:.6f} at iteration {i}")
                
                if current_score < best_score:
                    best_score = current_score
                    best_params = copy.deepcopy(current_params)
                    print(f"Iter {i}: New Best Score Found: {best_score:.6f}")
                    print(f"Best Parameters: {best_params}")
            current_T *= cooling

        if plotter is not None:
            plotter.update(resultStorage)

        self.param_dict = best_params
        self._installParams(best_params)
        return best_params