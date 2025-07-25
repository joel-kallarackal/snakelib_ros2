import numpy as np


def rolling(self, t=0, params=None):

    params = {} if params is None else params

    self.current_gait = "rolling"

    # Update the current parameters if params is not empty
    self.current_gait_params = self.update_params(self.default_gait_params.get(self.current_gait), params)

    ####
    # head_acc_x = params["head_acc_x"]
    # head_acc_y = params["head_acc_y"]
    # direction = params["direction"]

    # if abs(head_acc_x) > abs(head_acc_y):
    #     self.current_gait_params["A_odd"]*=direction
    #     if head_acc_x > 0:
    #         self.current_gait_params["A_odd"] *= -1
    # else:
    #     self.current_gait_params["A_even"]*=direction
    #     if head_acc_y > 0:
    #         self.current_gait_params["A_even"]*= -1 
    ###

    # Math formulation of rolling is identical to sidewinding
    N = self.num_modules
    alpha = np.zeros(N)

    for n in range(N):
        alpha[n] = self.compound_serpenoid(t, n, self.current_gait_params)

    return alpha
