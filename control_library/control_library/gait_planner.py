
class GaitPlanner:
    def __init__(self, t_stance, t_swing, phase_lag):
        self.t_stance = t_stance
        self.t_swing = t_swing
        self.phase_lag = phase_lag
        self.T_stride = self.t_swing + self.t_stance

    def signal_sample(self, time, leg):
        phase_i = self.phase_lag[leg]
        phase_time = phase_i * self.T_stride

        base_time = (time - phase_time) % self.T_stride
        while base_time < 0:
            base_time = self.T_stride - base_time

        if base_time <= self.t_stance:
            # in stride period
            return [False, base_time / self.t_stance]
        else:
            # in swing period
            return [True, (base_time - self.t_stance) / self.t_swing]
