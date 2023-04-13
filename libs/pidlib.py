class PID:
    def __init__(self, p, i, d):
        self.Kp = p
        self.Ki = i
        self.Kd = d
        self.err = 0
        self.sum_err = 0
        self.last_err = 0
        self.time = 0
    
    def P_control(self, target, curr):
        return self.Kp*(target - curr)
        
    def D_control(self, target, curr):
        if self.time == 0:
            self.last_err = target - curr
            return 0
        self.err = target - curr
        Dterm = self.err - self.last_err
        self.last_err = self.err
        return self.Kd*Dterm
        
    def I_control(self, target, curr):
        if self.time == 0:
            self.sum_err += target - curr
            return 0
        return self.Ki*self.sum_err / self.time
        
    def control(self, target, curr):
        result =  sum([
            self.P_control(target, curr),  
            self.D_control(target, curr),
            self.I_control(target, curr)
        ])
        self.time += 1
        return result
        
