class PID:
    def __init__(self, kp, ki, kd, const, maxmum):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.const = const
        self.p = 0
        self.i = 0
        self.d = 0
        self.last_e = 0
        self.intergration = 0
        self.maxmum = maxmum

    def adjust(self, now, target, step):
        now_e = target - now
        self.intergration += now_e * step
        self.intergration = min(max(self.intergration, -self.maxmum), self.maxmum)
        v = self.kp * now_e + self.ki * self.intergration + self.kd * (now_e - self.last_e) / step + self.const
        self.last_e = now_e
        return v

if __name__ == '__main__':
    import physics
    import random
    obj = physics.Motion(0, 0, 1)
    gravity = -9.8
    pid = PID(0.2, 0.05, 1, 0, 10000000)
    step = 0.1
    xList = [x * step for x in range(0, int(100/step))]
    yList = []
    for t in xList:
        control = pid.adjust(obj.pos, 100, step)
        obj.update(control+gravity+(random.random()-0.5)*10, step)
        yList.append(obj.pos)
    import drawing
    drawing.draw(xList, yList)
    
