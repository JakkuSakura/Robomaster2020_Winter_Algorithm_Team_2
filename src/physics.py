class Motion:
    def __init__(self, pos, vel, mass):
        self.pos = pos
        self.vel = vel
        self.mass = mass

    def update(self, force, time):
        self.vel += force / self.mass * time
        self.pos += self.vel * time
