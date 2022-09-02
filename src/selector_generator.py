class SelectorVec:
    def __init__(self) -> None:
        self.vec = [0] * 6
    def x(self):
        self.vec[0] = 1
        return self
    def y(self):
        self.vec[1] = 1
        return self
    def z(self):
        self.vec[2] = 1
        return self
    def rx(self):
        self.vec[3] = 1
        return self
    def ry(self):
        self.vec[4] = 1
        return self
    def rz(self):
        self.vec[5] = 1
        return self
    def get(self):
        return self.vec