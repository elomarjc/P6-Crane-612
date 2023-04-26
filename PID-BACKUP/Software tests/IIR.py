class IIR:
    def __init__(self,a:list,b:list):
        self.a = a
        self.b = b
        self.x = [0 for _ in range(len(b))]
        self.y = [0 for _ in range(len(a))]
        self.out = 0
    
    def update(self,input:float) -> float:
        
        # Shift input and output arrays
        self.x.insert(0,input)
        self.x.pop(len(self.x)-1)

        self.y.insert(0,None)
        self.y.pop(len(self.y)-1)

        self.out = 0
        for i in range(len(self.b)):
            self.out += self.b[i]*self.x[i]
        for i in range(1,len(self.a)):
            self.out -= self.a[i]*self.y[i]

        self.out = self.out/a[0]

        self.y[0] = self.out

        return float(self.out)

if __name__ == "__main__":

    from matplotlib import pyplot as plt

    # w0 = 2.35 hz, zeta = 0.01, ts = 0.01
    b = [1, -1.9780,0.9966]
    a = [1, -1.7276,0.7462]

    notch = IIR(a,b)

    impulse = [1]+[0 for _ in range(99)]

    step = [1 for _ in range(100)]

    out = []
    for i in impulse:
        out.append(notch.update(i))

    plt.plot([x/len(out) for x in range(len(out))],out)
    plt.grid()
    plt.show()
