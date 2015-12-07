import time

class PID:
    """
    Discrete PID control
    """

    def __init__(self, P, I, D, setpoint=0.0, bias=0, sampleTime=0.01, out_max=1, out_min=0):

        self.sampleTimeSec = sampleTime      # expected update frequency
        self.prevTime = time.time()          # current time in sec
        self.setKp(P)
        self.setKi(I)
        self.setKd(D)

        self.set_point = setpoint
        self.bias = bias
        self.Pvalue = 0.0
        self.Ivalue = 0.0
        self.Dvalue = 0.0

        self.enable = False

        self.out_min = out_min
        self.out_max = out_max
        self.prevPID = 0.0

        self.error = 0.0
        self.prevError = 0.0
        self.scaledErrSum = 0.0

        
    def update(self,current_value):
        """
        Calculate PID output value for given reference input and feedback
        """

        if not self.enable:
            return self.bias

        currTime = time.time()
        timeDiff = currTime - self.prevTime
        
        if (timeDiff >= self.sampleTimeSec):
            # Compute error vars
            self.error = self.set_point - current_value

            # Compute PID values
            self.Pvalue = self._Kp * self.error
            # Summing err*ki for each update to remove Integrator Bump
            self.Ivalue += self.error * self._Ki
            self.Dvalue = self._Kd * (self.error - self.prevError)

            # Reset windup protection 1
            if self.Ivalue > self.out_max:
              self.Ivalue = self.out_max
            elif self.Ivalue < self.out_min:
              self.Ivalue = self.out_min


            PID = self.bias + self.Pvalue + self.Ivalue + self.Dvalue

            # Reset windup protection 2
            if PID > self.out_max:
              PID = self.out_max
            elif PID < self.out_min:
              PID = self.out_min

            self.prevError = self.error
            self.prevTime = currTime
            self.prevPID = PID

            return PID

        return prevPID

    def setPoint(self,set_point):
        self.set_point = set_point
        self.Ivalue = 0
        self.prevError = 0

    def setKp(self,P):
        if P < 0:
            return
        self._Kp = P
        self._unscaledKp = P

    def setKi(self,I):
        if I < 0:
            return
        self._Ki = I * self.sampleTimeSec
        self._unscaledKi = I

    def setKd(self,D):
        if D < 0:
            return
        self._Kd = D / self.sampleTimeSec
        self._unscaledKd = D

    def getPoint(self):
        return self.set_point

    def getError(self):
        return self.error

    def getKp(self):
        return self._unscaledKp

    def getKi(self):
        return self._unscaledKi

    def getKd(self):
        return self._unscaledKd

    # Custom methods
    def setBias(self, bias):
        self.bias = bias

    def getBias(self):
        return self.bias

    def setSampleTime(self, newSampleTimeSec):
        if newSampleTimeSec < 0:
            return

        ratio = newSampleTimeSec / self.sampleTimeSec
        self._Ki *= ratio
        self._Kd /= ratio
        self.sampleTimeSec = newSampleTimeSec

    def toggleEnable(self):
        self.enable ^= True
        if self.enable:
            self.prevTime = time.time()
            self.resetI()

    def getEnable(self):
        return self.enable

    def resetI(self):
        self.Ivalue = 0
