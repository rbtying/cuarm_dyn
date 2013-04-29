import rospy

class PID:
    """ 
    PID Controller
    """

    def __init__(self, proportional, integral, derivative, integrationLimit, setpoint, name):
        self.kP = proportional
        self.kI = integral
        self.kD = derivative
        self.integrationLimit = integrationLimit
        self.setpoint = setpoint
        self.output = 0
        self.name = name

        self.error = 0
        self.previousError = 0
        self.accumulatedError = 0

    def __unicode__(self):
        return self.name

    def update(self, input_value, dt):
        self.error = self.setpoint - input_value
        
        self.accumulatedError += self.error * dt
        
        derror = (self.error - self.previousError) / dt

        if self.accumulatedError > self.integrationLimit:
            self.accumulatedError = self.integrationLimit
        elif self.accumulatedError < -self.integrationLimit:
            self.accumulatedError = -self.integrationLimit

        self.output = self.error * self.kP + self.kI * self.accumulatedError + self.kD * derror

        self.previousError = self.error

        return self.output

    def setSetpoint(self, setpoint):
        self.setpoint = setpoint

    def getOutput(self):
        return self.output

    def setParams(self, kP, kI, kD):
        self.kP = kP
        self.kI = kI
        self.kD = kD

        self.error = 0
        self.previousError = 0
        self.accumulatedError = 0
