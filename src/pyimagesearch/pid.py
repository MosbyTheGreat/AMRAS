# import necessary packages
import time


class PID:
    def __init__(self, kP=1, kI=0, kD=0):
        # initialize gains
        self.kP = kP
        self.kI = kI
        self.kD = kD

    def initialize(self):
        # intialize the current and previous time
        self.curr_time = time.time()
        self.prev_time = self.curr_time

        # initialize the previous error
        self.prev_error = 0

        # initialize the term result variables
        self.cP = 0
        self.cI = 0
        self.cD = 0

    def update(self, error, sleep=0.2):
        # pause for a bit
        time.sleep(sleep)

        # grab the current time and calculate delta time
        self.curr_time = time.time()
        delta_time = self.curr_time - self.prev_time

        # delta error
        delta_error = error - self.prev_error

        # proportional term
        self.cP = error

        # integral term
        self.cI += error * delta_time

        # derivative term and prevent divide by zero
        self.cD = (delta_error / delta_time) if delta_time > 0 else 0

        # save previous time and error for the next update
        self.prev_time = self.curr_time
        self.prev_error = error

        # sum the terms and return
        return sum([
            self.kP * self.cP,
            self.kI * self.cI,
            self.kD * self.cD])
