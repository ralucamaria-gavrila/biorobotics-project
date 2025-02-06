

def counter_to_angle_Long(counter):
    """
    Change the encoder count of m1 into an angle in degrees.
    The beginning position of the robot is set at m1 = 90 degrees and m2 = 180 degrees.
    """
    angles_per_counter = 360 / 8400  
    angle = counter * angles_per_counter
    angle += 90 # angle of m1 at beginning position
    return angle


def counter_to_angle_Short(counter):
    """
    Change the encoder count of m2 into an angle in degrees.
    The beginning position of the robot is set at m1 = 90 degrees and m2 = 180 degrees.
    """
    angles_per_counter = 360 / 8400  
    angle = counter * angles_per_counter
    angle += 180 # angle of m2 at beginning position
    return angle


class PIDcontroller (object):
    def __init__(self, t_step, p_gain, i_gain, d_gain):
        self.t_step = t_step
        self.p_gain = p_gain
        self.i_gain = i_gain
        self.d_gain = d_gain
        self.accumulated_error = 0
        self.error_prev = 0

        return

    def step(self, ref_angle, current_encoder, First):
        if First:
            encoder_angle = counter_to_angle_Long(current_encoder)      # deriving current angle m1 from current encoder count
        else:
            encoder_angle = counter_to_angle_Short(current_encoder)     # deriving current angle m2 from current encoder count
        error_angle = ref_angle - encoder_angle

        error_change = (error_angle - self.error_prev)/self.t_step      # error for the derivative part

        proportional = self.p_gain*error_angle                          # calculating the proportional
        integral = self.i_gain * self.accumulated_error                 # calculating the integral
        derivative = self.d_gain * error_change                         # calculating the derivative of the signal

        control_signal = (proportional+integral+derivative)             # calculating the control signal
        
        if abs(control_signal) < 1:
            self.accumulated_error = self.accumulated_error + error_angle*self.t_step  # error for the integral part

        pwm = control_signal

        self.error_prev = error_angle  # updating the error

        return pwm


