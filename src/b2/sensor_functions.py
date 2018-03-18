def volts_per_adc(vref, min_adc_reading, max_adc_reading):
    return vref / float(max_adc_reading - min_adc_reading)


def volts_at_cm_distance(dist_cm):
    # This function is the result of fitting the Voltage/Distance curve points in the
    # Sharp GP2Y0A60SZXF data sheet https://www.pololu.com/file/0J812/gp2y0a60szxf_e.pdf
    # using the site http://mycurvefit.com
    # The function takes in distance in cm, and outputs the voltage of the IR sensor's output
    return 0.5955366 + 6.8125134 / (1 + (dist_cm / 8.798111) ** 1.624654)


def adc_at_proximity_dist(prox_dist_m, v_per_adc):
    prox_dist_cm = prox_dist_m * 100
    v_at_prox_dist = volts_at_cm_distance(prox_dist_cm)
    return int(v_at_prox_dist / v_per_adc)


class MCP3008Stub:
    def __init__(self):
        self.channels = [0] * 8

    def read_adc(self, channel):
        return self.channels[channel]

    def set_adc(self, channel, val):
        self.channels[channel] = val
