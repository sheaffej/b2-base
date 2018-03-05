#!/usr/bin/env python
from __future__ import print_function

import rospy
import Adafruit_MCP3008
import Adafruit_GPIO
from b2_base.msg import Proximity

DEFAULT_NODE_NAME = "ir_sensors"
DEFAULT_TEST_MODE = True
DEFAULT_PUB_FREQ = 1  # hertz

DEFAULT_VREF = 5.0               # volts
DEFAULT_MIN_ADC_VAL = 0
DEFAULT_MAX_ADC_VAL = 1023
DEFAULT_PROXIMITY_DIST = 0.30    # meters
DEFAULT_ADC_CHANNEL_LIST = [0]

# Hardware SPI configuration:
SPI_PORT = 0
SPI_DEVICE = 0


class IRSensors:
    def __init__(self, node_name):
        self._node_name = node_name
        self._test_mode = rospy.get_param("~test_mode", DEFAULT_TEST_MODE)
        self._pub_rate = rospy.Rate(int(rospy.get_param("~pub_freq", DEFAULT_PUB_FREQ)))
        self._adc_channel_list = rospy.get_param("~adc_channel_list", DEFAULT_ADC_CHANNEL_LIST)

        # Defined constants for the Sharp GP2Y0A60SZxF IR sensor
        vref = rospy.get_param("~vref", DEFAULT_VREF)
        min_adc_val = rospy.get_param("~min_adc_val", DEFAULT_MIN_ADC_VAL)
        max_adc_val = rospy.get_param("~max_adc_val", DEFAULT_MAX_ADC_VAL)
        proximity_dist = rospy.get_param("~proximity_distance", DEFAULT_PROXIMITY_DIST)

        # Calculate the ADC value when an object is in "proximity"
        v_per_adc = self._volts_per_adc(vref, min_adc_val, max_adc_val)
        rospy.logdebug("v_per_adc: {}".format(v_per_adc))
        self._acd_at_prox_dist = self._adc_at_proximity_dist(proximity_dist, v_per_adc)
        rospy.logdebug("acd_at_prox_dist: {}".format(self._acd_at_prox_dist))

        if not self._test_mode:
            self._mcp = Adafruit_MCP3008.MCP3008(
                spi=Adafruit_GPIO.SPI.SpiDev(SPI_PORT, SPI_DEVICE)
            )
        else:
            self._mcp = MCP3008Stub()
            for channel in self._adc_channel_list:
                self._mcp.set_adc(channel, 700)

        self._center_pub = rospy.Publisher('~center', Proximity, queue_size=1)

    def run(self):

        try:
            while not rospy.is_shutdown():

                msg = Proximity()

                for channel in self._adc_channel_list:
                    is_proximity = False  # No object detected yet

                    # Read sensor values
                    val = self._mcp.read_adc(channel)
                    if val >= self._acd_at_prox_dist:
                        is_proximity = True

                    # Flip for debugging
                    if self._test_mode:
                        self._mcp.set_adc(channel, self._mcp.read_adc(channel) * -1)

                    # Publish sensor messages
                    msg.sensors.append(is_proximity)

                self._center_pub.publish(msg)
                self._pub_rate.sleep()

        except rospy.ROSInterruptException:
            rospy.logwarn("ROSInterruptException received in main loop")

    def _volts_per_adc(self, vref, min_adc_reading, max_adc_reading):
        return vref / float(max_adc_reading - min_adc_reading)

    def _volts_at_cm_distance(self, dist_cm):
        # This function is the result of fitting the Voltage/Distance curve points in the
        # Sharp GP2Y0A60SZXF data sheet https://www.pololu.com/file/0J812/gp2y0a60szxf_e.pdf
        # using the site http://mycurvefit.com
        # The function takes in distance in cm, and outputs the voltage of the IR sensor's output
        return 0.5955366 + 6.8125134 / (1 + (dist_cm / 8.798111) ** 1.624654)

    def _adc_at_proximity_dist(self, prox_dist_m, v_per_adc):
        prox_dist_cm = prox_dist_m * 100
        v_at_prox_dist = self._volts_at_cm_distance(prox_dist_cm)
        return int(v_at_prox_dist / v_per_adc)


class MCP3008Stub:
    def __init__(self):
        self.channels = [0] * 8

    def read_adc(self, channel):
        return self.channels[channel]

    def set_adc(self, channel, val):
        self.channels[channel] = val


if __name__ == "__main__":
    rospy.init_node(DEFAULT_NODE_NAME, log_level=rospy.DEBUG)
    node_name = rospy.get_name()
    node = IRSensors(node_name)
    node.run()
