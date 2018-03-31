#!/usr/bin/env python
from __future__ import print_function

import Adafruit_GPIO
import Adafruit_MCP3008
import b2_logic
from b2.msg import Proximity
import rospy


DEFAULT_NODE_NAME = "ir_sensors"
DEFAULT_PROXIMITY_TOPIC = "~proximity"
DEFAULT_TEST_MODE = True
DEFAULT_PUB_HZ = 1  # hertz

DEFAULT_VREF = 5.0               # volts
DEFAULT_MIN_ADC_VAL = 0
DEFAULT_MAX_ADC_VAL = 1023
DEFAULT_PROXIMITY_DIST = 0.30    # meters
DEFAULT_NUM_ADC_CHANNELS = 1

# Hardware SPI configuration:
SPI_PORT = 0
SPI_DEVICE = 0


class IRSensors:
    def __init__(self, node_name):
        self._node_name = node_name
        self._test_mode = rospy.get_param("~test_mode", DEFAULT_TEST_MODE)
        self._pub_rate = rospy.Rate(int(rospy.get_param("~pub_hz", DEFAULT_PUB_HZ)))
        self._num_adc_channels = rospy.get_param("~num_adc_channels", DEFAULT_NUM_ADC_CHANNELS)

        # Defined constants for the Sharp GP2Y0A60SZxF IR sensor
        vref = rospy.get_param("~vref", DEFAULT_VREF)
        min_adc_val = rospy.get_param("~min_adc_val", DEFAULT_MIN_ADC_VAL)
        max_adc_val = rospy.get_param("~max_adc_val", DEFAULT_MAX_ADC_VAL)
        proximity_dist = rospy.get_param("~proximity_distance", DEFAULT_PROXIMITY_DIST)

        # Calculate the ADC value when an object is in "proximity"
        v_per_adc = b2_logic.volts_per_adc(vref, min_adc_val, max_adc_val)
        rospy.logdebug("v_per_adc: {}".format(v_per_adc))
        self._acd_at_prox_dist = b2_logic.adc_at_proximity_dist(proximity_dist, v_per_adc)
        rospy.logdebug("acd_at_prox_dist: {}".format(self._acd_at_prox_dist))

        if not self._test_mode:
            self._mcp = Adafruit_MCP3008.MCP3008(
                spi=Adafruit_GPIO.SPI.SpiDev(SPI_PORT, SPI_DEVICE)
            )
        else:
            self._mcp = b2_logic.MCP3008Stub()
            for channel in range(self._num_adc_channels):
                self._mcp.set_adc(channel, 700)

        self._center_pub = rospy.Publisher(
            rospy.get_param("~proximity_topic", DEFAULT_PROXIMITY_TOPIC),
            Proximity,
            queue_size=1
        )

    def run(self):

        try:
            while not rospy.is_shutdown():

                msg = Proximity()

                for channel in range(self._num_adc_channels):
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


if __name__ == "__main__":
    rospy.init_node(DEFAULT_NODE_NAME, log_level=rospy.DEBUG)
    node_name = rospy.get_name()
    node = IRSensors(node_name)
    node.run()
