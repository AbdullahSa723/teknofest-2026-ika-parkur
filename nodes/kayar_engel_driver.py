#!/usr/bin/env python3
"""
Drive the kayar engel (S6.8).

    "Kayar engel saga ve sola dogru 20 cm/s hizla surekli rejimde git-gel
     yaparak hareket edecektir. Engel parkur disina ciktigi noktada
     beklemeden 20 cm/s hizla ters yonde hareketine devam edecektir."

Constant speed, instant reversal, no dwell at the ends.

WHY THIS IS OPEN LOOP
    The joint is velocity-commanded, and gz-sim's JointController with
    use_force_commands=false sets that velocity on the joint directly rather
    than through a force. The blade therefore moves at exactly the commanded
    speed, which means its position is a known function of time and there is
    nothing to correct. Feeding back joint state would add a subscription, a
    failure mode, and no accuracy.

    The one error term is quantisation: the sign flips only on a publish, so
    the blade can overrun a turnaround by speed / publish_rate. At the defaults
    that is 0.20 / 50 = 4 mm on a 1 m blade. It does not accumulate, because
    the sign is derived from absolute simulation time rather than integrated.

WHY SIM TIME
    The node runs on /clock. If it used wall time, the blade's speed would
    scale with the real-time factor -- so it would silently stop matching the
    sartname the moment your machine could not keep up, which is exactly when
    you would be least likely to notice.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64


class KayarEngelDriver(Node):

    def __init__(self):
        super().__init__("kayar_engel_driver")

        self.declare_parameter("topic", "/kayar_engel/cmd_vel")
        self.declare_parameter("speed", 0.20)        # S6.8, m/s
        self.declare_parameter("travel", 4.0)        # peak to peak, m
        self.declare_parameter("publish_rate", 50.0)

        # Without sim time the blade would run on wall-clock seconds.
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        self.speed = float(self.get_parameter("speed").value)
        self.travel = float(self.get_parameter("travel").value)
        rate = float(self.get_parameter("publish_rate").value)
        topic = self.get_parameter("topic").value

        # A full cycle is centre -> +A -> -A -> centre. The quarter period is
        # the time to cross half the travel.
        self.quarter = (self.travel / 2.0) / self.speed
        self.period = 4.0 * self.quarter

        self.pub = self.create_publisher(Float64, topic, 10)
        self.timer = self.create_timer(1.0 / rate, self._tick)
        self._last_sign = 0.0
        self._warned = False

        self.get_logger().info(
            f"kayar engel: {self.speed * 100:.0f} cm/s over {self.travel:.2f} m, "
            f"period {self.period:.1f} s, publishing on {topic}")

    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now <= 0.0:
            if not self._warned:
                self.get_logger().warn(
                    "waiting for /clock -- is the bridge running and is "
                    "Gazebo unpaused?")
                self._warned = True
            return

        # Triangle wave starting at the road centre and moving to +y first.
        phase = now % self.period
        rising = phase < self.quarter or phase >= 3.0 * self.quarter
        sign = 1.0 if rising else -1.0

        if sign != self._last_sign and self._last_sign != 0.0:
            self.get_logger().debug(f"reversal at t={now:.2f}s")
        self._last_sign = sign

        self.pub.publish(Float64(data=sign * self.speed))


def main(args=None):
    rclpy.init(args=args)
    node = KayarEngelDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Float64(data=0.0))     # leave the blade stopped
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
