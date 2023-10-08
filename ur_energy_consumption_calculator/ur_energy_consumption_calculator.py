import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from ur_custom_msgs.msg import JointWorkDoneStamped


class EnergyConsumptionCalculator(Node):

    def __init__(self):
        super().__init__('energy_consumption_calculator')
        self.joint_states_subscription_ = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10)
        self.joint_states_subscription_
        self.energy_consumption_publisher_ = self.create_publisher(
            JointWorkDoneStamped, '/estimated_energy_consumption', 10)
        self.total_work_done = 0.0
        self.previous_joint_position = None

    def joint_states_callback(self, msg: JointState):
        if self.previous_joint_position is None:
            self.previous_joint_position = msg.position

        self.joint_work_done = JointWorkDoneStamped()
        self.joint_work_done.header = msg.header
        # Calculate each joint work done and total work done for all joints
        for joint in range(len(msg.name)):
            joint_effort = abs(msg.effort[joint])
            joint_angular_displacement = abs(msg.position[joint] - self.previous_joint_position[joint])
            joint_work_done = joint_effort * joint_angular_displacement
            
            # Assign payload to JointWorkDoneStamped object
            self.joint_work_done.name.append(msg.name[joint])
            self.joint_work_done.work_done.append(joint_work_done)
            self.total_work_done += joint_work_done
            self.joint_work_done.total_work_done = self.total_work_done

            self.previous_joint_position[joint] = msg.position[joint]

        self.energy_consumption_publisher_.publish(self.joint_work_done)


def main(args=None):
    rclpy.init(args=args)

    energy_consumption_calculator = EnergyConsumptionCalculator()

    rclpy.spin(energy_consumption_calculator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    energy_consumption_calculator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
