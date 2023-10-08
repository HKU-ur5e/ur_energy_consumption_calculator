import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from ur_custom_msgs.msg import JointWorkDoneStamped
from std_srvs.srv import Trigger


class EnergyConsumptionCalculator(Node):

    def __init__(self):
        super().__init__('energy_consumption_calculator')
        # Subscription
        self.joint_states_subscription_ = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10)
        self.joint_states_subscription_
        # Publisher
        self.energy_consumption_publisher_ = self.create_publisher(
            JointWorkDoneStamped, '/estimated_energy_consumption', 10)
        # Service
        self.reset_work_done_service = self.create_service(Trigger, '/reset_joint_work_done', self.reset_work_done_callback)
        
        self.joint_cumulative_work_done = {}
        self.previous_joint_position = None

    def joint_states_callback(self, msg: JointState):
        if self.previous_joint_position is None:
            self.previous_joint_position = msg.position

        self.joint_work_done = JointWorkDoneStamped()
        self.joint_work_done.header = msg.header
        # Calculate each joint work done and pass it to the cumulative dictionary
        for joint in range(len(msg.name)):
            joint_effort = abs(msg.effort[joint])
            joint_angular_displacement = abs(msg.position[joint] - self.previous_joint_position[joint])
            joint_work_done = joint_effort * joint_angular_displacement
            if msg.name[joint] in self.joint_cumulative_work_done:
                self.joint_cumulative_work_done[msg.name[joint]] += joint_work_done
            else:
                self.joint_cumulative_work_done[msg.name[joint]] = joint_work_done
            self.previous_joint_position[joint] = msg.position[joint]

        # Assign payload to JointWorkDoneStamped object
        self.joint_work_done.name = list(self.joint_cumulative_work_done.keys())
        self.joint_work_done.work_done = list(self.joint_cumulative_work_done.values())
        self.joint_work_done.total_work_done = sum(self.joint_cumulative_work_done.values())

        self.energy_consumption_publisher_.publish(self.joint_work_done)

    def reset_work_done_callback(self, request, response: Trigger):
        # Reset all joint work done values to zero
        self.joint_cumulative_work_done = {}
        self.previous_joint_position = None

        # Return success response
        response.success = True
        response.message = 'Joint work done reset successful'
        return response


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
