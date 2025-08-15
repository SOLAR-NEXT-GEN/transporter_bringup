#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
import time
from example_interfaces.srv import AddTwoInts


class NavigationScheduler(Node):
    def __init__(self):
        super().__init__('navigation_scheduler')

        self.declare_parameter('button_index', 0)
        self.declare_parameter('debounce_time', 0.5)

        self.button_index = self.get_parameter('button_index').get_parameter_value().integer_value
        self.debounce_time = self.get_parameter('debounce_time').get_parameter_value().double_value

        self.current_status = "IDLE"
        self.last_button_time = 0.0
        self.button_pressed = False
        self.is_first_time = True
        self.arm_animating = False

        self.srv = self.create_service(AddTwoInts, 'mani_state', self.mani_state_callback)
        self.cli = self.create_client(AddTwoInts, 'Set_Mani')
        self.req = AddTwoInts.Request()

        self.hinge_status = [0,0] # hinge_status[0] = left hinge, hinge_status[1] = right hinge
        # 1 == down success // 2 == down fail // 0 == ready for order

        self.flex_status = [0,0,0,0] # flex_status[0] = left front, flex_status[1] = left back, flex_status[2] = right front, flex_status[3] = right back
        # 0 == not found // 1 == found

        self.dummy_state = 0
        self.dummy_time = 0

        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.status_sub = self.create_subscription(
            String,
            '/controller_status',
            self.status_callback,
            10
        )

        self.path_generator_client = self.create_client(
            Trigger,
            'generate_straight_path'
        )

        self.controller_client = self.create_client(
            SetBool,
            'robot_start'
        )

        self.hinges_up_client = self.create_client(
            SetBool,
            'set_hinges_up'
        )

        self.hinges_down_client = self.create_client(
            SetBool,
            'set_hinges_down'
        )

        self.get_logger().info('Enhanced Navigation Scheduler initialized')
        self.get_logger().info(f'Monitoring joy button {self.button_index}')

        self.timer = self.create_timer(0.01, self.timer_callback)
        self.task = 0
        self.wait_arm_up = 0

    def mani_state_callback(self,req,res):
        self.get_logger().info(self.translate_client_request(req.a, req.b))

        if (req.b == 0 and req.a !=0):
            hinge_id = req.a // 3 # 1 == left /// 2 == right 
            hinge_state = req.a % 3  # 1 == down success // 2 == down fail // 0 == ready for order 
            self.hinge_status[hinge_id -1 ] = hinge_state
            res.sum = 1
            return res
        if (req.a ==0 and req.b!= 0):
            flex_id = req.b // 2 # 1 == left front // 2 = left back // 3 = right front // 4 = right back 
            flex_state = req.b % 2 # 0 = not found // 1 = found
            self.flex_status[flex_id-1]=flex_state
            res.sum = 1
            return res
        else :
            res.sum = 0
            return res

    def Set_mani(self, a, b):
        self.req.a = a
        self.req.b = b
        return self.cli.call_async(self.req)

    def move_arm_down(self):
        self.Set_mani(1,1)
        # self.Set_mani(2,1)

    def move_arm_up(self):
        self.Set_mani(1,2)
        # self.Set_mani(2,2)

    def joy_callback(self, msg):
        if len(msg.buttons) <= self.button_index:
            return

        current_time = time.time()
        button_state = msg.buttons[self.button_index]

        if button_state == 1 and not self.button_pressed:
            if current_time - self.last_button_time > self.debounce_time:
                self.button_pressed = True
                self.last_button_time = current_time
                self.handle_button_press()
        elif button_state == 0:
            self.button_pressed = False

    def status_callback(self, msg):
        old_status = self.current_status
        self.current_status = msg.data

        if old_status == "FOLLOWING" and (self.current_status == "GOAL_REACHED" or self.current_status == "STOPPED"):
            self.get_logger().info(f'Robot stopped/reached goal, moving arm down')
            self.move_arm_down()

    def handle_button_press(self):
        if self.arm_animating:
            self.get_logger().info('Arm animation in progress, ignoring button press')
            return

        self.get_logger().info(f'Button pressed, current status: {self.current_status}')

        if self.current_status == "IDLE":
            if self.is_first_time:
                self.get_logger().info('First time - moving directly without arm animation')
                self.is_first_time = False
                self.generate_and_start_path()
            else:
                self.get_logger().info('Moving arm up before starting')
                self.move_arm_up_then_start()
        elif self.current_status == "FOLLOWING":
            self.get_logger().info('Stopping robot and moving arm down')
            self.stop_controller()
        elif self.current_status == "STOPPED":
            self.get_logger().info('Moving arm up before resuming')
            self.move_arm_up_then_resume()
        elif self.current_status == "GOAL_REACHED":
            self.get_logger().info('Moving arm up before generating new path')
            self.move_arm_up_then_start()
        else:
            self.get_logger().warn(f'Unknown status: {self.current_status}')

    def move_arm_up_then_start(self):
        current_time=time.time()
        if (self.arm_animating == False):
            self.move_arm_up()
            self.arm_animating = True
            self.dummy_state = 0
            self.dummy_time = time.time()
            self.task = 1

    def move_arm_up_then_resume(self):
        current_time=time.time()
        if (self.arm_animating == False):
            self.move_arm_up()
            self.arm_animating=True
            self.dummy_state = 0
            self.dummy_time = time.time()
            self.task = 2

    def generate_and_start_path(self):
        self.get_logger().info('Generating path and starting controller...')

        if not self.path_generator_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Path generator service not available')
            return

        path_request = Trigger.Request()
        path_future = self.path_generator_client.call_async(path_request)
        path_future.add_done_callback(self.path_generated_callback)

    def path_generated_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Path generated successfully')
                self.enable_controller()
            else:
                self.get_logger().error(f'Path generation failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Path generation service call failed: {e}')

    def enable_controller(self):
        self.get_logger().info('Attempting to enable controller...')

        if not self.controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Controller service not available')
            return

        controller_request = SetBool.Request()
        controller_request.data = True
        controller_future = self.controller_client.call_async(controller_request)
        controller_future.add_done_callback(self.controller_enabled_callback)

    def controller_enabled_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Controller enabled')
            else:
                self.get_logger().error('Failed to enable controller')
        except Exception as e:
            self.get_logger().error(f'Controller enable service call failed: {e}')

    def stop_controller(self):
        self.get_logger().info('Stopping controller...')

        if not self.controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Controller service not available')
            return

        controller_request = SetBool.Request()
        controller_request.data = False
        controller_future = self.controller_client.call_async(controller_request)
        controller_future.add_done_callback(self.controller_stopped_callback)

        if (self.hinge_status[0] == 1 and self.hinge_status[1] == 1):
            self.arm_animating = False

    def controller_stopped_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Controller stopped')
                self.move_arm_down
                self.task=3
                self.arm_animating = True
            else:
                self.get_logger().error('Failed to stop controller')
        except Exception as e:
            self.get_logger().error(f'Controller stop service call failed: {e}')

    def resume_controller(self):
        self.get_logger().info('Resuming controller...')

        if not self.controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Controller service not available')
            return

        controller_request = SetBool.Request()
        controller_request.data = True
        controller_future = self.controller_client.call_async(controller_request)
        controller_future.add_done_callback(self.controller_resumed_callback)

    def controller_resumed_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Controller resumed')
            else:
                self.get_logger().error('Failed to resume controller')
        except Exception as e:
            self.get_logger().error(f'Controller resume service call failed: {e}')

    def translate_client_request(self,a, b):
        """
        Alternative implementation using dictionary lookup.
        More efficient for frequent lookups.
        
        Args:
            a (int): First parameter value
            b (int): Second parameter value
            
        Returns:
            str: Corresponding client request string
        """
        lookup_table = {
            (4, 0): "CLIENT_REQ_L_DOWN_SUCCESS",
            (5, 0): "CLIENT_REQ_L_DOWN_FAIL \n\n\n // PLS WAIT FOR ''CLIENT_REQ_L_READY_FOR_ORDER'' // \n\n\n",
            (3, 0): "CLIENT_REQ_L_READY_FOR_ORDER",
            (7, 0): "CLIENT_REQ_R_DOWN_SUCCESS",
            (8, 0): "CLIENT_REQ_R_DOWN_FAIL \n\n\n // PLS WAIT FOR ''CLIENT_REQ_R_READY_FOR_ORDER'' // \n\n\n",
            (6, 0): "CLIENT_REQ_R_READY_FOR_ORDER",
            (0, 2): "CLIENT_REQ_FL_FLEX_PRESSED",
            (0, 3): "CLIENT_REQ_FL_FLEX_RELEASED",
            (0, 4): "CLIENT_REQ_BL_FLEX_PRESSED",
            (0, 5): "CLIENT_REQ_BL_FLEX_RELEASED",
            (0, 6): "CLIENT_REQ_FR_FLEX_PRESSED",
            (0, 7): "CLIENT_REQ_FR_FLEX_RELEASED",
            (0, 8): "CLIENT_REQ_BR_FLEX_PRESSED",
            (0, 9): "CLIENT_REQ_BR_FLEX_RELEASED",
        }

        return lookup_table.get((a, b), "UNKNOWN_CLIENT_REQUEST")

    def timer_callback(self):
        if (self.hinge_status[0] == 0 and self.hinge_status[1] == 0):
            self.arm_animating = False

        if (self.task!= 0):
            if (self.task == 1):
                current_time = time.time()
                if (self.hinge_status[0]==0 and self.hinge_status[1]==0 and self.dummy_state == 0):
                    self.generate_and_start_path()
                    self.arm_animating = False
                    self.dummy_state = 1
                    self.task = 0
                elif(current_time-self.dummy_time <= 10):
                    self.get_logger().info('waiting for arm going up')
                else:
                    self.get_logger().error(f'Arm UP failed: ')
                    self.dummy_state = 2
                    self.task = 0
            if(self.task ==2 ):
                current_time=time.time()
                if (self.hinge_status[0]==0 and self.hinge_status[1]==0 and self.dummy_state == 0):
                    self.resume_controller()
                    self.arm_animating=False
                    self.dummy_state = 1
                    self.task = 0
                elif(current_time-self.dummy_time <= 10):
                    self.get_logger().info('waiting for arm going up')
                else:
                    self.get_logger().error(f'Arm UP failed: ')
                    self.dummy_state = 2
                    self.task = 0

            if (self.task == 3):
                if(self.hinge_status[0] == 1 and self.hinge_status[1] == 1):
                    self.get_logger().info("arm down success")
                    self.arm_animating = False
                    self.task = 0
                    
                if(self.hinge_status[0] == 2 or self.hinge_status[1] == 2):
                    self.get_logger().error("\n\n\n arm down failed \n\n\n // PLS WAIT FOR ARM GOING UP // \n\n\n")
                    self.wait_arm_up = 1
                if (self.wait_arm_up == 1 and (self.hinge_status[0] == 0 and self.hinge_status[1] == 0)):
                    self.get_logger().info("ARM UP SUCCESS READY FOR ORDER")
                    self.arm_animating = False
                    self.wait_arm_up = 0
                    self.task = 0
                    
def main(args=None):
    rclpy.init(args=args)
    
    node = NavigationScheduler()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Navigation Scheduler...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()