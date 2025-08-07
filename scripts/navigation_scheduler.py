#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
import time


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
        self.arm_animating = True
        self.get_logger().info('Calling hinges UP service before starting movement...')
        
        request = SetBool.Request()
        request.data = True
        
        future = self.hinges_up_client.call_async(request)
        future.add_done_callback(self.arm_up_for_start_callback)
    
    def move_arm_up_then_resume(self):
        self.arm_animating = True
        self.get_logger().info('Calling hinges UP service before resuming movement...')
        
        request = SetBool.Request()
        request.data = True
        
        future = self.hinges_up_client.call_async(request)
        future.add_done_callback(self.arm_up_for_resume_callback)
    
    def move_arm_down(self):
        self.arm_animating = True
        self.get_logger().info('Calling hinges DOWN service...')
        
        request = SetBool.Request()
        request.data = True
        
        future = self.hinges_down_client.call_async(request)
        future.add_done_callback(self.arm_down_callback)
    
    def arm_up_for_start_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Arm UP completed, generating path and starting')
                self.generate_and_start_path()
            else:
                self.get_logger().error(f'Arm UP failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Arm UP service call failed: {str(e)}')
        finally:
            self.arm_animating = False
    
    def arm_up_for_resume_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Arm UP completed, resuming controller')
                self.resume_controller()
            else:
                self.get_logger().error(f'Arm UP failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Arm UP service call failed: {str(e)}')
        finally:
            self.arm_animating = False
    
    def arm_down_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Arm DOWN completed, robot ready')
            else:
                self.get_logger().error(f'Arm DOWN failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Arm DOWN service call failed: {str(e)}')
        finally:
            self.arm_animating = False
            
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
        
    def controller_stopped_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Controller stopped')
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