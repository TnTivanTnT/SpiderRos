#!/usr/bin/env python3
"""
Spider Robot Keyboard Teleoperation Node (Hito 2)

This node captures keyboard input (W/S/A/D/Q/E) and publishes joint position
commands to the spider robot's leg controllers in real-time.

Key mappings:
  W/S: Move all 4 femurs (pitch up/down)
  A/D: Move all 4 coxas (yaw left/right)
  Q/E: Move all 4 tibias (extend/retract)

Joint limits (from URDF):
  Coxa:  [-π/2, π/2]   ≈ [-1.57, 1.57] rad
  Femur: [-π/3, π/3]   ≈ [-1.05, 1.05] rad
  Tibia: [-2π/3, 0]    ≈ [-2.09, 0]    rad
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
import tty
import termios
import select
import signal
import math
from typing import Dict, List

class SpiderTeleopKey(Node):
    """Keyboard teleoperation node for spider robot joint control."""
    
    # Joint limits (from URDF)
    JOINT_LIMITS = {
        'coxa': {'min': -math.pi/2, 'max': math.pi/2},
        'femur': {'min': -math.pi/3, 'max': math.pi/3},
        'tibia': {'min': -2*math.pi/3, 'max': 0},
    }
    
    # Increment per keypress (radians)
    INCREMENT = 0.1
    
    def __init__(self):
        super().__init__('spider_teleop_key')
        
        # Create publishers for the 4 leg controllers
        self.leg_publishers = {
            'front_right': self.create_publisher(
                Float64MultiArray, '/front_right_leg_controller/commands', 10
            ),
            'front_left': self.create_publisher(
                Float64MultiArray, '/front_left_leg_controller/commands', 10
            ),
            'back_left': self.create_publisher(
                Float64MultiArray, '/back_left_leg_controller/commands', 10
            ),
            'back_right': self.create_publisher(
                Float64MultiArray, '/back_right_leg_controller/commands', 10
            ),
        }
        
        # Initialize position state for all 4 legs
        self.leg_positions = {
            'front_right': [0.0, 0.0, 0.0],  # [coxa, femur, tibia]
            'front_left': [0.0, 0.0, 0.0],
            'back_left': [0.0, 0.0, 0.0],
            'back_right': [0.0, 0.0, 0.0],
        }
        
        # Terminal state for restoration
        self.old_settings = None
        
        # Setup signal handlers for clean exit
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        self.get_logger().info('Spider Teleop Node started. Press W/S/A/D/Q/E to move legs. Ctrl+C to exit.')
        
    def _signal_handler(self, sig, frame):
        """Handle Ctrl+C gracefully."""
        self.get_logger().info('Interrupt received. Cleaning up...')
        self.restore_terminal()
        rclpy.shutdown()
        sys.exit(0)
    
    def setup_terminal(self):
        """Set terminal to non-blocking raw mode."""
        try:
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
        except Exception as e:
            self.get_logger().warning(f'Could not set raw terminal mode: {e}')
    
    def restore_terminal(self):
        """Restore terminal to cooked mode."""
        if self.old_settings:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            except Exception as e:
                self.get_logger().warning(f'Could not restore terminal: {e}')
    
    def getch(self) -> str:
        """
        Non-blocking keyboard input with timeout.
        Returns empty string if no key pressed within timeout.
        """
        try:
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)  # 100ms timeout
            if rlist:
                return sys.stdin.read(1).upper()
        except Exception:
            pass
        return ''
    
    def saturate(self, value: float, joint_type: str) -> float:
        """Saturate joint command to its limits."""
        limits = self.JOINT_LIMITS[joint_type]
        return max(limits['min'], min(limits['max'], value))
    
    def process_keypress(self, key: str):
        """Update joint positions based on keypress."""
        if key == 'W':
            # Femur up
            for leg in self.leg_positions:
                self.leg_positions[leg][1] = self.saturate(
                    self.leg_positions[leg][1] + self.INCREMENT, 'femur'
                )
            self.get_logger().debug('W: Femur +')
        
        elif key == 'S':
            # Femur down
            for leg in self.leg_positions:
                self.leg_positions[leg][1] = self.saturate(
                    self.leg_positions[leg][1] - self.INCREMENT, 'femur'
                )
            self.get_logger().debug('S: Femur -')
        
        elif key == 'A':
            # Coxa left (negative)
            for leg in self.leg_positions:
                self.leg_positions[leg][0] = self.saturate(
                    self.leg_positions[leg][0] - self.INCREMENT, 'coxa'
                )
            self.get_logger().debug('A: Coxa -')
        
        elif key == 'D':
            # Coxa right (positive)
            for leg in self.leg_positions:
                self.leg_positions[leg][0] = self.saturate(
                    self.leg_positions[leg][0] + self.INCREMENT, 'coxa'
                )
            self.get_logger().debug('D: Coxa +')
        
        elif key == 'Q':
            # Tibia retract (negative)
            for leg in self.leg_positions:
                self.leg_positions[leg][2] = self.saturate(
                    self.leg_positions[leg][2] - self.INCREMENT, 'tibia'
                )
            self.get_logger().debug('Q: Tibia -')
        
        elif key == 'E':
            # Tibia extend (positive toward 0)
            for leg in self.leg_positions:
                self.leg_positions[leg][2] = self.saturate(
                    self.leg_positions[leg][2] + self.INCREMENT, 'tibia'
                )
            self.get_logger().debug('E: Tibia +')
    
    def publish_commands(self):
        """Publish current joint positions to all leg controllers."""
        for leg_name, positions in self.leg_positions.items():
            msg = Float64MultiArray()
            msg.data = positions
            self.leg_publishers[leg_name].publish(msg)
    
    def run(self):
        """Main control loop."""
        self.setup_terminal()
        
        try:
            # Control loop at ~10 Hz (100 ms iteration)
            while rclpy.ok():
                # Read keyboard input (non-blocking, 100ms timeout)
                key = self.getch()
                if key:
                    self.process_keypress(key)
                
                # Publish commands
                self.publish_commands()
                
                # Minimal sleep to yield CPU and allow ROS2 to process
                rclpy.spin_once(self, timeout_sec=0.01)
        
        except KeyboardInterrupt:
            self.get_logger().info('Keyboard interrupt received.')
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')
        finally:
            self.restore_terminal()
            self.get_logger().info('Spider Teleop Node shutdown.')

def main(args=None):
    """Entry point for the ROS2 node."""
    rclpy.init(args=args)
    node = SpiderTeleopKey()
    
    try:
        node.run()
    except Exception as e:
        print(f'Fatal error: {e}', file=sys.stderr)
    finally:
        node.restore_terminal()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
