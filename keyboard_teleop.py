#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64
import sys, select, termios, tty

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        
        self.brake_pub = self.create_publisher(Float64, '/brake_command', 1)
        self.gear_pub = self.create_publisher(String, '/gear_command', 1)
        self.steering_pub = self.create_publisher(Float64, '/steering_command', 1)
        self.throttle_pub = self.create_publisher(Float64, '/throttle_command', 1)
        
        #Beállitások
        self.THROTTLE_STEP = 0.05      
        self.STEERING_STEP = 0.15  
        self.CENTERING_STEP = 0.15 

        self.current_throttle = 0.0      
        self.current_steering = 0.0      
        self.current_brake = 0.0         
        self.current_gear = 'forward'    
        
        self.publish_gear()
        
        self.get_logger().info('Billentyűzetes vezérlés aktív. Használd a gombokat...')

    def publish_gear(self):
        msg = String()
        msg.data = self.current_gear
        self.gear_pub.publish(msg)

    def publish_commands(self):
        self.throttle_pub.publish(Float64(data=self.current_throttle))
        self.steering_pub.publish(Float64(data=self.current_steering))
        self.brake_pub.publish(Float64(data=self.current_brake))
        
        #Állapot
        print(f"\rÁllapot -> Váltó: {self.current_gear} | Gáz: {self.current_throttle:.2f} | Kormány: {self.current_steering:.2f} | Fék: {self.current_brake:.1f}   ", end="")


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

settings = termios.tcgetattr(sys.stdin)

def main(args=None):
    rclpy.init(args=args)
    teleop = KeyboardTeleop()
    
    print("------------------------------------------------")
    print(" BILLENTYŰZETES IRÁNYÍTÁS (WASD)")
    print("------------------------------------------------")
    print("   w : Gáz")
    print("   s : Lassítás / Fék")
    print("   a : Balra")
    print("   d : Jobbra")
    print("   x : Kilépés")
    print("------------------------------------------------")

    try:
        while rclpy.ok():
            key = getKey()
            
            #gáz
            if key == 'w':
                teleop.current_brake = 0.0
                teleop.current_throttle = min(teleop.current_throttle + teleop.THROTTLE_STEP, 1.0)

            elif key == 's':
                if teleop.current_throttle > 0.01:
                    teleop.current_throttle = max(teleop.current_throttle - teleop.THROTTLE_STEP, 0.0)
                else:
                    teleop.current_brake = 1.0
            
            elif key == 'x':
                break

            #kormányzás
            if key == 'a':
                teleop.current_steering = max(teleop.current_steering - teleop.STEERING_STEP, -1.0)
            
            elif key == 'd':
                teleop.current_steering = min(teleop.current_steering + teleop.STEERING_STEP, 1.0)
            
            else:
                # középrehúzás
                if teleop.current_steering > 0.01:
                    teleop.current_steering = max(0.0, teleop.current_steering - teleop.CENTERING_STEP)
                elif teleop.current_steering < -0.01:
                    teleop.current_steering = min(0.0, teleop.current_steering + teleop.CENTERING_STEP)
                else:
                    teleop.current_steering = 0.0

            teleop.publish_commands()

    except Exception as e:
        print(e)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        teleop.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()