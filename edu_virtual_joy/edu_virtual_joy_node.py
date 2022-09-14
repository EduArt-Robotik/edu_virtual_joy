from .eduVirtualJoy import EduVirtualJoy

import rclpy

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(EduVirtualJoy())

if __name__ == '__main__':
    main()
