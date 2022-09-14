from email.mime import image
import os
import sys
import pygame
from pygame.locals import Color

from rclpy.node import Node
from rclpy.node import Rate
from rclpy import qos
from sensor_msgs.msg import Joy
from iotbot_interface.msg import Battery
from iotbot_interface.msg import RotationSpeed
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

# Screen size
_size = width, height = 640, 480

##
#@class EduVirtualJoy
#@brief Minimalistic GUI for steering and visualizing different robot types
#@author Stefan May
#@date 1.11.2021
class EduVirtualJoy(Node):
    def __init__(self):
        super().__init__('edu_virtual_joy')
        pygame.init()

        self._surface         = pygame.display.set_mode(_size, pygame.HWSURFACE | pygame.DOUBLEBUF)

        image_path            = '../../../../share/edu_virtual_joy/images/'
        path_logo             = os.path.join(os.path.dirname(__file__), image_path + "Logo_A_32.png")
        path_eduart           = os.path.join(os.path.dirname(__file__), image_path + "Logo_Edu_100.png")
        logo                  = pygame.image.load(path_logo)
        self._eduart          = pygame.image.load(path_eduart)
        self._eduart          = self._eduart.convert_alpha()
        pygame.display.set_caption("EduArt Virtual Joystick")
        pygame.display.set_icon(logo)

        topic_namespace       = "iotbot"
        self.declare_parameter("mecanum", 0)
        self._mecanum         = self.get_parameter("mecanum").get_parameter_value().integer_value

        if self._mecanum == 1:
            path_robot        = os.path.join(os.path.dirname(__file__), image_path + "iotbot_mecanum_top_vga.png")
        else:
            path_robot        = os.path.join(os.path.dirname(__file__), image_path + "iotbot_offroad_top_vga.png")
        path_arrow_straight   = os.path.join(os.path.dirname(__file__), image_path + "arrow_straight_small.png")
        path_arrow_curved     = os.path.join(os.path.dirname(__file__), image_path + "arrow_curved_small.png")
        self._robot           = pygame.image.load(path_robot)
        self._arrow_front     = pygame.image.load(path_arrow_straight)
        self._arrow_back      = pygame.transform.rotozoom(self._arrow_front, 180.0, 1.0)
        self._arrow_left      = pygame.transform.rotozoom(self._arrow_front,  90.0, 1.0)
        self._arrow_right     = pygame.transform.rotozoom(self._arrow_front, -90.0, 1.0)
        self._arrow_curved    = pygame.image.load(path_arrow_curved)

        self._voltage         = 0
        self._sub_voltage     = self.create_subscription(Battery, topic_namespace + '/battery', self.callback_voltage, 2)
        self._tof_front_left  = 0
        self._tof_front_right = 0
        self._tof_rear_left   = 0
        self._tof_rear_right  = 0
        self._sub_tof         = self.create_subscription(Float32MultiArray, topic_namespace + '/tof', self.callback_tof, 2)
        self._throttle        = 0.3
        self._sub_rpm         = self.create_subscription(RotationSpeed, topic_namespace + '/rpm', self.callback_rpm, 2)
        self._rpm             = [0, 0, 0, 0]
        self._pub             = self.create_publisher(Joy, '/joy', qos_profile=qos.qos_profile_system_default)


        # main method is called by timer, rate = 10 Hz
        self._timer = self.create_timer(0.1, self.process)

    def __del__(self):
        pass

    def renderText(self, text, coords, fontsize=20, rgb=(0, 0, 0), align_left=0):
        font = pygame.font.SysFont("arial", fontsize) 
        text = font.render(text, True, rgb)
        textrect = text.get_rect()
        textrect.centerx = coords[0]
        textrect.centery = coords[1]
        if align_left==1:
            textrect.centerx += textrect.width/2
        self._surface.blit(text, textrect)

    def renderTofDistance(self, dist, x, y, name):
        rgb=(0, 0, 0)
        textTof      = "{:.2f}".format(dist)
        rectTof      = pygame.Rect(x, y, 0, 0)
        if(dist<0.5):
            rgb=(160, 20, 20)
        self.renderText(name+"="+textTof+"m", rectTof, 16, rgb)

    def renderRPM(self, rpm, x, y):
        rgb=(0, 0, 0)
        textRPM      = "{:.2f}".format(rpm)
        rectRPM      = pygame.Rect(x, y, 0, 0)
        self.renderText(str(int(rpm))+" U/min", rectRPM, 16, rgb)

    def callback_voltage(self, data):
        self._voltage = data.voltage

    def callback_tof(self, data):
        self._tof_front_left  = data.data[0]
        self._tof_front_right = data.data[1]
        self._tof_rear_left   = data.data[2]
        self._tof_rear_right  = data.data[3]

    def callback_rpm(self, data):
        self._rpm[0]  = data.front_left_rpm
        self._rpm[1]  = data.front_right_rpm
        self._rpm[2]  = data.rear_left_rpm
        self._rpm[3]  = data.rear_right_rpm

    def process(self):
        joyMsg = Joy()
        joyMsg.header.frame_id = "joy"
        joyMsg.axes.append(0) # Axis move left/right
        joyMsg.axes.append(0) # Axis backward/forward
        joyMsg.axes.append(0) # Axis turn left/right
        joyMsg.axes.append(self._throttle-1.0) # Throttle
        for i in range(12):
            joyMsg.buttons.append(0)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_c and pygame.key.get_mods() & pygame.KMOD_CTRL:
                    sys.exit()
                elif event.key == pygame.K_PLUS:
                    self._throttle += 0.1
                    if self._throttle > 1.0:
                        self._throttle = 1.0
                elif event.key == pygame.K_MINUS:
                    self._throttle -= 0.1
                    if self._throttle < 0.0:
                        self._throttle = 0.0
        joyMsg.header.stamp = self.get_clock().now().to_msg()
        pygame.event.pump()
        keys = pygame.key.get_pressed()
        if self._mecanum == 1:
            joyMsg.axes[0] = keys[pygame.K_y] - keys[pygame.K_c]
        else:
            joyMsg.axes[0] = 0
        joyMsg.axes[1] = keys[pygame.K_w] - keys[pygame.K_s]
        joyMsg.axes[2] = keys[pygame.K_a] - keys[pygame.K_d]
        joyMsg.axes[3] = self._throttle*2-1.0
        joyMsg.buttons[0] = keys[pygame.K_1]
        joyMsg.buttons[1] = keys[pygame.K_2]
        joyMsg.buttons[2] = keys[pygame.K_3]
        joyMsg.buttons[3] = keys[pygame.K_4]
        joyMsg.buttons[4] = keys[pygame.K_5]
        joyMsg.buttons[5] = keys[pygame.K_6]
        joyMsg.buttons[6] = keys[pygame.K_7]
        joyMsg.buttons[7] = keys[pygame.K_8]
        joyMsg.buttons[8] = keys[pygame.K_9]
        joyMsg.buttons[9] = keys[pygame.K_0]
        joyMsg.buttons[10] = keys[pygame.K_e]
        joyMsg.buttons[11] = self._mecanum
        self._pub.publish(joyMsg)

        white = Color('white')
        self._surface.fill(white)
        self._surface.blit(self._robot, self._robot.get_rect())         

        # EduArt logo
        rectEdu = self._eduart.get_rect().move([ 2 , self._surface.get_height()-23 ])
        self._surface.blit(self._eduart, rectEdu)

        # Arrow forward movement
        rectFront = self._arrow_front.get_rect().move([ (self._surface.get_width() - self._arrow_front.get_width()) / 2, 0 ] )
        self._surface.blit(self._arrow_front, rectFront)
        self.renderText("w", [rectFront.centerx, rectFront.centery])

        # Arrow backward movement
        rectBack = self._arrow_back.get_rect().move([ (self._surface.get_width() - self._arrow_back.get_width()) / 2, self._surface.get_height() - self._arrow_back.get_height() ])
        self._surface.blit(self._arrow_back, rectBack)
        self.renderText("s", [rectBack.centerx, rectBack.centery])

        if self._mecanum == 1:
            # Arrow left side
            rectLeft = self._arrow_left.get_rect().move([ (self._surface.get_width() - self._arrow_left.get_width()) / 2 - 200, (self._surface.get_height() - self._arrow_left.get_height()) / 2 ])
            self._surface.blit(self._arrow_left, rectLeft)
            self.renderText("y", [rectLeft.centerx, rectLeft.centery])

        if self._mecanum == 1:
            # Arrow right side
            rectRight = self._arrow_right.get_rect().move([ (self._surface.get_width() - self._arrow_right.get_width()) / 2 + 200, (self._surface.get_height() - self._arrow_right.get_height()) / 2 ])
            self._surface.blit(self._arrow_right, rectRight)
            self.renderText("c", [rectRight.centerx, rectRight.centery])

        # Arrow rotational movement
        rectRotation = self._arrow_curved.get_rect().move([ (self._surface.get_width() - self._arrow_curved.get_width()) / 2, (self._surface.get_height() - self._arrow_curved.get_height()) / 2 ])
        self._surface.blit(self._arrow_curved, rectRotation)
        self.renderText("a | d", [rectRotation.centerx, rectRotation.centery])

        coordsHelpText = [self._surface.get_width() - 110, 10]
        self.renderText("e: enable", coordsHelpText, 16, (0, 0, 0), 1)
        coordsHelpText[1] += 25
        self.renderText("0: disable", coordsHelpText, 16, (0, 0, 0), 1)
        coordsHelpText[1] += 25
        self.renderText("[1-6]: lighting", coordsHelpText, 16, (0, 0, 0), 1)
        coordsHelpText[1] += 25
        textThrottle = "{:.1f}".format(round(self._throttle,1))
        self.renderText("-/+: throttle="+textThrottle, coordsHelpText, 16, (0, 0, 0), 1)

        textVoltage  = str(round(self._voltage,1))
        coordsVoltage  = [self._surface.get_width()/2, 120]
        self.renderText("U="+textVoltage+"V", coordsVoltage, 16)

        self.renderTofDistance(self._tof_front_left,  self._surface.get_width()/2-80, 30, "d1")
        self.renderTofDistance(self._tof_front_right, self._surface.get_width()/2+80, 30, "d2")
        self.renderTofDistance(self._tof_rear_left,  self._surface.get_width()/2-80, self._surface.get_height()-30, "d3")
        self.renderTofDistance(self._tof_rear_right, self._surface.get_width()/2+80, self._surface.get_height()-30, "d4")

        self.renderRPM(self._rpm[0], self._surface.get_width()/2+260, self._surface.get_height()-100)
        self.renderRPM(self._rpm[1], self._surface.get_width()/2+260, 120)
        self.renderRPM(self._rpm[2], self._surface.get_width()/2-260, self._surface.get_height()-100)
        self.renderRPM(self._rpm[3], self._surface.get_width()/2-260, 120)

        pygame.display.update()
