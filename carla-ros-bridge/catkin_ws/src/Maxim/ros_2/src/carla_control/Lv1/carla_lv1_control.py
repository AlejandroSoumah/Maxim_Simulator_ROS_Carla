#!/usr/bin/env python
#
# Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
Welcome to CARLA ROS manual control.

Use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    AD           : steer
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot
    M            : toggle manual transmission
    ,/.          : gear up/down
    B            : toggle manual control

    F1           : toggle HUD
    H/?          : toggle help
    ESC          : quit
"""

from __future__ import print_function
from PIL import Image as IMMG

import datetime
import math
import numpy as np
import rospy
import tf
import os
import csv
import argparse
import collections
import datetime
import glob
import logging
import math
import random
import re
import sys
import weakref
import cv2

import resources.local_planner 
import resources.behavioural_planner 
import resources.controller2d 

from std_msgs.msg import Float32


from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Image
from carla_msgs.msg import CarlaCollisionEvent
from carla_msgs.msg import CarlaLaneInvasionEvent
from carla_msgs.msg import CarlaEgoVehicleControl
from carla_msgs.msg import CarlaEgoVehicleStatus
from carla_msgs.msg import CarlaEgoVehicleInfo
from carla_msgs.msg import CarlaStatus

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_d
    from pygame.locals import K_h
    from pygame.locals import K_m
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_s
    from pygame.locals import K_w
    from pygame.locals import K_b
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')
# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================
NUM_PATHS = 7
BP_LOOKAHEAD_BASE      = 8.0              # m
BP_LOOKAHEAD_TIME      = 2.0              # s
PATH_OFFSET            = 1.5              # m
CIRCLE_OFFSETS         = [-1.0, 1.0, 3.0] # m
CIRCLE_RADII           = [1.5, 1.5, 1.5]  # m
TIME_GAP               = 1.0              # s
PATH_SELECT_WEIGHT     = 10
A_MAX                  = 1.5              # m/s^2
SLOW_SPEED             = 2.0              # m/s
STOP_LINE_BUFFER       = 3.5              # m
LEAD_VEHICLE_LOOKAHEAD = 20.0             # m
LP_FREQUENCY_DIVISOR   = 2  
DIST_THRESHOLD_TO_LAST_WAYPOINT = 2.0
WAYPOINTS_FILENAME = 'paths/course4_waypoints.txt' # waypoint file to load
# Path interpolation parameters
INTERP_MAX_POINTS_PLOT    = 10   # number of points used for displaying
                                 # selected path
INTERP_DISTANCE_RES       = 0.01 # distance between interpolated points

# controller output directory-*
CONTROLLER_OUTPUT_FOLDER = os.path.dirname(os.path.realpath(__file__)) +\
                           'controller_output/'
                           

# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================
on_view_image_global = None

class World(object):
    """
    Handle the rendering
    """

    def __init__(self, role_name, hud):
        self._surface = None
        self.hud = hud
        self.role_name = role_name
        self.image_subscriber = rospy.Subscriber(
            "/carla/{}/camera/rgb/front/image_color".format(self.role_name),
            Image, self.on_view_image)
        self.collision_subscriber = rospy.Subscriber(
            "/carla/{}/collision".format(self.role_name), CarlaCollisionEvent, self.on_collision)
        self.lane_invasion_subscriber = rospy.Subscriber(
            "/carla/{}/lane_invasion".format(self.role_name),
            CarlaLaneInvasionEvent, self.on_lane_invasion)

    def on_collision(self, data):
        """
        Callback on collision event
        """
        intensity = math.sqrt(data.normal_impulse.x**2 +
                              data.normal_impulse.y**2 + data.normal_impulse.z**2)
        self.hud.notification('Collision with {} (impulse {})'.format(
            data.other_actor_id, intensity))

    def on_lane_invasion(self, data):
        """
        Callback on lane invasion event
        """
        text = []
        for marking in data.crossed_lane_markings:
            if marking is CarlaLaneInvasionEvent.LANE_MARKING_OTHER:
                text.append("Other")
            elif marking is CarlaLaneInvasionEvent.LANE_MARKING_BROKEN:
                text.append("Broken")
            elif marking is CarlaLaneInvasionEvent.LANE_MARKING_SOLID:
                text.append("Solid")
            else:
                text.append("Unknown ")
        self.hud.notification('Crossed line %s' % ' and '.join(text))

    def on_view_image(self, image):
        """
        Callback when receiving a camera image
        """
        array = np.frombuffer(image.data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]

        global on_view_image_global
        on_view_image_global = array
        self._surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

    def render(self, display):
        """
        render the current image
        """
        if self._surface is not None:
            display.blit(self._surface, (0, 0))
        self.hud.render(display)

    def destroy(self):
        """
        destroy all objects
        """
        self.image_subscriber.unregister()
        self.collision_subscriber.unregister()
        self.lane_invasion_subscriber.unregister()


# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================

class KeyboardControl(object):
    """
    Handle input events
    """

    def __init__(self, role_name, hud):
        self.role_name = role_name
        self.hud = hud

        self._autopilot_enabled = False
        self._control = CarlaEgoVehicleControl()
        #self._steer_cache = 0.0

        self.vehicle_control_manual_override_publisher = rospy.Publisher(
            "/carla/{}/vehicle_control_manual_override".format(self.role_name),
            Bool, queue_size=1, latch=True)
        self.vehicle_control_manual_override = False
        self.auto_pilot_enable_publisher = rospy.Publisher(
            "/carla/{}/enable_autopilot".format(self.role_name), Bool, queue_size=1)

        self.vehicle_control_publisher = rospy.Publisher(
            "/carla/{}/vehicle_control_cmd_manual".format(self.role_name),
            CarlaEgoVehicleControl)
        self.carla_status_subscriber = rospy.Subscriber(
            "/carla/status", CarlaStatus, self._on_new_carla_frame)

        self.set_autopilot(self._autopilot_enabled)

        self.set_vehicle_control_manual_override(
            self.vehicle_control_manual_override)  # disable manual override

    def __del__(self):
        self.auto_pilot_enable_publisher.unregister()
        self.vehicle_control_publisher.unregister()
        self.vehicle_control_manual_override_publisher.unregister()

    def set_vehicle_control_manual_override(self, enable):
        """
        Set the manual control override
        """
        self.hud.notification('Set vehicle control manual override to: {}'.format(enable))
        self.vehicle_control_manual_override_publisher.publish((Bool(data=enable)))

    def set_autopilot(self, enable):
        """
        enable/disable the autopilot
        """
        self.auto_pilot_enable_publisher.publish(Bool(data=enable))

    # pylint: disable=too-many-branches
    def parse_events(self, clock):
        """
        parse an input event
        """
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_F1:
                    self.hud.toggle_info()
                elif event.key == K_h or (event.key == K_SLASH and
                                          pygame.key.get_mods() & KMOD_SHIFT):
                    self.hud.help.toggle()
                elif event.key == K_b:
                    self.vehicle_control_manual_override = not self.vehicle_control_manual_override
                    self.set_vehicle_control_manual_override(self.vehicle_control_manual_override)
                if event.key == K_q:
                    self._control.gear = 1 if self._control.reverse else -1
                elif event.key == K_m:
                    self._control.manual_gear_shift = not self._control.manual_gear_shift
                    self.hud.notification('%s Transmission' % (
                        'Manual' if self._control.manual_gear_shift else 'Automatic'))
                elif self._control.manual_gear_shift and event.key == K_COMMA:
                    self._control.gear = max(-1, self._control.gear - 1)
                elif self._control.manual_gear_shift and event.key == K_PERIOD:
                    self._control.gear = self._control.gear + 1
                elif event.key == K_p:
                    self._autopilot_enabled = not self._autopilot_enabled
                    self.set_autopilot(self._autopilot_enabled)
                    self.hud.notification('Autopilot %s' % (
                        'On' if self._autopilot_enabled else 'Off'))
        if not self._autopilot_enabled and self.vehicle_control_manual_override:
            self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
            #self._control.reverse = self._control.gear < 0

    def _on_new_carla_frame(self, data):
        """
        callback on new frame

        As CARLA only processes one vehicle control command per tick,
        send the current from within here (once per frame)
        """
        #if not self._autopilot_enabled and self.vehicle_control_manual_override:
        try:
            self.vehicle_control_publisher.publish(self._control)
        except ROSException as error:
            rospy.logwarn("Could not send vehicle control: {}".format(error))

    def _parse_vehicle_keys(self, keys, milliseconds):
        """
        parse key events
        """
        #self._control.throttle = 1.0 if keys[K_UP] or keys[K_w] else 0.0
        #steer_increment = 5e-4 * milliseconds
        #if keys[K_LEFT] or keys[K_a]:
        #    self._steer_cache -= steer_increment
        #elif keys[K_RIGHT] or keys[K_d]:
        #    self._steer_cache += steer_increment
        #else:
        #    self._steer_cache = 0.0
        #self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        #self._control.steer = round(self._steer_cache, 1)
        #self._control.brake = 1.0 if keys[K_DOWN] or keys[K_s] else 0.0
        #self._control.hand_brake = keys[K_SPACE]

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class HUD(object):
    """
    Handle the info display
    """

    def __init__(self, role_name, width, height):
        self.role_name = role_name
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        fonts = [x for x in pygame.font.get_fonts() if 'mono' in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 24), width, height)
        self._show_info = True
        self._info_text = []
        self.vehicle_status = CarlaEgoVehicleStatus()
        self.tf_listener = tf.TransformListener()
        self.vehicle_status_subscriber = rospy.Subscriber(
            "/carla/{}/vehicle_status".format(self.role_name),
            CarlaEgoVehicleStatus, self.vehicle_status_updated)
        self.vehicle_info = CarlaEgoVehicleInfo()
        self.vehicle_info_subscriber = rospy.Subscriber(
            "/carla/{}/vehicle_info".format(self.role_name),
            CarlaEgoVehicleInfo, self.vehicle_info_updated)
        self.latitude = 0
        self.longitude = 0
        self.manual_control = False
        self.gnss_subscriber = rospy.Subscriber(
            "/carla/{}/gnss/gnss1/fix".format(self.role_name), NavSatFix, self.gnss_updated)
        self.manual_control_subscriber = rospy.Subscriber(
            "/carla/{}/vehicle_control_manual_override".format(self.role_name),
            Bool, self.manual_control_override_updated)

        self.carla_status = CarlaStatus()
        self.status_subscriber = rospy.Subscriber(
            "/carla/status", CarlaStatus, self.carla_status_updated)

    def __del__(self):
        self.gnss_subscriber.unregister()
        self.vehicle_status_subscriber.unregister()
        self.vehicle_info_subscriber.unregister()

    def tick(self, clock):
        """
        tick method
        """
        self._notifications.tick(clock)

    def carla_status_updated(self, data):
        """
        Callback on carla status
        """
        self.carla_status = data
        self.update_info_text()

    def manual_control_override_updated(self, data):
        """
        Callback on vehicle status updates
        """
        self.manual_control = data.data
        self.update_info_text()

    def vehicle_status_updated(self, vehicle_status):
        """
        Callback on vehicle status updates
        """
        self.vehicle_status = vehicle_status
        self.update_info_text()

    def vehicle_info_updated(self, vehicle_info):
        """
        Callback on vehicle info updates
        """
        self.vehicle_info = vehicle_info
        self.update_info_text()

    def gnss_updated(self, data):
        """
        Callback on gnss position updates
        """
        self.latitude = data.latitude
        self.longitude = data.longitude
        self.update_info_text()

    def update_info_text(self):
        """
        update the displayed info text
        """
        if not self._show_info:
            return
        try:
            (position, quaternion) = self.tf_listener.lookupTransform(
                '/map', self.role_name, rospy.Time())
            _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
            yaw = -math.degrees(yaw)
            x = position[0]
            y = -position[1]
            z = position[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            x = 0
            y = 0
            z = 0
            yaw = 0
        heading = 'N' if abs(yaw) < 89.5 else ''
        heading += 'S' if abs(yaw) > 90.5 else ''
        heading += 'E' if 179.5 > yaw > 0.5 else ''
        heading += 'W' if -0.5 > yaw > -179.5 else ''
        fps = 0
        if self.carla_status.fixed_delta_seconds:
            fps = 1 / self.carla_status.fixed_delta_seconds
        self._info_text = [
            'Frame: % 22s' % self.carla_status.frame,
            'Simulation time: % 12s' % datetime.timedelta(
                seconds=int(rospy.get_rostime().to_sec())),
            'FPS: % 24.1f' % fps,
            '',
            'Vehicle: % 20s' % ' '.join(self.vehicle_info.type.title().split('.')[1:]),
            'Speed:   % 15.0f km/h' % (3.6 * self.vehicle_status.velocity),
            u'Heading:% 16.0f\N{DEGREE SIGN} % 2s' % (yaw, heading),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (x, y)),
            'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (self.latitude, self.longitude)),
            'Height:  % 18.0f m' % z,
            '']
        self._info_text += [
            ('Throttle:', self.vehicle_status.control.throttle, 0.0, 1.0),
            ('Steer:', self.vehicle_status.control.steer, -1.0, 1.0),
            ('Brake:', self.vehicle_status.control.brake, 0.0, 1.0),
            ('Reverse:', self.vehicle_status.control.reverse),
            ('Hand brake:', self.vehicle_status.control.hand_brake),
            ('Manual:', self.vehicle_status.control.manual_gear_shift),
            'Gear:        %s' % {-1: 'R', 0: 'N'}.get(self.vehicle_status.control.gear,
                                                      self.vehicle_status.control.gear),
            '']
        self._info_text += [('Manual ctrl:', self.manual_control)]
        if self.carla_status.synchronous_mode:
            self._info_text += [('Sync mode running:', self.carla_status.synchronous_mode_running)]
        self._info_text += ['', '', 'Press <H> for help']

    def toggle_info(self):
        """
        show/hide the info text
        """
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        """
        display a notification for x seconds
        """
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        """
        display an error
        """
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        """
        render the display
        """
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30)
                                  for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset + 50, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect(
                                (bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            f = 0.0
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        self.help.render(display)

# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
    """
    Support Class for info display, fade out text
    """

    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        """
        set the text
        """
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, clock):
        """
        tick for fading
        """
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        """
        render the fading
        """
        display.blit(self.surface, self.pos)

# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
    """
    Show the help text
    """

    def __init__(self, font, width, height):
        lines = __doc__.split('\n')
        self.font = font
        self.dim = (680, len(lines) * 22 + 12)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, n * 22))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        """
        Show/hide the help
        """
        self._render = not self._render

    def render(self, display):
        """
        render the help
        """
        if self._render:
            display.blit(self.surface, self.pos)

# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================

def servo_fine():
    throttle_pub = rospy.Publisher('acceleratorServo', Float32, queue_size=10)
    break_pub = rospy.Publisher('breakServo', Float32, queue_size=10)
    steer_pub = rospy.Publisher('steeringServo', Float32, queue_size=10)
    #rospy.init_node('servo_controller', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        code_control = CarlaEgoVehicleControl()
        throttle_angle = code_control.throttle * 270
        steer_angle = code_control.steer * 270
        break_angle = code_control.brake * 270

        rospy.loginfo(throttle_angle)
        rospy.loginfo(break_angle)
        rospy.loginfo(steer_angle)

        throttle_pub.publish(throttle_angle)
        break_pub.publish(break_angle)
        steer_pub.publish(steer_angle)

        rate.sleep()
view_bgr_image = None
top_bgr_image = None
def on_view_view_image(image):
    array = np.frombuffer(image.data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    #array = array[:, :, ::-1]
    global view_bgr_image
    view_bgr_image =  array
    #view_bgr_image_pillow = IMMG.fromarray(array)
    #view_bgr_image_pillow.save("1221599.jpg")
    #print(os.path.dirname(os.path.abspath(__file__)))

def on_view_top_image(image):
    array = np.frombuffer(image.data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    #array = array[:, :, ::-1]
    global top_bgr_image
    top_bgr_image =  array

    

def main():
    """
    main function
    """
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    code_control = CarlaEgoVehicleControl()
    vehicle_control_publisher = rospy.Publisher(
            "/carla/{}/vehicle_control_cmd_manual".format(role_name),
            CarlaEgoVehicleControl, queue_size=1)
    rospy.init_node('carla_manual_control', anonymous=True)




    # resolution should be similar to spawned camera with role-name 'view'
    view_subscriber = rospy.Subscriber(
            "/carla/{}/camera/rgb/view/image_color".format(role_name),
            Image, on_view_view_image)
    top_subscriber = rospy.Subscriber(
            "/carla/{}/camera/rgb/top/image_color".format(role_name),
            Image, on_view_top_image)

    throttle_pub = rospy.Publisher('acceleratorServo', Float32, queue_size=1)
    break_pub = rospy.Publisher('breakServo', Float32, queue_size=1)
    steer_pub = rospy.Publisher('steeringServo', Float32, queue_size=1)

    resolution = {"width": 1920, "height": 1080}

    pygame.init()
    pygame.font.init()
    pygame.display.set_caption("CARLA ROS manual control")
    world = None
    
    skip_first_frame = True
    prev_timestamp = 0
    tot_target_reached = 0
    num_min_waypoints = 21
    try:

        start_timestamp = 0

        display = pygame.display.set_mode(
            (resolution['width'], resolution['height']),
            pygame.HWSURFACE | pygame.DOUBLEBUF)
        hud = HUD(role_name, resolution['width'], resolution['height'])
        world = World(role_name, hud)
        controllerK = KeyboardControl(role_name, hud)
        clock = pygame.time.Clock()


        waypoints_file = WAYPOINTS_FILENAME

        waypoints_filepath =\
                os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                                WAYPOINTS_FILENAME)
        waypoints_np   = None
        with open(waypoints_filepath) as waypoints_file_handle:
            waypoints = list(csv.reader(waypoints_file_handle, 
                                        delimiter=',',
                                        quoting=csv.QUOTE_NONNUMERIC))
            waypoints_np = np.array(waypoints)
            
        
        wp_goal_index   = 0
        local_waypoints = None
        path_validity   = np.zeros((NUM_PATHS, 1), dtype=bool)
        controller = resources.controller2d.Controller2D(waypoints)
        bp = resources.behavioural_planner.BehaviouralPlanner(BP_LOOKAHEAD_BASE)
        lp = resources.local_planner.LocalPlanner(NUM_PATHS,
                                PATH_OFFSET,
                                CIRCLE_OFFSETS,
                                CIRCLE_RADII,
                                PATH_SELECT_WEIGHT,
                                TIME_GAP,
                                A_MAX,
                                SLOW_SPEED,
                                STOP_LINE_BUFFER)

        clock = pygame.time.Clock()
        frame = 0
        current_timestamp = start_timestamp
        reached_the_end = False

        tx = 0
        ty = 2.0
        tz = 2.0

        CameraFOV = 100
        Cu = 1920 / 2
        Cv = 1080/ 2
        f  = 1920/(2 * np.tan(CameraFOV * np.pi / 360))
        K = [[f, 0, Cu],
            [0, f, Cv],
            [0, 0, 1 ]]
        K_np = np.asmatrix(K)
        t = [[1,0,0,tx],
            [0,1,0,ty],
            [0,0,1,tz]]
         
        t_np = np.asmatrix(t)
        while not rospy.core.is_shutdown():
            vehicle_control_publisher.publish(code_control)
            frame = frame +1

            clock.tick_busy_loop(60)
            if controllerK.parse_events(clock):
                return
            hud.tick(clock)
            world.render(display)
            pygame.display.flip()

            ##transform = world.role_name.get_transform()
            ##vel = world.role_name.get_velocity()
            prev_timestamp = current_timestamp
            current_timestamp = rospy.get_time()

            ##control = world.role_name.get_control()
            try:
                tf_listener = hud.tf_listener
                (position, quaternion) = tf_listener.lookupTransform('/map', role_name, rospy.Time())

                _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)

                yaw = -math.degrees(yaw)
                x = position[0]
                y = -position[1]


            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                x = 0
                y = 0
                z = 0
                yaw = 0

            current_x = x
            current_y = y
            current_yaw = np.radians(yaw)
            velocity = hud.vehicle_status.velocity
            current_speed = velocity
            open_loop_speed = lp._velocity_planner.get_open_loop_speed(current_timestamp - prev_timestamp) 
            #open_loop_speed = lp._velocity_planner.get_open_loop_speed(current_timestamp - prev_timestamp)
 
            if frame % 5 == 0:
               # try:
                #    cv2.imshow("view_image",view_bgr_image)
                 #   cv2.imshow("top image",top_bgr_image)
                  #  #cv2.imshow("top image",top_bgr_image)
                   # 
                    #cv2.waitKey(1)
                    
                #except Exception as e:
                 #      print(e)

                #Lane detection

                ego_state = [current_x+2, current_y, current_yaw, open_loop_speed]
                print(ego_state)
                bp.set_lookahead(BP_LOOKAHEAD_BASE + BP_LOOKAHEAD_TIME * open_loop_speed)


                bp.transition_state(waypoints,waypoints, ego_state, current_speed)

                goal_state_set = lp.get_goal_state_set(bp._goal_index, bp._goal_state, waypoints, ego_state)
                #print("goal_state_set",goal_state_set)
                paths_vehicle_coordinate, path_validity = lp.plan_paths(goal_state_set)

               # print("Paths_local",paths_vehicle_coordinate)
                paths = resources.local_planner.transform_paths(paths_vehicle_coordinate, ego_state)

                #print("Paths_global",paths)
                best_index = lp._collision_checker.select_best_path_index(paths, bp._goal_state_hyp)

                ################################################ Compute paths visualize#####################
                #Whole Code
   #             try:
  #           
 #                   paths_vehicle_coordinate_np = np.array(paths_vehicle_coordinate)
#
 #                   Each_Path_length = len(paths_vehicle_coordinate_np[0][0])
#
 #                   pixel_coordinate_paths = []
#
                  #  for x in range(0,NUM_PATHS):
                 #       for i in range(0,Each_Path_length):
                #            world_coordinate = [paths_vehicle_coordinate_np[x][1][i],1,paths_vehicle_coordinate_np[x][0][i],1]
               #             world_coordinate_np = np.asmatrix(world_coordinate)
              #              camera_coordinates = np.dot(np.dot(K_np,t),world_coordinate_np.T)
             #               pixel_coordinates = [[camera_coordinates[0] / camera_coordinates[2]],
            #                                [camera_coordinates[1] / camera_coordinates[2]]
           #                                 ]
          #                  pixel_coordinate_paths.append(pixel_coordinates)
         #           pixel_coordinate_paths_np = np.array(pixel_coordinate_paths)
        #            pixel_plot = pixel_coordinate_paths_np[:,:,0, 0, 0]
       #             pix_1 = pixel_plot[0:49]
      #              pix_2 = pixel_plot[49:98]
     #               pix_3 = pixel_plot[98:147]
    #                pix_4 = pixel_plot[147:196]
   #                 pix_5 = pixel_plot[196:245]
  #                  pix_6 = pixel_plot[245:294]
 #                   pix_7 = pixel_plot[294:343]
#
     #               line_1 = np.array((pix_1.T[0],pix_1.T[1]))
    #                line_1 = pix_1.astype(int)
   #                 line_1 = line_1.reshape((-1,1,2))
  #                  
  #                  line_2 = pix_2.astype(int)
 #                   line_2 = line_2.reshape((-1,1,2))
#
 #                   line_3 = pix_3.astype(int)
#                    line_3 = line_3.reshape((-1,1,2))

                    #line_4 = pix_4.astype(int)
                    #line_4 = line_4.reshape((-1,1,2))

                    #line_5 = pix_5.astype(int)
                    #line_5 = line_5.reshape((-1,1,2))

                    #line_6 = pix_6.astype(int)
                    #line_6 = line_6.reshape((-1,1,2))

                    #line_7 = pix_7.astype(int)
                   # line_7 = line_7.reshape((-1,1,2))
                  #  on_view_image_global_np = np.array(on_view_image_global)
                 #   view_bgr_image_np = np.array(on_view_image_global)
                #    line_array = [line_1,line_2,line_3,line_4,line_5,line_6,line_7]
               #     print(line_array)
              #      print("works 1")
             #       image = cv2.polylines(on_view_image_global_np,[line_1,line_2,line_3,line_4,line_5,line_6,line_7],
            #                            isClosed=False,
           #                             color=(255, 255, 255),
          #                              thickness=10,
         #                               lineType=cv2.LINE_AA)
        #            print("Works here")
       #             image = cv2.polylines(image,[line_array[best_index]],
      #                                  isClosed=False,
     #                                   color=(0, 255, 0),
    #                                    thickness=10,
   #                                     lineType=cv2.LINE_AA)
  #                  cv2.imshow("front image",image[:,:,::-1])
 #                   cv2.waitKey(1)
#
                #except Exception as e:
                #    print(e)


                try:
                    if best_index == None:
                        best_path = lp._prev_best_path
                    else:
                        best_path = paths[best_index]
                        lp._prev_best_path = best_path
                except:
                    continue

                desired_speed = bp._goal_state[2]
                decelerate_to_stop = False
                print("Desired_speed", desired_speed)
                print("Current_speed", current_speed)
                local_waypoints = lp._velocity_planner.compute_velocity_profile(best_path, desired_speed, ego_state, current_speed)
                print("local_waypoints",local_waypoints)
                if local_waypoints != None:

                    wp_distance = []   # distance array
                    local_waypoints_np = np.array(local_waypoints)
                    for i in range(1, local_waypoints_np.shape[0]):
                        wp_distance.append(
                                np.sqrt((local_waypoints_np[i, 0] - local_waypoints_np[i-1, 0])**2 +
                                        (local_waypoints_np[i, 1] - local_waypoints_np[i-1, 1])**2))
                    wp_distance.append(0)              

                    wp_interp      = []   
                                          
                    for i in range(local_waypoints_np.shape[0] - 1):

                        wp_interp.append(list(local_waypoints_np[i]))

                        num_pts_to_interp = int(np.floor(wp_distance[i] /\
                                                     float(INTERP_DISTANCE_RES)) - 1)
                        wp_vector = local_waypoints_np[i+1] - local_waypoints_np[i]
                        wp_uvector = wp_vector / np.linalg.norm(wp_vector[0:2])

                        for j in range(num_pts_to_interp):
                            next_wp_vector = INTERP_DISTANCE_RES * float(j+1) * wp_uvector
                            wp_interp.append(list(local_waypoints_np[i] + next_wp_vector))

                    wp_interp.append(list(local_waypoints_np[-1]))
                    

                    controller.update_waypoints(wp_interp)
                    pass
            if local_waypoints != None and local_waypoints != []:

                controller.update_values(current_x, current_y, current_yaw, 
                                         current_speed,
                                         current_timestamp, frame)
                controller.update_controls()
                cmd_throttle, cmd_steer, cmd_brake = controller.get_commands()
            else:
                cmd_throttle = 0.0
                cmd_steer = 0.0
                cmd_brake = 0.0


            if skip_first_frame and frame == 0:
                pass
            elif local_waypoints == None:
                pass
            else:
               ## if i % 5 == 0:
                wp_interp_np = np.array(wp_interp)
                path_indices = np.floor(np.linspace(0, 
                                                    wp_interp_np.shape[0]-1,
                                                    INTERP_MAX_POINTS_PLOT))

            #if len(agent.get_local_planner().waypoints_queue) < num_min_waypoints and args.loop:
            #    agent.reroute(spawn_points)
            #    tot_target_reached += 1
            #    world.hud.notification("The target has been reached " +
            #                            str(tot_target_reached) + " times.", seconds=4.0)

            #elif len(agent.get_local_planner().waypoints_queue) == 0 and not args.loop:
            #    print("Target reached, mission accomplished...")
            #   break

            #speed_limit = world.role_name.get_speed_limit()
            #agent.get_local_planner().set_speed(speed_limit)

            #cmd_throttle = 0.4
            #cmd_steer = 0
            #cmd_brake = 0
            dist_to_last_waypoint = np.linalg.norm(np.array([
                waypoints[-1][0] - current_x,
                waypoints[-1][1] - current_y]))
            if  dist_to_last_waypoint < DIST_THRESHOLD_TO_LAST_WAYPOINT:
                reached_the_end = True
            if reached_the_end:
                cmd_steer = np.fmax(np.fmin(0, 1.0), -1.0)
                cmd_throttle = np.fmax(np.fmin(0, 1.0), 0)
                cmd_brake = np.fmax(np.fmin(0.1, 1.0), 0)
                break
            #cmd_steer = np.fmax(np.fmin(cmd_steer, 1.0), -1.0)
            kmd_throttle = np.fmax(np.fmin(cmd_throttle, 1.0), 0)
            #cmd_brake = np.fmax(np.fmin(cmd_brake, 1.0), 0)
            print("Throttle before", cmd_throttle)
            print("Throttle after", kmd_throttle)
           

            code_control.throttle = cmd_throttle
            code_control.steer = cmd_steer
            code_control.brake = cmd_brake
            code_control.gear = 3

            vehicle_control_publisher.publish(code_control)
            print("Code control",code_control)


            throttle_angle = code_control.throttle * 135 #Kind of makes sense but the maximum is 130 degrees lets say random guess :) #Ill have to change the zeros
            break_angle = code_control.brake * 135
            steer_angle = ( code_control.steer * -135 ) + 95 #here we state that 0 is the 95 degrees so anything below tends towards 0 and viceversa

            print("steer",cmd_steer)

            throttle_pub.publish(throttle_angle)
            break_pub.publish(break_angle)
            steer_pub.publish(steer_angle)

            #world.role_name.apply_control(carla.VehicleControl(throttle=cmd_throttle,
                  #                                           steer=cmd_steer,
                 #                                           brake=cmd_brake))

 

        if reached_the_end:
            print("Reached the end of path. Writing to controller_output...")
        else:
            print("Exceeded assessment time. Writing to controller_output...")


    finally:
        if world is not None:
            world.destroy()
        pygame.quit()


if __name__ == '__main__':

    main()
