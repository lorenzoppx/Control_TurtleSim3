# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist,Pose
from std_msgs.msg import String
from nav_msgs.msg import Odometry

import numpy as np
import os

class Minimal(Node):
    def __init__(self):
        super().__init__('turtle_control')
        self.pose_goal = Pose()
        self.pose = Pose()
        self.init_  = self.init_variables()
        self.subs = self.init_subscriber()
        self.pubs = self.init_publisher()
        
    def init_variables(self):
        self.pose_goal.orientation.z = 0.0
        self.here_goal = 0
        self.poses = [[-1.0,1.5],[-2.0,-0.1],[-1.5,-0.7]]

    def goal_callback(self):
        msg = Pose()
        pos_desejada = self.poses[self.here_goal]
        # posicao atual
        pos_atual = np.array([self.pose.position.x,self.pose.position.y])
        # conversao em coordenadas polares
        erro = pos_desejada - pos_atual
        ro = np.linalg.norm(erro)
        if ro<0.2:
            self.get_logger().info('Here on goal X: "%s"' % self.poses[self.here_goal][0])
            self.get_logger().info('Here on goal Y: "%s"' % self.poses[self.here_goal][1])
            self.here_goal = self.here_goal + 1
        else:
            self.get_logger().info('Not on goal')      
        msg.position.x = self.poses[self.here_goal][0]
        msg.position.y = self.poses[self.here_goal][1]
        self.publisher_goal.publish(msg)
        

    def init_publisher(self):
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.pub_callback)
        self.i = 0
        self.publisher_goal = self.create_publisher(Pose, '/goal', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.goal_callback)
        return self.publisher_,self.publisher_goal

    def pub_callback(self):
        msg = Twist()
        # posicao desejada
        #self.xd = 2
        #self.yd = 9
        pos_desejada = self.poses[self.here_goal]
        # posicao atual
        pos_atual = np.array([self.pose.position.x,self.pose.position.y])
        # conversao em coordenadas polares
        erro = pos_desejada - pos_atual
        ro = np.linalg.norm(erro)
        alpha = np.arctan2(erro[1],erro[0]) - self.pose.orientation.z

        if ro>0.2:
            # regra de controle
            vmax = 0.7*1/10
            vel_lin = vmax*np.tanh(ro)
            kw = 0.6*1/10
            vel_ang = -kw*alpha

            msg.linear.x = vel_lin
            #msg.linear.y = vel_lin
            msg.angular.z = vel_ang
        else:
            msg.linear.x = 0.0
            #msg.linear.y = 0.0
            msg.angular.z = 0.0
        
        #msg.x=1.0
        #msg.y=2.0

        self.publisher_.publish(msg)
        os.system('clear')
        self.get_logger().info('Publishing vel_lin: "%s"' % msg.linear.x)
        self.get_logger().info('Publishing vel_ang: "%s"' % msg.angular.z)
        self.get_logger().info('Distancia Euclidiana: "%s"' % str(ro))
        self.get_logger().info('Error X: "%s"' % str(self.poses[self.here_goal][0]-self.pose.position.x))
        self.get_logger().info('Error Y: "%s"' % str(self.poses[self.here_goal][1]-self.pose.position.y))
        self.get_logger().info('Goals: "%s"' % str(self.here_goal))
        self.get_logger().info('Goal coordenate: "%s"' % str(self.poses[self.here_goal]))
        #self.get_logger().info('X: "%s"' % str(self.pose.position.x))
        #self.get_logger().info('Y: "%s"' % str(self.pose.position.y))
        #self.get_logger().info('Z: "%s"' % str(self.pose.orientation.z))
        self.i += 1

    def init_subscriber(self):

        self.subscription_ = self.create_subscription(
            Odometry,
            '/odom',
            self.pose_callback,
            10)
        self.subscription_  # prevent unused variable warning

        return self.subscription_

    def pose_callback(self, msg):
        self.pose.position.x = msg.pose.pose.position.x
        self.pose.position.y = msg.pose.pose.position.y
        self.pose.orientation.z = msg.pose.pose.orientation.z
        #self.get_logger().info('msg: "%s"' % str(msg))
        #self.get_logger().info('X: "%s"' % str(self.pose.position.x))
        #self.get_logger().info('Y: "%s"' % str(self.pose.position.y))
        #self.get_logger().info('Z: "%s"' % str(self.pose.orientation.z))


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = Minimal()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
