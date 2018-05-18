#!/usr/bin/env python

"""Module to do simple 2D transformations and calculations."""

import roslib#; roslib.load_manifest('transform2d')
import math
import numpy 
import tf
from tf.transformations import euler_from_quaternion

class Transform2D(object):

    """ Class to implement simple 2D transformations, with points
    represented as 2D numpy arrays."""
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        """ Initialize with given x, y, theta (theta in radians). """
        self.x = x
        self.y = y
        self.theta = theta

    def __str__(self):
        """ Stringify this object. """
        return '(x:{0:g}, y:{1:g}, theta:{2:g})'.format(
            self.x, self.y, self.theta)

    def copy(self):
        """ Return a copy of this Transform2D object. """
        return Transform2D(self.x, self.y, self.theta)

    def translation(self):
        """ Return the translation part of the transformation as a
        length-2 numpy array. """
        return numpy.array((self.x, self.y))

    def transform_fwd(self, point):
        """ Return the forward transformation of the given point:
        output = R(theta) * point + (x, y)"""
        cos_theta = math.cos(self.theta)
        sin_theta = math.sin(self.theta)
        return numpy.array((cos_theta*point[0] - sin_theta*point[1] + self.x,
                            sin_theta*point[0] + cos_theta*point[1] + self.y))
    
    def transform_inv(self, point):
        """ Return the inverse transformation of the given point:
        output = R(-theta) * [point - (x,y)]"""
        cos_theta = math.cos(self.theta)
        sin_theta = math.sin(self.theta)
        rel_x = point[0] - self.x
        rel_y = point[1] - self.y
        return numpy.array((cos_theta*rel_x + sin_theta*rel_y,
                            -sin_theta*rel_x + cos_theta*rel_y))

    def compose_with(self, other):
        """ Return the product of this transformation with other such
        that the transforming a point forward via the result is
        equivalent to first transforming it forward via the other,
        then with this transformation."""
        (new_x, new_y) = self.transform_fwd((other.x, other.y))
        return Transform2D(new_x, new_y,
                           normalize_angle(self.theta + other.theta))

    def inverse(self):
        """ Return the transformation which undoes this transformation. """
        (new_x, new_y) = self.transform_inv((0, 0))
        return Transform2D(new_x, new_y, -self.theta)

    def __mul__(self, obj):
        """ Overloaded multiplication. If the right operand is a
        Transform2D, returns self.compose_with(obj), otherwise returns
        self.transform_fwd(obj). """
        if isinstance(obj, Transform2D):
            return self.compose_with(obj)
        else:
            return self.transform_fwd(obj)

def distance_2d(point_a, point_b):
    """Return the 2D distance between the two input vectors."""
    dist_x = point_b[0] - point_a[0]
    dist_y = point_b[1] - point_a[1]
    return math.sqrt(dist_x**2 + dist_y**2)

def norm_2d(point):
    """Return the 2D norm of the input vector."""
    x, y = point
    return math.sqrt(x**2 + y**2)

def transform2d_from_ros_transform(ros_xform):
    """Convert a ROS 3D rigid transform to a Transform2D object."""
    (x, y, _) = ros_xform[0]
    (_, _, yaw) = euler_from_quaternion(ros_xform[1])
    return Transform2D(x, y, yaw)

def transform2d_from_ros_pose(ros_pose):
    """Convert a ROS 3D rigid transform to a Transform2D object."""
    p = ros_pose.position
    o = ros_pose.orientation

    (_, _, yaw) = euler_from_quaternion(
        (o.x, o.y, o.z, o.w)
    )
    return Transform2D(p.x, p.y, yaw)

def normalize_angle(delta):
    """Normalize an angle to fall between -pi and pi."""
    if delta > 0:
        return math.fmod(delta + math.pi, 2*math.pi) - math.pi
    else:
        return math.pi - math.fmod(math.pi - delta, 2*math.pi)

def diff_angle_rad(alpha, beta):
    """Return the normalized difference between two angles alpha and beta such
that adding the difference to alpha yields an angle equivalent to beta.

    """
    return normalize_angle(beta-alpha)

def _do_tests():
    """ Some demos/tests. """

    print 'hi world'

    t1 = Transform2D(0.5, 1.0, math.pi/4)
    p = numpy.array((1, 0))
    q = t1.transform_fwd(p)

    print t1
    print
    print p
    print q
    print t1.transform_inv(q)
    print

    t2 = t1.inverse()
    pp = t2.transform_fwd(q)
    qq = t2.transform_inv(pp)

    print t2
    print
    print q
    print pp
    print qq
    print

    t12 = t1.compose_with(t2)
    t21 = t2.compose_with(t1)

    print t12
    print t21
    print t1 * t2
    print t1 * p
    print t2 * q

    print

    print (t1*t1) * p
    print t1 * (t1*p)

    ros_transform = ((1.0, 0.2, 3.0),
                     tf.transformations.quaternion_from_euler(0.0, 0.0, 0.5))

    print ros_transform
    print

    t = transform2d_from_ros_transform(ros_transform)
    print t

    print Transform2D(1.0, 0.2, 0.5)

    print diff_angle_rad(-math.pi/4, math.pi/4)
    print diff_angle_rad(7*math.pi/4, math.pi/4)

if __name__ == "__main__":

    _do_tests()
