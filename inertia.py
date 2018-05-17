#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" A file containing all relevant mass moment of inertias as functions
    Source: http://d2vlcm61l7u1fs.cloudfront.net/media%2F857%2F857134bf-4cae-49a7-ad91-75b4ae9e20bb%2FphpknWzow.png
"""

from collections import namedtuple


def inertia_format(i_xx, i_yy, i_zz):
    my_inertia_tuple = namedtuple('Inertia', ['Ixx', 'Iyy', 'Izz'])
    return my_inertia_tuple(i_xx, i_yy, i_zz)


def position_format(x, y, z):
    my_position_tuple = namedtuple('Position', ['x', 'y', 'z'])
    return my_position_tuple(x, y, z)


def cylinder(mass, radius, length):
    """ Useful for the inertia of the motors

    :param mass: Mass of object in SI kilogram
    :param radius: Radius of the cylinder
    :param length: Dimension of object along x-axis in SI m
    :rtype: tuple
    :return: Mass Moment of Inertia around x, y, z axis respectively
    """
    i_yy = 1.0/12.0 * mass * (3 * (radius ** 2) + (length ** 2))
    i_zz = i_yy
    i_xx = 0.5 * mass * (radius ** 2)
    return inertia_format(i_xx, i_yy, i_zz)


def thin_plate(mass, width, length):
    """ Useful for the inertia of the rotors

    :param mass: Mass of object in SI kilogram
    :param width: Dimension of object along y-axis in SI meter
    :param length: Dimension of object along x-axis in SI meter
    :rtype: tuple
    :return: Mass Moment of Inertia around x, y, z axis respectively
    """
    i_xx = 1.0/12.0 * mass * (width ** 2)
    i_yy = 1.0/12.0 * mass * (length ** 2)
    i_zz = 1.0/12.0 * mass * ((length ** 2) + (width ** 2))
    return i_xx, i_yy, i_zz


def hemi_sphere(mass, radius):
    """ Useful for rotor hub assembly

    :param mass: Mass of object in SI kilogram
    :param radius: Radius of the cylinder in SI meter
    :rtype: tuple
    :return: Mass Moment of Inertia around x, y, z axis respectively
    """

    i_xx = 0.259 * mass * (radius ** 2)
    i_yy = i_xx
    i_zz = 2.0 / 5.0 * mass * (radius ** 2)
    return i_xx, i_yy, i_zz


def box(mass, width, height, length):
    """ Useful for the main fuselage of the rotors

    :param mass: Mass of object in SI kilogram
    :param width: Dimension of object along y-axis in SI meter
    :param height: Dimension of the object along the z-axis in SI meter
    :param length: Dimension of object along x-axis in SI meter
    :rtype: tuple
    :return: Mass Moment of Inertia around x, y, z axis respectively
    """

    i_xx = (mass * ((width ** 2) + (height ** 2))) / 12.0
    i_yy = (mass * ((height ** 2) + (length ** 2))) / 12.0
    i_zz = (mass * ((width ** 2) + (length ** 2))) / 12.0
    return i_xx, i_yy, i_zz


if __name__ == '__main__':
    obj = cylinder(1, 2, 3)
    print obj

