#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" A file containing all relevant component shapes """

__author__ = ["San Kilkis"]

from inertia.component import *


class Box(Component):
    """ Useful for the main fuselage and rotor nacelle

    :param width: Dimension of object along y-axis in SI meter [m]
    :param length: Dimension of object along x-axis in SI meter [m]
    :param height: Dimension of the object along the z-axis in SI meter [m]
    :param mass: Mass of the box in SI kilogram [kg]
    :param position: Location of the object center w.r.t the global system of reference in SI meter [m]
    :param reference: Body Axis System origin (center of gravity (c.g)) w.r.t the global reference system in SI meter [m]
    """
    def __init__(self, width, length, height, **kwargs):
        self.width = width
        self.length = length
        self.height = height
        super(Box, self).__init__(**kwargs)

    def i_xx_prime(self):
        return 1.0/12.0 * self.mass * ((self.width ** 2) + (self.height ** 2))

    def i_yy_prime(self):
        return 1.0/12.0 * self.mass * ((self.length ** 2) + (self.height ** 2))

    def i_zz_prime(self):
        return 1.0/12.0 * self.mass * ((self.width ** 2) + (self.length ** 2))


class Cylinder(Component):
    """ Useful for the inertia of the motors. The cylinder is oriented such that the revolve direction is the x-axis

    :param radius: Radius of the cylinder
    :param length: Dimension of object along x-axis in SI m
    :param mass: Mass of the cylinder in SI kilogram [kg]
    :param position: Location of the object center w.r.t the global system of reference in SI meter [m]
    :param reference: Body Axis System origin (center of gravity (c.g)) w.r.t the global reference system in SI meter [m]
    """

    def __init__(self, radius, length, **kwargs):
        self.radius = radius
        self.length = length
        super(Cylinder, self).__init__(**kwargs)

    def i_xx_prime(self):
        return 0.5 * self.mass * (self.radius ** 2)

    def i_yy_prime(self):
        return self.mass * (1.0/12.0 * (self.length ** 2) + 1.0/4.0 * (self.radius ** 2))

    def i_zz_prime(self):
        return self.mass * (1.0/12.0 * (self.length ** 2) + 1.0/4.0 * (self.radius ** 2))


class Plate(Component):
    """ Useful for the inertia of the control surfaces (H Tail, V Tail), plate normal is the z-axis by default


    :param width: Dimension of object along y-axis in SI meter [m]
    :param length: Dimension of object along x-axis in SI meter [m]
    :param mass: Mass of the plate in SI kilogram [kg]
    :param position: Location of the object center w.r.t the global system of reference in SI meter [m]
    :param reference: Body Axis System origin (center of gravity (c.g)) w.r.t the global reference system in SI meter [m]
    """

    def __init__(self, width, length, **kwargs):
        self.width = width
        self.length = length
        super(Plate, self).__init__(**kwargs)

    def i_xx_prime(self):
        return 1.0/12.0 * self.mass * (self.width ** 2)

    def i_yy_prime(self):
        return 1.0/12.0 * self.mass * (self.length ** 2)

    def i_zz_prime(self):
        return 1.0/12.0 * self.mass * ((self.width ** 2) + (self.length ** 2))


class Disk(Component):
    """ Useful for the inertia of rotor. The disk is oriented such that the revolve direction is the z-axis, thus if the
    tail rotor inertia is desired be sure to use 'xzy'.

    :param radius: Radius of the disk
    :param mass: Mass of the disk in SI kilogram [kg]
    :param position: Location of the object center w.r.t the global system of reference in SI meter [m]
    :param reference: Body Axis System origin (center of gravity (c.g)) w.r.t the global reference system in SI meter [m]
    """

    def __init__(self, radius, **kwargs):
        self.radius = radius
        super(Disk, self).__init__(**kwargs)

    def i_xx_prime(self):
        return 0.25 * self.mass * (self.radius ** 2)

    def i_yy_prime(self):
        return 0.25 * self.mass * (self.radius ** 2)

    def i_zz_prime(self):
        return 0.5 * self.mass * (self.radius ** 2)


class SlenderBar(Component):
    """ A slender bar useful for push-rods or other slender objects. By default the bar axial direction is the x-axis

    :param length: Dimension of the bar along the x-axis in SI meter [m]
    :param mass: Mass of the bar in SI kilogram [kg]
    :param position: Location of the object center w.r.t the global system of reference in SI meter [m]
    :param reference: Body Axis System origin (center of gravity (c.g)) w.r.t the global reference system in SI meter [m]

    """

    def __init__(self, length, **kwargs):
        self.length = length
        super(SlenderBar, self).__init__(**kwargs)

    def i_xx_prime(self):
        return 0.0

    def i_yy_prime(self):
        return 1.0/12.0 * self.mass * (self.length ** 2)

    def i_zz_prime(self):
        return 1.0/12.0 * self.mass * (self.length ** 2)


class HemiSphere(Component):
    """ Useful for the inertia of the rotor hub-cap. The hemi-sphere is oriented such that the revolve direction is the
    z-axis

    :param radius: Radius of the hemisphere
    :param mass: Mass of the hemisphere in SI kilogram [kg]
    :param position: Location of the object center w.r.t the global system of reference in SI meter [m]
    :param reference: Body Axis System origin (center of gravity (c.g)) w.r.t the global reference system in SI meter [m]
    """

    def __init__(self, radius, **kwargs):
        self.radius = radius
        super(HemiSphere, self).__init__(**kwargs)

    def i_xx_prime(self):
        return 83.0/320.0 * self.mass * (self.radius ** 2)

    def i_yy_prime(self):
        return 83.0/320.0 * self.mass * (self.radius ** 2)

    def i_zz_prime(self):
        return 2.0/5.0 * self.mass * (self.radius ** 2)


class UserSpecified(Component):
    """ Useful for the inertia of a complex part that cannot be easily solved analytically. Thus utilizing CATIA,
    the inertia of the component can be found on it's own principal axes and inputting into this class definition:

    :param i_input: Input Mass Moment of Inertia vector in SI kilogram meter squared [kg m^2]
    :type i_input: Inertia
    :param mass: Mass of the user specified component in SI kilogram [kg]
    :param position: Location of the object center w.r.t the global system of reference in SI meter [m]
    :param reference: Body Axis System origin (center of gravity (c.g)) w.r.t the global reference system in SI meter [m]
    """

    def __init__(self, i_input=Inertia(0, 0, 0), **kwargs):
        self.i_input = i_input
        super(UserSpecified, self).__init__(**kwargs)

    def i_xx_prime(self):
        return self.i_input.xx

    def i_yy_prime(self):
        return self.i_input.yy

    def i_zz_prime(self):
        return self.i_input.zz


if __name__ == '__main__':
    obj = Cylinder(length=1.0, radius=1.0, mass=20.0, position=Point(0, 0, 0), reference=Point(0, 0, 0))
    print obj

