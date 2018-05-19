#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" A file containing all relevant mass moment of inertias as functions
    Source: http://d2vlcm61l7u1fs.cloudfront.net/media%2F857%2F857134bf-4cae-49a7-ad91-75b4ae9e20bb%2FphpknWzow.png
"""

__author__ = "Şan Kılkış"


class Point(object):
    """ A simple class to define a point """

    def __init__(self, x, y, z):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    def __repr__(self):
        return "Point(%f, %f, %f)" % (self.x, self.y, self.z)


class Inertia(object):
    """ A simple class to define an Inertia Vector (Neglecting off-diagonal terms for now"""
    def __init__(self, xx, yy, zz):
        self.xx = float(xx)
        self.yy = float(yy)
        self.zz = float(zz)

    def __repr__(self):
        return "Inertia(%f, %f, %f)" % (self.xx, self.yy, self.zz)


class Component(object):
    """ Class definition that every component of the helicopter inherits from. Due to class inheritance, when defining
    a new class only the attributes `i_xx_prime`, `i_yy_prime`, and 'i_zz_prime` need to be redefined. These are the
    equations describing the Mass Moment of Inertia of the Component w.r.t its own center of gravity.

    :param mass: Component mass in SI kilogram [kg]
    :type mass: float
    :param position: Component position w.r.t a global reference system in SI meter [m]
    :type position: Point
    :param reference: Body Axis System origin (center of gravity (c.g)) w.r.t the global reference system
    :type reference: Point
    """

    def __init__(self, mass, position=Point(0, 0, 0), reference=Point(0, 0, 0)):
        self.mass = float(mass)
        self.position = position
        self.reference = reference

    def i_xx_prime(self):
        return 0.0

    def i_yy_prime(self):
        return 0.0

    def i_zz_prime(self):
        return 0.0

    @property
    def i_prime(self):
        """

        :return: Mass Moment of Inertia w.r.t the component center SI kilogram meter squared [kg m^2]
        """
        return Inertia(self.i_xx_prime(), self.i_yy_prime(), self.i_zz_prime())

    @property
    def i(self):
        """

        :return: Mass Moment of Inertia w.r.t a global body axis system utilizing the Parallel Axis Theorem
        """
        return Inertia(self.i_prime.xx + self.steiner_term.xx,
                       self.i_prime.yy + self.steiner_term.yy,
                       self.i_prime.zz + self.steiner_term.zz)

    @property
    def steiner_term(self):
        """ Returns the increased inertia due to displacement of an object from the body center of gravity (c.g). This
        is also known as the parallel axis theorem. Due to the fact that the Mass Moment of Inertia describes the
        resistance of a 3D object to rotation over an axis, the parallel axis theorem requires

        :return: Mass Moment of Inertia vector in SI kilogram meter squared [kg m^2]
        :rtype: Inertia
        """

        distance_x = self.position.x - self.reference.x
        distance_y = self.position.y - self.reference.y
        distance_z = self.position.z - self.reference.z

        i_parallel_xx = self.mass * (distance_y ** 2 + distance_z ** 2)
        i_parallel_yy = self.mass * (distance_x ** 2 + distance_z ** 2)
        i_parallel_zz = self.mass * (distance_x ** 2 + distance_y ** 2)

        return Inertia(i_parallel_xx, i_parallel_yy, i_parallel_zz)


class Box(Component):
    def __init__(self, width, length, height, **kwargs):
        self.width = width
        self.length = length
        self.height = height
        super(Box, self).__init__(**kwargs)

    def i_xx_prime(self):
        return self.mass * ((self.width ** 2) + (self.height ** 2))

    # def i_yy_prime(self):


#
# def steiner_term(mass, position, cg_position=Point(0, 0, 0)):
#     """ Returns the increased inertia due to displacement of an object from the body center of gravity (c.g) also
#     known as the parallel axis theorem, due to the property of the mass moment of inertia describing a 3D quantity the
#     parallel axis theorem requires the
#
#     :param mass: Mass of the component in SI kg [kg]
#     :param distance: Longitudinal Distance to the center of gravity (c.g.) in SI meter [m]
#     :return: Mass Moment of Inertia in SI kilogram meter squared [kg m^2]
#     :rtype: float
#     """
#
#     i_parallel = mass * (distance1 ** 2 + distance2 ** 2)
#
#     return i_parallel


# def cylinder(mass, radius, length):
#     """ Useful for the inertia of the motors
#
#     :param mass: Mass of object in SI kilogram
#     :param radius: Radius of the cylinder
#     :param length: Dimension of object along x-axis in SI m
#     :rtype: tuple
#     :return: Mass Moment of Inertia around x, y, z axis respectively
#     """
#     i_yy = 1.0/12.0 * mass * (3 * (radius ** 2) + (length ** 2))
#     i_zz = i_yy
#     i_xx = 0.5 * mass * (radius ** 2)
#     return inertia_format(i_xx, i_yy, i_zz)
#
#
# def thin_plate(mass, width, length, position=Point(0, 0, 0)):
#     """ Useful for the inertia of the rotors
#
#     :param mass: Mass of object in SI kilogram
#     :param width: Dimension of object along y-axis in SI meter
#     :param length: Dimension of object along x-axis in SI meter
#     :param position: Location of the object center w.r.t the global system of reference in SI meter
#     :rtype: tuple
#     :return: Mass Moment of Inertia around x, y, z axis respectively
#     """
#     i_xx = 1.0/12.0 * mass * (width ** 2)
#     i_yy = 1.0/12.0 * mass * (length ** 2)
#     i_zz = 1.0/12.0 * mass * ((length ** 2) + (width ** 2))
#     return i_xx, i_yy, i_zz
#
#
# def thin_disk(mass, radius, position=Point(0, 0, 0)):
#     """ Useful for the inertia of the rotors, the thin disk is oriented such that the z-axis is the axis of revolve,
#     thus for the tail-rotor utilize the formula for I_zz for I_yy.
#
#     :param mass: Mass of object in SI kilogram
#     :param radius: Radius of the Disk along the y-axis in SI meter
#     :param position: Location of the object center w.r.t the global system of reference in SI meter
#     :rtype: tuple
#     :return: Mass Moment of Inertia around x, y, z axis respectively
#     """
#     i_xx_prime = 1.0/4.0 * mass * radius ** 2 + steiner_term(mass, position.x)
#     i_yy_prime = 1.0/4.0 * mass * radius ** 2 + steiner_term(mass, position.y)
#     i_zz_prime = 3.0/2.0 * mass * radius ** 2 + steiner_term(mass, position.z)
#     return i_xx, i_yy, i_zz
#
#
# def hemi_sphere(mass, radius, position=Point(0, 0, 0)):
#     """ Useful for rotor hub assembly
#
#     :param mass: Mass of object in SI kilogram
#     :param radius: Radius of the cylinder in SI meter
#     :param position: Location of the object center w.r.t the global system of reference in SI meter
#     :rtype: tuple
#     :return: Mass Moment of Inertia around x, y, z axis respectively
#     """
#
#     i_xx = 0.259 * mass * (radius ** 2) + steiner_term(mass, position.y, position.z)
#     i_yy = i_xx + steiner_term(mass, position.x, position.y)
#     i_zz = 2.0 / 5.0 * mass * (radius ** 2) + steiner_term(mass, position.z)
#     return i_xx, i_yy, i_zz
#
#
# def box(mass, width, height, length, position=Point(0, 0, 0)):
#     """ Useful for the main fuselage of the rotors
#
#     :param mass: Mass of object in SI kilogram
#     :param width: Dimension of object along y-axis in SI meter
#     :param height: Dimension of the object along the z-axis in SI meter
#     :param length: Dimension of object along x-axis in SI meter
#     :param position: Location of the object center w.r.t the global system of reference in SI meter
#     :rtype: tuple
#     :return: Mass Moment of Inertia around x, y, z axis respectively
#     """
#
#     i_xx = (mass * ((width ** 2) + (height ** 2))) / 12.0 + steiner_term(mass, position.x)
#     i_yy = (mass * ((height ** 2) + (length ** 2))) / 12.0 + steiner_term(mass, position.y)
#     i_zz = (mass * ((width ** 2) + (length ** 2))) / 12.0 + steiner_term(mass, position.z)
#     return i_xx, i_yy, i_zz


if __name__ == '__main__':
    obj = Box(length=1.0, width=1.0, height=1.0, mass=20.0, position=Point(0, 0, 0), reference=Point(0, 2, 3))
    print obj.i

