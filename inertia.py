#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" A file containing all relevant component shapes """

__author__ = "Şan Kılkış"

# TODO Re-upload final version to Overleaf due to the change


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
    a new class only the attributes `i_xx_prime`, `i_yy_prime`, and `i_zz_prime` need to be redefined. These are the
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
        """ Mass Moment of Inertia on the xx'-axis """
        return 0.0

    def i_yy_prime(self):
        """ Mass Moment of Inertia on the yy'-axis """
        return 0.0

    def i_zz_prime(self):
        """ Mass Moment of Inertia on the zz'-axis """
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
    """ Useful for the inertia of the control surfaces (H Tail, V Tail)


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
    tail rotor inertia is desired be sure to use `i.zz`.

    :param radius: Radius of the disk
    :param mass: Mass of the disk in SI kilogram [kg]
    :param position: Location of the object center w.r.t the global system of reference in SI meter [m]
    :param reference: Body Axis System origin (center of gravity (c.g)) w.r.t the global reference system in SI meter [m]
    """

    def __init__(self, radius, **kwargs):
        self.radius = radius
        super(Disk, self).__init__(**kwargs)

    def i_xx_prime(self):
        return 0.5 * self.mass * (self.radius ** 2)

    def i_yy_prime(self):
        return 1.0/4.0 * self.mass * (self.radius ** 2)

    def i_zz_prime(self):
        return 1.0/4.0 * self.mass * (self.radius ** 2)


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


if __name__ == '__main__':
    obj = Box(length=1.0, width=1.0, height=1.0, mass=20.0, position=Point(0, 0, 0), reference=Point(0, 0, 0))
    print obj

