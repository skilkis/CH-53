#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" A file containing all relevant class definitions a component of the inertia module """

__author__ = ["San Kilkis"]


class Point(object):
    """ A simple class to define a point """

    __slots__ = ['x', 'y', 'z']

    def __init__(self, x, y, z):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    def __repr__(self):
        return "Point(%f, %f, %f)" % (self.x, self.y, self.z)


class Inertia(object):
    """ A simple class to define an Inertia Vector (Neglecting off-diagonal terms for now"""

    __slots__ = ['xx', 'yy', 'zz']

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
    :param orientation: Mapping variable to orient the shape, Default 'xyz'
    :type: str

    """

    def __init__(self, mass, position=Point(0, 0, 0), reference=Point(0, 0, 0), orientation='xyz'):
        self.mass = float(mass)
        self.position = position
        self.reference = reference
        self.orientation = orientation

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
    def i_transformed(self):
        """ Simple mapped MMOI that orients the shape to the new coordinate system

        :return: Mapped Mass Moment of Inertia w.r.t the component center in SI kilogram meter squared [kg m^2]
        """
        _i_transformed = []
        for axis in self.orientation:
            if axis is 'x':
                _i_transformed.append(self.i_prime.xx)
            elif axis is 'y':
                _i_transformed.append(self.i_prime.yy)
            elif axis is 'z':
                _i_transformed.append(self.i_prime.zz)
            else:
                raise NameError("You have provided a string input that is not one of 'xyz' format")
        return Inertia(_i_transformed[0], _i_transformed[1], _i_transformed[2])

    @property
    def i(self):
        """

        :return: Mass Moment of Inertia w.r.t a global body axis system utilizing the Parallel Axis Theorem
        """
        return Inertia(self.i_transformed.xx + self.steiner_term.xx,
                       self.i_transformed.yy + self.steiner_term.yy,
                       self.i_transformed.zz + self.steiner_term.zz)

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

