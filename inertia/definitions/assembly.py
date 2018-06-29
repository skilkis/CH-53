#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" This file contains the class definition used to estimate the Mass Moment of Inertia of the CH53 Helicopter """

from primitives import *

__author__ = ["San Kilkis"]
__all__ = ["Part", "Assembly"]


class Part(property):
    """ Re-names the property decorator as Part to be able to distinguish between built-in class `property` """
    pass


class Assembly(object):

    def __init__(self, center_of_gravity=Point(0, 0, 0)):
        self.cg = center_of_gravity

    def __repr__(self):
        """ Since the representation is always called upon when creating a class, this ensures that the pass_two
        Center of Gravity (C.G.) will be utilized """
        return "%s Inertia(%f, %f, %f) about C.G. Point(%f, %f, %f)" % (self.__class__.__name__,
                                                                        self.get_inertia().xx,
                                                                        self.get_inertia().yy,
                                                                        self.get_inertia().zz,
                                                                        self.cg.x,
                                                                        self.cg.y,
                                                                        self.cg.z)

    def get_cg(self):
        """ Utilizes the class-method `get_children` to iterate through all of the objects in order to find the C.G.

        :return: Location of the center of gravity in SI meter [m]
        :rtype: Point
        """
        total_mass = 0
        moment_x = 0
        moment_y = 0
        moment_z = 0

        for child in self.get_children():
            fetched_object = child.__get__(self)
            if hasattr(fetched_object, 'mass') and hasattr(fetched_object, 'position'):
                mass = getattr(fetched_object, 'mass')
                total_mass = total_mass + mass
                position = getattr(fetched_object, 'position')

                moment_x = moment_x + mass * position.x
                moment_y = moment_y + mass * position.y
                moment_z = moment_z + mass * position.z

        if total_mass is not 0:
            cg_x = moment_x / total_mass
            cg_y = moment_y / total_mass
            cg_z = moment_z / total_mass
        else:
            cg_x = 0
            cg_y = 0
            cg_z = 0

        return Point(cg_x, cg_y, cg_z)

    def pass_two(self):
        """ Due to circular reference, the c.g. of the current object instance is passed to a new object instance of w/
        the correct c.g. location to calculate the correct inertia, in the case that the c.g. has not changed then the
        current object instance is passed so that performance is increased.

        :return: A new object of the current class w/ an updated c.g. location
        :rtype: Compound
        """

        if self.get_cg().x == 0 and self.get_cg().y == 0 and self.get_cg().z == 0:
            new_obj = self
        else:
            setattr(self, 'cg', self.get_cg())
            new_obj = self.__class__(center_of_gravity=self.cg)
        return new_obj

    def get_inertia(self):
        """ Utilizes the class-method `get_children` to sum up all component inertias.

        :return: Total Mass Moment of Inertia w.r.t the center of gravity in SI kilogram meter squared [kg m^2]
        :rtype: Inertia
        """
        i_xx = 0
        i_yy = 0
        i_zz = 0
        for child in self.pass_two().get_children():
            fetched_object = child.__get__(self)
            if hasattr(fetched_object, 'i'):
                i = getattr(fetched_object, 'i')
                i_xx = i_xx + i.xx
                i_yy = i_yy + i.yy
                i_zz = i_zz + i.zz

        return Inertia(i_xx, i_yy, i_zz)

    @classmethod
    def get_children(cls):
        return [value for value in vars(cls).values() if isinstance(value, Part)]


if __name__ == '__main__':
    obj = Assembly()
    print obj

