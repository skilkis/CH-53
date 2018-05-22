# -*- coding: utf-8 -*-
# from components import Battery
""" Explanation of Main File Here """

from inertia import *


class Helicopter(object):

    @property
    def weights(self):
        return 1.0

    @property
    def fuselage(self):
        return Box(length=20, width=2.1, height=2.1, mass=4000, position=Point(3, 0, 1), reference=Point(3.1, 0, 2))

    @property
    def payload(self):
        return Box(length=20, width=2.1, height=2.1, mass=4000, position=Point(3, 0, 1), reference=Point(3.1, 0, 2))

    @staticmethod
    def get_childen():
            return [value for value in Helicopter.__dict__.values() if type(value) is property]

    def get_mass(self):
        mass = []
        position = []
        for child in self.get_childen():
            fetched_object = child.__get__('')
            if hasattr(fetched_object, 'i'):
                mass.append(getattr(fetched_object, 'mass'))
                position.append(getattr(fetched_object, 'position'))

        return sum(mass)






    # @property
    # def center_of_gravity(self):
    #     for part in self.get_childen():
    #         inertia = getattr(self, part)
    #     return inertia



if __name__ == '__main__':
    obj = Helicopter()
    print obj.get_childen()
