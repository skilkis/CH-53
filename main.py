# -*- coding: utf-8 -*-
# from components import Battery
""" Explanation of Main File Here """

from inertia import *


class Helicopter(object):

    @property
    def weights(self):
        return 1.0

    @property
    def cg(self):
        return Point(7.18, 0, 1.82)

    @property
    def fuselage(self):
        return Box(length=12.5,
                   width=2.68,
                   height=2.68,
                   mass=3749,
                   position=Point(7.18, 0, 1.82),
                   reference=self.cg)

    @property
    def fuselage_top_extension(self):
        return Box(length=8.27,
                   width=1.07,
                   height=1.12,
                   mass=218.97,
                   position=Point(6.086, 0, 3.695),
                   reference=self.cg)

    @property
    def main_rotor(self):
        return Disk(radius=22.14/2.0,
                    mass=1524.445601,
                    position=Point(6.47, 0, 4.28),
                    reference=self.cg)

    @property
    def payload(self):
        return Box(length=20, width=2.1, height=2.1, mass=4000, position=Point(3, 0, 1), reference=Point(3.1, 0, 2))

    # This won't work
    def get_childen(self):
            return [value for value in self.__dict__.values() if type(value) is property]

    def get_mass(self):
        mass = []
        position = []
        for child in self.get_childen():
            fetched_object = child.__get__('')
            if hasattr(fetched_object, 'i'):
                mass.append(getattr(fetched_object, 'mass'))
                position.append(getattr(fetched_object, 'position'))

        return sum(mass)

if __name__ == '__main__':
    obj = Helicopter()
    print obj.get_childen()
