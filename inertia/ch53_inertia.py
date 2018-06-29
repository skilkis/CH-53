# -*- coding: utf-8 -*-
# from components import Battery
""" This file contains the class definition used to estimate the Mass Moment of Inertia of the CH53 Helicopter """


from definitions import *

__author__ = ["San Kilkis"]


class CH53Inertia(Assembly):

    @Part
    def fuselage(self):
        return Box(length=12.5,
                   width=2.68,
                   height=2.68,
                   mass=2880.046412,
                   position=Point(7.18, 0, 1.82),
                   reference=self.cg)

    @Part
    def fuselage_top_extension(self):
        return Box(length=8.27,
                   width=1.07,
                   height=1.12,
                   mass=954.61,
                   position=Point(6.085, 0, 3.695),
                   reference=self.cg)

    @Part
    def fuselage_side_extension_left(self):
        return Box(length=5.55,
                   width=1.02,
                   height=1.120,
                   mass=142.85,
                   position=Point(7.685, 1.85, 1.19),
                   reference=self.cg)

    @Part
    def fuselage_side_extension_right(self):
        return Box(length=5.55,
                   width=1.02,
                   height=1.120,
                   mass=142.85,
                   position=Point(7.685, 1.85, 1.19),
                   reference=self.cg)

    @Part
    def tail_boom(self):
        return Box(length=5.550,
                   width=1.610,
                   height=0.970,
                   mass=195.269,
                   position=Point(16.25, 0, 2.675),
                   reference=self.cg)

    @Part
    def nose_cone(self):
        return HemiSphere(radius=2.68/2.0,
                          mass=961.7508,
                          position=Point(0.4275, 0, 1.82),
                          reference=self.cg,
                          orientation='zyx')

    @Part
    def front_landing_gear(self):
        return Cylinder(radius=0.68/2.0,
                        length=0.68,
                        mass=216.349,
                        position=Point(0.59, 0, 0.34),
                        reference=self.cg,
                        orientation='yxz')

    @Part
    def rear_landing_gear_right(self):
        return Cylinder(radius=0.68/2.0,
                        length=0.68,
                        mass=216.349,
                        position=Point(8.95, -2.02, 0.29),
                        reference=self.cg,
                        orientation='yxz')

    @Part
    def rear_landing_gear_left(self):
        return Cylinder(radius=0.68/2.0,
                        length=0.68,
                        mass=216.349,
                        position=Point(8.95, 2.02, 0.29),
                        reference=self.cg,
                        orientation='yxz')

    @Part
    def engine_left(self):
        return Cylinder(radius=1.07/2.0,
                        length=3.5,
                        mass=369.8698,
                        position=Point(5.11, 1.655, 3.065),
                        reference=self.cg)

    @Part
    def engine_right(self):
        return Cylinder(radius=1.07/2.0,
                        length=3.5,
                        mass=369.8698,
                        position=Point(5.11, -1.655, 3.065),
                        reference=self.cg)

    @Part
    def internal_tank_right(self):
        return Box(length=2.97,
                   width=1.02,
                   height=1.120,
                   mass=237.093,
                   position=Point(7.085, 1.85, 1.19),
                   reference=self.cg)

    @Part
    def internal_tank_left(self):
        return Box(length=2.97,
                   width=1.02,
                   height=1.120,
                   mass=237.093,
                   position=Point(7.085, -1.85, 1.19),
                   reference=self.cg)

    @Part
    def external_tank_left(self):
        return Cylinder(radius=0.97/2.0,
                        length=5.6,
                        mass=289.1864,
                        position=Point(7.03, 3.07, 1.115),
                        reference=self.cg)

    @Part
    def external_tank_right(self):
        return Cylinder(radius=0.97/2.0,
                        length=5.6,
                        mass=289.1864,
                        position=Point(7.03, -3.07, 1.115),
                        reference=self.cg)

    @Part
    def main_hub(self):
        return Cylinder(radius=1.27/2.0,
                        length=2.14,
                        mass=1359.755,
                        position=Point(6.38, 0, 4.23),
                        reference=self.cg,
                        orientation='yzx')

    @Part
    def main_rotor(self):
        return UserSpecified(i_input=Inertia(33737.88, 33737.88, 67474.18),
                             mass=1524.446,
                             position=Point(6.47, 0, 4.845),
                             reference=self.cg)

    # @Part
    # def main_rotor_disk(self):
    #     return Disk(radius=22.14/2.0,
    #                 mass=1524.446,
    #                 position=Point(6.47, 0, 4.845),
    #                 reference=self.cg)

    @Part
    def vertical_tail(self):
        return Plate(length=1.46,
                     width=3.36,
                     mass=59.72297,
                     position=Point(19.71, 0, 3.87),
                     reference=self.cg,
                     orientation='xzy')

    @Part
    def horizontal_tail(self):
        return Plate(length=1.46,
                     width=2.720,
                     mass=48.34984,
                     position=Point(19.71, -1.36, 5.5625),
                     reference=self.cg)

    @Part
    def tail_rotor(self):
        return Disk(radius=4.8768/2.0,
                    mass=195.3984,
                    position=Point(19.95, 0.0375, 5.06),
                    reference=self.cg,
                    orientation='xzy')

    @Part
    def main_h_drive(self):
        return SlenderBar(length=2.680,
                          mass=1085.402,
                          position=Point(6.38, 0, 3.06),
                          reference=self.cg,
                          orientation='xyz')

    @Part
    def main_v_drive(self):
        return SlenderBar(length=1.56,
                          mass=631.8013,
                          position=Point(6.38, 0, 3.45),
                          reference=self.cg,
                          orientation='zyx')

    @Part
    def tail_h_drive(self):
        return SlenderBar(length=13.770,
                          mass=165.1365,
                          position=Point(13.065, 0, 3.06),
                          reference=self.cg,
                          orientation='xyz')

    @Part
    def tail_v_drive(self):
        return SlenderBar(length=2.140,
                          mass=25.66391,
                          position=Point(19.95, 0, 4.13),
                          reference=self.cg,
                          orientation='zyx')


if __name__ == '__main__':
    obj = CH53Inertia()
    print (obj.get_cg())
    print (obj.get_inertia())
