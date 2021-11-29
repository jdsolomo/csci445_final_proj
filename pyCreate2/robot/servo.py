"""
Module to control a Parallax Servo.
This is a template only and needs to be finished in Lab2
"""
from .pwm import *
import math


class Servo:
    def __init__(self, number):
        """Constructor.

        Args:
            number (integer): PWM number where the servo is connected to.
        """
        self.pwm = Pwm(number)
        self.pwm.set_frequency(50)
        self.pwm.enable()

    def go_to(self, angle):
        temp = (((2.25-0.75)/20)*100 + (angle/90)*3.75)
        self.pwm.set_duty_cycle(temp)
