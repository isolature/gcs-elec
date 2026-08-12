#!/usr/bin/env python3
"""Minimal integration example for the production RescueCar client."""

import time

from rescue_car_client import RescueCarClient


with RescueCarClient() as car:
    car.arm()
    car.set_velocity(150, 0)
    time.sleep(2.0)
    car.stop()
    car.disarm()
    print(car.snapshot())

