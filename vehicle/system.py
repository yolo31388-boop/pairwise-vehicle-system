"""
pairwise-vehicle-system - VehicleSystem

This module contains a deliberately broken implementation.
Fix all bugs so that tests pass.
"""


class VehicleSystem:
    def __init__(self):
        self.state = {}

    def update_vehicle(self, *args, **kwargs):
        """BUG: 载具更新不做质量分布，前后重量一样"""
        return None

    def calculate_steering(self, *args, **kwargs):
        """BUG: 转向不做回正力矩，松手方向盘不回"""
        return None

    def calculate_engine_force(self, *args, **kwargs):
        """BUG: 引擎力不做扭矩曲线，转速高低力一样"""
        return None

    def resolve_vehicle_collision(self, *args, **kwargs):
        """BUG: 载具碰撞不做冲量计算，直接弹开"""
        return None

    def get_vehicle_state(self, *args, **kwargs):
        """BUG: 载具状态不做燃油，永远有油"""
        return None

