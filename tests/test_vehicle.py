import pytest
from vehicle.system import VehicleSystem


class TestVehicleSystem:

    def test_update_vehicle(self):
        """载具更新不做质量分布，前后重量一样"""
        obj = VehicleSystem()
        result = obj.update_vehicle()
        self.assertIsNotNone(result)

    def test_calculate_steering(self):
        """转向不做回正力矩，松手方向盘不回"""
        obj = VehicleSystem()
        result = obj.calculate_steering()
        self.assertIsNotNone(result)

    def test_calculate_engine_force(self):
        """引擎力不做扭矩曲线，转速高低力一样"""
        obj = VehicleSystem()
        result = obj.calculate_engine_force()
        self.assertIsNotNone(result)

    def test_resolve_vehicle_collision(self):
        """载具碰撞不做冲量计算，直接弹开"""
        obj = VehicleSystem()
        result = obj.resolve_vehicle_collision()
        self.assertIsNotNone(result)

    def test_get_vehicle_state(self):
        """载具状态不做燃油，永远有油"""
        obj = VehicleSystem()
        result = obj.get_vehicle_state()
        self.assertIsNotNone(result)

