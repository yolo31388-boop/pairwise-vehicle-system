"""
pairwise-vehicle-system - VehicleSystem

A small but physically-plausible vehicle model:

* update_vehicle()          - mass distribution, center of gravity, suspension
* calculate_steering()      - return-to-center torque, angle limit, speed influence
* calculate_engine_force()  - torque curve, gears, max rpm
* resolve_vehicle_collision() - impulse at a collision point, damage accumulation
* get_vehicle_state()       - fuel, damage state, JSON serialization
"""

import json
import math


def _default_vehicle():
    return {
        "mass": 1200.0,                 # kg
        "wheelbase": 2.7,                 # m
        "cog_height": 0.5,                # m, center of gravity height
        "cog_offset": 0.1,                # m, CoG offset forward of mid-wheelbase
        "track_width": 1.6,               # m
        "wheel_radius": 0.33,             # m
        "position": [0.0, 0.0],
        "velocity": [0.0, 0.0],
        "heading": 0.0,                   # rad
        "yaw_rate": 0.0,
        "steering_angle": 0.0,            # rad, current front wheel angle
        "rpm": 900.0,
        "gear": 1,
        "throttle": 0.0,
        "fuel": 50.0,                     # litres
        "fuel_capacity": 50.0,
        "damage": 0.0,                    # 0.0 = pristine, 1.0 = totaled
        "wheels": {
            # suspension state per wheel
            name: {"compression": 0.0, "velocity": 0.0, "load": 0.0}
            for name in ("front_left", "front_right", "rear_left", "rear_right")
        },
    }


# Engine / drivetrain constants
IDLE_RPM = 900.0
MAX_RPM = 6500.0                        # hard rev limit
PEAK_TORQUE_RPM = 4000.0
PEAK_TORQUE = 320.0                     # Nm
GEAR_RATIOS = [0.0, 3.6, 2.2, 1.5, 1.1, 0.85]   # index = gear (0 = neutral)
FINAL_DRIVE = 3.9
MAX_STEERING_ANGLE = math.radians(35.0)  # steering lock
STEERING_RETURN_RATE = 4.0               # 1/s self-centering strength
SUSPENSION_STIFFNESS = 35000.0           # N/m per wheel
SUSPENSION_DAMPING = 3500.0              # N*s/m per wheel
FUEL_CONSUMPTION_IDLE = 0.0004           # litres/s
FUEL_CONSUMPTION_PER_FORCE = 2.0e-8      # litres per (N * s)


class VehicleSystem:
    def __init__(self):
        self.state = {}
        self.vehicles = {}

    # ------------------------------------------------------------------
    # vehicle lookup helpers
    # ------------------------------------------------------------------
    def _get_vehicle(self, vehicle=None):
        if vehicle is None:
            vehicle = self.state.get("vehicle")
            if vehicle is None:
                vehicle = _default_vehicle()
                self.state["vehicle"] = vehicle
        return vehicle

    # ------------------------------------------------------------------
    # Physics update: mass distribution, CoG, suspension
    # ------------------------------------------------------------------
    def mass_distribution(self, vehicle):
        """Static front/rear weight split from the CoG position."""
        wheelbase = vehicle["wheelbase"]
        cog = vehicle["cog_offset"]  # forward of centre
        rear_share = 0.5 + cog / wheelbase
        rear_share = min(max(rear_share, 0.0), 1.0)
        front_share = 1.0 - rear_share
        return {"front": front_share, "rear": rear_share}

    def center_of_gravity(self, vehicle):
        return {
            "height": vehicle["cog_height"],
            "longitudinal_offset": vehicle["cog_offset"],
            "lateral_offset": 0.0,
        }

    def _update_suspension(self, vehicle, dt, dist):
        """Spring/damper per wheel; wheels are NOT rigidly fixed to the body."""
        mass = vehicle["mass"]
        total_weight = mass * 9.81
        for name, share in (("front", dist["front"]), ("rear", dist["rear"])):
            for side in ("left", "right"):
                wheel = vehicle["wheels"]["%s_%s" % (name, side)]
                target_load = total_weight * share * 0.5
                # spring compression under load
                target_compression = target_load / SUSPENSION_STIFFNESS
                # damped approach to the target compression
                spring_force = SUSPENSION_STIFFNESS * (target_compression - wheel["compression"])
                damper_force = -SUSPENSION_DAMPING * wheel["velocity"]
                accel = (spring_force + damper_force) / (mass * 0.25)
                wheel["velocity"] += accel * dt
                wheel["compression"] += wheel["velocity"] * dt
                wheel["compression"] = max(wheel["compression"], 0.0)
                wheel["load"] = target_load

    def update_vehicle(self, vehicle=None, dt=1.0 / 60.0, throttle=None, brake=0.0):
        vehicle = self._get_vehicle(vehicle)
        if throttle is not None:
            vehicle["throttle"] = min(max(throttle, 0.0), 1.0)

        dist = self.mass_distribution(vehicle)
        cog = self.center_of_gravity(vehicle)
        self._update_suspension(vehicle, dt, dist)

        # longitudinal dynamics from the engine model
        engine = self.calculate_engine_force(vehicle)
        force = engine["force"] * vehicle["throttle"] - brake * 8000.0
        speed = math.hypot(*vehicle["velocity"])
        if speed > 0.0 and vehicle["fuel"] > 0.0 and vehicle["damage"] < 1.0:
            accel = force / vehicle["mass"]
            ux, uy = vehicle["velocity"][0] / speed, vehicle["velocity"][1] / speed
            vehicle["velocity"][0] += ux * accel * dt
            vehicle["velocity"][1] += uy * accel * dt
        vehicle["position"][0] += vehicle["velocity"][0] * dt
        vehicle["position"][1] += vehicle["velocity"][1] * dt

        # fuel burn
        vehicle["fuel"] = max(
            0.0,
            vehicle["fuel"]
            - (FUEL_CONSUMPTION_IDLE + abs(force) * FUEL_CONSUMPTION_PER_FORCE) * dt,
        )

        # rollover check: lateral accel * CoG height vs track width restoring moment
        lateral_accel = abs(vehicle["yaw_rate"]) * speed
        rollover = lateral_accel * cog["height"] > 9.81 * vehicle["track_width"] * 0.5

        return {
            "mass_distribution": dist,
            "center_of_gravity": cog,
            "wheels": vehicle["wheels"],
            "position": list(vehicle["position"]),
            "velocity": list(vehicle["velocity"]),
            "rollover": rollover,
            "fuel": vehicle["fuel"],
            "damage": vehicle["damage"],
        }

    # ------------------------------------------------------------------
    # Steering: return-to-center, angle limit, speed sensitivity
    # ------------------------------------------------------------------
    def calculate_steering(self, vehicle=None, steering_input=0.0, dt=1.0 / 60.0):
        vehicle = self._get_vehicle(vehicle)
        speed = math.hypot(*vehicle["velocity"])
        angle = vehicle["steering_angle"]

        # 1) angle limit: the wheel can never exceed the steering lock
        max_angle = MAX_STEERING_ANGLE
        # 2) speed influence: high speed shrinks the usable lock,
        #    so the turn radius grows with speed
        speed_factor = 1.0 / (1.0 + (speed / 30.0) ** 2)
        effective_max = max_angle * speed_factor

        target = min(max(steering_input, -1.0), 1.0) * effective_max

        # 3) self-aligning (return-to-center) torque: with no driver input
        #    the wheel is pulled back to straight ahead
        return_torque = -STEERING_RETURN_RATE * angle * (1.0 + speed * 0.05)
        angle += (target - angle) * min(1.0, 6.0 * dt) + return_torque * dt
        angle = min(max(angle, -max_angle), max_angle)
        vehicle["steering_angle"] = angle

        # bicycle-model turn radius for the current angle
        if abs(angle) > 1e-6:
            turn_radius = vehicle["wheelbase"] / math.tan(abs(angle))
        else:
            turn_radius = math.inf

        return {
            "steering_angle": angle,
            "return_torque": return_torque,
            "max_angle": effective_max,
            "turn_radius": turn_radius,
        }

    # ------------------------------------------------------------------
    # Engine: torque curve, gears, rev limit
    # ------------------------------------------------------------------
    def _torque_curve(self, rpm):
        """Normalized torque: rises to the peak, falls toward the redline."""
        if rpm <= 0.0 or rpm >= MAX_RPM:
            return 0.0
        x = rpm / PEAK_TORQUE_RPM
        if x <= 1.0:
            return 0.4 + 0.6 * x          # ramp up to the peak
        # falling branch, zero at the rev limit
        return max(0.0, 1.0 - (rpm - PEAK_TORQUE_RPM) / (MAX_RPM - PEAK_TORQUE_RPM))

    def calculate_engine_force(self, vehicle=None, throttle=None, gear=None, rpm=None):
        vehicle = self._get_vehicle(vehicle)
        if throttle is None:
            throttle = vehicle.get("throttle", 0.0)
        if rpm is None:
            rpm = vehicle.get("rpm", IDLE_RPM)
        if gear is None:
            gear = vehicle.get("gear", 1)

        # rev limit: rpm can never exceed MAX_RPM
        rpm = min(max(rpm, 0.0), MAX_RPM)
        vehicle["rpm"] = rpm

        # gears: clamp to the available gearbox
        gear = int(min(max(gear, 0), len(GEAR_RATIOS) - 1))
        vehicle["gear"] = gear
        gear_ratio = GEAR_RATIOS[gear]

        torque = PEAK_TORQUE * self._torque_curve(rpm) * min(max(throttle, 0.0), 1.0)
        wheel_torque = torque * gear_ratio * FINAL_DRIVE
        force = wheel_torque / vehicle["wheel_radius"]

        return {
            "rpm": rpm,
            "max_rpm": MAX_RPM,
            "gear": gear,
            "gear_ratio": gear_ratio,
            "torque": torque,
            "force": force,
        }

    # ------------------------------------------------------------------
    # Collision: impulse at a collision point, damage
    # ------------------------------------------------------------------
    def resolve_vehicle_collision(self, vehicle=None, other=None,
                                  collision_point=None, normal=None,
                                  restitution=0.3):
        vehicle = self._get_vehicle(vehicle)
        if other is None:
            other = _default_vehicle()
            other["velocity"] = [-v for v in vehicle["velocity"]]
        if collision_point is None:
            # midpoint between the two vehicles
            collision_point = [
                (vehicle["position"][0] + other["position"][0]) * 0.5,
                (vehicle["position"][1] + other["position"][1]) * 0.5,
            ]
        if normal is None:
            dx = other["position"][0] - vehicle["position"][0]
            dy = other["position"][1] - vehicle["position"][1]
            length = math.hypot(dx, dy)
            if length == 0.0:
                normal = [1.0, 0.0]
            else:
                normal = [dx / length, dy / length]

        m1, m2 = vehicle["mass"], other["mass"]
        rvx = other["velocity"][0] - vehicle["velocity"][0]
        rvy = other["velocity"][1] - vehicle["velocity"][1]
        vel_along_normal = rvx * normal[0] + rvy * normal[1]

        damage_done = 0.0
        if vel_along_normal < 0.0:
            # impulse-based resolution (no more teleport-bounce)
            j = -(1.0 + restitution) * vel_along_normal / (1.0 / m1 + 1.0 / m2)
            impulse = [j * normal[0], j * normal[1]]
            vehicle["velocity"][0] -= impulse[0] / m1
            vehicle["velocity"][1] -= impulse[1] / m2
            other["velocity"][0] += impulse[0] / m2
            other["velocity"][1] += impulse[1] / m2

            # damage scales with the impulse; it accumulates across hits
            damage_done = min(1.0, abs(j) / (m1 * 20.0))
            vehicle["damage"] = min(1.0, vehicle["damage"] + damage_done)
            other["damage"] = min(1.0, other.get("damage", 0.0) + damage_done)
        else:
            impulse = [0.0, 0.0]

        return {
            "impulse": impulse,
            "collision_point": list(collision_point),
            "normal": list(normal),
            "damage": vehicle["damage"],
            "damage_done": damage_done,
            "velocity": list(vehicle["velocity"]),
            "other_velocity": list(other["velocity"]),
        }

    # ------------------------------------------------------------------
    # State: fuel, damage, serialization
    # ------------------------------------------------------------------
    def get_vehicle_state(self, vehicle=None):
        vehicle = self._get_vehicle(vehicle)
        return {
            "position": list(vehicle["position"]),
            "velocity": list(vehicle["velocity"]),
            "heading": vehicle["heading"],
            "steering_angle": vehicle["steering_angle"],
            "rpm": vehicle["rpm"],
            "gear": vehicle["gear"],
            "fuel": vehicle["fuel"],
            "fuel_capacity": vehicle["fuel_capacity"],
            "out_of_fuel": vehicle["fuel"] <= 0.0,
            "damage": vehicle["damage"],
            "totaled": vehicle["damage"] >= 1.0,
            "drivable": vehicle["fuel"] > 0.0 and vehicle["damage"] < 1.0,
        }

    def serialize_state(self, vehicle=None):
        """JSON snapshot of the full vehicle state (position included)."""
        vehicle = self._get_vehicle(vehicle)
        return json.dumps(vehicle, sort_keys=True)

    def deserialize_state(self, payload):
        """Restore a vehicle exactly where it was saved."""
        vehicle = json.loads(payload)
        self.state["vehicle"] = vehicle
        return vehicle
