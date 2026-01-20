# Copyright 1996-2024 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Description: Example of collaborative work between two IPRs.
             Communication between robots is achieved using
             Emitter and Receiver devices.
             Control the IPR1 to grab and pass cubes and balls.
"""

from controller import Robot
import struct
import math

# Constants
MOTOR_NUMBER = 6
DISTANCE_SENSOR_NUMBER = 9
TOUCH_SENSOR_NUMBER = 4
POSITION_TOLERANCE = 0.0002
UPPER_ARM_MOTOR_TRANSITION_POSITION = -0.726919

# Motor indices
BASE_MOTOR = 0
UPPER_ARM_MOTOR = 1
FOREARM_MOTOR = 2
WRIST_MOTOR = 3
ROTATIONAL_WRIST_MOTOR = 4
GRIPPER_MOTOR = 5

# Collaboration states
GRAB_CUBE = 0
GIVE_CUBE = 1
LEAVE_CUBE = 2
THROW_CUBE = 3
# States for extra cubes (second batch)
GRAB_EXTRA = 4
GIVE_EXTRA = 5
LEAVE_EXTRA = 6
THROW_EXTRA = 7

CUBE_NUMBER = 3
EXTRA_CUBE_NUMBER = 1  # 1 extra cube (4 cubes total)

# IPR1 poses for cubes
GRAB_CUBE_POSITIONS = [
    [0.390582, -2.26583, 1.91850, -2.88388, -2.45437, 0.66207],
    [0.000010, -2.27120, 1.91850, -2.82253, -3.00660, 0.66207],
    [5.690000, -2.24000, 1.86000, -2.91500, -0.40000, 0.90000]
]

# IPR1 poses for extra cubes (2 additional cubes)
# Extra cube positions: cube1(1) at (-0.045, -1.42), cube2(1) at (-0.17, -1.429)
# Reference: cube2 original at (-0.17, -1.35) uses BASE=5.69, UPPER=-2.24
# - cube1(1): x=-0.045, y=-1.42 -> BASE ~6.2
# - cube2(1): x=-0.17 (same as cube2), y=-1.429 (further) -> BASE=5.69, more extension
GRAB_EXTRA_POSITIONS = [
    [6.000000, -2.27120, 1.91850, -2.82253, -3.00660, 0.66207],  # cube1(1) at (-0.045, -1.42)
    [5.690000, -2.30000, 1.91850, -2.91500, -0.40000, 0.90000]   # cube2(1) at (-0.17, -1.429)
]

DROP_POSITION = [3.016690, -0.86002, 0.77181, -1.96350, -1.22718, 0.66207, -0.66207]

# Alternative drop position for extra cubes (coming from left side) to avoid collision
# Slightly different BASE to create a different approach angle
DROP_POSITION_EXTRA = [3.200000, -0.86002, 0.77181, -1.96350, -1.22718, 0.66207, -0.66207]


def motor_name(motor_index):
    """Get motor name by index."""
    names = {
        BASE_MOTOR: "base",
        UPPER_ARM_MOTOR: "upperarm",
        FOREARM_MOTOR: "forearm",
        WRIST_MOTOR: "wrist",
        ROTATIONAL_WRIST_MOTOR: "rotational_wrist",
        GRIPPER_MOTOR: "gripper::right"
    }
    return names.get(motor_index, "")


class IPR1Collaboration(Robot):
    """IPR1 robot controller for collaborative cube and ball passing."""

    def __init__(self):
        super().__init__()
        self.time_step = int(self.getBasicTimeStep())

        # Initialize distance sensors
        self.distance_sensors = []
        for i in range(DISTANCE_SENSOR_NUMBER):
            ds = self.getDevice(f"ds{i}")
            if ds:
                ds.enable(self.time_step)
            self.distance_sensors.append(ds)

        # Initialize touch sensors
        self.touch_sensors = []
        for i in range(TOUCH_SENSOR_NUMBER):
            ts = self.getDevice(f"ts{i}")
            if ts:
                ts.enable(self.time_step)
            self.touch_sensors.append(ts)

        # Initialize motors
        self.motors = []
        for i in range(MOTOR_NUMBER):
            motor = self.getDevice(motor_name(i))
            self.motors.append(motor)

        # Initialize position sensors
        self.position_sensors = []
        for i in range(MOTOR_NUMBER):
            sensor = self.getDevice(f"{motor_name(i)}_sensor")
            if sensor:
                sensor.enable(self.time_step)
            self.position_sensors.append(sensor)

        # Initialize emitter and receiver for collaboration
        self.emitter = self.getDevice("emitter")
        self.receiver = self.getDevice("receiver")
        if self.receiver:
            self.receiver.enable(self.time_step)

    def simulation_step(self, steps_count=1):
        """Execute simulation steps."""
        while steps_count > 0:
            self.step(self.time_step)
            steps_count -= 1

    def motor_position(self, motor_index):
        """Get current motor position."""
        if motor_index < 0 or motor_index >= MOTOR_NUMBER:
            return float('inf')
        sensor = self.position_sensors[motor_index]
        if sensor:
            return sensor.getValue()
        return float('inf')

    def distance_sensor_value(self, sensor_index):
        """Get distance sensor value."""
        if sensor_index < 0 or sensor_index >= DISTANCE_SENSOR_NUMBER:
            return float('inf')
        ds = self.distance_sensors[sensor_index]
        if ds:
            return ds.getValue()
        return float('inf')

    def object_detected_in_gripper(self):
        """Check if an object is detected in the gripper."""
        value_center = self.distance_sensor_value(4)
        value_right1 = self.distance_sensor_value(5)
        value_right2 = self.distance_sensor_value(6)
        return (value_center + value_right1 + value_right2) > 80

    def position_reached(self, motor_index, target_position):
        """Check if motor has reached target position."""
        if motor_index < 0 or motor_index >= MOTOR_NUMBER:
            return False
        sensor = self.position_sensors[motor_index]
        if sensor:
            return abs(sensor.getValue() - target_position) <= POSITION_TOLERANCE
        return False

    def set_motor_position(self, motor_index, position):
        """Set motor target position without waiting."""
        if motor_index < 0 or motor_index >= MOTOR_NUMBER:
            return
        motor = self.motors[motor_index]
        if motor:
            motor.setPosition(position)

    def move_to_init_position(self):
        """Move all motors to initial position (0.0)."""
        for i in range(MOTOR_NUMBER):
            self.set_motor_position(i, 0.0)

        # Check if position reached
        for i in range(MOTOR_NUMBER):
            while not self.position_reached(i, 0.0):
                self.step(self.time_step)

    def move_to_position(self, motor_positions, move_gripper=False):
        """Move motors to specified positions and wait."""
        motor_count = MOTOR_NUMBER if move_gripper else GRIPPER_MOTOR
        for i in range(motor_count):
            self.set_motor_position(i, motor_positions[i])

        # Check if position reached
        for i in range(motor_count):
            while not self.position_reached(i, motor_positions[i]):
                self.step(self.time_step)

    def open_gripper(self, position=0.662):
        """Open the gripper to specified position."""
        self.set_motor_position(GRIPPER_MOTOR, position)
        while not self.position_reached(GRIPPER_MOTOR, position):
            self.step(self.time_step)

    def close_gripper(self):
        """Close the gripper."""
        self.set_motor_position(GRIPPER_MOTOR, 0.0)

        # Wait until it is closed as much as possible
        previous_gripper_position = float('inf')
        while True:
            current_gripper_position = self.motor_position(GRIPPER_MOTOR)
            if abs(current_gripper_position - previous_gripper_position) <= POSITION_TOLERANCE:
                break
            previous_gripper_position = current_gripper_position
            self.step(self.time_step)

    def grab_object(self, grab_position):
        """Grab an object (cube or ball) at the specified position."""
        # Set motors position objectives
        for i in range(MOTOR_NUMBER):
            if i == UPPER_ARM_MOTOR:
                self.set_motor_position(UPPER_ARM_MOTOR, UPPER_ARM_MOTOR_TRANSITION_POSITION)
            else:
                self.set_motor_position(i, grab_position[i])

        # Check if position reached
        for i in range(MOTOR_NUMBER):
            if i == UPPER_ARM_MOTOR:
                position = UPPER_ARM_MOTOR_TRANSITION_POSITION
            else:
                position = grab_position[i]
            while not self.position_reached(i, position):
                self.step(self.time_step)

        # Lower arm
        self.set_motor_position(UPPER_ARM_MOTOR, grab_position[UPPER_ARM_MOTOR])
        while not self.position_reached(UPPER_ARM_MOTOR, grab_position[UPPER_ARM_MOTOR]):
            self.step(self.time_step)

        # Wait until sensor detects an object
        while not self.object_detected_in_gripper():
            self.step(self.time_step)

        self.close_gripper()

    def wait_for_signal(self, state_signal):
        """Wait for a specific signal from the other robot."""
        while True:
            if self.receiver.getQueueLength() > 0:
                data = self.receiver.getInts()
                if data[0] == state_signal:
                    self.receiver.nextPacket()
                    break
                self.receiver.nextPacket()
            self.step(self.time_step)

    def emit_signal(self, state_signal):
        """Emit a signal to the other robot."""
        message = struct.pack('i', state_signal)
        self.emitter.send(message)

    def give_object(self, give_signal, leave_signal, throw_signal, drop_position=None):
        """Give the object to the other robot."""
        if drop_position is None:
            drop_position = DROP_POSITION
        self.set_motor_position(UPPER_ARM_MOTOR, -0.726919)
        self.simulation_step(5)
        self.move_to_position(drop_position, False)

        self.emit_signal(give_signal)
        self.wait_for_signal(leave_signal)

        # Wait a moment to let the other robot grip the cube securely
        self.simulation_step(15)

        self.open_gripper(1.0)

        # Rotate wrist
        self.set_motor_position(ROTATIONAL_WRIST_MOTOR, -3.015)
        while not self.position_reached(ROTATIONAL_WRIST_MOTOR, -3.015):
            self.step(self.time_step)

        # Raise arm
        self.set_motor_position(UPPER_ARM_MOTOR, 0.0)
        while not self.position_reached(UPPER_ARM_MOTOR, 0.0):
            self.step(self.time_step)

        self.emit_signal(throw_signal)


def main():
    ipr = IPR1Collaboration()

    # Phase 1: Pass all cubes
    for i in range(CUBE_NUMBER):
        if i > 0:
            ipr.wait_for_signal(GRAB_CUBE)

        ipr.grab_object(GRAB_CUBE_POSITIONS[i])
        ipr.give_object(GIVE_CUBE, LEAVE_CUBE, THROW_CUBE)

    # Phase 2: Pass extra cubes to same box as original cubes
    for i in range(EXTRA_CUBE_NUMBER):
        if i > 0:
            ipr.wait_for_signal(GRAB_EXTRA)
        else:
            # Wait for signal that first batch of cubes is done
            ipr.wait_for_signal(GRAB_EXTRA)

        ipr.grab_object(GRAB_EXTRA_POSITIONS[i])
        ipr.give_object(GIVE_EXTRA, LEAVE_EXTRA, THROW_EXTRA)  # Same drop position as first 3 cubes

    ipr.move_to_init_position()


if __name__ == "__main__":
    main()
