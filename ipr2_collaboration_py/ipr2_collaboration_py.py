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
             Control the IPR2 to take the cube passed by the
             other robot and throw it in the box.
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

OBJECT_NUMBER = 3

# IPR2 poses
WAIT_POSITION = [3.004520, -0.03308, 2.39263, -2.27029, 0.00000, 0.66207]
GRAB_POSITION = [3.004520, -0.03307, 3.47000, -2.27029, 0.00000, 0.66207]
THROW_POSITION = [2.513930, -0.00000, 3.39598, -2.27029, -1.33815, 0.00000]


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


class IPR2Collaboration(Robot):
    """IPR2 robot controller for collaborative cube catching and throwing."""

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

    def throw_cube(self):
        """Throw the cube into the box."""
        # First move to a "wind-up" position to ensure consistent throw momentum
        # Go back further than THROW_POSITION to build up rotation speed
        WINDUP_BASE_POSITION = 1.5  # Start further back for more momentum
        
        # Set all motors to throw position first
        for i in range(GRIPPER_MOTOR):
            if i == BASE_MOTOR:
                self.set_motor_position(i, WINDUP_BASE_POSITION)
            else:
                self.set_motor_position(i, THROW_POSITION[i])
        
        # Wait for all motors to reach position
        for i in range(GRIPPER_MOTOR):
            if i == BASE_MOTOR:
                while not self.position_reached(i, WINDUP_BASE_POSITION):
                    self.step(self.time_step)
            else:
                while not self.position_reached(i, THROW_POSITION[i]):
                    self.step(self.time_step)
        
        # Small delay to stabilize before throwing
        self.simulation_step(5)

        # Rotate base - the full swing from 1.5 to 5.95391
        self.set_motor_position(BASE_MOTOR, 5.95391)

        # Check distance from target - release point
        while True:
            if self.motor_position(BASE_MOTOR) > 5.30:
                break
            self.step(self.time_step)

        # Open gripper at the right moment
        self.set_motor_position(GRIPPER_MOTOR, 0.72)

        # Raise arm so that it can throw better
        self.set_motor_position(UPPER_ARM_MOTOR, -0.0330743)

        # Wait until movement completed
        while not self.position_reached(BASE_MOTOR, 5.95391):
            self.step(self.time_step)
        while not self.position_reached(UPPER_ARM_MOTOR, -0.0330743):
            self.step(self.time_step)
        while not self.position_reached(GRIPPER_MOTOR, 0.72):
            self.step(self.time_step)

    def take_cube(self):
        """Take the cube from the other robot."""
        self.wait_for_signal(GIVE_CUBE)

        self.open_gripper()

        self.move_to_position(GRAB_POSITION)

        self.close_gripper()

        self.emit_signal(LEAVE_CUBE)

        self.wait_for_signal(THROW_CUBE)


def main():
    ipr = IPR2Collaboration()

    for i in range(OBJECT_NUMBER):
        ipr.move_to_position(WAIT_POSITION)
        ipr.take_cube()
        if i < (OBJECT_NUMBER - 1):
            ipr.emit_signal(GRAB_CUBE)
        ipr.throw_cube()

    ipr.move_to_init_position()


if __name__ == "__main__":
    main()

