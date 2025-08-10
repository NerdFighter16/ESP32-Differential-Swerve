# ESP32 Differential Swerve Drive Robot

Firmware for a three-wheeled, differential swerve drive robot built on the ESP32. It features holonomic movement and a shooter mechanism, all controlled via a Bluetooth gamepad.

The differential swerve drive uses two motors per module to simultaneously control wheel speed and steering angle, enabling a compact and powerful drivetrain.

## Features

*   **Holonomic Drivetrain**: Allows the robot to move and rotate in any direction simultaneously.
*   **Bluetooth Gamepad Control**: Uses the `Bluepad32` library for wireless control with modern gamepads (PS4, Xbox, etc.).
*   **PID Control**: Each module uses PID controllers for precise azimuth and wheel speed management. The code also includes commented-out structures for LQR control.
*   **Shooter & Defense Mechanism**: Equipped with a variable-speed flywheel shooter and an adjustable hood.
*   **Modular Code**: Organized into logical classes for the robot controller, swerve modules, joystick input, and sensors.

## Required Libraries

*   **`Bluepad32`**: Handles all Bluetooth gamepad input.
*   **`ESP32Encoder`**: Reads motor quadrature encoders.
*   **`QuickPID`**: Manages the PID control loops.
*   **`HardwareSerial`**: Standard ESP32 serial communication.

## Code Structure

*   `main.ino`: Main program entry point. Initializes and runs the robot.
*   `RobotController`: The main class. Manages the control loop, reads gamepad input, and commands the modules.
*   `DiffSwerveModule`: Represents a single swerve module, handling its motor control.
*   `JoystickProcessor`: Processes raw gamepad input into clean, rate-limited robot velocity commands.
*   `ModuleSensors`: Reads and processes encoder data.
*   `shooter`: Controls the flywheel, hood, and other shooter functions.
*   `Config.h`: **Crucial configuration file.** All hardware pin assignments, gear ratios, and PID gains are here.
*   `Utils.h`: Helper functions for vector math, angle wrapping, etc.

## Setup

1.  **Hardware**: Wire the ESP32 according to the pin definitions in `Config.h`.
2.  **Libraries**: Install the required libraries in your Arduino or PlatformIO environment.
3.  **Configuration**: Modify `Config.h` to match your robot's hardware (pins, encoder PPR, gear ratios).
4.  **Upload**: Compile and upload the code.
5.  **Gamepad**: Put your gamepad in pairing mode to connect. The serial monitor will show the status.