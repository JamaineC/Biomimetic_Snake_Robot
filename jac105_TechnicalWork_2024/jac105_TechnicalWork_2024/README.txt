
# Biomimetic Snake Robot Project

## Project Description

This project simulates a biomimetic snake robot in Webots, designed to mimic the complex locomotion of biological snakes. The robot utilizes various sensors and actuators to navigate through complex environments, demonstrating capabilities such as obstacle avoidance and maze traversal. This simulation aims to explore the potential applications of biomimetic principles in robotics, enhancing the robot's ability to operate in constrained spaces effectively.

## File Structure

- `first_controller.cpp`: Main controller file containing the logic for robot movement, sensor integration, and environmental interaction.

## Prerequisites

- **Webots R2020b or later**: Ensure you have the latest version of Webots installed to support all features used in this project.

## Setup and Installation

1. **Install Webots from their website**
2. **Open the project in Webots**:
   - Launch Webots.
   - Open the project by navigating to the project directory and selecting the serpentineAvoidance world file.

3. **Build the controller** (if not automatically built):
   - In Webots, go to `Tools > Build`.

## Running the Simulation

- Open the world file associated with the project.
- uncomment the type of movement you would like to perform
- rebuild the controller
- Press the `Play` button in Webots to start the simulation.
- The snake robot will automatically begin executing the programmed behaviors based on sensor inputs and predefined logic.

## Controller Details

### Overview

`first_controller.cpp` integrates various Webots API functionalities to control a simulated snake robot. It includes implementations for sensor readings, motor control, PID control for navigation, and basic logic for obstacle avoidance and maze traversal.

### Key Components

- **Sensors**:
  - `DistanceSensor`: Used for detecting obstacles.
  - `RangeFinder`: Provides depth perception for obstacle detection.
  - `InertialUnit`: Measures the robot's orientation.
  - `TouchSensor`: Detects physical contact with surfaces.
  - `Camera`: Used for object recognition (though not fully functional).
  
- **Actuators**:
  - `Motor`: Controls the movement of the joints for the snake-like locomotion.

### Functionality

- **Motion Control**: Defines sinusoidal movement patterns for lateral and vertical joint motors mimicking snake movement.
- **PID Control**: Adjusts the robot's heading to maintain a straight path or correct its trajectory based on sensor feedback.
- **Obstacle Avoidance**: Uses sensor data to avoid collisions with obstacles by adjusting the robot's path dynamically.
- **Maze Traversal**: Employs a strategic approach to navigate through mazes using sensor inputs to make turning decisions.

## Customization

- Users can adjust the PID coefficients, sensor thresholds, and movement parameters to experiment with different behaviors and responses.
- Additional sensors or actuators can be added by modifying the controller code and the corresponding robot model in Webots.
- Users can change world files to see previous design iterations or different scenario setups.

## Troubleshooting

- **Build Issues**: Ensure all dependencies are correctly configured and that Webots is set up to use the correct compiler for C++.
- **Simulation Errors**: Check the console in Webots for error messages that can provide insights into issues like sensor malfunctions or logic errors in the controller.


## License

This project is released under the [MIT License](LICENSE.txt).
