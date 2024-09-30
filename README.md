# Oxebots Observers

This package contains the observer modules used by the **Oxebots** team for the **SSL RoboCup league**. It includes nodes that watch the game and retrieve the game state from messages received from the [**Oxebots Comms**](https://github.com/OxeBots/oxebots_comms) package, aggregating and processing the data to provide a comprehensive view of the game state within our ROS2 infrastructure.

## Table of Contents
- [Oxebots Observers](#oxebots-observers)
  - [Table of Contents](#table-of-contents)
  - [Features](#features)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
  - [Usage](#usage)
  - [Reporting Issues](#reporting-issues)
  - [License](#license)

## Features

- Aggregates and processes game state data from messages received via [**Oxebots Comms**](https://github.com/OxeBots/oxebots_comms) package.
- Provides real-time game state information including robot positions and ball position.
- Compatible with ROS2 Humble on Ubuntu 22.04.

## Prerequisites

- **Operating System**: Ubuntu 22.04 LTS.
- **ROS2 Distribution**: Humble Hawksbill.

## Installation

To install the package, clone the repository into your colcon workspace and build it:

```bash
# Source your ROS2 environment
source /opt/ros/humble/setup.bash

# Navigate to your colcon workspace
cd ${YOUR_COLCON_WORKSPACE}/src

# Clone the Oxebots Observers repository
git clone git@github.com:OxeBots/oxebots_observers.git

# Navigate back to the workspace root
cd ..

# Install the dependencies
rosdep install --from-paths src --ignore-src -r -i -y --rosdistro=$ROS_DISTRO

# Build the workspace
colcon build --packages-select oxebots_observers

# Source the workspace
source install/setup.bash
```

*Note:* This package is part of the Oxebots software stack and depends on other packages from the team, including [**Oxebots Comms**](https://github.com/OxeBots/oxebots_comms). You can find our complete software stack at [OxeBots/software_ws](https://github.com/OxeBots/software_ws).

## Usage

After building the package, you can run a observer node using:

```bash
ros2 run oxebots_observers game_observer_node
```

This will start the `game_observer_node`, which subscribes to robot and ball data topics provided by [**Oxebots Comms**](https://github.com/OxeBots/oxebots_comms), processes the incoming messages, and publishes the aggregated game state to the `game_data` topic.

## Reporting Issues

If you encounter any issues or have suggestions for improvements, please open an issue on the [GitHub repository](https://github.com/OxeBots/oxebots_observers/issues).

## License

This project is licensed under the **GPL-3.0 license** - see the [LICENSE](LICENSE) file for details.
