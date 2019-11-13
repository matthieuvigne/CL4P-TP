# simulation

This folder contains a python simulation for Claptrap control, relying on the open-source
kinematics and dynamics library ```pinocchio```.

This simulation basically takes a URDF representation of Claptrap as input ; forward dynamics is done
using minimal coordinates, assuming a slipless contact of the wheel with the ground.
For more details about the simulation and the applied controllers, see ControlDocumentation.pdf

## Installation

This code depends on the following python packages:

 - ```pinocchio``` python bindigns, see [github](https://github.com/stack-of-tasks/pinocchio) for installation instructions.
 - ```meshcat```, which can be installed simply using ```pip install --user meshcat```.

## Running the simulation

To simply run a simulation, first start the meshcat viewer with the following command
```
meshcat-server
```

Open a web browser at the given address to visualize a rendering of the simulation. Then in another terminal run

```
python simulation.py
```

This outputs a csv log file in the ```/tmp/``` directory, that can be visualized using ```log_plot```.
