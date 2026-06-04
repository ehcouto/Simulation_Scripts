# Version

- v1.0 - Initial Release

# Main Features

- PMSM Motor implemented in dq integrated with Runge Kutta 4a Order.

# Installation Details:
- Algorithm is configured to be executed in VS Code editor (Windows): tasks.json & launch.json were configured to properly compile the code. 
- GCC compiler shall be installed in the computer, and path might be updated in tasks.json & launch.json files.
- GCC compiler used can be downloaded on [winlibs.com](https://winlibs.com/). Use GCC compiler version 16.1.0 or higher.
- Once GCC compiler is installed and paths updated, execute the task by pressing ctrl + shift + B.
- There are 2 different tasks:
-- ArchSim_FOC build tools: Used to compiled the code. 
-- Build + Run + Plot: Will build, run and plot the data using the python script. Python used is version 3.6


# Possible Updates

- Ideal / Real Inverter Model to allow the duty cycles to be the input to the PMSM model.
- ADC Sensor Simulations