# FOC Beko SIL Architecture Simulation

## Author
- Eduardo H Couto (eduardo.h.couto@beko.com)

## Version

- **v1.3:** Improved PWM Inverter Model + Architecture Updates (06/2026)
- **v1.2:** Added Inverter Models (06/2026)
- **v1.1:** Added different motor features and ADC sensor simulation (06/2026)
- **v1.0:** Initial Release (05/2026)

## Main Features

- PMSM Motor implemented in dq integrated with Runge Kutta 4a Order.
- Beko FOC Code Integrated.
- Basic Board Features Implemented.

## Installation Details:
- Algorithm is configured to be executed in VS Code editor (Windows): tasks.json & launch.json were configured to properly compile the code. 
- GCC compiler shall be installed in the computer, and path might be updated in tasks.json & launch.json files.
- GCC compiler used can be downloaded on [winlibs.com](https://winlibs.com/). Use GCC compiler version 16.1.0 or higher.
- Once GCC compiler is installed and paths updated, execute the task by pressing ctrl + shift + B.
- There are 2 different tasks:

    1- **ArchSim_FOC build tools**: *Used to simply compile the code*. 

    2- **Build + Run + Plot**: *Will build, run and plot the data using the python script. Recommended WinPython version 3.14 or higher + pandas + matplotlib libraries.*
- To install WinPython, use the following link "https://winpython.github.io/" - For this simulation version used (WinPython64-3.14.5.0).
- By pressing F5, you can start the debugger window.

## Next Steps to Implement:

- Inverter Non-Linearities Simulation.
- VBus Voltage model to include voltage ripple in the simulation. 