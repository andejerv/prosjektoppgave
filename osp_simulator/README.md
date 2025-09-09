# milliampere_osp_sim
Models and simulators for the milliAmpere 1 & 2 autonomous passenger ferries built on the Open Simulation Platform

# Python dependencies
The `QuayModel` and `UltrasonicDistanceSensor` FMUs are running in Python, wrapped using the `pythonfmu` package. These requires that you have a Python interpreter installed (install through https://www.python.org/, do not use the Microsoft Store). Furthermore, a couple of packages has to be installed using pip:
- `numpy`
- `pythonfmu`
- `quay_models`
    - This package is located in this repo. Install it by navigating to `Python models\quay_models` and run `pip install -e .`

Attempting to run these FMUs withouth the dependencies installed will result in runtime exceptions, without very reasonanble output (at least for mortals).
