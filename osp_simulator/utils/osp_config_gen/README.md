# Guide
This tool introduces a layer of abstraction to easily generate large numbers of OSP configs with slightly varying initial conditions and environmental loads. A full list of currently supported configuration options is included below.

See config_generation_plan.json for an example.

To use the tool, it is recommended to open this folder in VS Code and create or use a Python virtual environment. Then run the following:
```
pip install -r requirements.txt
pip install -e .
python src/osp_config_gen.py config_generation_plan.json
```
The configurations specified in config_generation_plan.json will generated and saved in the output folder, which by default is 'osp_configs'.

# Necessary parameters
Generation will fail if all configs do not (at least) include the following parameters, with one of the specified options selected:
- config_name
- vessel
    - estelle
    - ma2

# Optional parameters
- sim_type (Defaults to SIL. HIL is not yet supported)
    - SIL
    - HIL-NO-DP
- initial_position (Defaults to 'ravnkloa'. Can also be overridden by specifying a 3DOF initial position, where heading is given clockwise in radians)
    - ravnkloa
    - fosenkaia
    - midpoint (Additional positions can be specified in config.yml)
- invert_transit (Inverts heading from standard operating direction, defaults to false)
    - true
    - false
- hatch_position (Defaults to closed. 'open' and 'close' will automatically refer to the docked end of the vessel. Not supported on ma2)
    - open
    - closed
    - fore_open
    - aft_open
    - both_open
- wave_loads (Presets can be overriden by using a list of six frequency amplitudes. Defaults to none)
    - none
    - low
    - medium
    - high
- wind_speed (Mean wind speed in m/s. Defaults to 0.0)
- wind_direction (Mean wind direction in degrees (direction wind is blowing towards). Values must be a float between 0 and 360 (defined clockwise). Defaults to 0/north)
- wind_direction_noise (Noise applied to wind direction. A value of 10 results in roughly +-15 degrees. Defaults to 0.0)