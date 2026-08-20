# MATLAB Model Guide

The `descriptions/*/matlab` folders define the acados optimal-control models
used by `acados_nmpc_controller`. Use these files when changing vehicle
dynamics, cost terms, constraints, horizons, or generated solver code.

## Directory Layout

```text
descriptions/bluerov/bluerov2_heavy/matlab/
|-- main.m
|-- BlueROV_heavy_model.m
|-- BlueROV_heavy_model_sim.m
|-- env.sh
`-- c_generated_code/

descriptions/subcat/matlab/
|-- main.m
|-- subcat_model.m
|-- env.sh
`-- c_generated_code/
```

Generated files such as `acados_ocp_nlp.json`, `acados_sim.json`, `build/`,
MEX files, and `c_generated_code/` are produced by acados. Do not hand-edit
generated C or MEX files unless you are debugging generated code directly.

## Prerequisites

This project expects acados v0.4.5. Install it using the README build
instructions first, then export its install path before running MATLAB:

```bash
export ACADOS_INSTALL_DIR=$HOME/acados
export LD_LIBRARY_PATH=$ACADOS_INSTALL_DIR/lib:$LD_LIBRARY_PATH
```

The official acados MATLAB/Octave setup guide is:
[MATLAB + Simulink and Octave Interface](https://docs.acados.org/matlab_octave_interface/index.html).
Use it for MATLAB path, CasADi, MEX, and platform-specific setup details while
keeping this repository's acados checkout at `v0.4.5`.
CasADi 3.7 has also been tested successfully with these MATLAB models.

The helper script in each MATLAB directory adds acados, CasADi, and MEX paths.
Source it from the target model directory:

```bash
cd ~/underwater_ws/src/underwater_ros2_control/descriptions/subcat/matlab
source env.sh
```

`env.sh` must be sourced, not executed. If `ACADOS_INSTALL_DIR` is already set,
that value is used; otherwise the script falls back to its default path.

## BlueROV2 Heavy

Path:

```bash
cd ~/underwater_ws/src/underwater_ros2_control/descriptions/bluerov/bluerov2_heavy/matlab
```

Files:

- `BlueROV_heavy_model.m`: OCP model used by the ROS 2 NMPC controller.
- `BlueROV_heavy_model_sim.m`: plant simulation model used by `main.m`; it
  includes thrust-health parameters for simulation.
- `main.m`: builds the acados OCP and simulator, runs a closed-loop MATLAB
  tracking simulation, and regenerates acados code.

Model dimensions:

| Item | Value |
| --- | --- |
| State | 13: position, quaternion, linear velocity, angular velocity |
| Input | 8 thruster commands |
| Solver ID | `bluerov2_heavy` |
| acados model name | `BlueROV_Heavy` |

Run from MATLAB:

```matlab
main
```

Or from a shell:

```bash
source env.sh
matlab -batch "main"
```

## Subcat

Path:

```bash
cd ~/underwater_ws/src/underwater_ros2_control/descriptions/subcat/matlab
```

Files:

- `subcat_model.m`: OCP and simulation model used by acados.
- `main.m`: builds the acados OCP and simulator, runs a closed-loop MATLAB
  tracking simulation, and regenerates acados code.

Model dimensions:

| Item | Value |
| --- | --- |
| State | 17: position, quaternion, linear velocity, angular velocity, 4 servo angles |
| Input | 8: 4 thruster commands and 4 servo angular-rate commands |
| Solver ID | `subcat` |
| acados model name | `subcat` |

Run from MATLAB:

```matlab
main
```

Or from a shell:

```bash
source env.sh
matlab -batch "main"
```

## Regenerating Solver Code

Use this workflow after editing any model or OCP settings:

1. Edit the model file or `main.m`.
2. Source the local acados environment:

   ```bash
   cd ~/underwater_ws/src/underwater_ros2_control/descriptions/bluerov/bluerov2_heavy/matlab
   source env.sh
   ```

   Or:

   ```bash
   cd ~/underwater_ws/src/underwater_ros2_control/descriptions/subcat/matlab
   source env.sh
   ```

3. Run `main` in MATLAB.
4. Confirm that `c_generated_code/`, `acados_ocp_nlp.json`, and
   `acados_sim.json` were regenerated.
5. Rebuild the ROS 2 packages that consume the generated solver:

   ```bash
   cd ~/underwater_ws
   colcon build --symlink-install --packages-select \
     acados_nmpc_controller \
     bluerov2_heavy \
     subcat
   source install/setup.bash
   ```

The controller solver directories are symbolic links to these generated-code
folders:

```text
controllers/acados_nmpc_controller/solvers/bluerov2_heavy/c_generated_code
controllers/acados_nmpc_controller/solvers/subcat/c_generated_code
```

Because of those links, regenerating code under `descriptions/*/matlab` updates
the source used by the controller build.

## Keeping ROS Configuration Consistent

When model dimensions or command meanings change, update the matching ROS
configuration as well:

- `controllers/acados_nmpc_controller/solvers/<solver_id>/solver.yaml`
- `descriptions/bluerov/bluerov2_heavy/config/gazebo.yaml`
- `descriptions/bluerov/bluerov2_heavy/config/real.yaml`
- `descriptions/bluerov/bluerov2_heavy/urdf/*.xacro`
- `descriptions/subcat/config/gazebo.yaml`
- `descriptions/subcat/config/real.yaml`
- `descriptions/subcat/urdf/*.xacro`

Typical changes that need ROS-side updates:

- Changing `nx`, `nu`, or `model.name`.
- Reordering state variables or control inputs.
- Changing thruster or servo command units.
- Changing input limits in `main.m` and expecting matching runtime limits.
- Changing hydrodynamic parameters that are also represented in Gazebo plugins.

## Notes

- MATLAB model files use the internal NED/FRD convention used by the controller.
- Gazebo visualization and RViz topics may convert positions or paths for
  display, so compare frames carefully when validating trajectories.
- Machine-specific MEX binaries can depend on the MATLAB, compiler, and acados
  versions used to generate them.
- If MATLAB cannot find `AcadosOcp`, `AcadosModel`, or CasADi symbols, re-source
  `env.sh` and verify `ACADOS_INSTALL_DIR`.
