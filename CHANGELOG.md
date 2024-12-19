# Changelog
### WinchModels v0.3.6 - 2024-12-19
#### Added
- example speed_control_step which tests the step response of the asynchronous generator
- example speed_control_step_tc which tests the step response of the speed controller connected to the torque
  controlled generator

#### Changed
- add field `upwind_dir`, remove vector `v_wind_ref` from `Settings`and yaml files
- add the fields `max_acc`, `p_speed` and `i_speed` to `Settings`and yaml files;
  the `max_acc` value is now taken into account correctly for both winch models
- the torque controlled winch can now also operate with a `set_speed` value, using a PI controller

