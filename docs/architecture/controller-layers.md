# Controller Layers

GriffinSim's rebuilt scenario stack now supports four controller-expression layers, each compiling deterministically into actuator timelines:

1. **Raw PWM schedules**
   - direct `pwm_schedule` ranges
2. **1D waypoint schedules**
   - `x_waypoints`
   - compiled to 1-channel PWM
3. **2D holonomic controller schedules**
   - `xy_controller_waypoints`
   - compiled to 2 PWM channels `(x, y)`
4. **2D pose and velocity-aware pose controller schedules**
   - `xytheta_controller_waypoints`
   - `xytheta_velocity_waypoints`
   - compiled to 3 PWM channels `(x, y, theta)`

## Design intent

These controller layers are deterministic compilers, not full control-theory or kinematics-complete controllers.
They exist to let regression scenarios express progressively richer intent while preserving byte-stable replay.

## Current limitation

The controller layers approximate motion through simple feedforward/feedback compilation to PWM channels. They do not yet implement a full holonomic chassis controller, path planner integration, or field-relative trajectory tracking with kinematic constraints.
