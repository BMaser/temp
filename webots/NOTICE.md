# Notice for the Webots base lap

- The world `worlds/base_lap.wbt` uses the Crazyflie robot of Webots by reference
  (`webots://projects/robots/bitcraze/crazyflie/protos/Crazyflie.proto`, Webots R2025a). It is not copied. Its header says:
  "Copyright Cyberbotics Ltd. Licensed for use only with Webots." The Webots repository (github.com/cyberbotics/webots, tag
  R2025a) is under the Apache License 2.0; a copy of the Webots license file and of the files we read is kept with the run in
  `coursera_mirror/mirrors/Autonomous_Racing_Drone_Path/runs/2026-10-15_webots_base/sources/` (outside this repository).
- `controllers/base_lap/lap_core.py` takes from the Webots example controller of the Crazyflie only FACTS: the motor names
  (`m1_motor` .. `m4_motor`), their turning signs, and which motors a roll, pitch and yaw command raise
  (`controllers/crazyflie_py_wallfollowing/pid_controller.py`, MIT License, Copyright (c) 2023 Bitcraze AB). No code of it is
  copied; the controller is our own cascade.
- The gate model (frame, banners, post, collision boxes) is derived from the gate of the LSY drone racing environment
  (MIT License, Copyright (c) 2024 Learning Systems and Robotics Lab); see `assets/gates/NOTICE.md` in the private repository.
- Nothing is taken from the EPFL course repository `lis-epfl/micro-502` (no license).
