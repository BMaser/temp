# racing-drone-bench

Test runs in the cloud for an autonomous racing drone project. The goal of the project is a small
drone that flies through gates by itself, with one normal camera and a motion sensor (an IMU) on
the drone, and nothing outside the drone that measures where it is.

The tests in this repository run on GitHub Actions, on machines with no screen and no graphics
card.

## The first test

It asks two questions.

1. Does the simulator [Crazyflow](https://github.com/utiasDSL/crazyflow) draw the picture of the
   camera on the drone, with the gates of
   [LSY Drone Racing](https://github.com/utiasDSL/lsy_drone_racing), on a machine with no screen?
2. Does the published controller code of the paper MonoRace
   ([arXiv:2601.15222](https://arxiv.org/abs/2601.15222), code in
   [optimal_quad_control_RL](https://github.com/tudelft/optimal_quad_control_RL/tree/a2rl2025))
   fly in the simulator of its authors?

Each run saves its pictures and its numbers as files of that run.

## What is not in this repository

Code of other projects is not copied in. A test downloads the code that it needs when it runs.
