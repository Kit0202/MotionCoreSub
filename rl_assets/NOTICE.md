# Bundled Unitree assets

Source: the user's uploaded unitree_rl_gym.zip.

The pretrained G1 policy is copied unchanged from deploy/pre_train/g1/motion.pt.
The 12-joint scene, model and referenced meshes are copied unchanged from
resources/robots/g1_description. Original copyright and redistribution terms
are retained in LICENSE in this directory.

The G1 YAML configuration retains the supplied policy gains, observation scales,
action scale, default angles, physics timestep and control decimation. Its model
and policy paths are made relative to this bundle. Live commands supersede
cmd_init and the worker runs until Stop/window close rather than the YAML timer.

MotionCore's backend/rl_policy.py and backend/rl_worker.py adapt the observation,
PD control and gravity calculations from the supplied MuJoCo deployment. Changes
include CPU inference, command ramping, explicit Euler-angle telemetry, measured
velocity, atomic session messages, heartbeat checks, failure reporting and
controlled simulation shutdown. The pretrained policy was not retrained.
