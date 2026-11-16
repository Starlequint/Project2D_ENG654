# Project2D_ENG654

This repository contains the MATLAB, Simulink, notebook, and report materials for a 2D kinematically redundant planar parallel manipulator study, including forward/inverse kinematics and animation of the robot motion.

- Jacobian.pdf — PDF of the Jacobian derivation and motion/constraint analysis used for the robot model.
- project_2D.pdf — Final project report or summary PDF covering the 2D robot design and kinematic analysis.
- project_kinematics.m — MATLAB script that computes the robot geometry and plots the solved 2D configuration for the three 5-bar branches.
- animate_parallel_robot_rotation_simscape.m — MATLAB script that builds a Simscape animation model and replays the robot motion from the notebook trajectory.
- Kinematically_Redundant_Planar_Parallel_Manipulator_six_input_rotation.slx — Simulink model for the six-input rotational motion animation of the robot.
- parallel_5L_robot2.mlx — MATLAB Live Script used for interactive exploration of the 5-bar parallel robot kinematics and behavior.
- forward_kinematic_complete.ipynb — Jupyter notebook solving the complete forward kinematics of the robot and its end-effector triangle.
- reverse_kinematic_complete.ipynb — Jupyter notebook solving the complete inverse kinematics from a desired platform pose and connector directions.
- reverse_kinematic_animation_0_1m.ipynb — Notebook focused on the inverse-kinematics animation for the 0.1 m motion case.
- overleaf.zip — Export of the Overleaf/LaTeX project files used for the written report part1.

The top-level scripts and notebooks are the main working files for the project; the slprj files are generated artifacts used by the MATLAB/Simulink environment rather than hand-edited source code.
