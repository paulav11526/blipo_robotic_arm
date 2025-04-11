# blipo_robotic_arm
This repository contains the control files for the Yahboom Jetcobot 7, a robotic arm powered by the Jetson Orin Nano. Blipo is a programmable 6 DOF robotic arm designed to work with ROS.

## Project Overview
The goal of the project is to control the arm and perform contact-based inspections. Based on camera feedback, the arm will move to the desired location and use depth sensing to bring the end effector into contact with the surface. 

## Features
- **ROS**: Blipo is fully integrated with ROS for control.
- **Camera Control**: The system uses vision for precise targeting

## Requirements 
- Jetson Orin Nano with Ubuntu 20.04 LTS
- ROS installed (Noetic)
- Yahboom Jetcobot 7 hardware

## Acknowledgments

This project is based on the **Yahboom Jetcobot 7**, which is powered by the **Jetson Orin Nano**. For the official documentation and additional resources, please visit the [Yahboom Jetcobot 7 GitHub repository](https://github.com/YahboomTechnology/JetCobot).

The code and setup provided in this repository build upon Yahboom's original work and extend its functionality for specific control and camera integration using ROS.
