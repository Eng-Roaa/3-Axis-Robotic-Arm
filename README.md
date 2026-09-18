# 3-Axis Robotic Arm | Design, Simulation, and Control 🦾

A 3-axis robotic arm developed as a complete robotics project, covering mechanical design, simulation, kinematics, hardware development, and motion control.

## 📌 Project Overview

The project started with the **CAD design** of the robotic arm and its mechanical components using **SolidWorks**. Before fabrication, the system was tested in a simulation environment to verify the robot's motion and kinematic calculations.

After validating the design and motion, the physical robotic arm was fabricated using **3D printing** and integrated with stepper motors and electronic control components.

## 🔹 CAD Design

The robotic arm and its mechanical components were designed using **SolidWorks**.

The design includes the main arm structure, joints, and **25:1 strain wave gearboxes** used with the stepper motors.

## 🔹 Simulation & Kinematics

A Python-based control system was developed to work with the robotic arm simulation.

The system implements:

* Forward Kinematics (FK)
* Inverse Kinematics (IK)
* Joint position control
* Robot motion simulation

The kinematic calculations were tested before moving to the physical implementation.

## 🔹 Hardware Development

After the simulation and testing stage, the robotic arm components and gearboxes were **3D-printed**.

### Main Hardware

* 3 × NEMA 17 Stepper Motors
* 3 × TMC2208 Stepper Motor Drivers
* CNC Shield V3
* Arduino
* 25:1 Strain Wave Gearboxes
* 3D-printed mechanical components
* 3 Limit switches

## 🔹 Motion Control

The robotic arm is controlled using a **Python application running on a PC**.

Python performs the main calculations, including forward and inverse kinematics, and sends the target joint angles to the **Arduino through serial communication**.

The Arduino then controls the stepper motors according to the received commands.

## 🔹 Synchronized Motion

A synchronized motion mechanism was implemented to ensure that the three motors **start and stop at the same time**, even when different angular movements are required for each joint.

This allows the joints to complete their movements together rather than having one motor finish earlier than the others.

## 🔹 Homing & Limit Switches

A homing procedure was implemented to establish a known starting position for the robotic arm.

**Limit switches** are used to detect the mechanical limits of the joints and stop the corresponding motion when the arm reaches the limit.

## 🛠️ Technologies Used

| Category      | Technologies                 |
| ------------- | ---------------------------- |
| CAD           | SolidWorks                   |
| Programming   | Python, Arduino/C++          |
| Control       | Serial Communication         |
| Kinematics    | Forward & Inverse Kinematics |
| Motors        | NEMA 17 Stepper Motors       |
| Drivers       | TMC2208                      |
| Controller    | Arduino + CNC Shield V3      |
| Manufacturing | 3D Printing                  |


## 📷 Project Media

Project images and videos are available in the `Media` folder.


## 🎥 Project Demonstration

A demonstration video of the robotic arm will be added here.
https://drive.google.com/file/d/13j7fY6XMhpEBEr_zSgsBxvps_PN-o1Ip/view?usp=drive_link

## 🚀 Future Improvements

* Improve positioning accuracy
* Implement smoother trajectory planning
* Add acceleration and deceleration profiles
* Improve the simulation environment
* Develop more advanced motion planning
* Improve the user interface

## 👩‍💻 Author

**Roaa Ghanem**

Biomedical Engineer

---

*This project was developed for educational and practical exploration of robotics, CAD, kinematics, and motion control.*
