# 5-Axis-Hybrid-Printer-Klipper

A 5-axis hybrid 3D printer running Klipper, featuring a polar bed, tilting nozzle, and linear Y-axis. Developed as a mechanical engineering diploma capstone project.

This platform operates as a **true hybrid system**. It functions flawlessly as a rigid, high-speed 3-axis printer for standard slicing architectures, or can seamlessly switch modes for complex 5-axis conformal, non-planar, and support-free toolpaths.

---

## 🤝 Attribution & Inspiration

The mechanical foundation of this printer—specifically the core R-Theta polar bed and the tilting nozzle mechanism—is heavily based on the incredible open-source work of **Joshua Bird (jyjblrd)** and his [Core-R-Theta-4-Axis-Printer](https://github.com/jyjblrd/Core-R-Theta-4-Axis-Printer). 

This repository builds upon that core concept by introducing a physical linear Y-axis, updating hardware tolerances for standard assembly, and porting the entire kinematic system from RepRapFirmware/Duet3D over to Klipper using a custom XBY kinematics module.

---

## 🚀 Key Engineering & Firmware Enhancements

### 1. Kinematic Decoupling & Expanded Print Area
The integration of a physical, front-to-back linear Y-axis structure underneath the rotating polar assembly expands structural flexibility. This allows the machine to enhanced capabilities over traditional fixed-base polar variants.

### 2. Custom XBY Kinematics Module (`xby.py`)
Because Klipper does not natively support an core_x-b, a dedicated Python kinematics module (`xby.py`) was coded. 
*   **Internal X:** Manages the linear X travel via an XB1/XB2 differential belt pair.
*   **Internal Y:** Resolves the B-axis belt displacement in millimeters to calculate toolhead tilt inclination.
*   **Internal Z:** Resolves physical bed-Y linear structural motion.
*   **Auxiliary Axes (U/V):** Driven via structured `manual_stepper` routines to isolate vertical toolhead position (Z) and infinite polar bed rotation (C-axis) seamlessly.

This math layer directly mitigates virtual coordinate desync and interface jogging anomalies common in multi-axis Klipper setups.

### 3. Practical Design Refinements
*   **Y-Axis Base Plate Integration:** Engineered a dedicated moving base plate to mount the C-axis assembly. By mobilizing this entire platform front-to-back, the machine achieves its independent linear Y-axis motion.
*   **Structural Rigidity Upgrade:** Upgraded the main structural arm from standard 2020 aluminum extrusion to a heavier, more robust 3030 extrusion. This drastically improves rigidity and minimizes deflection during multi-axis toolpath execution.
*   **Heated Bed & Slip Ring Routing:** Integrated a heated bed into the continuously rotating C-axis. To allow for infinite polar rotation without wire fatigue, tangling, or snapping, the high-current heater wires are successfully routed through a continuous slip ring.
*   **Mass-Compensated Kinematics:** The integration of the heated bed and heavier base components significantly increased the rotational mass of the C-axis. The provided `printer.cfg` profile mathematically accounts for this heavy load by deliberately governing the C-axis rotational speed and acceleration, ensuring smooth, skip-free motion without inducing aggressive frame vibration.

---

## 📂 Repository Architecture

*   `/Firmware`: Contains the custom Klipper Python kinematics script (`xby.py`) and the unified stable system config (`printer.cfg`).
*   `/Hardware`: Universal STEP assembly files, modified Y-carriage STLs, Biqu H2 mounts, and frame extensions.
*   `/Slicer`: Baseline Cartesian orchestration profiles and post-processor templates.

---
