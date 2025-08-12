# EmioLabs_Mimo
Labs for dynamic control of Emio with Multiple-Input Multiple-Output.

Emio is a robotic platform designed for educational purposed by [Compliance Robotics](https://compliance-robotics.com/compliance-lab/).
This lab is higly related to the lab with Single-Input Single-Output (SISO) control, which can be found in the [EmioLabs_Siso](https://github.com/AlessandriniAntoine/EmioLabs_Siso)

# Installation
An additional python package ([control](https://python-control.readthedocs.io/en/0.10.2/)) is required to run the labs. Is it included in the `requirements.txt` file. To install it, run the following command:

```bash
pip install -r requirements.txt
```

# Objectives

The objective of this lab is to control the Emio platform using a Multiple-Input Multiple-Output (MIMO) system. The lab will cover the following topics:
1. Understanding the dynamics of the Emio platform.
2. Model order reduction techniques.
3. Linear system identification.
4. Designing a MIMO controller for the Emio platform.
5. Design state estimation techniques for the MIMO system.
6. Applying the MIMO control structure to the Emio platform.
