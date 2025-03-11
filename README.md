# IMUSpinalMapping

This project employs an array of five (initially aligned) ICM-20948 Inertial Measurement Units (IMUs) to create a comprehensive spinal monitoring system spanning from the thoracic (TH1) to lumbar (L5) vertebrae and to profile the customer's back.
Each sensor integrates accelerometer, gyroscope, and magnetometer functionality to capture precise movement data across all planes of motion. Strategically positioned along the spine's posterior aspect, being equidistant, these sensors cooperate 
to provide high-fidelity motion tracking and orientation measurements. While the magnetometers enhance spatial orientation accuracy, their readings are calibrated to account for environmental magnetic interference that could otherwise compromise data integrity. 
Most of the time, temporal influences still unevitably downgrade the quality of the data. 

At the system's core, an ESP32 microcontroller processes the multi-sensor data stream and enables wireless transmission for real-time analysis (with BLE being the proposed technology). 
This architecture supports immediate biofeedback for posture profiling and (bedding) advicement, creating applications ranging from clinical rehabilitation to preventative spinal health management. 
