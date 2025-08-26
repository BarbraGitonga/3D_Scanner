# 3D Scanner Project

The goal of this project is to create a handheld 3D scanner that is cheap and used by hobbyists.

The project is mainly in the STM32F756G-DISCO board.

## Requirements

- STM32F7 board - If you have another board you could change the libraries to work for your preferred board.
- STM32CubeIDE - main IDE I am using.
- MPU6050 - an IMU sensor for recording spatial parameters.
- HMC5888L3 - magnetometer sensor board that is used or heading.
- VL530XV2 - Time of Flight sensor to record the distance

## Approach

The project will have the sensors record three parameters while the handheld 3d scanner is in motion: roll, yaw, pitch and distance. The recorded data will be stored in an SD card that will be read by your computer.

The data will be used to form a point cloud representation of the object in 3D space.
The Point Cloud representation will then be meshed to form a closed 3D object.

There will be use of AI in the future to enhance the 3D image created if need be. The mesh will the be converted to .STL or .STEP format and can therefore be opened on a 3D modelling sofware such as Fusion 360 and Blender to be modified for future use.