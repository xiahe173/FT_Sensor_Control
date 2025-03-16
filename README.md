# Dummy Robot Hand Guidance with 6 Axis FT Sensor

## Firmware

- **Pins:**
  Pin configuration on Arduino is as follows:

  ```c++
  const int CLK = 13;
  const int DOUT[] = {7, 8, 9, 10, 11, 12};
  ```

- **Calibration:**
  By default, the firmware calibrates the sensor by reading 100 samples and taking the average. This is done once the sensor is powered on.
  To recalibrate, send 't\n' to the serial port.

## Software

The python script reads the sensor values, calculates the force and torque, followed by the hand guidance algorithm, and finally sends the desired position to the Dummy robot.
