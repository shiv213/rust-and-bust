# Rust and Bust

Jessica Singh, Shayan Azmoodeh, Shiv Trivedi

jsingh17, shayana3, shivvt2

## Project Introduction
- Gesture interpretation using an IMU
- We aim to create a gesture-based handwriting character recognition system using embedded Rust and ML applied to IMU sensor data.

## System Overview
Major components include the microcontroller to compile embedded rust, an inertial measurement unit (IMU) sensor, a concurrent algorithm to process data, and border detection to identify various gestures as inputted through the IMU.

### Roadmap
1. Set up the IMU and ensure that we are able to create a pipeline to get data from the hardware to the main software component of the project.
2. Use sensor data to train the ML model for handwriting recognition.
3. Deploy ML model to embedded Rust system.
4. Send output data back to the host machine
5. Test

## Possible Challenges
- Hardware/software incompatibility 
- Language barriers in implementing ML
- Time sink due to prototyping/testing different hardware
- Parsing raw sensor data

## References
- N/A

## Running the Project
- Clone the repository
- Plug in Pico in bootloader mode
- From the root directory, run the following command:
- `cargo run --release`