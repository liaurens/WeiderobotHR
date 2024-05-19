"""
# WeiderobotHR

## Overview

WeiderobotHR is a project aimed at developing an autonomous robot capable of navigating through a weide (meadow) to perform tasks such as spreading hay. The robot utilizes GPS and compass sensors to determine its direction and navigate efficiently. This project comprises several prototypes and scripts developed using Arduino and Python.

## Contents

1. **Arduino Sketches**
    - `1stPrototype2-30-11-23.ino`
    - `1stPrototype29-11-23.ino`
    - `gps05-10-23/gps05-10-23.ino`

2. **Python Scripts**
    - `approx_coords_rasp.py`

3. **Documentation**
    - `README.md`

## File Descriptions

### Arduino Sketches

1. **`1stPrototype2-30-11-23.ino`**
    - Description: This sketch contains the code for the second prototype of the weiderobot developed on November 30, 2023. It includes the basic functionalities required for navigation and obstacle avoidance.

2. **`1stPrototype29-11-23.ino`**
    - Description: This sketch contains the code for the first prototype of the weiderobot developed on November 29, 2023. It serves as the initial implementation of the robot's core features.

3. **`gps05-10-23/gps05-10-23.ino`**
    - Description: This sketch handles GPS functionalities for the weiderobot. It includes code for retrieving and processing GPS data to aid in navigation.

### Python Scripts

1. **`approx_coords_rasp.py`**
    - Description: This Python script calculates approximate coordinates, which can be used to enhance the navigation system of the weiderobot.

## Setup Instructions

### Arduino Sketches

1. **Prerequisites:**
    - Arduino IDE installed on your computer.
    - Required libraries: `Servo`, `Wire`, `Adafruit_Sensor`, `Adafruit_HMC5883_U`, `TinyGPS++`.

2. **Steps:**
    1. Open the Arduino IDE.
    2. Load the `.ino` files from this project.
    3. Verify and upload the sketches to the Arduino board connected to the weiderobot.

### Python Scripts

1. **Prerequisites:**
    - Python 3 installed on your computer or Raspberry Pi.
    - Required libraries: `math`, `geopy`.

2. **Steps:**
    1. Navigate to the directory containing the `approx_coords_rasp.py` file.
    2. Run the script using the command:
       ```bash
       python3 approx_coords_rasp.py
       ```

## Usage

1. **Arduino Sketches:**
    - After uploading the sketches to the Arduino board, ensure the GPS and compass modules are connected correctly.
    - Power the weiderobot and observe its navigation and obstacle avoidance capabilities.

2. **Python Script:**
    - The script `approx_coords_rasp.py` can be run to calculate and print approximate coordinates. This can be integrated with the robot's navigation system for improved accuracy.

## Contributions

Contributions to this project are welcome. Please fork the repository and submit a pull request with your enhancements.

## License

This project is licensed under the MIT License.
"""
