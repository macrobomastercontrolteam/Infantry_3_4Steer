# MacRM_Standard_2023
2023 McMaster Robomaster Standard **Infantry 3** Robot Code with Steering-wheel-chassis Mechanical Model

This is the code for Steering-Motor-Controller, which is supposed to receive CAN messages from [main control board](https://github.com/macrobomastercontrolteam/Infantry_1_4Mecanum) to control steering motors (GM6020) that are isolated from all other CAN devices in the robot.

Developed based on [GM6020 Demo Code by DJI](https://www.robomaster.com/en-US/products/components/general/gm6020)

# Mechanical Features
- Steering motor chassis: four GM6020 steering motors

# Firmware Environment
- InfantryMain controller: Type C Development Board
    - FreeRTOS
- InfantrySteer controller: Type A Development Board
    - Type C Board sends target encoder values (absolute angle) of steering motors to Type A Board with **CAN ID 0x112**
    - Type A Board controls steering motors with PID without responding back to Type C Board
