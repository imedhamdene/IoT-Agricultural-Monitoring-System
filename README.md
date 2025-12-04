# IoT Agricultural Monitoring System

This is a complete, full-stack IoT solution for real-time agricultural monitoring, developed as part of my professional experience at the ESPRIT Tech Lab. The system consists of a dual-node network: an autonomous mobile rover and a stationary sensor station, both reporting to a custom-built backend.

## Key Features

*   **Autonomous Rover:** An ESP32-S3 based vehicle that navigates using 6 HC-SR04 ultrasonic sensors for collision avoidance. It controls DC motors via an L298N driver and a servo for camera positioning.
*   **Delegated Image Capture:** The rover offloads image capture tasks to a dedicated ESP32-CAM module. The main MCU triggers the camera via UART, and the ESP32-CAM captures and transmits the image over HTTP.
*   **Stationary Sensor Node:** An ESP32-based node that collects and transmits environmental data (Temperature, Humidity, Soil Moisture, Light) via the **MQTT** protocol.
*   **Full Backend & Dashboard:** A complete backend built with **Node-RED** that subscribes to the MQTT topics, providing a real-time web dashboard for data visualization and actuator control.
*   **Cloud Database:** All sensor data is logged for historical analysis in a cloud-hosted **MongoDB Atlas** database.

## Technologies Used

*   **Hardware:** ESP32-S3, ESP32, ESP32-CAM, HC-SR04 Sensors, DHT22, LDR, L298N Motor Driver.
*   **Firmware (C/C++):** Arduino IDE, ESP-IDF concepts.
*   **Communication Protocols:** MQTT, HTTP, UART.
*   **Backend & Database:** Node-RED, MongoDB Atlas.
