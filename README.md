# Energy_Counter
ESP32 software to read the electricity/gas and water consumption of my private house:

- electricity: Using the SML-protocol of the smart meter ISKRA MT681 to transfer data via ESP32 and MQTT to broker.  

- gas: Reading of the silver dot at postion number 6 of a ITRON gas meter with the help of an extended TCRT5000 light barrier

- water: Reading the rotating wheel of a standard water meter using an extended TCRT5000 light barrier

Conversion of a TCRT5000 with2 IR-Diodes to detect a complete cycle of the gas or water meter. This manipulated device with 2 IR-diodes recognizes the movement of the counter by the correct sequence of the positive and negative edges at the inputs of the ESP32:

![TCRT5000 doppelt](https://user-images.githubusercontent.com/76279852/213930673-a9e1f557-3765-40b4-a3f6-ac5b35fe5bd1.jpg)
