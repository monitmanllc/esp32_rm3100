ESP32-C6                    RM3100 (MagI2C)
---------                   ---------------
3.3V ----------------+----> AVDD (4, 14, 26)
                     |
                     +----> I2CEN (22)
                     
GND -----------------+----> AVSS (5), DVSS (19)
                     |
                     +----> SA0 (3), SA1 (28)
                     |
                     +----> RES (2, 20)
                     |
                     +--[33kΩ]-- REXT (25)

GPIO6 (SDA) --------------> SDA (1)
GPIO7 (SCL) --------------> SCL (27)
GPIO10 (optional) --------> DRDY (23)

                     Sensor Coils
                     ------------
                     Sen-XY-f --> X pins (15-18)
                     Sen-XY-f --> Y pins (10-13)
                     Sen-Z-f  --> Z pins (6-9)
