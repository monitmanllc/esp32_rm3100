RM3100 Breakout          ESP32-C6
═════════════════════════════════════
Pin 1  (SCL)      →      GPIO6 (SCL) or your custom I2C pin
Pin 3  (SDA)      →      GPIO7 (SDA) or your custom I2C pin
Pin 10 (I2CEN)    →      3.3V (to enable I2C mode)
Pin 12 (DVDD)     →      3.3V
Pin 13 (AVDD)     →      3.3V
Pin 14 (DVSS)     →      GND
Pin 7  (AVSS)     →      GND

Optional:
Pin 5  (DRDY)     →      Any GPIO (for data ready interrupt)
Pin 4  (SA0)      →      GND or 3.3V (I2C address bit 0)
Pin 2  (SA1)      →      GND or 3.3V (I2C address bit 1)
