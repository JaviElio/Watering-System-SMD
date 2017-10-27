# Watering-System-SMD

Watering system with Attiny84 and i2c moisture sensor.

THINGSPEAK LINK:

https://thingspeak.com/channels/339098

FEATURES:

 - Measures soil moisture, light and temperature.
 - It waters only at night.
 - It doesn't water if temperature is below 4ºC.
 - It waters depending on a preset value and a hysteresis.

 - It takes measurements every hour, then microcontroller goes to deep sleep.
 - If it irrigates, then checks moisture again after five minutes.
 - Measurements are uploaded to thingspeak.com



BILL OF MATERIALS:

 - Valve: https://www.aliexpress.com/item/Free-shipping-Plastic-solenoid-valve-12VDC-for-drink-water-air-6-35mm-1-4-Quick-plug/1411672083.html?spm=2114.13010608.0.0.Ew7MCd
 
 - I2C Soil moisture sensor: https://www.tindie.com/products/miceuz/i2c-soil-moisture-sensor/?pt=full_prod_search
 
 - Enclosure: https://www.aliexpress.com/item/New-1-PC-Waterproof-115x90x55MM-Cover-Plastic-Electronic-Project-Box-Enclosure-Case-VE837-P0-3/32701644399.html?spm=2114.13010608.0.0.FBugqX
 
 - Attiny84
 - MCP1703
 - 2N7000
 - AO3400
 - ESP-01


**TO IMPROVE**
 - Change protection diode with a schottky diode
 - Add a resistor in serial with diode D2 to close the EV slower
 - Connect the electrovalve and the diode D2 after the diode protection diode.


**PICTURES:**

<img src="./Images/Picture1.JPG"  width=400>
<img src="./Images/Picture2.JPG"  width=400>

