## Hardware used

1) 1x Arduino Mega

2) 2x CMPS11 Tilt-Compensated Compass Modules (https://www.robot-electronics.co.uk/htm/cmps11doc.htm)

3) 1x CAN-BUS Shield (http://www.watterott.com/de/Arduino-CANdiy-Shield)

4) 1x NMEA0183 Wind-Vane (http://www.lcjcapteurs.com/product/cv7-c-is-the-high-speed-ultrasonic-wind-sensor/?lang=en)

5) 1x SD-Card Breakout (http://www.watterott.com/de/mSD-Breakout)

6) 1x MAX3232 RS232 to TTL Converter

7) 12V->5V DC-DC adaptor


```
                            WINDVANE                                                    
                               |------------------------------12VDC---------------|
                               |                                                  |
                            MAX3232                                               |
                               |                                                  |   
                               |                                                  |
 CMPS11_1------------| ----------------------------|------------|                  |                                 
 on Mast             |                             |            |                  |                                      
                     |                             |            |--------------NMEA2000-Bus------------                             
                     |  ARDUINO                    | CAN-BUS    |                  |                       
                     |                             | SHIELD     |                  |                     
                     |                             |            |                  |                     
 CMPS11_2----------- |-----------------------------|------------|                  |                               
 in Hull                 |               |                                         |
                         |               |                                         |
                      SDCARD             ------12V5V DCDC -------------------------- 
```

## Dependencies

Install the great "NMEA2000 library for Arduino" with its dependencies as described here:
https://github.com/ttlappalainen/NMEA2000
