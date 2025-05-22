
This program is a brige between the sx1276 lora module and the Mosquitto broker.

The program, spitest.cpp has prereq:
wiringpi
paho-mqtt

The Lora module is wired up using CE1 as so:

These are the RPi Zero W pin header#s.

9 - GND
              IRQ(GPIO4) - 16
17 - 3.3v     RST(GPIO5) - 18
19 - MOSI
21 - MISO
23 - SCLK
              CE1        - 26

This is compiled on a RPI Zero W as so:

g++ spitest.cpp -o spitest -lwiringPi -lpthread -lpaho-mqtt3c -g -DUSERNAME=\"YOUR_USERNAME\" -DPASSWORD=\"Your_Mosquitto_PASSWORD\" -DADDRESS=\"tcp://YOUR_BROKER:1883\" -DCLIENTID=\"YOUR_CLIENTID\"
