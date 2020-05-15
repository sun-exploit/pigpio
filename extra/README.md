# zip files from http://abyz.me.uk/rpi/pigpio/examples.html

   pigpio library   

[![](images/pigpio-logo.gif)](index.html)pigpio library

![](images/rotary.jpg)

![](images/rf-rx.jpg)

![](images/pins.jpg)

[pigpio](index.html) [pigpio C I/F](cif.html) [pigpiod](pigpiod.html) [pigpiod C I/F](pdif2.html) [Python](python.html) [pigs](pigs.html) [piscope](piscope.html) [Misc](misc.html) [Examples](examples.html) [Download](download.html) [FAQ](faq.html) [Site Map](sitemap.html)

Examples
--------

The following examples show various ways pigpio may be used to communicate with sensors via the GPIO.  
  
Although many are complete programs they are intended to be a starting point in producing your own code, not an end point.  
  
[Index](#Index)  
  
[Hardware](#Hardware)  
  
[Shell code](#Shell code)  
  
[C code](#C code)  
  
[C++ code](#C++ code)  
  
[pigpiod\_if2 code](#pigpiod_if2 code)  
  
[Python code](#Python code)  
  
[Miscellaneous related code](#Miscellaneous related code)  
  
[External links](#External links)  
  

### Hardware

A few practical examples of using pigpio with hardware.  
  

[IR Receiver](ex_ir_remote.html)  
2013-06-09

Reading an infrared remote receiver.  
  

[Light Dependent Resistor](ex_LDR.html)  
2013-06-09

Measuring brightness with a light dependent resistor (LDR). Improved methods of timing the start of the capacitor recharge are given for [C](#C_pot_cap_charge_c) and [Python](#Python_pot_cap_py).  
  

[Motor Shield](ex_motor_shield.html)  
2013-12-15

Using an Arduino motor shield.  
  

[Rotary Encoder](ex_rotary_encoder.html)  
2013-06-09

Reading a rotary encoder.  
  

[Sonar Ranger](ex_sonar_ranger.html)  
2013-06-10

Measuring range with a sonar ranger.  
  

### Shell code

Examples of using pigpio with shell code.  
  

[GPIO test](code/gpiotest.zip)  
2014-08-11

This bash script tests the user GPIO. [Video](http://youtu.be/sCJFLKWaxHo)  
  

### C code

Examples of C pigpio programs.  
  
If your program is called foobar.c then build with  
  
gcc -Wall -pthread -o foobar foobar.c -lpigpio -lrt  
  

[Frequency Counter 1](code/freq_count_1.zip)  
2014-08-20

A program showing how to use the [gpioSetAlertFunc](cif.html#gpioSetAlertFunc) function to set a callback for GPIO state changes. A frequency count is generated for each monitored GPIO (frequencies up to 500kHz with a sample rate of 1μs).  
  

[Frequency Counter 2](code/freq_count_2.zip)  
2014-08-20

A program showing how to use the [gpioSetGetSamplesFunc](cif.html#gpioSetGetSamplesFunc) function to set a callback for accumulated GPIO state changes over the last millisecond. A frequency count is generated for each monitored GPIO (frequencies up to 500kHz with a sample rate of 1μs). Generally the method used is more complicated but more efficient than frequency counter 1.  
  

[Hall Effect Sensor](code/hall.zip)  
2014-06-13

Program to show status changes for a Hall effect sensor.  
  

[I2C Sniffer](code/I2C_sniffer.zip)  
2014-06-15

A program to passively sniff I2C transactions (100kHz bus maximum) and display the results. This C program uses pigpio notifications.  
  

[IR Receiver](code/ir_hasher_c.zip)  
2015-02-25

Function to hash a code from an IR receiver (reading an IR remote control).  
  

[PCF8591 YL-40](code/PCF8591.zip)  
2014-08-26

A program to display readings from the (I2C) PCF8591.  
  

[Pot + Capacitor Recharge Timing](code/pot_cap_charge_c.zip)  
2014-03-14

Function to time capacitor charging (through a resistance). The time can be used to estimate the resistance.  
  

[Rotary Encoder](code/rotary_encoder_c.zip)  
2015-10-03

Function to decode a mechanical rotary encoder.  
  

[SPI bit bang MCP3008](code/rawMCP3008_c.zip)  
2016-03-20

This program shows how to read multiple MCP3008 ADC simultaneously with accurately timed intervals. One 10-bit channel of each ADC may be sampled at up to 25k samples per second.  
  

[SPI bit bang MCP3202](code/rawMCP3202_c.zip)  
2016-03-20

This program shows how to read multiple MCP3202 ADC simultaneously with accurately timed intervals. One 12-bit channel of each ADC may be sampled at up to 25k samples per second.  
  

[SPI bit bang MCP3008 and MCP3202](code/rawMCP3XXX_c.zip)  
2016-03-20

This program shows how to read multiple MCP3008 and MCP3202 ADC simultaneously with accurately timed intervals. One channel of each ADC may be sampled at up to 25k samples per second. The 10-bit MCP3008 readings are multiplied by 4 so they have the same range (0-4095) as the 12-bit MCP3202.  
  

[Servo Pulse Generator](code/servo_demo.zip)  
2016-10-08

This program generates servo pulses on one or more GPIO. Each connected servo is swept between 1000µs and 2000µs at a different speed.  
  
sudo ./servo\_demo # Generate pulses on GPIO 4.  
  
sudo ./servo\_demo 5 9 20 # Generate pulses on GPIO 5, 9, and 20.  
  

[SPI pigpio driver speed test](code/spi-pigpio-speed.c)  
2016-11-06

This C code is used to benchmark the pigpio SPI driver on the Pi. The code executes a given number of loops at a given baud rate and bytes per transfer.  
  

[Wiegand Reader](code/wiegand_c.zip)  
2013-12-30

Function to read a Wiegand Reader.  
  

### C++ code

Examples of C++ pigpio programs.  
  
If your program is called foobar.cpp then build with  
  
g++ -Wall -pthread -o foobar foobar.cpp -lpigpio -lrt  
  

[IR Receiver](code/ir_hasher_cpp.zip)  
2015-02-22

Class to hash a code from an IR receiver (reading an IR remote control).  
  

[Rotary Encoder](code/rotary_encoder_cpp.zip)  
2013-12-30

Class to decode a mechanical rotary encoder.  
  

[Wiegand Reader](code/wiegand_cpp.zip)  
2013-12-30

Class to read a Wiegand Reader.  
  

### pigpiod\_if2 code

The pigpiod\_if2 code examples are linked with libpigpiod\_if2 and are written in C.  
  
The pigpiod\_if2 library may be compiled and run on any Linux machine and allows control of the GPIO on one or more networked Pis.  
  
It should be possible to adapt the library to run on Macs and PCs.  
  
Each Pi needs the pigpio daemon to be running. The pigpio daemon may be started with the command sudo pigpiod.  
  

[433MHz Keyfob RX/TX](code/_433D.zip)  
2015-11-17

Code to read and transmit 313 and 434 MHz key fob codes. The codes to be read must use Manchester encoding. The transmitted codes use Manchester encoding.  
  
./\_433D -r10 # Print fob keycodes received on GPIO 10.  
  
./\_433D -t5 8246184 # Transmit code on GPIO 5.  
  
./\_433D -r10 -t5 8246184 # Transmit code on GPIO 5 then listen for codes  
  
./\_433D -? for options.  
  

[DHT11/21/22/33/44 Sensor](code/DHTXXD.zip)  
2016-02-16

Code to read the DHT temperature and humidity sensors. The sensor may be auto detected. A DHT11 sensor may be read once per second. The other sensors should not be read more often than once every three seconds.  
  
The code auto detects the DHT model and generally only the GPIO needs to be specified.  
  
./DHTXXD -g17 # Read a DHT connected to GPIO 17.  
  
./DHTXXD -g5 -i3 # Read a DHT connected to GPIO 5 every three seconds.  
  
./DHTXXD -? # for options.  
  

[Rotary Encoder](code/RED.zip)  
2015-11-18

Code to monitor a rotary encoder and show the position changes. By default the detent changes are shown. There is an option to show the four steps per detent instead.  
  
./RED -a7 -b8 -s30 # Show encoder on 7/8 detent changes for 30 seconds.  
  
./RED -a5 -b6 -m1 # Show encoder on 5/6 step changes forever.  
  
./RED -? # for options.  
  

[Servo Pulse Generator](code/servo_demo_D.zip)  
2016-10-08

This program generates servo pulses on one or more GPIO. Each connected servo is swept between 1000µs and 2000µs at a different speed.  
  
./servo\_demo\_D # Generate pulses on GPIO 4.  
  
./servo\_demo\_D 5 9 20 # Generate pulses on GPIO 5, 9, and 20.  
  

[Sonar Ranger](code/SRTED.zip)  
2015-11-16

Code to read the SRF-04 and SRF-05 type of sonar rangers which use the trigger echo method of operation. A 10 μs trigger pulse initiates a series of high frequency sonar chirps. The echo line then goes high and stays high until an echo from an object is received. The echo high time is used to calculate the distance of the object.  
  
For a one-off reading only the trigger and echo GPIO need to be specified.  
  
./SRTED -t5 -e6 # Read a sonar ranger connected to GPIO 5/6.  
  
./SRTED -t11 -e5 -i0.1 # Read a sonar ranger connected to GPIO 11/5 every 0.1 seconds.  
  
./SRTED -? # for options.  
  

[Transmit Rotary Encoder Test Signals](code/tx_RED.zip)  
2015-11-25

Code to transmit quadrature signals to test rotary encoder software.  
  
tx\_RED -aGPIO -bGPIO \[options\]  
  
tx\_RED -? for options  
  
E.g.  
  
tx\_RED -a5 -b6 -s20 -r-100  
  

[Transmit Wiegand Test Signals](code/tx_WD.zip)  
2015-11-25

Code to transmit Wiegand codes to test Wiegand decoder software.  
  
tx\_WD -gGPIO -wGPIO \[options\] {code}+  
  
tx\_WD -? for options  
  
E.g.  
  
tx\_WD -g5 -w6 -s37 12345 67890 123 899999  
  

[Wiegand Reader](code/WD.zip)  
2015-11-25

Code to read a Wiegand Reader.  
  
./WD -g7 -w8 -s30 # Read Wiegand codes from GPIO 7/8 for 30 seconds.  
  
./WD -g5 -w6 # Read Wiegand codes from GPIO 5/6 forever.  
  
./WD -? # for options.  
  

### Python code

The Python code may be run on any Python machine and allows control of the GPIO on one or more networked Pis.  
  
The Python machine need not be a Pi, it may run Windows, Mac, Linux, anything as long as it supports Python.  
  
Each Pi needs the pigpio daemon to be running. The pigpio daemon may be started with the command sudo pigpiod.  
  

[433MHz Keyfob RX/TX](code/_433_py.zip)  
2015-10-30

Classes to send and receive 433MHz wireless keyfob codes. These keyfobs are widely used for remote control of devices.  
  

[7-Segment LED Display Multiplexing](code/_7_segment.zip)  
2016-12-12

Script to multiplex several 7-segment LED displays. Each display has the segments a-g and the decimal point connected in parallel but has an individual enable GPIO (connected to the common anode or cathode).  
  

[APA102 LED strip driver](code/test-APA102_py.zip)  
2017-03-28

Script to drive an APA102 LED strip. Three different methods are demonstrated - using spidev SPI (only works on the local Pi), pigpio SPI, and pigpio waves. The SPI solutions only work with the dedicated SPI GPIO. Waves may use any spare GPIO. Four different examples are given including a LED strip clock.  
  

[BME280 Sensor](code/BME280_py.zip)  
2016-08-05

Class to read the relative humidity, temperature, and pressure from a BME280 sensor. The sensor has both an I2C and a SPI interface which are both  
supported by the class.  
  

[DHT11/21/22/33/44 Sensor](code/DHT.py)  
2019-11-07

Class to read the relative humidity and temperature from a DHT sensor. It can automatically recognize the sensor type.  
  
The default script prints the reading from the specified DHT every 2 seconds. E.g. ./DHT.py 22 27 displays the data for DHT connected to GPIO 22 and 27.  
  
The following data is printed for each DHT: timestamp, GPIO, status, temperature, and humidity.  
  
The timestamp is the number of seconds since the epoch (start of 1970).  
  
The status will be one of: 0 - a good reading, 1 - checksum failure, 2 - data had one or more invalid values, 3 - no response from sensor.  
  

[DHT22 AM2302 Sensor](code/DHT22_py.zip)  
2014-07-11

Class to read the relative humidity and temperature from a DHT22/AM2302 sensor.  
  

[DS18B20 Temperature Sensor](code/DS18B20-1_py.zip)  
2016-06-29

Script to read the temperature from any DS18B20 sensors connected to the 1-wire bus.  
  
To enable the 1-wire bus add the following line to /boot/config.txt and reboot.  
  
dtoverlay=w1-gpio  
  
By default you should connect the DS18B20 data line to GPIO 4 (pin 7).  
  
Connect 3V3 or 5V for power, ground to ground, 4k7 pull-up on data line to 3V3, and data line to GPIO 4.  
  
This script uses the file features of pigpio to access the remote file system.  
  
The following entry must be in /opt/pigpio/access.  
  
/sys/bus/w1/devices/28\*/w1\_slave r  
  

[Dust Sensor](code/PPD42NS_py.zip)  
2015-11-22

Class to read a Shinyei PPD42NS Dust Sensor, e.g. as used in the Grove dust sensor.  
  

[GPIO Status](code/gpio_status_py.zip)  
2014-06-12

Script to display the status of GPIO 0-31.  
  

[Hall Effect Sensor](code/hall.zip)  
2014-06-13

Program to show status changes for a Hall effect sensor.  
  

[HX711 24-bit ADC](code/HX711_py.zip)  
2018-03-05

Class to read the channels of a HX711 24-bit ADC.  
  

[I2C ADXL345 Accelerometer](code/i2c_ADXL345_py.zip)  
2015-04-01

Script to display the X, Y, and Z values read from an ADXL345 accelerometer.  
  

[I2C HMC5883L Magnetometer](code/i2c_HMC5883L_py.zip)  
2015-04-01

Script to display the X, Y, and Z values read from a HMC5883L Magnetometer (compass).  
  

[I2C ITG3205 Gyroscope](code/i2c_ITG3205_py.zip)  
2015-04-01

Script to display the X, Y, Z, and temperature values read from an ITG3205 gyroscope.  
  

[I2C LCD Display](code/i2c_lcd_py.zip)  
2016-04-20

Class to display text on a LCD character display. The class supports the PCF8574T 8-bit I2C port expander connected to a HD44780 based LCD display. These displays are commonly available in 16x2 and 20x4 character formats.  
  

[I2C slave device](code/bsc_arduino_py.zip)  
2016-10-31

This script demonstrates how to transfer messages from an Arduino acting as the I2C bus master to the Pi acting as an I2C slave device.  
  

[I2C Sniffer](code/I2C_sniffer.zip)  
2015-06-15

A program to passively sniff I2C transactions (100kHz bus maximum) and display the results.  
  

[I2C Sonar](code/i2c_sonar_py.zip)  
2016-03-24

A class to read up to 8 HC-SR04 sonar rangers connected to an MCP23017 port expander.  
  

[IR Receiver](code/ir_hasher_py.zip)  
2014-06-12

Class to hash a code from an IR receiver (reading an IR remote control).  
  

[IR Record and Playback](code/irrp_py.zip)  
2015-12-21

This script may be used to record and play back arbitrary IR codes.  
  
To record the GPIO connected to the IR receiver, a file for the recorded codes, and the codes to be recorded are given.  
  
E.g. ./irrp.py -r -g4 -fir-codes vol+ vol- 1 2 3 4 5 6 7 8 9 0  
  
To playback the GPIO connected to the IR transmitter, the file containing the recorded codes, and the codes to be played back are given.  
  
E.g. ./irrp.py -p -g18 -fir-codes 2 3 4  
  
./irrp.py -h # for options  
  

[Kivy GPIO control](code/kivy_GPIO_py.zip)  
2016-12-11

This example shows how to use Kivy to control a Pi's GPIO. The GPIO may be configured as inputs, outputs, or to generate Servo or PWM pulses. Kivy is an Open source Python library for rapid development of applications.  
  

[MAX6675 SPI Temperature Sensor](code/MAX6675_py.zip)  
2016-05-02

A script to read the temperature from a MAX6675 connected to a K-type thermocouple. The MAX6675 supports readings in the range 0 - 1023.75 C. Up to 4 readings may be made per second.  
  

[Monitor GPIO](code/monitor_py.zip)  
2016-09-17

Script to monitor GPIO for level changes. By default all GPIO are monitored. At a level change the GPIO, new level, and microseconds since the last change is printed.  
  

[Morse Code](code/morse_code_py.zip)  
2015-06-17

Script to transmit the morse code corresponding to a text string.  
  

[NRF24 radio transceiver](code/NRF24.py)  
2018-01-06

Script to transmit and receive messages using the nRF24L01 radio transceiver.  
  

[PCA9685 16 Channel PWM](code/PCA9685_py.zip)  
2016-01-31

Class to control the 16 PWM channels of the I2C PCA9685. All channels use the same frequency. The duty cycle or pulse width may be set independently for each channel.  
  

[PCF8591 YL-40](code/PCF8591.zip)  
2014-08-26

Script to display readings from the (I2C) PCF8591.  
  

[PPM (Pulse Position Modulation) generation](code/PPM.py)  
2016-02-19

Script to generate PPM signals on a chosen GPIO.  
  

[PPM (Pulse Position Modulation) to servo pulses](code/PPM_to_servo.py)  
2019-10-09

Script to read a PPM signal on a GPIO and generate the corresponding servo signals on chosen GPIO.  
  

[pigpio Benchmark](code/bench_1_py.zip)  
2014-06-12

Script to benchmark the pigpio Python module's performance.  
  

[pigpio CGI](code/pigpio_cgi_py.zip)  
2015-05-04

Script demonstrating how to access the pigpio daemon using CGI from a browser. Instructions on how to use with Apache2 on the Pi are given in the comments.  
  

[Playback piscope recordings](code/playback_py.zip)  
2016-12-23

Script to playback GPIO data recorded in piscope format.  
  
To playback GPIO 4 to GPIO 4 from file data.piscope  
./playback.py data.piscope 4  
  
To playback GPIO 4 to GPIO 7 from file rec.txt  
./playback.py rec.txt 7=4  
  

[Pot + Capacitor Recharge Timing](code/pot_cap_py.zip)  
2016-09-26

Class to time capacitor charging (through a resistance). The time can be used to estimate the resistance.  
  

[PWM Monitor](code/read_PWM_py.zip)  
2015-12-08

Class to monitor a PWM signal and calculate the frequency, pulse width, and duty cycle.  
  

[Rotary Encoder](code/rotary_encoder_py.zip)  
2014-06-12

Class to decode a mechanical rotary encoder.  
  

[RPM Monitor](code/read_RPM_py.zip)  
2016-01-20

Class to monitor speedometer pulses and calculate the RPM (Revolutions Per Minute).  
  

[Si7021 I2C Temperature and Humidity Sensor](code/Si7021_py.zip)  
2016-05-07

Class to read the temperature and relative humidity from a Si7021.  
  

[SPI Monitor](code/SPI_mon_py.zip)  
2016-09-21

A program to passively sniff SPI transactions and display the results. The SPI rate should be limited to about 70kbps if using the default pigpio 5µs sampling rate.  
  

[Servo Pulse Generator](code/servo_demo_py.zip)  
2016-10-07

This script generates servo pulses on one or more GPIO. Each connected servo is swept between 1000µs and 2000µs at a different speed.  
  
./servo\_demo.py # Generate pulses on GPIO 4.  
  
./servo\_demo.py 5 9 20 # Generate pulses on GPIO 5, 9, and 20.  
  

[Sonar Ranger](code/sonar_trigger_echo_py.zip)  
2014-06-12

Class to read sonar rangers with separate trigger and echo pins.  
  

[TCS3200 Colour Sensor](code/TCS3200_py.zip)  
2015-07-03

Class to read the TCS3200 colour sensor  
  

[Virtual Wire](code/vw.zip)  
2015-10-31

Class to send and receive radio messages compatible with the Virtual Wire library for Arduinos. This library is commonly used with 313MHz and 434MHz radio tranceivers.  
  

[Wave create](code/create_wave.py)  
2019-11-18

Script to generate waves from a template defined in a text file.  
  
You can also specify one of py, c, or pdif - the script output will then be a complete program to generate the wave (py for Python script, c for a C program, pdif for a C program using the pigpio daemon I/F).  
  
If none of py, c, or pdif are chosen the waveform will be generated for 30 seconds.  
  
Example text file  
  
\# GPIO levels  
23 11000001  
11 01110000  
12 00011100  
4 00000111  
  
To generate a pdif program with a bit time of 100 microseconds  
./create\_wave.py wave\_file 100 pdif >wave\_pdif.c  
  
To just transmit the wave with a bit time of 50 microseconds  
./create\_wave.py wave\_file 50  
  

[Wave PWM 1](code/wave_PWM_py.zip)  
2016-03-19

Script to show how waves may be used to generate PWM at (one) arbitrary frequency on multiple GPIO. For instance PWM at 10kHz may be generated with 100 steps between off and fully on.  
  

[Wave PWM 2](code/wavePWM_py.zip)  
2016-10-06

Class to generate PWM on multiple GPIO. It is more flexible than the Wave PWM 1 example in that the start of the pulse within each cycle may be specified as well as the duty cycle. The start and length of each pulse may be specified on a GPIO by GPIO basis in microseconds or as a fraction of the cycle time. The class includes a \_\_main\_\_ to demostrate its ability to send servo pulses.  
  

[Wiegand Reader](code/wiegand_py.zip)  
2014-06-12

Class to read a Wiegand reader.  
  

### Miscellaneous related code

The following code examples do not use pigpio.  
  

[ADXL345](code/adxl345_c.zip)  
2014-03-12

This C program reads x, y, and z accelerations from the ADXL345 via I2C address 0x53.  
  

[DS18B20 Temperature Sensor](code/DS18B20_py.zip)  
2016-04-25

This Python script reads the temperature from any DS18B20 sensors connected to the 1-wire bus.  
  
To enable the 1-wire bus add the following line to /boot/config.txt and reboot.  
  
dtoverlay=w1-gpio  
  
By default you should connect the DS18B20 data line to GPIO 4 (pin 7).  
  
Connect 3V3 or 5V for power, ground to ground, 4k7 pull-up on data line to 3V3, and data line to GPIO 4.  
  

[Easy as Pi Server](code/EasyAsPiServer.zip)  
2014-09-15

This Python class implements a simple server which allows broswer commands to be executed on the Pi.  
  

[Minimal Clock Access](code/minimal_clk.zip)  
2015-05-20

This C code sets GPIO 4 to a specified clock frequency. The frequency can be set between 4.6875 kHz and 500 MHz (untested). The clock can be preferentially set from one of the sources OSC (19.2MHz), HDMI (216MHz), PLLD (500MHz), or PLLC (1000MHz). MASH can be set between 0 and 3. MASH may not work properly for clock dividers less than 5.  
  

[Minimal GPIO Access](code/minimal_gpio.zip)  
2019-07-03

This C code has a minimal set of functions needed to control the GPIO and other Broadcom peripherals. The program requires root privileges to run. See Tiny GPIO access for an alternative which controls the GPIO (but not the other peripherals) and does not require root access.  
  
The code has been updated for the BCM2711 (Pi4B).  
  
The following functions are provided.  
  
gpioInitialise  
gpioSetMode  
gpioGetMode  
gpioSetPullUpDown  
gpioRead  
gpioWrite  
gpioTrigger  
gpioReadBank1  
gpioReadBank2  
gpioClearBank1  
gpioClearBank2  
gpioSetBank1  
gpioSetBank2  
gpioHardwareRevision  
gpioTick  
  

[Nanosecond Pulse Generation](code/nanopulse_c.zip)  
2014-01-29

This C program uses the PWM peripheral to generate precisely timed pulses of very short duration. Pulses as short as 4 nano seconds can be generated.  
  

[PCF8591 YL-40](code/PCF8591-x.zip)  
2014-08-26

C and Python code to read the (I2C) PCF8591.  
  

[SPI Linux driver speed test](code/spi-driver-speed.c)  
2016-11-06

This C code is used to benchmark the Linux SPI driver on the Pi. The code executes a given number of loops at a given baud rate and bytes per transfer.  
  

[Tiny GPIO Access](code/tiny_gpio.zip)  
2016-04-30

This C code has a minimal set of functions needed to control the GPIO without needing root privileges (it uses /dev/gpiomem to access the GPIO).  
  
You may need to change the permissions and ownership of /dev/gpiomem if they have not been correctly set up.  
  
sudo chown root:gpio /dev/gpiomem  
sudo chmod g+rw /dev/gpiomem  
  
The user (default pi) needs to be in the gpio group.  
  
sudo adduser pi gpio  
  
The following functions are provided.  
  
gpioInitialise  
gpioSetMode  
gpioGetMode  
gpioSetPullUpDown  
gpioRead  
gpioWrite  
gpioTrigger  
gpioReadBank1  
gpioReadBank2  
gpioClearBank1  
gpioClearBank2  
gpioSetBank1  
gpioSetBank2  
gpioHardwareRevision  
  

### External links

Related code.  
  

[NRF24](https://pypi.org/project/nrf24/)  
2020-04-20

Python Package Index (Pypi) NRF24 module.  
pip install nrf24  
  

[NRF24](https://github.com/bjarne-hansen/py-nrf24)  
2020-04-20

Code and example usage of the Pypi NRF24 module. Cleaned up and added support for reading from multiple pipes using open\_reading\_pipe(pipe, address) and open\_writing\_pipe(address) in order to be more "compatible" with the way NRF24 is used on Arduinos.  
  

[Stepper Motor](https://github.com/stripcode/pigpio-stepper-motor)  
2016-08-12

Stepper motor code.  
  

[Parallax ActivityBot 360](https://github.com/choeffer/360pibot)  
2018-11-03

Python 3 implementation for programming a Parallax ActivityBot 360 Robot Kit with a Raspberry Pi.  
  

### Index

433MHz Keyfob RX/TX

[pdif2](#pdif2__433D) [Python](#Python__433_py)

7-Segment LED Display Multiplexing

[Python](#Python__7_segment)

ADXL345

[Misc](#Misc_adxl345_c)

APA102 LED strip driver

[Python](#Python_test-APA102_py)

BME280 Sensor

[Python](#Python_BME280_py)

DHT11/21/22/33/44 Sensor

[pdif2](#pdif2_DHTXXD) [Python](#Python_code/DHT.py)

DHT22 AM2302 Sensor

[Python](#Python_DHT22_py)

DS18B20 Temperature Sensor

[Python](#Python_DS18B20-1_py) [Misc](#Misc_DS18B20_py)

Dust Sensor

[Python](#Python_PPD42NS_py)

Easy as Pi Server

[Misc](#Misc_EasyAsPiServer)

Frequency Counter 1

[C](#C_freq_count_1)

Frequency Counter 2

[C](#C_freq_count_2)

GPIO Status

[Python](#Python_gpio_status_py)

GPIO test

[Shell](#Shell_gpiotest)

Hall Effect Sensor

[C](#C_hall) [Python](#Python_hall)

HX711 24-bit ADC

[Python](#Python_HX711_py)

I2C ADXL345 Accelerometer

[Python](#Python_i2c_ADXL345_py)

I2C HMC5883L Magnetometer

[Python](#Python_i2c_HMC5883L_py)

I2C ITG3205 Gyroscope

[Python](#Python_i2c_ITG3205_py)

I2C LCD Display

[Python](#Python_i2c_lcd_py)

I2C slave device

[Python](#Python_bsc_arduino_py)

I2C Sniffer

[C](#C_I2C_sniffer) [Python](#Python_I2C_sniffer)

I2C Sonar

[Python](#Python_i2c_sonar_py)

IR Receiver

[Hardware](#Hardware_ex_ir_remote) [C](#C_ir_hasher_c) [C++](#C++_ir_hasher_cpp) [Python](#Python_ir_hasher_py)

IR Record and Playback

[Python](#Python_irrp_py)

Kivy GPIO control

[Python](#Python_kivy_GPIO_py)

Light Dependent Resistor

[Hardware](#Hardware_ex_LDR)

MAX6675 SPI Temperature Sensor

[Python](#Python_MAX6675_py)

Minimal Clock Access

[Misc](#Misc_minimal_clk)

Minimal GPIO Access

[Misc](#Misc_minimal_gpio)

Monitor GPIO

[Python](#Python_monitor_py)

Morse Code

[Python](#Python_morse_code_py)

Motor Shield

[Hardware](#Hardware_ex_motor_shield)

Nanosecond Pulse Generation

[Misc](#Misc_nanopulse_c)

NRF24

[External](#External_https://pypi.org/project/nrf24/) [External](#External_https://github.com/bjarne-hansen/py-nrf24)

NRF24 radio transceiver

[Python](#Python_code/NRF24.py)

Parallax ActivityBot 360

[External](#External_https://github.com/choeffer/360pibot)

PCA9685 16 Channel PWM

[Python](#Python_PCA9685_py)

PCF8591 YL-40

[C](#C_PCF8591) [Python](#Python_PCF8591) [Misc](#Misc_PCF8591-x)

pigpio Benchmark

[Python](#Python_bench_1_py)

pigpio CGI

[Python](#Python_pigpio_cgi_py)

Playback piscope recordings

[Python](#Python_playback_py)

Pot + Capacitor Recharge Timing

[C](#C_pot_cap_charge_c) [Python](#Python_pot_cap_py)

PPM (Pulse Position Modulation) generation

[Python](#Python_code/PPM.py)

PPM (Pulse Position Modulation) to servo pulses

[Python](#Python_code/PPM_to_servo.py)

PWM Monitor

[Python](#Python_read_PWM_py)

Rotary Encoder

[Hardware](#Hardware_ex_rotary_encoder) [C](#C_rotary_encoder_c) [C++](#C++_rotary_encoder_cpp) [pdif2](#pdif2_RED) [Python](#Python_rotary_encoder_py)

RPM Monitor

[Python](#Python_read_RPM_py)

Servo Pulse Generator

[C](#C_servo_demo) [pdif2](#pdif2_servo_demo_D) [Python](#Python_servo_demo_py)

Si7021 I2C Temperature and Humidity Sensor

[Python](#Python_Si7021_py)

Sonar Ranger

[Hardware](#Hardware_ex_sonar_ranger) [pdif2](#pdif2_SRTED) [Python](#Python_sonar_trigger_echo_py)

SPI bit bang MCP3008

[C](#C_rawMCP3008_c)

SPI bit bang MCP3008 and MCP3202

[C](#C_rawMCP3XXX_c)

SPI bit bang MCP3202

[C](#C_rawMCP3202_c)

SPI Linux driver speed test

[Misc](#Misc_code/spi-driver-speed.c)

SPI Monitor

[Python](#Python_SPI_mon_py)

SPI pigpio driver speed test

[C](#C_code/spi-pigpio-speed.c)

Stepper Motor

[External](#External_https://github.com/stripcode/pigpio-stepper-motor)

TCS3200 Colour Sensor

[Python](#Python_TCS3200_py)

Tiny GPIO Access

[Misc](#Misc_tiny_gpio)

Transmit Rotary Encoder Test Signals

[pdif2](#pdif2_tx_RED)

Transmit Wiegand Test Signals

[pdif2](#pdif2_tx_WD)

Virtual Wire

[Python](#Python_vw)

Wave create

[Python](#Python_code/create_wave.py)

Wave PWM 1

[Python](#Python_wave_PWM_py)

Wave PWM 2

[Python](#Python_wavePWM_py)

Wiegand Reader

[C](#C_wiegand_c) [C++](#C++_wiegand_cpp) [pdif2](#pdif2_WD) [Python](#Python_wiegand_py)

[\[pigpio\]](index.html) [\[pigpio C I/F\]](cif.html) [\[pigpiod\]](pigpiod.html) [\[pigpiod C I/F\]](pdif2.html) [\[Python\]](python.html) [\[pigs\]](pigs.html) [\[piscope\]](piscope.html) [\[Misc\]](misc.html) [\[Examples\]](examples.html) [\[Download\]](download.html) [\[FAQ\]](faq.html) [\[Site Map\]](sitemap.html)

© 2012-2020

e-mail: pigpio @ abyz.me.uk

Updated: 07/05/2020
