![image](https://raw.githubusercontent.com/robert-saramet/alset-v2/db09ef69e4e6436f0b647ece5fb87bbe2354e2f0/docs/images/1.jpg)

### Introduction
*Alset is a small-scale proof-of-concept autonomous car which can react to traffic signs and navigate via GPS, follow road lanes, and pathplan around obstacles. Moreover, Alset can be used as a fully universal kit to make any and all RC cars intelligent. It is highly modular and very safe.*

### Features
- [x] Traffic sign detection
- [x] Road/lane following
- [x] Extensive communications
- [x] Redundant safety features
- [x] PS4 controller support
- [x] GPS navigation webapp
- [x] Android app
- [x] Pathfinding
- [x] Modularity
- [x] LCD display

### To-Do
- [x] React to other traffic signs
- [x] Better power management
- [x] Hardware revision
- [ ] Crossroad detection & steering
- [ ] Max speed slider (WIP)
- [ ] Speed control (WIP)
- [ ] GPS TTS directions

---

### Controller
Alset can be operated using a PS4 controller connected via bluetooth. In this scope, the MAC address of the console bound to the controller must be obtained using [this tool](https://github.com/user-none/sixaxispairer) and assigned to the ESP32. In order to drive forward, hold the right trigger. To brake, hold both the right and left trigger. To reverse, first brake, then hold the left trigger. The acceleration is proportional to the strength applied to the trigger. There is a turbo mode available by holding the triangle button, which increases the minimum and maximum speed. Moreover, by pressing the cross button, the user can switch between fully manual and assisted mode. In fully manual mode, the user controls both the motor throttle and the servo direction. In assisted mode, the pathfinding algorithm controls the direction and the user remains in control of throttle. This is done as an additional safety feature. Regardless of mode, feedback on distance to obstacles is provided to the user in two ways: first, the RGB leds on the controller fade from green to red depending on distance to nearest object; second, the dual vibration motors of the controller vibrate proportional to the distance to the nearest object on each side (front left and front right).


### Safety
Three individual switches for motors, arduino circuit and raspberry pi + router allow for easy testing without any risks, as well as disabling features not currently desired (e.g. disabling raspberry pi when not using opencv). If any device loses power while driving or if signal is lost or too outdated, the car stops immediately.

### Traffic Sign Detection
The Haar cascade model is used with opencv, thus traffic signs can easily be implemented with the scripts in the "tools" folder.
The pipeline consists of creating a reasonable number of positive samples (500+ images that contain the sign) and at least half the number of negative images (images that do not contain the sign). The dataset we used can be found [here](https://www.mapillary.com/dataset/trafficsign) and consists of ~40000 images in total, all labeled in JSON files.

Next, a pos.txt file must be created containing all the positive image filenames (this can be achieved via the tools/parse_json.py script). It will be used for creating the .vec file. To do that, you will also need the opencv toolkit. On UNIX systems, the package manager can install everything for you, but on Windows you have to download the [3.4.x version](https://sourceforge.net/projects/opencvlibrary/files/opencv-win/), not the latest one, since future versions no longer contain the cascade training toolkit. After downloading the opencv tools, you are ready to start. To generate the .vec file mentioned earlier, you need to use the `openv_createsamples`{:.shell} program.
For example: `openv_createsamples -info pos.txt -w 24 -h 24 -num 1000 -vec pos.vec`{:.shell}

With the .vec file you've just created and a neg.txt file containing all the negative images filenames, you can use the `opencv_traincascade`{:.shell} program:

```shell
opencv_traincascade -data YourCascadeFolder/ -vec pos.vec -bg neg.txt -w 24 -h 24 -numPos YourNumOfPosImg, -numNeg YourNumOfNegImg
```
    
Complete documention on these commands can be found on the [opencv website]( https://docs.opencv.org/3.4/dc/d88/tutorial_traincascade.html).

The final cascade.xml file can be found in `YourCascadeFolder`{:.shell}, as well as the stages (stage0.xml, stage1.xml, stage2.xml etc), which you won't need at the moment. They are mainly used for downgrading your cascade or for saving progress when the training stops unexpectedly.
Alternatively, you can use the unofficial [GUI version](https://amin-ahmadi.com/cascade-trainer-gui/).

The HAAR cascades are loaded at runtime by the raspberry pi, which uses opencv to recognize the signs captured by the camera. The generated output(position, distance etc) is then processed.

### Lane Following
This works only on marked roads. It detects centre lines using Canny edge detection. After processing, a geometric line is generated for determining the car's position relative to the road. 

### GPS
- ##### **Arduino Side**
For GPS navigation, a U-Blox Neo-6M module is connected to the Pololu 328PB, which extracts latitude, longitude, speed and direction information from NMEA sentences using the TinyGPS++ library. Destination coordinates will be sent by the ESP32 from the Raspberry Pi webapp. Once the waypoint is selected, navigation data/steering information will be obtained through the GPRMB NMEA sentence. For route planning, the starting location is provided by the phone. 
- ##### **Raspberry Pi Side**
The communication with the raspberry pi is done through a Flask server, which can be accessed while in range of the car by connecting to its wi-fi router. The webapp allows the user to enter the destination address and, should it exist, a route will be selected by the raspberry pi, and longitude & latitude are passed on to the arduino. The backend for this is implemented using HERE Maps REST API.

### Phone app
A phone app made in the Blynk platform is available, providing a joystick, kill switch, and speed slider, along with some features not currently implemented on the arduino side. While both wifi and bluetooth are available, we have found bluetooth to be more reliable, and it also has the added advantage of not requiring an internet connection on the ESP32. While this app remains available for use when lacking a PS4 controller, we have currently abandoned it due to it's unstable connectivity and closed-source nature.

### Security
The security of Alset is as efficient as it is simple. There are three ways of interacting with Alset: the PS4 controller, the phone app, and the wi-fi network. Both the phone app and the PS4 controller use bluetooth and require initial pairing, which implies close proximity to the device and knowledge of the pairing PIN. Aditionally, the device will only connect to the MAC address of the controller, or, when using the phone app, to the unique API key assigned to the phone. Moreover, the raspberry pi is connected to a portable wifi router placed on the car, which is protected with a strong password. These features make it hard for any attackers to cause damage to/with Alset.

### Communications
Every board is connected to the ESP32 via UART (since ESP32 does not properly support I2C slave mode), with the exception of the 328PB, which connects to the 32u4 via I2C and is forwarded to the ESP32 via UART (due to the lack of serial ports). The ESP32 acts as the main hub and logic controller, receiving any data from modules not directly connected to it through the SerialTransfer library. This allows us to update the code in only one place, while still relying on the extra boards, so as not to overload the ESP32, both in terms of processing power and GPIO/UART interfaces.

### Mechanics
As you might know, Alset v1 used differential steering and was of much smaller scale. However, since the purpose of this project is to serve as proof-of-concept, we decided to create v2 to be as close as possible to a real car. Using a single motor and Ackermann steering, while also being much larger (1/10 scale), Alset v2 easily reaches that goal.

### Modularity
Another enhancement from v1 is the high modularity, enabling simple addition, removal and replacement of parts and providing easy access to everything. The base is attached to the body using the standard rc car system of shell clips, allowing easy *universal* mounting and unmounting within a blink. Furthermore, *all* that is required for Alset to interface with *any* electric RC car is the connection of *just two cables* from the car radio receiver to their socket on the Alset board: the one for the ESC and the one for the servo. Both are then driven by Alset using PWM and PPM (pulse-position modulation). Electronics aside, everything is mounted with screws, the batteries are connected with screw terminals and the weight distribution is optimised for the perfect driving experience.

### Parts
- [Raspberry Pi 3 B](https://www.raspberrypi.org/products/raspberry-pi-3-model-b/) (openCV, server)
- [Sparkfun ESP32 Thing](https://www.sparkfun.com/products/13907) (main processor, comms, IMU)
- [Pololu A-Star Micro 32u4](https://www.pololu.com/product/3101) (motors, ultrasonic & speed sensors)
- [Pololu A-Star Micro 328PB](https://www.pololu.com/category/239/a-star-328pb-micro) (GPS, LCD, ultrasonic & voltage/current sensors)
- [Maverick Quantum MT](https://hpi-racing.ro/automodele-rc/7332-automodel-maverick-quantum-mt-110-brushed-albastru-rtr-rc.html) (RC car)
- [TP-Link Portable 4G Router](https://www.emag.ro/router-wireless-n300-tp-link-3g-4g-portabil-tl-mr3020/pd/EC23TBBBM/) (optional)
- [PS4 DualShock Controller](https://www.playstation.com/en-ro/accessories/dualshock-4-wireless-controller/) (primary input)
- [6x HC-SR04 Ultrasound Sensors](https://cleste.ro/senzor-ultrasonic-hc-sr04.html)
- [U-blox NEO-6M GPS](https://www.u-blox.com/en/product/neo-6-series)
- [Adafruit NXP 9DOF IMU](https://www.adafruit.com/product/3463)
- [2004 I2C LCD](https://cleste.ro/ecran-oled-0-91.html)
- [Pololu 3.3V/5V Step-Up Regulator](https://www.pololu.com/product/2872/specs)
- [TP4056 Battery Charger](https://cleste.ro/modul-incarcare-baterii-litiu-1a-usb-c-tp4056.html)
- [2x Switches](https://cleste.ro/buton-panou.html)
- [2x I2C Level Shifters](https://cleste.ro/modul-ic-i2c-nivel-conversie.html)
- [18650 2P Battery Holder](https://www.optimusdigital.ro/ro/suporturi-de-baterii/12108-suport-de-baterii-2-x-18650-conectare-in-paralel.html)
- [Passive Buzzer](https://cleste.ro/modul-buzzer-pasiv.html)
- Some things that are not currently implemented, such as:
LEDs, level shifters, 8p DIP switch, LED bar graph

---

### Third Party

- #### [Mapillary Traffic Sign Dataset](https://www.mapillary.com/dataset/trafficsign)
- #### [HERE Maps](https://developer.here.com/develop/rest-apis)
- #### [OpenCV](https://opencv.org/)
- #### [Cascade Trainer GUI](https://amin-ahmadi.com/cascade-trainer-gui/)
- #### [PySerial Library](https://github.com/pyserial/pyserial)
- #### [PS4 Controller Library](https://github.com/aed3/PS4-esp32)
- #### [TinyGPS++](https://github.com/mikalhart/TinyGPSPlus)


---

### Roles
- #### Robert Saramet
  - PS4 Controller
  - Pathfinding
  - Communcations
  - Bluetooth app
  - Hardware & electronics
  - Documentation
  - All arduino code
- #### Bogdan Maciuca
  - Road/lane following
  - Image recognition
  - GPS navigation webapp
  - Flask & Node servers
  - Documentation
  - All raspberry pi code

---

### Support us
- ##### BTC: bc1q9zjrnzd04w27sx4dh0hy9n06hu624dmvjc495wc
[![219971756-519102119360658-4905840849781814643-n.png](https://i.postimg.cc/nLCd65GD/219971756-519102119360658-4905840849781814643-n.png)](https://postimg.cc/JyCcXpgr)
