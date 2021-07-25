![image](https://raw.githubusercontent.com/robert-saramet/alset-v2/db09ef69e4e6436f0b647ece5fb87bbe2354e2f0/docs/images/1.jpg)

### Features
- [x] Traffic sign detection
- [x] Road/lane following
- [x] Extensive communications
- [x] Redundant safety features
- [x] PS4 controller support
- [x] GPS navigation webapp
- [x] Android app
- [x] Pathfinding

### To-Do
- [x] React to other traffic signs
- [x] Better power management
- [x] Hardware revision
- [ ] Crossroad detection & steering
- [ ] Max speed slider (WIP)
- [ ] Speed control (WIP)
- [ ] GPS TTS directions

---

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
############

### Safety
Three individual switches for motors, arduino circuit and raspberry pi + router allow for easy testing without any risks, as well as disabling features not currently desired (e.g. disabling raspberry pi when not using opencv). If any device loses power while driving or if signal is lost, the car stops immediately.

### Controller
Alset can be operated using a PS4 controller connected via bluetooth. In this scope, the MAC address of the console bound to the controller must be obtained using [this tool](https://github.com/user-none/sixaxispairer) and assigned to the ESP32. In order to drive forward, hold the right trigger. To brake, hold both the right and left trigger. To reverse, first brake, then hold the left trigger. The acceleration is proportional to the strength applied to the trigger. There is a turbo mode available by holding the triangle button, which increases the minimum and maximum speed. Moreover, by pressing the cross button, the user can switch between fully manual and assisted mode. In fully manual mode, the user controls both the motor throttle and the servo direction. In assisted mode, the pathfinding algorithm controls the direction and the user remains in control of throttle. This is done as an additional safety feature. Regardless of mode, feedback on distance to obstacles is provided to the user in two ways: first, the RGB leds on the controller fade from green to red depending on distance to nearest object; second, the dual vibration motors of the controller vibrate proportional to the distance to the nearest object on each side (front left and front right).

### GPS
For GPS navigation, a U-Blox Neo-6M module is connected to the Pololu 328PB, which extracts latitude, longitude, speed and direction information from NMEA sentences. Destination coordinates will be sent by the ESP32 from the Raspberry Pi webapp. Once the waypoint is selected, navigation data/steering information will be obtained through the GPRMB NMEA sentence. For route planning, the starting location is provided by the phone. 

### Server
The communication with the server is done via a Flask server, which can be accessed while in close proximity to the car by connecting to its wi-fi router. The webapp allows the user to enter the destination address and if it exists, a route will be chosen by the raspberry pi, and longitude & latitude are passed on to the arduino.

---

### Third Party

- #### [Mapillary Traffic Sign Dataset](https://www.mapillary.com/dataset/trafficsign)
- #### [OpenCV](https://opencv.org/)
- #### [Cascade Trainer GUI](https://amin-ahmadi.com/cascade-trainer-gui/)
- #### [PySerial Library](https://github.com/pyserial/pyserial)
- #### [PS4 Controller Library](https://github.com/aed3/PS4-esp32)
- #### [TinyGPS++](https://github.com/mikalhart/TinyGPSPlus)

### Parts

- #### Robot
  - [Raspberry Pi 3 B](https://cleste.ro/raspberry-pi-3-model-b.html)
  - [Arduino Mega 2560](https://cleste.ro/placa-de-dezvoltare-compatibila-cu-arduino-mega-2560.html)
  - [Wemos D1 Mini ESP8266](https://cleste.ro/placa-dezvoltare-esp12-mini-v2.html)
  - [Chasis & Motors](https://www.optimusdigital.ro/ro/robotica-kit-uri-de-roboti/11883-kit-robot-cu-4-motoare-negru.html)
  - [L298N Motor Driver](https://cleste.ro/modul-l298n-cu-punte-h-dubla.html)
  - [U-blox NEO-6M GPS](https://cleste.ro/modul-gps-ublox-neo-6m-cu-antena.html)
  - [RC522 RFID Reader](https://cleste.ro/modul-rfid-cu-card-si-tag.html)
  - [Cytron Maker Line Sensor](https://www.optimusdigital.ro/ro/altele/12072-senzori-de-linie-maker-line-pentru-incepatori.html?search_query=maker+line&results=2)
  - [2x HC-SR04 Ultrasound Sensors](https://cleste.ro/senzor-ultrasonic-hc-sr04.html)
  - [Collision Switch](https://cleste.ro/modul-impact.html)
  - [TP4056 Battery Charger](https://cleste.ro/modul-incarcare-baterii-litiu-1a-usb-c-tp4056.html)
  - [2x Panel Switch](https://cleste.ro/buton-panou.html)
  - [18650 Battery Holder](https://www.emag.ro/suport-acumulator-3-7v-18650-x-4-sloturi-s18650-4/pd/D8C49WBBM/)
- #### Joystick
  - [Lolin32 ESP32](https://cleste.ro/placa-dezvoltare-nodemcu-wifi-bluetooth-esp32.html)
  - [2x PS2 Joystick](https://cleste.ro/modul-joystick-ps2-compatibil-arduino.html)
  - [Passive Buzzer](https://cleste.ro/modul-buzzer-pasiv.html)
  - [SSD 1306 OLED](https://cleste.ro/ecran-oled-0-91.html)

---

### Roles
- #### Robert
  - Obstacle detection
  - Pathfinding
  - Communcations
  - Arduino code
  - Hardware & electronics
- #### Bogdan
  - Machine learning
  - Image recognition
  - Server
  - Raspberry Pi code
  - GPS webapp
