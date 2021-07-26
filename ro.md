
![image](https://raw.githubusercontent.com/robert-saramet/alset-v2/db09ef69e4e6436f0b647ece5fb87bbe2354e2f0/docs/images/1.jpg)

### Rezumat
*Alset este o masina autonoma proof-of-concept la scara mica care poate reactiona la semne de circulatie, naviga prin GPS, urmari strazi/benzi si ocoli obstacole. In plus, Alset poate fi utilizat ca un kit complet universal pentru a face orice masina cu radiocomanda inteligenta. Este modular si ofera multa siguranta.*

### Functii
- [x] Identificarea semnelor de circulatie
- [x] Urmarirea strazii/benzii
- [x] Webapp pentru navigare GPS
- [x] Aplicatie pentru telefon
- [x] Operare prin controller de PS4
- [x] Functii de siguranta redundante
- [x] Evitarea obstacolelor
- [x] Modularitate
- [x] Debugging usor
- [x] Versatilitate

### To-Do
- [x] Implementarea mai multor semne
- [x] Gestionare mai buna a puterii
- [x] Revizie hardware
- [ ] Detectarea intersectiilor
- [ ] Slider de viteza (in lucru)
- [ ] Viteza constanta automata (in lucru)
- [ ] Directii prin TTS pentru GPS

---

### Controller
Alser poate fi condus cu ajutorul unui controller de PS4 conectat prin bluetooth. In acest scop, adresa MAC a consolei aferente controller-ului trebuie obtinuta cu ajutorul [acestei unelte](https://github.com/user-none/sixaxispairer) si atribuit pe ESP32. Pentru a inainta apasati trigger-ul drept. Pentru a frana, apasati atat trigger-ul drept cat si cel stang. Pentru a merge cu spatele, apasati trigger-ul stang. VIteza este proportionala cu forta aplicata pe trigger. Un mod turbo este disponibil prin apasarea butonului triunghi, crescand viteza minima si maxima. In plus, prin apasarea butonului cruce/x, utilizatorul poate schimba intre modul asistat si modul complet manual. In modul manual, utilizatorul controleaza atat acceleratia la motor cat si directia la servo. In modul asistat, utilizatorul ramane in controlul acceleratiei (pentru siguranta), dar directia este controlata de algoritmul de pathfinding cu senzori ultrasonici. Indiferent de mod, feedback cu privire la distanta pana la obstacole este oferit utilizatorului in doua moduri: in primul rand, led-urile RGB de pe controller isi schimba gradat culoarea de la verde la rosu in functie de distanta pana la cel mai apropiat obnstacol; in al doilea rand, motoarele de vibratie duale ale controller-ului vibreaza proportional cu distanta pana la cel mai apropiat obiect din fiecare directie (fata-stanga si fata-dreapta).

### Safety
Trei intrerupatoare individuale pentru motoare, circuitul cu arduino si raspberry pi + router permit testarea usoara si fara risc, oferind si posibilitatea dezactivarii functiilor momentan nedorite (ex. dezactivarea raspberry pi-ului cand nu se foloseste opencv). Daca orice dispozitiv se opreste in timpul operarii sau daca semnalul este pierdut sau prea invechit, masina opreste instant.

### Traffic Sign Detection
The Haar cascade model is used with opencv, thus traffic signs can easily be implemented with the scripts in the "tools" folder.
The pipeline consists of creating a reasonable number of positive samples (500+ images that contain the sign) and at least half the number of negative images (images that do not contain the sign). The dataset we used can be found [here](https://www.mapillary.com/dataset/trafficsign) and consists of ~40000 images in total, all labeled in JSON files.

Next, a pos.txt file must be created containing all the positive image filenames (this can be achieved via the tools/parse_json.py script). It will be used for creating the .vec file. To do that, you will also need the opencv toolkit. On UNIX systems, the package manager can install everything for you, but on Windows you have to download the [3.4.x version](https://sourceforge.net/projects/opencvlibrary/files/opencv-win/), not the latest one, since future versions no longer contain the cascade training toolkit. After downloading the opencv tools, you are ready to start. To generate the .vec file mentioned earlier, you need to use the `openv_createsamples` program.
For example:

```
openv_createsamples -info pos.txt -w 24 -h 24 -num 1000 -vec pos.vec
```

With the .vec file you've just created and a neg.txt file containing all the negative images filenames, you can use the `opencv_traincascade` program:

```shell
opencv_traincascade -data YourCascadeFolder/ -vec pos.vec -bg neg.txt -w 24 -h 24 -numPos YourNumOfPosImg, -numNeg YourNumOfNegImg
```
    
Complete documention on these commands can be found on the [opencv website]( https://docs.opencv.org/3.4/dc/d88/tutorial_traincascade.html).

The final cascade.xml file can be found in `YourCascadeFolder`, as well as the stages (stage0.xml, stage1.xml, stage2.xml etc), which you won't need at the moment. They are mainly used for downgrading your cascade or for saving progress when the training stops unexpectedly.
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

### Speed control
Speed control is a work in progress. Currently, wheel speed can be measured using an optical speed sensor connected to an interrupt-capable pin. The speed control algorithm itself remains to be implemented soon. A slider for setting min and max speed range will also be enabled.

### Powering
There are three power sources used on Alset, as follows: RC car is powered by its built-in NiMH battery pack, arduino circuit is powered by two 18650 cells in parallel, and a powerbank provides for the raspberry pi and router. The reasons for this seemingly complicated scheme are:
- since RC Cars require special battery packs with very high discharge rates and almost no safety features, they are a bad choice for powering anything other than the motors.
- the arduino circuits cannot procure enough power from usb, plus so many cables would be quite messy
- for a raspberry pi and router, a commercial powerbank makes much more sense than a custom circuit
- furthermore, the aforementioned two devices might be considered irrelevant by other users, and may be skipped altogether

In order to charge the batteries of the arduino circuit, just connect a regular usb charger to the module on the board. Do the same for the powerbank. For charging the battery of the RC car, since there are countless charger, battery, and connector types, and since Alset tries to be as cross-platform as possible, you should use the OEM hardware.

### Debugging
Debugging Alset v2 is made easy by the board design that allows easy testing of electrical connections and quick removal of components, as well as the debug flag available to turn on usb communications for inspecting every value. The LCD screen also displays many of these values, offering helpful insight without the need for a PC. Status LEDS will soon be enabled to further assist in this scope. In addition, all usb ports are placed on the same outer side of the robot to facilitate debugging.

---

### Components
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
- Some things that are not currently implemented, such as:
LEDs, level shifters, 8p DIP switch, LED bar graph, buzzer

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
### License
Copyright 2021 Robert Saramet, Bogdan Maciuca

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

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
- ##### BTC: bc1q9zjrnzd04w27sx4d0hy9n06hu624dmvjc495wc
[![219971756-519102119360658-4905840849781814643-n.png](https://i.postimg.cc/nLCd65GD/219971756-519102119360658-4905840849781814643-n.png)](https://postimg.cc/JyCcXpgr)
