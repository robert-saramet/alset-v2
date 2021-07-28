## Main architecture

---
#### *Please note that some parts of this page are outdated.*
---

### 1. Boards
<h4>
     <ul>
         <li> <span class="esp32"> Sparkfun Thing ESP32 </span> (main MCU) </li>
         <li> Pololu A-Star Micro (motors, ultrasonic & speed sensors) </li>
         <li> Raspberry Pi 3 or 4 (openCV, Blynk server, flashing) </li>
    </ul>
</h4>

### 2. Sensors
- 4x ultrasonic sensors
    - front center
    - front left
    - front right
    - left center
    - right center
    - back center

### 3. Comms
- Raspberry Pi => HW UART => Thing
- A-Star <=> HW/SW UART <=> Thing
- Thing <=> BT <=> Phone (emulated serial)
- Thing <=> ESP-NOW <=> Joystick

### 4. Peripherals
- Thing:
    - ESC & servo (2x PWM)
    - IMU (I2C)
    - GPS (UART)
    - IR
- A-Star:
    - Ultrasonic
    - Proximity
    - Buzzer
    - LCD (I2C)
    - Encoder
    - Battery check

### 5. Power
- Motors: included NiMH pack
- 2p li-ion
- TP4056 charger
- 3.3v regulator
- TP4056 USB connected to powerbank
- if using Pi 4:
    - must upgrade powerbank
    - need to add solar panel to chassis

### 6. Board
- frequent usb connectors
    - on left side bottom
    - inwards, internally connected
- batteries connected via screw terminals
- 3.3v regulator output on power rail
- all gpio pins broken out (only for pcb)
- connectors for all modules

### 7. Safety
- Batteries
    - ESC has auto poweroff
    - 3v3 regulator has auto poweroff
    - must implement poweroff for Pi
    - perform shutdown functions on low voltage
- Obstacles
    - if any proximity sensor goes on stop immediately
    - if any distance sensor reads less than 10 cm stop immediately
    - add capacitive coating to chassis, connect to touch pin
    - if touch pin goes on stop immediately
    - wait for three seconds after any of the above triggers
    - if any distance sensor reads less than:
        - 20 cm: stop to read sensors
        - 30 cm: steer in clearest direction
    - if all distance sensors read less than 30 cm stop immediately
        - if clear after three seconds proceed normally
        - else try to rotate in 45* steps
- Connection
    - on connect do not start until brake is toggled
    - start with 10% speed
    - on connection lost for more than 100 ms stop
    - if connection lost multiple times in less than two minutes fully stop
    - if more than two consecutive packets are lost stop

### 8. Chassis
- manufacturing
    - 3D printing (most complicated, best long-term)
    - plexiglass/lexan (easiest, most durable, not quite industrial)
    - aluminium ????? (must check car torque first)
    - foam (difficut, tutorial available)
- design
    - small hole for ESC wires
    - holes for ultrasonic sensors
    - cable hooks???
    - panel for connectors
    - holes for cables
    - easy access to everything
    - simple way to detach from wheelbase
