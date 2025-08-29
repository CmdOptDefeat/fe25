# Bill of Materials

## Chassis
| Item | Part No. | Quantity | Notes/Description | Link | 
|:-----------:|:------------:|:------------:| :------------:| :------------:|
| N20 Motor Mounting Bracket | - | 1 | Hold N20 motor in place in the rear drive section | [Product Link](https://robu.in/product/mounting-bracket-n20-micro-gear-motors) |
| Gears | Metal 24 tooth spur gear, 3mm | 2 | Used in the rear drive | [Product Link](https://robu.in/product/metal-48p-spur-gear-24t/) |
| Rear Drive Section | Custom design | 1 | Holds rear axle and N20 motor and connection through gear | [Link to Design](https://github.com/pkr2308/Ctrl-Alt-Defeat-WRO-Future-Engineers-2025/blob/main/Design%20Files/N20%20Motor%20Holder.stl) |
| Shaft | 3mm | 1 | Used as rear drive shaft |
| Chassis | RBP68138C | 1 | Front steering and single axle rear drive | [Product Link](https://www.elecrow.com/4wd-smart-car-robot-chassis-for-arduino-servo-steering.html) |
| 18650 Batteries | 3.7 V Different mAH options availaible | 2 | Provide power | [Product Link](https://robu.in/product/dmegc-inr18650-26e-3-7v-2600mah-li-ion-battery) |
| 21700 Batteries | 3.6 V Used with UPS Hat to power the Pi | 4 | Provide power | [Product Link](https://robu.in/product/bak-nmc-n21700cgp-3-6v-4000mah-10c-li-ion-battery) |
| Battery Holder | - | 1 | Hold the 18650 batteries, and has an on/off switch | [Product Link](https://robu.in/product/18650-x-2-battery-holder-with-cover-and-on-off-switch) |
| Custom Designed Parts | - | 1 each | Used for attaching various components to the robot |[Design files](https://github.com/pkr2308/Ctrl-Alt-Defeat-WRO-Future-Engineers-2025/tree/main/Design%20Files) |
| Miscellaneous | - | - | Wires, Screws, Pillar Screws etc. | [Pillar Screws](https://robu.in/product/pro-range-120pcs-m3-hexagon-copper-pillar-screw-kit/) |

## RP2040 System
| Item | Part No. | Quantity | Notes/Description | Link | 
|:-----------:|:------------:|:------------:| :------------:| :------------:|
| RP2040 | Waveshare RP2040-Zero | 1 | Real time control of motors, sensors | [Product Link](https://robu.in/product/waveshare-rp2040-zero-without-header/) |
| Peripherals Board | Custom design | 1 | Various compnents may be easily connected to RP2040 | [Link to Design](https://github.com/pkr2308/Ctrl-Alt-Defeat-WRO-Future-Engineers-2025/tree/main/hw/peripherals-board-v2) |
| 1D LiDAR | Benewake TF-LUNA Micro | 4 | Finding distances to front, sides and back of robot | [Product Link](https://robu.in/product/benewake-tf-luna-lidar-distance-sensor/) |
| 1D LiDAR Connectors | 28AWG 1.25mm 6 Pin Female Connector | 4 | Connect LiDAR to peripherals board | [Product Link](https://robu.in/product/a1251-06y-1-25mm-6-pin-female-housing-connector-with-300mm-wire30-awg/?gad_source=1&gad_campaignid=17427802703&gbraid=0AAAAADvLFWcLc1GCq-hK5RDM6ID4Mj9-K) |
| IMU | BNO055 | 1 | Obtaining yaw readings | [Product Link](https://thinkrobotics.com/products/9-dof-absolute-orientation-bno055-sensor) |
| Wireless Radio Transciever (Optional) | NRF24L01 Wireless Transceiver | 1 | Wireless communication during testing and debugging | [Product Link](https://robu.in/product/m177-nrf24l01-2-4ghz-antenna-wireless-transceiver-module)
| Motor Driver Board | TB6612FNG | 1 | Control N20 Motor | [Product Link](https://robu.in/product/motor-driver-tb6612fng-module-performance-ultra-small-volume-3-pi-matching-performance-ultra-l298n/) |
| Motor with Encoder | N20 6V 200RPM Micro Metal Gear Motor With Encoder | 1 | Rear driving motor | [Product Link](https://robu.in/product/n20-6v-200rpm-micro-metal-gear-motor-with-encoder/) |
| Servo Motor | MG996R 180° | 1 | For steering | [Product Link](https://robocraze.com/products/mg996r-servo-motor?variant=40192896368793) |
| Wire Management | 3, 4 Pin JST-XH 2.5mm wafers and housing; DuPont 2.54mm female headers | 1, 6, 34 | Connect various components easily onto peripherals board | - |
| Screw Terminals | 2 Pin Screw Terminals | 2 | Connecting power and motor pins to peripherals board | [Product Link](https://robu.in/product/xy128vc-5-0-xinya-2-pin-screw-terminal-blockgreen) |

## Raspberry Pi 5 System
| Item | Part No. | Quantity | Notes/Description | Link | 
|:-----------:|:------------:|:------------:| :------------:| :------------:|
| Raspberry Pi 5 | Raspberry Pi 5 Model 8GB | 1 | Object and lane detection and route planning | [Product Link](https://robu.in/product/raspberry-pi-5-model-8gb/) |
| Cooler for Pi | Raspberry Pi 5 Active Cooler | 1 | Cool the Pi when running | [Product Link](https://robu.in/product/official-raspberry-pi-5-active-cooler) |
| Picamera | Pi Camera Module 3 - Wide | 1 | Object, colour detection | [Product Link](https://robu.in/product/raspberry-pi-camera-module-3-wide) |
| Picamera Wire | Raspberry Pi 5 Camera FPC Cable 300mm | 1 | Connects PiCamera to RPi 5 | [Product Link](https://robu.in/product/official-raspberry-pi-5-camera-fpc-cable-300mm/) |
| USB A - USB C Cable | - | 1 | Connect the Pi to the RP2040 | - |
| UPS Hat | WaveShare UPS Hat E | 1 | Used power constant 5V supply to RPi | [Product Link](https://robu.in/product/ups-hat-e-for-raspberry-pi/) |
