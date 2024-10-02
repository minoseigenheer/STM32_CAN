# STM32_CAN library

Designed for STM32F105 CAN gateway.
[https://github.com/minoseigenheer/STM32_NMEA2000_CAN_gateway](https://github.com/minoseigenheer/STM32_CAN_gateway)
Might also works with F103 / F107 / F405 / F407... and other STM32 MCU's with integrated bxCAN controller.


### To use this library, you will also need:
  - STM32 HAL for your MCU

### Can be used with the following libraries
  - NMEA2000_STM32 library https://github.com/minoseigenheer/NMEA2000_STM32
    & NMEA2000 library https://github.com/ttlappalainen/NMEA2000

### Example
- [STM32CubeIDE NMEA2000 battery project example](https://github.com/minoseigenheer/STM32_CAN_gateway/tree/main/STM32CubeIDE%20NMEA2000%20battery%20example)
  Import project into your STM32CubeIDE workspace

### How to use the library
```
#include "STM32_CAN.hpp"

// create CANbus1 object on stack
tSTM32_CAN &CANbus1 = *( new tSTM32_CAN(&hcan1, tSTM32_CAN::CAN250kbit) );

// send a frame
const unsigned char buf[8] = {
  0xFF,
  0xFF,
  0xFF,
  0xFF,
  0xFF,
  0xFF,
  0xFF,
  0xFF
};
CANbus1.CANSendFrame(0x19003838, 8, &buf);

// get a frame from the RX buffer
unsigned long id;
char len;
unsigned char buf[8];
CANbus1.CANGetFrame(&id, &len, &buf);

```

---
### STM23CubeIDE setup 
If you are not very experienced with STM23CubeIDE this guide can help to configure a project for your own STM32 based hardware.
**I recommend start with this [example](https://github.com/minoseigenheer/STM32_CAN_gateway/tree/main/STM32CubeIDE%20NMEA2000%20battery%20example)**
  - Create a STM32 C++ project in STM23CubeIDE
  - Select your MCU which will add the correct HAL to your project.
  - We can not directly call C++ from the main.c
    > Create your own application.cpp & application.hpp files with your C++ project setup() and loop() which are called from with extern "C" {... 
  - In Cube MX configure external clock settings of your MCU. 
    > CAN bus needs an external clock source!
  - Activate the CAN bus you want to use.
  - Enable the CAN RX1 interrupt in the NVIC settings
  - Enable the CAN TX interrupt in the NVIC settings
  - If you want to use this library with the NMEA2000 library
    >Copy the NMEA2000 and NMEA_STM32 library's to your "Libs" folder of your STM32Cube project 
    >**or if you are using github use linked git submodules!**.
    > ```
    > // Use the following git commands to add submodules
    > cd YourProjectRepo/Libs
    > git submodule add https://github.com/ttlappalainen/NMEA2000
    > git submodule update --init --recursive
    > git submodule add https://github.com/minoseigenheer/NMEA2000_STM32
    > git submodule update --init --recursive
    > // and update them with 
    > git submodule foreach git pull
    > ```
  - Add the library folders to the G++ compiler include paths:
    > - Select your project in the Project Explorer.
    > - go to: File > Properties > C/C++ Build > Settings > MCU G++ Compiler > Include paths
    > - Add all folders inside libs to the included paths
  - If you want to use the CAN init of the library, which sets the CAN baud rate and filters, disable the STM32CubeIDE generated code.
    > Navigate to “Project Manager” > “Advanced Settings” find MX_CAN#_Init” 
    > and enable “Do Not Generate Function Call”.
  - By default, the peripheral MSP initialization function HAL_CAN_MspInit (in stm32xxx_hal_msp.c) is automatically called and takes care of the configuration of the CAN_RX/TX GPIOs, enabling of the peripheral clock and enabling of the CAN interrupts



## Known limitations
- The 3 TX mailboxes are used in FIFO mode to make sure sequenced messages (like NMEA2000 fast packet) are sent in the correct order. This can result in a higher priority message waiting for up to 3 lower priority messages being sent first if they are already in the mailboxes. 
In the ring buffer of the library priority and order is handled correctly!
- For dual CAN devices with 28 RX filters the filters are equaly split CAN1: 0...13 | CAN2: 14...27

---
Thanks for the great NMEA2000 CAN lib example Teensyx by ttlappalainen.
https://github.com/ttlappalainen/NMEA2000_Teensyx

---
## License

    The MIT License

    Copyright (c) 2022 Minos Eigenheer

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-
    1301  USA
