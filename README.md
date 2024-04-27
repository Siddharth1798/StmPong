# StmPong <br />
### 1 &nbsp; Description <br />
A Simple Multiplayer Game Console to play Ping-Pong. The STM is connected to an OLED display to track the movements of the ball which utilizes concepts of I2C and Timers. The communication between the two devices is established using ESP8266 which communicates with the STM using UART. The ESP8266 solely handles Wi-Fi connectivity to synchronize OLED displays, ensuring both consoles show identical game states. One ESP8266 will act as an access point and the other will connect to this acting as a client. The button status and Game state will be communicated between the to STM using this connection.

### 2 &nbsp; Components Required <br />
- Stm32discovery board
- Esp01 module
- SSD1306 OLED display
- Buzzer
- Game console PCB
- Push Buttons
- MOSFET
- 3 x 10k Resistors

### 3 &nbsp; Functional Block Diagram <br />
<p align="center">
<img src="/images/block_diagram.png" height="80%" width="80%">  
</p>

### 4 &nbsp; Connection Table <br />
| **OLED**  | **STM32**   
| -------------|------------- 
| VCC	|3.3 V
| SDA	 | PB11(SDA)
| SCL	| PB13 (SCL)
| GND	| GND

| **ESP01**  | **STM32**   
| -------------|------------- 
| VCC	|3.3 V
| ESP_RX	 | PC4(USART3_TX)
| ESP_TX	| PC5(USART3_RX)
| ENABLE	| 3.3V
| GND	| GND

| **BUZZER**	| **STM32**
| -------------|------------- 
| +ive	| PB2
| -ive	| GND

| **Buttons** |	**STM32**
| -------------|------------- 
| Button 1	| PA4
| Button 2	| PA5
| Button 3	| PA6
| Button 4	| PA7

### 5 &nbsp; Built-in Peripherals <br />
| **Peripheral**  | **Function**   
| -------------|------------- 
| USART3	       | It's used to communicate STM32 with PC
| USART1         | It's used for communication between STM32 and ESP01
| I2C         | It's used to interface STM32 with OLED


### 6 &nbsp; Milestones <br />
|          | **Milestones Achieved**   
| -------------|------------- 
| Milestone 1       | A custom PCB will be designed to hold the OLED display, ESP8266 and the buttons. This PCB Design will be shown as Milestone 1 and will be fabricated once it is reviewed by the TA.
| Milestone 2       | Interfacing the OLED display using I2C and displaying a simple object to make sure the display connection is properly established. Interfacing ESP8266 with STM32 and establishing a simple connection between two STMs. Making sure a switch from one STM toggles the LED on the second STM to make sure the connection is established. Interfacing all the buttons using interrupts.
| Milestone 3       | Creating the required ball motion in the oled display. Making the paddle move when the button is pressed accordingly. Creating a simple menu to connect to another device and start the game.
| Milestone 4       | Completed the game in single-player mode.

### 7 &nbsp; Challenges faced during implementation <br />
- The OLED's refresh rate was 30 fps but the default firmware of ESP01 had a lower transmission rate which caused data loss hindering the establishment of WiFi Communication between two consoles.
- The OLED interfacing was challenging to implement.
- The esp01 and oled combined were not able to draw sufficient current to work simultaneously.
- The Buzzer didn't draw enough current from GPIO pins to trigger.


### Contributors
Tharnath Bagirathan<br>
Philip Gabriel Johnson<br>
Siddharth Kaushal Manga<br>
