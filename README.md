# ARRC-CubeSat
   *Editor : Vim*  
   *Toolcahin : gcc-arm-none-eabi*  
   *Debuger : Openocd/st-link*  
   -------------------------------
   *Hardware : stm32f407discovery*  
   *System OS : FreeRTOS <https://www.freertos.org/index.html>*  
   
***
### CubeSat-System List
- [x] Close
  - [x] CubeSat system structure
     - Launch test Mode 
     - Beacon Mode 
     - Payload Mode(part on-going) 
- [ ] On-going 
  - [ ] GPIO Interrupt Launch signal & Separate signal
  - [ ] Battery ADC onvert
  - [ ] Payload ADC & data packet
***
### Main flow chart

'''mermaid
graph LR
A[Square Rect] -- Link text --> B((Circle))
A --> C(Round Rect)
B --> D{Rhombus}
C --> D
'''



