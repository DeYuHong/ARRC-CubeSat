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
     - ~~Launch test Mode~~   
     - ~~Beacon Mode~~  
     - Payload Mode(part on-going) 
- [ ] On-going 
  - [ ] GPIO Interrupt Launch signal & Separate signal
  - [ ] Battery ADC onvert
  - [ ] Payload ADC & data packet
***
### Main flowchart

***  
```flow
st=>start: 使用者登陸
op=>operation: 登陸操作
cond=>condition: 登陸成功 Yes or No?
e=>end: 進入後臺

st->op->cond
cond(yes)->e
cond(no)->op

```




