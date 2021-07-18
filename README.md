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
st=>start: 開始
e=>end: 結束
op=>operation: 我的操作
op2=>operation: 啦啦啦
cond=>condition: 是或否？

st->op->op2->cond
cond(yes)->e
cond(no)->op2
```




