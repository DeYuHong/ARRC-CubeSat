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

```mermaid
%% Example of sequence diagram
  sequenceDiagram
    Alice->>Bob: Hello Bob, how are you?
    alt is sick
    Bob->>Alice: Not so good :(
    else is well
    Bob->>Alice: Feeling fresh like a daisy
    end
    opt Extra response
    Bob->>Alice: Thanks for asking
    end
​```



