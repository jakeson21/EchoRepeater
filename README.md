# EchoRepeater
STM32F722 based Audio Echo Repeater

# Description
Utilizes an onboard ADC and DAC to echo back the sampled audio, stored in a short buffer, sampled at 50kHz. 
It uses a power-threshold based trigger to begin recording into a configurable (up to 0.5s) length buffer.
Once filled, it ignores the incoming samples and plays back the buffer.
Preceeding playback is a FIR filter, currently set as a 64-tap moving average filter.
- All scaling, data conversion, and DSP routines use [ARM CMISS](https://github.com/ARM-software/CMSIS/tree/v4.3.0/CMSIS)
routines as much as possible for performance.
- DMA is used to transfer samples from the ADC to memory and from another memory location into the DAC.
- TIMER4 CH1 is configured as a Global Interrupt source and as `Output Compare CH1` with `Toggle on match` set, running at 50kHz period.
  - PD12 is toggled on match to enable tracking with an external logic analyzer
  - TIM4 trigger event sets the DMA trgger and conversion rate of the ADC, as well as te DAC transfer trigger
  - ADC1 External Conversion Source is set to `Timer 4 Trigger Out event` with  External Trigger Conversion Edge set to 'Trigger on rising edge`
    - ADC1 is set to 12bit Right aligned
  - DAC is triggered by Timer 4 Trigger Out event
  - ADC and DAC buffer length `ADC_BUF_LEN` is set to 64, with a block processing length `BLOCK_SIZE` of 32
  
  
