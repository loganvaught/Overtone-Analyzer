# Overtone-Analyzer
Harmonic analysis tools to help musicians visualize how fundamental frequencies and their harmonic overtones interact with each other.
Currently in progress, debug comments and print statements present.

# To Be Added
- UART to PC communication allowing data visualization through a Python program
- Custom PCB and 3D printed enclosure using large OLED screen for completely embedded device
- Configurable variables through physical components like switches and sliders

## Currently Implemented
- ADC polling with DMA and circular buffers
- CMSIS RTOS V2 implementation with one task for processing data
- Identification of multiple musical fundamental frequencies and their overtone frequencies, and relative volumes
- UART serial debugging outputs harmonic data to terminal

## Results
- Device successfully visualizes differences in individual instrument harmonic 'footprints' through overtone richness and relative volume
- Parabolic interpolation efficiently and accurately estimates frequencies using FFT bins
- Great tool for musicians to visualize how their tone behaves in various contexts using different techniques

## Repo Organization
STM32CubeMX generates a lot of required files found in this repo.
The code that I wrote can be found in these locations:
```
Overtone Analyzer/ (Project container folder)
└── Core/
      └── Src/                  # main.c
└── Overtone Analyzer.ioc       # Pin mapping in STM32CubeMX
```
## Parts
- STM32F411CEU6 Blackpill Board
- MAX9814 Audio Amplifier Microphone

# Design
Note: Block diagram to be added.

## Peripherals / Functionality
- UART: Serial debugging
- Timers/ADC: Triggers continuous ADC conversion from MAX9814 output
- DMA: Allows continuous storage of MAX9814 output while CPU processes data from circular buffer
- FreeRTOS through CMSIS RTOS V2: Enables concurrent data processing and transmission by managing tasks with different priority levels
- FFTs: Uses Fast Fourier Transforms through CMSIS DSP with parabolic interpolation to estimate frequencies and relative volume.
## Project Challenges
- Maximizing RAM usage for DSP task while staying within F411 limits
- Creating a continuous embedded DSP system that transforms ADC data into abstracted, understandable data visualization on PC
- Designing an algorithm to filter out noise, then identify fundamental frequency candidates, and their harmonic overtones
## Design Choices / Tradeoffs
- F411: Built in FPU (floating point unit) accelerates FFT calculations. RAM size allows for FFT bin resolutions that are reasonable for musicians
- MAX9814: Automatically adjusts gain, consistently providing a measurable ADC output for quieter instruments and larger setups. However, removes ability for direct volume measurement of fundamental frequencies.
## How to Make
- Clone the repo, and open the Klaus container folder (inside this repo) in STM32CubeIDE
- Use an ST-Link (or other method) to flash the STM32F411CEU6
- View pin mapping by opening .ioc file in STM32CubeMX
- View block diagram below for help putting device together. (Note: diagram does not show rotary encoder or mode-switching button)
- Supply 5v to the STM32 5v pin, and the DRV2605L haptic board. Supply 3.3v to the NRF24L01+ board; or, use an NRF24L01 adapter board, which takes 5V. 
