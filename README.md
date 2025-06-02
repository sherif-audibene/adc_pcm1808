# ESP32 PCM1808 ADC Interface with PCM5102 DAC

This project demonstrates how to interface the PCM1808 24-bit ADC and PCM5102 DAC with an ESP32 microcontroller to create a complete audio loop. The PCM1808 is a high-quality audio ADC, and the PCM5102 is a high-quality audio DAC.

## Hardware Setup

### Required Components
- ESP32 Development Board
- PCM1808 ADC Module
- PCM5102 DAC Module
- Audio Input Source (microphone, line-in, etc.)
- Audio Output (speakers, headphones, etc.)
- Jumper Wires

### Connections
```
ESP32    ->   PCM1808 (ADC)
GPIO0    ->   MCLK    (Master Clock)
GPIO5    ->   BCK     (Bit Clock)
GPIO7    ->   WS      (Word Select/LRCLK)
GPIO6    ->   DOUT    (Data Output)
3.3V     ->   VCC
GND      ->   GND

ESP32    ->   PCM5102 (DAC)
GPIO18   ->   BCK     (Bit Clock)
GPIO17   ->   DIN     (Data Input)
GPIO16   ->   LCK     (Left/Right Clock)
3.3V     ->   VCC
GND      ->   GND
```

## Software Features

### Clock Generation
- Generates a 12.288 MHz MCLK signal using ESP32's LEDC peripheral
- This clock is used to drive the PCM1808 ADC
- The MCLK frequency is set to 256 × 48kHz for optimal performance

### I2S Configuration
- Configured in I2S master mode
- Sample rate: 48kHz (PCM1808 default)
- 32-bit data width (PCM1808 outputs 24-bit data)
- Mono channel format
- I2S bus shared between ADC and DAC

### Signal Analysis
The code provides real-time analysis of the audio signal:
- Signal level monitoring (as percentage of maximum)
- Peak value detection
- Maximum and minimum values
- Average signal level
- Visual level meter
- Raw sample values with percentage of maximum

### Audio Loop
- Captures audio from PCM1808 ADC
- Processes and analyzes the signal
- Outputs the processed audio to PCM5102 DAC
- Real-time audio monitoring capability

## Usage

1. Connect the hardware as described above
2. Flash the code to your ESP32
3. Monitor the serial output to see the signal analysis
4. Connect speakers or headphones to the DAC output
5. The output will show:
   - Signal level as percentage of maximum
   - Peak values
   - Visual level meter
   - Raw sample values
6. You should hear the input audio through the speakers/headphones

## Signal Interpretation

- Signal levels are shown as percentages of the maximum 24-bit value
- Values typically range from -8,388,608 to 8,388,607
- A good signal should show:
  - Both positive and negative values
  - Varying levels (not constant)
  - No clipping (not hitting maximum values)
  - Clean transitions

## Troubleshooting

If you're not getting expected results:
1. Check all connections
2. Verify MCLK is present on GPIO0
3. Ensure audio input is properly connected
4. Check signal levels in the serial output
5. Verify power supply is stable
6. Check DAC output connections
7. Verify speaker/headphone connections
8. Check volume levels on output device

## Future Improvements

Potential enhancements:
1. Automatic gain control
2. Frequency analysis
3. Audio processing features
4. Data logging capabilities
5. Web interface for monitoring
6. Volume control
7. Audio effects processing
8. Multi-channel support

## License

This project is open source and available under the MIT License.
