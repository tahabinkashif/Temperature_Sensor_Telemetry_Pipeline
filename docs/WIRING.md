\# Hardware Wiring \& Electrical Notes



This document describes the physical connections between:

\- LM35 temperature sensor

\- STM32 BluePill (STM32F103C8)

\- ESP32 gateway



---



\## Power Domains



| Component              | Voltage   |

|------------------------|-----------|

| STM32F103C8T6 BluePill | 3.3 V     |

| ESP32                  | 3.3 V     |

| LM35                   | 3.0–5.0 V |



\*\*Important:\*\*  

All grounds \*\*must be common\*\*.



---



\## LM35 → STM32 BluePill



| LM35 Pin | Function | STM32 Pin            |

|----------|----------|-----------           |

| 1        | VCC      | 3.3 V                |

| 2        | VOUT     | PA0 (ADC1 Channel 0) |

| 3        | GND      | GND                  |



Notes:

\- LM35 output = \*\*10 mV / °C\*\*

\- ADC reference = 3.3 V

\- Sampling time = 239.5 cycles (stable for high impedance)



---



\## STM32 ADC Configuration Summary



\- ADC: ADC1

\- Channel: 0 (PA0)

\- Trigger: TIM3 TRGO (update event)

\- Mode: Single conversion

\- Resolution: 12-bit

\- DMA: Circular mode

\- Buffer length: 200 samples



---



\## STM32 → ESP32 UART Connection



| STM32            | ESP32   |

|------------------|---------|

| PA9 (USART1 TX)  | UART RX |

| GND              | GND     |



UART settings:

\- Baud rate: 115200

\- Data bits: 8

\- Parity: None

\- Stop bits: 1



STM32 uses \*\*TX DMA\*\*; ESP32 UART driver handles RX buffering.



---



\## Clock \& Timing



\- STM32 system clock: 72 MHz (HSE + PLL)

\- TIM3 configuration:

&nbsp; - Prescaler: 7199

&nbsp; - Period: 9

&nbsp; - Update rate ≈ \*\*1 kHz ADC trigger\*\*



---



\## Electrical \& Signal Integrity Notes



\- Keep LM35 output trace short

\- Avoid powering ESP32 from BluePill 3.3 V regulator (insufficient current)



---



\## Bring-up Checklist



1\. Verify 3.3 V rails

2\. Confirm ADC readings with static voltage

3\. Observe UART output (logic analyzer / USB-UART)

4\. Verify frame sync (`0xAA55`)

5\. Check CRC validity on ESP32 side



