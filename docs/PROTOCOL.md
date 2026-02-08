\# Telemetry Binary Protocol Specification



This document defines the \*\*binary telemetry frame format\*\* transmitted from the

STM32 BluePill (edge node) to the ESP32 (gateway).



The protocol is optimized for:

\- Deterministic parsing

\- Low overhead

\- DMA-friendly framing

\- Forward compatibility via versioning



All fields and examples match the current STM32 firmware implementation.



---



\## Byte Order



All multi-byte integers are \*\*little-endian\*\*.



Example:

\- `0x1234` → `34 12`



---



\## Frame Overview



Each telemetry update is sent as a \*\*single binary frame\*\*:



| Field          | Size (bytes) | Description                       |

|----------------|--------------|-----------------------------------|

| Magic          | 2            | Frame sync word `0xAA55`          |

| Version        | 1            | Protocol version (`1`)            |

| Msg Type       | 1            | Message type (`0x01` = telemetry) |

| Payload Length | 2            | Length of payload in bytes        |

| Sequence       | 2            | Monotonic packet counter          |

| Timestamp      | 4            | `HAL\_GetTick()` in ms             |

| Payload        | N            | Telemetry payload                 |

| CRC16          | 2            | CRC-16/CCITT-FALSE                |



Total frame size = `12 + payload\_length + 2`



---



\## Header Layout (12 bytes)



| Offset | Field          | Type     |

|--------|----------------|----------|

| 0      | Magic          | `uint16` |

| 2      | Version        | `uint8`  |

| 3      | Msg Type       | `uint8`  |

| 4 	 | Payload Length | `uint16` |

| 6 	 | Sequence       | `uint16` |

| 8 	 | Timestamp (ms) | `uint32` |



---



\## Payload Layout (Telemetry Message)



Payload length: \*\*5 bytes\*\*



| Offset |     Field     | Type    | Units      |

|--------|---------------|---------|------------|

| 0      | `filt\_adc`    | `uint16`| ADC counts |

| 2      | `temp\_c\_x100` | `int16` | °C × 100   |

| 4      | `flags`       | `uint8` | Bitfield   |



---



\## Flags Field



| Bit | Mask   | Meaning                   |

|-----|--------|---------------------------|

| 0   | `0x01` | ADC half-buffer processed |

| 1   | `0x02` | ADC full-buffer processed |

| 2   | `0x04` | Filter initialized        |

| 3–7 |   —    | Reserved                  |



(Current firmware sets flags to `0x00`.)



---



\## CRC16 Definition



\- Algorithm: \*\*CRC-16/CCITT-FALSE\*\*

\- Polynomial: `0x1021`

\- Init value: `0xFFFF`

\- XOROUT: `0x0000`

\- RefIn / RefOut: `false`



CRC is computed over:



\[Magic .. Payload]





and appended as \*\*little-endian\*\*.



---



\## Example Frame (Annotated)



Assume:

\- `filt\_adc = 2048`

\- `temp\_c\_x100 = 2534` (25.34 °C)

\- `flags = 0x00`

\- `sequence = 12`

\- `timestamp = 123456 ms`



\### Payload

00 08 E6 09 00





\### Header (little-endian)

55 AA  // Magic

01  // Version

01  // Telemetry

05 00  // Payload length

0C 00  // Sequence

40 E2 01 00  // Timestamp





\### CRC Input (hex)

55 AA 01 01 05 00 0C 00 40 E2 01 00 00 08 E6 09 00





\### Full Frame

55 AA 01 01 05 00 0C 00 40 E2 01 00 00 08 E6 09 00 XX XX





(`XX XX` = CRC16, little-endian)



---



\## Parser Requirements (ESP32)



A compliant receiver must:

1\. Search for `0xAA55`

2\. Read fixed 12-byte header

3\. Read `payload\_length` bytes

4\. Read CRC16

5\. Compute CRC and compare

6\. On CRC error: drop frame and resync



---



\## Versioning Strategy



\- `Version` field allows backward-compatible extensions

\- New payload fields should increase `payload\_length`

\- New message types should use a new `msg\_type` value



---



\## Message Types



| Type   | Meaning   |

|--------|-----------|

| `0x01` | Telemetry |

| `0x02` | Reserved  |

| `0x03` | Reserved  |









