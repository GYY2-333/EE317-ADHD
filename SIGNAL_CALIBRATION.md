# Signal-mode channel calibration (2026-09-22)

Only `Core/Src/app_hid_handler.c` was changed for this calibration.
`CalibrateSignalFrame()` runs after copying a new 30-sample frame, only
when `current_mode == MODE_SIGNAL`. Payload is little-endian and interleaved
CH0..CH5. Header, impedance history/delays, sampling ISR, and USB retry logic
are unchanged. Pending frames are not calibrated again on retry.

Sources in `C:/Users/ADMIN/Desktop/ADHD数据/`:
- `新32_1.csv`, `新32_2.csv` (1250 samples/channel each).
- `原设备1.csv`, `原设备2.csv` (1250 samples/channel each).

User confirmed identical input conditions. Old `32_*.csv` files were not used.
DC = arithmetic mean of all samples. Vpp = max - min per recording.
For each channel, average the two source DC/Vpp measurements and the two
reference measurements separately. Gain = reference Vpp / source Vpp.
Output = source sample * gain + offset; offset = reference DC - source DC * gain.
Gain is quantized to Q16 first, then offset is computed with that quantized
gain. Round to nearest code and saturate to 0..65535 using 64-bit intermediate
arithmetic. No AGC, filtering, or phase correction is applied.

| Channel | Source DC | Source Vpp | Reference DC | Reference Vpp | Gain Q16 | Offset Q16 |
|---|---:|---:|---:|---:|---:|---:|
| CH0 | 32041.3104 | 6761 | 39415.0580 | 7918 | 76751 | 123902627 |
| CH1 | 32041.3080 | 6771 | 39416.1180 | 7922.5 | 76681 | 126215171 |
| CH2 | 32041.3832 | 6763 | 35825.4464 | 6958 | 67426 | 187434152 |
| CH3 | 32041.1712 | 6759 | 16810.0132 | 3278.5 | 31789 | 83104234 |
| CH4 | 32041.0628 | 6752 | 24499.9784 | 4758.5 | 46187 | 125750017 |
| CH5 | 32041.0936 | 6753.5 | 35836.8696 | 6966 | 67598 | 182691241 |

## Verification

Keil ARMCC 5.06 build: 0 errors, 0 warnings. J-Link SWD programmed and verified
STM32F373CC flash. Hardware USB HID test exercised signal -> impedance -> signal.
Impedance capture: 200 frames, no sequence gaps, original uncalibrated channel
levels approximately 31950..31960 codes (mean), 854..861 codes Vpp.

Initial power/mode transition produced a large transient including clipping.
After settling, two further signal captures each used 1250 samples/channel
(first 100 frames discarded); both had no sequence gaps:

| Channel | Live DC 1 | Live Vpp 1 | Live DC 2 | Live Vpp 2 |
|---|---:|---:|---:|---:|
| CH0 | 39420.6136 | 7936 | 39386.0808 | 7965 |
| CH1 | 39422.0432 | 7924 | 39386.7440 | 7952 |
| CH2 | 35830.8552 | 6962 | 35798.8496 | 7002 |
| CH3 | 16812.6992 | 3281 | 16797.3304 | 3309 |
| CH4 | 24504.3232 | 4767 | 24481.6400 | 4808 |
| CH5 | 35843.5544 | 6984 | 35809.8464 | 7026 |

Fixed coefficients match these calibration conditions; they do not force every
future recording to the same amplitude. Input changes remain visible. Larger
signals can clip at the unsigned 16-bit output limits. Firmware was left in
signal mode, waiting for start after handshake; host may need to reconnect.
