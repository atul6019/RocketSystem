# A35 Application Layer (Linux / Python)

Runs on the Cortex-A35 (Linux) side of each MYIR MYD-YM62X. One driver per
physical device plus the application logic that turns sensor data into actuator
commands. Every driver is written against the device's **real** wire protocol
but takes its transport (I2C/UART) as an injected object, so the same code runs
on the flight computer and on a dev machine using the in-memory sim transports.

## Device -> driver map

| Hardware | Driver | Bus | Protocol notes |
|----------|--------|-----|----------------|
| UPS HAT (C) / (D) | `drivers/ina219.py` | I2C | INA219 regs 0x00–0x05; SoC from cell-voltage curve |
| JY901S IMU | `drivers/jy901s.py` | UART | WitMotion 0x55 frames (0x51/0x52/0x53) |
| BMP388 baro | `drivers/bmp388.py` | I2C | Bosch float compensation (primary backup baro) |
| BME688 env | `drivers/bme68x.py` | I2C | T/P/H float comp; gas needs BSEC (out of scope) |
| LC76G GNSS | `drivers/lc76g.py` | UART | NMEA 0183 GGA/RMC/VTG |
| ST3025 servo | `drivers/st3025.py` | UART | **Feetech/STS**, header `0xFF 0xFF`, LE, broadcast 0xFE |
| Servo Driver (ESP32) | `drivers/servo_driver_esp32.py` | USB/UART | serial-forwarding bridge to the STS bus |
| TOF Laser Range | `drivers/tofsense.py` | UART | Nooploop NLink frame, header `0x57` |
| IMX415 camera | `drivers/imx415.py` | USB/UVC | V4L2 via ffmpeg, event-triggered recording |

> Two corrections surfaced while verifying datasheets: the ST3025 servo bus uses
> the **`0xFF 0xFF`** Feetech header (the project spec's `0x55 0x55` is wrong),
> and the TOF sensor speaks the **Nooploop NLink** frame, not a raw VL53L0X.

## Application logic

- `flight_types.py` — shared dataclasses/enums (the common currency).
- `config.py` — loads `config.json`, converts to SI.
- `hal.py` — sensor/link abstraction + a scripted-flight `SimBackend`.
- `sensor_fusion.py` — 3-state vertical Kalman filter (altitude/velocity/accel)
  fusing baro + IMU + GPS, plus tilt and apogee detection/prediction.
- `recovery_manager.py` — apogee/deploy decision logic that advises the M4.
- **`attitude_control.py`** — the sensor→actuator loop: IMU attitude/rates → PID
  → fin mix → ST3025 `SYNC_WRITE`. Four fins in a `+` layout, per-axis PD with
  gyro rate damping, deflection-limited, all fins commanded in one bus frame.

## Run the demo / tests

```sh
cd a35-app
python3 sim/run_actuator_loop.py
```

Part 1 decodes each driver's real protocol through the sim transports. Part 2
runs the closed loop: a +10° pitch disturbance read from the IMU driver drives
fin A below neutral and fin C above it (countering the tilt), and both relax to
neutral when level. Expected tail: `ALL CHECKS PASSED`.

Force simulation explicitly anywhere with `ROCKET_HAL=sim`.

## Not yet wired

`main_flight.py` (the threaded orchestrator that ties drivers → fusion →
recovery → telemetry/logging together and gates the attitude controller by
flight phase) and the standalone `data_logger`/`telemetry_handler`/`health_monitor`
modules. The pieces they'd compose all exist above.
