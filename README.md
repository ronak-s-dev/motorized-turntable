# Motorized Cake Decorating Turntable

An ESP32-based motorized turntable designed to make cake decorating more precise, consistent, and convenient.

The project combines programmable rotational speed, stepper motor control, and a custom-built mechanical enclosure into a compact, aesthetically pleasing machine.

**Status:** Active development — firmware build-tested; hardware validation pending.

## Project Goals

The goal is to build a reliable motorized turntable suitable for small bakery and cake-decorating applications.

Planned capabilities include:

- Adjustable rotational speed from 1–100 RPM.
- Clockwise and anticlockwise rotation.
- Rotary encoder for speed selection.
- Push-button start/stop control.
- OLED display showing actual rotational speed.
- Closed-loop speed regulation using a Hall-effect sensor.
- Smooth, low-vibration operation under load.
- Physical emergency stop.
- Manual operation through a mechanical drive disengagement mechanism.
- Custom-designed plywood enclosure and removable food-contact platter.

## Hardware

The project uses an ESP32 microcontroller as its central controller.

| Component                | Purpose                                  |
| ------------------------ | ---------------------------------------- |
| ESP32 DevKit V1          | Main controller                          |
| EC11 rotary encoder      | Speed selection and user input           |
| SSD1306 OLED (128×64)    | User interface                           |
| TMC2209 stepper driver   | Stepper motor control                    |
| Bipolar stepper motor    | Rotational drive                         |
| A3144 Hall-effect sensor | Rotational speed feedback                |
| 24V DC power supply      | Main motor power                         |
| Buck converter           | Regulated supply for control electronics |

Some components are awaiting mechanical integration or physical testing.

## Firmware

The firmware is developed in **C using ESP-IDF 6.1**.

### Development Environment

- ESP32 DevKit V1
- ESP-IDF 6.1
- C programming language
- FreeRTOS
- Zed Editor
- Debian on WSL2

### Implemented and Build-Tested

- [x] ESP-IDF installation and project setup.
- [x] GPIO input and output configuration.
- [x] LED blinking using FreeRTOS delays.
- [x] Push-button input using internal pull-up resistors.
- [x] Software debouncing using elapsed-time measurement.
- [x] Button-controlled LED toggle.
- [x] Rotary encoder GPIO configuration.
- [x] Quadrature encoder state detection.
- [x] Direction detection using a 4×4 lookup table.
- [x] Encoder transition counting.
- [x] RPM selection logic using encoder detents.
- [x] Target RPM boundaries (0–100).
- [x] SSD1306 OLED low-level initialization and graphics testing.
- [x] MCPWM-based STEP pulse generation.
- [x] Motor direction and enable GPIO configuration.
- [x] TMC2209 enable/disable control logic.
- [x] Motor start/stop control logic.
- [x] Acceleration and deceleration ramp implementation.
- [x] Error handling for motor control and GPIO initialization.

**Note:** The motor-control implementation has compiled successfully but has not yet been validated with the physical motor, driver, and power supply. The current RPM-to-step-frequency mapping is provisional and requires calibration.

### Pending Firmware Development and Validation

- [ ] Physical motor and TMC2209 integration.
- [ ] Verify STEP pulse generation and motor direction.
- [ ] Validate acceleration and deceleration behaviour.
- [ ] Calibrate step frequency against actual platter RPM.
- [ ] Complete OLED RPM display integration.
- [ ] Hall-effect sensor integration.
- [ ] Closed-loop RPM regulation.
- [ ] Clockwise/anticlockwise user control.
- [ ] Hardware emergency stop integration.
- [ ] Final firmware and hardware integration.

## Mechanical Design

The mechanical assembly is planned around a large rotating platter supported by a suitable bearing and driven through a timing-belt transmission.

Design priorities include:

- Stable support for cakes weighing approximately 10 kg.
- Minimal platter wobble and rotational runout.
- Low vibration at slow rotational speeds.
- Adjustable belt tension.
- Mechanical disengagement for manual operation.
- Removable, food-safe platter.
- An elegant enclosure inspired by premium audio turntables.

The enclosure is being designed around laminated plywood, with custom-designed electronic mounting features and a clean, minimal user interface.

CAD models, 3D-printed components, and fabrication files will be added as development progresses.

## Safety Considerations

Safety is an important part of the design.

Planned provisions include:

- A physical emergency stop that independently interrupts motor-drive power.
- Appropriate electrical protection and fusing.
- Separation of food-contact surfaces from the wooden enclosure.
- Secure mounting and guarding of rotating mechanical components.
- Testing under progressively increasing loads before regular use.

The machine will not be considered ready for regular use until electrical, mechanical, and operational safety have been validated.

## Project Structure

```text
motorized-turntable/
├── firmware/       # ESP32 firmware
├── electronics/    # Circuit diagrams and wiring
├── mechanical/     # CAD and fabrication files
├── docs/           # Architecture and development notes
└── README.md
```

The repository will evolve as the project progresses.

## Learning and Development

This project is also an opportunity to explore embedded systems and mechanical design through practical implementation.

Key learning areas include:

- Embedded C and ESP-IDF.
- GPIO, digital inputs, and hardware interfaces.
- Rotary encoder quadrature decoding.
- FreeRTOS task scheduling.
- MCPWM peripheral and stepper motor control.
- Stepper driver configuration.
- Sensor feedback and closed-loop control.
- CAD and 3D printing.
- Mechanical transmission design.
- Fabrication and enclosure construction.

## License

To be determined.
