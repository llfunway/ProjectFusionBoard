# Conversation Handoff

## Project Context

- Project: `ProjectFusionBoard`
- MCU firmware project with:
  - `3x AD7177` ADC path
  - `MPU6050` IMU path
  - UART text output
- Main files involved in this conversation:
  - [Core/Src/main.c](Core/Src/main.c)
  - [Drivers/I2C/mpu6050.c](Drivers/I2C/mpu6050.c)
  - [Drivers/I2C/mpu6050.h](Drivers/I2C/mpu6050.h)
  - [Drivers/I2C/i2c_hal_wrapper.c](Drivers/I2C/i2c_hal_wrapper.c)

## User Goal History

The user asked to:

1. Read the project README and reconnect MPU6050 attitude functionality into the main program.
2. Keep the ADC path and use "方案1":
   - initialize MPU6050 in the main program
   - update attitude in the loop
   - output ADC + attitude together
3. Later add a fused result column:
   - `vertical_proj`
   - final UART output should be 6-column txt/tab format
4. Then investigate slow startup and slow settling of `pitch`
5. Then improve the MPU6050 fusion logic for startup and drift behavior

## Current `main.c` State

The current disk content of [Core/Src/main.c](Core/Src/main.c) is **not the original old version**. It already contains the main integration changes.

### What is currently in `main.c`

- Dual-rate scheduling:
  - `ADC_FREQUENCY_HZ = 50`
  - `IMU_FREQUENCY_HZ = 200`
- MPU6050 is initialized in `Init_All_Drivers()`
- MPU6050 is updated in the main loop via:
  - `MPU6050_ReadRawData(&hMPU6050)`
  - `MPU6050_UpdateAttitude(&hMPU6050)`
- UART output is 6 columns:
  - `adc1`
  - `adc2`
  - `adc3`
  - `roll`
  - `pitch`
  - `vertical_proj`
- `vertical_proj` is computed in `Process_Sensor_Data()`
- Debug prints that were temporarily added earlier have been removed from `main.c`
  - no `BOOT`
  - no `MPU_INIT ...`
  - no `I2C_SCAN ...`
- Error loop still prints:
  - `INIT_ERROR:<code>`

### Important note

During cleanup, `main.c` was rechecked and confirmed to still contain:

- MPU6050 integration
- 6-column UART output
- `vertical_proj`
- IMU/ADC split timing

So if board behavior seems inconsistent, it is more likely due to:

- the current `mpu6050.c` fusion logic
- build output not matching edited files
- or runtime behavior, not because `main.c` reverted to the original repo version

## Current `mpu6050.c` State

The current disk content of [Drivers/I2C/mpu6050.c](Drivers/I2C/mpu6050.c) still contains the fusion-related modifications added during this conversation.

### What is currently in `mpu6050.c`

- Startup alignment logic:
  - `MPU6050_ALIGN_STARTUP_DELAY_MS`
  - discard samples
  - alignment sample collection
  - fallback sample path
- Startup attitude initialization from accelerometer:
  - `MPU6050_InitAttitudeFromAccel()`
- Startup quaternion initialization from roll/pitch:
  - `MPU6050_SetQuaternionFromRollPitch()`
- Gyro and accel calibration offset subtraction in `MPU6050_ReadRawData()`
- Mahony startup boost logic:
  - `nominalTwoKp`
  - `nominalTwoKi`
  - `startupBoostUntil`
- Continuous accel trust weighting in `MPU6050_UpdateAttitude()`:
  - `accelWeight`
  - based on deviation of `|a|` from `1g`
- Runtime gyro bias adaptation while static:
  - `runtimeGyroBiasX`
  - `runtimeGyroBiasY`
  - `runtimeGyroBiasZ`
- Runtime gyro bias is subtracted from gyro before attitude integration

### Current interpretation

Driver-side modifications were **not lost**. They are still present on disk.

## Current `mpu6050.h` State

[Drivers/I2C/mpu6050.h](Drivers/I2C/mpu6050.h) currently includes additional state fields added during this conversation:

- `nominalTwoKp`
- `nominalTwoKi`
- `startupBoostUntil`
- `runtimeGyroBiasX`
- `runtimeGyroBiasY`
- `runtimeGyroBiasZ`

## Current `i2c_hal_wrapper.c` State

[Drivers/I2C/i2c_hal_wrapper.c](Drivers/I2C/i2c_hal_wrapper.c) was changed earlier in the conversation from transmit/receive register access to STM32 HAL memory access:

- `HAL_I2C_Mem_Read`
- `HAL_I2C_Mem_Write`

This change was made because:

- I2C scan showed the MPU6050 device existed on the bus
- but register reads were failing with the original wrapper

## Important Debug History

### Earlier MPU6050 init failure

At one point the board repeatedly printed:

- `INIT_ERROR:1`

The investigation found:

1. Device existed on I2C bus
2. Address handling and startup timing mattered
3. The original I2C wrapper implementation for register access was not suitable
4. Cold-start timing affected MPU6050 bring-up

### Serial behavior changes over time

Temporary debug prints were added during investigation:

- `BOOT`
- `MPU_INIT ...`
- `I2C_SCAN ...`

Those were later removed from `main.c`, leaving only:

- normal 6-column data output
- `INIT_ERROR:<code>` in the failure path

## What the user observed during tuning

The user reported:

- `roll` looked mostly acceptable
- `pitch` seemed to show cumulative drift / slow return behavior
- the user wanted direct code changes rather than more planning

The last driver-side changes specifically targeted:

1. online gyro bias adaptation
2. continuous accel trust weighting

These changes are now present in `mpu6050.c`

## Known Limitations

- No local firmware toolchain was available in this session
  - no Keil compile verification
  - no real binary build confirmation
- All conclusions about current state were based on:
  - direct file reads
  - diffs against repository content
  - the user's reported board behavior

## Suggested Next Steps In A New Conversation

If a new conversation needs to continue from here, the most useful next actions are:

1. Verify actual build source selection
   - confirm Keil project is compiling this exact `Core/Src/main.c`
   - confirm no duplicate `main.c` or excluded file confusion
2. Verify actual build source selection for driver files
   - confirm Keil is compiling this exact `Drivers/I2C/mpu6050.c`
   - confirm the modified `i2c_hal_wrapper.c` is also in the build
3. If build inputs are confirmed correct, continue tuning MPU6050 behavior
   - inspect `pitch` drift with fresh serial logs
   - tune:
     - `MPU6050_GYRO_BIAS_ADAPT_ALPHA`
     - static detection thresholds
     - accel trust thresholds
     - Mahony `Kp/Ki`
4. If the board behavior still does not match current source, suspect:
   - stale build output
   - wrong target configuration in Keil
   - source file exclusion / duplication in project settings

## Quick Summary For Future Continuation

Use this summary in a new chat:

- `main.c` is already modified and includes MPU6050 + 6-column UART output + vertical projection.
- `mpu6050.c` is already modified and includes startup alignment, accel-based init, startup gain boost, continuous accel weighting, and online gyro bias adaptation.
- `i2c_hal_wrapper.c` was changed to use `HAL_I2C_Mem_Read/Write`.
- The remaining problem is likely not "main.c failed to change", but either:
  - runtime fusion tuning
  - or the build system not compiling the edited files.
