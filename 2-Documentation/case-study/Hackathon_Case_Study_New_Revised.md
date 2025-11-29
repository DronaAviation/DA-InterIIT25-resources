# Hackathon Case Study: Integrating PAW3903 Optic Flow & VL53L1X Ranging Sensors into Pluto Drone (MagisV2 Firmware)

## 1\. Introduction
This case study guides student teams through integrating two key perception sensors, the **PixArt PAW3903 optical flow sensor** and the **ST VL53L1X Time‑of‑Flight distance sensor,** into the **Pluto drone** using the **MagisV2 firmware repository**. The goal is to enable stable low‑altitude position hold and enhanced navigation.
Students will be provided with:
*   `paw3903_opticflow.h / paw3903_opticflow.cpp`
*   `ranging_vl53l1x.h / ranging_vl53l1x.cpp`
These contain basic drivers and low‑level I/O functions to communicate with and fetch data from the sensors.
* * *
## 2\. Scope of the Case Study
This case study focuses on:
*   Understanding how the optic flow and ranging sensors work.
*   Integrating both sensors into MagisV2’s firmware architecture.
*   Updating existing modules (`flight/opticflow` and `flight/altitudehold`) to use the new data.
*   Ensuring the build system (Makefile) compiles any new files added by the students.
**Not included in scope:**
*   PID tuning beyond basic stabilization
*   Full navigation stack or SLAM
*   Advanced multi‑sensor fusion
* * *
## 3\. System Architecture Overview
### 3.1 Sensors
**PAW3903 Optic Flow Sensor**
*   Provides 2D displacement (`deltaX`, `deltaY`) and quality metrics.
*   Communicates via SPI.
*   Useful for horizontal position estimation during hover.
**VL53L1X Ranging Sensor**
*   Provides distance/altitude measurement.
*   Communicates via I2C.
*   Useful for altitude hold and scaling optic flow velocities.
### 3.2 Firmware Integration Points
*   `/drivers/paw3903_opticflow.cpp` → low‑level SPI communication.
*   `/drivers/ranging_vl53l1x.cpp` → low‑level I2C communication.
*   `/flight/opticflow.cpp` → combines optic flow + gyro for velocity estimation.
*   `/flight/altitudehold.cpp` → uses VL53L1X + barometer for altitude hold.
* * *
## 4\. What Students Can Use
Students can use the following firmware components:
### 4.1 Provided Sensor Driver Files
*   **Optic Flow:**
    *   `paw3903_opticflow.h`
    *   `paw3903_opticflow.cpp`
*   **Ranging:**
    *   `ranging_vl53l1x.h`
    *   `ranging_vl53l1x.cpp`
### 4.2 Existing Flight Modules
*   `/flight/opticflow.*` → currently contains partial implementation.
*   `/flight/altitudehold.*` → includes basic structure but not fully implemented.
* * *
## 5\. Available APIs for Integration
### 5.1 Optic Flow Driver API
The PAW3903 optical flow support is implemented as a set of **C-style driver functions**, not a C++ class. There is **no global object** to create or use. Students must call the existing functions declared in:

```cpp
paw3903_opticflow.h
paw3903_opticflow.cpp
```

Use only these functions; do **not** re‑implement low-level SPI or register handling.
* * *
## PAW3903 Core Setup Functions
### `bool paw3903_init ( void )`
High-level initialization helper. Internally sets up SPI and checks IDs. Use this as your main init entry point.

```cpp
if (!paw3903_init()) {
    failureFlag |= ( 1 << FAILURE_PAW3903 );
}
```

* * *
## Mode & Configuration
### `bool paw3903_set_resolution ( PAW3903_ResolutionCPI_t res)`
Sets motion resolution in CPI (counts per inch). Effective CPI = reg\_value \* 50.
*   Use lower CPI (≈1200–1600) for drone navigation (less noise, longer range).

```scss
paw3903_set_resolution(1600);   // example
```

### `PAW3903_ResolutionCPI_t paw3903_get_resolution ( void )`
Reads back the configured resolution.
* * *
## Power & Orientation
### `bool paw3903_power_up_reset ( void )`
Performs a power-up reset sequence on the sensor.
### `bool paw3903_shutdown ( void )`
Puts the sensor into shutdown mode. Typically not needed during flight, but useful for power saving.
### `bool paw3903_set_orientation ( PAW3903_Orientation_t orient )`

```cpp
paw3903_set_orientation ( PAW3903_ORIENT_NORMAL );
```

The PAW3903 supports multiple axis configurations depending on how it is mounted on the drone.

```cpp
typedef enum {
  PAW3903_ORIENT_NORMAL   = 0x00, /**< Default orientation (no swap, no invert) */
  PAW3903_ORIENT_INVERT_X = 0x20, /**< Invert X-axis (bit 5 = 1) */
  PAW3903_ORIENT_INVERT_Y = 0x10, /**< Invert Y-axis (bit 4 = 1) */
  PAW3903_ORIENT_SWAP_XY  = 0x08, /**< Swap X and Y axes (bit 3 = 1) */

  /* Common combinations */
  PAW3903_ORIENT_INVERT_XY      = 0x30, /**< Invert both X and Y axes */
  PAW3903_ORIENT_SWAP_INVERT_X  = 0x28, /**< Swap X/Y, invert X only */
  PAW3903_ORIENT_SWAP_INVERT_Y  = 0x18, /**< Swap X/Y, invert Y only */
  PAW3903_ORIENT_SWAP_INVERT_XY = 0x38  /**< Swap and invert both axes */
} PAW3903_Orientation_t;
```

### `PAW3903_Orientation_t paw3903_get_orientation ( void )`
Configures or reads the orientation setting so that X/Y motion lines up with the drone body frame.
* * *
## Reading Motion & Quality
### PAW3903\_Data structure
Motion and status are returned via a `PAW3903_Data` struct (defined in your header), typically containing:
*   `int16_t deltaX` – pixel deltas
*   `init16_t deltaY` – pixel deltas
*   `uint8_t squal` – surface quality
*   `uint16_t shutter` – exposure
*   `uint8_t observation` – status bits
* * *
## Burst Motion Read
### `bool paw3903_read_motion_burst ( PAW3903_Data &out )`
Performs a burst read of all motion data (motion flag, dx/dy, SQUAL, shutter, observation) in one SPI transaction. All fields come from the same frame, which is preferred for flight control.

```cpp
PAW3903_Data of;
if (paw3903_read_motion_burst(of)) {
    // use of.deltaX, of.deltaY, of.squal, of.shutter, of.observation
}
```

Use this in your high-rate loop (e.g., 100–200 Hz) instead of reading individual registers.
* * *
## How Students Should Integrate (Optic Flow)

1. **Initialization (once at startup in main.cpp, uncomment the line 436 - 438)**

```cpp
// if ( ! XVision.init ( VL53L1_DISTANCEMODE_LONG ) ) {
//   failureFlag |= ( 1 << FAILURE_VL53L1X );
// }

// Optional Configuration
paw3903_set_resolution(1600);      // example CPI
paw3903_set_orientation(PAW3903_ORIENT_DEFAULT); // example Orientation
```

1. **Periodic motion read (100–200 Hz)**

```cpp
PAW3903_Data of;
if (paw3903_read_motion_burst(of)) {
    // convert of.dx, of.dy → velocity
}
```

1. **Convert dx/dy to body-frame velocities** using altitude from `XVision.getLaserRange()` and loop `dt`.
2. **Handle poor tracking** by checking `of.squal` and/or `of.observation` and discarding data when below a threshold.
3. **Feed computed velocities into** **`flight/opticflow.cpp`**, which then interfaces with the rest of the flight controller.
* * *
### 5.2 Ranging (VL53L1X) Driver API
Ranging (VL53L1X) Driver API
The VL53L1X ranging functionality is provided through the `LaserSensor_L1` class. **Do not create a new object** — the instance is already declared globally:

```cpp
extern LaserSensor_L1 XVision;
```

Use this existing `XVision` instance directly in your implementation.
Below is the full description of the functions available in the class and how students should use them.
* * *
## **LaserSensor\_L1 Class Overview**
Before using the class, note that the VL53L1X sensor supports multiple **distance modes**, defined as:

```cpp
/** @defgroup VL53L1_define_DistanceModes_group Defines Distance modes
 *  Defines all possible Distance modes for the device
 *  @{
 */
typedef uint8_t VL53L1_DistanceModes;

#define VL53L1_DISTANCEMODE_SHORT   ((VL53L1_DistanceModes)1)
#define VL53L1_DISTANCEMODE_MEDIUM  ((VL53L1_DistanceModes)2)
#define VL53L1_DISTANCEMODE_LONG    ((VL53L1_DistanceModes)3)
```

**Recommended:** Use `VL53L1_DISTANCEMODE_LONG` for drone altitude sensing.
* * *
## **LaserSensor\_L1 Class Overview**\*\*
The class is structured as follows:

```cpp
class LaserSensor_L1 {
  VL53L1_RangingMeasurementData_t _RangingMeasurementData_L1x;
  VL53L1_Dev_t MyDevice_L1x;
  int16_t _range;

 public:
  LaserSensor_L1() : _Global_Status_L1x(VL53L1_ERROR_NONE), Range_Status_L1x(0), _range(0) {}

  bool init(VL53L1_DistanceModes _DistanceMode);
  void setAddress(uint8_t address);
  bool startRanging();
  int16_t getLaserRange();

 private:
  VL53L1_Error _Global_Status_L1x;
  uint8_t Range_Status_L1x;
};

extern LaserSensor_L1 XVision;
```

* * *
## **Function Descriptions & How to Use Them**
### **`bool init ( VL53L1_DistanceModes _DistanceMode)`**
Initializes the VL53L1X sensor.
*   Configures communication parameters
*   Waits for device boot
*   Applies the selected distance mode

**Usage:**

Initialization (once at start-up in **main.cpp**, uncomment the line  441 - 443)
```cs
// if ( ! XVision.init(VL53L1_DISTANCEMODE_MEDIUM) ) {
//  failureFlag |= ( 1 << FAILURE_PAW3903 );
// }
```

* * *
### **`void setAddress ( uint8_t address)`**
Sets a new I2C address for the sensor. Usually not needed unless using multiple VL53L1X sensors.
**Usage (optional):**

```scss
XVision.setAddress(0x30);
```

* * *
### **`bool startRanging ( void )`**
Starts the measurement cycle and updates the internal range data.
*   Returns `true` only if valid data is available.
*   Filters out invalid or out-of-range values (>4500 mm).
**Usage inside periodic loop:**

```cpp
if (XVision.startRanging()) {
    int16_t distance = XVision.getLaserRange();
}
```

This replaces the typical `isDataReady()` + `readDistanceMM()` flow — the class already handles those internally.
* * *
### **`int16_t getLaserRange (void)`**
Returns the last measured valid range in millimeters.
**Usage:**

```cpp
int16_t altitude_mm = XVision.getLaserRange();
```

You should only call this after `startRanging()` returns `true`.
* * *
## **How Students Should Integrate (Using Existing Instance)**
*   Use the already-created `XVision` instance.
*   Initialize once during boot: `XVision.init(...)`.
*   In a scheduler loop (50–100 Hz), call `startRanging()`.
*   If it returns true, fetch altitude using `getLaserRange()`.
*   Feed the altitude measurement to `altitudehold.cpp`.
*   **Do not modify the LaserSensor\_L1 driver.**
*   **Do not instantiate a new LaserSensor\_L1 object.**
* * *
### 5.3 MagisV2 Sensor Fusion Points MagisV2 Sensor Fusion Points MagisV2 Sensor Fusion Points MagisV2 Sensor Fusion Points
Students should update:
*   `opticflow.cpp` to compute: `velocityX`, `velocityY`.
*   `altitudehold.cpp` to compute: `target_throttle` corrections.

### 5.4 Timing & Delay Utilities (drivers/system.c)
These functions provide low‑level timing controls used across drivers, sensor readouts, and control loops. Students may need them when writing custom drivers or adding delays required by sensor datasheets.
#### **Available Functions**

```cpp
void delayMicroseconds(uint32_t us);
void delay(uint32_t ms);

uint32_t micros(void);
uint32_t millis(void);
```

#### **Function Descriptions**
*   **delayMicroseconds(us)**
Pauses execution for the specified number of microseconds.
Useful for precise timing between SPI/I2C operations or sensor reset sequences.
*   **delay(ms)**
Pauses execution for the given number of milliseconds.
Typically used for initialization delays required by some sensors.
*   **micros()**
Returns the number of microseconds elapsed since system startup.
Commonly used for performance measurement or loop timing.
*   **millis()**
Returns the number of milliseconds elapsed since system startup.
Often used to schedule periodic tasks or implement timeout logic.

### 5.5 Debugging
The Monitor debug functions provide a convenient way to output debug information from your PlutoX2 drone firmware. These functions are part of the MagisV2 flight controller software and allow real-time monitoring of variables and system states.
#### **Available Functions**

Monitor_Print - Outputs debug information without adding a newline character.

```cpp
void Monitor_Print(const char *msg)
void Monitor_Print(const char *tag, int number)
void Monitor_Print(const char *tag, double number, uint8_t precision)
```
#### Parameters

- `msg`: String message to print
- `tag`: Label/identifier for the data being printed
- `number`: Integer or double value to print
- `precision`: Number of decimal places for double values (0-7, constrained automatically)

Monitor_Println - Outputs debug information followed by a newline character.

```cpp
void Monitor_Println(const char *msg)
void Monitor_Println(const char *tag, int number)
void Monitor_Println(const char *tag, double number, uint8_t precision)
```

#### Parameters

Same as Monitor_Print functions.

#### Basic Usage Examples

##### Simple Text Output

```cpp
// Print a simple message
Monitor_Print("System initialized");
Monitor_Println("Ready for flight");
```

##### Variable Monitoring

```cpp
int batteryLevel = 85;
double altitude = 12.5;
double temperature = 23.456;

// Print integer values
Monitor_Print("Battery: ", batteryLevel);
Monitor_Println("Voltage: ", voltage);

// Print double values with precision
Monitor_Print("Alt: ", altitude, 2);        // Shows: Alt: 12.50
Monitor_Println("Temp: ", temperature, 1);  // Shows: Temp: 23.5
```

##### Graph Format Printing Guide

To create graph-like visualizations using the monitor functions, you can output data in structured formats that can be parsed by external tools or displayed in a meaningful way.

##### Time Series Data Format

For plotting sensor data over time, use a consistent format:

The function will only work if you use "GRAPH:"

```cpp
    Monitor_Println("GRAPH:", sensor);
```

How to Start Monitoring the Data
1. Connect your drone via Wi-Fi
2. In the PlutoIDE Extension, click on “Pluto Monitor” to start live monitoring
3. Once Pluto Monitor is running, open Teleplot
4. Teleplot will automatically detect and display all incoming "GRAPH:" data streams.
5. Ensure your firmware prints data using the correct Monitor_Println("GRAPH:", ...) format
* * *
## 6\. Required Changes to Makefile
If students create new `.cpp` files, they **must** add them to the correct section of the Makefile so they are compiled.
Below are the three relevant lists from the Makefile:
### **DRONA\_FLIGHT**
For flight‑logic related code such as control loops, estimators, sensor fusion, etc.

```plain
DRONA_FLIGHT = \
    flight/acrobats.cpp \
    flight/posControl.cpp \
    flight/posEstimate.cpp \
    flight/opticflow.cpp
```

➡ **If students create a new flight‑related** **`.cpp`** **file, they must insert it directly into this list**, for example:

```plain
DRONA_FLIGHT = \
    flight/acrobats.cpp \
    flight/posControl.cpp \
    flight/posEstimate.cpp \
    flight/opticflow.cpp \
    flight/my_new_velocity_filter.cpp
```

**Do NOT use** **`+=`** **— append the file inside the existing block.**\*\*
* * *
### **DRONA\_DRIVERS**
For hardware drivers such as sensors, electronics interfaces, and low‑level peripherals.

```markdown
DRONA_DRIVERS = \
    drivers/opticflow_paw3903.cpp \
    drivers/paw3903_opticflow.cpp \
    drivers/display_ug2864hsweg01 \
    drivers/ranging_vl53l0x.cpp \
    drivers/ranging_vl53l1x.cpp \
    drivers/sc18is602b.cpp \
    drivers/bridge_sc18is602b.cpp
```

➡ **Add new driver** **`.cpp`** **files directly to this list**, like:

```markdown
DRONA_DRIVERS = \
    drivers/opticflow_paw3903.cpp \
    drivers/paw3903_opticflow.cpp \
    drivers/display_ug2864hsweg01 \
    drivers/ranging_vl53l0x.cpp \
    drivers/ranging_vl53l1x.cpp \
    drivers/sc18is602b.cpp \
    drivers/bridge_sc18is602b.cpp \
    drivers/my_custom_barometer.cpp
```

**Do NOT use** **`+=`** **— place the new file inside the existing block.**\*\*
* * *
### **DRONA\_COMMAND**
For command‑processing or high‑level behavior logic.

```bash
DRONA_COMMAND = \
    command/command.cpp \
    command/localisationCommand.cpp
```

➡ **Add new command** **`.cpp`** **files directly inside this block**, for example:

```bash
DRONA_COMMAND = \
    command/command.cpp \
    command/localisationCommand.cpp \
    command/my_new_command_module.cpp
```

**Do NOT use** **`+=`** **— always insert the new file directly into the existing list.**\*\*
* * *
### **Important Notes**
*   Only **`.cpp`** files go into these lists, header files (`.h`) do **not** need to be added.
*   If the student adds a new folder, they must include the relative path correctly.
*   Incorrect placement will cause linking errors or missing functionality.
*   After editing the Makefile, students must re‑compile to verify it builds successfully.
* * *
## 7\. System Execution Flow (How Everything Runs Internally)
Below is a clear breakdown of how the Pluto firmware loop works and how **Optic Flow (PAW3903)** and **Ranging (VL53L1X)** integrate into it.
### **7.1 High-Level Main Loop Flow (mw.cpp)**
Every cycle, the firmware performs:
1. **Read & process RC input**
    *   `updateRx()` collects radio data.
    *   If needed, `processRx()` updates modes, arming state, failsafe, commands.
2. **If not processing RC → run background sensor tasks**
    *   `executePeriodicTasks()` updates sensors on a rotating schedule.
    *   Includes Optic Flow and Laser ToF tasks.
3. **Main flight-control loop** (runs at the configured looptime)
    *   `imuUpdate()` → updates gyro/accel and attitude
    *   `annexCode()` → processes RC into control commands
    *   `userCode()` → runs PlutoPilot / DevMode code
    *   Altitude & flow controllers update: `applyAltHold()`, `runFlowHold()`
    *   `pid_controller()` computes flight corrections
    *   `mixTable()` + `writeMotors()` drives ESC outputs
4. **Telemetry, LEDs, status updates** at end of loop
* * *
### **7.2 Periodic Tasks: Where Optic Flow & Ranging Run**
`executePeriodicTasks()` cycles through tasks such as:
*   `UPDATE_OPTICFLOW` → updates optic flow data
    *   `updateSpiOpticFlow()` calls `paw3903_read_motion_burst()`
    *   `runFlowHold()` computes velocity/drift
*   `UPDATE_LASER_TOF_TASK` → updates VL53L1X laser range
    *   `getRange_L1()` uses the global `XVision` instance
    *   Produces altitude for flow scaling + altitude hold
These tasks run **whenever RX processing is NOT happening**, ensuring sensors update consistently.
* * *
### **7.3 Sensor Data Flow**
#### **Optic Flow (PAW3903)**
1. Initialization: `paw3903_init()`, set mode/resolution/orientation
2. Periodic reads: `paw3903_read_motion_burst(PAW3903_Data &out)`
3. Motion → velocity: handled in `runFlowHold()`
4. Output goes into `posControl` / drift compensation
#### **Ranging (VL53L1X)**
1. Initialization: `XVision.init(VL53L1_DISTANCEMODE_LONG)`
2. Periodic ranging: `XVision.startRanging()` + `XVision.getLaserRange()`
3. Altitude is fused in altitude estimator + used to scale optic flow
* * *
### **7.4 ASCII Diagram of the Complete Data Flow**

```sql
          ┌────────────────────┐
          │        loop()      │
          └───────┬────────────┘
                  │
         ┌────────▼────────┐
         │ updateRx(...)   │
         └────────┬────────┘
      shouldProcessRx? 
          ┌───────┴───────────┐
          │ YES               │ NO
          ▼                   ▼
   ┌──────────────┐   ┌───────────────────┐
   │  processRx() │   │ executePeriodic   │
   │ (modes,      │   │ Tasks()           │
   │  failsafe…)  │   │  - OPTICFLOW      │
   └───────┬──────┘   │    • updateSpi    │
           │          │      OpticFlow()  │
           │          │    • runFlowHold()│
           │          │  - LASER_TOF      │
           │          │    • getRange_L1()│
           │          └─────────┬─────────┘
           │                    │
           └────────────┬───────┘
                        ▼
                 ┌──────────────┐
                 │  IMU update  │
                 │ imuUpdate()  │
                 └──────┬───────┘
                        ▼
                 ┌──────────────┐
                 │ annexCode()  │
                 │(RC→commands) │
                 └──────┬───────┘
                        ▼
                 ┌──────────────┐
                 │ userCode()   │
                 └──────┬───────┘
                        ▼
                 ┌──────────────┐
                 │ Alt/Flow Hold│
                 │ applyAltHold │
                 │ runFlowHold  │
                 └──────┬───────┘
                        ▼
                 ┌──────────────┐
                 │ PID + Mixer  │
                 │ pid_ctrl     │
                 │ writeMotors()│
                 └──────────────┘
```

* * *
## 8\. Tasks for Hackathon Teams
### Task A: Enable PAW3903 Sensor
*   Initialize SPI driver.
*   Read motion data at 100–200 Hz.
*   Print debug values via serial.
### Task B: Enable VL53L1X Ranging
*   Initialize I2C sensor.
*   Measure altitude.
*   Validate noise & range.
### Task C: Modify `/flight/opticflow.cpp`
*   Convert `deltaX/deltaY` → horizontal velocity.
*   Scale with altitude from VL53L1X.
### Task D: Modify `/flight/altitudehold.cpp`
*   Use VL53L1X altitude to implement a simple PID loop.
### Task E: Integration + Demo
*   Drone should hover stably at 0.5–1.5m.
*   Demonstrate drift compensation using optic flow.
* * *
## 8\. Expected Deliverables
*   Firmware code modifications.
*   README explaining implementation.
*   Flight demo video.
* * *
## 9\. Evaluation Criteria
*   Correct integration of both sensors.
*   Stability during hover.
*   Code quality and architecture.
*   Innovation (visualization, debug tools, tuning scripts, etc.).
* * *
## 10\. References
*   MagisV2 Firmware Docs (internal)
*   VL53L1X Datasheet
*   PAW3903 Datasheet
* * *
This document may be expanded once screenshots, Makefile details, and API descriptions are added.

---