# ArduPilot based Bisection Algorithm

A small Python project for flying an ArduPilot-based drone through a search pattern that progressively shrinks the search area using a **bisection algorithm**.


## What is in this repo?

- `section.py` — the main bisection mission script
- `square.py` — a simple square-flight script for basic motion testing

## What the code does

### `section.py`
This is the main mission script.

It:
- connects to the vehicle through MAVSDK
- arms and takes off
- flies to the corners of a user-defined polygon
- measures a score at each point
- picks the best edge based on the measurements
- shrinks the polygon toward that promising region
- repeats until the remaining area is small enough
- flies to the estimated optimum location
- lands if the mission completes successfully

Important detail:
- the current implementation uses a **simulated radio metric**, not live RF data from the vehicle or payload
- the initial polygon corners are **hard-coded** and must be changed before real deployment
- the script aborts and switches to **HOLD/LOITER** if movement is rejected or the mission times out

### `square.py`
This is a simpler helper script for validation.

It:
- arms and takes off
- enters Offboard mode
- flies a 100 m square in local NED coordinates
- exits Offboard mode
- triggers RTL at the end

This script is useful as a first motion test before trying the bisection mission.

## How the bisection logic works

At a high level, the mission works like this:

1. Start with a quadrilateral search area.
2. Measure the signal score at each corner.
3. Score each edge using the measurements at its two end points.
4. Keep the edge that looks most promising.
5. Shrink the polygon toward that region.
6. Re-measure the new polygon.
7. Repeat until the remaining area is small.
8. Move to the center of the final region.

This gives you a practical "search-and-refine" behavior without doing an exhaustive sweep of the whole area.

## Dependencies

Install Python packages first:

```bash
python3 -m pip install --upgrade pip
python3 -m pip install mavsdk numpy shapely
```

Recommended:
- Python 3.10+
- ArduPilot SITL for initial testing
- Mission Planner or QGroundControl for monitoring

## Recommended test order

Do not start with the real aircraft.

Use this order instead:

1. **Read the code and update coordinates**
2. **Run in SITL**
3. **Try `square.py` first**
4. **Then test `section.py` in SITL**
5. **Move to a real drone only after controlled validation**

## Running in simulation first

A safe way to validate the scripts is with ArduPilot SITL.

Typical flow:

1. Start ArduCopter SITL.
2. Make sure a MAVLink stream is available on UDP port `14540` or change the code accordingly.
3. Run one of the scripts:

```bash
python3 square.py
```

or

```bash
python3 section.py
```

If your MAVLink endpoint is different, update the connection string in the code:

```python
await drone.connect(system_address="udpin://0.0.0.0:14540")
```

## Real-world assumptions in the current code

Before using this on hardware, keep these project assumptions in mind:

- `section.py` uses **hard-coded GPS corners** near Boston by default
- the mission score is based on a **synthetic UE cluster and fake radio model**
- the code does **not** currently ingest real payload or modem measurements
- the scripts assume the flight controller accepts guided/goto style commands through MAVLink
- geofence behavior is treated as a mission stop condition

## Files worth editing before field use

### In `section.py`
You will want to change:
- `TARGET_AREA_M2`
- `MIN_AREA_STOP_M2`
- `DRONE_SPEED_MPS`
- `MEASURE_HOLD_S`
- `SHRINK_FACTOR`
- `initial_corners`
- `generate_ue_cluster(...)` logic if you replace the fake radio with real measurements
- `system_address` if your vehicle is not exposing MAVLink on `14540`

### In `square.py`
You will likely want to change:
- `target_alt`
- the 100 m waypoint square
- the MAVSDK connection string

## Suggested folder setup on your laptop

```text
ardupilot-wines-main/
├── README.md
├── section.py
└── square.py
```

Run from inside the project folder:

```bash
cd ardupilot-wines-main
python3 square.py
```

## Standard operating procedure (SOP) for bringing this onto an Inspired Flight IF800

These are practical, safety-first SOP notes for adapting this project to an **Inspired Flight IF800 Tomcat** running ArduPilot. Inspired Flight documents say the IF800 Tomcat uses ArduPilot on a **CubePilot Cube Blue H7**, and their setup workflow uses Mission Planner. citeturn925972search0turn925972search3turn925972search9

### 1) Start with paperwork, permissions, and a safe test plan
Before powering anything:
- fly only under your organization’s flight approval process
- use a qualified pilot in command
- confirm your airspace, geofence, and emergency procedures
- test this code first in SITL and then in a controlled outdoor test area
- remove propellers for bench setup and connection testing

### 2) Confirm the aircraft baseline is healthy
On the IF800 side:
- make sure the aircraft is fully assembled per Inspired Flight procedures
- verify batteries, payload, GNSS, RC link, and telemetry link are healthy
- confirm the autopilot firmware and parameters are the approved baseline for your aircraft
- if you need to update aircraft firmware, Inspired Flight provides a Mission Planner based process over USB, with batteries removed during the update procedure. citeturn925972search3

### 3) Connect in Mission Planner and verify the basics
Using Mission Planner on the laptop:
- connect to the IF800 flight controller
- verify GPS lock, EKF health, battery status, and radio link
- confirm the vehicle reports the expected ArduCopter frame and flight mode support
- review the full parameter set before changing anything

ArduPilot’s own docs recommend working through the first-time setup and first-flight/tuning process before operational use. citeturn925972search15

### 4) Verify mission-critical safety settings before running custom code
At minimum, confirm:
- return-to-launch behavior
- failsafe actions for RC loss / GCS loss / battery
- arming checks enabled
- geofence configured and tested
- takeoff altitude and speed limits appropriate for the site

ArduPilot supports both cylindrical and polygon inclusion/exclusion fences in Copter, and these are worth enabling before guided automation tests. citeturn925972search2turn925972search11turn925972search20

### 5) Bench-test the telemetry path with props off
This project relies on MAVLink reaching the laptop where Python/MAVSDK is running.

Bench-test this first:
- power the aircraft safely with props removed
- confirm the IF800 telemetry path reaches your laptop or companion computer
- make sure you can see heartbeats and position updates in your GCS
- verify that your Python environment can connect to the same MAVLink stream

The current code expects MAVSDK to receive the vehicle on:

```python
udpin://0.0.0.0:14540
```

So your field setup must expose MAVLink to that endpoint, or you need to change the connection string to match your actual transport. MAVSDK supports UDP and other transports depending on how the vehicle is connected. citeturn925972search1

### 6) Use a staged validation flow on the IF800
A good progression is:

**Stage A — bench only**
- confirm connection
- confirm telemetry
- do not arm props-on indoors

**Stage B — controlled hover / basic guided validation**
- use your normal GCS workflow first
- verify basic takeoff, hold, RTL, and landing behavior without custom automation

**Stage C — run `square.py` in a large, clear test area**
- start with a much smaller square than 100 m if needed
- keep altitude conservative
- keep manual override available at all times

**Stage D — run `section.py` only after you replace the default corners**
- set corners for your real test site
- make sure the entire polygon lies well inside the geofence
- confirm the script’s final optimum is also inside the safe flight area

### 7) Adapt the code for real IF800 deployment
Before real deployment, I would strongly recommend these project changes:
- replace the hard-coded Boston coordinates with your field coordinates
- replace the fake radio score with a real payload/modem measurement source
- add structured logging for each waypoint, score, and mode change
- add explicit pre-arm validation in code
- add a dry-run mode that prints waypoints without flying
- reduce takeoff altitude and mission size for first trials

### 8) Post-flight SOP
After each test flight:
- disarm and secure the aircraft
- download flight logs from Mission Planner
- review GPS, EKF, battery, mode changes, fence events, and failsafes
- compare actual path vs intended path
- only expand the test envelope after a clean review

Mission Planner supports downloading ArduPilot dataflash logs for post-flight analysis. citeturn925972search14

## Minimum field checklist

Use this as a quick pre-flight reminder.

### Code checklist
- [ ] Correct MAVLink endpoint configured
- [ ] Correct takeoff altitude configured
- [ ] Safe `initial_corners` entered
- [ ] Polygon fully inside test area
- [ ] Fake radio model replaced or intentionally accepted for demo
- [ ] Script tested in SITL

### Aircraft checklist
- [ ] IF800 firmware baseline verified
- [ ] Batteries healthy and secured
- [ ] GPS/EKF healthy
- [ ] Geofence enabled
- [ ] RTL tested in standard workflow
- [ ] RC/manual override available
- [ ] Test area clear

## Limitations

This repository is a mission prototype, not a finished field deployment package.

Known limitations:
- no live RF sensor integration yet
- hard-coded coordinates
- limited logging
- minimal configuration management
- no mission replay or plotting utilities
- no safety supervisor process outside the script itself

## Good next improvements

If you want to keep building this project, the most useful next steps are:

1. connect the score function to a real modem / payload measurement stream
2. move all mission parameters into a config file
3. add CSV logging and map plotting
4. add waypoint preview before arming
5. support different polygon shapes and stop conditions
6. add proper field-deployment documentation for the IF800 telemetry path

## Disclaimer

This project can command a real aircraft. Use it only in a controlled and legally compliant environment with appropriate pilot oversight. Do not treat the current code as production-ready autonomy.
