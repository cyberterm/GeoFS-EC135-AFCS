# GeoFS Eurocopter EC-135 Automatic Flight Control System (AFCS)

An authentic, complete **Automatic Flight Control System (AFCS)** and flight assistance suite for the **Eurocopter EC-135** (`ID 9`) in [GeoFS](https://www.geo-fs.com/).

Designed to eliminate twitchy stock handling, remove the exhausting need for constant forward stick pressure in cruise flight, and bring authentic Airbus Helicopters / Eurocopter avionics to GeoFS — from high-rate stability augmentation to hands-off cruising and precision auto-hovering.

---

## Quickstart

### 1. Installation

#### Primary Method (Recommended)

[![Install from Greasy Fork](https://img.shields.io/badge/Greasy_Fork-Install_AFCS_Suite-04aa6d?style=for-the-badge&logo=tampermonkey&logoColor=white)](https://greasyfork.org/en/scripts/597871-geofs-eurocopter-ec-135-afcs-suite)

**[Click here to Install via Greasy Fork](https://greasyfork.org/en/scripts/597871-geofs-eurocopter-ec-135-afcs-suite)**  
*(Requires [Tampermonkey](https://www.tampermonkey.net/) or [Violentmonkey](https://violentmonkey.github.io/). Provides 1-click installation and automatic background updates).*

#### Alternative Methods
* **Direct GitHub Raw:** If you prefer installing straight from GitHub, you can use the [GitHub Raw Script Link](https://raw.githubusercontent.com/cyberterm/GeoFS-EC135-AFCS/main/eurocopter_afcs.user.js).
* **Browser Console:** For restricted devices (Chromebooks/managed browsers) where extensions are blocked:
  1. Spawn in the EC-135 in GeoFS.
  2. Open Developer Tools (<kbd>F12</kbd> or right-click $\to$ **Inspect** $\to$ **Console**).
  3. Paste the contents of [`eurocopter_afcs.user.js`](eurocopter_afcs.user.js) and press <kbd>Enter</kbd>.

*(Individual standalone modular scripts are also preserved in the `standalone-scripts` branch for specialized use cases).*

---

### 2. Controls & Keybindings

You do not need to press any keys to enjoy realistic, stable flight. The core stability and pitch-trim systems **arm automatically on spawn** and work seamlessly in the background.

#### Primary Flight Modes
| Key | Mode | Status | When to Use |
| :---: | :--- | :---: | :--- |
| *(None)* | **Realistic Flight (SAS + A.TRIM)** | **Default** | **Normal flying:** Take off, push forward to cruise speed, and release the stick. The helicopter flies stable and trimmed hands-off. |
| <kbd>A</kbd> | **Cruise Autopilot** | Optional | **Long cross-country flights:** Locks barometric altitude and heading. Adjust your airspeed using the collective. |
| <kbd>G</kbd> | **Hover Assist** | Optional | **Helipad landings & hovering:** Hands-off self-leveling to 0° wings-level hover with automatic heading lock. |

#### Advanced System Diagnostics (Optional)
These keys are only needed if you want to disarm individual systems to test raw helicopter physics:
| Key | System Toggle | Description |
| :---: | :--- | :--- |
| <kbd>Z</kbd> | **A.TRIM Disarm** | Toggles pitch auto-trim off for direct mechanical pitch response without attitude retention. |
| <kbd>Caps Lock</kbd> | **SAS Disarm** | Toggles stability augmentation off to fly with raw, unaugmented aerodynamic twitchiness. |

---

## How to Fly with the AFCS

### 1. Normal Flight (Takeoff & Fast Cruise)
* **On Spawn:** Both **SAS** and **A.TRIM** arm automatically (replicating a real EC-135 cockpit startup).
* **Accelerating:** Pull collective to climb, then push the cyclic forward to pitch down and accelerate (e.g. $-8^\circ$ nose dip for 120 kts).
* **Hands-Off Cruise:** Release the cyclic. **A.TRIM** captures and holds that nose-down attitude hands-off. You can fly long distances at high speed without touching the pitch axis or holding continuous forward pressure.
* **Maneuvering:** Pushing or pulling the stick moves the cyclic smoothly. When you find a new desired pitch angle and release the stick, A.TRIM captures the new attitude automatically.

### 2. Precision Hovering & Landings (<kbd>G</kbd>)
* **Entering Hover:** As you approach a helipad, press <kbd>G</kbd>. A.TRIM yields and **Hover Assist** takes over.
* **Self-Leveling Cyclic:** Stick inputs command exact pitch and roll angles rather than angular rates. Centering the stick automatically brings the helicopter to a $0^\circ$ flat, wings-level hover.
* **Spot Turns & Heading Lock:** Deflecting the pedals rotates the tail cleanly on the spot. Releasing the pedals actively halts rotation and locks onto your current heading.
* **Departing:** Press <kbd>G</kbd> to disengage Hover Assist. The realistic stack seamlessly takes back over.

### 3. Long-Distance Cruise Autopilot (<kbd>A</kbd>)
* **Engaging:** At cruising altitude, press <kbd>A</kbd> (or click the cockpit autopilot button).
* **Altitude & Heading Hold:** The autopilot captures your current barometric altitude (rounded to the nearest 100 ft) and heading.
* **Speed Management:** Raising or lowering collective changes your airspeed. The autopilot automatically trims cyclic pitch to hold exact altitude as power changes.
* **Disengaging:** Press <kbd>A</kbd> again. The system performs a bumpless handover back to manual flight.

---

## In-Depth System Details

For pilots and developers interested in the avionics and mathematics behind the suite:

### 1. Stability Augmentation System (SAS)
Replicates the high-frequency series actuators (SEMAs) of the real Eurocopter EC-135:
* **Gyro Rate Damping:** Dampens pitch, roll, and yaw angular rates ($\text{deg/sec}$) calculated frame-rate independently ($\Delta t$), ensuring identical flight feel across 30, 60, and 144+ FPS.
* **Progressive Damping Blend on Yaw:** Hands-off pedals receive 100% gyro damping to cancel spin and weathercocking. When maneuvering ($|\text{pedal}| > 0.03$), damping fades smoothly to prevent control fighting or sluggish turns.
* **Collective-to-Yaw Mixing (Torque Anticipator):** A filtered feedforward compensator applies tail rotor bias during collective pulls, neutralizing the violent torque kick before yaw displacement can develop.

### 2. Pitch Auto-Trim (A.TRIM)
Replicates the electric parallel trim actuator of the EC-135:
* **Flapback Counter-Torque:** Natural rotor aerodynamics cause main rotor flapback, pitching the nose up as forward airspeed increases. A.TRIM builds and maintains the steady forward cyclic needed to counteract flapback hands-off.
* **Parallel Trim Migration:** Holding stick deflection forward or aft slews the trim datum progressively, eliminating residual stick force.
* **Bumpless Handover:** Releasing the stick smoothly transfers commanded cyclic into the trim register with zero cyclic drop or ballooning.

### 3. Hover Assist
A specialized low-speed control augmentation mode:
* **Proportional-Derivative Attitude Hold:** Maps cyclic deflection directly to target bank and pitch angles ($\pm 20^\circ$).
* **Two-Phase Heading Lock:** When pedals are released from a turn, the system applies dynamic gyro braking to bring rotation below $1.5^\circ/\text{sec}$, then captures and holds the settled heading with a critically damped PD loop.

### 4. Cruise Autopilot (AP)
A cascaded 3-loop flight director and autopilot:
* **Outer Altitude Loop:** Converts altitude error to target vertical speed ($\pm 500\text{ ft/min}$).
* **Middle V/S Loop:** Integrates vertical speed error to calculate the exact nose-down pitch angle required for the current collective setting.
* **Inner Pitch Loop:** Tracks target attitude and drives the swashplate cyclic with rate damping.
* **Heading Hold PID:** Proportional-Integral-Derivative tail rotor controller with anti-windup clamping.

---

## Compatibility & Architecture

* **Input Hardware:** 100% compatible with Mouse Flight, Keyboard controls, and USB Flight Sticks / Gamepads (HTML5 Gamepad API).
* **Frame-Rate Invariant:** All integral, derivative, and damping calculations are normalized against `performance.now()` $\Delta t$ delta time.
* **Clean Logging:** All status notifications are delivered cleanly to the browser Developer Console (`[EC-135 AFCS]`), leaving your cockpit view free of immersion-breaking HUD banners.
* **Fail-Safe Flight Loop:** The internal animation loop is isolated in a protected execution block, ensuring flight controls remain responsive under all conditions.

---

## Roadmap

### v1.1
- [ ] **Beep Trim:** Fine-tune target heading and altitude using hat-switch style key adjustments without disengaging AP.
- [ ] **Coordinated Turn Roll-In:** Smooth roll-into-turn banking mechanics for high-speed cruising rather than flat tail-rotor yawing.
- [ ] **Smart Ground Decoupling:** Automatic ground-idle state detection preventing trim accumulation and ensuring safe landings on sloped helipads.

### v1.2
- [ ] **NAV Mode:** Waypoint and flight plan route tracking.
- [ ] **Vertical Speed Selection (V.VEL):** Dial in specific climb or descent rates (e.g., -500 ft/min approach).
- [ ] **GeoFS AP Panel Integration:** Proper two-way integration with the native GeoFS autopilot control window.

### v2.0
- [ ] **4-Axis Autopilot Integration:** Full 4th-axis collective actuator automation with altitude hover and airspeed selection loop.
- [ ] **GPS Position Hold (True Auto-Hover):** Geographic ground-drift zeroing to lock the helicopter stationary over a helipad even in gusty crosswinds.
- [ ] **Authentic Audio Cues (Synthesized Web Audio):** Native zero-asset audio tones for AP disconnect chime, mode switches, and flight limit alerts.
- [ ] **Airbus CAD / Instrument Panel:** Authentic Caution & Advisory Display (CAD) status indicators and avionics strip.
- [ ] **Configurable Keybindings & Controls:** In-game binding customization for gamepads, flight sticks, and international keyboard layouts.

---

## License

This project is licensed under the [MIT License](LICENSE) - created by **cyberterm**. Free and open-source for the GeoFS flight simulation community.