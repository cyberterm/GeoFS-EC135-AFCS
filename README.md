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
| Key / UI Button | Mode | Status | When to Use |
| :---: | :--- | :---: | :--- |
| *(None)* | **Realistic Flight (SAS + A.TRIM)** | **Default** | **Normal flying:** Take off, push forward to cruise speed, and release the stick. The helicopter flies stable and trimmed hands-off. |
| <kbd>A</kbd> / `AUTOPILOT` | **Cruise Autopilot (`HDG` / `NAV`)** | Optional | **Long cross-country flights:** Locks barometric altitude, vertical speed, and heading or GPS/radio navigation route. Adjust airspeed using the collective. |
| <kbd>G</kbd> / `HOVER` | **Hover Assist** | Optional | **Helipad landings & hovering:** Hands-off self-leveling to $0^\circ$ wings-level hover with automatic heading lock. |

#### Autopilot Panel & In-Flight Adjustments (Beep Trim)
When **Cruise Autopilot (<kbd>A</kbd>)** is engaged, the native GeoFS autopilot drawer unfolds with full two-way synchronization (`HDG`/`NAV`, `ALT`, and `VERT SPEED`), and you can also use the arrow keys for hands-on Beep Trim:
| Key / Control | Adjustment | Fast Step (<kbd>Shift</kbd> + Key) | Cockpit & UI Feedback |
| :---: | :--- | :--- | :--- |
| <kbd>▲ Up</kbd> / <kbd>▼ Down</kbd> | Altitude $\pm 100\text{ ft}$ | $\pm 500\text{ ft}$ fast climb/descent | Syncs `ALT` box, PFD bug & auto-assigns $\pm 700\text{ fpm}$ `VERT SPEED` |
| <kbd>◄ Left</kbd> / <kbd>► Right</kbd> | Heading $\pm 1^\circ$ | $\pm 5^\circ$ course turn | Syncs `HDG` box & HSI bug (reverts `NAV` $\to$ `HDG`) |
| `HDG` / `NAV` Switch | Lateral Mode Toggle | — | Switches between Heading Hold and GPS / Radio Navigation |
| `VERT SPEED` Input | Custom Climb/Descent Rate | — | Command any climb/descent rate ($\text{ft/min}$); auto-zeros on altitude capture |

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
* **Hands-Off Cruise:** Release the cyclic. **A.TRIM** smoothly captures and holds that nose-down attitude hands-off. You can fly long distances at high speed without touching the pitch axis or holding continuous forward pressure.
* **Maneuvering:** Pushing or pulling the stick moves the cyclic smoothly. With our **Settle-Then-Latch** logic, releasing after aggressive pitch maneuvers lets the nose coast naturally to a halt and lock into its new resting attitude — **zero rubber-banding or snapback**.

### 2. Precision Hovering & Landings (<kbd>G</kbd> or `HOVER` Pad)
* **Entering Hover:** As you approach a helipad, press <kbd>G</kbd> or click the **HOVER** button next to `AUTOPILOT` on the bottom bar. A.TRIM yields and **Hover Assist** takes over.
* **Self-Leveling Cyclic:** Stick inputs command exact pitch and roll angles rather than angular rates. Centering the stick automatically brings the helicopter to a $0^\circ$ flat, wings-level hover.
* **Spot Turns & Heading Lock:** Deflecting the pedals rotates the tail cleanly on the spot. Releasing the pedals actively halts rotation and locks onto your current heading.
* **Departing:** Press <kbd>G</kbd> (or click **HOVER**) to disengage Hover Assist. The realistic stack seamlessly takes back over.

### 3. Long-Distance Cruise Autopilot (<kbd>A</kbd> or `AUTOPILOT` Pad)
* **Engaging:** At cruising altitude, press <kbd>A</kbd> or click the **AUTOPILOT** button. The native fixed-wing airspeed (`KTS`) box is automatically hidden because helicopter airspeed is governed by the collective lever.
* **Altitude & Vertical Speed (`VERT SPEED`):** Captures your current barometric altitude (rounded to the nearest $100\text{ ft}$). Dialing in a new target altitude (via UI or arrow keys) automatically schedules a $\pm 700\text{ fpm}$ climb/descent rate, or you can enter any custom vertical speed in the `VERT SPEED` box. As you approach within $100\text{ ft}$ of your target altitude, the autopilot smoothly flares into Altitude Hold and resets `VERT SPEED` to `0`.
* **Navigation (`NAV`) Mode & `OBS` Course Intercept:**
  * Click **NAV** to track your active GeoFS Flight Plan route (`FPL`) or tuned radio navaid (`VOR` / `ILS` / GPS fix).
  * **Waypoint Altitude Capture:** If a flight plan waypoint has a designated altitude, `NAV` mode automatically updates your target altitude upon engaging and as you sequence through each leg.
  * **`TRK` (Direct-To) vs. Manual `OBS`:** By default, GPS waypoints use `TRK` (Direct-To bearing) and automatically reset `OBS` to `TRK` whenever a new waypoint is selected or sequenced. In strong crosswinds, you can dial a specific `OBS` radial on the cockpit HSI/NAV unit to fly a crabbed, zero-drift course line with up to a $45^\circ$ intercept angle.
* **Coordinated Banking:** In cruise, turns roll in up to $20^\circ$ of bank with cyclic roll rather than dragging the fuselage flat with tail rotor yaw.
* **Disengaging & Fly-Through Override:** Press <kbd>A</kbd>, click **AUTOPILOT**, or simply deflect the cyclic/pedals past the manual override threshold to disconnect the autopilot with a bumpless handover back to manual flight.
* **Touchdown Safety:** If you land with AP active, the system automatically disengages on ground contact and flushes all integrators to ensure a clean, stable subsequent takeoff.

---

## In-Depth System Details

For pilots and developers interested in the avionics and mathematics behind the suite:

### 1. Stability Augmentation System (SAS)
Replicates the high-frequency series actuators (SEMAs) of the real Eurocopter EC-135:
* **Gyro Rate Damping:** Dampens pitch, roll, and yaw angular rates ($\text{deg/sec}$) calculated frame-rate independently ($\Delta t$), ensuring identical flight feel across 30, 60, and 144+ FPS.
* **Progressive Damping Blend on Yaw:** Hands-off pedals receive 100% gyro damping to cancel spin and weathercocking. When maneuvering ($|\text{pedal}| > 0.03$), damping fades smoothly to prevent control fighting or sluggish turns.
* **Collective-to-Yaw Mixing (Torque Anticipator):** A filtered feedforward compensator applies tail rotor bias during collective pulls, neutralizing violent torque kick before yaw displacement can develop.

### 2. Pitch Auto-Trim (A.TRIM)
Replicates the electric parallel trim actuator of the EC-135 with authentic aerospace control laws:
* **Flapback Counter-Torque:** Natural rotor aerodynamics cause main rotor flapback, pitching the nose up as forward airspeed increases. A.TRIM builds and maintains the steady forward cyclic needed to counteract flapback hands-off.
* **Settle-Then-Latch Attitude Capture:** When releasing the cyclic from an aggressive pitch maneuver, A.TRIM allows the nose to coast naturally while angular rate subsides ($|\dot{\theta}| < 1.5^\circ/\text{sec}$), latching the resting attitude with zero snapback or rubber-banding.

### 3. Hover Assist
A specialized low-speed control augmentation mode:
* **Proportional-Derivative Attitude Hold:** Maps cyclic deflection directly to target bank and pitch angles ($\pm 20^\circ$).
* **Two-Phase Heading Lock:** When pedals are released from a turn, the system applies dynamic gyro braking to bring rotation below $1.5^\circ/\text{sec}$, then captures and holds the settled heading with a critically damped PD loop.

### 4. Cruise Autopilot (AP) & Navigation Director
A standalone, cascaded 3-loop helicopter flight director that completely bypasses GeoFS's fixed-wing autopilot engine (`geofs.autopilot.on` remains permanently `false` so GeoFS never hijacks pilot controls):
* **Outer Altitude & Commanded V/S Loop:** Tracks pilot-commanded `VERT SPEED` with proportional flare capture (`alt_Kp = 3.0`) into target altitude, or holds altitude within $\pm 750\text{ ft/min}$ when `VERT SPEED` is `0`.
* **Two-Stage V/S Low-Pass Filter & Deviation-Scaled PID:** Filters vertical speed and vertical acceleration (`vs_Kd`) so short wind gusts are absorbed naturally by SAS and A.TRIM while large V/S deviations (such as rapid collective changes) receive full proportional and derivative braking authority within realistic pitch limits ($-10^\circ$ climb to $+12^\circ$ cruise dip).
* **Slew-Rate Limiter:** Limits flight director pitch commands to $3.5^\circ/\text{sec}$, simulating real electric trim jacks and eliminating sudden cyclic jerks.
* **Lateral `HDG` & `NAV` Director:** Computes great-circle bearings (`TRK`) or `OBS` radial intercept headings ($\text{OBS} + \text{clamp}(10 \times \Delta\text{course}, -45^\circ, +45^\circ)$) across Flight Plan waypoints and tuned radio navaids.
* **Speed-Scheduled Coordinated Turning:** Airspeed-scheduled turn director that rolls up to $20^\circ$ of bank with cyclic roll above $60\text{ kts}$ while fading pedal yaw to pure turn coordination.
* **Ground Decoupling & Fly-Through Disconnect:** Auto-disengages on touchdown or manual control deflection and flushes all integrators to prevent ground resonance or post-touchdown ballooning.

---

## Compatibility & Architecture

* **Input Hardware:** 100% compatible with Mouse Flight, Keyboard controls, and USB Flight Sticks / Gamepads (HTML5 Gamepad API).
* **Frame-Rate Invariant:** All integral, derivative, and damping calculations are normalized against `performance.now()` $\Delta t$ delta time.
* **Clean Logging:** Status notifications are delivered cleanly via an auto-fading HUD pill banner and browser Developer Console (`[EC-135 AFCS]`), leaving your cockpit view free of clutter.
* **Fail-Safe Flight Loop:** The internal animation loop is isolated in a protected execution block, ensuring flight controls remain responsive under all conditions.

---

## Roadmap

### v1.1
- [x] **Beep Trim:** Fine-tune target heading and altitude using arrow keys (<kbd>Shift</kbd> for fast step) with live PFD/HSI bug synchronization.
- [x] **Coordinated Turn Roll-In:** Smooth roll-into-turn banking mechanics for high-speed cruising rather than flat tail-rotor yawing.
- [x] **Smart Ground Decoupling:** Automatic ground-idle state detection and AP auto-disengagement on touchdown, preventing trim accumulation on landing.
- [x] **Improved A.TRIM Handling:** Settle-then-latch attitude capture (zero rubber-banding on release), expanded parallel trim headroom (0.85), and sanitized AP handover. PID finetuning.

### v1.2
- [x] **NAV Mode:** Flight plan (`FPL`) and radio navaid (`VOR`/`ILS`/GPS) route tracking with automatic waypoint altitude capture, `TRK` Direct-To mode (with automatic `OBS` reset on waypoint changes), and manual `OBS` radial intercept support for crosswind drift correction.
- [x] **Vertical Speed Selection (V.VEL):** Full two-way `VERT SPEED` integration in the autopilot bar with automatic $\pm 700\text{ fpm}$ climb/descent scheduling on altitude changes and smooth flare-to-capture.
- [x] **GeoFS AP Panel Integration:** Complete standalone integration with the native GeoFS autopilot bar (`AUTOPILOT`, custom `HOVER` pad, `HDG`/`NAV`, `ALT`, `VERT SPEED`), and fly-through manual override disconnect.

### v2.0
- [ ] **4-Axis Autopilot Integration:** Full 4th-axis collective actuator automation with altitude hover and airspeed selection loop.
- [ ] **GPS Position Hold (True Auto-Hover):** Geographic ground-drift zeroing to lock the helicopter stationary over a helipad even in gusty crosswinds.
- [ ] **Authentic Audio Cues (Synthesized Web Audio):** Native zero-asset audio tones for AP disconnect chime, mode switches, and flight limit alerts.
- [ ] **Airbus CAD / Instrument Panel:** Authentic Caution & Advisory Display (CAD) status indicators and avionics strip.
- [ ] **Configurable Keybindings & Controls:** In-game binding customization for gamepads, flight sticks, and international keyboard layouts.

---

## License

This project is licensed under the [MIT License](LICENSE) - created by **cyberterm**. Free and open-source for the GeoFS flight simulation community.