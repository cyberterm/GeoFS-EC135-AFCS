# GeoFS Eurocopter EC-135 AFCS & Flight Assists

A modular flight control suite for the **Eurocopter EC-135** (`ID 9`) in [GeoFS](https://www.geo-fs.com/). 

Designed to fix the twitchy stock handling, eliminate constant forward stick pressure in cruise, and provide realistic stability augmentation, auto-hover, and autopilot.

---

## Quickstart

Install any or all scripts via Tampermonkey. They coordinate automatically through a shared flight bus without conflicts.

### Control Cheat-Sheet

| Script | What It Does | Default | Toggle |
| :--- | :--- | :--- | :--- |
| **`eurocopter_sas.js`** | **Stability Augmentation System** — Gyro rate damping across pitch, roll, and yaw. Eliminates twitchiness. | **Active on Spawn** | <kbd>Caps Lock</kbd> |
| **`eurocopter_atrim.js`** | **Pitch Auto-Trim (A.TRIM)** — Holds forward cruise pitch hands-off against rotor flapback. No stick holding needed. | **Active on Spawn** | <kbd>Z</kbd> |
| **`eurocopter_hover.js`** | **Hover Assist** — Self-leveling angle mode. Auto-levels to 0° wings-level hover and holds heading on stick release. | Inactive | <kbd>G</kbd> |
| **`eurocopter_ap.js`** | **Cruise Autopilot** — Locks barometric altitude and heading. Collective controls airspeed. | Inactive | <kbd>A</kbd> |

### Recommended Setups

* **Realistic Cockpit Stack (Recommended):** Install `eurocopter_sas.js` + `eurocopter_atrim.js`. Both arm on spawn. Push forward to accelerate, release the stick, and cruise hands-off while retaining 100% natural cyclic feel.
* **Easy Hovering / Helipad Landings:** Add `eurocopter_hover.js`. Cruise in the realistic stack, tap <kbd>G</kbd> on approach to auto-level into a hover, and tap <kbd>G</kbd> again to depart.
* **Long-Distance Cruise:** Add `eurocopter_ap.js`. Tap <kbd>A</kbd> to lock altitude and heading; use collective to set your cruise speed.

---

## Installation

### Option 1: Tampermonkey (Recommended)
1. Install the [Tampermonkey](https://www.tampermonkey.net/) extension in your browser.
2. Click the extension icon and select **Create a new script**.
3. Replace the default template with the code from any script (`eurocopter_atrim.js`, `eurocopter_sas.js`, `eurocopter_hover.js`, or `eurocopter_ap.js`).
4. Save (<kbd>Ctrl</kbd> + <kbd>S</kbd>) and load [GeoFS](https://www.geo-fs.com/geofs.php). The scripts activate automatically when you spawn in the EC-135.

### Option 2: Browser Developer Console (F12)
1. Spawn in the EC-135 in GeoFS.
2. Press <kbd>F12</kbd> (or right-click → **Inspect**), open the **Console** tab, paste the script code, and press <kbd>Enter</kbd>.

---

## Flight Operations & Keybinds

```
                      [ Cruise Autopilot ('A') ]
                                  │
                                  ▼
                        [ Hover Assist ('G') ]
                                  │
                                  ▼
               [ Realistic Stack: SAS (Caps) + A.TRIM ('Z') ]
```

1. **Takeoff & Cruise (Default):**
   * Spawning activates **SAS** and **A.TRIM** automatically.
   * Pull collective to climb. Push the cyclic forward to reach your desired cruising speed (e.g. -8° nose dip for 120 kts).
   * Release the stick. A.TRIM holds that nose-down attitude hands-off. Pulling or pushing the stick overrides it smoothly without violent pitch-up snaps.
2. **Transitioning to Hover (<kbd>G</kbd>):**
   * Approaching a helipad, press <kbd>G</kbd>. A.TRIM and SAS yield, and **Hover Assist** takes over.
   * Release the controls: the helicopter automatically levels its wings to 0° and locks onto its current heading.
   * Press <kbd>G</kbd> again to depart. The realistic stack seamlessly resumes.
3. **Engaging Autopilot (<kbd>A</kbd>):**
   * Press <kbd>A</kbd> (or click the cockpit AP button) at cruising altitude.
   * The AP captures your current barometric altitude (rounded to the nearest 100 ft) and heading.
   * Lower assist modes automatically yield. Press <kbd>A</kbd> again to disconnect and resume manual trimmed flight with zero bump.

---

## System Breakdown

### `eurocopter_atrim.js` (Pitch Auto-Trim)
Replicates the parallel longitudinal trim motor of the Eurocopter EC-135:
* **Hands-Off Cruise Retention:** Holds the commanded nose-down pitch attitude against natural rotor flapback.
* **Smooth Pilot Override:** Deflecting the cyclic moves the stick relative to the trim datum (`Cyclic = trimDatum + pilotInput`), eliminating the violent snapback of basic attitude holds.
* **Parallel Trim Migration:** Holding stick deflection slowly migrates the trim datum (~1.5°/sec), washing out stick force.
* **Clean Pitch Focus:** Modulates pitch only. Roll and yaw remain 100% direct mechanical controls with zero cross-coupling.

### `eurocopter_sas.js` (Stability Augmentation System)
Replicates the high-speed series actuators (SEMAs) of the real EC-135:
* **Gyro Rate Damping:** Dampens pitch rate, roll rate, and yaw rate to eliminate aerodynamic twitchiness and tail fishtailing.
* **Auto-Yielding:** Steps aside when Autopilot (<kbd>A</kbd>) or Hover Assist (<kbd>G</kbd>) are active, and re-hooks automatically upon disengagement.
* **Toggle:** Press <kbd>Caps Lock</kbd> to toggle raw, unaugmented manual flight.

### `eurocopter_hover.js` (Hover Assist)
Self-centering angle mode designed for easy hovering and precision landings:
* **Self-Leveling Cyclic:** Stick deflection commands pitch and roll angles; centering the stick returns wings and pitch to 0° level.
* **Pedal Heading Hold:** Centering the rudder pedals locks onto current heading hands-off. Deflecting pedals turns the tail with rate damping.
* **Toggle:** Press <kbd>G</kbd> to switch between the realistic flight stack and hover mode.

### `eurocopter_ap.js` (Cruise Autopilot)
A 3-loop cruise autopilot for long-distance flights:
* **Altitude Hold:** Outer altitude loop commands vertical speed, middle loop commands pitch attitude, and inner loop drives cyclic pitch.
* **Heading Hold:** PID loop drives the Fenestron tail rotor to hold heading.
* **Speed Control:** Collective remains 100% manual; raising collective commands forward cyclic to maintain altitude, increasing airspeed.
* **Toggle:** Press <kbd>A</kbd> or click the UI autopilot toggle.

---

## Technical Details (For Modders)

* **Inter-Script Coordination:** Scripts share a global bus (`window._ec135.hoverActive`). When Hover Assist engages, A.TRIM resets forward cruise trim to 0 and yields, while SAS steps aside to prevent dual damping loops.
* **Handshake on AP Disengagement:** When `eurocopter_ap.js` disconnects, it unhooks 3D parts back to default animation values. `eurocopter_sas.js` listens for this transition and re-hooks the cyclic and tail rotor parts back to `fbwPitch`/`fbwRoll`/`fbwYaw`, while `eurocopter_atrim.js` inherits the AP's last cruise pitch trim position for a bump-free handover.
* **Control Interception:** `eurocopter_atrim.js` uses `Object.defineProperty` on `geofs.animation.values.pitch`. This cleanly intercepts control inputs before SAS and GeoFS physics without modifying stock game files.
* **HUD Notifications:** All toggles trigger a non-intrusive floating HUD banner that automatically fades after 2 seconds without blocking clicks or pausing the simulator.