# GeoFS-EC135-AFCS

An **Automatic Flight Control System (AFCS)** suite for the Eurocopter EC135 in GeoFS, featuring:
* **Stability Augmentation System (SAS)** — Real-time rate damping for smooth manual flight.
* **Fly-By-Wire (FBW)** — Self-leveling angle mode for easy handling.
* **Cruise Autopilot (AP)** — Barometric altitude and heading hold with manual collective speed control.

---

## Included Systems

### `eurocopter_sas.js`
This is closer to what the real helicopter uses. It damps angular movement across pitch, roll, and yaw, making the helicopter much more stable and predictable while preserving full manual control.
* **Toggle Key:** <kbd>Caps Lock</kbd> (starts disarmed by default).

### `eurocopter_fbw.js`
Fly-by-wire self-leveling system (similar to drone "angle mode"). It automatically centers and levels the helicopter, mapping your stick deflection directly to target pitch and roll angles.
* **Toggle Key:** <kbd>Caps Lock</kbd> (starts disarmed by default).

### `eurocopter_ap.js`
A cruise-style autopilot designed specifically for the EC-135. When engaged, it reads the indicated cockpit altimeter (accounting for live barometric weather/QNH pressure shifts), rounds to the nearest 100 ft, and locks onto your current heading.
* **Altitude** is held via pitch trim integration.
* **Heading** is held via the tail rotor.
* **Collective/Throttle** remains fully in pilot hands as a cruise speed control (adding collective pushes the nose down to hold altitude, increasing airspeed).
* **Toggle Key:** <kbd>A</kbd>

---

## Compatibility Note

> **Only one system should be actively engaged at a time.**
> * You can pair the **Autopilot (`eurocopter_ap.js`)** with either **SAS** or **FBW** for manual flight, but always disengage SAS/FBW (<kbd>Caps Lock</kbd>) before turning on the Autopilot (<kbd>A</kbd>), and vice versa. Having two systems active concurrently will cause them to fight for control.
> * **Do not use `eurocopter_sas.js` and `eurocopter_fbw.js` together**, as both share the same <kbd>Caps Lock</kbd> toggle key and control loops.

---

## Installation

You can run these scripts using either a userscript manager (recommended) or directly via the browser developer console:

### Option 1: Tampermonkey (Recommended)
This runs the script automatically every time you fly in GeoFS:
1. Install the [Tampermonkey](https://www.tampermonkey.net/) extension for your browser (Chrome, Firefox, Edge, etc.).
2. Click the Tampermonkey extension icon and select **Create a new script**.
3. Delete any default template code, then copy and paste the entire contents of the desired `.js` file (`eurocopter_ap.js`, `eurocopter_sas.js`, or `eurocopter_fbw.js`).
4. Press <kbd>Ctrl</kbd> + <kbd>S</kbd> (or **File > Save**).
5. Load or refresh [GeoFS](https://www.geo-fs.com/geofs.php). The script will initialize as soon as the aircraft loads.

### Option 2: Browser Console (F12)
Ideal for a quick test without installing extensions:
1. Open [GeoFS](https://www.geo-fs.com/geofs.php) and spawn in the EC-135.
2. Press <kbd>F12</kbd> (or right-click anywhere on the page and select **Inspect**), then switch to the **Console** tab.
3. Copy the entire code of the script you want to use.
4. Paste it into the console prompt and press <kbd>Enter</kbd>.
   *(Note: You will need to re-paste the script if you refresh the browser page).*