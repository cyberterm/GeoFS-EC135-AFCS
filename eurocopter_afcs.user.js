// ==UserScript==
// @name         GeoFS Eurocopter EC-135 AFCS Suite
// @namespace    https://github.com/cyberterm/GeoFS-EC135-AFCS
// @version      1.1.1
// @description  Complete Automatic Flight Control System (AFCS) for the GeoFS Eurocopter EC-135. Features Stability Augmentation (SAS), Pitch Auto-Trim (A.TRIM), Auto-Hover, and Cruise Autopilot in a single unified, frame-rate independent flight loop with bumpless state transitions.
// @author       cyberterm
// @match        *://*.geo-fs.com/*
// @include      *://*.geo-fs.com/*
// @run-at       document-idle
// @grant        none
// @homepageURL  https://github.com/cyberterm/GeoFS-EC135-AFCS
// @supportURL   https://github.com/cyberterm/GeoFS-EC135-AFCS/issues
// @license      MIT
// @updateURL    https://raw.githubusercontent.com/cyberterm/GeoFS-EC135-AFCS/main/eurocopter_afcs.user.js
// @downloadURL  https://raw.githubusercontent.com/cyberterm/GeoFS-EC135-AFCS/main/eurocopter_afcs.user.js
// ==/UserScript==

(function() {
    'use strict';

    // -------------------------------------------------------------------------
    // 1. CONFIGURATION
    // -------------------------------------------------------------------------
    const EC135_ID = '9';

    const AFCS_CONFIG = {
        // --- Stability Augmentation System (SAS) - High-Rate Series SEMAs ---
        sas: {
            pitchDamping: 0.015,          // Cyclic pitch damping per deg/sec
            rollDamping: 0.015,           // Cyclic roll damping per deg/sec
            maxSemaPitch: 0.12,           // SEMA series authority limit (+/- 12% cyclic pitch)
            maxSemaRoll: 0.12,            // SEMA series authority limit (+/- 12% cyclic roll)

            // SEMA Series Yaw Damper (Progressive Blend)
            yawDampStrength: 0.015,       // Tail rotor damping per deg/sec
            maxSemaYaw: 0.18,             // SEMA series authority limit (+/- 18% tail rotor)

            // Collective-to-Yaw Decoupler (Torque Anticipator / Mixing Unit)
            collectiveTorqueComp: 0.06,   // Tail rotor bias per filtered unit/sec of collective movement
            maxTorqueComp: 0.06           // Max anticipator tail rotor authority (+/- 6%)
        },

        // --- Automatic Pitch Trim (A.TRIM) - Longitudinal Parallel Motor ---
        atrim: {
            stickDeadzone: 0.03,          // Stick threshold for hands-off retention
            trimMigrationRate: 0.15,      // Migration rate per second of held stick deflection
            pitch_Kp: 0.02,              // Base retention proportional gain (cyclic / deg)
            pitch_Ki: 0.02,              // Base retention integral trim gain against flapback
            pitch_Kd: 0.01,              // Retention pitch rate damping
            maxPitchTrim: 0.85            // Maximum parallel trim authority (expanded headroom for high-speed cruise)
        },

        // --- Hover Assist (Self-Leveling Angle Mode & Heading Hold) ---
        hover: {
            pitchSensitivity: 20,         // Max target pitch angle in degrees (full stick)
            Kp_pitch: 0.025,              // Auto-leveling pitch stiffness
            Kd_pitch: 0.45,               // Pitch rate damping

            rollSensitivity: 20,          // Max target roll angle in degrees (full stick)
            Kp_roll: 0.025,               // Auto-leveling roll stiffness
            Kd_roll: 0.45,                // Roll rate damping

            yawDeadzone: 0.05,            // Pedals deadzone for heading capture
            yaw_Kp: 0.020,                // Heading hold proportional gain
            yaw_Kd: 0.80,                 // Yaw rate damping (matches SAS gyro strength)
            maxYawCmd: 0.45               // Max tail rotor command limit
        },

        // --- Cruise Autopilot (Altitude & Heading Hold) ---
        // Altitude mode acts as an upper director commanding A.TRIM target pitch attitude
        ap: {
            alt_Kp: 4.0,                  // Target V/S per foot of altitude error (smooth, cushioned flare)
            alt_vsMax: 750,               // Max target vertical speed (ft/min)

            vs_Kp: 0.02,                 // Pitch attitude adjustment per ft/min of V/S error
            vs_Ki: 0.004,                // Integral trim rate matching pitch attitude to V/S
            vs_Kd: 0.03,                 // Derivative rate damping (braking) on vertical acceleration
            pitchMin: -10,                // Max climb angle (deg) - realistic cruise envelope
            pitchMax: 12,                 // Max cruise nose-down angle (deg) - prevents phantom dive & trim windup

            // Roll & Coordinated Turn Settings
            roll_Kp: 0.025,               // Roll attitude tracking gain
            roll_Kd: 0.40,                // Roll rate damping
            roll_outputMax: 0.35,         // Max cyclic roll command
            bank_Kp: 0.90,                // Target bank angle per degree of heading error (deg bank / deg error)
            maxBankAngle: 18,             // Maximum bank angle in coordinated turn (deg)
            turnTransitionMinIas: 45,     // Airspeed where roll banking starts (kts)
            turnTransitionMaxIas: 60,     // Airspeed where turn is 100% coordinated roll (kts)

            hdg_Kp: 0.02,                 // Heading hold PID gains
            hdg_Ki: 0.0003,
            hdg_Kd: 0.40,
            hdg_integralMax: 20,
            hdg_outputMax: 0.35
        },

        // --- Keybindings ---
        keys: {
            toggleSas: 'capslock',        // CapsLock: Toggle SAS
            toggleAtrim: 'z',             // Z: Toggle A.TRIM
            toggleHover: 'g',             // G: Toggle Hover Assist
            toggleAp: 'a'                 // A: Toggle Cruise Autopilot
        }
    };

    // -------------------------------------------------------------------------
    // 2. CENTRALIZED STATE TRACKING
    // -------------------------------------------------------------------------
    let isEC135 = false;
    let animationFrameId;

    const AFCS_STATE = {
        // Active mode flags
        sasActive: true,                  // Armed on spawn by default (authentic cockpit stack)
        atrimActive: true,                // Armed on spawn by default (authentic cockpit stack)
        hoverActive: false,               // Inactive by default
        apActive: false,                  // Inactive by default

        // Raw pilot control inputs
        rawPitch: 0,
        rawRoll: 0,
        rawYaw: 0,
        rawThrottle: 0,

        // A.TRIM state
        trimPitch: 0,                     // Parallel pitch trim datum position
        lastBasePitch: 0,                 // Last commanded cyclic pitch for bumpless handover
        targetPitchAtrim: null,           // Reference pitch attitude held hands-off
        wasPitchDeflected: false,         // Transition tracking for bumpless handover
        pitchReleaseTime: 0,              // Timestamp for settle-then-latch attitude capture on stick release

        // Hover Assist state
        targetHeadingHover: null,
        wasPedalDeflectedHover: false,

        // Autopilot state
        targetAltitude: 0,
        targetHeadingAp: 0,
        cruisePitchDatum: 0,              // Nominal baseline cruise pitch attitude
        integratedPitchAp: 0,             // Maintained for backward compatibility
        lastVS: null,                     // Previous vertical speed for rate damping (braking)
        hdgIntegralAp: 0,
        hdgLastErrorAp: 0,
        apEngagePitch: 0,                 // Baseline control input at engagement for manual override
        apEngageRoll: 0,
        apEngageYaw: 0,

        // Previous sensor history for delta-t rate computations
        lastHeading: 0,
        lastPitch: 0,
        lastRoll: 0,
        lastThrottle: 0,
        filteredCollRate: 0,              // Low-pass filtered collective rate for smooth torque compensation
        lastTime: performance.now(),

        // Transition flags
        wasApActive: false,
        wasHoverActive: false
    };

    // Expose for live console inspection and debugging
    window._ec135 = window._ec135 || {};
    window._ec135.afcs = AFCS_STATE;
    window._ec135.config = AFCS_CONFIG;

    // -------------------------------------------------------------------------
    // 3. HELPERS
    // -------------------------------------------------------------------------
    function clamp(val, min, max) {
        return Math.max(min, Math.min(max, val));
    }

    function wrapAngle(delta) {
        while (delta > 180) delta -= 360;
        while (delta < -180) delta += 360;
        return delta;
    }

    function checkIsEC135() {
        try {
            return String(geofs.aircraft.instance.id) === String(EC135_ID);
        } catch (e) {
            return false;
        }
    }

    function isAutopilotButtonActive() {
        let btn = document.querySelector(".geofs-autopilot-toggle.geofs-active");
        return !!btn;
    }

    function updateUIButtonState(active) {
        document.querySelectorAll(".geofs-autopilot-toggle").forEach(function(btn) {
            if (active) {
                btn.classList.add("geofs-active");
            } else {
                btn.classList.remove("geofs-active");
            }
        });
    }

    function syncAPInstrumentBugs() {
        try {
            if (typeof geofs !== 'undefined') {
                if (geofs.autopilot) {
                    geofs.autopilot.values = geofs.autopilot.values || {};
                    // Altitude bug
                    geofs.autopilot.values.altitude = AFCS_STATE.targetAltitude;
                    geofs.autopilot.targetAltitude = AFCS_STATE.targetAltitude;

                    // Heading / Course bug: GeoFS autopilot and HSI/PFD instruments use 'course'
                    geofs.autopilot.values.course = AFCS_STATE.targetHeadingAp;
                    geofs.autopilot.values.heading = AFCS_STATE.targetHeadingAp;
                    geofs.autopilot.targetHeading = AFCS_STATE.targetHeadingAp;
                    geofs.autopilot.targetCourse = AFCS_STATE.targetHeadingAp;
                    geofs.autopilot.course = AFCS_STATE.targetHeadingAp;
                    geofs.autopilot.heading = AFCS_STATE.targetHeadingAp;

                    if (typeof geofs.autopilot.setCourse === 'function') {
                        geofs.autopilot.setCourse(Math.round(AFCS_STATE.targetHeadingAp));
                    }
                    if (typeof geofs.autopilot.setAltitude === 'function') {
                        geofs.autopilot.setAltitude(Math.round(AFCS_STATE.targetAltitude));
                    }
                }

                if (geofs.animation && geofs.animation.values) {
                    geofs.animation.values.targetAltitude = AFCS_STATE.targetAltitude;
                    geofs.animation.values.altitudeBug = AFCS_STATE.targetAltitude;
                    geofs.animation.values.targetHeading = AFCS_STATE.targetHeadingAp;
                    geofs.animation.values.targetCourse = AFCS_STATE.targetHeadingAp;
                    geofs.animation.values.headingBug = AFCS_STATE.targetHeadingAp;
                    geofs.animation.values.courseBug = AFCS_STATE.targetHeadingAp;
                    geofs.animation.values.course = AFCS_STATE.targetHeadingAp;
                }

                // Synchronize DOM input fields if the Autopilot / NAV panel is open
                let courseInputs = document.querySelectorAll(".geofs-autopilot-course, [data-method='setCourse']");
                courseInputs.forEach(function(input) {
                    input.value = Math.round(AFCS_STATE.targetHeadingAp);
                });
                let altInputs = document.querySelectorAll(".geofs-autopilot-altitude, [data-method='setAltitude']");
                altInputs.forEach(function(input) {
                    input.value = Math.round(AFCS_STATE.targetAltitude);
                });

                // Trigger instruments refresh if available
                if (typeof instruments !== 'undefined' && typeof instruments.update === 'function') {
                    instruments.update();
                }
                if (geofs.instruments && typeof geofs.instruments.update === 'function') {
                    geofs.instruments.update();
                }
            }
        } catch (e) {
            // Silently ignore
        }
    }

    function log(system, msg) {
        console.log(`[EC-135 ${system}] ${msg}`);
    }

    function notify(msg, duration = 2000) {
        try {
            let id = "ec135-hud-notification";
            let banner = document.getElementById(id);
            if (!banner) {
                banner = document.createElement("div");
                banner.id = id;
                banner.style.position = "fixed";
                banner.style.top = "60px";
                banner.style.left = "50%";
                banner.style.transform = "translateX(-50%)";
                banner.style.backgroundColor = "rgba(10, 15, 20, 0.85)";
                banner.style.color = "#00ffcc";
                banner.style.padding = "7px 18px";
                banner.style.borderRadius = "20px";
                banner.style.fontFamily = "monospace, sans-serif";
                banner.style.fontSize = "13px";
                banner.style.fontWeight = "bold";
                banner.style.letterSpacing = "0.5px";
                banner.style.boxShadow = "0 4px 14px rgba(0, 0, 0, 0.6)";
                banner.style.border = "1px solid rgba(0, 255, 204, 0.35)";
                banner.style.zIndex = "100000";
                banner.style.pointerEvents = "none";
                banner.style.transition = "opacity 0.3s ease";
                document.body.appendChild(banner);
            }
            banner.textContent = msg;
            banner.style.opacity = "1";
            clearTimeout(banner._fadeTimer);
            banner._fadeTimer = setTimeout(function() {
                banner.style.opacity = "0";
            }, duration);
        } catch (e) {
            // Silently ignore
        }
    }

    // -------------------------------------------------------------------------
    // 4. MODE SWITCHING & BUMPLESS HANDOVERS
    // -------------------------------------------------------------------------
    function setAutopilot(enable) {
        if (enable === AFCS_STATE.apActive) return;

        const vals = geofs.animation ? geofs.animation.values : null;
        if (!vals) return;

        AFCS_STATE.apActive = enable;
        updateUIButtonState(enable);

        // Always ensure GeoFS built-in fixed-wing autopilot does not interfere
        if (typeof geofs !== 'undefined' && geofs.autopilot && geofs.autopilot.isActive) {
            geofs.autopilot.turnOff();
        }

        if (enable) {
            // Autopilot takes highest precedence: disengage Hover if active
            if (AFCS_STATE.hoverActive) {
                AFCS_STATE.hoverActive = false;
                window._ec135.hoverActive = false;
                log("HOVER", "Yielded to Autopilot.");
            }

            // AP Upper Director commands through A.TRIM, ensure A.TRIM is active
            if (!AFCS_STATE.atrimActive) {
                AFCS_STATE.atrimActive = true;
                log("A.TRIM", "Armed by Autopilot.");
            }

            // Capture altitude (nearest 100 ft) and current heading
            let currentAltFeet = vals.altThousands || vals.altitude || 0;
            AFCS_STATE.targetAltitude = Math.round(currentAltFeet / 100) * 100;
            AFCS_STATE.targetHeadingAp = Math.round(vals.heading360 || 0);

            // Seamless pitch initialization: initialize cruise datum to current pitch attitude!
            let currentPitch = vals.atilt || 0;
            AFCS_STATE.cruisePitchDatum = clamp(currentPitch, AFCS_CONFIG.ap.pitchMin, AFCS_CONFIG.ap.pitchMax);
            AFCS_STATE.integratedPitchAp = AFCS_STATE.cruisePitchDatum;
            AFCS_STATE.targetPitchAtrim = currentPitch;
            AFCS_STATE.lastVS = vals.verticalSpeed || 0;
            AFCS_STATE.hdgIntegralAp = 0;
            AFCS_STATE.hdgLastErrorAp = 0;

            // Record baseline control positions to detect manual override (Fly-Through Disconnect)
            AFCS_STATE.apEngagePitch = AFCS_STATE.rawPitch || 0;
            AFCS_STATE.apEngageRoll = AFCS_STATE.rawRoll || 0;
            AFCS_STATE.apEngageYaw = AFCS_STATE.rawYaw || 0;

            log("AP", `ENGAGED - ALT ${AFCS_STATE.targetAltitude}ft, HDG ${AFCS_STATE.targetHeadingAp}° (Collective controls airspeed)`);
            notify(`EC-135 AP: ON (${AFCS_STATE.targetAltitude}ft | ${AFCS_STATE.targetHeadingAp}°)`);
            syncAPInstrumentBugs();
        } else {
            // Seamless AP Disengage Handover:
            // A.TRIM takes over from current pitch attitude with sanitized trim datum
            let currentPitch = vals.atilt || 0;
            AFCS_STATE.targetPitchAtrim = currentPitch;
            AFCS_STATE.wasPitchDeflected = false;
            AFCS_STATE.pitchReleaseTime = 0;
            AFCS_STATE.wasApActive = true;

            // Prevent handover of a fully saturated trim datum: clamp trimPitch to leave active headroom
            AFCS_STATE.trimPitch = clamp(AFCS_STATE.trimPitch, -0.75, 0.75);
            AFCS_STATE.lastBasePitch = AFCS_STATE.trimPitch;

            log("AP", "DISENGAGED - Smooth handover to Realistic Stack (A.TRIM + SAS).");
            notify("EC-135 AP: OFF");
        }
    }

    function setHover(enable) {
        if (enable === AFCS_STATE.hoverActive) return;

        const vals = geofs.animation ? geofs.animation.values : null;
        if (!vals) return;

        AFCS_STATE.hoverActive = enable;
        window._ec135.hoverActive = enable;

        if (enable) {
            // Disengage Autopilot if active
            if (AFCS_STATE.apActive) {
                setAutopilot(false);
            }

            // Reset hover attitude & heading tracking
            AFCS_STATE.targetHeadingHover = null;
            AFCS_STATE.wasPedalDeflectedHover = false;
            AFCS_STATE.trimPitch = 0; // Neutralize forward cruise trim for hover
            AFCS_STATE.lastBasePitch = 0;
            log("HOVER", "ENGAGED - Self-leveling cyclic & heading hold active.");
            notify("EC-135 Hover: ON");
        } else {
            // Disengaging Hover: return cleanly to Realistic Stack
            let currentPitch = vals.atilt || 0;
            AFCS_STATE.trimPitch = 0;
            AFCS_STATE.lastBasePitch = 0;
            AFCS_STATE.targetPitchAtrim = currentPitch;
            AFCS_STATE.wasPitchDeflected = false;
            AFCS_STATE.wasHoverActive = true;
            log("HOVER", "DISENGAGED - Returned to realistic flight stack.");
            notify("EC-135 Hover: OFF");
        }
    }

    function setAtrim(enable) {
        AFCS_STATE.atrimActive = enable;
        const vals = geofs.animation ? geofs.animation.values : null;
        if (enable) {
            hookAFCSParts();
            AFCS_STATE.targetPitchAtrim = vals ? (vals.atilt || null) : null;
            if (AFCS_STATE.trimPitch === 0 && AFCS_STATE.rawPitch !== 0) {
                AFCS_STATE.trimPitch = clamp(AFCS_STATE.rawPitch, -AFCS_CONFIG.atrim.maxPitchTrim, AFCS_CONFIG.atrim.maxPitchTrim);
                AFCS_STATE.lastBasePitch = AFCS_STATE.trimPitch;
            }
            AFCS_STATE.wasPitchDeflected = false;
            log("A.TRIM", "ENGAGED - Hands-off pitch attitude retention active.");
            notify("EC-135 A.TRIM: ON");
        } else {
            AFCS_STATE.trimPitch = 0;
            AFCS_STATE.lastBasePitch = 0;
            AFCS_STATE.targetPitchAtrim = null;
            AFCS_STATE.wasPitchDeflected = false;
            log("A.TRIM", "DISENGAGED - Direct raw pilot pitch control.");
            notify("EC-135 A.TRIM: OFF");
        }
    }

    function setSas(enable) {
        AFCS_STATE.sasActive = enable;
        if (enable) {
            AFCS_STATE.lastHeading = 0;
            AFCS_STATE.lastPitch = 0;
            AFCS_STATE.lastRoll = 0;
            AFCS_STATE.lastThrottle = 0;
            AFCS_STATE.lastTime = performance.now();
            log("SAS", "ENGAGED - Series gyro rate damping & torque anticipator active.");
            notify("EC-135 SAS: ON");
        } else {
            log("SAS", "DISENGAGED - Gyro rate damping disabled (Direct manual control).");
            notify("EC-135 SAS: OFF");
        }
    }

    // -------------------------------------------------------------------------
    // 5. UNIFIED AFCS FLIGHT CONTROL LOOP
    // -------------------------------------------------------------------------
    function updateAFCS() {
        if (!isEC135 || !geofs.animation || !geofs.animation.values) return;

        const vals = geofs.animation.values;
        let now = performance.now();
        let dt = (now - AFCS_STATE.lastTime) / 1000;
        AFCS_STATE.lastTime = now;

        // Fetch current states
        let currentAltFeet = vals.altThousands || vals.altitude || 0;
        let currentHeading = vals.heading360 || 0;
        let currentPitch = vals.atilt || 0;      // Positive = nose down, negative = nose up
        let currentRoll = vals.aroll || 0;        // Positive = right bank, negative = left bank
        let currentVS = vals.climbrate || 0;      // Vertical speed in ft/min
        let currentThrottle = vals.throttle || 0; // Collective position (0.0 to 1.0)

        // Read raw pilot control inputs from GeoFS
        const ctrl = (typeof window !== 'undefined' && (window.controls || (window.geofs && window.geofs.controls))) 
                  || (typeof controls !== 'undefined' ? controls : null);

        let rawPitch = (typeof vals.pitch === 'number')
                     ? vals.pitch
                     : ((ctrl && typeof ctrl.pitch === 'number') ? ctrl.pitch : 0);

        let rawRoll = (typeof vals.roll === 'number') 
                    ? vals.roll 
                    : ((ctrl && typeof ctrl.roll === 'number') ? ctrl.roll : 0);

        let rawYaw = (typeof vals.yaw === 'number') 
                   ? vals.yaw 
                   : ((ctrl && typeof ctrl.yaw === 'number') ? ctrl.yaw : 0);

        AFCS_STATE.rawPitch = rawPitch;
        AFCS_STATE.rawRoll = rawRoll;
        AFCS_STATE.rawYaw = rawYaw;
        AFCS_STATE.rawThrottle = currentThrottle;

        // Guard against tab switch, window minimize, or pause spikes
        if (dt <= 0 || dt > 0.5 || (AFCS_STATE.lastHeading === 0 && AFCS_STATE.lastPitch === 0 && AFCS_STATE.lastRoll === 0)) {
            AFCS_STATE.lastHeading = currentHeading;
            AFCS_STATE.lastPitch = currentPitch;
            AFCS_STATE.lastRoll = currentRoll;
            AFCS_STATE.lastThrottle = currentThrottle;
            return;
        }

        // --- Physical Frame-Rate Invariant Angular Velocities (deg/sec) ---
        let pitchRateSec = (currentPitch - AFCS_STATE.lastPitch) / dt;
        let rollRateSec = (currentRoll - AFCS_STATE.lastRoll) / dt;
        // Standard gyro sign: positive = right turn (heading increasing), negative = left turn
        let yawRateSec = wrapAngle(currentHeading - AFCS_STATE.lastHeading) / dt;
        let collectiveRate = (currentThrottle - AFCS_STATE.lastThrottle) / dt;

        // Smooth collective rate filter to prevent derivative kick spikes on key/mouse steps
        AFCS_STATE.filteredCollRate = (AFCS_STATE.filteredCollRate * 0.85) + (collectiveRate * 0.15);

        // --- Ground / Skid Contact Safety ---
        // On the ground, zero out integrals and trim to prevent false build-up against friction
        if (vals.groundContact) {
            // Cruise Autopilot must immediately disengage upon ground contact/landing
            if (AFCS_STATE.apActive) {
                setAutopilot(false);
                notify("EC-135 AP: DISENGAGED (GROUND CONTACT)", 2500);
            }

            AFCS_STATE.trimPitch = 0;
            AFCS_STATE.lastBasePitch = rawPitch;
            AFCS_STATE.targetPitchAtrim = currentPitch;
            AFCS_STATE.integratedPitchAp = currentPitch;
            AFCS_STATE.cruisePitchDatum = currentPitch;
            AFCS_STATE.hdgIntegralAp = 0;
            AFCS_STATE.hdgLastErrorAp = 0;
            AFCS_STATE.lastVS = 0;
            AFCS_STATE.targetHeadingHover = null;
            AFCS_STATE.wasPitchDeflected = false;
            AFCS_STATE.pitchReleaseTime = 0;
            AFCS_STATE.wasPedalDeflectedHover = false;
            vals.fbwPitch = rawPitch;
            vals.fbwRoll = rawRoll;
            vals.fbwYaw = rawYaw;

            AFCS_STATE.lastHeading = currentHeading;
            AFCS_STATE.lastPitch = currentPitch;
            AFCS_STATE.lastRoll = currentRoll;
            AFCS_STATE.lastThrottle = currentThrottle;
            return;
        }

        // Base control surface demands (initialized to raw pilot inputs)
        let basePitch = rawPitch;
        let baseRoll = rawRoll;
        let baseYaw = rawYaw;

        // =====================================================================
        // MODE 1: HOVER ASSIST (Self-Leveling Angle Mode & Heading Hold)
        // =====================================================================
        if (AFCS_STATE.hoverActive) {
            const hvrCfg = AFCS_CONFIG.hover;

            // Pitch Auto-Leveling Angle Mode
            // Forward stick (rawPitch < 0) commands nose down (positive pitchTarget)
            let pitchTarget = hvrCfg.pitchSensitivity * -rawPitch;
            let pitchError = pitchTarget - currentPitch;
            let pitchCmd = -((pitchError * hvrCfg.Kp_pitch) - (pitchRateSec * 0.01667 * hvrCfg.Kd_pitch));
            let fbwPitch = clamp(pitchCmd, -1.0, 1.0);

            // Roll Auto-Leveling Angle Mode
            // Right stick (rawRoll > 0) commands right bank (negative rollTarget)
            let rollTarget = hvrCfg.rollSensitivity * -rawRoll;
            let rollError = rollTarget - currentRoll;
            let rollCmd = -((rollError * hvrCfg.Kp_roll) - (rollRateSec * 0.01667 * hvrCfg.Kd_roll));
            let fbwRoll = clamp(rollCmd, -0.6, 0.6);

            // Yaw: Critically Damped Heading Hold with Two-Phase Turn Braking
            let isPedalDeflected = Math.abs(rawYaw) > hvrCfg.yawDeadzone;
            let fbwYaw = 0;

            if (isPedalDeflected) {
                // Actively commanding turn: clean, linear spot-turn authority without fighting pilot
                AFCS_STATE.wasPedalDeflectedHover = true;
                AFCS_STATE.targetHeadingHover = null;
                fbwYaw = clamp(rawYaw, -1.0, 1.0);
            } else {
                // Pedals centered:
                if (AFCS_STATE.wasPedalDeflectedHover) {
                    // Turn recently released: actively brake rotation to zero before locking heading
                    if (Math.abs(yawRateSec) > 1.5) { // > 1.5 deg/sec rotation
                        fbwYaw = clamp(-(yawRateSec * 0.025), -hvrCfg.maxYawCmd, hvrCfg.maxYawCmd);
                    } else {
                        // Rotation fully halted: capture settled heading
                        AFCS_STATE.wasPedalDeflectedHover = false;
                        AFCS_STATE.targetHeadingHover = currentHeading;
                        fbwYaw = 0;
                    }
                } else {
                    // Lock and hold settled heading with critical damping
                    if (AFCS_STATE.targetHeadingHover === null) AFCS_STATE.targetHeadingHover = currentHeading;
                    let hdgError = wrapAngle(AFCS_STATE.targetHeadingHover - currentHeading);
                    let yawCmd = (hdgError * hvrCfg.yaw_Kp) - (yawRateSec * 0.020);
                    fbwYaw = clamp(yawCmd, -hvrCfg.maxYawCmd, hvrCfg.maxYawCmd);
                }
            }

            vals.fbwPitch = fbwPitch;
            vals.fbwRoll = fbwRoll;
            vals.fbwYaw = fbwYaw;

            // Keep A.TRIM datum neutral for when hover is switched off
            AFCS_STATE.trimPitch = 0;
            AFCS_STATE.targetPitchAtrim = currentPitch;

            // Save history states
            AFCS_STATE.lastHeading = currentHeading;
            AFCS_STATE.lastPitch = currentPitch;
            AFCS_STATE.lastRoll = currentRoll;
            AFCS_STATE.lastThrottle = currentThrottle;
            return;
        }

        // =====================================================================
        // MODE 2: REALISTIC FLIGHT STACK (A.TRIM + SAS)
        // =====================================================================

        // --- 2A. CRUISE AUTOPILOT UPPER MODE DIRECTOR ---
        // When AP is active, it guides A.TRIM's target pitch attitude, holds wings level, and locks heading
        if (AFCS_STATE.apActive) {
            // Manual Pilot Control Override (Fly-Through Disconnect):
            // If the pilot moves pitch, roll, or pedals deliberately beyond engagement baseline,
            // immediately disconnect Autopilot and hand full manual control back.
            let dPitch = Math.abs(rawPitch - AFCS_STATE.apEngagePitch);
            let dRoll = Math.abs(rawRoll - AFCS_STATE.apEngageRoll);
            let dYaw = Math.abs(rawYaw - AFCS_STATE.apEngageYaw);

            if (dPitch > 0.15 || dRoll > 0.15 || dYaw > 0.20) {
                setAutopilot(false);
                notify("EC-135 AP: DISENGAGED (MANUAL OVERRIDE)", 2000);
                log("AP", "Disengaged by pilot manual control override.");
            }
        }

        if (AFCS_STATE.apActive) {
            const apCfg = AFCS_CONFIG.ap;

            // Outer Altitude Loop -> Target Vertical Speed
            let altError = AFCS_STATE.targetAltitude - currentAltFeet;
            let targetVS = clamp(altError * apCfg.alt_Kp, -apCfg.alt_vsMax, apCfg.alt_vsMax);

            // Middle V/S Loop -> Target Pitch Attitude with Integral Trim & Rate Damping
            let vsError = targetVS - currentVS;
            let vsDerivative = 0;
            if (AFCS_STATE.lastVS !== null && dt > 0) {
                vsDerivative = (currentVS - AFCS_STATE.lastVS) / dt;
            }
            AFCS_STATE.lastVS = currentVS;

            // Integral accumulates exact pitch attitude needed to match actual V/S target
            AFCS_STATE.integratedPitchAp -= (vsError * apCfg.vs_Ki) * dt;
            AFCS_STATE.integratedPitchAp = clamp(AFCS_STATE.integratedPitchAp, apCfg.pitchMin, apCfg.pitchMax);

            // Target attitude = Integrated trim - P correction + D rate damping (braking to prevent bouncing)
            let desiredPitch = clamp(
                AFCS_STATE.integratedPitchAp - (vsError * apCfg.vs_Kp) + (vsDerivative * apCfg.vs_Kd),
                apCfg.pitchMin,
                apCfg.pitchMax
            );

            // Slew-rate limit the target pitch command (max 3.5 deg/sec) to eliminate sudden jerks
            let maxPitchSlew = 3.5 * dt;
            let currentTarget = AFCS_STATE.targetPitchAtrim !== null ? AFCS_STATE.targetPitchAtrim : desiredPitch;
            let pitchDiff = desiredPitch - currentTarget;
            AFCS_STATE.targetPitchAtrim = currentTarget + clamp(pitchDiff, -maxPitchSlew, maxPitchSlew);

            // Airspeed-Scheduled Coordinated Turning
            let ias = (typeof vals.kias === 'number' ? vals.kias : (vals.indicatedAirspeed || 0)) || 0;
            let turnBlend = clamp((ias - apCfg.turnTransitionMinIas) / (apCfg.turnTransitionMaxIas - apCfg.turnTransitionMinIas), 0.0, 1.0);

            // Heading error relative to target
            let hdgError = wrapAngle(AFCS_STATE.targetHeadingAp - currentHeading);

            // Target bank angle: Negative = right bank, Positive = left bank (authentic GeoFS aroll coordinates)
            let targetBank = - clamp(hdgError * apCfg.bank_Kp, -apCfg.maxBankAngle, apCfg.maxBankAngle) * turnBlend;

            // Cyclic Roll: Tracks target bank angle in coordinated turn, wings-level in hover/slow flight
            let rollError = targetBank - currentRoll;
            let rollCmd = -((rollError * apCfg.roll_Kp) - (rollRateSec * 0.01667 * apCfg.roll_Kd));
            baseRoll = clamp(rollCmd, -apCfg.roll_outputMax, apCfg.roll_outputMax);

            // Tail Rotor: Fades from low-speed pedal steering to high-speed turn coordination
            AFCS_STATE.hdgIntegralAp += hdgError * dt;
            AFCS_STATE.hdgIntegralAp = clamp(AFCS_STATE.hdgIntegralAp, -apCfg.hdg_integralMax, apCfg.hdg_integralMax);

            // In cruise, cyclic roll carves the turn; tail rotor provides turn coordination and slip damping
            let yawCoordFactor = 1.0 - (0.65 * turnBlend);
            let yawDamping = -(yawRateSec * (apCfg.hdg_Kd * 0.02));
            let hdgOutput = ((apCfg.hdg_Kp * hdgError) + (apCfg.hdg_Ki * AFCS_STATE.hdgIntegralAp)) * yawCoordFactor + yawDamping;
            baseYaw = clamp(hdgOutput, -apCfg.hdg_outputMax, apCfg.hdg_outputMax);

            // Keep GeoFS AP values and animation values continuously synced for cockpit dials/bugs
            if (typeof geofs !== 'undefined') {
                if (geofs.autopilot && geofs.autopilot.values) {
                    geofs.autopilot.values.course = AFCS_STATE.targetHeadingAp;
                    geofs.autopilot.values.heading = AFCS_STATE.targetHeadingAp;
                    geofs.autopilot.values.altitude = AFCS_STATE.targetAltitude;
                }
                if (geofs.animation && geofs.animation.values) {
                    geofs.animation.values.targetHeading = AFCS_STATE.targetHeadingAp;
                    geofs.animation.values.targetCourse = AFCS_STATE.targetHeadingAp;
                    geofs.animation.values.headingBug = AFCS_STATE.targetHeadingAp;
                    geofs.animation.values.courseBug = AFCS_STATE.targetHeadingAp;
                    geofs.animation.values.course = AFCS_STATE.targetHeadingAp;
                }
            }
        }

        // --- 2B. A.TRIM LONGITUDINAL PITCH LOOP ---
        if (AFCS_STATE.atrimActive) {
            const atrimCfg = AFCS_CONFIG.atrim;
            // When AP is active, manual stick displacement must not hijack pitch director unless manual override triggers
            let isPitchDeflected = !AFCS_STATE.apActive && (Math.abs(rawPitch) > atrimCfg.stickDeadzone);

            if (isPitchDeflected) {
                AFCS_STATE.wasPitchDeflected = true;
                AFCS_STATE.pitchReleaseTime = 0;

                // Pilot actively steering cyclic: target tracks current attitude
                AFCS_STATE.targetPitchAtrim = currentPitch;

                // Parallel trim migration: holding stick deflects the trim datum forward/aft
                // Additive rate integration: never washes out to zero when stick centers
                let trimSlew = rawPitch * (atrimCfg.trimMigrationRate * dt);
                AFCS_STATE.trimPitch = clamp(AFCS_STATE.trimPitch + trimSlew, -atrimCfg.maxPitchTrim, atrimCfg.maxPitchTrim);

                basePitch = clamp(rawPitch + AFCS_STATE.trimPitch, -1, 1);
                AFCS_STATE.lastBasePitch = basePitch;

            } else {
                // BUMPLESS HANDOVER ON STICK RELEASE (Settle-then-latch):
                // If stick was recently released from a manual maneuver, let the nose naturally coast and settle
                // before latching the reference attitude. This eliminates rubber-banding / snapback.
                if (AFCS_STATE.wasPitchDeflected) {
                    if (!AFCS_STATE.apActive) {
                        if (AFCS_STATE.pitchReleaseTime === 0) {
                            AFCS_STATE.pitchReleaseTime = performance.now();
                            AFCS_STATE.trimPitch = clamp(AFCS_STATE.lastBasePitch !== undefined ? AFCS_STATE.lastBasePitch : basePitch, -atrimCfg.maxPitchTrim, atrimCfg.maxPitchTrim);
                        }
                        let elapsedSettle = (performance.now() - AFCS_STATE.pitchReleaseTime) / 1000;

                        // Target tracks current attitude continuously while residual pitch rate subsides
                        AFCS_STATE.targetPitchAtrim = currentPitch;

                        // Once rotation halts (< 1.5 deg/sec) or settle window expires (0.6s), latch final attitude
                        if (Math.abs(pitchRateSec) < 1.5 || elapsedSettle > 0.6) {
                            AFCS_STATE.wasPitchDeflected = false;
                            AFCS_STATE.pitchReleaseTime = 0;
                        }
                    } else {
                        AFCS_STATE.wasPitchDeflected = false;
                        AFCS_STATE.pitchReleaseTime = 0;
                    }
                }

                if (AFCS_STATE.targetPitchAtrim === null) AFCS_STATE.targetPitchAtrim = currentPitch;

                // Hands-off attitude retention PID against rotor flapback
                let pitchError = AFCS_STATE.targetPitchAtrim - currentPitch;
                AFCS_STATE.trimPitch -= (pitchError * atrimCfg.pitch_Ki) * dt;
                AFCS_STATE.trimPitch = clamp(AFCS_STATE.trimPitch, -atrimCfg.maxPitchTrim, atrimCfg.maxPitchTrim);

                let pCorr = -(pitchError * atrimCfg.pitch_Kp);
                let dCorr = pitchRateSec * atrimCfg.pitch_Kd;
                basePitch = clamp(AFCS_STATE.trimPitch + pCorr + dCorr, -1, 1);
                AFCS_STATE.lastBasePitch = basePitch;
            }
        } else {
            // A.TRIM Disarmed: direct 1:1 raw pilot pitch control (as if A.TRIM wasn't even there)
            basePitch = rawPitch;
        }

        // --- 3B. SAS SERIES GYRO RATE DAMPING & TORQUE ANTICIPATOR ---
        let finalPitch = basePitch;
        let finalRoll = baseRoll;
        let finalYaw = baseYaw;

        if (AFCS_STATE.sasActive) {
            const sasCfg = AFCS_CONFIG.sas;

            // Pitch & Roll Series SEMA Damping (Frame-rate invariant)
            let pitchCorrection = clamp(pitchRateSec * sasCfg.pitchDamping, -sasCfg.maxSemaPitch, sasCfg.maxSemaPitch);
            let rollCorrection = clamp(rollRateSec * sasCfg.rollDamping, -sasCfg.maxSemaRoll, sasCfg.maxSemaRoll);

            finalPitch = clamp(basePitch + pitchCorrection, -1.0, 1.0);
            finalRoll = clamp(baseRoll + rollCorrection, -1.0, 1.0);

            // Authentic SEMA Yaw Damper with Progressive Damping Blend
            let damperOutput = 0;
            let pedalDeflection = Math.abs(rawYaw);

            if (pedalDeflection <= 0.03) {
                // Feet-off pedals: 100% series gyro rate damping directly opposing rotation & spin
                damperOutput = clamp(-(yawRateSec * sasCfg.yawDampStrength), -sasCfg.maxSemaYaw, sasCfg.maxSemaYaw);
            } else {
                // Maneuvering: damping fades smoothly between 0.03 and 0.15 pedal deflection
                // Eliminates initial +27% kick surge and subsequent -27% rate fighting
                let dampFactor = clamp(1.0 - (pedalDeflection - 0.03) / 0.12, 0.0, 1.0);
                damperOutput = clamp(-(yawRateSec * sasCfg.yawDampStrength * dampFactor), -sasCfg.maxSemaYaw, sasCfg.maxSemaYaw);
            }

            // Filtered collective-to-yaw feedforward mixing: cancels torque kick smoothly
            let torqueComp = clamp(AFCS_STATE.filteredCollRate * sasCfg.collectiveTorqueComp, -sasCfg.maxTorqueComp, sasCfg.maxTorqueComp);

            finalYaw = clamp(baseYaw + damperOutput + torqueComp, -1.0, 1.0);
        }

        // Output to swashplate and tail rotor
        vals.fbwPitch = finalPitch;
        vals.fbwRoll = finalRoll;
        vals.fbwYaw = finalYaw;

        // Save history states
        AFCS_STATE.lastHeading = currentHeading;
        AFCS_STATE.lastPitch = currentPitch;
        AFCS_STATE.lastRoll = currentRoll;
        AFCS_STATE.lastThrottle = currentThrottle;
    }

    function flightLoop() {
        try {
            updateAFCS();
        } catch (e) {
            console.error("[EC-135 AFCS Loop Error]", e);
        }
        animationFrameId = requestAnimationFrame(flightLoop);
    }

    // -------------------------------------------------------------------------
    // 6. AIRCRAFT 3D PART HOOKING
    // -------------------------------------------------------------------------
    function hookAFCSParts() {
        if (!geofs.aircraft || !geofs.aircraft.instance || !geofs.aircraft.instance.parts) return;
        const parts = geofs.aircraft.instance.parts;

        // Tailrotor (Yaw) -> fbwYaw
        if (parts.tailrotor && parts.tailrotor.animations[2]) {
            parts.tailrotor.animations[2].value = "fbwYaw";
        }

        // Cyclic Pitch -> fbwPitch
        if (parts.cyclicLeft && parts.cyclicLeft.animations[0]) parts.cyclicLeft.animations[0].value = "fbwPitch";
        if (parts.cyclicRight && parts.cyclicRight.animations[0]) parts.cyclicRight.animations[0].value = "fbwPitch";
        if (parts.cyclicRotorNegative && parts.cyclicRotorNegative.animations[0]) parts.cyclicRotorNegative.animations[0].value = "fbwPitch";
        if (parts.cyclicRotorPositive && parts.cyclicRotorPositive.animations[0]) parts.cyclicRotorPositive.animations[0].value = "fbwPitch";

        // Cyclic Roll -> fbwRoll
        if (parts.cyclicLeft && parts.cyclicLeft.animations[1]) parts.cyclicLeft.animations[1].value = "fbwRoll";
        if (parts.cyclicRight && parts.cyclicRight.animations[1]) parts.cyclicRight.animations[1].value = "fbwRoll";
        if (parts.cyclicRotorNegative && parts.cyclicRotorNegative.animations[1]) parts.cyclicRotorNegative.animations[1].value = "fbwRoll";
        if (parts.cyclicRotorPositive && parts.cyclicRotorPositive.animations[1]) parts.cyclicRotorPositive.animations[1].value = "fbwRoll";

        log("AFCS", "All 3D cyclic and tail rotor parts hooked to unified AFCS bus.");
    }

    function unhookAFCSParts() {
        if (!geofs.aircraft || !geofs.aircraft.instance || !geofs.aircraft.instance.parts) return;
        const parts = geofs.aircraft.instance.parts;

        if (parts.tailrotor && parts.tailrotor.animations[2]) parts.tailrotor.animations[2].value = "yaw";

        if (parts.cyclicLeft && parts.cyclicLeft.animations[0]) parts.cyclicLeft.animations[0].value = "pitch";
        if (parts.cyclicRight && parts.cyclicRight.animations[0]) parts.cyclicRight.animations[0].value = "pitch";
        if (parts.cyclicRotorNegative && parts.cyclicRotorNegative.animations[0]) parts.cyclicRotorNegative.animations[0].value = "pitch";
        if (parts.cyclicRotorPositive && parts.cyclicRotorPositive.animations[0]) parts.cyclicRotorPositive.animations[0].value = "pitch";

        if (parts.cyclicLeft && parts.cyclicLeft.animations[1]) parts.cyclicLeft.animations[1].value = "roll";
        if (parts.cyclicRight && parts.cyclicRight.animations[1]) parts.cyclicRight.animations[1].value = "roll";
        if (parts.cyclicRotorNegative && parts.cyclicRotorNegative.animations[1]) parts.cyclicRotorNegative.animations[1].value = "roll";
        if (parts.cyclicRotorPositive && parts.cyclicRotorPositive.animations[1]) parts.cyclicRotorPositive.animations[1].value = "roll";

        log("AFCS", "All 3D cyclic and tail rotor parts restored to raw mechanical controls.");
    }

    // -------------------------------------------------------------------------
    // 7. KEYBOARD & UI CONTROLS LISTENER
    // -------------------------------------------------------------------------
    function hookControls() {
        // Master Key Listener
        document.addEventListener("keydown", function(event) {
            if (!isEC135) return;
            if (document.activeElement.tagName === "INPUT" || document.activeElement.tagName === "TEXTAREA") return;

            let key = event.key.toLowerCase();

            // 'CapsLock' -> Toggle SAS
            if (event.key === "CapsLock") {
                event.preventDefault();
                event.stopImmediatePropagation();
                setSas(!AFCS_STATE.sasActive);
                return;
            }

            // 'Z' -> Toggle A.TRIM
            if (key === AFCS_CONFIG.keys.toggleAtrim.toLowerCase() && !event.shiftKey && !event.ctrlKey && !event.altKey) {
                event.preventDefault();
                event.stopImmediatePropagation();
                setAtrim(!AFCS_STATE.atrimActive);
                return;
            }

            // 'G' -> Toggle Hover Assist
            if (key === AFCS_CONFIG.keys.toggleHover.toLowerCase() && !event.shiftKey && !event.ctrlKey && !event.altKey) {
                event.preventDefault();
                event.stopImmediatePropagation();
                setHover(!AFCS_STATE.hoverActive);
                return;
            }

            // 'A' -> Toggle Autopilot
            if (key === AFCS_CONFIG.keys.toggleAp.toLowerCase() && !event.shiftKey && !event.ctrlKey && !event.altKey) {
                event.preventDefault();
                event.stopImmediatePropagation();
                setAutopilot(!AFCS_STATE.apActive);
                return;
            }

            // Arrow Keys -> Autopilot Beep Trim (Altitude & Heading Adjustments)
            if (AFCS_STATE.apActive && (event.key === "ArrowUp" || event.key === "ArrowDown" || event.key === "ArrowLeft" || event.key === "ArrowRight")) {
                event.preventDefault();
                event.stopImmediatePropagation();

                // Altitude Adjustments (Up / Down)
                // Normal: +/- 100 ft | Shift: +/- 500 ft (fast climb/descent rate)
                if (event.key === "ArrowUp") {
                    let step = event.shiftKey ? 500 : 100;
                    AFCS_STATE.targetAltitude += step;
                    syncAPInstrumentBugs();
                    notify(`EC-135 AP: ALT ${AFCS_STATE.targetAltitude}ft (+${step})`, 1500);
                    return;
                }
                if (event.key === "ArrowDown") {
                    let step = event.shiftKey ? 500 : 100;
                    AFCS_STATE.targetAltitude = Math.max(0, AFCS_STATE.targetAltitude - step);
                    syncAPInstrumentBugs();
                    notify(`EC-135 AP: ALT ${AFCS_STATE.targetAltitude}ft (-${step})`, 1500);
                    return;
                }

                // Heading Adjustments (Left / Right)
                // Normal: +/- 1 deg | Shift: +/- 5 deg (faster course turn)
                if (event.key === "ArrowLeft") {
                    let step = event.shiftKey ? 5 : 1;
                    AFCS_STATE.targetHeadingAp = (AFCS_STATE.targetHeadingAp - step + 360) % 360;
                    syncAPInstrumentBugs();
                    notify(`EC-135 AP: HDG ${AFCS_STATE.targetHeadingAp}° (-${step}°)`, 1500);
                    return;
                }
                if (event.key === "ArrowRight") {
                    let step = event.shiftKey ? 5 : 1;
                    AFCS_STATE.targetHeadingAp = (AFCS_STATE.targetHeadingAp + step) % 360;
                    syncAPInstrumentBugs();
                    notify(`EC-135 AP: HDG ${AFCS_STATE.targetHeadingAp}° (+${step}°)`, 1500);
                    return;
                }
            }
        }, true);

        // Cockpit UI Autopilot Button Listener
        document.addEventListener("click", function(event) {
            if (!isEC135) return;
            let target = event.target;
            if (target && target.closest && target.closest(".geofs-autopilot-toggle, [data-method='autopilotToggle'], [data-toggle='autopilot']")) {
                event.stopImmediatePropagation();
                event.preventDefault();
                setAutopilot(!AFCS_STATE.apActive);
            }
        }, true);
    }

    // -------------------------------------------------------------------------
    // 8. INITIALIZATION & MONITORING
    // -------------------------------------------------------------------------
    console.log("EC-135 Unified AFCS Suite waiting for GeoFS...");

    let waitForReady = setInterval(function() {
        if (typeof geofs !== 'undefined' && geofs.aircraft && geofs.aircraft.instance && geofs.aircraft.instance.parts && geofs.animation && geofs.animation.values) {
            clearInterval(waitForReady);

            isEC135 = checkIsEC135();
            let lastAircraftId = geofs.aircraft.instance.id;

            hookControls();
            animationFrameId = requestAnimationFrame(flightLoop);

            if (isEC135) {
                hookAFCSParts();
                if (typeof geofs !== 'undefined' && geofs.autopilot) {
                    if (geofs.autopilot.isActive) geofs.autopilot.turnOff();

                    // Intercept any native GeoFS autopilot triggers so they route cleanly to AFCS
                    let origTurnOn = geofs.autopilot.turnOn;
                    geofs.autopilot.turnOn = function() {
                        if (isEC135) {
                            setAutopilot(true);
                            return;
                        }
                        if (typeof origTurnOn === 'function') origTurnOn.apply(this, arguments);
                    };

                    let origTurnOff = geofs.autopilot.turnOff;
                    geofs.autopilot.turnOff = function() {
                        if (isEC135) {
                            setAutopilot(false);
                            return;
                        }
                        if (typeof origTurnOff === 'function') origTurnOff.apply(this, arguments);
                    };
                }
                console.log("%c[EC-135 AFCS Suite] Engaged & Active! Ready for flight.", "color: #00ffcc; font-weight: bold;");
                console.log("[EC-135 AFCS] Controls: 'Z' = A.TRIM | 'CapsLock' = SAS | 'G' = Hover Assist | 'A' = Cruise Autopilot");
                notify("EC-135 AFCS: Active (SAS + A.TRIM)", 3500);
            } else {
                console.log("[EC-135 AFCS] Current aircraft is not an EC-135. Suite standing by.");
            }

            // Aircraft model change monitor
            setInterval(function() {
                try {
                    if (!geofs.aircraft || !geofs.aircraft.instance) return;

                    let currentId = geofs.aircraft.instance.id;
                    if (currentId !== lastAircraftId) {
                        lastAircraftId = currentId;
                        isEC135 = checkIsEC135();

                        if (isEC135) {
                            hookAFCSParts();
                            // Reset state for new spawn
                            AFCS_STATE.sasActive = true;
                            AFCS_STATE.atrimActive = true;
                            AFCS_STATE.hoverActive = false;
                            AFCS_STATE.apActive = false;
                            AFCS_STATE.trimPitch = 0;
                            AFCS_STATE.lastBasePitch = 0;
                            AFCS_STATE.targetPitchAtrim = null;
                            updateUIButtonState(false);
                            if (typeof geofs !== 'undefined' && geofs.autopilot && geofs.autopilot.isActive) {
                                geofs.autopilot.turnOff();
                            }
                            console.log("%c[EC-135 AFCS] EC-135 detected. AFCS Stack armed by default.", "color: #00ffcc; font-weight: bold;");
                            notify("EC-135 AFCS: Active (SAS + A.TRIM)", 3500);
                        } else {
                            unhookAFCSParts();
                            AFCS_STATE.apActive = false;
                            AFCS_STATE.hoverActive = false;
                            updateUIButtonState(false);
                        }
                    }
                } catch (e) {
                    // Silently ignore
                }
            }, 2000);
        }
    }, 1000);

})();
