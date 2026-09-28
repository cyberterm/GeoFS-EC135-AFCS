// ==UserScript==
// @name         GeoFS Eurocopter EC-135 AFCS Suite
// @namespace    https://github.com/cyberterm/GeoFS-EC135-AFCS
// @version      1.0.0
// @description  Complete Automatic Flight Control System (AFCS) for the GeoFS Eurocopter EC-135. Features Stability Augmentation (SAS), Pitch Auto-Trim (A.TRIM), Auto-Hover, and Cruise Autopilot in a single unified, frame-rate independent flight loop with bumpless state transitions.
// @author       cyberterm
// @match        *://*.geo-fs.com/*
// @include      *://*.geo-fs.com/*
// @run-at       document-idle
// @grant        none
// @homepageURL  https://github.com/cyberterm/GeoFS-EC135-AFCS
// @supportURL   https://github.com/cyberterm/GeoFS-EC135-AFCS/issues
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
            pitch_Kp: 0.035,              // Retention proportional gain (cyclic / deg)
            pitch_Ki: 0.035,              // Retention integral trim gain against flapback
            pitch_Kd: 0.020,              // Retention pitch rate damping
            maxPitchTrim: 0.75            // Maximum parallel trim authority
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
        ap: {
            alt_Kp: 3.0,                  // Target V/S per foot of altitude error
            alt_vsMax: 500,               // Max target vertical speed (ft/min)

            vs_Kp: 0.01,                  // Pitch attitude per ft/min of V/S error
            vs_Ki: 0.003,                 // Integral trim rate for cruise nose-down angle
            pitchMin: -15,                // Max climb angle (deg)
            pitchMax: 30,                 // Max cruise nose-down angle (deg)

            pitch_Kp: 0.04,               // Cyclic pitch tracking gain
            pitch_Kd: 0.80,               // Pitch rate damping
            cyclicPitchMax: 1.0,

            roll_Kp: 0.02,                // Wings-level roll hold gain
            roll_Kd: 0.40,
            roll_outputMax: 0.35,

            hdg_Kp: 0.015,                // Heading hold PID gains
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

        // Hover Assist state
        targetHeadingHover: null,
        wasPedalDeflectedHover: false,

        // Autopilot state
        targetAltitude: 0,
        targetHeadingAp: 0,
        integratedPitchAp: 0,
        hdgIntegralAp: 0,
        hdgLastErrorAp: 0,
        lastFbwPitchAp: 0,

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

    function log(system, msg) {
        console.log(`[EC-135 ${system}] ${msg}`);
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

            // Capture altitude (nearest 100 ft) and current heading
            let currentAltFeet = vals.altThousands || vals.altitude || 0;
            AFCS_STATE.targetAltitude = Math.round(currentAltFeet / 100) * 100;
            AFCS_STATE.targetHeadingAp = Math.round(vals.heading360 || 0);

            // Seamless pitch initialization: initialize cruise trim to current pitch attitude!
            let currentPitch = vals.atilt || 0;
            AFCS_STATE.integratedPitchAp = clamp(currentPitch, AFCS_CONFIG.ap.pitchMin, AFCS_CONFIG.ap.pitchMax);
            AFCS_STATE.hdgIntegralAp = 0;
            AFCS_STATE.hdgLastErrorAp = 0;

            log("AP", `ENGAGED - ALT ${AFCS_STATE.targetAltitude}ft, HDG ${AFCS_STATE.targetHeadingAp}° (Collective controls airspeed)`);
        } else {
            // Seamless AP Disengage Handover:
            // A.TRIM inherits the AP's cruise attitude and trim position so there is zero pitch snap
            let currentPitch = vals.atilt || 0;
            AFCS_STATE.targetPitchAtrim = currentPitch;
            if (AFCS_STATE.lastFbwPitchAp !== 0) {
                AFCS_STATE.trimPitch = clamp(AFCS_STATE.lastFbwPitchAp, -AFCS_CONFIG.atrim.maxPitchTrim, AFCS_CONFIG.atrim.maxPitchTrim);
                AFCS_STATE.lastBasePitch = AFCS_STATE.trimPitch;
            }
            AFCS_STATE.wasPitchDeflected = false;
            AFCS_STATE.wasApActive = true;
            log("AP", "DISENGAGED - Smooth handover to Realistic Stack (A.TRIM + SAS).");
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
        } else {
            // Disengaging Hover: return cleanly to Realistic Stack
            let currentPitch = vals.atilt || 0;
            AFCS_STATE.trimPitch = 0;
            AFCS_STATE.lastBasePitch = 0;
            AFCS_STATE.targetPitchAtrim = currentPitch;
            AFCS_STATE.wasPitchDeflected = false;
            AFCS_STATE.wasHoverActive = true;
            log("HOVER", "DISENGAGED - Returned to realistic flight stack.");
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
        } else {
            AFCS_STATE.trimPitch = 0;
            AFCS_STATE.lastBasePitch = 0;
            AFCS_STATE.targetPitchAtrim = null;
            AFCS_STATE.wasPitchDeflected = false;
            log("A.TRIM", "DISENGAGED - Direct raw pilot pitch control.");
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
        } else {
            log("SAS", "DISENGAGED - Gyro rate damping disabled (Direct manual control).");
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
            AFCS_STATE.trimPitch = 0;
            AFCS_STATE.lastBasePitch = rawPitch;
            AFCS_STATE.targetPitchAtrim = currentPitch;
            AFCS_STATE.targetHeadingHover = null;
            AFCS_STATE.wasPitchDeflected = false;
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

        // =====================================================================
        // MODE 1: CRUISE AUTOPILOT (Highest Precedence)
        // =====================================================================
        if (AFCS_STATE.apActive) {
            const apCfg = AFCS_CONFIG.ap;

            // Outer Altitude Loop -> Target Vertical Speed
            let altError = AFCS_STATE.targetAltitude - currentAltFeet;
            let targetVS = clamp(altError * apCfg.alt_Kp, -apCfg.alt_vsMax, apCfg.alt_vsMax);

            // Middle V/S Loop -> Target Pitch Attitude
            let vsError = targetVS - currentVS;
            AFCS_STATE.integratedPitchAp -= (vsError * apCfg.vs_Ki) * dt;
            AFCS_STATE.integratedPitchAp = clamp(AFCS_STATE.integratedPitchAp, apCfg.pitchMin, apCfg.pitchMax);

            let targetPitch = clamp(AFCS_STATE.integratedPitchAp - (vsError * apCfg.vs_Kp), apCfg.pitchMin, apCfg.pitchMax);

            // Inner Pitch Tracking Loop -> Cyclic Pitch (fbwPitch)
            let pitchError = targetPitch - currentPitch;
            let pitchCmd = -((pitchError * apCfg.pitch_Kp) - (pitchRateSec * 0.01667 * apCfg.pitch_Kd));
            let fbwPitch = clamp(pitchCmd, -apCfg.cyclicPitchMax, apCfg.cyclicPitchMax);

            // Roll Hold (Wings Level) -> Cyclic Roll (fbwRoll)
            let rollError = 0 - currentRoll;
            let rollCmd = -((rollError * apCfg.roll_Kp) - (rollRateSec * 0.01667 * apCfg.roll_Kd));
            let fbwRoll = clamp(rollCmd, -apCfg.roll_outputMax, apCfg.roll_outputMax);

            // Heading Hold PID -> Tail Rotor (fbwYaw)
            let hdgError = wrapAngle(AFCS_STATE.targetHeadingAp - currentHeading);
            AFCS_STATE.hdgIntegralAp += hdgError * dt;
            AFCS_STATE.hdgIntegralAp = clamp(AFCS_STATE.hdgIntegralAp, -apCfg.hdg_integralMax, apCfg.hdg_integralMax);

            let hdgDerivative = (hdgError - AFCS_STATE.hdgLastErrorAp);
            AFCS_STATE.hdgLastErrorAp = hdgError;

            let hdgOutput = (apCfg.hdg_Kp * hdgError) + (apCfg.hdg_Ki * AFCS_STATE.hdgIntegralAp) + (apCfg.hdg_Kd * hdgDerivative);
            let fbwYaw = clamp(hdgOutput, -apCfg.hdg_outputMax, apCfg.hdg_outputMax);

            // Output to swashplate and tail
            vals.fbwPitch = fbwPitch;
            vals.fbwRoll = fbwRoll;
            vals.fbwYaw = fbwYaw;
            AFCS_STATE.lastFbwPitchAp = fbwPitch;

            // Synchronize lower-layer datums so disconnecting AP is 100% bump-free
            AFCS_STATE.trimPitch = clamp(fbwPitch, -AFCS_CONFIG.atrim.maxPitchTrim, AFCS_CONFIG.atrim.maxPitchTrim);
            AFCS_STATE.targetPitchAtrim = currentPitch;

            // Save history states
            AFCS_STATE.lastHeading = currentHeading;
            AFCS_STATE.lastPitch = currentPitch;
            AFCS_STATE.lastRoll = currentRoll;
            AFCS_STATE.lastThrottle = currentThrottle;
            return;
        }

        // =====================================================================
        // MODE 2: HOVER ASSIST (Self-Leveling Angle Mode & Heading Hold)
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
        // MODE 3: REALISTIC FLIGHT STACK (A.TRIM + SAS)
        // =====================================================================
        let basePitch = rawPitch;
        let baseRoll = rawRoll;
        let baseYaw = rawYaw;

        // --- 3A. A.TRIM LONGITUDINAL PITCH LOOP ---
        if (AFCS_STATE.atrimActive) {
            const atrimCfg = AFCS_CONFIG.atrim;
            let isPitchDeflected = Math.abs(rawPitch) > atrimCfg.stickDeadzone;

            if (isPitchDeflected) {
                AFCS_STATE.wasPitchDeflected = true;

                // Pilot actively steering cyclic: target tracks current attitude
                AFCS_STATE.targetPitchAtrim = currentPitch;

                // Parallel trim migration: holding stick deflects the trim datum forward/aft
                // Additive rate integration: never washes out to zero when stick centers
                let trimSlew = rawPitch * (atrimCfg.trimMigrationRate * dt);
                AFCS_STATE.trimPitch = clamp(AFCS_STATE.trimPitch + trimSlew, -atrimCfg.maxPitchTrim, atrimCfg.maxPitchTrim);

                basePitch = clamp(rawPitch + AFCS_STATE.trimPitch, -1, 1);
                AFCS_STATE.lastBasePitch = basePitch;

            } else {
                // BUMPLESS HANDOVER ON STICK RELEASE:
                // The instant cyclic centers into deadzone, lock commanded cyclic into trimPitch
                // and capture current pitch attitude. Zero cyclic drop, zero flapback ballooning!
                if (AFCS_STATE.wasPitchDeflected) {
                    AFCS_STATE.wasPitchDeflected = false;
                    AFCS_STATE.targetPitchAtrim = currentPitch;
                    AFCS_STATE.trimPitch = clamp(AFCS_STATE.lastBasePitch !== undefined ? AFCS_STATE.lastBasePitch : basePitch, -atrimCfg.maxPitchTrim, atrimCfg.maxPitchTrim);
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
        }, true);

        // Cockpit UI Autopilot Button Listener
        document.addEventListener("click", function(event) {
            if (!isEC135) return;
            let target = event.target;
            if (target && target.closest && target.closest(".geofs-autopilot-toggle")) {
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
                if (typeof geofs !== 'undefined' && geofs.autopilot && geofs.autopilot.isActive) {
                    geofs.autopilot.turnOff();
                }
                console.log("%c[EC-135 AFCS Suite] Engaged & Active! Ready for flight.", "color: #00ffcc; font-weight: bold;");
                console.log("[EC-135 AFCS] Controls: 'Z' = A.TRIM | 'CapsLock' = SAS | 'G' = Hover Assist | 'A' = Cruise Autopilot");
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
