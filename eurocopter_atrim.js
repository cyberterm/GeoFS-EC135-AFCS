// ==UserScript==
// @name         GeoFS EC-135 Pitch Auto-Trim (A.TRIM)
// @namespace    https://github.com/cyberterm
// @version      2026-09-25
// @description  Authentic EC-135 Pitch Automatic Trim (A.TRIM) for GeoFS. Hands-off cruise attitude retention with smooth parallel trim migration and direct mechanical cyclic authority.
// @author       cyberterm
// @match        *://*.geo-fs.com/*
// @include      *://*.geo-fs.com/*
// @run-at       document-idle
// @grant        none
// ==/UserScript==

(function() {
    'use strict';

    // ----------------------------------------
    // 1. CONFIGURATION
    // ----------------------------------------
    const EC135_ID = '9';

    const ATRIM_CONFIG = {
        // Stick deadzone (below this = hands-off / hold trimmed attitude)
        stickDeadzone: 0.03,

        // A.TRIM Longitudinal Pitch Loop (Parallel Trim Motor)
        // In real helicopters, electric parallel trim motors migrate trim at ~2 deg/sec
        trimMigrationRate: 0.15,   // Migration rate per second of stick deflection
        pitch_Kp: 0.035,           // Proportional gain to hold target attitude hands-off
        pitch_Ki: 0.035,           // Integral trim gain: builds steady forward cyclic against flapback
        pitch_Kd: 0.020,           // Pitch rate damping gain
        maxPitchTrim: 0.75,        // Maximum cyclic pitch trim authority

        // Master Toggle Key
        toggleKey: 'z'             // Press 'Z' to toggle A.TRIM system on/off
    };

    // ----------------------------------------
    // 2. STATE TRACKING
    // ----------------------------------------
    let atrimEnabled = true;       // Active by default (authentic cockpit A.TRIM setting)
    let isEC135 = false;

    // Raw pilot pitch input intercepted from GeoFS controls
    let _rawPitch = 0;

    // Parallel Pitch Trim Datum (like the electric trim motor in the real EC-135)
    let trimPitch = 0;             // Current parallel pitch trim position
    let targetPitch = null;        // Captured pitch attitude in degrees (atilt: positive = nose down)

    // Controller output applied to the swashplate
    let outputPitch = 0;

    let lastPitch = 0;
    let lastTime = performance.now();
    let wasApActive = false;
    let wasHoverActive = false;
    let wasPitchDeflected = false;
    let animationFrameId;

    // ----------------------------------------
    // 3. HELPERS
    // ----------------------------------------
    function clamp(val, min, max) {
        return Math.max(min, Math.min(max, val));
    }

    function checkIsEC135() {
        try {
            return String(geofs.aircraft.instance.id) === String(EC135_ID);
        } catch (e) {
            return false;
        }
    }

    function isAutopilotActive() {
        let btn = document.querySelector(".geofs-autopilot-toggle.geofs-active");
        return !!btn;
    }

    function isHoverActive() {
        return !!(window._ec135 && window._ec135.hoverActive);
    }

    function showNotification(msg) {
        console.log("[EC-135 A.TRIM] " + msg);
    }

    // ----------------------------------------
    // 4. CORE A.TRIM PITCH CONTROL LOOP
    // ----------------------------------------
    function updateATRIM() {
        if (!atrimEnabled || !isEC135 || !geofs.animation || !geofs.animation.values) {
            outputPitch = _rawPitch;
            return;
        }

        const vals = geofs.animation.values;
        let now = performance.now();
        let dt = (now - lastTime) / 1000;
        lastTime = now;

        if (dt <= 0 || dt > 0.5) {
            lastPitch = vals.atilt || 0;
            return;
        }

        // Current pitch attitude (Positive = nose down, negative = nose up)
        let currentPitch = vals.atilt || 0;
        let pitchRate = (currentPitch - lastPitch) / dt;
        lastPitch = currentPitch;

        // Ground / Taxi Safety: Reset trim while on the ground
        if (vals.groundContact) {
            trimPitch = 0;
            targetPitch = currentPitch;
            outputPitch = _rawPitch;
            wasPitchDeflected = false;
            return;
        }

        // Upper Autopilot cooperation (eurocopter_ap.js):
        // While AP is active, AP drives fbwPitch directly; synchronize our datum to prevent bumps
        let apActive = isAutopilotActive();
        if (apActive) {
            wasApActive = true;
            targetPitch = currentPitch;
            if (vals.fbwPitch !== undefined && vals.fbwPitch !== 0) {
                trimPitch = clamp(vals.fbwPitch, -ATRIM_CONFIG.maxPitchTrim, ATRIM_CONFIG.maxPitchTrim);
            }
            outputPitch = _rawPitch;
            wasPitchDeflected = false;
            return;
        }

        // Seamless handover when AP disconnects:
        if (wasApActive) {
            wasApActive = false;
            targetPitch = currentPitch;
            lastPitch = currentPitch;
            wasPitchDeflected = false;
        }

        // Hover Assist cooperation (eurocopter_hover.js):
        // While Hover is active, self-leveling angle mode drives cyclic; yield and reset trim
        let hoverActive = isHoverActive();
        if (hoverActive) {
            wasHoverActive = true;
            trimPitch = 0;
            targetPitch = currentPitch;
            outputPitch = _rawPitch;
            wasPitchDeflected = false;
            return;
        }

        // Seamless handover when Hover disengages:
        if (wasHoverActive) {
            wasHoverActive = false;
            trimPitch = 0;
            targetPitch = currentPitch;
            lastPitch = currentPitch;
            wasPitchDeflected = false;
        }

        if (targetPitch === null) targetPitch = currentPitch;

        // ==========================================
        // PITCH: A.TRIM PARALLEL MOTOR LOGIC
        // ==========================================
        let isPitchDeflected = Math.abs(_rawPitch) > ATRIM_CONFIG.stickDeadzone;

        if (isPitchDeflected) {
            wasPitchDeflected = true;

            // PILOT MANEUVERING (Direct Mechanical Link):
            // The pilot directly moves the cyclic. Target attitude tracks current attitude.
            targetPitch = currentPitch;

            // Parallel trim motor migration:
            // Holding forward stick (negative _rawPitch) slews trim forward.
            // Holding aft stick (positive _rawPitch) slews trim aft.
            // Slew is additive in the direction of deflection; it does not decay when stick centers.
            let trimSlew = _rawPitch * (ATRIM_CONFIG.trimMigrationRate * dt);
            trimPitch = clamp(trimPitch + trimSlew, -ATRIM_CONFIG.maxPitchTrim, ATRIM_CONFIG.maxPitchTrim);

            // Direct cyclic command = pilot stick + current trim datum
            outputPitch = clamp(_rawPitch + trimPitch, -1, 1);

        } else {
            // BUMPLESS HANDOVER ON STICK RELEASE:
            // The moment pilot releases cyclic into deadzone, capture the commanded cyclic
            // into trimPitch and lock the current attitude. Zero cyclic drop, zero flapback snap!
            if (wasPitchDeflected) {
                wasPitchDeflected = false;
                targetPitch = currentPitch;
                trimPitch = clamp(outputPitch, -ATRIM_CONFIG.maxPitchTrim, ATRIM_CONFIG.maxPitchTrim);
                lastPitch = currentPitch;
            }

            // HANDS-OFF ATTITUDE RETENTION:
            // Pilot released stick -> hold target pitch attitude hands-off
            let pitchError = targetPitch - currentPitch;

            // Integral accumulates steady forward cyclic required to counter rotor flapback
            trimPitch -= (pitchError * ATRIM_CONFIG.pitch_Ki) * dt;
            trimPitch = clamp(trimPitch, -ATRIM_CONFIG.maxPitchTrim, ATRIM_CONFIG.maxPitchTrim);

            // Proportional and damping correction around trim datum
            let pCorr = -(pitchError * ATRIM_CONFIG.pitch_Kp);
            let dCorr = pitchRate * ATRIM_CONFIG.pitch_Kd;
            let correction = pCorr + dCorr;

            outputPitch = clamp(trimPitch + correction, -1, 1);
        }
    }

    function flightLoop() {
        updateATRIM();
        animationFrameId = requestAnimationFrame(flightLoop);
    }

    // ----------------------------------------
    // 5. PROPERTY INTERCEPTION
    // ----------------------------------------
    /**
     * Intercepts geofs.animation.values.pitch:
     * Feeds the trimmed mechanical pitch control position directly to:
     * - Manual flight (EC-135 3D cyclic pitch animation evaluates values.pitch)
     * - eurocopter_sas.js (reads values.pitch; damps pitch rate on top of A.TRIM)
     * Roll and Yaw remain 100% direct mechanical controls (unaltered by A.TRIM).
     */
    function installInterceptors() {
        if (!geofs.animation || !geofs.animation.values) return;

        const vals = geofs.animation.values;

        if (typeof vals.pitch === 'number') _rawPitch = vals.pitch;

        // Pitch interceptor: Direct stick + A.TRIM datum
        Object.defineProperty(vals, 'pitch', {
            get: function() {
                if (!isEC135 || !atrimEnabled) return _rawPitch;
                return clamp(outputPitch, -1, 1);
            },
            set: function(v) {
                _rawPitch = v;
            },
            configurable: true,
            enumerable: true
        });

        console.log("[EC-135 A.TRIM] Mechanical pitch interceptor installed. Roll and Yaw are 100% direct pilot controls.");
    }

    // ----------------------------------------
    // 6. TOGGLE LISTENER
    // ----------------------------------------
    function hookToggleKey() {
        document.addEventListener("keydown", function(event) {
            if (!isEC135) return;
            if (document.activeElement.tagName === "INPUT" || document.activeElement.tagName === "TEXTAREA") return;

            if (event.key.toLowerCase() === ATRIM_CONFIG.toggleKey.toLowerCase() && !event.shiftKey && !event.ctrlKey && !event.altKey) {
                atrimEnabled = !atrimEnabled;
                if (atrimEnabled) {
                    targetPitch = (geofs.animation && geofs.animation.values) ? (geofs.animation.values.atilt || null) : null;
                    trimPitch = clamp(_rawPitch, -ATRIM_CONFIG.maxPitchTrim, ATRIM_CONFIG.maxPitchTrim);
                    wasPitchDeflected = false;
                    showNotification("EC-135 A.TRIM: ENGAGED");
                } else {
                    trimPitch = 0;
                    outputPitch = _rawPitch;
                    wasPitchDeflected = false;
                    showNotification("EC-135 A.TRIM: DISENGAGED (Raw Flight)");
                }
            }
        });
    }

    // ----------------------------------------
    // 7. INITIALIZATION & MONITORING
    // ----------------------------------------
    console.log("EC-135 A.TRIM Script waiting for GeoFS...");

    let waitForReady = setInterval(function() {
        if (typeof geofs !== 'undefined' && geofs.aircraft && geofs.aircraft.instance && geofs.animation && geofs.animation.values) {
            clearInterval(waitForReady);

            installInterceptors();
            hookToggleKey();
            animationFrameId = requestAnimationFrame(flightLoop);

            let lastAircraftId = geofs.aircraft.instance.id;
            isEC135 = checkIsEC135();

            if (isEC135) {
                console.log("EC-135 detected. A.TRIM Pitch ready & active. Press 'Z' to toggle.");
            } else {
                console.log("Current aircraft is not an EC-135. A.TRIM will activate when you switch to the EC-135.");
            }

            // Aircraft change monitor
            setInterval(function() {
                try {
                    if (!geofs.aircraft || !geofs.aircraft.instance) return;

                    let currentId = geofs.aircraft.instance.id;
                    if (currentId !== lastAircraftId) {
                        lastAircraftId = currentId;
                        isEC135 = checkIsEC135();

                        if (isEC135) {
                            installInterceptors();
                            targetPitch = null;
                            trimPitch = 0;
                            console.log("EC-135 detected. A.TRIM active.");
                        } else {
                            trimPitch = 0;
                            targetPitch = null;
                            outputPitch = _rawPitch;
                        }
                    }
                } catch (e) {
                    // Silently catch errors
                }
            }, 2000);
        }
    }, 1000);

})();
