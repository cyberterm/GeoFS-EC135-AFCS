// ==UserScript==
// @name         GeoFS EC-135 Autopilot
// @namespace    https://github.com/cyberterm
// @version      2026-09-20
// @description  Cruise Altitude & Heading Hold Autopilot for the EC-135
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

    const AP_CONFIG = {
        // Outer Loop: Altitude Error (ft) -> Target V/S (ft/min)
        alt_Kp: 3.0,                 // 30 ft error -> 90 ft/min target V/S
        alt_vsMax: 500,              // Max target vertical speed (ft/min)

        // Middle Loop: V/S Error (ft/min) -> Target Pitch Attitude (degrees)
        // Integrates V/S error to hold the required cruise nose-down angle for current throttle
        vs_Kp: 0.01,                // Proportional pitch response to immediate V/S changes
        vs_Ki: 0.003,                // Integral trim rate: builds nose-down cruise pitch
        pitchMin: -15,               // Max climb attitude (degrees)
        pitchMax: 30,                // Max cruise nose-down dip (degrees)

        // Inner Loop: Target Pitch (degrees) -> Cyclic Pitch (fbwPitch)
        pitch_Kp: 0.04,              // Cyclic deflection per degree of pitch error
        pitch_Kd: 0.8,              // Pitch rate damping (prevents oscillations)
        cyclicPitchMax: 1.0,        // Max cyclic pitch command

        // Heading Hold PID -> Tail Rotor (fbwYaw)
        hdg_Kp: 0.015,
        hdg_Ki: 0.0003,
        hdg_Kd: 0.4,
        hdg_integralMax: 20,
        hdg_outputMax: 0.35,

        // Roll Hold PD -> Wings Level (fbwRoll)
        roll_Kp: 0.02,
        roll_Kd: 0.4,
        roll_outputMax: 0.35
    };

    // ----------------------------------------
    // 2. STATE
    // ----------------------------------------
    let apActive = false;
    let isEC135 = false;
    let animationFrameId;

    // AP Targets (captured on engagement)
    let targetAltitude = 0;   // feet
    let targetHeading = 0;    // degrees (0-360)

    // Pitch & V/S Controller State
    let integratedPitch = 0;  // Cruise trim pitch angle (degrees)
    let lastPitch = 0;        // degrees (atilt)
    let lastRoll = 0;         // degrees (aroll)
    let lastTime = 0;         // performance.now() timestamp

    // Heading Controller State
    let hdg_integral = 0;
    let hdg_lastError = 0;

    // ----------------------------------------
    // 3. MATH HELPERS
    // ----------------------------------------
    function clamp(value, min, max) {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * Normalize an angle difference to the range [-180, 180].
     */
    function wrapAngle(delta) {
        while (delta > 180) delta -= 360;
        while (delta < -180) delta += 360;
        return delta;
    }

    // ----------------------------------------
    // 4. CORE AUTOPILOT LOOP
    // ----------------------------------------
    function updateAP() {
        if (!apActive) return;

        try {
            const vals = geofs.animation.values;
            let now = performance.now();
            let dt = (now - lastTime) / 1000; // seconds
            lastTime = now;

            // Guard against pause, tab switch, or first frame anomalies
            if (dt <= 0 || dt > 0.5) {
                lastPitch = vals.atilt || 0;
                lastRoll = vals.aroll || 0;
                return;
            }

            // --- Current readings ---
            let currentAltFeet = vals.altThousands || vals.altitude || 0;
            let currentHeading = vals.heading360 || 0;
            let currentPitch = vals.atilt || 0;
            let currentRoll = vals.aroll || 0;
            let currentVS = vals.climbrate || 0; // Direct from GeoFS physics (ft/min)

            // ==========================================
            // 1. OUTER LOOP: Altitude Error -> Target V/S
            // ==========================================
            let altError = targetAltitude - currentAltFeet;
            let targetVS = clamp(altError * AP_CONFIG.alt_Kp, -AP_CONFIG.alt_vsMax, AP_CONFIG.alt_vsMax);

            // ==========================================
            // 2. MIDDLE LOOP: V/S Error -> Target Pitch Attitude
            // ==========================================
            let vsError = targetVS - currentVS; // ft/min
            // Climbing (vsError < 0) -> dip nose down (increase target pitch)
            // Sinking (vsError > 0) -> raise nose up (decrease target pitch)
            integratedPitch -= (vsError * AP_CONFIG.vs_Ki) * dt;
            integratedPitch = clamp(integratedPitch, AP_CONFIG.pitchMin, AP_CONFIG.pitchMax);

            let targetPitch = integratedPitch - (vsError * AP_CONFIG.vs_Kp);
            targetPitch = clamp(targetPitch, AP_CONFIG.pitchMin, AP_CONFIG.pitchMax);

            // ==========================================
            // 3. INNER LOOP: Pitch Tracking -> Cyclic Pitch (fbwPitch)
            // ==========================================
            let pitchError = targetPitch - currentPitch;
            let pitchRate = currentPitch - lastPitch;
            lastPitch = currentPitch;

            // GeoFS: negative fbwPitch = nose-up, positive fbwPitch = nose-down
            let pitchCmd = -((pitchError * AP_CONFIG.pitch_Kp) - (pitchRate * AP_CONFIG.pitch_Kd));
            vals.fbwPitch = clamp(pitchCmd, -AP_CONFIG.cyclicPitchMax, AP_CONFIG.cyclicPitchMax);

            // ==========================================
            // 4. ROLL HOLD: Wings Level -> Cyclic Roll (fbwRoll)
            // ==========================================
            let rollError = 0 - currentRoll;
            let rollRate = currentRoll - lastRoll;
            lastRoll = currentRoll;

            let rollCmd = -((rollError * AP_CONFIG.roll_Kp) - (rollRate * AP_CONFIG.roll_Kd));
            vals.fbwRoll = clamp(rollCmd, -AP_CONFIG.roll_outputMax, AP_CONFIG.roll_outputMax);

            // ==========================================
            // 5. HEADING HOLD: Target Heading -> Tail Rotor (fbwYaw)
            // ==========================================
            let hdgError = wrapAngle(targetHeading - currentHeading);
            hdg_integral += hdgError * dt;
            hdg_integral = clamp(hdg_integral, -AP_CONFIG.hdg_integralMax, AP_CONFIG.hdg_integralMax);

            let hdgDerivative = (hdgError - hdg_lastError);
            hdg_lastError = hdgError;

            let hdgOutput = (AP_CONFIG.hdg_Kp * hdgError)
                          + (AP_CONFIG.hdg_Ki * hdg_integral)
                          + (AP_CONFIG.hdg_Kd * hdgDerivative);

            vals.fbwYaw = clamp(hdgOutput, -AP_CONFIG.hdg_outputMax, AP_CONFIG.hdg_outputMax);

            // Throttle is NOT touched — collective is fully pilot-controlled!

        } catch (error) {
            // Silently catch errors
        }
    }

    function flightLoop() {
        if (window.geofs && geofs.animation && geofs.animation.values) {
            updateAP();
        }
        animationFrameId = requestAnimationFrame(flightLoop);
    }

    // ----------------------------------------
    // 5. AIRCRAFT PART HOOKING
    // ----------------------------------------
    function hookAPControls() {
        const parts = geofs.aircraft.instance.parts;

        // Tail Rotor (Yaw)
        if (parts.tailrotor && parts.tailrotor.animations[2]) {
            parts.tailrotor.animations[2].value = "fbwYaw";
        }

        // Cyclic Pitch (Index 0) -> fbwPitch
        if (parts.cyclicLeft && parts.cyclicLeft.animations[0]) parts.cyclicLeft.animations[0].value = "fbwPitch";
        if (parts.cyclicRight && parts.cyclicRight.animations[0]) parts.cyclicRight.animations[0].value = "fbwPitch";
        if (parts.cyclicRotorNegative && parts.cyclicRotorNegative.animations[0]) parts.cyclicRotorNegative.animations[0].value = "fbwPitch";
        if (parts.cyclicRotorPositive && parts.cyclicRotorPositive.animations[0]) parts.cyclicRotorPositive.animations[0].value = "fbwPitch";

        // Cyclic Roll (Index 1) -> fbwRoll
        if (parts.cyclicLeft && parts.cyclicLeft.animations[1]) parts.cyclicLeft.animations[1].value = "fbwRoll";
        if (parts.cyclicRight && parts.cyclicRight.animations[1]) parts.cyclicRight.animations[1].value = "fbwRoll";
        if (parts.cyclicRotorNegative && parts.cyclicRotorNegative.animations[1]) parts.cyclicRotorNegative.animations[1].value = "fbwRoll";
        if (parts.cyclicRotorPositive && parts.cyclicRotorPositive.animations[1]) parts.cyclicRotorPositive.animations[1].value = "fbwRoll";
    }

    function unhookAPControls() {
        const parts = geofs.aircraft.instance.parts;

        // Tail Rotor
        if (parts.tailrotor && parts.tailrotor.animations[2]) {
            parts.tailrotor.animations[2].value = "yaw";
        }

        // Cyclic Pitch
        if (parts.cyclicLeft && parts.cyclicLeft.animations[0]) parts.cyclicLeft.animations[0].value = "pitch";
        if (parts.cyclicRight && parts.cyclicRight.animations[0]) parts.cyclicRight.animations[0].value = "pitch";
        if (parts.cyclicRotorNegative && parts.cyclicRotorNegative.animations[0]) parts.cyclicRotorNegative.animations[0].value = "pitch";
        if (parts.cyclicRotorPositive && parts.cyclicRotorPositive.animations[0]) parts.cyclicRotorPositive.animations[0].value = "pitch";

        // Cyclic Roll
        if (parts.cyclicLeft && parts.cyclicLeft.animations[1]) parts.cyclicLeft.animations[1].value = "roll";
        if (parts.cyclicRight && parts.cyclicRight.animations[1]) parts.cyclicRight.animations[1].value = "roll";
        if (parts.cyclicRotorNegative && parts.cyclicRotorNegative.animations[1]) parts.cyclicRotorNegative.animations[1].value = "roll";
        if (parts.cyclicRotorPositive && parts.cyclicRotorPositive.animations[1]) parts.cyclicRotorPositive.animations[1].value = "roll";
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

    // ----------------------------------------
    // 6. ENGAGE / DISENGAGE
    // ----------------------------------------
    function engageAP() {
        const vals = geofs.animation.values;

        // Capture current altitude (rounded to nearest 100ft) and heading
        let currentAltFeet = vals.altThousands || vals.altitude || 0;
        targetAltitude = Math.round(currentAltFeet / 100) * 100;
        targetHeading = Math.round(vals.heading360 || 0);

        // Reset Heading PID state
        hdg_integral = 0;
        hdg_lastError = 0;

        // Bump-free engagement: start cruise trim from current pitch attitude
        let currentPitch = vals.atilt || 0;
        integratedPitch = clamp(currentPitch, AP_CONFIG.pitchMin, AP_CONFIG.pitchMax);
        lastPitch = currentPitch;
        lastRoll = vals.aroll || 0;
        lastTime = performance.now();

        hookAPControls();
        apActive = true;
        updateUIButtonState(true);

        console.log("EC-135 AP: ENGAGED — ALT " + targetAltitude + "ft, HDG " + targetHeading + "° (Collective is speed control)");
    }

    function disengageAP() {
        apActive = false;
        unhookAPControls();

        // Clear AP outputs
        if (geofs.animation && geofs.animation.values) {
            geofs.animation.values.fbwPitch = 0;
            geofs.animation.values.fbwRoll = 0;
            geofs.animation.values.fbwYaw = 0;
        }

        updateUIButtonState(false);
        console.log("EC-135 AP: DISENGAGED");
    }

    // ----------------------------------------
    // 7. EC-135 DETECTION & AUTOPILOT BUTTON HOOK
    // ----------------------------------------
    function checkIsEC135() {
        try {
            return String(geofs.aircraft.instance.id) === String(EC135_ID);
        } catch (e) {
            return false;
        }
    }

    /**
     * Intercept the autopilot toggle so it drives our AP on the EC-135.
     * We listen for the 'A' keypress and also hook the autopilot toggle button.
     */
    function hookAutopilotButton() {
        // Intercept the 'A' key (GeoFS default autopilot toggle)
        document.addEventListener("keydown", function(event) {
            if (!isEC135) return;
            if (document.activeElement.tagName === "INPUT") return;

            if (event.key === "a" || event.key === "A") {
                event.stopImmediatePropagation();
                event.preventDefault();

                if (apActive) {
                    disengageAP();
                } else {
                    engageAP();
                }
            }
        }, true); // Use capture phase to intercept before GeoFS

        // Also intercept clicks on the autopilot toggle button
        document.addEventListener("click", function(event) {
            if (!isEC135) return;

            let target = event.target;
            if (target && target.closest && target.closest(".geofs-autopilot-toggle")) {
                event.stopImmediatePropagation();
                event.preventDefault();

                if (apActive) {
                    disengageAP();
                } else {
                    engageAP();
                }
            }
        }, true); // Use capture phase to intercept before GeoFS
    }

    // ----------------------------------------
    // 8. INITIALIZATION
    // ----------------------------------------
    console.log("EC-135 AP Script waiting for GeoFS to load...");
    let waitForReady = setInterval(function() {
        if (typeof geofs !== 'undefined' && geofs.aircraft && geofs.aircraft.instance && geofs.aircraft.instance.parts) {
            clearInterval(waitForReady);

            // Start the flight loop (always runs, updateAP checks apActive)
            animationFrameId = requestAnimationFrame(flightLoop);

            // Hook the autopilot button once (handlers check isEC135 internally)
            hookAutopilotButton();

            // Check initial aircraft
            let lastAircraftId = geofs.aircraft.instance.id;
            isEC135 = checkIsEC135();
            if (isEC135) {
                console.log("EC-135 detected. Autopilot system ready (Disarmed). Press 'A' or use the autopilot button to engage.");
            } else {
                console.log("Current aircraft is not an EC-135. AP will activate when you switch to the EC-135.");
            }

            // Continuously monitor for aircraft changes
            setInterval(function() {
                try {
                    if (!geofs.aircraft || !geofs.aircraft.instance) return;

                    let currentId = geofs.aircraft.instance.id;
                    if (currentId !== lastAircraftId) {
                        lastAircraftId = currentId;

                        // Disengage if AP was active on the previous aircraft
                        if (apActive) {
                            disengageAP();
                        }

                        isEC135 = checkIsEC135();

                        if (isEC135) {
                            console.log("EC-135 detected. Autopilot system ready (Disarmed).");
                        } else {
                            console.log("Aircraft changed — not an EC-135. AP system inactive.");
                        }
                    }
                } catch (e) {
                    // Silently catch errors
                }
            }, 2000);
        }
    }, 1000);

})();
