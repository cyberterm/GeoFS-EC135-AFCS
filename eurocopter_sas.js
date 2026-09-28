// ==UserScript==
// @name         GeoFS EC-135 SAS
// @namespace    https://github.com/cyberterm
// @version      2026-09-25
// @description  Stability Augmentation System for GeoFS EC-135. Active by default, auto-yields to Autopilot, with optional CapsLock disable.
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

    const SAS_CONFIG = {
        // Gyro Rate Damping (Physical units: cyclic / (deg/sec))
        // Frame-rate independent: identical damping feel across 30, 60, and 144+ FPS
        pitchDamping: 0.015,          // Cyclic pitch damping per deg/sec
        rollDamping: 0.015,           // Cyclic roll damping per deg/sec
        maxSemaPitch: 0.12,           // SEMA series authority limit (+/- 12% cyclic pitch)
        maxSemaRoll: 0.12,            // SEMA series authority limit (+/- 12% cyclic roll)

        // Rate-Command Yaw Damper (Dynamic Spin Cancellation)
        yawDampStrength: 0.018,       // Tail rotor damping per deg/sec
        yawRateSensitivity: 18.0,     // Commanded turn rate (deg/sec) at full pedal
        maxSemaYaw: 0.20,             // SEMA series authority limit (+/- 20% tail rotor)

        // Collective-to-Yaw Decoupler (Torque Anticipator / Mixing Unit)
        // Neutralizes the violent torque kick when pulling or lowering collective
        collectiveTorqueComp: 0.15,   // Tail rotor bias per unit/sec of collective movement
        maxTorqueComp: 0.15           // Max anticipator tail rotor authority
    };

    // State Tracking
    let isEC135 = false;
    let sasActive = true;       // Active by default (authentic EC-135 behavior)
    let lastHeading = 0;
    let lastPitch = 0;
    let lastRoll = 0;
    let lastThrottle = 0;
    let lastTime = performance.now();
    let animationFrameId;

    // ----------------------------------------
    // 2. HELPERS
    // ----------------------------------------
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

    function isAutopilotActive() {
        let btn = document.querySelector(".geofs-autopilot-toggle.geofs-active");
        return !!btn;
    }

    function showNotification(msg) {
        console.log("[EC-135 SAS] " + msg);
    }

    function isHoverActive() {
        return !!(window._ec135 && window._ec135.hoverActive);
    }

    let wasApActive = false;
    let wasHoverActive = false;

    // ----------------------------------------
    // 3. CORE MATH & DAMPING
    // ----------------------------------------
    function updateSAS() {
        if (!sasActive || !isEC135 || !geofs.animation || !geofs.animation.values) return;

        let apActive = isAutopilotActive();
        let hoverActive = isHoverActive();

        // Auto-yield to Autopilot or Hover Assist: let outer loop drive
        if (apActive || hoverActive) {
            if (apActive) wasApActive = true;
            if (hoverActive) wasHoverActive = true;
            lastHeading = 0;
            lastPitch = 0;
            lastRoll = 0;
            lastThrottle = 0;
            lastTime = performance.now();
            return;
        }

        // When AP or Hover disengages, re-hook parts back to "fbwPitch"/"fbwRoll"/"fbwYaw"
        if (wasApActive || wasHoverActive) {
            wasApActive = false;
            wasHoverActive = false;
            lastHeading = 0;
            lastPitch = 0;
            lastRoll = 0;
            lastThrottle = 0;
            lastTime = performance.now();
            hookSAS();
        }

        try {
            const vals = geofs.animation.values;
            let now = performance.now();
            let dt = (now - lastTime) / 1000;
            lastTime = now;

            // Read Current States (|| 0 prevents NaN crashes)
            let currentHeading = vals.heading360 || 0;
            let currentPitch = vals.atilt || 0;
            let currentRoll = vals.aroll || 0;
            let currentThrottle = vals.throttle || 0;

            let pitchInput = vals.pitch || 0;
            let rollInput = vals.roll || 0;
            let yawInput = vals.yaw || 0;

            // Guard against pause, tab switch, or first frame anomalies
            if (dt <= 0 || dt > 0.5 || (lastHeading === 0 && lastPitch === 0 && lastRoll === 0)) {
                lastHeading = currentHeading;
                lastPitch = currentPitch;
                lastRoll = currentRoll;
                lastThrottle = currentThrottle;
                return;
            }

            // ==========================================
            // 1. FRAME-RATE INVARIANT ANGULAR RATES (deg/sec)
            // ==========================================
            let pitchRateSec = (currentPitch - lastPitch) / dt;
            let rollRateSec = (currentRoll - lastRoll) / dt;
            let yawRateSec = wrapAngle(lastHeading - currentHeading) / dt;

            // ==========================================
            // 2. PITCH & ROLL SEMA RATE DAMPING
            // ==========================================
            let pitchCorrection = clamp(pitchRateSec * SAS_CONFIG.pitchDamping, -SAS_CONFIG.maxSemaPitch, SAS_CONFIG.maxSemaPitch);
            let rollCorrection = clamp(rollRateSec * SAS_CONFIG.rollDamping, -SAS_CONFIG.maxSemaRoll, SAS_CONFIG.maxSemaRoll);

            vals.fbwPitch = clamp(pitchInput + pitchCorrection, -1.0, 1.0);
            vals.fbwRoll = clamp(rollInput + rollCorrection, -1.0, 1.0);

            // ==========================================
            // 3. YAW DAMPER & COLLECTIVE ANTICIPATOR
            // ==========================================
            // Commanded turn rate from pilot pedals (deg/sec)
            let commandedRateSec = -yawInput * SAS_CONFIG.yawRateSensitivity;
            let rateErrorSec = yawRateSec - commandedRateSec;

            // SEMA gyro rate damping: actively cancels uncommanded spin & chatter
            let damperOutput = clamp(rateErrorSec * SAS_CONFIG.yawDampStrength, -SAS_CONFIG.maxSemaYaw, SAS_CONFIG.maxSemaYaw);

            // Collective-to-Yaw feedforward mixing: cancels torque kick during power changes
            let collectiveRate = (currentThrottle - lastThrottle) / dt;
            let torqueComp = clamp(collectiveRate * SAS_CONFIG.collectiveTorqueComp, -SAS_CONFIG.maxTorqueComp, SAS_CONFIG.maxTorqueComp);

            vals.fbwYaw = clamp(yawInput + damperOutput + torqueComp, -1.0, 1.0);

            // Save history states
            lastHeading = currentHeading;
            lastPitch = currentPitch;
            lastRoll = currentRoll;
            lastThrottle = currentThrottle;

        } catch (error) {
            // Silently catch errors
        }
    }

    function flightLoop() {
        if (window.geofs && geofs.animation && geofs.animation.values) {
            updateSAS();
        }
        animationFrameId = requestAnimationFrame(flightLoop);
    }

    // ----------------------------------------
    // 4. AIRCRAFT PART HOOKING
    // ----------------------------------------
    function hookSAS() {
        if (!geofs.aircraft || !geofs.aircraft.instance || !geofs.aircraft.instance.parts) return;
        const parts = geofs.aircraft.instance.parts;

        // Tailrotor (Yaw)
        if (parts.tailrotor && parts.tailrotor.animations[2]) parts.tailrotor.animations[2].value = "fbwYaw";

        // Cyclic Pitch (Index 0)
        if (parts.cyclicLeft && parts.cyclicLeft.animations[0]) parts.cyclicLeft.animations[0].value = "fbwPitch";
        if (parts.cyclicRight && parts.cyclicRight.animations[0]) parts.cyclicRight.animations[0].value = "fbwPitch";
        if (parts.cyclicRotorNegative && parts.cyclicRotorNegative.animations[0]) parts.cyclicRotorNegative.animations[0].value = "fbwPitch";
        if (parts.cyclicRotorPositive && parts.cyclicRotorPositive.animations[0]) parts.cyclicRotorPositive.animations[0].value = "fbwPitch";

        // Cyclic Roll (Index 1)
        if (parts.cyclicLeft && parts.cyclicLeft.animations[1]) parts.cyclicLeft.animations[1].value = "fbwRoll";
        if (parts.cyclicRight && parts.cyclicRight.animations[1]) parts.cyclicRight.animations[1].value = "fbwRoll";
        if (parts.cyclicRotorNegative && parts.cyclicRotorNegative.animations[1]) parts.cyclicRotorNegative.animations[1].value = "fbwRoll";
        if (parts.cyclicRotorPositive && parts.cyclicRotorPositive.animations[1]) parts.cyclicRotorPositive.animations[1].value = "fbwRoll";
    }

    function unhookSAS() {
        if (!geofs.aircraft || !geofs.aircraft.instance || !geofs.aircraft.instance.parts) return;
        const parts = geofs.aircraft.instance.parts;

        // Tailrotor (Yaw)
        if (parts.tailrotor && parts.tailrotor.animations[2]) parts.tailrotor.animations[2].value = "yaw";

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

    // ----------------------------------------
    // 5. INITIALIZATION & MONITORING
    // ----------------------------------------
    console.log("SAS Script waiting for GeoFS aircraft to load...");

    let waitForReady = setInterval(function() {
        if (typeof geofs !== 'undefined' && geofs.aircraft && geofs.aircraft.instance && geofs.aircraft.instance.parts) {
            clearInterval(waitForReady);

            let lastAircraftId = geofs.aircraft.instance.id;
            isEC135 = checkIsEC135();

            if (isEC135) {
                sasActive = true;
                hookSAS();
                showNotification("EC-135 SAS: ENGAGED (Armed by default)");
                console.log("EC-135 detected. SAS active by default. Press CapsLock to toggle.");
            } else {
                sasActive = false;
                console.log("Current aircraft is not an EC-135. SAS will activate when switching to the EC-135.");
            }

            animationFrameId = requestAnimationFrame(flightLoop);

            // Optional Keyboard Toggle
            document.addEventListener("keydown", function(event) {
                if (!isEC135) return;
                if (document.activeElement.tagName === "INPUT" || document.activeElement.tagName === "TEXTAREA") return;

                if (event.key === "CapsLock") {
                    sasActive = !sasActive;

                    if (sasActive) {
                        lastHeading = 0;
                        lastPitch = 0;
                        lastRoll = 0;
                        lastThrottle = 0;
                        lastTime = performance.now();
                        hookSAS();
                        showNotification("EC-135 SAS: ENGAGED");
                    } else {
                        unhookSAS();
                        showNotification("EC-135 SAS: DISENGAGED (Raw Flight)");
                    }
                }
            });

            // Aircraft change monitor
            setInterval(function() {
                try {
                    if (!geofs.aircraft || !geofs.aircraft.instance) return;

                    let currentId = geofs.aircraft.instance.id;
                    if (currentId !== lastAircraftId) {
                        lastAircraftId = currentId;
                        isEC135 = checkIsEC135();

                        if (isEC135) {
                            sasActive = true;
                            lastHeading = 0;
                            lastPitch = 0;
                            lastRoll = 0;
                            lastThrottle = 0;
                            lastTime = performance.now();
                            hookSAS();
                            showNotification("EC-135 SAS: ENGAGED");
                        } else {
                            sasActive = false;
                            unhookSAS();
                        }
                    }
                } catch (e) {
                    // Silently catch errors
                }
            }, 2000);
        }
    }, 1000);

})();
