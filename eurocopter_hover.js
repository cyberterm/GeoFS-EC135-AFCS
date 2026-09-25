// ==UserScript==
// @name         GeoFS EC-135 Hover Assist (Auto-Hover)
// @namespace    https://github.com/cyberterm
// @version      2026-09-25
// @description  Auto-Hover & Attitude Hold system for GeoFS EC-135. Self-leveling angle mode for hands-off hovering. Toggle with 'G'.
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

    const HOVER_CONFIG = {
        // Pitch Angle Mode
        pitchSensitivity: 20,     // Max target pitch angle in degrees (full stick)
        Kp_pitch: 0.025,          // Auto-leveling spring strength
        Kd_pitch: 0.45,           // Pitch rate damping

        // Roll Angle Mode
        rollSensitivity: 20,      // Max target roll angle in degrees (full stick)
        Kp_roll: 0.025,           // Auto-leveling spring strength
        Kd_roll: 0.45,            // Roll rate damping

        // Yaw Heading Hold in Hover
        yawDeadzone: 0.05,        // Pedals deadzone to capture & hold hover heading
        yaw_Kp: 0.025,            // Heading hold proportional gain
        yaw_Kd: 0.35,             // Yaw rate damping gain
        maxYawCmd: 0.40,          // Max yaw command limit

        // Master Toggle Key
        toggleKey: 'g'            // Press 'G' to switch between Hover and Realistic Stack
    };

    // Shared global state for stack coordination
    window._ec135 = window._ec135 || {};
    window._ec135.hoverActive = false;

    // State Tracking
    let hoverActive = false;      // Inactive by default
    let isEC135 = false;
    let targetHeading = null;
    let lastHeading = 0;
    let lastPitch = 0;
    let lastRoll = 0;
    let wasApActive = false;
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
        console.log("[EC-135 HOVER] " + msg);
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
            }, 2000);
        } catch (e) {
            // Silently ignore
        }
    }

    // ----------------------------------------
    // 3. CORE HOVER CONTROL LOOP
    // ----------------------------------------
    function updateHover() {
        if (!hoverActive || !isEC135) return;

        // Auto-yield to Autopilot if AP takes over
        let apActive = isAutopilotActive();
        if (apActive) {
            wasApActive = true;
            lastHeading = 0;
            lastPitch = 0;
            lastRoll = 0;
            targetHeading = null;
            return;
        }

        // When AP disengages while Hover is still armed, re-hook controls
        if (wasApActive) {
            wasApActive = false;
            hookHoverControls();
            lastHeading = 0;
            lastPitch = 0;
            lastRoll = 0;
            targetHeading = null;
        }

        try {
            const vals = geofs.animation.values;

            let currentHeading = vals.heading360 || 0;
            let currentPitch = vals.atilt || 0;    // Positive = nose down, negative = nose up
            let currentRoll = vals.aroll || 0;      // Positive = right bank, negative = left bank

            // Raw pilot inputs (if A.TRIM is loaded, vals.pitch returns raw pilot stick during hover)
            let pitchInput = vals.pitch || 0;
            let rollInput = vals.roll || 0;
            let yawInput = vals.yaw || 0;

            if (lastHeading === 0 && lastPitch === 0 && lastRoll === 0) {
                lastHeading = currentHeading;
                lastPitch = currentPitch;
                lastRoll = currentRoll;
            }

            let pitchRate = currentPitch - lastPitch;
            let rollRate = currentRoll - lastRoll;
            let yawRate = wrapAngle(currentHeading - lastHeading);

            // ==========================================
            // 1. PITCH: AUTO-LEVELING ANGLE MODE
            // ==========================================
            // When stick is centered (pitchInput == 0), pitchTarget is 0° (level hover)
            // When stick is pushed forward (pitchInput < 0), pitchTarget is positive (nose down)
            let pitchTarget = HOVER_CONFIG.pitchSensitivity * -pitchInput;
            let pitchError = pitchTarget - currentPitch;
            let pitchCmd = -((pitchError * HOVER_CONFIG.Kp_pitch) - (pitchRate * HOVER_CONFIG.Kd_pitch));
            vals.fbwPitch = clamp(pitchCmd, -1.0, 1.0);

            // ==========================================
            // 2. ROLL: AUTO-LEVELING ANGLE MODE
            // ==========================================
            // When stick is centered (rollInput == 0), rollTarget is 0° (wings level)
            // When stick is deflected right (rollInput > 0), rollTarget is negative (right bank)
            let rollTarget = HOVER_CONFIG.rollSensitivity * -rollInput;
            let rollError = rollTarget - currentRoll;
            let rollCmd = -((rollError * HOVER_CONFIG.Kp_roll) - (rollRate * HOVER_CONFIG.Kd_roll));
            vals.fbwRoll = clamp(rollCmd, -0.6, 0.6);

            // ==========================================
            // 3. YAW: PEDAL HEADING HOLD IN HOVER
            // ==========================================
            let isPedalDeflected = Math.abs(yawInput) > HOVER_CONFIG.yawDeadzone;

            if (isPedalDeflected) {
                // Pilot commanding turn: follow pedal input directly with rate damping
                targetHeading = currentHeading;
                vals.fbwYaw = clamp(yawInput - (yawRate * (HOVER_CONFIG.yaw_Kd * 0.1)), -1.0, 1.0);
            } else {
                // Pedals centered: lock & hold current heading
                if (targetHeading === null) targetHeading = currentHeading;
                let hdgError = wrapAngle(targetHeading - currentHeading);
                let yawCmd = (hdgError * HOVER_CONFIG.yaw_Kp) - (yawRate * (HOVER_CONFIG.yaw_Kd * 0.1));
                vals.fbwYaw = clamp(yawCmd, -HOVER_CONFIG.maxYawCmd, HOVER_CONFIG.maxYawCmd);
            }

            // Save states
            lastHeading = currentHeading;
            lastPitch = currentPitch;
            lastRoll = currentRoll;

        } catch (error) {
            // Silently catch errors
        }
    }

    function flightLoop() {
        if (window.geofs && geofs.animation && geofs.animation.values) {
            updateHover();
        }
        animationFrameId = requestAnimationFrame(flightLoop);
    }

    // ----------------------------------------
    // 4. AIRCRAFT PART HOOKING
    // ----------------------------------------
    function hookHoverControls() {
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

    function unhookHoverControls() {
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
    // 5. MASTER TOGGLE & STACK COORDINATION
    // ----------------------------------------
    function setHoverState(enable) {
        hoverActive = enable;
        window._ec135.hoverActive = enable;

        if (hoverActive) {
            // Disengage Autopilot if active so Hover takes priority
            if (isAutopilotActive()) {
                let apBtn = document.querySelector(".geofs-autopilot-toggle.geofs-active");
                if (apBtn) apBtn.click();
            }

            lastHeading = 0;
            lastPitch = 0;
            lastRoll = 0;
            targetHeading = null;
            hookHoverControls();
            showNotification("EC-135 HOVER: ENGAGED (Auto-Level Active)");
            console.log("[EC-135 HOVER] Engaged. Self-leveling hover active.");
        } else {
            unhookHoverControls();

            // Clear outputs
            if (geofs.animation && geofs.animation.values) {
                geofs.animation.values.fbwPitch = 0;
                geofs.animation.values.fbwRoll = 0;
                geofs.animation.values.fbwYaw = 0;
            }

            showNotification("EC-135 HOVER: DISENGAGED (Realistic Stack Active)");
            console.log("[EC-135 HOVER] Disengaged. Returned to realistic flight stack.");
        }
    }

    function hookToggleKey() {
        document.addEventListener("keydown", function(event) {
            if (!isEC135) return;
            if (document.activeElement.tagName === "INPUT" || document.activeElement.tagName === "TEXTAREA") return;

            if (event.key.toLowerCase() === HOVER_CONFIG.toggleKey.toLowerCase() && !event.shiftKey && !event.ctrlKey && !event.altKey) {
                event.preventDefault();
                event.stopImmediatePropagation();
                setHoverState(!hoverActive);
            }
        }, true);
    }

    // ----------------------------------------
    // 6. INITIALIZATION & MONITORING
    // ----------------------------------------
    console.log("EC-135 Hover Script waiting for GeoFS...");

    let waitForReady = setInterval(function() {
        if (typeof geofs !== 'undefined' && geofs.aircraft && geofs.aircraft.instance && geofs.aircraft.instance.parts) {
            clearInterval(waitForReady);

            let lastAircraftId = geofs.aircraft.instance.id;
            isEC135 = checkIsEC135();

            hookToggleKey();
            animationFrameId = requestAnimationFrame(flightLoop);

            if (isEC135) {
                console.log("EC-135 detected. Hover Assist ready (Inactive by default). Press 'G' to engage Auto-Hover.");
            } else {
                console.log("Current aircraft is not an EC-135. Hover Assist ready for EC-135.");
            }

            // Aircraft change monitor
            setInterval(function() {
                try {
                    if (!geofs.aircraft || !geofs.aircraft.instance) return;

                    let currentId = geofs.aircraft.instance.id;
                    if (currentId !== lastAircraftId) {
                        lastAircraftId = currentId;

                        if (hoverActive) {
                            setHoverState(false);
                        }

                        isEC135 = checkIsEC135();
                    }
                } catch (e) {
                    // Silently catch errors
                }
            }, 2000);
        }
    }, 1000);

})();
