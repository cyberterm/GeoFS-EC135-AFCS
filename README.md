# GeoFS-eurocopter-stability-assist

This repository includes a couple of scripts aiming to enhance the flying experience of the Eurocopter in GeoFS.

### eurocopter_fbw.js
This is like flying with training wheels. The new system takes over control and tries to match the attitude of the helicopter with your stick position. It's very similar to the angle mode drones use.  

### eurocopter_sas.js
This is closer to what the real thing has. It's a system trying to damp any movement, making the helicopter much more stable.

Both scripts include some basic yaw damping. Note that both systems start disarmed by default; press Caps Lock to toggle them on or off during flight.

### eurocopter_ap.js
A cruise-style autopilot for the EC-135. When engaged, it captures the current altitude (rounded to the nearest 100 ft) and heading, then holds both using PID controllers — altitude is maintained through pitch, heading through yaw, and roll is kept level. The collective/throttle remains fully under pilot control, acting like a speed lever. Press 'A' to engage/disengage.


Note on Compatibility:
You can install the Autopilot alongside either SAS or FBW, but only one system should be engaged at a time. Before turning on the Autopilot (A), make sure SAS or FBW is disengaged (Caps Lock), and vice versa. Having two systems active simultaneously will cause them to fight for control. (Note: Do not install SAS and FBW together, as both share the same toggle key).