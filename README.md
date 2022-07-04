# Landing Throttle Manager

Manager for throttle when landing in X-Plane. Intended for use in VR with X-Crafts ERJ family.

Tested with X-Plane 11.55 and X-Crafts ERJ-145 1.4.2.

This plugin will manage the throttle and reverse thrust during landing which helps when using VR as the pilot can concentrate on getting the threshold speed right and looking out of the window at the runway, rather than fumbling for the throttle levers and the reverse-thrust hotspot.

When the plugin is enabled (e.g. using a configured button) it checks if the landing conditions are met, for example less than 160KIAS, flaps at 18+ deg, gears are down
and 500ft or less above ground.
If the conditions are met it will reduce throttle to idle and then wait for all wheels to be on the ground.
When all wheels are down reverse thrust is applied until a speed of 60KIAS is reached at which point reverse thrust is disabled and the throttle returned to idle.
If the conditions are not met to enable the plugin then voice guidance will be given as to which conditions are not being met

## Installation

Copy the folder Release\plugins\LandingThrottleManager to the X-Plane Resources\plugins folder so that you have Resources\plugins\LandingThrottleManager

## Configuration

Enable VR and go to the Joystick settings. Choose a button to use, e.g. pressing down the right thumbstick.
Choose Edit and then search for 'Enable for Landing Throttle Manager'

## Use

After crossing the runway threshold get to the desired height and press the configured button. The throttle will be smoothly reduced to idle. Glide the aircraft down onto the runway and lower the nose wheel onto the ground. Reverse thrust will be automatically applied and then removed at 60KIAS.

While the plugin is controlling the throttle the levers cannot be used. It shouldn't be necessary, but if needed to immediately stop the plugin and release it's control of the throttle go to the X-Plane Plugins menu and choose Landing Throttle Manager -> Stop and Disable
