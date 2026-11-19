# FallGuard

A fall detection system that sends a robot to come help you.

My MacBook camera tracks my body and draws a skeleton over me in Processing. If I fall and stay down, it sends my Arduino tank across my dorm room to me. The tank has a red cup on top so the camera can find it, and the sketch steers it to my head over Bluetooth.

![demo](screenshots/demo.gif)

(That's the demo mode that plays when the camera isn't connected.)

## What it does

- Green skeleton when you're standing normally
- Yellow/orange when your limbs bend, your torso starts tipping, or your hips drop fast
- Red when your torso has been basically horizontal for almost a second
- After 3 seconds of red it dispatches the rescue tank (press C to cancel if you're fine)

## How it's put together

There are four pieces:

1. `pose_bridge.py` reads the camera and runs MediaPipe Pose to get 33 body points every frame. It also finds the colored marker on the tank with OpenCV. It sends all of that to Processing over UDP on localhost.
2. `FallGuard/FallGuard.pde` is the Processing sketch. It draws everything, works out joint angles and torso tilt, decides if you've fallen, and drives the tank.
3. `robot_link.py` connects to the tank's HM-10 Bluetooth module and passes drive commands along.
4. `rescue_tank/rescue_tank.ino` runs on the tank and just listens for single-letter commands (F, B, L, R, S and a few others). It stops by itself if it doesn't hear anything for 0.7 seconds, so it can't run off if the connection drops.

When you press Run in Processing, the sketch starts the Python bridge itself, and the bridge starts the Bluetooth link, so you don't need Terminal open.

## How the tank finds you

This was the hardest part and took most of the trial and error.

With one marker on the tank, the camera knows where the tank is but not which way it's facing. My first versions tried to steer constantly, and the tank kept overshooting and swinging back and forth. Part of that is because my tank has loose tracks and pulls to one side.

What ended up working is pretty simple:

1. Drive straight for a short stretch
2. Look at which way the marker actually moved
3. If that line is heading at your head, keep going
4. If not, turn a little (about half of what it thinks it needs) and try again

It learns how far its turns go as it drives, so the corrections get better. If it drives but the marker doesn't move, it assumes it hit something, backs up, turns, and goes around.

It aims for your head. If your face is hidden it uses a point just past your shoulders. If you press T to test while standing up, it drives to your feet instead.

You can also add a second marker in a different color at the back of the tank (press B). Then it knows its direction right away without having to move first.

## What you need

- A Mac with a camera
- Keyestudio Mini Tank Robot V2 (KS0428). Any Arduino Uno robot with an HM-10 should work with some pin changes.
- Something bright to put on the tank. I use a red cup. There's also a phone wallpaper in `screenshots/tank-marker-phone.png` you can use as a two-color marker.
- Processing 4
- Python 3.10 to 3.12
- Arduino IDE with Arduino AVR Boards installed

## Setup

**Tank**

1. Unplug the Bluetooth module first or the upload fails
2. Plug the tank in over USB and open `rescue_tank/rescue_tank.ino`
3. Tools > Board > Arduino Uno, Tools > Port > /dev/cu.usbserial-something
4. Upload, unplug USB, put the Bluetooth module back (VCC to 5V, GND to GND), switch it on

My tank had the left and right motor pins swapped from what Keyestudio's docs say, and both tracks were reversed, so the code is set up for that. If yours drives wrong, connect it and press 1, 2, 3 and 4 in the sketch. Each one spins one track for a second. Then change `LEFT_FLIP`, `RIGHT_FLIP` and `PWM_INVERTS` at the top of the .ino.

**Sketch**

1. Open `FallGuard/FallGuard.pde` in Processing and hit Run
2. First run installs the Python packages by itself (mediapipe, opencv-python, bleak), which takes a couple minutes. Allow Processing to use the camera and Bluetooth when macOS asks.
3. Wait for "rescue tank: connected to HMSoft" at the bottom. Drive with the arrow keys to make sure it works.
4. Press M and click the marker on the tank
5. Back up so your whole body is in the shot, then lie down (or press T)

## Controls

- Arrow keys: drive the tank yourself
- M: click the marker on the tank
- B: optional second marker on the back
- T: test a rescue
- C: cancel the alarm
- X: flip steering if the tank turns away from you instead of toward you
- 1-4: motor test
- K: switch cameras (my Mac kept grabbing my iPhone through Continuity Camera)
- V: camera image on/off
- R: restart the camera bridge
- D: demo mode
- S: screenshot

## Problems I ran into

- **Stuck in demo mode:** read the yellow "camera:" text in the bottom left. Usually it's a missing package or camera permission.
- **It opens the iPhone camera:** press K, or turn off Continuity Camera on the phone.
- **"searching for the tank":** turn the tank off and on. Also make sure you don't have two FallGuard windows running, because they fight over the Bluetooth connection.
- **The tank doesn't show up in Bluetooth settings:** that's normal. The HM-10 is Bluetooth Low Energy and macOS doesn't list it there. robot_link.py finds it by name.
- **The ring locks onto the wrong thing:** click right on the marker. Don't use a color close to your floor or walls. Yellow didn't work for me at all because of my wood floor.
- **Arrow keys do nothing:** click the sketch window first.
- **The tank keeps getting stuck:** charge the batteries and keep it off rug edges.

## Limitations

- The camera has to see your whole body and the tank
- Lying on your bed looks the same as lying on the floor
- It's a fun project, not a real safety device

## Built with

Processing, MediaPipe Pose Landmarker, OpenCV, bleak, Arduino

MIT license, so feel free to use it or build your own version.
