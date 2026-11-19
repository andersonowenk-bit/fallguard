/*
  FallGuard
  ---------
  Pose-based fall detection visualizer with a rescue-robot hook.

  How it works
    When you press Run, this sketch starts pose_bridge.py in the background.
    The bridge opens the MacBook camera, runs MediaPipe Pose, and sends
    33 body landmarks (UDP :12000) plus the camera picture (UDP :12002)
    back to this sketch, which draws you with the skeleton on top.
    This sketch smooths the landmarks, measures joint angles and torso tilt,
    and colors every limb:
      GREEN   standing, limbs straight
      YELLOW  limbs bending, torso tipping, or hips dropping fast
      RED     torso horizontal (on the ground) for a moment
    After DISPATCH_DELAY_S seconds of RED it calls dispatchRobot(), which
    sends "RESCUE" back to the bridge on port 12001 (the bridge will relay
    it to the Keyestudio tank in step 2).

  First run installs the tracking packages by itself (1 to 3 minutes) and
  macOS asks to let Processing use the camera. Until the camera is ready,
  the sketch plays a DEMO loop.

  Keys
    D  toggle demo mode        C  cancel the alarm ("I'm OK")
    T  test-fire a dispatch    S  save a screenshot to /screenshots
    V  camera picture on/off   R  restart the camera bridge
    K  switch to the next camera (if it opens your iPhone instead of the Mac)
    Arrow keys  drive the rescue tank by hand
    1-4  tank motor test (spins one track for 1 second)
    M  click the marker on the tank (one is enough, e.g. a red cup)
    B  optional second marker at the back, in a different color
*/

import java.net.DatagramSocket;
import java.net.DatagramPacket;
import java.net.InetAddress;
import java.net.SocketTimeoutException;
import java.io.*;
import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;

// ---------------- camera bridge ----------------
final boolean AUTO_START_BRIDGE = true;  // start pose_bridge.py when you press Run
final String  BRIDGE_DIR = "";           // folder with pose_bridge.py; "" = the folder above this sketch
final String  PYTHON = "python3";        // or a full path, e.g. "/Users/you/miniconda3/bin/python3"
final int     VIDEO_PORT = 12002;
int camIndex = -1;                       // -1 = auto: the Mac's own camera, not the iPhone. K cycles.

// ---------------- tuning ----------------
final int   UDP_PORT         = 12000; // landmarks in; robot commands go out on UDP_PORT + 1
final float BEND_START       = 165;   // joint angle (deg) where a limb starts turning yellow
final float BEND_FULL        = 110;   // joint angle where it is fully yellow
final float TILT_WARN        = 25;    // torso degrees from vertical that counts as tipping
final float TILT_DOWN        = 60;    // torso degrees from vertical that counts as on the ground
final float KNEE_WARN        = 140;   // knee angle that counts as legs buckling
final float DROP_WARN        = 1.6;   // hip drop speed in torso-lengths per second
final float DOWN_CONFIRM_S   = 0.8;   // horizontal this long before going RED
final float RECOVER_S        = 1.0;   // upright this long before leaving RED
final float DISPATCH_DELAY_S = 3.0;   // RED this long before the robot is called

// ---------------- MediaPipe Pose landmark indices ----------------
final int N = 33;
final int NOSE = 0, L_SH = 11, R_SH = 12, L_EL = 13, R_EL = 14, L_WR = 15, R_WR = 16,
          L_HIP = 23, R_HIP = 24, L_KN = 25, R_KN = 26, L_AN = 27, R_AN = 28, L_FT = 31, R_FT = 32;

// bone = {from, to, driver}. driver: 0 L elbow, 1 R elbow, 2 L knee, 3 R knee, 4 torso
final int[][] SEG = {
  {L_SH, L_EL, 0}, {L_EL, L_WR, 0},
  {R_SH, R_EL, 1}, {R_EL, R_WR, 1},
  {L_HIP, L_KN, 2}, {L_KN, L_AN, 2}, {L_AN, L_FT, 2},
  {R_HIP, R_KN, 3}, {R_KN, R_AN, 3}, {R_AN, R_FT, 3},
  {L_SH, R_SH, 4}, {L_HIP, R_HIP, 4}, {L_SH, L_HIP, 4}, {R_SH, R_HIP, 4}
};
// joint -> which bone's color it borrows
final int[][] JOINTS = {
  {L_SH, 12}, {R_SH, 13}, {L_EL, 0}, {R_EL, 2}, {L_WR, 1}, {R_WR, 3},
  {L_HIP, 4}, {R_HIP, 7}, {L_KN, 4}, {R_KN, 7}, {L_AN, 5}, {R_AN, 8}
};

final int STANDING = 0, FALLING = 1, DOWN = 2;
final String[] STATE_NAME = {"STANDING", "FALL RISK", "PERSON DOWN"};

color GREEN, YELLOW, ORANGE, RED, CYAN, DIM;

// ---------------- network ----------------
DatagramSocket sock;
volatile boolean running = true;
final Object lock = new Object();
float[][] netPose = new float[N][3];
volatile int lastPacketMs = -100000;
volatile int lastPersonMs = -100000;
DatagramSocket vsock;
volatile PImage camFrame;
volatile int lastFrameMs = -100000;
boolean showVideo = true;
Process bridge;
volatile String bridgeStatus = "not started";
File bridgeDir;
volatile boolean needInstall = false;
boolean installTried = false;
String pythonPath = null;              // real path to python3, found once
int bridgeStartMs = 0;
int autoRestarts = 0;
volatile boolean reconnecting = false;

// ---------------- rescue tank ----------------
volatile String robotStatus = "robot link not started";
volatile int robotStatusMs = -100000;
char driveCmd = 0;                     // arrow key held: 'F','B','L','R'
int lastDriveSendMs = 0;

// ---------------- tank tracking (two colored paper markers on the tank) ----------------
volatile float tankFx = -1, tankFy = -1, tankBx = -1, tankBy = -1;   // front / back marker, 0..1
volatile int lastTankMs = -100000;
volatile float rawFx = -1, rawFy = -1, rawBx = -1, rawBy = -1;      // biggest blob of each color
volatile boolean tankSingle = false;   // one marker only (e.g. a red cup): direction learned from motion
int pickStep = 0;                      // 1 = click front marker, 2 = click back marker

// ---------------- autopilot: drive the tank to the person ----------------
final int AP_OFF = 0, AP_CAL = 1, AP_SEEK = 2, AP_ARRIVED = 3;
final float ARRIVE_DIST = 0.10;        // stop this close to your body (fraction of screen width)
final float TURN_START  = 30;          // turn in place when off target by more than this (deg)
final float TURN_WHILE_DRIVING = 45;   // ...or this much while already driving forward
final int   TURN_PULSE_MS = 120;       // short turn taps so it doesn't overshoot
final int   SETTLE_MS = 250;           // wait for the camera to see the result
int   apMode = AP_OFF, apSub = 0, apPhaseMs = 0, apStartMs = 0;
float apH0 = 0;
int   turnSign = 1;                    // +1 if 'R' turns the tank clockwise on screen
String apMsg = "";

// one-marker mode: heading is measured by driving a little and watching which way the marker moves
final int   PROBE_MS = 300;            // short forward drive to measure the heading
final int   LEG_MS = 450;              // forward drive length between heading checks
final float MIN_MOVE_PX = 8;           // movement needed to trust a heading measurement
float   estHeading = 0;
boolean hasHeading = false;
float   turnRate = 0.25;               // degrees per ms of turning, learned on the way
boolean justTurned = false;
float   headingBeforeTurn = 0, intendedTurn = 0;
int     turnPulseMs = 150, probeFails = 0;
PVector legStart = new PVector();
boolean keepLegStart = false;          // tiny movement: keep adding legs until there's enough to measure
int     escapes = 0;                   // back-up-and-turn attempts when the tank is stuck
final int ESCAPE_BACK_MS = 500, ESCAPE_TURN_MS = 450, DETOUR_MS = 900, MAX_ESCAPES = 8;

// continuous steering (fixes drift from loose wheels): the heading is re-measured from the
// marker's motion every frame while driving, and the tank veers to stay pointed at your head
final int   S_PROBE = 0, S_PROBING = 1, S_MEASURE = 2, S_CALTURN = 3, S_CALSETTLE = 4,
            S_STEER = 5, S_TURN = 6, S_TURNSETTLE = 7;
final int   CAL_TURN_MS = 250;         // test turn at the start to learn turn direction + speed
final int   HEADING_WINDOW_MS = 700;   // heading = direction the marker moved over this window
final float HEADING_SMOOTH = 0.3;      // blend new readings in gently, so one bad reading can't swing it
final float MIN_HEADING_MOVE_PX = 10;  // ignore heading readings from tiny movements (noise)
final float STRAIGHT_DEG = 20;         // within this: drive straight
final float VEER_DEG = 60;             // within this: slight veer; beyond: hard curve
final float SPIN_DEG = 100;            // beyond this: stop and turn in place
final int   MIN_HOLD_MS = 350;         // keep each steering command at least this long (no flip-flopping)
int     lastCmdChangeMs = 0;
boolean turnCalibrated = false, calPending = false;
// Seen through the (mirrored) laptop camera, the tank's R turn always goes counter-clockwise on
// screen, so the steering direction is known; no guessing. Press X if yours ever turns away.
boolean steerFlip = false;
int     steerStartMs = 0;
ArrayList<float[]> tankHist = new ArrayList<float[]>();   // {ms, x, y} of the marker, last 1.5 s
char  apDrive = 0;
int   lastApSendMs = 0;                  // self-heal attempts since the last R press
String[] logTail = new String[4];      // last bridge messages, shown on screen

// ---------------- pose + analysis ----------------
float[][] pose = new float[N][3];   // smoothed landmarks: x, y (0..1), visibility
boolean poseInit = false;
boolean forceDemo = false;
color[] segColor = new color[SEG.length];

int     state = STANDING;
float   tilt, kneeL = 180, kneeR = 180, elbowL = 180, elbowR = 180, hipVel;
float   prevHipY = -1;
float   downTimer, recoverTimer, redTimer;
boolean dispatched = false, silenced = false;
int     dispatchMs = -100000;
int     lastMs;

PFont mono, monoBold, monoBig;

void setup() {
  size(1280, 720);
  frameRate(60);
  GREEN  = color(40, 255, 120);
  YELLOW = color(255, 220, 40);
  ORANGE = color(255, 140, 20);
  RED    = color(255, 45, 60);
  CYAN   = color(60, 200, 255);
  DIM    = color(120, 150, 175);
  mono     = createFont("Menlo", 13);
  monoBold = createFont("Menlo-Bold", 15);
  monoBig  = createFont("Menlo-Bold", 34);
  for (int i = 0; i < segColor.length; i++) segColor[i] = GREEN;

  try {
    sock = new DatagramSocket(UDP_PORT);
    sock.setSoTimeout(250);
    thread("receiveLoop");
    println("Listening for pose_bridge.py on UDP " + UDP_PORT);
  } catch (Exception e) {
    println("Could not open UDP " + UDP_PORT + " (demo only): " + e);
  }
  try {
    vsock = new DatagramSocket(VIDEO_PORT);
    vsock.setReceiveBufferSize(1 << 20);
    vsock.setSoTimeout(250);
    thread("videoLoop");
  } catch (Exception e) {
    println("Could not open UDP " + VIDEO_PORT + " (no camera picture): " + e);
  }
  if (AUTO_START_BRIDGE) startBridge();
  lastMs = millis();
}

void draw() {
  int now = millis();
  float dt = constrain((now - lastMs) / 1000.0, 0.001, 0.1);
  lastMs = now;

  boolean bridgeAlive = bridge != null && bridge.isAlive();
  boolean live    = !forceDemo && (now - lastPacketMs < 1000);
  boolean waiting = !forceDemo && !live && (bridgeAlive || reconnecting);   // camera starting: no demo, just wait
  boolean person  = live ? (now - lastPersonMs < 500) : !waiting;

  float[][] target = new float[N][3];
  if (live) {
    synchronized (lock) {
      for (int i = 0; i < N; i++) for (int j = 0; j < 3; j++) target[i][j] = netPose[i][j];
    }
  } else if (!waiting) {
    demoPose(now / 1000.0, target);
  }

  if (person) {
    float k = poseInit ? 0.55 : 1.0;   // light smoothing kills camera jitter
    for (int i = 0; i < N; i++) for (int j = 0; j < 3; j++) pose[i][j] = lerp(pose[i][j], target[i][j], k);
    poseInit = true;
    analyze(dt);
    for (int s = 0; s < SEG.length; s++) segColor[s] = lerpColor(segColor[s], targetColor(SEG[s][2]), 0.2);
  }

  drawBackground(now, live && showVideo && now - lastFrameMs < 1000 ? camFrame : null);
  if (state == DOWN) drawAlarm(now);
  if (poseInit) drawSkeleton(person ? 1.0 : 0.25);
  recordTank(now);
  if (person && poseInit) updateTarget();
  updateAutopilot(now);
  drawHud(now, live, waiting, person);
  drawTank(now);                         // on top of the HUD so the label is never hidden

  // arrow keys / autopilot drive the tank; resend while active (the tank stops itself after 0.7 s of silence)
  if (driveCmd != 0 && now - lastDriveSendMs > 150) {
    sendToBridge("DRIVE " + driveCmd);
    lastDriveSendMs = now;
  }
  if (apDrive != 0 && now - lastApSendMs > 150) {
    sendToBridge("DRIVE " + apDrive);
    lastApSendMs = now;
  }

  if (pickStep > 0) {
    textFont(monoBig);
    textAlign(CENTER, CENTER);
    fill(CYAN);
    text(pickStep == 1 ? "CLICK THE MARKER ON THE TANK" : "CLICK THE BACK MARKER", width / 2, 90);
    textFont(mono);
    fill(DIM);
    text(pickStep == 1 ? "click the red cup (or whatever is on the tank)   -   press M to cancel"
                       : "optional: click a second, different color at the back of the tank   -   press B to cancel", width / 2, 125);
    textAlign(LEFT, TOP);
  }
}

// =====================================================================
// Analysis: angles, tilt, drop speed, and the GREEN / YELLOW / RED state
// =====================================================================

PVector P(int i) { return new PVector(pose[i][0] * width, pose[i][1] * height); }
boolean vis(int i) { return pose[i][2] > 0.5; }

// angle at joint b between a-b and c-b, 180 = straight
float jointAngle(int a, int b, int c) {
  if (!vis(a) || !vis(b) || !vis(c)) return 180;
  PVector ba = PVector.sub(P(a), P(b));
  PVector bc = PVector.sub(P(c), P(b));
  if (ba.mag() < 1 || bc.mag() < 1) return 180;
  return degrees(PVector.angleBetween(ba, bc));
}

void analyze(float dt) {
  PVector sh = PVector.add(P(L_SH), P(R_SH)).mult(0.5);
  PVector hp = PVector.add(P(L_HIP), P(R_HIP)).mult(0.5);
  PVector spine = PVector.sub(sh, hp);
  float torsoLen = max(spine.mag(), 1);
  tilt = degrees(acos(constrain(-spine.y / torsoLen, -1, 1)));   // 0 = upright, 90 = flat

  elbowL = jointAngle(L_SH, L_EL, L_WR);
  elbowR = jointAngle(R_SH, R_EL, R_WR);
  kneeL  = jointAngle(L_HIP, L_KN, L_AN);
  kneeR  = jointAngle(R_HIP, R_KN, R_AN);

  if (prevHipY >= 0) {
    float v = (hp.y - prevHipY) / dt / torsoLen;    // positive = hips moving down
    hipVel = lerp(hipVel, v, 0.3);
  }
  prevHipY = hp.y;

  boolean horizontal = tilt > TILT_DOWN;
  boolean risky = tilt > TILT_WARN || min(kneeL, kneeR) < KNEE_WARN || hipVel > DROP_WARN;

  if (state == DOWN) {
    recoverTimer = (tilt < TILT_DOWN - 15) ? recoverTimer + dt : 0;
    if (recoverTimer > RECOVER_S) {
      state = risky ? FALLING : STANDING;
      resetAlarm();
    }
  } else {
    downTimer = horizontal ? downTimer + dt : 0;
    if (downTimer > DOWN_CONFIRM_S) {
      state = DOWN;
      redTimer = 0;
      recoverTimer = 0;
    } else {
      state = risky ? FALLING : STANDING;
    }
  }

  if (state == DOWN && !silenced) {
    redTimer += dt;
    if (!dispatched && redTimer >= DISPATCH_DELAY_S) dispatchRobot();
  }
}

void resetAlarm() {
  if (apMode != AP_OFF) stopAutopilot("person is back up, tank stopped");
  downTimer = 0;
  redTimer = 0;
  dispatched = false;
  silenced = false;
}

// 0 = straight limb, 1 = fully bent
float bendAmount(int driver) {
  float ang;
  switch (driver) {
    case 0:  ang = elbowL; break;
    case 1:  ang = elbowR; break;
    case 2:  ang = kneeL;  break;
    case 3:  ang = kneeR;  break;
    default: return constrain(map(tilt, 8, TILT_WARN + 10, 0, 1), 0, 1);
  }
  return constrain(map(ang, BEND_START, BEND_FULL, 0, 1), 0, 1);
}

color targetColor(int driver) {
  if (state == DOWN) return RED;
  color c = lerpColor(GREEN, YELLOW, bendAmount(driver));
  if (state == FALLING) {
    float severity = constrain(map(tilt, TILT_WARN, TILT_DOWN, 0, 1), 0, 1);
    c = lerpColor(lerpColor(c, YELLOW, 0.7), ORANGE, severity);
  }
  return c;
}

color stateColor() {
  if (state == DOWN) return RED;
  if (state == FALLING) return YELLOW;
  return GREEN;
}

// =====================================================================
// Rescue hook
// =====================================================================

void dispatchRobot() {
  dispatched = true;
  dispatchMs = millis();
  String t = nf(hour(), 2) + ":" + nf(minute(), 2) + ":" + nf(second(), 2);
  if (lastTankMs > 0 && poseInit) {
    // The camera has seen the tank's marker before: never drive blind. If the marker is
    // hidden right now, the autopilot waits (tank stopped) until it can see it again.
    println(t + "  RESCUE -> autopilot" + (tankSeen(millis()) ? "" : " (waiting to see the tank)"));
    startAutopilot();
  } else {
    println(t + "  RESCUE -> no tank marker set up (press M), running the fixed rescue route");
    sendToBridge("RESCUE");
  }
}

// =====================================================================
// Tank tracking + autopilot
// =====================================================================

boolean tankSeen(int now) { return now - lastTankMs < 500; }

PVector tankFront()  { return new PVector(tankFx * width, tankFy * height); }
PVector tankBack()   { return new PVector(tankBx * width, tankBy * height); }
PVector tankCenter() { return tankSingle ? tankFront() : PVector.add(tankFront(), tankBack()).mult(0.5); }
float   tankHeading() {
  if (tankSingle) return estHeading;
  PVector d = PVector.sub(tankFront(), tankBack());
  return degrees(atan2(d.y, d.x));
}

float angleDiff(float a, float b) { return ((a - b) % 360 + 540) % 360 - 180; }

// ---- rescue target ----
// Lying down: your head (average of whatever face points are visible, or, if the face is hidden,
// a point just past your shoulders). Standing (e.g. a T test): your feet, since that's where the
// floor is. The target is smoothed so a flickering face can't make it jump.
PVector apTarget = null;

PVector rawTarget() {
  if (state != DOWN && vis(L_AN) && vis(R_AN)) return PVector.add(P(L_AN), P(R_AN)).mult(0.5);
  float sx = 0, sy = 0;
  int n = 0;
  for (int i = 0; i <= 10; i++) {
    if (pose[i][2] > 0.3) { sx += pose[i][0] * width; sy += pose[i][1] * height; n++; }
  }
  if (n > 0) return new PVector(sx / n, sy / n);
  if (vis(L_SH) && vis(R_SH) && vis(L_HIP) && vis(R_HIP)) {
    PVector sh = PVector.add(P(L_SH), P(R_SH)).mult(0.5);
    PVector hp = PVector.add(P(L_HIP), P(R_HIP)).mult(0.5);
    return PVector.add(sh, PVector.sub(sh, hp).mult(0.5));
  }
  return null;
}

void updateTarget() {
  PVector t = rawTarget();
  if (t == null) return;
  if (apTarget == null || PVector.dist(apTarget, t) > width * 0.5) apTarget = t.copy();
  else apTarget.lerp(t, 0.08);
}

PVector nearestBodyPoint(PVector c) { return apTarget; }

void setDrive(char c) {
  if (c == apDrive) return;
  apDrive = c;
  if (c == 0) sendToBridge("STOP");
  else { sendToBridge("DRIVE " + c); lastApSendMs = millis(); }
}

void startAutopilot() {
  apMode = tankSingle ? AP_SEEK : AP_CAL;   // one-marker mode learns while it drives
  apSub = 0;
  hasHeading = false;
  justTurned = false;
  keepLegStart = false;
  probeFails = 0;
  escapes = 0;
  turnSign = steerFlip ? 1 : -1;
  turnRate = 0.3;                           // first guess at turn speed (deg per ms); learned on the way
  stuckCount = 0;
  pendingTap = false;
  turnCalibrated = true;                    // one-marker mode: direction is known, skip the test turn
  calPending = false;
  apTarget = null;
  updateTarget();
  apStartMs = apPhaseMs = millis();
  apMsg = "rescue started";
}

void stopAutopilot(String why) {
  apMode = AP_OFF;
  setDrive((char) 0);
  apMsg = why;
  println("autopilot: " + why);
}

void updateAutopilot(int now) {
  if (apMode == AP_OFF || apMode == AP_ARRIVED) return;
  if (!tankSeen(now)) {
    setDrive((char) 0);
    apMsg = "can't see the tank's marker, waiting (tank stopped)";
    if (now - lastTankMs > 30000) stopAutopilot("lost the tank, rescue stopped");
    return;
  }
  if (now - apStartMs > 90000) { stopAutopilot("rescue timed out"); return; }
  if (tankSingle) { updateAutopilotSingle(now); return; }
  float hd = tankHeading();

  // 1) calibrate: tap 'R' once and watch which way the tank turns on screen
  if (apMode == AP_CAL) {
    if (apSub == 0) { apH0 = hd; setDrive('R'); apSub = 1; apPhaseMs = now; apMsg = "checking which way the tank turns"; }
    else if (apSub == 1 && now - apPhaseMs > 250) { setDrive((char) 0); apSub = 2; apPhaseMs = now; }
    else if (apSub == 2 && now - apPhaseMs > 400) {
      float d = angleDiff(hd, apH0);
      turnSign = (abs(d) < 4 || d > 0) ? 1 : -1;
      println("autopilot: R turned the tank " + nf(d, 0, 1) + " deg on screen, turnSign=" + turnSign);
      apMode = AP_SEEK; apSub = 0; apPhaseMs = now;
    }
    return;
  }

  // 2) seek: turn toward the closest body point, then drive at it
  PVector c = tankCenter();
  PVector target = nearestBodyPoint(c);
  if (target == null) { setDrive((char) 0); apMsg = "can't see you, waiting..."; return; }
  float dist = PVector.dist(c, target) / width;
  if (dist < ARRIVE_DIST) {
    setDrive((char) 0);
    apMode = AP_ARRIVED;
    apMsg = "stopped right next to you";
    println("autopilot: arrived next to the person");
    return;
  }
  if (apSub == 1) { if (now - apPhaseMs > TURN_PULSE_MS) { setDrive((char) 0); apSub = 2; apPhaseMs = now; } return; }
  if (apSub == 2) { if (now - apPhaseMs > SETTLE_MS) apSub = 0; return; }

  float err = angleDiff(degrees(atan2(target.y - c.y, target.x - c.x)), hd);
  if (abs(err) > SPIN_DEG) {                     // way off: turn in place in short taps
    setDrive(err * turnSign > 0 ? 'R' : 'L');
    apSub = 1; apPhaseMs = now;
    apMsg = "turning toward you";
  } else {                                       // otherwise steer continuously while driving
    char cmd = steerFor(err);
    setDrive(cmd);
    apMsg = cmd == 'F' ? "driving to you" : "driving to you, correcting course";
  }
}

// Tank history for measuring direction from motion
void recordTank(int now) {
  if (!tankSeen(now)) return;
  PVector c = tankCenter();
  tankHist.add(new float[] {now, c.x, c.y});
  while (tankHist.size() > 0 && now - tankHist.get(0)[0] > 1500) tankHist.remove(0);
}

// how far the marker moved in the last `ms` milliseconds (pixels)
PVector movedOver(int now, int ms) {
  if (tankHist.size() < 2) return new PVector();
  float[] last = tankHist.get(tankHist.size() - 1);
  for (float[] h : tankHist) {
    if (now - h[0] <= ms) return new PVector(last[1] - h[1], last[2] - h[2]);
  }
  return new PVector();
}

// steering command for a heading error: straight, slight veer, or hard curve toward the target
char steerFor(float err) {
  boolean right = err * turnSign > 0;
  float a = abs(err);
  if (a < STRAIGHT_DEG) return 'F';
  if (a < VEER_DEG) return right ? 'd' : 's';
  return right ? 'g' : 'a';
}

// One marker: "drive straight, measure, correct a little, repeat".
// The tank's direction is only ever measured from a straight stretch it just drove, so it never
// steers on old information. Turns deliberately under-correct (about half), and the real turn
// speed is learned from each turn, so it closes in on your head without swinging back and forth.
final int   L_START = 0, L_DRIVE = 1, L_SETTLE = 2, L_DECIDE = 3, L_TAP = 4, L_TAPSETTLE = 5;
final float LEG_PX = 25;               // drive straight until the marker has moved this far...
final int   LEG_MIN_MS = 250, LEG_MAX_MS = 900;   // ...within these limits
final float ON_COURSE_DEG = 20;        // within this: keep going straight
final float UNDER_CORRECT = 0.5;       // turn about half of the measured error each time
int     legT = 0, stuckCount = 0, tapDir = 0;
boolean pendingTap = false;

void updateAutopilotSingle(int now) {
  PVector c = tankCenter();
  PVector target = nearestBodyPoint(c);
  if (target == null) { setDrive((char) 0); apMsg = "can't see you, waiting..."; return; }
  float d = PVector.dist(c, target);
  if (d / width < ARRIVE_DIST) {
    setDrive((char) 0);
    apMode = AP_ARRIVED;
    apMsg = "stopped right next to you";
    println("autopilot: arrived next to the person");
    return;
  }
  float want = degrees(atan2(target.y - c.y, target.x - c.x));

  switch (apSub) {
    case L_START:                                   // drive a straight stretch
      legStart = c.copy();
      legT = now;
      setDrive('F');
      apSub = L_DRIVE;
      apMsg = hasHeading ? "driving to you" : "finding which way the tank faces";
      break;

    case L_DRIVE: {
      float moved = PVector.dist(c, legStart);
      if ((moved >= LEG_PX && now - legT >= LEG_MIN_MS) || now - legT > LEG_MAX_MS) {
        setDrive((char) 0);
        apPhaseMs = now;
        apSub = L_SETTLE;
      }
      break;
    }

    case L_SETTLE: {                                // measure the direction it just drove
      if (now - apPhaseMs < 200) break;
      PVector move = PVector.sub(c, legStart);
      if (move.mag() < MIN_MOVE_PX) {               // didn't really move
        if (++stuckCount >= 2) {                    // twice in a row: stuck (wall, furniture)
          stuckCount = 0;
          if (++escapes > MAX_ESCAPES) { stopAutopilot("the tank keeps getting stuck (clear the path or charge the battery)"); return; }
          setDrive('B');
          apPhaseMs = now;
          apSub = 10;
          apMsg = "blocked, backing up (" + escapes + ")";
          pendingTap = false;
          break;
        }
        apSub = L_START;
        break;
      }
      stuckCount = 0;
      float measured = degrees(atan2(move.y, move.x));
      if (pendingTap) {                             // learn how far a turn really goes (size only)
        float got = angleDiff(measured, headingBeforeTurn);
        if (got * tapDir > 0 && abs(got) > 5) turnRate = constrain(lerp(turnRate, abs(got) / turnPulseMs, 0.5), 0.05, 1.5);
        pendingTap = false;
      }
      estHeading = measured;
      hasHeading = true;
      apSub = L_DECIDE;
      break;
    }

    case L_DECIDE: {                                // on course? keep going. Off? small turn.
      float err = angleDiff(want, estHeading);
      boolean ahead = cos(radians(err)) > 0;
      float missBy = d * abs(sin(radians(err)));    // how far the current line passes from your head
      boolean onCourse = ahead && (abs(err) < ON_COURSE_DEG || missBy < ARRIVE_DIST * width * 0.7);
      if (onCourse) {
        apSub = L_START;
        break;
      }
      tapDir = err > 0 ? 1 : -1;
      turnPulseMs = (int) constrain(UNDER_CORRECT * abs(err) / turnRate, 60, 400);
      headingBeforeTurn = estHeading;
      pendingTap = true;
      setDrive(err * turnSign > 0 ? 'R' : 'L');
      apPhaseMs = now;
      apSub = L_TAP;
      apMsg = "small turn toward you";
      break;
    }

    case L_TAP:
      if (now - apPhaseMs > turnPulseMs) { setDrive((char) 0); apPhaseMs = now; apSub = L_TAPSETTLE; }
      break;
    case L_TAPSETTLE:
      if (now - apPhaseMs > 150) apSub = L_START;
      break;

    case 10:                                        // blocked: back up...
      if (now - apPhaseMs > ESCAPE_BACK_MS) {
        setDrive(((escapes - 1) / 2) % 2 == 0 ? 'R' : 'L');   // ...turn away (same side twice, then the other)
        apPhaseMs = now;
        apSub = 11;
        apMsg = "blocked, going around";
      }
      break;
    case 11:
      if (now - apPhaseMs > ESCAPE_TURN_MS) { setDrive((char) 0); apPhaseMs = now; apSub = 12; }
      break;
    case 12:                                        // ...and drive a detour stretch before aiming again
      if (now - apPhaseMs > SETTLE_MS) { legStart = c.copy(); setDrive('F'); apPhaseMs = now; apSub = 13; }
      break;
    case 13:
      if (now - apPhaseMs > DETOUR_MS) { setDrive((char) 0); apPhaseMs = now; apSub = L_SETTLE; }
      break;
  }
}

void drawTank(int now) {
  // small dots: the biggest blob of each marker color, so you can see what the camera latches onto
  noStroke();
  textFont(mono);
  textAlign(CENTER, CENTER);
  if (rawFx >= 0) { fill(CYAN, 150); ellipse(rawFx * width, rawFy * height, 12, 12); fill(255); text("F", rawFx * width, rawFy * height - 14); }
  if (rawBx >= 0) { fill(CYAN, 150); ellipse(rawBx * width, rawBy * height, 12, 12); fill(255); text("B", rawBx * width, rawBy * height - 14); }
  textAlign(LEFT, TOP);
  if (!tankSeen(now)) return;
  PVector f = tankFront(), b = tankBack(), c = tankCenter();
  float r = tankSingle ? 50 : max(PVector.dist(f, b) * 1.4, 45);
  float pulse = 0.5 + 0.5 * sin(now * 0.008);
  noFill();
  // cyan ring
  for (int k = 3; k >= 1; k--) { stroke(CYAN, 25); strokeWeight(k * 6); ellipse(c.x, c.y, r * 2, r * 2); }
  stroke(CYAN, 200 + 55 * pulse);
  strokeWeight(3);
  ellipse(c.x, c.y, r * 2, r * 2);
  // rotating ticks
  float a0 = now * 0.003;
  strokeWeight(2);
  for (int i = 0; i < 4; i++) {
    float a = a0 + i * HALF_PI;
    line(c.x + cos(a) * (r + 6), c.y + sin(a) * (r + 6), c.x + cos(a) * (r + 18), c.y + sin(a) * (r + 18));
  }
  // where it should go: faint white arrow toward your head while a rescue is running
  if (apMode == AP_SEEK && poseInit) {
    PVector t = nearestBodyPoint(c);
    if (t != null) {
      PVector want = PVector.sub(t, c).setMag(r * 1.6);
      stroke(255, 140);
      strokeWeight(2);
      line(c.x, c.y, c.x + want.x, c.y + want.y);
    }
  }
  // which way it is travelling: bold cyan arrow (one-marker mode: once measured from its motion)
  PVector dir = tankSingle ? PVector.fromAngle(radians(estHeading)).mult(10) : PVector.sub(f, b);
  if (dir.mag() > 1 && (!tankSingle || hasHeading)) {
    dir.setMag(r * 1.6);
    PVector tip = PVector.add(c, dir);
    stroke(CYAN);
    strokeWeight(5);
    line(c.x, c.y, tip.x, tip.y);
    PVector side = new PVector(-dir.y, dir.x).setMag(12);
    PVector back = dir.copy().setMag(20);
    line(tip.x, tip.y, tip.x - back.x + side.x, tip.y - back.y + side.y);
    line(tip.x, tip.y, tip.x - back.x - side.x, tip.y - back.y - side.y);
  }
  // path line to the person while the autopilot is driving
  if (apMode == AP_SEEK && poseInit) {
    PVector t = nearestBodyPoint(c);
    if (t != null) {
      stroke(CYAN, 120);
      strokeWeight(2);
      float L = PVector.dist(c, t);
      for (float u = 0; u < L; u += 16) {          // dashed line
        PVector p1 = PVector.lerp(c, t, u / L), p2 = PVector.lerp(c, t, min(u + 8, L) / L);
        line(p1.x, p1.y, p2.x, p2.y);
      }
    }
  }
  noStroke();
  fill(CYAN);
  textFont(monoBold);
  textAlign(CENTER, BOTTOM);
  text("RESCUE BOT", c.x, c.y - r - 22);
  textAlign(LEFT, TOP);
}

// Robot commands go back to pose_bridge.py on UDP_PORT + 1.
// The bridge owns the Bluetooth link to the tank, so this sketch never has to.
void sendToBridge(String cmd) {
  if (sock == null) return;
  try {
    byte[] b = cmd.getBytes("UTF-8");
    sock.send(new DatagramPacket(b, b.length, InetAddress.getLoopbackAddress(), UDP_PORT + 1));
  } catch (Exception e) {
    println("Could not send '" + cmd + "': " + e);
  }
}

// =====================================================================
// Network (runs on its own thread)
// =====================================================================

public void receiveLoop() {
  byte[] buf = new byte[8192];
  DatagramPacket pkt = new DatagramPacket(buf, buf.length);
  while (running) {
    try {
      pkt.setLength(buf.length);
      sock.receive(pkt);
      handleMessage(new String(pkt.getData(), 0, pkt.getLength(), "UTF-8").trim());
    } catch (SocketTimeoutException e) {
      // no packet this tick, keep looping
    } catch (Exception e) {
      if (running) println("UDP error: " + e);
    }
  }
}

// "P;x,y,v;x,y,v;..." (33 landmarks)  or  "N" (camera on, nobody in frame)
void handleMessage(String msg) {
  if (msg.startsWith("T;")) {            // tank marker positions from the bridge
    String[] groups = split(msg.substring(2), ';');
    String[] v = split(groups[0], ',');
    if (v.length == 4) {
      tankFx = parseFloat(v[0]); tankFy = parseFloat(v[1]);
      tankBx = parseFloat(v[2]); tankBy = parseFloat(v[3]);
      tankSingle = tankBx < -1.5;        // the bridge sends -2 in one-marker mode
      if (tankFx >= 0 && (tankBx >= 0 || tankSingle)) lastTankMs = millis();
    }
    if (groups.length > 1) {
      String[] r = split(groups[1], ',');
      if (r.length == 4) {
        rawFx = parseFloat(r[0]); rawFy = parseFloat(r[1]);
        rawBx = parseFloat(r[2]); rawBy = parseFloat(r[3]);
      }
    }
    return;
  }
  if (msg.startsWith("R;")) {            // status from robot_link.py
    robotStatus = msg.substring(2);
    robotStatusMs = millis();
    return;
  }
  lastPacketMs = millis();
  if (!msg.startsWith("P;")) return;
  String[] parts = split(msg.substring(2), ';');
  if (parts.length < N) return;
  float[][] tmp = new float[N][3];
  for (int i = 0; i < N; i++) {
    String[] v = split(parts[i], ',');
    if (v.length < 3) return;
    tmp[i][0] = parseFloat(v[0]);
    tmp[i][1] = parseFloat(v[1]);
    tmp[i][2] = parseFloat(v[2]);
  }
  synchronized (lock) {
    for (int i = 0; i < N; i++) for (int j = 0; j < 3; j++) netPose[i][j] = tmp[i][j];
  }
  lastPersonMs = millis();
}

// =====================================================================
// Camera bridge: start pose_bridge.py, show its log, receive the picture
// =====================================================================

void startBridge() {
  stopBridge();
  File dir = new File(BRIDGE_DIR.length() > 0 ? BRIDGE_DIR : sketchPath(".."));
  bridgeDir = dir;
  needInstall = false;
  File script = new File(dir, "pose_bridge.py");
  if (!script.exists()) {
    bridgeStatus = "pose_bridge.py not found in " + dir.getAbsolutePath();
    println(bridgeStatus);
    return;
  }
  try {
    // Run python directly (not inside a shell) so Stop / R really closes it
    // and the camera is released every time.
    ProcessBuilder pb = new ProcessBuilder(findPython(), "-u", "pose_bridge.py",
      "--managed", "--camera", str(camIndex));
    pb.directory(dir.getCanonicalFile());
    pb.redirectErrorStream(true);
    bridge = pb.start();
    bridgeStartMs = millis();
    bridgeStatus = "starting camera...";
    thread("bridgeLogLoop");
  } catch (Exception e) {
    bridgeStatus = "could not start bridge: " + e.getMessage();
    println(bridgeStatus);
  }
}

// Ask a login shell once where python3 lives (conda, Homebrew...), the same one Terminal uses.
String findPython() {
  if (pythonPath != null) return pythonPath;
  pythonPath = PYTHON;
  if (PYTHON.contains("/")) return pythonPath;
  try {
    Process p = new ProcessBuilder("/bin/zsh", "-ilc", "command -v " + PYTHON).redirectErrorStream(true).start();
    BufferedReader r = new BufferedReader(new InputStreamReader(p.getInputStream()));
    String line;
    while ((line = r.readLine()) != null) {
      if (line.trim().startsWith("/")) pythonPath = line.trim();
    }
    p.waitFor();
  } catch (Exception e) {
    println("could not look up python3: " + e);
  }
  println("Using Python: " + pythonPath);
  return pythonPath;
}

void stopBridge() {
  if (bridge != null) {
    Process p = bridge;
    bridge = null;
    p.destroy();
    try {
      if (!p.waitFor(2, java.util.concurrent.TimeUnit.SECONDS)) p.destroyForcibly();
    } catch (Exception e) {
      p.destroyForcibly();
    }
  }
}

public void bridgeLogLoop() {
  Process p = bridge;
  if (p == null) return;
  try {
    BufferedReader r = new BufferedReader(new InputStreamReader(p.getInputStream()));
    String line;
    while ((line = r.readLine()) != null) {
      println("[bridge] " + line);
      if (line.contains("Missing package")) needInstall = true;
      if (line.trim().length() > 0 && !line.startsWith("E0000") && !line.startsWith("W0000")
          && !line.startsWith("[robot]")) {
        bridgeStatus = line.trim();
        for (int i = 0; i < logTail.length - 1; i++) logTail[i] = logTail[i + 1];
        logTail[logTail.length - 1] = line.trim();
      }
    }
    int code = p.waitFor();
    if (!running || p != bridge) return;      // we stopped it on purpose
    if (running && p == bridge && needInstall && !installTried) {
      installTried = true;
      installPackages();
      return;
    }
    println("[bridge] stopped with exit code " + code);
    // Self-heal: camera trouble (code 2) -> try the next camera; anything else -> restart.
    if (autoRestarts < 6) {
      autoRestarts++;
      if (code == 2) camIndex = (max(camIndex, 0) + 1) % 4;
      bridgeStatus = "camera dropped, reconnecting (try " + autoRestarts + " of 6)...";
      println("[bridge] " + bridgeStatus + "  camera " + (camIndex < 0 ? "auto" : str(camIndex)));
      reconnecting = true;
      delay(1000);
      if (running) startBridge();
      reconnecting = false;
    } else {
      bridgeStatus = "stopped: " + bridgeStatus + "  [R] retry";
    }
  } catch (Exception e) {
    // sketch is closing
  }
}

// First run: install mediapipe + opencv for you, then start the camera.
void installPackages() {
  bridgeStatus = "installing tracking packages, one time only (1 to 3 min)...";
  println("[setup] " + bridgeStatus);
  try {
    ProcessBuilder pb = new ProcessBuilder("/bin/zsh", "-ilc",
      PYTHON + " -m pip install --disable-pip-version-check -r requirements.txt");
    pb.directory(bridgeDir.getCanonicalFile());
    pb.redirectErrorStream(true);
    Process ip = pb.start();
    BufferedReader r = new BufferedReader(new InputStreamReader(ip.getInputStream()));
    String line;
    while ((line = r.readLine()) != null) println("[setup] " + line);
    if (ip.waitFor() == 0) {
      bridgeStatus = "packages installed, starting camera...";
      startBridge();
    } else {
      bridgeStatus = "package install failed, see console  [R] retry";
    }
  } catch (Exception e) {
    bridgeStatus = "package install failed: " + e.getMessage();
  }
}

// Pictures arrive in numbered pieces (macOS limits one local UDP message to ~9 KB).
// Each piece: [frame id: 2 bytes][piece #: 1][piece count: 1][JPEG bytes]
public void videoLoop() {
  byte[] buf = new byte[65536];
  DatagramPacket pkt = new DatagramPacket(buf, buf.length);
  int curId = -1, got = 0, count = 0;
  byte[][] pieces = new byte[256][];
  while (running) {
    try {
      pkt.setLength(buf.length);
      vsock.receive(pkt);
      int len = pkt.getLength();
      if (len < 5) continue;
      byte[] d = pkt.getData();
      int id = ((d[0] & 0xFF) << 8) | (d[1] & 0xFF);
      int idx = d[2] & 0xFF, n = d[3] & 0xFF;
      if (n == 0 || idx >= n) continue;
      if (id != curId) {                 // new picture: start over
        curId = id;
        got = 0;
        count = n;
        for (int i = 0; i < 256; i++) pieces[i] = null;
      }
      if (pieces[idx] == null) {
        pieces[idx] = java.util.Arrays.copyOfRange(d, 4, len);
        got++;
      }
      if (got < count) continue;
      ByteArrayOutputStream jpg = new ByteArrayOutputStream();
      for (int i = 0; i < count; i++) jpg.write(pieces[i]);
      curId = -1;                        // done with this picture
      BufferedImage bi = ImageIO.read(new ByteArrayInputStream(jpg.toByteArray()));
      if (bi == null) continue;
      PImage img = new PImage(bi.getWidth(), bi.getHeight(), RGB);
      bi.getRGB(0, 0, bi.getWidth(), bi.getHeight(), img.pixels, 0, bi.getWidth());
      img.updatePixels();
      camFrame = img;
      lastFrameMs = millis();
    } catch (SocketTimeoutException e) {
      // no piece this tick
    } catch (Exception e) {
      if (running) println("video error: " + e);
    }
  }
}

void cleanup() {
  running = false;
  stopBridge();
  if (sock != null) sock.close();
  if (vsock != null) vsock.close();
}

public void exit() {
  cleanup();
  super.exit();
}

public void dispose() {
  cleanup();
  super.dispose();
}

// =====================================================================
// Drawing
// =====================================================================

void drawBackground(int now, PImage cam) {
  background(6, 10, 18);
  if (cam != null) {
    tint(150, 185, 220, 210);          // cool, dimmed camera picture so the skeleton pops
    image(cam, 0, 0, width, height);
    noTint();
    noStroke();
    fill(6, 10, 18, 70);
    rect(0, 0, width, height);
  }
  strokeWeight(1);
  stroke(30, 60, 90, 55);
  for (int x = 0; x < width; x += 40) line(x, 0, x, height);
  for (int y = 0; y < height; y += 40) line(0, y, width, y);
  noStroke();
  float sy = (now * 0.12) % height;   // slow scan line
  fill(60, 200, 255, 14);
  rect(0, sy, width, 2);
  fill(60, 200, 255, 6);
  rect(0, sy - 30, width, 30);
}

void drawAlarm(int now) {
  float pulse = 0.5 + 0.5 * sin(now * 0.012);
  noFill();
  strokeWeight(6);
  for (int k = 0; k < 10; k++) {
    stroke(255, 45, 60, (70 - k * 7) * pulse);
    rect(k * 6, k * 6, width - k * 12, height - k * 12);
  }
  // target ring around the person
  PVector c = PVector.add(P(L_HIP), P(R_HIP)).mult(0.5);
  float r = 140 + 25 * pulse;
  stroke(255, 45, 60, 160);
  strokeWeight(2);
  ellipse(c.x, c.y, r * 2, r * 2);
  float a0 = now * 0.002;
  for (int i = 0; i < 4; i++) {
    float a = a0 + i * HALF_PI;
    line(c.x + cos(a) * (r + 6), c.y + sin(a) * (r + 6), c.x + cos(a) * (r + 26), c.y + sin(a) * (r + 26));
  }
}

void glowLine(PVector p1, PVector p2, color c, float a) {
  for (int k = 4; k >= 1; k--) {
    stroke(c, 20 * a);
    strokeWeight(k * 7);
    line(p1.x, p1.y, p2.x, p2.y);
  }
  stroke(c, 255 * a);
  strokeWeight(5);
  line(p1.x, p1.y, p2.x, p2.y);
  stroke(255, 140 * a);
  strokeWeight(1.5);
  line(p1.x, p1.y, p2.x, p2.y);
}

void drawSkeleton(float a) {
  strokeCap(ROUND);
  for (int s = 0; s < SEG.length; s++) {
    if (vis(SEG[s][0]) && vis(SEG[s][1])) glowLine(P(SEG[s][0]), P(SEG[s][1]), segColor[s], a);
  }

  // head + neck
  if (vis(NOSE) && vis(L_SH) && vis(R_SH)) {
    PVector nose = P(NOSE);
    PVector sh = PVector.add(P(L_SH), P(R_SH)).mult(0.5);
    float r = max(PVector.dist(P(L_SH), P(R_SH)) * 0.32, 14);
    color hc = segColor[10];
    PVector toSh = PVector.sub(sh, nose);
    if (toSh.mag() > r) {
      toSh.setMag(r);
      glowLine(PVector.add(nose, toSh), sh, hc, a);
    }
    noFill();
    for (int k = 4; k >= 1; k--) {
      stroke(hc, 20 * a);
      strokeWeight(k * 6);
      ellipse(nose.x, nose.y, r * 2, r * 2);
    }
    stroke(hc, 255 * a);
    strokeWeight(4);
    ellipse(nose.x, nose.y, r * 2, r * 2);
  }

  // joints
  noStroke();
  for (int[] j : JOINTS) {
    if (!vis(j[0])) continue;
    PVector p = P(j[0]);
    fill(segColor[j[1]], 70 * a);
    ellipse(p.x, p.y, 20, 20);
    fill(255, 230 * a);
    ellipse(p.x, p.y, 7, 7);
  }
}

void panel(float x, float y, float w, float h) {
  fill(8, 16, 28, 200);
  stroke(CYAN, 70);
  strokeWeight(1);
  rect(x, y, w, h, 6);
}

void drawHud(int now, boolean live, boolean waiting, boolean person) {
  color sc = stateColor();

  // --- title + state (top left)
  panel(20, 20, 330, 150);
  textFont(monoBold);
  fill(CYAN);
  textAlign(LEFT, TOP);
  text("FALLGUARD", 36, 32);
  textFont(mono);
  fill(DIM);
  text("pose fall detection // rescue dispatch", 36, 54);

  // source badge
  boolean blink = (now / 500) % 2 == 0;
  noStroke();
  color badge = live ? GREEN : waiting ? YELLOW : CYAN;
  fill(badge, blink ? 255 : 90);
  ellipse(42, 89, 9, 9);
  fill(badge);
  String cam = camIndex < 0 ? "auto" : "#" + camIndex;
  text((live ? "LIVE CAMERA" : waiting ? "STARTING CAMERA" : "DEMO MODE") + "   cam " + cam, 54, 82);

  textFont(monoBig);
  fill(sc);
  text(STATE_NAME[state], 36, 112);

  // --- telemetry (top right)
  float px = width - 320, py = 20;
  panel(px, py, 300, 196);
  textFont(monoBold);
  fill(CYAN);
  text("TELEMETRY", px + 16, py + 12);
  float y = py + 42;
  meter("TORSO TILT", round(tilt) + "°", tilt / 90.0, tilt > TILT_DOWN ? RED : tilt > TILT_WARN ? YELLOW : GREEN, px + 16, y); y += 25;
  meter("L KNEE",  round(kneeL) + "°",  bendAmount(2), lerpColor(GREEN, YELLOW, bendAmount(2)), px + 16, y); y += 25;
  meter("R KNEE",  round(kneeR) + "°",  bendAmount(3), lerpColor(GREEN, YELLOW, bendAmount(3)), px + 16, y); y += 25;
  meter("L ELBOW", round(elbowL) + "°", bendAmount(0), lerpColor(GREEN, YELLOW, bendAmount(0)), px + 16, y); y += 25;
  meter("R ELBOW", round(elbowR) + "°", bendAmount(1), lerpColor(GREEN, YELLOW, bendAmount(1)), px + 16, y); y += 25;
  float drop = constrain(hipVel / (DROP_WARN * 2), 0, 1);
  meter("DROP SPEED", nf(max(hipVel, 0), 0, 1), drop, hipVel > DROP_WARN ? YELLOW : GREEN, px + 16, y);

  // --- dispatch bar (bottom center)
  float bw = 560, bx = (width - bw) / 2, by = height - 92;
  panel(bx, by, bw, 66);
  textAlign(CENTER, TOP);
  textFont(monoBold);
  if (state == DOWN && silenced) {
    fill(DIM);
    text("ALARM CANCELLED  //  STILL WATCHING", width / 2, by + 14);
  } else if (state == DOWN && !dispatched) {
    float left = max(DISPATCH_DELAY_S - redTimer, 0);
    fill(RED);
    text("RESCUE BOT DISPATCH IN " + nf(left, 0, 1) + "s   [C] I'M OK", width / 2, by + 14);
    noStroke();
    fill(RED, 50);
    rect(bx + 20, by + 40, bw - 40, 10, 5);
    fill(RED);
    rect(bx + 20, by + 40, (bw - 40) * constrain(redTimer / DISPATCH_DELAY_S, 0, 1), 10, 5);
  } else if (dispatched) {
    boolean arrived = apMode == AP_ARRIVED;
    fill(arrived ? CYAN : RED, arrived || (now / 250) % 2 == 0 ? 255 : 120);
    text(arrived ? ">> RESCUE BOT HAS ARRIVED <<" : ">> RESCUE BOT DISPATCHED <<", width / 2, by + 14);
    textFont(mono);
    fill(apMode != AP_OFF ? CYAN : DIM);
    String sub = apMode != AP_OFF || apMsg.length() > 0 ? apMsg : "tank en route";
    text(sub + "  //  " + nf((now - dispatchMs) / 1000.0, 0, 1) + "s since call", width / 2, by + 40);
  } else {
    fill(state == FALLING ? YELLOW : GREEN);
    text(state == FALLING ? "WATCHING CLOSELY" : "MONITORING", width / 2, by + 14);
    textFont(mono);
    boolean robotFresh = now - robotStatusMs < 5000;
    boolean robotOk = robotFresh && robotStatus.startsWith("connected");
    fill(robotOk ? GREEN : robotFresh ? YELLOW : DIM);
    text("rescue tank: " + (robotFresh ? robotStatus : "no link") + (robotOk ? "   [arrows] drive" : ""), width / 2, by + 40);
  }

  // --- no person warning
  if (!person) {
    textFont(monoBig);
    textAlign(CENTER, CENTER);
    fill(YELLOW, (now / 400) % 2 == 0 ? 255 : 110);
    text(waiting ? "STARTING CAMERA..." : "NO PERSON IN FRAME", width / 2, height / 2);
  }

  // --- camera bridge status (until live)
  textFont(mono);
  textAlign(LEFT, BOTTOM);
  if (!live) {
    // current status, with the last few bridge messages above it
    String cur = bridgeStatus.length() > 90 ? bridgeStatus.substring(0, 87) + "..." : bridgeStatus;
    fill(YELLOW, 220);
    text("camera: " + cur, 20, height - 34);
    float ly = height - 52;
    fill(YELLOW, 120);
    for (int i = logTail.length - 1; i >= 0; i--) {
      String st = logTail[i];
      if (st == null || st.equals(bridgeStatus)) continue;
      text("        " + (st.length() > 90 ? st.substring(0, 87) + "..." : st), 20, ly);
      ly -= 18;
    }
  }

  // --- key help
  fill(DIM, 160);
  text("[D] demo  [C] I'm OK  [T] test  [V] video  [K] camera  [M] markers  [R] restart  [S] shot", 20, height - 14);
  textAlign(LEFT, TOP);
}

void meter(String label, String val, float frac, color c, float x, float y) {
  textFont(mono);
  textAlign(LEFT, TOP);
  fill(DIM);
  text(label, x, y);
  textAlign(RIGHT, TOP);
  fill(c);
  text(val, x + 268, y);
  noStroke();
  fill(255, 20);
  rect(x + 100, y + 5, 120, 6, 3);
  fill(c);
  rect(x + 100, y + 5, 120 * constrain(frac, 0, 1), 6, 3);
  textAlign(LEFT, TOP);
}

// =====================================================================
// Keys
// =====================================================================

void keyPressed() {
  if (key == CODED) {
    if (apMode != AP_OFF) stopAutopilot("manual control");
    char c = 0;
    if (keyCode == UP) c = 'F';
    if (keyCode == DOWN) c = 'B';
    if (keyCode == LEFT) c = 'L';
    if (keyCode == RIGHT) c = 'R';
    if (c != 0 && c != driveCmd) {
      driveCmd = c;
      sendToBridge("DRIVE " + c);
      lastDriveSendMs = millis();
    }
    return;
  }
  if (key == 'd' || key == 'D') forceDemo = !forceDemo;
  if (key == 'c' || key == 'C') {
    if (state == DOWN) silenced = true;
    if (apMode != AP_OFF) stopAutopilot("cancelled");
  }
  if (key == 'm' || key == 'M') pickStep = pickStep == 1 ? 0 : 1;   // click the marker on the tank
  if (key == 'b' || key == 'B') pickStep = pickStep == 2 ? 0 : 2;   // optional second (back) marker
  if (key == 't' || key == 'T') dispatchRobot();
  if (key >= '1' && key <= '4') {          // tank motor test: one track for 1 second
    sendToBridge("TEST " + key);
    println("Motor test " + key + " sent");
  }
  if (key == 'x' || key == 'X') {
    steerFlip = !steerFlip;
    turnSign = steerFlip ? 1 : -1;
    println("steering direction " + (steerFlip ? "FLIPPED" : "normal"));
  }
  if (key == 'v' || key == 'V') showVideo = !showVideo;
  if (key == 'r' || key == 'R') { installTried = false; autoRestarts = 0; startBridge(); }
  if (key == 'k' || key == 'K') {           // try the next camera
    camIndex = (camIndex + 1) % 4;
    autoRestarts = 0;
    println("Switching to camera " + camIndex);
    startBridge();
  }
  if (key == 's' || key == 'S') {
    saveFrame(sketchPath("../screenshots/fallguard-####.png"));
    println("screenshot saved to screenshots/");
  }
}

// M: click one marker (e.g. the red cup) and you're done.
// B: optionally click a second, different color at the back of the tank.
void mousePressed() {
  if (pickStep == 0) return;
  String xy = nf(mouseX / (float) width, 0, 4) + " " + nf(mouseY / (float) height, 0, 4);
  if (pickStep == 1) {
    sendToBridge("PICK F " + xy);
    sendToBridge("PICK CLEAR B");          // one-marker mode until a back marker is added
  } else {
    sendToBridge("PICK B " + xy);
  }
  pickStep = 0;
}

void keyReleased() {
  if (key == CODED && driveCmd != 0) {
    driveCmd = 0;
    sendToBridge("STOP");
  }
}

// =====================================================================
// Demo: a synthetic body so the sketch looks alive without the camera
// =====================================================================

float smoothstep(float a, float b, float x) {
  float t = constrain((x - a) / (b - a), 0, 1);
  return t * t * (3 - 2 * t);
}

void demoPose(float tSec, float[][] out) {
  float t = tSec % 15.0;
  float bend = 0, fall = 0, wave = 0;
  if (t < 4) {
    wave = sin(PI * t / 4);                                   // standing, waving
  } else if (t < 6.5) {
    bend = smoothstep(4, 6, t) * 0.85;                        // legs buckle
  } else if (t < 7.6) {
    float u = smoothstep(6.5, 7.6, t);                        // collapse sideways
    bend = 0.85 - 0.5 * u;
    fall = u;
  } else if (t < 13) {
    bend = 0.35;                                              // on the ground
    fall = 1;
  } else {
    float u = smoothstep(13, 14.6, t);                        // gets back up
    fall = 1 - u;
    bend = 0.35 * (1 - u);
  }
  float sway = sin(t * 8);
  float breathe = sin(t * 2) * 0.006;

  for (int i = 0; i < N; i++) { out[i][0] = 0.5; out[i][1] = 0.5; out[i][2] = 0; }

  float hipY = 0.92 - 0.38 * bend;
  float shY  = hipY + 0.50 + breathe;
  float kneeX = 0.14 + 0.16 * bend, kneeY = 0.48 - 0.10 * bend;

  setLocal(out, NOSE, 0, shY + 0.22);
  setLocal(out, L_SH, -0.20, shY);
  setLocal(out, R_SH,  0.20, shY);
  // left arm hangs, reaches forward (looks raised) as the knees bend
  setLocal(out, L_EL, -0.25 - 0.05 * bend, shY - 0.28 + 0.15 * bend);
  setLocal(out, L_WR, -0.27 - 0.03 * bend, shY - 0.55 + 0.35 * bend);
  // right arm waves while standing
  float rex = lerp(0.25, 0.42, wave) + 0.05 * bend, rey = lerp(shY - 0.28, shY - 0.02, wave) + 0.15 * bend;
  float rwx = lerp(0.27, 0.48 + 0.08 * sway, wave) + 0.03 * bend, rwy = lerp(shY - 0.55, shY + 0.25, wave) + 0.35 * bend * (1 - wave);
  setLocal(out, R_EL, rex, rey);
  setLocal(out, R_WR, rwx, rwy);
  setLocal(out, L_HIP, -0.13, hipY);
  setLocal(out, R_HIP,  0.13, hipY);
  setLocal(out, L_KN, -kneeX, kneeY);
  setLocal(out, R_KN,  kneeX, kneeY);
  setLocal(out, L_AN, -0.13, 0.06);
  setLocal(out, R_AN,  0.13, 0.06);
  setLocal(out, L_FT, -0.18, 0.0);
  setLocal(out, R_FT,  0.18, 0.0);

  // tip the whole body over around the feet, in the image plane
  float phi = radians(84) * fall;
  float cs = cos(phi), sn = sin(phi);
  float aspect = 720.0 / 1280.0;
  for (int i = 0; i < N; i++) {
    if (out[i][2] == 0) continue;
    float X = out[i][0], Y = out[i][1];
    float xr = X * cs + Y * sn;
    float yr = -X * sn + Y * cs;
    out[i][0] = 0.40 + xr * 0.40 * aspect;
    out[i][1] = 0.80 - yr * 0.40 - 0.10 * fall;   // lift so the lying body clears the HUD
  }
}

void setLocal(float[][] out, int i, float x, float y) {
  out[i][0] = x;
  out[i][1] = y;
  out[i][2] = 1;
}
