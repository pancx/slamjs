/** Implement the first move model for the Lego robot. */
var plot = require('plotter').plot;
var LegoLogfile = require('./legoRobot').LegoLogfile;

/**
 * This function takes the old (x, y, heading) pose and the motor ticks
 * (ticks_left, ticks_right) and returns the new (x, y, heading).
 * @param  {Array} oldPose - Pose in [x, y, theta].
 * @param  {Array} motorTicks - Ticks of left and right motors.
 * @param  {Number} ticksToMm - Unit conversion between ticks and mm.
 * @param  {Number} robotWidth - The width of the two wheels of the robot.
 * @return {Array} Pose in [x, y, theta].
 */
function filterStep(oldPose, motorTicks, ticksToMm, robotWidth) {
  // Find out if there is a turn at all.
  if (motorTicks[0] == motorTicks[1]) {
    // No turn. Just drive straight.
    let dist = motorTicks[0] * ticksToMm;
    let theta = oldPose[2];
    let x = oldPose[0] + dist * Math.cos(oldPose[2]);
    let y = oldPose[1] + dist * Math.sin(oldPose[2]);
    return [x, y, theta];
  }
  else {
    //Turn. Compute alpha, R, etc.
    // Get old center.
    let oldTheta = oldPose[2];
    let oldX = oldPose[0];
    let oldY = oldPose[1];

    //Modification: subtract offset to compute center.

    let l = motorTicks[0] * ticksToMm;
    let r = motorTicks[1] * ticksToMm;
    let alpha = (r - l) / robotWidth;
    let R = l / alpha;
    let newTheta = (alpha + oldTheta) % (2 * Math.PI);
    let newX = oldX + (R + robotWidth / 2.0) * (Math.sin(newTheta) - Math.sin(oldTheta));
    let newY = oldY + (R + robotWidth / 2.0) * (- Math.cos(newTheta) + Math.cos(oldTheta));

    return [newX, newY, newTheta];
  }
}


// Empirically derived conversion from ticks to mm.
var ticksToMm = 0.349;

// Measured width of the robot (wheel gauge), in mm.
var robotWidth = 150.0;

// Read data.
var logfile = new LegoLogfile();
logfile.read('robot4_motors.txt');

// Start at origin (0,0), looking along x axis (alpha = 0).
var pose = [0.0, 0.0, 0.0];

// Loop over all motor tick records generate filtered position list.
var filtered = [];
for (var i in logfile.motorTicks) {
  pose = filterStep(pose, logfile.motorTicks[i], ticksToMm, robotWidth);
  filtered.push(pose);
}

// print filtered
for (var j in filtered)
  console.log(filtered[j]);

/*
// todo: Plot filter motor.
// Draw result.
var p0 = [];
var p1 = [];
for (var j in filtered) {
  console.log(filtered[j]);
  p0.push(filtered[j][0]);
  p1.push(filtered[j][1]);
}*/