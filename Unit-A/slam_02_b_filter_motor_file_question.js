/**
 * Implement the second move model for the Lego robot.
 * The difference to the first implementation is:
 * - added a scanner displacement
 * - added a different start pose (measured in the real world)
 * - result is now output to a file, as 'F' ('filtered') records.
 */
var fs = require('fs');
var LegoLogfile = require('./legoRobot').LegoLogfile;

/**
 * This function takes the old (x, y, heading) pose and the motor ticks
 * (ticks_left, ticks_right) and returns the new (x, y, heading).
 * @param  {Array} oldPose - Pose in [x, y, theta].
 * @param  {Array} motorTicks - Ticks of left and right motors.
 * @param  {Number} ticksToMm - Unit conversion between ticks and mm.
 * @param  {Number} robotWidth - The width of the two wheels of the robot.
 * @param  {Number} robotWidth - Empirically derived distance between scanner and assumed center of robot.
 * @return {Array} Pose in [x, y, theta].
 */
function filterStep(oldPose, motorTicks, ticksToMm, robotWidth, scannerDisplacement) {
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
    oldX -= Math.cos(oldTheta) * scannerDisplacement;
    oldY -= Math.sin(oldTheta) * scannerDisplacement;

    let l = motorTicks[0] * ticksToMm;
    let r = motorTicks[1] * ticksToMm;
    let alpha = (r - l) / robotWidth;
    let R = l / alpha;
    let newTheta = (alpha + oldTheta) % (2 * Math.PI);
    let newX = oldX + (R + robotWidth / 2.0) * (Math.sin(newTheta) - Math.sin(oldTheta));
    let newY = oldY + (R + robotWidth / 2.0) * (- Math.cos(newTheta) + Math.cos(oldTheta));

    // Modification: add offset to compute location of scanner.
    newX += Math.cos(newTheta) * scannerDisplacement;
    newY += Math.sin(newTheta) * scannerDisplacement;

    return [newX, newY, newTheta];
  }
}



// Empirically derived distance between scanner and assumed
// center of robot.
var scannerDisplacement = 30.0;

// Empirically derived conversion from ticks to mm.
var ticksToMm = 0.349;

// Measured width of the robot (wheel gauge), in mm.
var robotWidth = 150.0;

// Measured start position.
var pose = [1850.0, 1897.0, 213.0 / 180.0 * Math.PI];

// Read data.
var logfile = new LegoLogfile();
logfile.read('robot4_motors.txt');

// Loop over all motor tick records generate filtered position list.
var filtered = [];
for (var i in logfile.motorTicks) {
  pose = filterStep(pose, logfile.motorTicks[i], ticksToMm, robotWidth, scannerDisplacement);
  filtered.push(pose);
}

// Write all filtered positions to file.
var f = fs.openSync('posesFromTicks.txt', 'w');

for (var j in filtered) 
  fs.writeSync(f, 'F ' + filtered[j][0] + ' ' + filtered[j][1] + ' ' + filtered[j][2] + '\r\n');
fs.closeSync(f);