/**
 * This file contains helper functions for Unit D of the SLAM lecture,
 * most of which were developed in earlier units.
 */
var fs = require('fs');
var LegoLogfile = require('./legoRobot').LegoLogfile;

/**
 * Utility to write a list of cylinders to (one line of) a given file.
 * Line header defines the start of each line, e.g. "D C" for a detected
 * cylinder or "W C" for a world cylinder.
 * @param  {file descriptor} fileDesc - file descriptor of a given file
 * @param  {String} lineHeader - the start of each line
 * @param  {Array} cylinderList - cylinders' coordinate
 */
function writeCylinders(fileDesc, lineHeader, cylinderList) {
  fs.writeSync(fileDesc, lineHeader);
  for (var c in cylinderList) 
    fs.writeSync(fileDesc, ' ' + cylinderList[c][0].toFixed(1) + ' ' + cylinderList[c][1].toFixed(1));
  fs.writeSync(fileDesc, '\r\n');
}

/**
 * Utility to write a list of error ellipses to (one line of) a given file.
 * Line header defines the start of each line.
 * @param  {file descriptor} fileDesc - file descriptor of a given file
 * @param  {String} lineHeader - the start of each line
 * @param  {Array} errorEllipseList Array of error ellipses
 */
function writeErrorEllipses(fileDesc, lineHeader, errorEllipseList) {
  fs.writeSync(fileDesc, lineHeader);
  for (var c in errorEllipseList) 
    fs.writeSync(fileDesc, ' ' + errorEllipseList[c][0].toFixed(3) + ' ' + errorEllipseList[c][1].toFixed(1) + ' ' + errorEllipseList[c][2].toFixed(1));
  fs.writeSync(fileDesc, '\r\n');
}

/**
 * Find the derivative in scan data, ignoring invalid measurements.
 * @param  {Array} scan - The Array of scan data.
 * @param  {number} minDist - An threshold value for computing derivating.
 * @return {Array} The result array of derivative.
 */
function computeDerivative(scan, minDist) {
  var jumps = [0];
  for(var i = 1; i < scan.length - 1; i++) {
    var l = scan[i-1];
    var r = scan[i+1];
    if (l > minDist && r > minDist) {
      var derivative = (r - l) / 2.0;
      jumps.push(derivative);
    }
    else 
      jumps.push(0);
  }
  jumps.push(0);
  return jumps;
}

/**
 * For each area between a left falling edge and a right rising edge,
 * determine the average ray number and the average depth.
 * @param  {Array} scan - Scan data 
 * @param  {Array} scanDerivative - Scan derivative
 * @param  {Number} jump - Threshold to determine if there is a cylinder.
 * @param  {Number} minDist - Filter the scan data smaller than minDist.
 * @return {Array} positions of cylinders
 */
function findCylinders(scan, scanDerivative, jump, minDist) {
  let cylinderList = [];
  let onCylinder = false;
  let [sumRay, sumDepth, rays] = [0.0, 0.0, 0];

  for (var i in scanDerivative) {
    if (scanDerivative[i] < -jump) {
      // Start a new cylinder, independent of onCylinder.
      onCylinder = true;
      [sumRay, sumDepth, rays] = [0.0, 0.0, 0];
    }
    else if (scanDerivative[i] > jump) {
      // Save cylinder if there was one.
      if (onCylinder && rays) 
        cylinderList.push([sumRay/rays, sumDepth/rays]);
      onCylinder = false;
    }
    // Always add point, if it is a valid measurement.
    else if (scan[i] > minDist) {
      sumRay += Number(i);
      sumDepth += scan[i];
      rays += 1;
    }
  }
  return cylinderList;
}

/**
 * This function does all processing needed to obtain the cylinder observations.
 * It matches the cylinders and returns distance and angle observations together
 * with the cylinder coordinates in the world system, the scanner
 * system, and the corresponding cylinder index (in the list of estimated parameters).
 * In detail:
 * - It takes scan data and detects cylinders.
 * - For every detected cylinder, it computes its world coordinate using
 *   the polar coordinates from the cylinder detection and the robot's pose,
 *   taking into account the scanner's displacement.
 * - Using the world coordinate, it finds the closest cylinder in the
 *   list of current (estimated) landmarks, which are part of the current state.
 *   
 * - If there is such a closest cylinder, the (distance, angle) pair from the
 *   scan measurement (these are the two observations), the (x, y) world
 *   coordinates of the cylinder as determined by the measurement, the (x, y)
 *   coordinates of the same cylinder in the scanner's coordinate system,
 *   and the index of the matched cylinder are added to the output list.
 *   The index is the cylinder number in the robot's current state.
 * - If there is no matching cylinder, the returned index will be -1.
 * @param  {Array} scan - scan data
 * @param  {Number} jump - Threshold to determine if there is a cylinder.
 * @param  {Number} minDist - Filter the scan data smaller than minDist.
 * @param  {Number} cylinderOffset - cylinder offset due to the radius of the cylinder
 * @param  {Object} robot - instance of ExtendedKalmanFilterSLAM
 * @param  {Number} maxCylinderDistance
 * @return {Array} Positions of cylinders in different coordinates.
 */
function getObservations(scan, jump, minDist, cylinderOffset, robot, maxCylinderDistance) {
  var der = computeDerivative(scan, minDist);
  var cylinders = findCylinders(scan, der, jump, minDist);
  // Compute scanner pose from robot pose.
  var scannerPose = [
    robot.state[0] + Math.cos(robot.state[2]) * robot.scannerDisplacement,
    robot.state[1] + Math.sin(robot.state[2]) * robot.scannerDisplacement,
    robot.state[2]];

  // For every detected cylinder which has a closest matching pole in the
  // cylinders that are part of the current state, put the measurement
  // (distance, angle) and the corresponding cylinder index into the result list.
  var result = [];
  for (var c in cylinders) {
    // Compute the angle and distance measurements.
    var angle = LegoLogfile.beamIndexToAngle(cylinders[c][0]);
    var distance = cylinders[c][1] + cylinderOffset;
    // Compute x, y of cylinder in world coordinates.
    var [xs, ys] = [distance*Math.cos(angle), distance*Math.sin(angle)];
    var [x, y] = LegoLogfile.scannerToWorld(scannerPose, [xs, ys]);
    // Find closest cylinder in the state.
    var bestDist2 = maxCylinderDistance * maxCylinderDistance;
    var bestIndex = -1;
    for (var index = 0; index < robot.numberOfLandmarks; index++) {
      var [poleX, poleY] = [robot.state[3+2*index], robot.state[3+2*index+1]];
      var [dx, dy] = [poleX - x, poleY - y];
      var dist2 = dx * dx + dy * dy;
      if (dist2 < bestDist2) {
        bestDist2 = dist2;
        bestIndex = index;        
      }
    }
    // Always add result to list. Note bestIndex may be -1.
    result.push([[distance, angle], [x, y], [xs, ys], bestIndex]);
  }

  return result;   
}

exports.writeCylinders = writeCylinders;
exports.writeErrorEllipses = writeErrorEllipses;
exports.getObservations = getObservations;

