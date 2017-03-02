/**
 * For each cylinder in the scan, find its cartesian coordinates,
 * in the scanner's coordinate system.
 * Write the result to a file which contains all cylinders, for all scans.
 */
var fs = require('fs');
var LegoLogfile = require('./legoRobot').LegoLogfile;

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
 * Given detected cylinder coordinates: (beamId, distance), return
 * cartesian coordinates (x, y). This is a polar to cartesian conversion
 * with an added offset.
 * @param  {Array} cylinders - cylinders (beamId, distance)
 * @param  {Number} cylinderOffset - cylinder offset due to the radius of the cylinder
 * @return {Array} cartesian coordinates (x, y)
 */
function computeCartesianCoordinates(cylinders, cylinderOffset) {
  var result = [];
  for (var c in cylinders) {
    var angle = LegoLogfile.beamIndexToAngle(cylinders[c][0]);
    var r = cylinders[c][1] + cylinderOffset;
    result.push([r*Math.cos(angle), r*Math.sin(angle)]);
  }
  return result;
}

var minimumValidDistance = 20.0;
var depthJump = 100.0;
var cylinderOffset = 90.0;

// Read the logfile which contains all scans.
var logfile = new LegoLogfile();
logfile.read('robot4_scan.txt');

// Write a result file containing all cylinder records.
// Format is: D C x[in mm] y[in mm] ...
// With zero or more points.
// Note 'D C' is also written for otherwise empty lines (no
// cylinders in scan)
var outFile = fs.openSync('cylinders.txt', 'w');
for (var i in logfile.scanData) {
  // Find cylinders.
  var der = computeDerivative(logfile.scanData[i], minimumValidDistance);
  var cylinders = findCylinders(logfile.scanData[i], der, depthJump, minimumValidDistance);
  var cartesianCylinders = computeCartesianCoordinates(cylinders, cylinderOffset);
  // Write to file.
  fs.writeSync(outFile, 'D C');
  for (var c in cartesianCylinders)
    fs.writeSync(outFile, ' ' + cartesianCylinders[c][0].toFixed(1) + ' ' + cartesianCylinders[c][1].toFixed(1));
  fs.writeSync(outFile, '\r\n');
}
fs.closeSync(outFile);