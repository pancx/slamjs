/** For each cylinder in the scan, find its ray and depth. */
var plot = require('plotter').plot;
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


var minimumValidDistance = 20.0;
var depthJump = 100.0;

// Read the logfile which contains all scans.
var logfile = new LegoLogfile();
logfile.read('robot4_scan.txt');

// Pick one scan.
var scan = logfile.scanData[8];

// Find cylinders.
var der = computeDerivative(scan, minimumValidDistance);
var cylinders = findCylinders(scan, der, depthJump, minimumValidDistance);

for (var i in cylinders)
  console.log(cylinders[i]);

/*
// todo: Plot results.
plot({
  data:   scan,
  filename: 'findCylinders.png'
});
scatter([c[0] for c in cylinders], [c[1] for c in cylinders],
    c='r', s=200)
show()
*/