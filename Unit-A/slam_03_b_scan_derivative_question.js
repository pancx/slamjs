/** Compute the derivative of a scan. */
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

var minimumValidDistance = 20.0;

/** Read the logfile which contains all scans. */
var logfile = new LegoLogfile();
logfile.read('robot4_scan.txt');

/** Pick one scan. */
var scanNo = 7;
var scan = logfile.scanData[scanNo];

/** Compute derivative, (-1, 0, 1) mask. */
var der = computeDerivative(scan, minimumValidDistance);

/** Plot scan and derivative. */
plot({
  data:   { scan, der },
  filename: 'scan7.png'
});