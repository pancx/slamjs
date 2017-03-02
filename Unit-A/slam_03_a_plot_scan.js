/** Plot a scan of the robot using matplotlib. */
var plot = require('plotter').plot;
var LegoLogfile = require('./legoRobot').LegoLogfile;

/** Read the logfile which contains all scans. */
var logfile = new LegoLogfile();
logfile.read('robot4_scan.txt');

/** Plot one scan. */
plot({
  data:   logfile.scanData[8],
  filename: 'scan.png'
});