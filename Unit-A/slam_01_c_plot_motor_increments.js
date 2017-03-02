/** Plot the increments of the left and right motor. */
var plot = require('plotter').plot;
var LegoLogfile = require('./legoRobot').LegoLogfile;

var logfile = new LegoLogfile();
logfile.read('robot4_motors.txt');
var left = [];
var right = [];
for(var i in logfile.motorTicks) {
  left.push(logfile.motorTicks[i][0]);
  right.push(logfile.motorTicks[i][1]);
}
plot({
  data:   {left, right},
  filename: 'motorIncrements.png'
});