/** Print the increments of the left and right motor. */
var LegoLogfile = require('./legoRobot').LegoLogfile;
var logfile = new LegoLogfile();

//if __name__ == '__main__':
//todo: how to transfer to js?

logfile.read('robot4_motors.txt');
for (var i = 0; i < 20; i++)
  console.log(logfile.motorTicks[i]);