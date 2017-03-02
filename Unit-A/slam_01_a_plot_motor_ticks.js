/** Plot the ticks from the left and right motor. */

var fs = require('fs');
var plot = require('plotter').plot;


var data = fs.readFileSync('robot4_motors.txt');

var ticks = data.toString().trim().split('\r\n');

var leftList=new Array(); 
var rightList=new Array(); 

for (var l in ticks)
{
  var sp = ticks[l].split(' ');
  leftList.push(Number(sp[2]));
  rightList.push(Number(sp[6]));
}

plot({
  data:		{leftList, rightList},
  filename:	'ticks.png'
});