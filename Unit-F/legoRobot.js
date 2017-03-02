/** @module legoRobot */
/* JavaScript routines useful for handling ikg's LEGO robot data. */
var fs = require('fs');

/**
 * In previous versions, the S record included the number of scan points.
 * If so, set this to true.
 * @type {Boolean} sRecordHasCount
 */
var sRecordHasCount = true;

/**
 * Class holding log data of our Lego robot.
 * The logfile understands the following records:
 * P reference position (of the robot)
 * S scan data
 * I indices of poles in the scan data (determined by an external algorithm)
 * M motor (ticks from the odometer) data
 * F filtered data (robot position, or position and heading angle)
 * L landmark (reference landmark, fixed)
 * D detected landmark, in the scanner's coordinate system
 */
class LegoLogfile {
  /** @constructor */
  constructor() {
    this.referencePositions = new Array();
    this.scanData = new Array(); 
    this.poleIndices = new Array(); 
    this.motorTicks = new Array(); 
    this.filteredPositions = new Array(); 
    this.landmarks = new Array(); 
    this.detectedCylinders = new Array();
    this.lastTicks = null; 
  }

  /**
   * Reads log data from file. Calling this multiple times with different
   * files will result in a merge of the data, i.e. if one file contains
   * M and S data, and the other contains M and P data, then LegoLogfile
   * will contain S from the first file and M and P from the second file.
   * @param  {string} filename - a text file of log data
   */
  read(filename) {
    /** 
     * If information is read in repeatedly, replace the lists instead of appending,
     * but only replace those lists that are present in the data.
     */
    var firstReferencePositions = true;
    var firstScanData = true;
    var firstPoleIndices = true;
    var firstMotorTicks = true;
    var firstFilteredPositions = true;
    var firstLandmarks = true;
    var firstDetectedCylinders = true;

    var f = fs.readFileSync(filename).toString().trim().split('\r\n');
    for (var l in f) {
      /**
       * P is the reference position.
       * File format: P timestamp[in ms] x[in mm] y[in mm].
       * Stored: A list of tuples [(x, y), ...] in referencePositions.
       */
      var sp = f[l].split(' ');
      if (sp[0] == 'P') {
        if (firstReferencePositions) {
          this.referencePositions = [];
          firstReferencePositions = false;
        }
        this.referencePositions.push([Number(sp[2]), Number(sp[3])]);
      }

      /**
       * S is the scan data.
       * File format:
       *  S timestamp[in ms] distances[in mm] ...
       * Or, in previous versions (set sRecordHasCount to true):
       *  S timestamp[in ms] count distances[in mm] ...
       * Stored: A list of tuples [ [(scan1_distance,... ), (scan2_distance,...) ]
       *   containing all scans, in scanData.
       */
      else if (sp[0] == 'S') {
        if (firstScanData) {
          this.scanData = [];
          firstScanData = false;
        }
        if (sRecordHasCount)
          this.scanData.push(sp.slice(3, sp.length).map(Number));
        else
          this.scanData.push(sp.slice(2, sp.length).map(Number));
      }

      /**
       * I is indices of poles in the scan.
       * The indices are given in scan order (counterclockwise).
       * -1 means that the pole could not be clearly detected.
       * File format: I timestamp[in ms] index ...
       * Stored: A list of tuples of indices (including empty tuples):
       *  [(scan1_pole1, scan1_pole2,...), (scan2_pole1,...)...]  
       */
      else if (sp[0] == 'I') {
        if(firstPoleIndices) {
          this.poleIndices = [];
          firstPoleIndices = false;
        }
        this.poleIndices.push(sp.slice(2, sp.length).map(Number));
      }


      /**
       * M is the motor data.
       * File format: M timestamp[in ms] pos[in ticks] tachoCount[in ticks] acceleration[deg/s^2] rotationSpeed[deg/s] ...
       *   (4 values each for: left motor, right motor, and third motor (not used)).
       * Stored: A list of tuples [ (inc-left, inc-right), ... ] with tick increments, in motorTicks.
       * Note that the file contains absolute ticks, but motorTicks contains the increments (differences).
       */
      else if (sp[0] == 'M') {
        var ticks = [Number(sp[2]), Number(sp[6])];
        if (firstMotorTicks) {
          this.motorTicks = [];
          firstMotorTicks = false;
          this.lastTicks = ticks;
        }
        this.motorTicks.push([ticks[0]-this.lastTicks[0], ticks[1]-this.lastTicks[1]]);
        this.lastTicks = ticks;
      }

      /**
       * F is filtered trajectory. No time stamp is used.
       * File format: F x[in mm] y[in mm]
       * OR:          F x[in mm] y[in mm] heading[in radians]
       * Stored: A list of tuples, each tuple is (x y) or (x y heading)
       */
      else if (sp[0] == 'F') {
        if (firstFilteredPositions) {
          this.filteredPositions = [];
          firstFilteredPositions = false;             
        }
        this.filteredPositions.push(sp.slice(1, sp.length).map(Number));
      }

 
      /**
       * L is landmark. This is actually background information, independent
       * of time.
       * File format: L <type> info...
       * Supported types:
       * Cylinder: L C x y diameter.
       * Stored: List of (<type> info) tuples.  
       */
      else if (sp[0] == 'L') {
        if (firstLandmarks) {
          this.landmarks = [];
          firstLandmarks = false;
        }
        if (sp[1] == 'C') {
          this.landmarks.push(['C'].concat(sp.slice(2, sp.length).map(Number)));
        }
      }     

      /**
       * D is detected landmarks (in each scan).
       * File format: D <type> info...
       * Supported types:
       * Cylinder: D C x y x y ...
       * Stored: List of lists of (x, y) tuples of the cylinder positions,
       *  one list per scan.  
       */
      else if (sp[0] == 'D') {
        if (sp[1] == 'C') {
          if (firstDetectedCylinders) {
            this.detectedCylinders = [];
            firstDetectedCylinders = false;
          }
          var cyl = sp.slice(2, sp.length).map(Number);
          var cylArray = [];
          var i = 0;
          while (i < cyl.length/2) {
            cylArray.push([cyl[2*i], cyl[2*i+1]]);
            i++;
          }
          this.detectedCylinders.push(cylArray);
        }
      }      
    }
  }

  /**
   * Return the number of entries. Take the max, since some lists may be empty.
   * @return {Number} The number of entries.
   */
  size() {
    return Math.max(this.referencePositions.length, this.scanData.length,
                    this.poleIndices.length, this.motorTicks.length,
                    this.filteredPositions.length, this.detectedCylinders.length);
  }

  /**
   * Convert a beam index to an angle, in radians.
   * @param  {Number} i - The beam index
   * @param  {[Number]} mountingAngle - 
   * @return {Number} An angle in radians
   */
  static beamIndexToAngle(i, mountingAngle = -0.06981317007977318) {
    return (i - 330.0) * 0.006135923151543 + mountingAngle;
  }

  /**
   * Prints reference pos, number of scan points, and motor ticks. 
   * @param  {Number} i - The beam index
   * @return {String} A string contains reference pos, number of scan points, and motor ticks.
   */
  info(i) {
    var s = '';
    if (i < this.referencePositions.length)
      s += ' | ref-pos: ' + this.referencePositions[i][0] + ' ' + this.referencePositions[i][1];

    if (i < this.scanData.length) 
      s += ' | scan-points: ' + this.scanData[i].length;

    if(i < this.poleIndices.length) {
      var indices = this.poleIndices[i];
      if (indices.length != 0) {
        s += ' | pole-indices:';
        for (var idx in indices)
          s += ' ' + indices[idx];
      }
      else 
        s += ' | (no pole indices)';
    }

    if (i < this.motorTicks.length) 
      s += ' | motor: ' + this.motorTicks[i][0] + ' ' + this.motorTicks[i][1];

    if (i < this.filteredPositions.length) {
      var f = this.filteredPositions[i];
      s += ' | filtered-pos:';
      for (var j in f) 
        s += ' ' + f[j].toFixed(1);
    }
    return s;
  }
}

exports.LegoLogfile = LegoLogfile;