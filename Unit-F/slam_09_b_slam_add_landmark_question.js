/** EKF SLAM - adding one landmark. */
var fs = require('fs');
var LegoLogfile = require('./legoRobot').LegoLogfile;
var nj = require('numjs');
var numeric = require('./numeric');
var sl = require('./slam_f_library');

class ExtendedKalmanFilterSLAM {
  /**
   * @constructor
   * @param  {Array} state -Position and heading.
   * @param  {Array} covariance - Covariance in control space depends on move distance.
   * @param  {Number} robotWidth - The width of robot.
   * @param  {Number} scannerDisplacement - The distance between LiDAR and the center of the body.
   * @param  {Number} controlMotionFactor - Error in motor control.
   * @param  {Number} controlTurnFactor - Additional error due to slip when turning.
   */
  constructor(state, covariance, robotWidth, scannerDisplacement, controlMotionFactor, controlTurnFactor) {
    // The state. This is the core data of the Kalman filter.
    this.state = state;
    this.covariance = covariance;

    // Some constants.
    this.robotWidth = robotWidth;
    this.scannerDisplacement = scannerDisplacement;
    this.controlMotionFactor = controlMotionFactor;
    this.controlTurnFactor = controlTurnFactor;

    // Currently, the number of landmarks is zero.
    this.numberOfLandmarks = 0;
  }    

  /**
   * Generate the current state using the old state and control.
   * @param  {Array} state - Position and heading
   * @param  {Array} control - The control information.
   * @param  {Number} w - The width of robot.
   * @return {Array} New state
   */
  static g(state, control, w) {
    var [x, y, theta] = [state.get(0), state.get(1), state.get(2)];
    var [l, r] = control;
    var alpha, rad, g1, g2, g3; 
    if (r != l) {
      alpha = (r - l) / w;
      rad = l/alpha;
      g1 = x + (rad + w/2.)*(Math.sin(theta+alpha) - Math.sin(theta));
      g2 = y + (rad + w/2.)*(-Math.cos(theta+alpha) + Math.cos(theta));
      g3 = (theta + alpha + Math.PI) % (2*Math.PI) - Math.PI;
    }
    else {
      g1 = x + l * Math.cos(theta);
      g2 = y + l * Math.sin(theta);
      g3 = theta;
    }


    return nj.array([g1, g2, g3]); 
  }

  /**
   * The derivative of the state function.
   * @param  {Array} state - Position and heading
   * @param  {Array} control - The control information.
   * @param  {Number} w - The width of robot.
   * @return {Array} The derivative of the state
   */
  static dgDstate(state, control, w) {
    var theta = state.get(2);
    var [l, r] = control;
    if (r != l) {
      var alpha = (r-l)/w;
      var newTheta = theta + alpha;
      var rpw2 = l/alpha + w/2.0;
      var m = nj.array([[1.0, 0.0, rpw2*(Math.cos(newTheta) - Math.cos(theta))],
                [0.0, 1.0, rpw2*(Math.sin(newTheta) - Math.sin(theta))],
                [0.0, 0.0, 1.0]]);     
    }
    else
      m = nj.array([[1.0, 0.0, -l*Math.sin(theta)],
                [0.0, 1.0,  l*Math.cos(theta)],
                [0.0, 0.0,  1.0]]);
    return m;
  }

  /**
   * The control noise computation of the measurement
   * @param  {Array} state - Position and heading
   * @param  {Array} control - The control information.
   * @param  {Number} w - The width of robot.
   * @return {Array} The control noise of the measurement
   */
  static dgDcontrol(state, control, w) {
    var theta = state.get(2);
    var [l, r] = control;
    var rml, rml2, newTheta, dg1dl, dg2dl, dg1dr, dg2dr;
    if (r != l) {
      rml = r - l;
      rml2 = rml * rml;
      newTheta = theta + rml/w;
      dg1dl = w*r/rml2*(Math.sin(newTheta)-Math.sin(theta))  - (r+l)/(2*rml)*Math.cos(newTheta);
      dg2dl = w*r/rml2*(-Math.cos(newTheta)+Math.cos(theta)) - (r+l)/(2*rml)*Math.sin(newTheta);
      dg1dr = (-w*l)/rml2*(Math.sin(newTheta)-Math.sin(theta)) + (r+l)/(2*rml)*Math.cos(newTheta);
      dg2dr = (-w*l)/rml2*(-Math.cos(newTheta)+Math.cos(theta)) + (r+l)/(2*rml)*Math.sin(newTheta);
    }  
    else {
      dg1dl = 0.5*(Math.cos(theta) + l/w*Math.sin(theta));
      dg2dl = 0.5*(Math.sin(theta) - l/w*Math.cos(theta));
      dg1dr = 0.5*(-l/w*Math.sin(theta) + Math.cos(theta));
      dg2dr = 0.5*(l/w*Math.cos(theta) + Math.sin(theta));
    }
    var dg3dl = -1.0/w;
    var dg3dr = 1.0/w;
    var m = nj.array([[dg1dl, dg1dr], [dg2dl, dg2dr], [dg3dl, dg3dr]]);
        
    return m;
  }

  /**
   * The prediction step of the Kalman filter.
   * @param  {Array} control - The control information.
   */
  predict(control) {
    // covariance' = G * covariance * GT + R
    // where R = V * (covariance in control space) * VT.
    // Covariance in control space depends on move distance.
    var G3 = ExtendedKalmanFilterSLAM.dgDstate(this.state, control, this.robotWidth);

    var [left, right] = control;
    var leftVar = Math.pow((this.controlMotionFactor * left), 2) +
               Math.pow((this.controlTurnFactor * (left-right)), 2);
    var rightVar = Math.pow((this.controlMotionFactor * right), 2) +
                Math.pow((this.controlTurnFactor * (left-right)), 2);
    var controlCovariance = nj.diag([leftVar, rightVar]);
    var V = ExtendedKalmanFilterSLAM.dgDcontrol(this.state, control, this.robotWidth);
    var R3 = nj.dot(V, nj.dot(controlCovariance, V.T));

    var G = nj.identity(3 + 2 * this.numberOfLandmarks);
    for (var i = 0; i < 3; i++)
      for (var j = 0; j < 3; j++)
        G.set(i, j, G3.get(i, j));
    var R = nj.zeros([3 + 2 * this.numberOfLandmarks, 3 + 2 * this.numberOfLandmarks]);
    for (i = 0; i < 3; i++)
      for (j = 0; j < 3; j++)
        R.set(i, j, R3.get(i, j));

    // Now enlarge G3 and R3 to accomodate all landmarks. Then, compute the
    // new covariance matrix this.covariance.
    this.covariance = nj.add(nj.dot(G, nj.dot(this.covariance, G.T)), R);
    var temp = ExtendedKalmanFilterSLAM.g(this.state, control, this.robotWidth);
    for (i = 0; i < 3; i++)
      this.state.set(i, temp.get(i));
  }

  /**
   * Enlarge the current state and covariance matrix to include one more
   * landmark, which is given by its initialCoords (an [x, y] array).
   * Returns the index of the newly added landmark.
   * @param {Array} initialCoords - an [x, y] coord.
   * @return {Number} the index of the newly added landmark.
   */
  addLandmarkToState(initialCoords) {
    var [s1, s2] = [this.state.size, this.state.size + 2];
    var state = nj.zeros(s2);
    for (var i = 0; i < s1; i++) 
      state.set(i, this.state.get(i));
    state.set(s1, initialCoords[0]);
    state.set(s1 + 1, initialCoords[1]);
    this.numberOfLandmarks += 1;

    var covariance = nj.zeros([s2, s2]);
    for (var j = 0; j < s1; j++)
      for (var k = 0; k < s1; k++)
        covariance.set(j, k, this.covariance.get(j, k));
    covariance.set(s1, 1e10);
    covariance.set(s1 + 1, 1e10);

    this.state = state;
    this.covariance = covariance;

    return this.numberOfLandmarks - 1;
  } 
  
  /**
   * Returns a array of [x, y] arrayss of all landmark positions
   * @return {Array} a list of all landmark positions
   */
  getLandmarks() {
    var landmarks = [];
    for (var j = 0; j < this.numberOfLandmarks; j++) 
      landmarks.push([this.state.get(3+2*j), this.state.get(3+2*j+1)]);
    return landmarks;
  }

  /**
   * Returns a array of all error ellipses, one for each landmark.
   * @return {Array} a array of all error ellipses, one for each landmark.
   */
  getLandmarkErrorEllipses() {
    var ellipses = [];
    for (var i = 0; i < this.numberOfLandmarks; i++) {
      var j = 3 + 2 * i;
      ellipses.push(ExtendedKalmanFilterSLAM.getErrorEllipse(this.covariance.slice([j, j+2], [j, j+2])));
    }
    return ellipses;
  }

  /**
   * Return the position covariance (which is the upper 2x2 submatrix)
   * as a triple: (mainAxisAngle, stddev1, stddev2), where
   * mainAxisAngle is the angle (pointing direction) of the main axis,
   * along which the standard deviation is stddev1, and stddev2 is the
   * standard deviation along the other (orthogonal) axis.
   * @param  {Array} covariance - Covariance in control space depends on move distance.
   * @return {Array} the position covariance
   */
  static getErrorEllipse(covariance) {
    var cArray = [[covariance.get(0,0), covariance.get(0,1)], [covariance.get(1,0), covariance.get(1,1)]];

    var Eig = numeric.eig(cArray);
    var [eigenvals, eigenvects] = [nj.array(Eig.lambda.x), nj.array(Eig.E.x)];
    var angle = Math.atan2(eigenvects.get(1, 0), eigenvects.get(0, 0));

    return [angle, Math.sqrt(eigenvals.get(0)), Math.sqrt(eigenvals.get(1))];
  }
}   

// Robot constants.
var scannerDisplacement = 30.0;
var ticksToMm = 0.349;
var robotWidth = 155.0;

// Filter constants.
var controlMotionFactor = 0.35;  // Error in motor control.
var controlTurnFactor = 0.6;  // Additional error due to slip when turning.

// Arbitrary start position.
var initialState = nj.array([500.0, 0.0, 45.0 / 180.0 * Math.PI]);

// Covariance at start position.
var initialCovariance = nj.zeros([3,3]);

// Setup filter.
var kf = new ExtendedKalmanFilterSLAM(initialState, initialCovariance,
                              robotWidth, scannerDisplacement,
                              controlMotionFactor, controlTurnFactor);

// Just to test the algorithm, add one landmark.
kf.addLandmarkToState([400.0, 700.0]);
// To make the error ellipse visible, set a smaller variance.
if (kf.numberOfLandmarks > 0) {
  var [m, n] = kf.covariance.shape;
  kf.covariance.set(m-2,n-2, 300.0*300.0);  // 300 mm in x.
  kf.covariance.set(m-1,n-1, 500.0*500.0);  // 500 mm in y.
}
// Read data.
var logfile = new LegoLogfile();
logfile.read('robot4_motors.txt');

// Loop over all motor tick records and all measurements and generate
// filtered positions and covariances.
// This is the EKF SLAM loop.`
var f = fs.openSync('ekfSlamAddLandmarks.txt', 'w');
for (var i in logfile.motorTicks) {
  // Prediction.
  var control = logfile.motorTicks[i].map(x => x * ticksToMm);
  kf.predict(control);

  // End of EKF SLAM - from here on, data is written.

  // Output the center of the scanner, not the center of the robot.
  var temp = nj.add(kf.state.slice([0, 3]), nj.array([scannerDisplacement * Math.cos(kf.state.get(2)),
    scannerDisplacement * Math.sin(kf.state.get(2)),
    0.0]));

  fs.writeSync(f, `F ${temp.get(0)} ${temp.get(1)} ${temp.get(2)}\r\n`);

  // Write covariance matrix in angle stddev1 stddev2 stddev-heading form
  var e = ExtendedKalmanFilterSLAM.getErrorEllipse(kf.covariance);
  fs.writeSync(f, `E ${e[0]} ${e[1]} ${e[2]} ${Math.sqrt(kf.covariance.get(2,2))}\r\n`);
  // Write estimates of landmarks.
  sl.writeCylinders(f, 'W C', kf.getLandmarks());
  // Write error ellipses of landmarks.
  sl.writeErrorEllipses(f, 'W E', kf.getLandmarkErrorEllipses());
}

fs.closeSync(f);


