/**
 * Skid-Steer Kinematics
 *
 * Coordinate system conventions:
 *   - X axis: positive up
 *   - Y axis: perpendicular to X axis, positive to the left
 *   - theta: heading angle, counterclockwise is positive
 *
 * Wheel layout (top view):
 *        Forward direction ↑
 *    FL ──────── FR
 *    |     ↑     |
 *    |     L     |
 *    |     ↓     |
 *    RL ──────── RR
 *        <- W ->
 *
 * @param {Object} pose       - Current pose { x, y, theta }
 * @param {number} vFL        - Front-left wheel linear speed (m/s)
 * @param {number} vFR        - Front-right wheel linear speed (m/s)
 * @param {number} vRL        - Rear-left wheel linear speed (m/s)
 * @param {number} vRR        - Rear-right wheel linear speed (m/s)
 * @param {number} w          - Track width, distance between left/right wheel centers (m)
 * @param {number} l          - Wheelbase, distance between front/rear axles (m)
 * @param {number} dt         - Time step (s)
 * @returns {Object}          - Updated pose and motion state information
 */
function calculatePositionChange(pose, vFL, vFR, vRL, vRR, w, l, dt) {
  // Step 1: compute equivalent speeds for left and right sides
  const vL = (vFL + vRL) / 2;
  const vR = (vFR + vRR) / 2;

  // Step 2: compute body kinematics
  const vx = (vL + vR) / 2;           // Longitudinal velocity (m/s)
  const omega = (vR - vL) / w;         // Yaw rate (rad/s)

  // Step 3: compute turning radius
  const EPS = 1e-9;
  const radius = Math.abs(omega) > EPS ? vx / omega : Infinity;

  // Step 4: update pose
  // Use exact arc integration (more accurate than Euler integration)
  let newX, newY, newTheta;

  if (Math.abs(omega) > EPS) {
    // Curved motion: exact circular-arc integration
    newTheta = pose.theta + omega * dt;
    newX = pose.x + (vx / omega) * (Math.sin(newTheta) - Math.sin(pose.theta));
    newY = pose.y - (vx / omega) * (Math.cos(newTheta) - Math.cos(pose.theta));
  } else {
    // Straight motion: omega ≈ 0, reduce to linear update
    newTheta = pose.theta;
    newX = pose.x + vx * Math.cos(pose.theta) * dt;
    newY = pose.y + vx * Math.sin(pose.theta) * dt;
  }

  // Step 5: normalize angle to [-pi, pi]
  newTheta = normalizeAngle(newTheta);

  // Step 6: detect speed differences on the same side (slip indicator)
  const slipLeft  = Math.abs(vFL - vRL);
  const slipRight = Math.abs(vFR - vRR);
  const lateralSlipIndicator = (slipLeft + slipRight) / 2;

  // Step 7: classify motion state
  const motionState = classifyMotion(vL, vR, omega, vx);

  return {
    // Updated pose
    pose: {
      x: newX,
      y: newY,
      theta: newTheta,
    },
    // Pose delta
    delta: {
      dx: newX - pose.x,
      dy: newY - pose.y,
      dTheta: newTheta - pose.theta,
    },
    // Kinematic parameters
    kinematics: {
      vLeft: vL,               // Left-side equivalent speed (m/s)
      vRight: vR,              // Right-side equivalent speed (m/s)
      vx: vx,                  // Longitudinal velocity (m/s)
      omega: omega,            // Yaw rate (rad/s)
      omegaDeg: omega * 180 / Math.PI,  // Yaw rate (deg/s)
      radius: radius,          // Turning radius (m)
    },
    // Diagnostic information
    diagnostics: {
      lateralSlipIndicator,    // Lateral slip indicator (higher means more slip)
      motionState,             // Motion state description
    },
  };
}

/**
 * Normalize angle to [-pi, pi]
 */
function normalizeAngle(angle) {
  while (angle > Math.PI)  angle -= 2 * Math.PI;
  while (angle < -Math.PI) angle += 2 * Math.PI;
  return angle;
}

/**
 * Motion state classification
 */
function classifyMotion(vL, vR, omega, vx) {
  const EPS = 1e-6;

  if (Math.abs(vL) < EPS && Math.abs(vR) < EPS) {
    return "STATIONARY";             // Stationary
  }
  if (Math.abs(omega) < EPS) {
    return vx > 0 ? "STRAIGHT_FORWARD" : "STRAIGHT_BACKWARD";
  }
  if (Math.abs(vx) < EPS) {
    return omega > 0 ? "PIVOT_LEFT" : "PIVOT_RIGHT";  // In-place rotation
  }
  if (vx > 0) {
    return omega > 0 ? "FORWARD_LEFT" : "FORWARD_RIGHT";
  } else {
    return omega > 0 ? "BACKWARD_LEFT" : "BACKWARD_RIGHT";
  }
}