import * as THREE from 'three';

const clamp = THREE.MathUtils.clamp;
const wrapAngle = (angle) => Math.atan2(Math.sin(angle), Math.cos(angle));

export class FlightController {
  constructor(config = {}) {
    this.config = {
      pilotFullScaleDeg: 18,
      rollInputSign: 1,
      climbInputSign: 1,
      yawInputSign: -1,
      maxLateralSpeed: 8,
      lateralKp: 2.8,
      lateralBrake: 3.2,
      speedKp: 1.5,
      maxClimbSpeed: 5,
      altitudeKp: 2.4,
      climbKd: 2.0,
      maxVerticalAcceleration: 3.5,
      attitudeKp: 4.0,
      attitudeKd: 1.8,
      yawKp: 1.8,
      yawKd: 0.8,
      maxTorque: 0.12,
      maxYawTorque: 0.035,
      inputSmoothing: 0.045,
      tiltCompensationFloor: 0.65,
      maxTiltRad: THREE.MathUtils.degToRad(30),
      laneLimit: 11,
      ...config
    };

    this.targetAltitude = 14.5;
    this.targetForwardSpeed = 15;
    this.targetYaw = 0;
    this.altFloor = -Infinity;
    this.altCeil = Infinity;
    this.sPitch = 0;
    this.sRoll = 0;
    this._euler = new THREE.Euler(0, 0, 0, 'YXZ');
  }

  reset(altitude = 14.5) {
    this.targetAltitude = altitude;
    this.targetYaw = 0;
    this.sPitch = 0;
    this.sRoll = 0;
  }

  limitMotorTorques(collective, torque, motor) {
    const maxF = motor.thrustCoefficient * motor.maxOmegaRadS ** 2;
    const col = clamp(collective, 0, 4 * maxF);
    const cf = motor.rollPitchCoefficient;
    const cd = motor.yawCoefficient;

    const motorForces = (scaleRP, scaleYaw) => [
      (col - torque.x * scaleRP / cf + torque.z * scaleRP / cf - torque.y * scaleYaw / cd) / 4,
      (col + torque.x * scaleRP / cf + torque.z * scaleRP / cf + torque.y * scaleYaw / cd) / 4,
      (col + torque.x * scaleRP / cf - torque.z * scaleRP / cf - torque.y * scaleYaw / cd) / 4,
      (col - torque.x * scaleRP / cf - torque.z * scaleRP / cf + torque.y * scaleYaw / cd) / 4
    ];

    const feasible = (rp, yaw) => motorForces(rp, yaw).every((f) => f >= 0 && f <= maxF);
    if (feasible(1, 1)) return torque;

    let yawScale = 1;
    if (!feasible(1, 0)) yawScale = 0;
    else {
      let lo = 0, hi = 1;
      for (let i = 0; i < 14; i++) {
        const mid = (lo + hi) / 2;
        if (feasible(1, mid)) lo = mid;
        else hi = mid;
      }
      yawScale = lo;
    }

    let rpScale = 1;
    if (!feasible(1, yawScale)) {
      let lo = 0, hi = 1;
      for (let i = 0; i < 14; i++) {
        const mid = (lo + hi) / 2;
        if (feasible(mid, 0)) lo = mid;
        else hi = mid;
      }
      rpScale = lo;
      yawScale = 0;
    }

    return {
      x: torque.x * rpScale,
      y: torque.y * yawScale,
      z: torque.z * rpScale
    };
  }

  compute(dt, physics, pilot, groundY) {
    const c = this.config;
    const pc = physics.config;
    const gravity = pc.gravity;
    const mass = pc.massKg;
    const p = physics.position;
    const v = physics.velocity;
    const w = physics.angularVelocity;
    const e = physics.getEuler(this._euler);

    dt = Math.max(1e-4, Math.min(dt, 0.05));
    const smoothing = 1 - Math.exp(-dt / Math.max(c.inputSmoothing, 1e-3));
    const fullScale = Math.max(c.pilotFullScaleDeg, 1);

    this.sPitch += (clamp((pilot.angleX || 0) / fullScale, -1, 1) - this.sPitch) * smoothing;
    this.sRoll += (clamp((pilot.angleY || 0) / fullScale, -1, 1) - this.sRoll) * smoothing;

    if (Number.isFinite(pilot.angleZ)) {
      this.targetYaw = THREE.MathUtils.degToRad(pilot.angleZ) * c.yawInputSign;
    }
    this.targetYaw = wrapAngle(this.targetYaw);

    const climbCommand = this.sPitch * c.climbInputSign * c.maxClimbSpeed;
    if (Math.abs(this.sPitch) > 0.04) {
      this.targetAltitude += climbCommand * dt;
    }
    this.targetAltitude = clamp(this.targetAltitude, this.altFloor, this.altCeil);

    const altitudeError = this.targetAltitude - p.y;
    const verticalAccel = clamp(
      c.altitudeKp * altitudeError + c.climbKd * (climbCommand - v.y),
      -c.maxVerticalAcceleration,
      c.maxVerticalAcceleration
    );

    const dragAccel = (pc.drag.linear + pc.drag.quadratic * v.length()) / Math.max(mass, 1e-3);
    let lateralSpeedCommand = this.sRoll * c.rollInputSign * c.maxLateralSpeed;
    const lane = c.laneLimit;
    if (p.x > lane) lateralSpeedCommand = -(p.x - lane) * 2;
    else lateralSpeedCommand = Math.min(lateralSpeedCommand, Math.sqrt(Math.max(0, 2 * c.lateralBrake * (lane - p.x))));
    if (p.x < -lane) lateralSpeedCommand = (-lane - p.x) * 2;
    else lateralSpeedCommand = Math.max(lateralSpeedCommand, -Math.sqrt(Math.max(0, 2 * c.lateralBrake * (lane + p.x))));

    let ax = c.lateralKp * (lateralSpeedCommand - v.x) + dragAccel * v.x;
    let az = c.speedKp * (-this.targetForwardSpeed - v.z) + dragAccel * v.z;

    const thrustY = Math.max(gravity + verticalAccel + dragAccel * v.y, gravity * 0.5);
    const maxHorizontalAccel = thrustY * Math.tan(c.maxTiltRad);
    const horizontalMagnitude = Math.hypot(ax, az);
    if (horizontalMagnitude > maxHorizontalAccel) {
      const scale = maxHorizontalAccel / horizontalMagnitude;
      ax *= scale;
      az *= scale;
    }

    const cy = Math.cos(e.y), sy = Math.sin(e.y);
    const bodyX = ax * cy - az * sy;
    const bodyZ = ax * sy + az * cy;
    const desiredPitch = clamp(Math.atan2(bodyZ, thrustY), -c.maxTiltRad, c.maxTiltRad);
    const desiredRoll = clamp(-Math.atan2(bodyX, thrustY), -c.maxTiltRad, c.maxTiltRad);

    const tx = c.attitudeKp * (desiredPitch - e.x) - c.attitudeKd * w.x;
    const tz = c.attitudeKp * (desiredRoll - e.z) - c.attitudeKd * w.z;
    const yawError = wrapAngle(this.targetYaw - e.y);
    const ty = c.yawKp * yawError - c.yawKd * w.y;

    const torques = {
      x: clamp(tx, -c.maxTorque, c.maxTorque),
      y: clamp(ty, -c.maxYawTorque, c.maxYawTorque),
      z: clamp(tz, -c.maxTorque, c.maxTorque)
    };

    const tiltFactor = Math.max(c.tiltCompensationFloor, Math.cos(e.x) * Math.cos(e.z));
    const collectiveN = mass * thrustY / tiltFactor;

    return {
      collectiveN,
      torques: this.limitMotorTorques(collectiveN, torques, pc.motor),
      groundY
    };
  }
}
