import * as THREE from 'three';

export const FLIGHT_CONFIG = Object.freeze({
  massKg: 0.5,
  gravity: 9.80665,
  inertiaKgM2: { x: 0.025, y: 0.045, z: 0.025 },
  motor: {
    thrustCoefficient: 3e-6,
    maxOmegaRadS: 1000,
    spoolUpSeconds: 0.08,
    spoolDownSeconds: 0.08,
    yawCoefficient: 0.02,
    rollPitchCoefficient: 0.17
  },
  drag: { linear: 0.08, quadratic: 0.0008 },
  maxTiltRad: THREE.MathUtils.degToRad(35),
  maxAngularRate: 5,
  fixedStep: 1 / 240,
  maxSubSteps: 24,
  groundRestitution: 0.04,
  groundFriction: 1.8
});

const clamp = THREE.MathUtils.clamp;
const approachExp = (current, target, tau, dt) =>
  target + (current - target) * Math.exp(-dt / Math.max(tau, 1e-4));

export class DronePhysics {
  constructor(config = {}) {
    this.config = {
      ...FLIGHT_CONFIG,
      ...config,
      motor: { ...FLIGHT_CONFIG.motor, ...(config.motor || {}) },
      drag: { ...FLIGHT_CONFIG.drag, ...(config.drag || {}) },
      inertiaKgM2: { ...FLIGHT_CONFIG.inertiaKgM2, ...(config.inertiaKgM2 || {}) }
    };

    this.position = new THREE.Vector3();
    this.velocity = new THREE.Vector3();
    this.orientation = new THREE.Quaternion();
    this.angularVelocity = new THREE.Vector3();
    this.motorOmega = [0, 0, 0, 0];
    this.motorThrust = [0, 0, 0, 0];
    this.acceleration = new THREE.Vector3();
    this._accumulator = 0;
    this._bodyUp = new THREE.Vector3(0, 1, 0);
    this._worldThrust = new THREE.Vector3();
    this._drag = new THREE.Vector3();
    this._torque = new THREE.Vector3();
    this._angularMomentum = new THREE.Vector3();
    this._gyro = new THREE.Vector3();
    this._angularAccel = new THREE.Vector3();
    this._axis = new THREE.Vector3();
    this._deltaQuat = new THREE.Quaternion();
    this._euler = new THREE.Euler(0, 0, 0, 'YXZ');
    this.reset();
  }

  reset(position = new THREE.Vector3(0, 14.5, 0), velocity = null) {
    this.position.copy(position);
    this.velocity.copy(velocity || new THREE.Vector3());
    this.orientation.identity();
    this.angularVelocity.set(0, 0, 0);
    const m = this.config.motor;
    const hoverOmega = Math.sqrt(this.config.massKg * this.config.gravity / (4 * m.thrustCoefficient));
    this.motorOmega.fill(hoverOmega);
    this.motorThrust.fill(m.thrustCoefficient * hoverOmega * hoverOmega);
    this.acceleration.set(0, 0, 0);
    this._accumulator = 0;
  }

  getEuler(target = new THREE.Euler()) {
    return target.setFromQuaternion(this.orientation, 'YXZ');
  }

  getSpeed() {
    return this.velocity.length();
  }

  step(dt, command = {}) {
    if (!Number.isFinite(dt) || dt <= 0) return;
    const c = this.config;
    const m = c.motor;
    const mass = Math.max(c.massKg, 1e-3);
    const maxMotorThrust = m.thrustCoefficient * m.maxOmegaRadS ** 2;
    const collective = clamp(Number.isFinite(command.collectiveN) ? command.collectiveN : 0, 0, 4 * maxMotorThrust);
    const t = command.torques || { x: 0, y: 0, z: 0 };
    const tx = Number.isFinite(t.x) ? t.x : 0;
    const ty = Number.isFinite(t.y) ? t.y : 0;
    const tz = Number.isFinite(t.z) ? t.z : 0;
    const cf = Math.max(m.rollPitchCoefficient, 1e-5);
    const cd = Math.max(m.yawCoefficient, 1e-5);

    // X-frame quad allocation. Motor forces are clipped individually;
    // the controller reduces torque when the requested mix is infeasible.
    const targets = [
      (collective - tx / cf + tz / cf - ty / cd) / 4,
      (collective + tx / cf + tz / cf + ty / cd) / 4,
      (collective + tx / cf - tz / cf - ty / cd) / 4,
      (collective - tx / cf - tz / cf + ty / cd) / 4
    ].map((f) => clamp(f, 0, maxMotorThrust));

    for (let i = 0; i < 4; i++) {
      const omegaTarget = Math.sqrt(targets[i] / m.thrustCoefficient);
      const tau = omegaTarget > this.motorOmega[i] ? m.spoolUpSeconds : m.spoolDownSeconds;
      this.motorOmega[i] = approachExp(this.motorOmega[i], omegaTarget, tau, dt);
      this.motorThrust[i] = m.thrustCoefficient * this.motorOmega[i] ** 2;
    }

    const totalThrust = this.motorThrust.reduce((sum, f) => sum + f, 0);
    this._worldThrust.copy(this._bodyUp).applyQuaternion(this.orientation).multiplyScalar(totalThrust);

    const speed = this.velocity.length();
    this._drag.copy(this.velocity)
      .multiplyScalar(-c.drag.linear)
      .addScaledVector(this.velocity, -c.drag.quadratic * speed);

    this.acceleration.copy(this._worldThrust).add(this._drag).multiplyScalar(1 / mass);
    this.acceleration.y -= c.gravity;

    // Semi-implicit Euler integration, stable at the fixed simulation step.
    this.velocity.addScaledVector(this.acceleration, dt);
    this.position.addScaledVector(this.velocity, dt);

    const f = this.motorThrust;
    this._torque.set(
      cf * (-f[0] + f[1] + f[2] - f[3]),
      cd * (-f[0] + f[1] - f[2] + f[3]),
      cf * (f[0] + f[1] - f[2] - f[3])
    );

    const I = c.inertiaKgM2;
    this._angularMomentum.set(this.angularVelocity.x * I.x, this.angularVelocity.y * I.y, this.angularVelocity.z * I.z);
    this._gyro.crossVectors(this.angularVelocity, this._angularMomentum);
    this._angularAccel.set(
      (this._torque.x - this._gyro.x) / Math.max(I.x, 1e-5),
      (this._torque.y - this._gyro.y) / Math.max(I.y, 1e-5),
      (this._torque.z - this._gyro.z) / Math.max(I.z, 1e-5)
    );

    this.angularVelocity.addScaledVector(this._angularAccel, dt).clampLength(0, c.maxAngularRate);
    const angularSpeed = this.angularVelocity.length();
    if (angularSpeed > 1e-8) {
      this._axis.copy(this.angularVelocity).multiplyScalar(1 / angularSpeed);
      this._deltaQuat.setFromAxisAngle(this._axis, angularSpeed * dt);
      this.orientation.multiply(this._deltaQuat).normalize();
    }

    if (command.groundContact !== false && Number.isFinite(command.groundY) && this.position.y < command.groundY) {
      this.position.y = command.groundY;
      if (this.velocity.y < 0) {
        this.velocity.y = -this.velocity.y * c.groundRestitution;
        if (Math.abs(this.velocity.y) < 0.12) this.velocity.y = 0;
      }
      const friction = Math.exp(-c.groundFriction * dt);
      this.velocity.x *= friction;
      this.velocity.z *= friction;
    }
  }

  update(frameDt, commandFactory) {
    if (!Number.isFinite(frameDt) || frameDt <= 0) return 0;
    const c = this.config;
    this._accumulator += Math.min(frameDt, 0.1);
    let steps = 0;
    while (this._accumulator >= c.fixedStep && steps < c.maxSubSteps) {
      const command = typeof commandFactory === 'function'
        ? commandFactory(c.fixedStep, this)
        : commandFactory;
      this.step(c.fixedStep, command || {});
      this._accumulator -= c.fixedStep;
      steps++;
    }
    if (steps >= c.maxSubSteps) this._accumulator = 0;
    return steps;
  }
}
