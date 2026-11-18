import * as THREE from 'three';
export const FLIGHT_CONFIG=Object.freeze({
massKg:0.5,
gravity:9.80665,
inertiaKgM2:{x:0.025,y:0.045,z:0.025},
motor:{thrustCoefficient:3e-6,maxOmegaRadS:1000,spoolUpSeconds:0.05,spoolDownSeconds:0.06,yawCoefficient:0.02,rollPitchCoefficient:0.17},
drag:{linear:0.02,quadratic:0.0006},
maxTiltRad:THREE.MathUtils.degToRad(35),
maxAngularRate:6,
fixedStep:1/240,
maxSubSteps:32,
groundRestitution:0.08,
groundFriction:0.82
});
const clamp = THREE.MathUtils.clamp;
const expApproach = (current, target, tau, dt) => {
  return target + (current - target) * Math.exp(-dt / Math.max(tau, 1e-4));
};
export class DronePhysics {
  constructor(config = {}) {
    this.config = {
      ...FLIGHT_CONFIG,
      ...config,
      motor: {
        ...FLIGHT_CONFIG.motor,
        ...(config.motor || {})
      },
      drag: {
        ...FLIGHT_CONFIG.drag,
        ...(config.drag || {})
      },
      inertiaKgM2: {
        ...FLIGHT_CONFIG.inertiaKgM2,
        ...(config.inertiaKgM2 || {})
      }
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
    this._omega = new THREE.Vector3();
    this._gyroscopic = new THREE.Vector3();
    this._angularAccel = new THREE.Vector3();
    this._rotationAxis = new THREE.Vector3();
    this._deltaRotation = new THREE.Quaternion();
    this._euler = new THREE.Euler(0, 0, 0, 'YXZ');
    this.reset();
  }
reset(position=new THREE.Vector3(0,14.5,0),velocity=null){
this.position.copy(position);
if(velocity)this.velocity.copy(velocity);else this.velocity.set(0,0,0);
this.orientation.identity();
this.angularVelocity.set(0,0,0);
const c=this.config,hover=Math.sqrt(c.massKg*c.gravity/4/c.motor.thrustCoefficient);
this.motorOmega.fill(hover);
this.motorThrust.fill(c.motor.thrustCoefficient*hover*hover);
this.acceleration.set(0,0,0);
this._accumulator=0;
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
    const motor = c.motor;
    const mass = Math.max(c.massKg, 0.001);
    const desiredCollective = clamp(
      Number.isFinite(command.collectiveN) ? command.collectiveN : 0,
      0,
      4 * motor.thrustCoefficient * motor.maxOmegaRadS ** 2
    );
    const tau = command.torques || {
      x: 0,
      y: 0,
      z: 0
    };
    const torqueX = Number.isFinite(tau.x) ? tau.x : 0;
    const torqueY = Number.isFinite(tau.y) ? tau.y : 0;
    const torqueZ = Number.isFinite(tau.z) ? tau.z : 0;
    const maxMotorThrust =
      motor.thrustCoefficient * motor.maxOmegaRadS ** 2;
    const cf = motor.rollPitchCoefficient;
    const cd = motor.yawCoefficient;
    const mix = [
      [1, -1 / cf,  1 / cf, -1 / cd],
      [1,  1 / cf,  1 / cf,  1 / cd],
      [1,  1 / cf, -1 / cf, -1 / cd],
      [1, -1 / cf, -1 / cf,  1 / cd]
    ];
    const targets = mix.map(row => {
      const thrust = (
        row[0] * desiredCollective +
        row[1] * torqueX +
        row[2] * torqueZ +
        row[3] * torqueY
      ) / 4;
      return clamp(thrust, 0, maxMotorThrust);
    });
    for (let i = 0; i < 4; i++) {
      const targetOmega = Math.sqrt(
        targets[i] / motor.thrustCoefficient
      );
      const spoolTime =
        targetOmega > this.motorOmega[i]
          ? motor.spoolUpSeconds
          : motor.spoolDownSeconds;
      this.motorOmega[i] = expApproach(
        this.motorOmega[i],
        targetOmega,
        spoolTime,
        dt
      );
      this.motorThrust[i] =
        motor.thrustCoefficient * this.motorOmega[i] ** 2;
    }
    const totalThrust = this.motorThrust.reduce(
      (sum, thrust) => sum + thrust,
      0
    );
    this._worldThrust
      .copy(this._bodyUp)
      .applyQuaternion(this.orientation)
      .multiplyScalar(totalThrust);
    const speed = this.velocity.length();
    this._drag
      .copy(this.velocity)
      .multiplyScalar(-c.drag.linear)
      .addScaledVector(
        this.velocity,
        -c.drag.quadratic * speed
      );
    this.acceleration
      .copy(this._worldThrust)
      .add(this._drag)
      .multiplyScalar(1 / mass);
    this.acceleration.y -= c.gravity;
    this.velocity.addScaledVector(
      this.acceleration,
      dt
    );
    this.position.addScaledVector(
      this.velocity,
      dt
    );
    const f = this.motorThrust;
    this._torque.set(
      cf * (-f[0] + f[1] + f[2] - f[3]),

      cd * (-f[0] + f[1] - f[2] + f[3]),

      cf * (f[0] + f[1] - f[2] - f[3])
    );
    const I = c.inertiaKgM2;
    const Ix = Math.max(I.x, 1e-5);
    const Iy = Math.max(I.y, 1e-5);
    const Iz = Math.max(I.z, 1e-5);
    this._omega.set(
      this.angularVelocity.x * Ix,
      this.angularVelocity.y * Iy,
      this.angularVelocity.z * Iz
    );
    this._gyroscopic.crossVectors(
      this.angularVelocity,
      this._omega
    );
    this._angularAccel.set(
      (this._torque.x - this._gyroscopic.x) / Ix,
      (this._torque.y - this._gyroscopic.y) / Iy,
      (this._torque.z - this._gyroscopic.z) / Iz
    );
    this.angularVelocity.addScaledVector(
      this._angularAccel,
      dt
    );
    this.angularVelocity.clampLength(
      0,
      c.maxAngularRate
    );
    const angularSpeed = this.angularVelocity.length();
    if (angularSpeed > 1e-8) {
      this._rotationAxis
        .copy(this.angularVelocity)
        .multiplyScalar(1 / angularSpeed);
      this._deltaRotation.setFromAxisAngle(
        this._rotationAxis,
        angularSpeed * dt
      );
      this.orientation
        .multiply(this._deltaRotation)
        .normalize();
    }
    if (
      command.groundContact !== false &&
      Number.isFinite(command.groundY)
    ) {
      const groundY = command.groundY;
      if (this.position.y < groundY) {
        this.position.y = groundY;
        if (this.velocity.y < 0) {
          this.velocity.y =
            -this.velocity.y * c.groundRestitution;
          if (Math.abs(this.velocity.y) < 0.15) {
            this.velocity.y = 0;
          }
        }
        const frictionFactor = Math.exp(
          -c.groundFriction * dt
        );

        this.velocity.x *= frictionFactor;
        this.velocity.z *= frictionFactor;
      }
    }
  }
  update(frameDt, commandFactory) {
    const c = this.config;

    if (!Number.isFinite(frameDt) || frameDt <= 0) {
      return 0;
    }
    this._accumulator += Math.min(frameDt, 0.1);
    let steps = 0;
    while (
      this._accumulator >= c.fixedStep &&
      steps < c.maxSubSteps
    ) {
      const command =
        typeof commandFactory === 'function'
          ? commandFactory(c.fixedStep, this)
          : commandFactory;

      this.step(c.fixedStep, command || {});
      this._accumulator -= c.fixedStep;
      steps++;
    }
    if (steps === c.maxSubSteps) {
      this._accumulator = 0;
    }
    return steps;
  }
}