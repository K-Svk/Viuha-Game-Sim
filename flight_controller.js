import * as THREE from 'three';
const clamp=THREE.MathUtils.clamp;
export class FlightController{
constructor(config={}){
this.config={
maxTiltRad:THREE.MathUtils.degToRad(32),
pilotFullScaleDeg:18,
rollInputSign:1,
climbInputSign:1,
yawInputSign:-1,
maxLateralSpeed:13,
lateralKp:3.2,
lateralBrake:3.5,
speedKp:1.6,
maxClimbSpeed:9,
altitudeKp:6,
climbKd:4,
maxVerticalAcceleration:4.5,
attitudeKp:2,
attitudeKd:0.4,
yawKp:0.25,
yawKd:0.15,
maxTorque:0.5,
maxYawTorque:0.06,
inputSmoothing:0.035,
tiltCompensationFloor:0.6,
laneLimit:11,
...config};
this.targetAltitude=14.5;this.targetForwardSpeed=15;this.targetYaw=0;
this.altFloor=-Infinity;this.altCeil=Infinity;
this.sPitch=0;this.sRoll=0;this.sYaw=0;
this._euler=new THREE.Euler(0,0,0,'YXZ');}
reset(altitude=14.5){this.targetAltitude=altitude;this.targetYaw=0;this.sPitch=0;this.sRoll=0;this.sYaw=0;}
limitMotorTorques(collective,t,motor){
const maxF=motor.thrustCoefficient*motor.maxOmegaRadS**2;
const col=clamp(collective,0,4*maxF);
const cf=motor.rollPitchCoefficient,cd=motor.yawCoefficient;
const mix=[[1,-1/cf,1/cf,-1/cd],[1,1/cf,1/cf,1/cd],[1,1/cf,-1/cf,-1/cd],[1,-1/cf,-1/cf,1/cd]];
const ok=(sx,sy)=>mix.every(r=>{const f=(r[0]*col+r[1]*t.x*sx+r[2]*t.z*sx+r[3]*t.y*sy)/4;return f>=0&&f<=maxF;});
const bis=(fn)=>{let lo=0,hi=1;for(let i=0;i<12;i++){const mid=(lo+hi)/2;if(fn(mid))lo=mid;else hi=mid;}return lo;};
if(ok(1,1))return t;
// sacrifice yaw first, keep pitch/roll authority
if(ok(1,0))return{x:t.x,y:t.y*bis(s=>ok(1,s)),z:t.z};
const s=bis(q=>ok(q,0));
return{x:t.x*s,y:0,z:t.z*s};}
compute(dt,physics,pilot,groundY){
const c=this.config,pc=physics.config,g=pc.gravity,m=pc.massKg;
dt=Math.max(dt,1e-4);
const e=physics.getEuler(this._euler),v=physics.velocity,p=physics.position,w=physics.angularVelocity;
// light input smoothing only (about 35 ms) so keys feel instant
const a=1-Math.exp(-dt/Math.max(c.inputSmoothing,1e-3));
const fs=c.pilotFullScaleDeg;
this.sPitch+=(clamp((pilot.angleX||0)/fs,-1,1)-this.sPitch)*a;
this.sRoll+=(clamp((pilot.angleY||0)/fs,-1,1)-this.sRoll)*a;
const yawIn=Number.isFinite(pilot.angleZ)?THREE.MathUtils.degToRad(pilot.angleZ)*c.yawInputSign:this.targetYaw;
this.sYaw+=(yawIn-this.sYaw)*a;
// ---- vertical: pilot pitch = climb rate, release = altitude hold ----
const climbing=Math.abs(this.sPitch)>0.05;
const vFF=climbing?this.sPitch*c.climbInputSign*c.maxClimbSpeed:0;
if(climbing)this.targetAltitude=clamp(this.targetAltitude+vFF*dt,p.y-3,p.y+3);
this.targetAltitude=clamp(this.targetAltitude,this.altFloor,this.altCeil);
const aV=clamp(c.altitudeKp*(this.targetAltitude-p.y)+c.climbKd*(vFF-v.y),-c.maxVerticalAcceleration,c.maxVerticalAcceleration);
// ---- horizontal: pilot roll = lateral speed, runner speed = forward speed ----
const dk=(pc.drag.linear+pc.drag.quadratic*v.length())/m; // cancel drag so speeds are held exactly
let vxCmd=this.sRoll*c.rollInputSign*c.maxLateralSpeed;
const L=c.laneLimit;
if(p.x>L)vxCmd=-(p.x-L)*2;else vxCmd=Math.min(vxCmd,Math.sqrt(2*c.lateralBrake*(L-p.x)));
if(p.x<-L)vxCmd=(-L-p.x)*2;else vxCmd=Math.max(vxCmd,-Math.sqrt(2*c.lateralBrake*(L+p.x)));
let ax=c.lateralKp*(vxCmd-v.x)+dk*v.x;
let az=c.speedKp*(-this.targetForwardSpeed-v.z)+dk*v.z;
const Ty=Math.max(g+aV+dk*v.y,0.5*g);
const hMax=Ty*Math.tan(c.maxTiltRad),hMag=Math.hypot(ax,az);
if(hMag>hMax){const s=hMax/hMag;ax*=s;az*=s;}
// world accel -> body frame (works at any yaw)
const cy=Math.cos(e.y),sy=Math.sin(e.y);
const xb=ax*cy-az*sy,zb=ax*sy+az*cy;
const desPitch=Math.atan2(zb,Ty),desRoll=-Math.atan2(xb,Ty);
let tx=c.attitudeKp*(desPitch-e.x)-c.attitudeKd*w.x;
let tz=c.attitudeKp*(desRoll-e.z)-c.attitudeKd*w.z;
const yawErr=THREE.MathUtils.euclideanModulo(this.sYaw-e.y+Math.PI,Math.PI*2)-Math.PI;
let ty=c.yawKp*yawErr-c.yawKd*w.y;
tx=clamp(tx,-c.maxTorque,c.maxTorque);
tz=clamp(tz,-c.maxTorque,c.maxTorque);
ty=clamp(ty,-c.maxYawTorque,c.maxYawTorque);
const tilt=Math.max(c.tiltCompensationFloor,Math.cos(e.x)*Math.cos(e.z));
const collectiveN=m*Ty/tilt;
const torques=this.limitMotorTorques(collectiveN,{x:tx,y:ty,z:tz},pc.motor);
return{collectiveN,torques,groundY};}}