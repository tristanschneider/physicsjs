import { safeDivide, Vec2 } from './vec2';

//Always cube shaped Rigidbody
export class Rigidbody {
  constructor(pos, scale, density = 1.0) {
    this.pos = pos;
    this.rot = 0.0;
    this.linVel = new Vec2();
    this.angVel = 0.0;
    //Vector from center to top right corner
    this.scale = scale;
    //Scale is half, get the whole size
    this.gravity = 20;
    this.linDamp = 0.995;
    this.angDamp = 0.99;
    this.mass = 0;
    this.inertia = 0;
    this.resetMass(density);
  }
  integrateVelocity(dt) {
    if(this.mass != 0.0)
      this.linVel.y += dt*this.gravity;
  }
  integratePosition(dt) {
    //Integrate and apply some fake damping
    if(this.mass != 0.0) {
      this.pos = this.pos.add(this.linVel.mul(dt));
      this.linVel = this.linVel.mul(this.linDamp);
    }
    if(this.inertia != 0.0) {
      this.rot += this.angVel*dt;
      this.angVel *= this.angDamp;
    }
  }
  resetMass(density) {
    let width = this.scale.x*2.0;
    let height = this.scale.y*2.0;
    this.mass = width*height*density;
    this.inertia = this.mass*(width*width + height*height)/12.0;
    this.mass = safeDivide(1.0, this.mass);
    this.inertia = safeDivide(1.0, this.inertia);
  }
  //Inefficient, would be better to store right vector so you can cross to get up
  getUp(scaled) {
    let result = new Vec2(-Math.sin(this.rot), Math.cos(this.rot));
    return scaled ? result.mul(this.scale.y) : result;
  }
  getRight(scaled) {
    let result = new Vec2(Math.cos(this.rot), Math.sin(this.rot));
    return scaled ? result.mul(this.scale.x) : result;
  }
  modelToWorld(model) {
    return model.mulVec(this.scale).rotate(this.rot).add(this.pos); 
  }
  worldToModel(world) {
    return world.sub(this.pos).rotate(-this.rot).mulVec(this.scale.recip());
  }
  isMobile() {
    return this.mass != 0 || this.inertia != 0;
  }
}