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
    this.mass = this.scale.x*this.scale.y*density;
    this.inertia = this.mass*(this.scale.x*this.scale.x + this.scale.y*this.scale.y)/12.0;
    this.mass = safeDivide(1.0, this.mass);
    this.inertia = safeDivide(1.0, this.inertia);
  }
  integrateVelocity(dt) {
    if(this.mass != 0.0)
      this.linVel.y += dt*9.8;
  }
  integratePosition(dt) {
    //Integrate and apply some fake damping
    if(this.mass != 0.0) {
      this.pos = this.pos.add(this.linVel.mul(dt));
      this.linVel = this.linVel.mul(0.999);
    }
    if(this.inertia != 0.0) {
      this.rot += this.angVel*dt;
      this.angVel *= 0.999;
    }
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
}