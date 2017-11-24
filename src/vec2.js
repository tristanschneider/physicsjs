export function safeDivide(num, denom) {
  return Math.abs(denom) < 0.00001 ? 0.0 : num/denom;
}

export class Vec2 {
  constructor(x = 0.0, y = 0.0) {
    this.x = x;
    this.y = y;
  }

  add(rhs) {
    return new Vec2(this.x + rhs.x, this.y + rhs.y);
  }

  sub(rhs) {
    return new Vec2(this.x - rhs.x, this.y - rhs.y);
  }

  dot(rhs) {
    return this.x*rhs.x + this.y*rhs.y;
  }

  cross(rhs) {
    return this.x*rhs.y - this.y*rhs.x;
  }

  mul(scalar) {
    return new Vec2(this.x*scalar, this.y*scalar);
  }

  mulVec(v) {
    return new Vec2(this.x*v.x, this.y*v.y);
  }

  neg() {
    return new Vec2(-this.x, -this.y);
  }

  recip() {
    return new Vec2(safeDivide(1.0, this.x), safeDivide(1.0, this.y));
  }

  projScalar(onto) {
    return safeDivide(this.dot(onto), onto.dot(onto));
  }

  proj(onto) {
    return onto.mul(this.projScalar(onto));
  }

  rotate(rad) {
    let cosAngle = Math.cos(rad);
    let sinAngle = Math.sin(rad);
    return new Vec2(this.x*cosAngle - this.y*sinAngle,
      this.x*sinAngle + this.y*cosAngle);
  }

  normalize() {
    let len = Math.sqrt(this.dot(this));
    this.x = safeDivide(this.x, len);
    this.y = safeDivide(this.y, len);
    return len;
  }

  dist2(other) {
    let to = other.sub(this);
    return to.dot(to);
  }
}