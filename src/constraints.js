import { safeDivide, Vec2 } from './vec2';

class Jacobian {
  constructor(linearA, angularA, linearB, angularB) {
    this.linA = linearA;
    this.angA = angularA;
    this.linB = linearB;
    this.angB = angularB;
  }
}

class Constraint {
  constructor(bodyA, bodyB) {
    this.bodyA = bodyA;
    this.bodyB = bodyB;
    this.lowerBound = -Number.MAX_VALUE;
    this.upperBound = Number.MAX_VALUE;
    this.lambdaSum = 0.0;
    this.j = new Jacobian();
    this.jm = new Jacobian();
    this.shouldRemove = false;
    this.shouldEnforce = true;
    this.bias = 0.0;
    this.constraintMass = 0.0;
    this.slop = 0.05;
    this.baumgarteTerm = 0.3*30;
  }

  setBias(error) {
    if(error >= this.slop)
      this.bias = (error - this.slop)*this.baumgarteTerm;
    else if(error <= -this.slop)
      this.bias = (error + this.slop)*this.baumgarteTerm;
    else
      this.bias = 0.0;
  }

  //Derived classes must set the jacobian, and can set whatever else, then must call super.setup
  setup(drawer) {
    this.shouldEnforce = this.shouldEnforce && !this.shouldRemove;

    this.jm = new Jacobian(this.j.linA.mul(this.bodyA.mass),
      this.j.angA*this.bodyA.inertia,
      this.j.linB.mul(this.bodyB.mass),
      this.j.angB*this.bodyB.inertia);
    this.constraintMass = safeDivide(1.0, this.j.linA.dot(this.jm.linA) +
      this.j.angA*this.jm.angA +
      this.j.linB.dot(this.jm.linB) +
      this.j.angB*this.jm.angB);

    //Warm start
    if(this.shouldEnforce && this.lambdaSum != 0.0) {
      this.applyImpulse(this.lambdaSum);
    }
  }

  setAnchoredJacobian(anchorA, anchorB, axis) {
    this.j.linA = axis;
    this.j.angA = anchorA.sub(this.bodyA.pos).cross(this.j.linA);
    this.j.linB = axis.neg();
    this.j.angB = anchorB.sub(this.bodyB.pos).cross(this.j.linB);
  }

  solve() {
    if(!this.shouldEnforce)
      return 0.0;

    let jv = this.j.linA.dot(this.bodyA.linVel) +
      this.j.angA*this.bodyA.angVel +
      this.j.linB.dot(this.bodyB.linVel) +
      this.j.angB*this.bodyB.angVel;

    let jvb = jv - this.bias;
    let lambda = -jvb*this.constraintMass;

    let oldSum = this.lambdaSum;
    this.lambdaSum += lambda;
    this.lambdaSum = Math.min(this.upperBound, Math.max(this.lowerBound, this.lambdaSum));
    lambda = this.lambdaSum - oldSum;
    this.applyImpulse(lambda);
    return Math.abs(safeDivide(lambda, this.constraintMass));
  }

  applyImpulse(lambda) {
    this.bodyA.linVel = this.bodyA.linVel.add(this.jm.linA.mul(lambda));
    this.bodyA.angVel += this.jm.angA*lambda;
    this.bodyB.linVel = this.bodyB.linVel.add(this.jm.linB.mul(lambda));
    this.bodyB.angVel += this.jm.angB*lambda;
  }
}

export class DistanceConstraint extends Constraint {
  constructor(bodyA, bodyB, modelAnchorA, modelAnchorB, distance) {
    super(bodyA, bodyB);
    this.anchorA = modelAnchorA;
    this.anchorB = modelAnchorB;
    this.distance = distance;
  }

  setup(drawer) {
    let worldA = this.bodyA.modelToWorld(this.anchorA);
    let worldB = this.bodyB.modelToWorld(this.anchorB);
    let axis = worldA.sub(worldB);
    let dist = axis.normalize();
    this.setAnchoredJacobian(worldA, worldB, axis);

    if(drawer) {
      drawer.setLineColor('white');
      drawer.drawLine(worldA, worldB);
    }

    this.setBias(this.distance - dist);
    super.setup(drawer);
  }
}

export class ContactConstraint extends Constraint {
  constructor(bodyA, bodyB, contact, normal, penetration) {
    super(bodyA, bodyB);
    this.contact = contact;
    this.normal = normal;
    this.penetration = penetration;
    this.lowerBound = 0.0;
  }

  update(contact, normal, penetration) {
    this.contact = contact;
    this.normal = normal;
    this.peneteration = penetration;
  }

  setup(drawer) {
    //Normal is from reference to incident, we want to push apart, so flip
    this.setAnchoredJacobian(this.contact, this.contact, this.normal.neg());
    this.setBias(this.penetration);


    super.setup(drawer);
  }
}

export class FrictionConstraint extends Constraint {
  constructor(bodyA, bodyB, contact, normal, contactConstraint) {
    super(bodyA, bodyB);
    this.contact = contact;
    this.normal = normal;
    this.cc = contactConstraint;
    //Set each frame to normal force
    this.upperBound = this.lowerBound = 0.0;
    this.frictionTerm = 0.99;
  }

  update(normal) {
    this.normal = normal;
  }

  setup(drawer) {
    this.setAnchoredJacobian(this.contact, this.contact, new Vec2(-this.normal.y, this.normal.x));
    super.setup();
  }

  solve() {
    this.upperBound = this.cc.lambdaSum*this.frictionTerm;
    this.lowerBound = -this.upperBound;
    return super.solve();
  }
}