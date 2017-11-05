function safeDivide(num, denom) {
  return Math.abs(denom) < 0.00001 ? 0.0 : num/denom;
}
class Vec2 {
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
}

//Always cube shaped Rigidbody
class Rigidbody {
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

class Line {
  constructor(start, end, color) {
    this.start = start;
    this.end = end;
    this.color = color;
  }
}

class Demo {
  constructor(width, height) {
    this.width = width;
    this.height = height;
    this.canvas = document.querySelector('canvas');
    this.context = this.canvas.getContext('2d');
    this.objects = [];
    this.constraints = [];
    this.narrowphase = new Narrowphase();
    this.lines = [];
    this.curLineColor = 'white';
    
    this.initScene();
    this.queueUpdate();
  }

  initScene() {
    let s = new Vec2(5.0, 5.0);
    let b = new Rigidbody(new Vec2(190.0, 40.0), s);
    let dist = 10.0;
    b.mass = 0.0;
    this.objects.push(b);
    let last = b;
    let modelAnchor = new Vec2(1, 1);
    for(let i = 0; i < 5; ++i) {
      let offset = (i + 1)*(s.x + dist);
      let rb = new Rigidbody(new Vec2(b.pos.x + offset, b.pos.y + offset), s);
      if(last) {
        this.constraints.push(new DistanceConstraint(last, rb, modelAnchor, modelAnchor.neg(), dist));
      }
      this.objects.push(rb);
      last = rb;
    }

    this.objects.push(new Rigidbody(new Vec2(200, 200), new Vec2(200, 5), 0.0));
    let o = new Rigidbody(new Vec2(100, 150), s);
    o.angVel = 1.0;
    o.linVel = new Vec2(10.0, -10.0);
    this.objects.push(o);
  }
  
  update(dt) {
    this.draw();
    this.integrateVelocity(dt);
    this.doNarrowphase();
    this.solveConstraints();
    this.integratePosition(dt);
  }

  queueUpdate() {
    //30fps
    const intervalMS = 33.3;
    const intervalS = intervalMS/1000.0;
    //this.last = performance.now();
    setInterval(()=>{
      //let now = performance.now();
      //console.log(now - this.last);
      //this.last = now;
      this.update(intervalS);
    }, intervalMS*0);
  }

  integrateVelocity(dt) {
    for(let i = 0; i < this.objects.length; ++i)
      this.objects[i].integrateVelocity(dt);
  }

  integratePosition(dt) {
    for(let i = 0; i < this.objects.length; ++i)
      this.objects[i].integratePosition(dt);
  }

  doNarrowphase() {
    for(let i = 0; i < this.objects.length; ++i)
      for(let j = i + 1; j < this.objects.length; ++j) {
        let m = this.narrowphase.getManifold(this.objects[i], this.objects[j]);
        if(m) {
          for(let c = 0; c < m.contacts.length; ++c) {
            let cc = new ContactConstraint(m.objA, m.objB, m.contacts[c], m.normal, m.penetrations[c]);
            this.constraints.push(cc);
            this.constraints.push(new FrictionConstraint(m.objA, m.objB, m.contacts[c], m.normal, cc));
          }
        }
      }
  }

  solveConstraints() {
    //Precompute per frame values
    for(let i = 0; i < this.constraints.length; ++i)
      this.constraints[i].setup();

    //Solve until iteration cap or global solution
    for(let i = 0; i < 10; ++i) {
      let globalError = 0.0;
      for(let j = 0; j < this.constraints.length; ++j)
        globalError += this.constraints[j].solve();
      if(globalError < 0.001)
        break;
    }

    //Swap remove expired constraints
    for(let i = 0; i < this.constraints.length;) {
      if(this.constraints[i].shouldRemove) {
        this.constraints[i] = this.constraints[this.constraints.length - 1];
        this.constraints.pop();
      }
      else
        ++i;
    }
  }

  draw() {
    //Draw background
    this.context.fillStyle = 'black';
    this.context.fillRect(0, 0, this.width, this.height);

    this.context.fillStyle = "red";
    for(let i = 0; i < this.objects.length; ++i) {
      let o = this.objects[i];
      this.context.translate(o.pos.x, o.pos.y);
      this.context.rotate(o.rot);
      this.context.fillRect(-o.scale.x, -o.scale.y, o.scale.x*2.0, o.scale.y*2.0);
      this.context.setTransform(1.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    }
    //Draw debug lines last so they always show up
    for(let i = 0; i < this.lines.length; ++i) {
      let line = this.lines[i];
      this.context.strokeStyle = line.color;
      this.context.beginPath();
      this.context.moveTo(line.start.x, line.start.y);
      this.context.lineTo(line.end.x, line.end.y);
      this.context.stroke();
    }
    this.lines = [];
  }

  setLineColor(color) {
    this.curLineColor = color;
  }

  drawLine(start, end) {
    this.lines.push(new Line(start, end, this.curLineColor));
  }
}

class Manifold {
  //More thorough would be model space for both objects so it could be re-used in future frames
  constructor(contacts, penetrations, normal, objA, objB) {
    this.contacts = contacts;
    this.penetrations = penetrations;
    this.normal = normal;
    this.objA = objA;
    this.objB = objB;
  }
}

class Narrowphase {
  getLineSupport(supportDir, lineDir) {
    return supportDir.dot(lineDir) > 0.0 ? lineDir : lineDir.neg();
  }

  getBoxSupport(supportDir, body) {
    let result = new Vec2(body.pos.x, body.pos.y);
    result = result.add(this.getLineSupport(supportDir, body.getRight(true)));
    return result.add(this.getLineSupport(supportDir, body.getUp(true)));
  }

  //Get the edge of the box most in this direction
  getBoxEdge(supportDir, body) {
    let r = body.getRight(true);
    let u = body.getUp(true);
    let rd = r.dot(supportDir);
    let ud = u.dot(supportDir);
    //Find most significant axis
    if(Math.abs(rd) > Math.abs(ud)) {
      if(rd > 0.0)
        return [body.pos.add(r).add(u), body.pos.add(r).sub(u)];
      //Orthogonal to most significant axis to get edge direction
      return [body.pos.sub(r).add(u), body.pos.sub(r).sub(u)];
    }
    if(ud > 0.0)
      return [body.pos.add(u).add(r), body.pos.add(u).sub(r)];
    return [body.pos.sub(u).add(r), body.pos.sub(u).sub(r)];
  }

  getIntersect(start, end, startDot, endDot, pDot) {
    return end.sub(start).mul(safeDivide(pDot - startDot, endDot - startDot)).add(start);
  }

  //One iteration of Sutherland Hodgman clipping
  clip(start, end, againstNormal, againstPoint, pushIntersect) {
    let pDot = againstNormal.dot(againstPoint);
    let startDot = againstNormal.dot(start);
    if(end == undefined)
      return startDot > pDot ? [] : [start];
    let endDot = againstNormal.dot(end);
    //Start is in front of line
    if(startDot > pDot) {
      //Start and end are in front of line
      if(endDot > pDot)
        return null;
      //Start in front, end inside
      return pushIntersect ? [this.getIntersect(start, end, startDot, endDot, pDot), end] : [end];
    }
    //Start inside end outside
    else if(endDot > pDot)
      return pushIntersect ? [start, this.getIntersect(start, end, startDot, endDot, pDot)] : [start];
    //Both inside
    return [start, end];
  }

  //Clip line [start, end] against the other line
  clipEdgeEdge(line, againstLine, againstNormal) {
    let leftNormal = againstLine[0].sub(againstLine[1]);
    let rightNormal = leftNormal.neg();
    let normals = [leftNormal, rightNormal, againstNormal];
    let pointsOnNormal = [againstLine[0], againstLine[1], againstLine[0]];

    //Sutherland Hodgman clip against normal and two adjacent edges
    for(let i = 0; i < 3; ++i) {
      //Don't push intersection with reference edge, that's bad for stability
      line = this.clip(line[0], line[1], normals[i], pointsOnNormal[i], i != 1);
      if(!line)
        return null;
    }
    return line;
  }

  getAxes(bodyA, bodyB) {
    //Pos and neg can be combined in the SAT loop, but adding them here is simpler
    let ar = bodyA.getRight();
    let au = bodyA.getUp();
    let br = bodyB.getRight();
    let bu = bodyB.getUp();
    return [ar, ar.neg(), au, au.neg(), br, br.neg(), bu, bu.neg()]
  }
  
  getManifold(bodyA, bodyB) {
    let axes = this.getAxes(bodyA, bodyB);
    let bestAxis = 0;
    let leastPenetration = Number.MAX_VALUE;
    //SAT to determine if there's a collision and get separating axis
    for(let i = 0; i < axes.length; ++i) {
      let axis = axes[i];
      let supportA = this.getBoxSupport(axis, bodyA);
      let supportB = this.getBoxSupport(axis.neg(), bodyB);
      let penetration = supportA.dot(axis) - supportB.dot(axis);
      //Objects are separating, no collision
      if(penetration < 0.0)
        return null;
      else if(penetration < leastPenetration) {
        leastPenetration = penetration;
        bestAxis = i;
      }
    }

    //Reference is the owner of the best normal, incident owns the contact point
    let referenceObject, incidentObject;
    let normal = axes[bestAxis];
    if(bestAxis < axes.length/2) {
      referenceObject = bodyA;
      incidentObject = bodyB;
    }
    else {
      referenceObject = bodyB;
      incidentObject = bodyA;
      normal = normal.neg();
    }

    let incidentEdge = this.getBoxEdge(normal.neg(), incidentObject);
    let referenceEdge = this.getBoxEdge(normal, referenceObject);

    let contacts = this.clipEdgeEdge(incidentEdge, referenceEdge, normal);
    //Shouldn't happen, as SAT found objects overlapping
    if(!contacts)
      return null;

    let penetrations = [];
    let refD = referenceEdge[0].dot(normal);
    for(let i = 0; i < contacts.length; ++i) {
      penetrations.push(refD - contacts[i].dot(normal));
    }

    return new Manifold(contacts, penetrations, normal, referenceObject, incidentObject);
  }
}

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
  }

  setBias(error) {
    const slop = 0.01;
    const baumgarteTerm = 0.3;
    if(error >= slop)
      this.bias = (error - slop)*baumgarteTerm;
    else if(error <= -slop)
      this.bias = (error + slop)*baumgarteTerm;
    else
      this.bias = 0.0;
  }

  //Derived classes must set the jacobian, and can set whatever else, then must call super.setup
  setup() {
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
    return Math.abs(jvb);
  }

  applyImpulse(lambda) {
    this.bodyA.linVel = this.bodyA.linVel.add(this.jm.linA.mul(lambda));
    this.bodyA.angVel += this.jm.angA*lambda;
    this.bodyB.linVel = this.bodyB.linVel.add(this.jm.linB.mul(lambda));
    this.bodyB.angVel += this.jm.angB*lambda;
  }
}

class DistanceConstraint extends Constraint {
  constructor(bodyA, bodyB, modelAnchorA, modelAnchorB, distance) {
    super(bodyA, bodyB);
    this.anchorA = modelAnchorA;
    this.anchorB = modelAnchorB;
    this.distance = distance;
  }

  setup() {
    let worldA = this.bodyA.modelToWorld(this.anchorA);
    let worldB = this.bodyB.modelToWorld(this.anchorB);
    let axis = worldA.sub(worldB);
    let dist = axis.normalize();
    this.setAnchoredJacobian(worldA, worldB, axis);

    demo.setLineColor('white');
    demo.drawLine(worldA, worldB);

    this.setBias(this.distance - dist);
    super.setup();
  }
}

class ContactConstraint extends Constraint {
  constructor(bodyA, bodyB, contact, normal, penetration) {
    super(bodyA, bodyB);
    this.contact = contact;
    this.normal = normal;
    this.penetration = penetration;
    this.lowerBound = 0.0;
  }

  setup() {
    //Normal is from reference to incident, we want to push apart, so flip
    this.setAnchoredJacobian(this.contact, this.contact, this.normal.neg());
    this.setBias(this.penetration);
    //No persistent manifolds, so throw out these constraints after solving each frame
    this.shouldRemove = true;
    super.setup();
  }
}

class FrictionConstraint extends Constraint {
  constructor(bodyA, bodyB, contact, normal, contactConstraint) {
    super(bodyA, bodyB);
    this.contact = contact;
    this.normal = normal;
    this.cc = contactConstraint;
    //Set each frame to normal force
    this.upperBound = this.lowerBound = 0.0;
  }
  
  setup() {
    this.setAnchoredJacobian(this.contact, this.contact, new Vec2(-this.normal.y, this.normal.x))
    this.shouldRemove = true;
    super.setup();
  }
  
  solve() {
    const frictionTerm = 0.8;
    this.upperBound = this.cc.lambdaSum*frictionTerm;
    this.lowerBound = -this.upperBound;
    super.solve();
  }
}

var demo = new Demo(400, 200);