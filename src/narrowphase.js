import { safeDivide, Vec2 } from './vec2';

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

export class Narrowphase {
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
  clip(start, end, againstNormal, againstPoint) {
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
      return [this.getIntersect(start, end, startDot, endDot, pDot), end];
    }
    //Start inside end outside
    else if(endDot > pDot)
      return [start, this.getIntersect(start, end, startDot, endDot, pDot)];
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
      line = this.clip(line[0], line[1], normals[i], pointsOnNormal[i]);
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
    if(!bodyA.isMobile() && !bodyB.isMobile())
      return null;
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