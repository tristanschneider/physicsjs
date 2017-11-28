import { ContactConstraint, FrictionConstraint } from './constraints'

function swapRemove(container, index) {
  if(container.length)
    container[index] = container[container.length - 1];
  container.pop();
}

class ContactPair {
  constructor(normal) {
    this.normal = normal;
    //Array of constraint pairs, where 0 is contact, 1 is friction
    this.constraints = [];
  }

  pushConstraints(contact, friction) {
    this.constraints.push([contact, friction]);
  }
}

export class ContactCache {
  constructor(constraints, slop) {
    //Container to push new constraints in to
    this.constraints = constraints;
    //Map of manifodl keys to contact pairs
    this.contactPairs = {};
    //Slop to give contact constraints
    this.slop = slop;
    //If points are within this distance they are matches as the same contact
    this.matchThreshold = 0.05;
  }

  setConstraintContainer(constraints) {
    this.constraints = constraints;
  }

  clear() {
    this.contactPairs = {};
  }

  //Add the constraints associated with the given manifold, but attempt to match it with the existing constraints for warm starting
  addManifold(m, idA, idB) {
    let key = this.getManifoldKey(idA, idB);
    let pair = this.contactPairs[key];
    //If a pair already exists we need to try to match the contacts to preserve warm starts
    if(pair) {
      this.mergeManifoldWithPair(m, pair);
    }
    else {
      // Store a b and normal on the pair as it's more convenient than extracting from the constraints
      let newPair = new ContactPair(m.normal);
      //No existing pair, add new contacts
      this.populatePairFromManifold(newPair, m);
      this.contactPairs[key] = newPair;
    }
  }

  removeManifold(idA, idB) {
    let key = this.getManifoldKey(idA, idB)
    let pair = this.contactPairs[key];
    if(pair) {
      for(let i = 0; i < pair.constraints.length; ++i)
        pair.constraints[i][0].shouldRemove = pair.constraints[i][1].shouldRemove = true;
      pair.constraints = [];
      this.contactPairs[key] = null;
    }
  }

  getManifoldKey(idA, idB) {
    return idA.toString() + ',' + idB.toString();
  }

  mergeManifoldWithPair(m, pair) {
    let normalDiff = pair.normal.dot(m.normal);
    //Ensure normals are pointing the same way
    if(normalDiff < 0) {
      normalDiff = -normalDiff;
      m.normal = m.normal.neg();
    }

    //If normal is too far off, normal cannot be re-used
    let attemptMatch = normalDiff > 0.95;

    //Mark all for removal, so if they aren't matched we'll remove them
    for(let i = 0; i < pair.constraints.length; ++i)
      pair.constraints[i][0].shouldRemove = true;

    if(attemptMatch) {
      for(let i = 0; i < m.contacts.length;) {
        let newPoint = m.contacts[i];
        let matchFound = false;
        for(let j = 0; j < pair.constraints.length; ++j) {
          let constraint = pair.constraints[j][0];
          //More precise would be to store points in model space, transorm them here to world then compare distance, but this will do
          if(newPoint.dist2(constraint.contact) < this.matchThreshold) {
            constraint.shouldRemove = false;
            constraint.update(newPoint, m.normal, m.penetrations[i]);
            swapRemove(m.contacts, i);
            swapRemove(m.penetrations, i);
            matchFound = true;
            break;
          }
        }

        //If a match was found it was swap removed, otherwise increment to next one
        if(!matchFound)
          ++i;
      }
    }

    //Remove unmatched constraints left on pair
    for(let i = 0; i < pair.constraints.length;) {
      let cc = pair.constraints[i][0];
      let fc = pair.constraints[i][1];
      //Make sure both are marked for deletion so they are removed from this.cosntraints next time solving rolls around
      if(cc.shouldRemove) {
        fc.shouldRemove = true;
        swapRemove(pair.constraints, i);
      }
      else
        ++i
    }

    //Add any remaining points from manifold that weren't matched
    this.populatePairFromManifold(pair, m);
  }

  populatePairFromManifoldIndex(pair, m, index) {
    let cc = new ContactConstraint(m.objA, m.objB, m.contacts[index], m.normal, m.penetrations[index]);
    cc.slop = this.slop;
    let fc = new FrictionConstraint(m.objA, m.objB, m.contacts[index], m.normal, cc);
    this.constraints.push(cc, fc);
    pair.pushConstraints(cc, fc);
  }

  populatePairFromManifold(pair, m) {
    for(let c = 0; c < m.contacts.length; ++c)
      this.populatePairFromManifoldIndex(pair, m, c);
  }
}