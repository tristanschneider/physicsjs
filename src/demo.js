import { safeDivide, Vec2 } from './vec2';
import { Rigidbody } from './rigidbody';
import { DistanceConstraint } from './constraints';
import { Scene } from './scene';

function addWalls(scene, max, thickness) {
  let halfMax = max.mul(0.5);
  scene.objects.push(new Rigidbody(new Vec2(halfMax.x, max.y), new Vec2(halfMax.x, thickness), 0.0));
  scene.objects.push(new Rigidbody(new Vec2(0, halfMax.y), new Vec2(thickness, halfMax.y), 0.0));
  scene.objects.push(new Rigidbody(new Vec2(max.x, halfMax.y), new Vec2(thickness, halfMax.y), 0.0));
  scene.objects.push(new Rigidbody(new Vec2(halfMax.x, 0), new Vec2(halfMax.x, thickness), 0.0));
}

function addScene(canvasName, initFunc, resetName) {
  let canvas = document.getElementById(canvasName);
  if(!canvas)
    return;
  let scene = new Scene(canvas);
  let resetButton = document.getElementById(resetName ? resetName : canvasName + 'Reset');
  if(resetButton) {
    resetButton.addEventListener('click', ()=>{
      scene.clear();
      initFunc(scene);
    });
  }
  initFunc(scene);
}

function createChain(scene, pos, dist, set, chain) {
  let s = new Vec2(1, 1);
  let last = new Rigidbody(pos, s);
  last.mass = 0;
  scene.objects.push(last);
  for(let i = 0; i < chain; ++i) {
    let cur = new Rigidbody(new Vec2(last.pos.x + dist + s.x*2, last.pos.y), s);
    let constraint = new DistanceConstraint(last, cur, new Vec2(1, 0), new Vec2(-1, 0), dist);
    if(set)
      set(constraint);
    scene.objects.push(cur);
    scene.constraints.push(constraint);
    last = cur;
  }
}

//Build a scene with two distance constraints configured by the two callbacks
function distCompare(scene, setA, setB, chain) {
  scene.scale = 10;
  createChain(scene, new Vec2(10, 20), 2.5, setA, chain);
  createChain(scene, new Vec2(26, 20), 2.5, setB, chain);
}

function addStack(scene, origin, objects, space) {
  for(let i = 0; i < objects; ++i)
    scene.objects.push(new Rigidbody(origin.add(new Vec2(0, -(2 + space)*i)), new Vec2(1, 1)));
}

function sceneStackCompare(aName, bName, resetName, initFunc) {
  let init = (scene)=>{
    scene.scale = 10;
    addWalls(scene, new Vec2(20, 40), 0.5);
    addStack(scene, new Vec2(10, 38), 5, 0);
  };
  addScene(aName, (scene)=>{
    init(scene);
    initFunc(scene, true);
  }, resetName);
  addScene(bName, (scene)=>{
    init(scene);
    initFunc(scene, false);
  }, resetName);
}

addScene('constraints', (scene)=>{
  scene.scale = 10;
  let s = new Vec2(1.0, 1.0);
  let b = new Rigidbody(new Vec2(19, 4), s);
  let dist = 2.0;
  b.mass = 0.0;
  scene.objects.push(b);
  let last = b;
  let modelAnchor = new Vec2(1, 1);
  for(let i = 0; i < 5; ++i) {
    let offset = (i + 1)*(s.x + dist);
    let rb = new Rigidbody(new Vec2(b.pos.x + offset, b.pos.y + offset), s);
    if(last) {
      scene.constraints.push(new DistanceConstraint(last, rb, modelAnchor, modelAnchor.neg(), dist));
    }
    scene.objects.push(rb);
    last = rb;
  }

  addWalls(scene, new Vec2(40, 40), 0.5);
  for(let i = 0; i < 10; ++i) {
    let o = new Rigidbody(new Vec2(10, 15), s);
    o.angVel = 3.141*2;
    o.linVel = new Vec2(15.0, -15.0);
    scene.objects.push(o);
  }
});

addScene('noBias', (scene)=>{
  distCompare(scene, (ab)=>{
    ab.baumgarteTerm = 0.0;
  },
  null, 1);
});

addScene('biasRange', (scene)=>{
  distCompare(scene, (ab)=>{
    //10% per second
    ab.baumgarteTerm = 0.1;
  },
  (ab)=>{
    //100% per frame
    ab.baumgarteTerm = 30;
  },
  3);
});

sceneStackCompare('slop', 'noSlop', 'slopReset', (scene, first)=>{
  if(!first)
    scene.setContactSlop(0);
});

sceneStackCompare('warm', 'cold', 'warmReset', (scene, first)=>{
  if(!first)
    scene.setContactMatchThreshold(0);
  scene.setConstraintIterations(10);
});

addScene('direction', (scene)=>{
  distCompare(scene, (ab)=>{
    ab.upperBound = 0;
    ab.distance = 6;
  },
  (ab)=>{
    ab.lowerBound = 0;
    ab.distance = 6;
  },
  1);
  for(let i = 0; i < scene.objects.length; ++i)
    scene.objects[i].resetMass(1);
  addWalls(scene, new Vec2(40, 40), 0.5);
});

addScene('capped', (scene)=>{
  scene.scale = 10;
  let s = new Vec2(1, 1);
  let a = new Rigidbody(new Vec2(20, 20), s);
  a.mass = 0;
  let b = new Rigidbody(new Vec2(25, 20), s);
  let dist = new DistanceConstraint(a, b, new Vec2(1, 0), new Vec2(-1, 0), 6);
  let bound = 5;
  dist.upperBound = bound;
  dist.lowerBound = -bound;
  scene.constraints.push(dist);
  scene.objects.push(a, b);
});

class StickyDistanceConstraint extends DistanceConstraint {
  constructor(bodyA, bodyB, modelAnchorA, modelAnchorB, distance) {
    super(bodyA, bodyB, modelAnchorA, modelAnchorB, distance);
  }

  setup(drawer) {
    let dist = this.bodyA.modelToWorld(this.anchorA).sub(this.bodyB.modelToWorld(this.anchorB)).normalize();
    this.shouldEnforce = dist > this.distance;
    super.setup(drawer);
  }
}

addScene('sticky', (scene)=>{
  scene.scale = 10;
  let s = new Vec2(1, 1);
  let a = new Rigidbody(new Vec2(20, 20), s);
  let b = new Rigidbody(new Vec2(25, 20), s);
  scene.constraints.push(new StickyDistanceConstraint(a, b, new Vec2(1, 0), new Vec2(-1, 0), 6));
  scene.objects.push(a, b);
  addWalls(scene, new Vec2(40, 40), 0.5);
});
