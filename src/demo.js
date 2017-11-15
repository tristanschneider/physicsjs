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

function addScene(canvasName, initFunc) {
  let canvas = document.getElementById(canvasName);
  let scene = new Scene(canvas);
  let resetButton = document.getElementById(canvasName + 'Reset');
  if(resetButton) {
    resetButton.onclick = ()=>{
      scene.objects = [];
      scene.constraints = [];
      initFunc(scene);
    };
  }
  initFunc(scene);
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

// Build a scene with two distance constraints configured by the two callbacks
function distCompare(scene, setA, setB, chain) {
  scene.scale = 10;
  createChain(scene, new Vec2(10, 20), 2.5, setA, chain);
  createChain(scene, new Vec2(26, 20), 2.5, setB, chain);
}

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