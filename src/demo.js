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

addScene('nobias', (scene)=>{
  scene.scale = 10;
  addWalls(scene, new Vec2(40, 40), 0.5);
  let s = new Vec2(1, 1);
  let a = new Rigidbody(new Vec2(15, 20), s, 0);
  let b = new Rigidbody(new Vec2(20, 20), s);
  let distAB = new DistanceConstraint(a, b, new Vec2(1, 0), new Vec2(-1, 0), 3);
  distAB.baumgarteTerm = 0.0;
  let c = new Rigidbody(new Vec2(30, 20), s, 0);
  let d = new Rigidbody(new Vec2(35, 20), s);
  let distCD = new DistanceConstraint(c, d, new Vec2(1, 0), new Vec2(-1, 0), 3);
  scene.objects.push(a, b, c, d);
  scene.constraints.push(distAB, distCD);
});