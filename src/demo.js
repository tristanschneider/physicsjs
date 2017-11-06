import { safeDivide, Vec2 } from './vec2';
import { Rigidbody } from './rigidbody';
import { DistanceConstraint } from './constraints';
import { Scene } from './scene';

let c1 = document.getElementById('canvas1');
var demo = new Scene(c1);
demo.scale = 10;
(function initScene(scene) {
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

  scene.objects.push(new Rigidbody(new Vec2(20, 40), new Vec2(20, 0.5), 0.0));
  scene.objects.push(new Rigidbody(new Vec2(0, 20), new Vec2(0.5, 20), 0.0));
  scene.objects.push(new Rigidbody(new Vec2(40, 20), new Vec2(0.5, 20), 0.0));
  scene.objects.push(new Rigidbody(new Vec2(20, 0), new Vec2(20, 0.5), 0.0));
  for(let i = 0; i < 10; ++i) {
    let o = new Rigidbody(new Vec2(10, 15), s);
    o.angVel = 3.141*2;
    o.linVel = new Vec2(15.0, -15.0);
    scene.objects.push(o);
  }
})(demo);

let c2 = document.getElementById('canvas2');
demo = new Scene(c2);
demo.scale = 10;
(function init(scene) {
  let s = new Vec2(1.5, 1.0);
  scene.objects.push(new Rigidbody(new Vec2(20, 40), new Vec2(20, 0.5), 0.0));
  scene.objects.push(new Rigidbody(new Vec2(0, 20), new Vec2(0.5, 20), 0.0));
  scene.objects.push(new Rigidbody(new Vec2(40, 20), new Vec2(0.5, 20), 0.0));
  scene.objects.push(new Rigidbody(new Vec2(20, 0), new Vec2(20, 0.5), 0.0));
  for(let i = 0; i < 20; ++i) {
    let o = new Rigidbody(new Vec2(10 + i*0.5, 39 - i*2), s);
    //o.angVel = 3.141*2;
    //o.linVel = new Vec2(15.0, -15.0);
    scene.objects.push(o);
  }
})(demo);