import { safeDivide, Vec2 } from './vec2';
import { Rigidbody } from './rigidbody';
import { DistanceConstraint } from './constraints';
import { Scene } from './scene';

let c1 = document.getElementById('canvas1');
var demo = new Scene(c1);
(function initScene(scene) {
  let s = new Vec2(5.0, 5.0);
  let b = new Rigidbody(new Vec2(190.0, 40.0), s);
  let dist = 10.0;
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

  scene.objects.push(new Rigidbody(new Vec2(200, 200), new Vec2(200, 5), 0.0));
  let o = new Rigidbody(new Vec2(100, 150), s);
  o.angVel = 1.0;
  o.linVel = new Vec2(10.0, -10.0);
  scene.objects.push(o);
})(demo);
