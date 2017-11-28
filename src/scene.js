import { Narrowphase } from './narrowphase';
import { ContactConstraint, FrictionConstraint, DistanceConstraint } from './constraints';
import { Vec2 } from './vec2';
import { Rigidbody } from './rigidbody';
import { ContactCache } from './contactCache'

class Line {
  constructor(start, end, color) {
    this.start = start;
    this.end = end;
    this.color = color;
  }
}

export class Scene {
  constructor(canvas) {
    this.canvas = canvas;
    this.context = canvas.getContext('2d');
    this.width = canvas.width;
    this.height = canvas.height;
    this.objects = [];
    this.constraints = [];
    this.narrowphase = new Narrowphase();
    this.lines = [];
    this.curLineColor = 'white';
    this.scale = 5.0;
    this.mousePos = new Vec2();
    this.mouseObject = new Rigidbody(new Vec2(0, 0), new Vec2(0, 0), 0);
    this.mouseConstraint = null;
    this.accumulatedTime = 0.0;
    this.lastTime = performance.now();
    this.contactCache = new ContactCache(this.constraints, 0.02);
    this.constraintIterations = 20;

    this.setupInputEvents();
    this.queueUpdate();
  }

  clear() {
    this.objects = [];
    this.constraints = [];
    this.contactCache.clear();
    this.contactCache.setConstraintContainer(this.constraints);
  }

  setContactSlop(slop) {
    this.contactCache.slop = slop;
  }

  setContactMatchThreshold(threshold) {
    this.contactCache.matchThreshold = threshold;
  }

  setConstraintIterations(iterations) {
    this.constraintIterations = iterations;
  }

  setupInputEvents() {
    let mouseMove = (e)=> {
      let rect = this.canvas.getBoundingClientRect();
      this.mousePos = new Vec2(e.clientX - rect.left, e.clientY - rect.top);
      this.mousePos = this.mousePos.mul(1.0/this.scale);
    };
    this.canvas.addEventListener('mousemove', mouseMove, false);
    this.canvas.addEventListener('touchmove', (e)=>{ mouseMove(this.touchToMouse(e)); }, false);

    let mouseDown = (e)=> {
      if(!this.mouseConstraint) {
        let obj = this.pick(this.mousePos);
        if(obj) {
          this.createMouseConstraint(obj, this.mousePos);
        }
      }
      else {
        this.removeMouseConstraint();
      }
    }
    this.canvas.addEventListener('mousedown', mouseDown, false);

    this.canvas.addEventListener('touchstart', (e)=>{
      let te = this.touchToMouse(e);
      //Hack to set the mouse position as down doesn't update it
      mouseMove(te);
      mouseDown(te);
    }, false);

    this.canvas.addEventListener('touchend', (e)=>{
      this.removeMouseConstraint();
    }, false);
  }

  touchToMouse(touchEvent) {
    let t = touchEvent.changedTouches[0];
    //Only these properties are used by my function
    if(t) {
      return {
        'clientX': t.clientX,
        'clientY': t.clientY
      };
    }
    //Not sure how this would happen, but undefined would make objects disappear
    return {
      'clientX': 0,
      'clientY': 0
    };
  }

  update(dt) {
    this.mouseObject.pos = this.mousePos;
    if(this.mouseConstraint) {
      let o = this.mouseConstraint.bodyA;
      //Damp a lot for mouse object since the position corrections 
      o.linVel = o.linVel.mul(0.95);
      o.angVel *= 0.95;
    }

    this.integrateVelocity(dt);
    this.doNarrowphase();
    this.solveConstraints();
    this.integratePosition(dt);
    this.draw();
  }

  createMouseConstraint(obj, point) {
      this.mouseConstraint = new DistanceConstraint(obj, this.mouseObject, obj.worldToModel(point), new Vec2(0, 0), 0.5);
      this.bias = 0.3;
      this.mouseConstraint.upperBound = 15.0;
      this.mouseConstraint.lowerBound = -15.0;
      this.constraints.push(this.mouseConstraint);
  }

  removeMouseConstraint() {
    if(this.mouseConstraint)
      this.mouseConstraint.shouldRemove = true;
    this.mouseConstraint = null;
  }

  queueUpdate() {
    //30fps
    const intervalMS = 33.3;
    const intervalS = intervalMS/1000.0;

    window.requestAnimationFrame((timestamp)=>{
      this.accumulatedTime += timestamp - this.lastTime;

      const maxBacklog = 10;
      let runFrames = 0;
      while(this.accumulatedTime > intervalMS) {
        this.update(intervalS);
        this.accumulatedTime -= intervalMS;
        runFrames += 1;
        if(runFrames > maxBacklog)
          this.accumulatedTime = 0;
      }
      this.lastTime = performance.now();
      this.queueUpdate();
    });
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
    for(let i = 0; i < this.objects.length; ++i) {
      for(let j = i + 1; j < this.objects.length; ++j) {
        let m = this.narrowphase.getManifold(this.objects[i], this.objects[j]);
        if(m)
          this.contactCache.addManifold(m, i, j);
        else
          this.contactCache.removeManifold(i, j);
      }
    }
  }

  pick(point) {
    for(let i = 0; i < this.objects.length; ++i) {
      let obj = this.objects[i];
      let model = obj.worldToModel(point);
      if(Math.abs(model.x) < 1.0 && Math.abs(model.y) < 1.0) {
        return obj;
      }
    }
  }

  solveConstraints() {
    //Precompute per frame values
    for(let i = 0; i < this.constraints.length; ++i)
      this.constraints[i].setup(this);

    //Solve until iteration cap or global solution
    let globalError = 0.0;
    for(let i = 0; i < this.constraintIterations; ++i) {
      globalError = 0.0;
      for(let j = 0; j < this.constraints.length; ++j)
        globalError += this.constraints[j].solve();
      if(globalError < 0.00001)
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

    this.context.fillStyle = 'red';
    for(let i = 0; i < this.objects.length; ++i) {
      let o = this.objects[i];
      this.context.translate(o.pos.x*this.scale, o.pos.y*this.scale);
      this.context.rotate(o.rot);
      this.context.fillRect(-o.scale.x*this.scale, -o.scale.y*this.scale, o.scale.x*2.0*this.scale, o.scale.y*2.0*this.scale);
      this.context.setTransform(1.0, 0.0, 0.0, 1.0, 0.0, 0.0);
    }
    //Draw debug lines last so they always show up
    for(let i = 0; i < this.lines.length; ++i) {
      let line = this.lines[i];
      this.context.strokeStyle = line.color;
      this.context.beginPath();
      this.context.moveTo(line.start.x*this.scale, line.start.y*this.scale);
      this.context.lineTo(line.end.x*this.scale, line.end.y*this.scale);
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