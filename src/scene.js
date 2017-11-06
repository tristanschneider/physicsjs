import { Narrowphase } from './narrowphase';
import { ContactConstraint, FrictionConstraint } from './constraints';

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

    this.queueUpdate();
  }

  update(dt) {
    this.integrateVelocity(dt);
    this.doNarrowphase();
    this.solveConstraints();
    this.integratePosition(dt);
    this.draw();
  }

  queueUpdate() {
    //30fps
    const intervalMS = 33.3;
    const intervalS = intervalMS/1000.0;
    setInterval(()=>{
      this.update(intervalS);
    }, intervalMS);
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
        if(m) {
          for(let c = 0; c < m.contacts.length; ++c) {
            let cc = new ContactConstraint(m.objA, m.objB, m.contacts[c], m.normal, m.penetrations[c]);
            this.constraints.push(cc);
            this.constraints.push(new FrictionConstraint(m.objA, m.objB, m.contacts[c], m.normal, cc));
          }
        }
      }
    }
  }

  solveConstraints() {
    //Precompute per frame values
    for(let i = 0; i < this.constraints.length; ++i)
      this.constraints[i].setup(this);

    //Solve until iteration cap or global solution
    let globalError = 0.0;
    for(let i = 0; i < 20; ++i) {
      globalError = 0.0;
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