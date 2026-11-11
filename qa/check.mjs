import assert from 'node:assert/strict';
import fs from 'node:fs';
import {forward,jacobian,solve} from '../dist/kinematics.js';
import {PRESETS,JOINTS} from '../dist/model.js';
const r=JSON.parse(fs.readFileSync(new URL('./reference.json',import.meta.url)));
let worst=0;
for(const s of r.samples){const p=forward(s.q).p,J=jacobian(s.q);p.forEach((x,i)=>worst=Math.max(worst,Math.abs(x-s.p[i])));J.forEach((row,i)=>row.forEach((x,j)=>worst=Math.max(worst,Math.abs(x-s.jacobian[i][j]))));}
assert(worst<1e-12);
const result=solve(r.target);
assert.equal(result.status,'converged');assert.equal(result.iterations,r.iterations);
assert(Math.abs(result.error-r.error)<1e-12);
result.q.forEach((v,i)=>{assert(Math.abs(v-r.solution[i])<1e-10);assert(v>=JOINTS[i].lower&&v<=JOINTS[i].upper);});
assert.equal(solve(PRESETS.higher).status,'converged');
assert.notEqual(solve(PRESETS.far).status,'converged');
console.log('PASS: browser FK, Jacobian and DLS agree with Pinocchio; presets and bounds checked. Max model difference:',worst);
