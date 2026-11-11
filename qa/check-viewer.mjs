import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import * as THREE from '../dist/vendor/three.module.min.js';
import {createRobot} from '../dist/robot-model.js';
import {JOINTS, START, PRESETS} from '../dist/model.js';
import {forward, solve} from '../dist/kinematics.js';

const model = JSON.parse(readFileSync(new URL('../dist/assets/g1/robot.json', import.meta.url)));
const bytes = readFileSync(new URL('../dist/assets/g1/meshes.bin', import.meta.url));
const buffer = bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength);
const robot = createRobot(model, buffer);
let seed = 4189, maxError = 0;
const poses = [START, ...Object.values(PRESETS).map(p => solve(p).q)];
for (let i = 0; i < 50; i++) poses.push(JOINTS.map(j => {
  seed = (1664525 * seed + 1013904223) >>> 0;
  return j.lower + (j.upper - j.lower) * seed / 2 ** 32;
}));
for (const q of poses) {
  robot.setPose(q);
  const tip = robot.tool.getWorldPosition(new THREE.Vector3()).toArray();
  const expected = forward(q);
  maxError = Math.max(maxError, ...tip.map((v, i) => Math.abs(v - expected.p[i])));
  assert.ok(maxError < 1e-12, `Mesh FK differs from IK: ${maxError}`);
  const torso = robot.links.torso_link.getWorldPosition(new THREE.Vector3());
  assert.ok(torso.length() < 1e-12);
  // Every visual joint axis must agree, not just the final hand position.
  JOINTS.forEach((j, i) => {
    const joint = robot.joints[j.name];
    const p = joint.pivot.getWorldPosition(new THREE.Vector3()).toArray();
    assert.ok(p.every((x, k) => Math.abs(x - expected.points[i][k]) < 1e-12));
    const axis = joint.axis.clone().applyQuaternion(joint.pivot.getWorldQuaternion(new THREE.Quaternion())).toArray();
    assert.ok(axis.every((x, k) => Math.abs(x - expected.axes[i][k]) < 1e-12));
  });
}
assert.equal(Object.keys(model.meshes).length, 36);
for (const m of Object.values(model.meshes)) {
  const vertices = new Float32Array(buffer, m.vertexOffset, m.vertexCount * 3);
  const indices = new Uint32Array(buffer, m.indexOffset, m.indexCount);
  assert.ok(vertices.every(Number.isFinite));
  assert.ok(indices.every(i => i < m.vertexCount));
}
console.log(`PASS: 36 meshes; ${poses.length} poses; mesh tool, joint positions and axes match IK (max error ${maxError} m).`);
