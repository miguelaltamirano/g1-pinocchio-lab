import * as THREE from './vendor/three.module.min.js';
import {OrbitControls} from './vendor/OrbitControls.js';
import {createRobot} from './robot-model.js';
import {forward} from './kinematics.js';

export async function createViewer(canvas) {
  const renderer = new THREE.WebGLRenderer({canvas, antialias: true, powerPreference: 'low-power'});
  renderer.setPixelRatio(Math.min(devicePixelRatio || 1, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.shadowMap.autoUpdate = false;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.3;
  const scene = new THREE.Scene();
  scene.background = new THREE.Color('#252c34');
  scene.fog = new THREE.Fog('#252c34', 4, 12);
  const camera = new THREE.PerspectiveCamera(38, 1, .01, 40);
  camera.up.set(0, 0, 1);
  const controls = new OrbitControls(camera, canvas);
  controls.minDistance = .35;
  controls.maxDistance = 6;
  controls.maxPolarAngle = Math.PI * .49;
  controls.enableDamping = false; // Render only when the scene or camera changes.
  controls.screenSpacePanning = true;
  // Coalesce rapid slider/IK changes into one GPU draw per browser frame.
  let scheduled = false;
  const render = () => {
    if (scheduled) return;
    scheduled = true;
    requestAnimationFrame(() => { scheduled = false; renderer.render(scene, camera); });
  };
  controls.addEventListener('change', render);
  const ambient = new THREE.HemisphereLight('#d7e7ff', '#62676a', 2.6);
  ambient.position.set(0, 0, 4);
  scene.add(ambient);
  const key = new THREE.DirectionalLight('#fff2de', 3.3);
  key.position.set(2, -2, 4);
  key.castShadow = true;
  key.shadow.mapSize.set(1024, 1024);
  Object.assign(key.shadow.camera, {left: -1.5, right: 1.5, top: 1.5, bottom: -1.5, near: .1, far: 10});
  key.shadow.bias = -.0003;
  key.shadow.normalBias = .002;
  scene.add(key);
  const rim = new THREE.DirectionalLight('#d5eaff', 2);
  rim.position.set(-1, 2, 2);
  scene.add(rim);

  const floor = -.847;
  const pixels = new Uint8Array([106,114,120,255, 122,130,136,255, 122,130,136,255, 106,114,120,255]);
  const texture = new THREE.DataTexture(pixels, 2, 2, THREE.RGBAFormat);
  texture.wrapS = texture.wrapT = THREE.RepeatWrapping;
  texture.magFilter = THREE.NearestFilter;
  texture.repeat.set(40, 40);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.needsUpdate = true;
  const ground = new THREE.Mesh(new THREE.PlaneGeometry(16, 16), new THREE.MeshStandardMaterial({map: texture, roughness: 1}));
  ground.position.z = floor;
  ground.receiveShadow = true;
  scene.add(ground);
  const grid = new THREE.GridHelper(4, 20, '#a1acb5', '#818c94');
  grid.rotation.x = Math.PI / 2;
  grid.position.z = floor + .0005;
  grid.material.transparent = true;
  grid.material.opacity = .22;
  scene.add(grid);

  const marker = new THREE.Group();
  marker.add(new THREE.Mesh(new THREE.SphereGeometry(.018, 16, 12), new THREE.MeshBasicMaterial({color: '#f39b58', wireframe: true, transparent: true, opacity: .8})));
  for (let i = 0; i < 3; i++) {
    const a = [0, 0, 0], b = [0, 0, 0]; a[i] = -.035; b[i] = .035;
    marker.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(...a), new THREE.Vector3(...b)]), new THREE.LineBasicMaterial({color: '#ffc07c'})));
  }
  scene.add(marker);
  marker.traverse(object => {
    if (object.material) { object.material.depthTest = false; object.material.depthWrite = false; object.renderOrder = 5; }
  });
  const tip = new THREE.Mesh(new THREE.SphereGeometry(.009, 16, 12), new THREE.MeshBasicMaterial({color: '#92ffd0'}));
  scene.add(tip);
  tip.material.depthTest = false; tip.material.depthWrite = false; tip.renderOrder = 6;
  const residualGeometry = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
  const residual = new THREE.Line(residualGeometry, new THREE.LineDashedMaterial({color: '#f2a366', dashSize: .012, gapSize: .009, transparent: true, opacity: .8}));
  scene.add(residual);
  const pathGeometry = new THREE.BufferGeometry();
  const pathPositions = new Float32Array(402 * 3);
  pathGeometry.setAttribute('position', new THREE.BufferAttribute(pathPositions, 3));
  pathGeometry.setDrawRange(0, 0);
  const path = new THREE.Line(pathGeometry, new THREE.LineBasicMaterial({color: '#7bebc0', transparent: true, opacity: .7}));
  path.frustumCulled = false;
  scene.add(path);
  const frames = new THREE.Group();
  frames.visible = false;
  frames.add(new THREE.AxesHelper(.12));
  scene.add(frames);
  const axes = Array.from({length: 7}, () => new THREE.AxesHelper(.065));
  let robot = null, focused = false, lastIteration = -1, count = 0, lastPoint = null;
  function resetCamera(focus = false) {
    focused = focus;
    controls.target.set(...(focus ? [.15, .19, .10] : [0, 0, -.20]));
    camera.position.copy(controls.target).add(new THREE.Vector3(...(focus ? [.82, .72, .32] : [1.6, 1.45, .70])));
    controls.update(); render();
  }
  function resize() {
    const {width, height} = canvas.getBoundingClientRect();
    if (!width || !height) return;
    renderer.setSize(width, height, false);
    camera.aspect = width / height;
    camera.updateProjectionMatrix(); render();
  }
  const observer = new ResizeObserver(resize);
  observer.observe(canvas);
  resetCamera(); resize();
  let contextLost = false;
  canvas.addEventListener('webglcontextlost', e => {
    e.preventDefault(); contextLost = true;
    const loading = document.querySelector('#viewer-loading');
    loading.hidden = false; loading.textContent = 'The 3D view was interrupted. Reload the page to restore it.';
  });
  const responses = await Promise.all(['robot.json', 'meshes.bin'].map(name => fetch(new URL(`./assets/g1/${name}`, import.meta.url))));
  if (responses.some(r => !r.ok)) throw new Error('Robot assets could not be loaded');
  const [model, buffer] = await Promise.all([responses[0].json(), responses[1].arrayBuffer()]);
  robot = createRobot(model, buffer);
  scene.add(robot.root);
  const armNames = ['shoulder_pitch','shoulder_roll','shoulder_yaw','elbow','wrist_roll','wrist_pitch','wrist_yaw'];
  axes.forEach((axis, i) => { robot.links[`left_${armNames[i]}_link`].add(axis); axis.visible = false; });
  const toolAxes = new THREE.AxesHelper(.10);
  robot.tool.add(toolAxes); toolAxes.visible = false;
  const box = new THREE.Box3().setFromObject(robot.root);
  ground.position.z = box.min.z - .001;
  grid.position.z = ground.position.z + .0005;
  renderer.shadowMap.needsUpdate = true;

  function update(q, target, status, iteration) {
    robot.setPose(q);
    const point = forward(q).p;
    tip.position.fromArray(point);
    marker.position.fromArray(target);
    const p = residualGeometry.attributes.position;
    p.setXYZ(0, ...point); p.setXYZ(1, ...target); p.needsUpdate = true;
    residualGeometry.computeBoundingSphere(); residual.computeLineDistances();
    if (iteration < lastIteration || iteration === 0) { count = 0; lastPoint = null; }
    if ((!lastPoint || point.some((x, i) => Math.abs(x - lastPoint[i]) > 1e-10)) && count < 402) {
      pathPositions.set(point, count++ * 3); lastPoint = point;
      pathGeometry.attributes.position.needsUpdate = true; pathGeometry.setDrawRange(0, count);
    }
    lastIteration = iteration;
    renderer.shadowMap.needsUpdate = true;
    if (!contextLost) render();
  }
  canvas.addEventListener('keydown', e => {
    if (['ArrowLeft', 'ArrowRight'].includes(e.key)) {
      e.preventDefault();
      const offset = camera.position.clone().sub(controls.target);
      offset.applyAxisAngle(new THREE.Vector3(0, 0, 1), e.key === 'ArrowLeft' ? .15 : -.15);
      camera.position.copy(controls.target).add(offset); controls.update(); render();
    } else if (['+', '=', '-'].includes(e.key)) { e.preventDefault(); zoom(e.key === '-' ? 1.15 : .87); }
  });
  function zoom(factor) {
    const delta = camera.position.clone().sub(controls.target);
    delta.setLength(Math.max(controls.minDistance, Math.min(controls.maxDistance, delta.length() * factor)));
    camera.position.copy(controls.target).add(delta); controls.update(); render();
  }
  return {update, zoom, focus: () => {resetCamera(!focused); return focused;},
    reset: () => resetCamera(false),
    setFrames: enabled => {frames.visible = enabled; axes.forEach(a => a.visible = enabled); toolAxes.visible = enabled; render();},
    setPath: enabled => {path.visible = enabled; render();}};
}
