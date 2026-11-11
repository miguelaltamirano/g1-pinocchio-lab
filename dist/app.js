import {JOINTS,START,PRESETS} from './model.js';
import {forward,step,sub,norm} from './kinematics.js';
import {LESSONS} from './lessons.js';
const $=s=>document.querySelector(s);
const esc=s=>s.replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
function highlight(s){const pattern=/(#[^\n]*)|("(?:[^"\\]|\\.)*"|'(?:[^'\\]|\\.)*')|\b(from|import|as|for|in|if|else|return|def|while|break|continue|True|False|None|raise|with)\b|\b(\d+(?:\.\d+)?(?:e-?\d+)?)\b/g;let out='',last=0;for(const match of s.matchAll(pattern)){out+=esc(s.slice(last,match.index));const type=match[1]?'comment':match[2]?'string':match[3]?'keyword':'number';out+=`<span class="tok-${type}">${esc(match[0])}</span>`;last=match.index+match[0].length;}return out+esc(s.slice(last));}
let toastTimer;
function toast(text){$('#toast').textContent=text;$('#toast').classList.add('show');clearTimeout(toastTimer);toastTimer=setTimeout(()=>$('#toast').classList.remove('show'),2400);}
async function copy(text){try{await navigator.clipboard.writeText(text);toast('Copied to clipboard');}catch{const t=document.createElement('textarea');t.value=text;t.style.position='fixed';t.style.opacity='0';document.body.append(t);t.select();const ok=document.execCommand('copy');t.remove();toast(ok?'Copied to clipboard':'Select the code and copy it manually');}}
let q=[...START],target=[...PRESETS.forward],iterations=0,status='ready',running=false,damping=.03,lastTick=0;
const labels=['Shoulder pitch','Shoulder roll','Shoulder yaw','Elbow','Wrist roll','Wrist pitch','Wrist yaw'];
$('#joint-angles').innerHTML=JOINTS.map((j,i)=>`<div class="joint-item"><div><span>${labels[i]}</span><output id="angle-${i}">0.000</output></div><div class="joint-track"><div class="joint-fill" id="joint-${i}"></div></div></div>`).join('');
function updateStatus(){const value=norm(sub(target,forward(q).p))*1000;$('#error-value').textContent=value<.001?'< 0.001':value.toFixed(value<10?3:1);$('#iteration-value').textContent=iterations;const states={ready:'Ready',running:'Solving…',converged:'Target reached',stalled:'No improvement',iteration_limit:'Iteration limit'};const s=$('#solver-status');s.textContent=states[status];s.className='status '+status;$('#solve-btn').innerHTML=running?'Pause <span aria-hidden="true">Ⅱ</span>':'Solve reach <span aria-hidden="true">→</span>';$('#step-btn').disabled=running;$('#solver-note').textContent=status==='converged'?'Position error is below 0.1 mm. Try another target.':status==='stalled'||status==='iteration_limit'?'This local search stopped. Inspect the residual; a returned pose is not always a solution.':status==='running'?'Recompute the tool position and Jacobian after each accepted update.':'Start from the same arm posture and watch each update.';q.forEach((v,i)=>{$(`#angle-${i}`).textContent=v.toFixed(3);$(`#joint-${i}`).style.width=`${100*(v-JOINTS[i].lower)/(JOINTS[i].upper-JOINTS[i].lower)}%`;});}
function resetSolver(){running=false;q=[...START];iterations=0;status='ready';updateStatus();draw();}
function syncSliders(){['x','y','z'].forEach((a,i)=>{$(`#target-${a}`).value=target[i];$(`#${a}-output`).textContent=target[i].toFixed(3);});}
for(const b of document.querySelectorAll('[data-preset]'))b.addEventListener('click',()=>{document.querySelectorAll('[data-preset]').forEach(x=>x.classList.toggle('selected',x===b));target=[...PRESETS[b.dataset.preset]];syncSliders();resetSolver();});
['x','y','z'].forEach((axis,i)=>{$(`#target-${axis}`).addEventListener('input',e=>{target[i]=Number(e.target.value);$(`#${axis}-output`).textContent=target[i].toFixed(3);document.querySelectorAll('[data-preset]').forEach(b=>b.classList.remove('selected'));resetSolver();});});
$('#damping').addEventListener('change',e=>{damping=Number(e.target.value);resetSolver();});
function singleStep(){if(iterations>=400){status='iteration_limit';running=false;updateStatus();return;}const r=step(q,target,damping);q=r.q;if(r.moved)iterations++;status=r.status;if(status==='running'&&iterations>=400)status='iteration_limit';if(status!=='running')running=false;updateStatus();draw();}
$('#solve-btn').addEventListener('click',()=>{if(running){running=false;status='ready';updateStatus();return;}if(status==='stalled'||status==='iteration_limit'||status==='converged')resetSolver();running=true;status='running';lastTick=0;updateStatus();requestAnimationFrame(animate);});
function animate(t){if(!running)return;if(t-lastTick>65){singleStep();lastTick=t;}if(running)requestAnimationFrame(animate);}
$('#step-btn').addEventListener('click',()=>{singleStep();if(status==='running'){status='ready';updateStatus();}});
$('#reset-btn').addEventListener('click',()=>{target=[...PRESETS.forward];damping=.03;$('#damping').value='.03';// assign the exact option value below
$('#damping').value='0.03';document.querySelectorAll('[data-preset]').forEach(b=>b.classList.toggle('selected',b.dataset.preset==='forward'));syncSliders();resetSolver();});
// Load the mesh renderer separately so code lessons and IK remain usable if WebGL fails.
let viewer = null;
function draw(){viewer?.update(q,target,status,iterations);}
const viewerButtons = ['focus-btn','camera-reset','zoom-in','zoom-out','show-frames','show-path'];
import('./viewer.js').then(m=>m.createViewer($('#robot-canvas'))).then(v=>{
  viewer=v; draw(); $('#viewer-loading').hidden=true;
  $('#robot-canvas').dataset.ready='true';
  viewerButtons.forEach(id=>$('#'+id).disabled=false);
}).catch(error=>{
  console.error('3D viewer:',error);
  $('#viewer-loading').textContent='The 3D view could not load. Enable WebGL and reload, or download the MuJoCo lab below. The IK controls and code guide still work.';
  $('#robot-canvas').dataset.ready='error';
});
$('#focus-btn').addEventListener('click',()=>{const focused=viewer.focus();$('#focus-btn').setAttribute('aria-pressed',String(focused));$('#focus-btn').textContent=focused?'Show body ↙':'Focus arm ↗';});
$('#camera-reset').addEventListener('click',()=>{viewer.reset();$('#focus-btn').setAttribute('aria-pressed','false');$('#focus-btn').textContent='Focus arm ↗';});
$('#zoom-in').addEventListener('click',()=>viewer.zoom(.85));
$('#zoom-out').addEventListener('click',()=>viewer.zoom(1.18));
$('#show-frames').addEventListener('change',e=>viewer.setFrames(e.target.checked));
$('#show-path').addEventListener('change',e=>viewer.setPath(e.target.checked));
// Implementation guide.
let lessonIndex=0;
$('#lesson-nav').innerHTML=LESSONS.map((s,i)=>`<button data-lesson="${i}" ${i===0?'aria-current="step"':''}><span class="step-number">${String(i+1).padStart(2,'0')}</span><span><span class="step-title">${s.title}</span><span class="step-description">${s.label}</span></span></button>`).join('');
function setLesson(i){lessonIndex=i;const s=LESSONS[i];document.querySelectorAll('[data-lesson]').forEach((b,n)=>n===i?b.setAttribute('aria-current','step'):b.removeAttribute('aria-current'));$('#lesson-counter').textContent=`STEP ${String(i+1).padStart(2,'0')} / 06`;$('#function-name').textContent=s.function;$('#lesson-title').textContent=s.heading;$('#lesson-description').textContent=s.description;$('#lesson-code').innerHTML=highlight(s.code);$('#lesson-notes').innerHTML=s.notes.map(([h,p])=>`<div><h4>${esc(h)}</h4><p>${esc(p)}</p></div>`).join('');$('#lesson-check').textContent=s.check;$('#prev-step').disabled=i===0;$('#next-step').textContent=i===5?'Run it locally ↗':'Next step →';}
document.querySelectorAll('[data-lesson]').forEach(b=>b.addEventListener('click',()=>setLesson(Number(b.dataset.lesson))));
$('#prev-step').addEventListener('click',()=>{if(lessonIndex>0)setLesson(lessonIndex-1);});
$('#next-step').addEventListener('click',()=>{if(lessonIndex<5)setLesson(lessonIndex+1);else $('#setup').scrollIntoView({behavior:'smooth'});});
$('#copy-step').addEventListener('click',()=>copy(LESSONS[lessonIndex].code));
const setupCommands='conda env create -f environment.yml\nconda activate g1-left-arm\npython left_arm_ik.py';
$('#copy-setup').addEventListener('click',()=>copy(setupCommands));
for(const b of document.querySelectorAll('[data-os]'))b.addEventListener('click',()=>{document.querySelectorAll('[data-os]').forEach(x=>x.setAttribute('aria-selected',String(x===b)));$('#os-note').textContent=b.dataset.os==='windows'?'Open Miniforge Prompt in the extracted g1-left-arm folder.':b.dataset.os==='mac'?'Open a Conda-enabled Terminal in the extracted folder. This script computes IK; it does not open the MuJoCo viewer.':'Open a Conda-enabled terminal in the extracted g1-left-arm folder.';});
let fullCode='';
$('#open-code').addEventListener('click',async()=>{try{if(!fullCode){const r=await fetch('downloads/left_arm_ik.py');if(!r.ok)throw Error('File unavailable');fullCode=await r.text();}$('#full-code').innerHTML=highlight(fullCode);$('#code-dialog').showModal();}catch{toast('Could not load the code. Download the example ZIP instead.');}});
$('#close-code').addEventListener('click',()=>$('#code-dialog').close());$('#copy-full').addEventListener('click',()=>copy(fullCode));$('#code-dialog').addEventListener('click',e=>{if(e.target===$('#code-dialog')){const r=e.target.getBoundingClientRect();if(e.clientX<r.left||e.clientX>r.right||e.clientY<r.top||e.clientY>r.bottom)e.target.close();}});
setLesson(0);syncSliders();$('#damping').value=String(damping);updateStatus();draw();
