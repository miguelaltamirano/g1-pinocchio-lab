import {JOINTS, START} from './model.js';
export const add=(a,b)=>a.map((v,i)=>v+b[i]);
export const sub=(a,b)=>a.map((v,i)=>v-b[i]);
export const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
export const norm=a=>Math.sqrt(dot(a,a));
export const scale=(a,s)=>a.map(v=>v*s);
export const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const mv=(A,v)=>A.map(r=>dot(r,v));
const mm=(A,B)=>A.map(r=>B[0].map((_,j)=>r.reduce((s,v,k)=>s+v*B[k][j],0)));
const identity=()=>[[1,0,0],[0,1,0],[0,0,1]];
function rotation(a,t){const c=Math.cos(t),s=Math.sin(t),C=1-c,[x,y,z]=a;return [[c+x*x*C,x*y*C-z*s,x*z*C+y*s],[y*x*C+z*s,c+y*y*C,y*z*C-x*s],[z*x*C-y*s,z*y*C+x*s,c+z*z*C]];}
export function forward(q){let R=identity(),p=[0,0,0];const points=[],axes=[];for(let i=0;i<7;i++){const j=JOINTS[i];p=add(p,mv(R,j.xyz));R=mm(R,j.rotation);points.push([...p]);axes.push(mv(R,j.axis));R=mm(R,rotation(j.axis,q[i]));}p=add(p,mv(R,[.08,0,0]));return {p,R,points:[...points,p],axes};}
export function jacobian(q){const f=forward(q);const columns=f.axes.map((a,i)=>cross(a,sub(f.p,f.points[i])));return [0,1,2].map(r=>columns.map(c=>c[r]));}
function linearSolve(A,b){const M=A.map((r,i)=>[...r,b[i]]);for(let i=0;i<3;i++){let pivot=i;for(let j=i+1;j<3;j++)if(Math.abs(M[j][i])>Math.abs(M[pivot][i]))pivot=j;[M[i],M[pivot]]=[M[pivot],M[i]];const d=M[i][i];if(Math.abs(d)<1e-16)throw new Error('Singular system');for(let k=i;k<4;k++)M[i][k]/=d;for(let j=0;j<3;j++){if(j===i)continue;const f=M[j][i];for(let k=i;k<4;k++)M[j][k]-=f*M[i][k];}}return M.map(r=>r[3]);}
export function step(q,target,damping=.03){const e=sub(target,forward(q).p),error=norm(e);if(error<1e-4)return {q:[...q],error,status:'converged',moved:false};const J=jacobian(q),A=J.map((r,i)=>J.map((s,j)=>dot(r,s)+(i===j?damping*damping:0)));const x=linearSolve(A,e);let dq=q.map((_,j)=>J.reduce((s,r,k)=>s+r[j]*x[k],0));dq=scale(dq,Math.min(1,.12/Math.max(norm(dq),1e-15)));for(let k=0;k<15;k++){const a=2**(-k),candidate=q.map((v,i)=>Math.max(JOINTS[i].lower,Math.min(JOINTS[i].upper,v+a*dq[i])));const next=norm(sub(target,forward(candidate).p));if(next<error-1e-13)return {q:candidate,error:next,status:next<1e-4?'converged':'running',moved:true};}return {q:[...q],error,status:'stalled',moved:false};}
export function solve(target,damping=.03,maxIterations=400){let q=[...START],result,iterations=0;while(iterations<maxIterations){result=step(q,target,damping);q=result.q;if(result.moved)iterations++;if(result.status!=='running')return {...result,iterations};}return {...result,status:'iteration_limit',iterations};}
