import test from 'node:test';
import assert from 'node:assert/strict';
import {PreviewState,decodePacket} from '../experiments/petal_designer_ui/state.js';
import {identity,multiply,translation,scale,rotateX,rotateZ,normalMatrix,lookAt} from '../experiments/petal_designer_ui/renderer.js';
test('saving retains displayed parameters and click-time pose during edits',()=>{
  const s=new PreviewState();s.accept(0,{height:18});const pose={opening:14};const snapshot=s.snapshot(pose);s.savePending=true;
  s.edit();pose.opening=3;s.savePending=false;
  assert.equal(s.canSave,false);assert.deepEqual(snapshot,{parameters:{height:18},pose:{opening:14}});
  s.accept(s.revision,{height:20});assert.equal(s.canSave,true);
});
test('old preview and import success/error cannot overwrite the latest edit',()=>{
  const s=new PreviewState();const importId=s.edit();const editId=s.edit();s.accept(editId,{height:21});
  assert.equal(s.accept(importId,{height:18}),false);s.fail(importId);assert.equal(s.canSave,true);
  s.fail(editId);assert.equal(s.canSave,false);s.accept(editId,{height:21});s.displayFailed=true;assert.equal(s.canSave,false);
});
function packet(change={},dataChange=null){
 const meta={protocol:'petal-preview/v1',units:'mm',vertex_count:3,parameters:{height:18},base_height_mm:14,nominal_separation_mm:46.4,stop_scale:1,warnings:[],curve:{angles:[0,1],heights:[0,1],baseline:[0,1],radii:[1,2],crest:[1,2],valley:[0,1]},...change};
 let raw=new TextEncoder().encode(JSON.stringify(meta));const n=Math.ceil(raw.length/4)*4;const bytes=new ArrayBuffer(4+n+72);const view=new Uint8Array(bytes);view.fill(32,4,4+n);view.set(raw,4);new DataView(bytes).setUint32(0,n,true);if(dataChange)dataChange(new Float32Array(bytes,4+n));return bytes;
}
test('packet validation rejects truncation, bad lengths, versions, coordinates and UI metadata',()=>{
 assert.equal(decodePacket(packet()).vertices.length,18);
 for(const bytes of [new ArrayBuffer(3),packet().slice(0,-1),packet({protocol:'unknown'}),packet({vertex_count:4}),packet({curve:null}),packet({stop_scale:null}),packet({},a=>a[0]=Infinity)])assert.throws(()=>decodePacket(bytes));
 const bytes=packet();new DataView(bytes).setUint32(0,5,true);assert.throws(()=>decodePacket(bytes));
});
test('matrix composition preserves translation and expected flipped mating direction',()=>{
 const m=multiply(translation(4,5,6),scale(2,3,4));assert.deepEqual([...m].slice(12,15),[4,5,6]);
 assert.deepEqual([...multiply(identity(),m)],[...m]);const flipped=multiply(rotateZ(Math.PI/4),rotateX(Math.PI));assert.ok(Math.abs(flipped[10]+1)<1e-6);assert.ok(Math.abs(flipped[0]-Math.SQRT1_2)<1e-6);
});

test('normal transforms remain perpendicular under rotated nonuniform scale; camera stays finite near poles',()=>{
 const m=multiply(rotateZ(.7),scale(2,3,4)),n=normalMatrix(m);
 const apply=(a,v,size)=>v.map((_,i)=>v.reduce((sum,x,j)=>sum+a[j*size+i]*x,0));
 const tangent=apply(m,[1,-1,0],4),normal=apply(n,[1,1,0],3);
 assert.ok(Math.abs(tangent.reduce((sum,x,i)=>sum+x*normal[i],0))<1e-6);
 for(const z of [-155,155])assert.ok([...lookAt([.001,0,z],[0,0,0])].every(Number.isFinite));
});

test('actual app handlers keep save disabled after edits and discard stale import responses',async()=>{
 const {readFile}=await import('node:fs/promises');
 const elements=new Map();
 class Element {
   children=[];listeners={};hidden=false;disabled=false;value='';clientWidth=0;
   set id(value){this._id=value;elements.set(value,this);}get id(){return this._id;}
   append(...children){this.children.push(...children);}replaceChildren(...children){this.children=children;}
   setAttribute(){}addEventListener(name,fn){this.listeners[name]=fn;}
   fire(name){return this.listeners[name]?.({});}
 }
 globalThis.document={getElementById:id=>{if(!elements.has(id)){const e=new Element();e.id=id;}return elements.get(id);},createElement:()=>new Element(),createElementNS:()=>new Element(),createTextNode:text=>text,querySelectorAll:()=>[]};
 globalThis.ResizeObserver=class{observe(){}};
 globalThis.FakeViewer=class{setMesh(){}draw(){}camera(){}};
 const spec={height:[10,24,1,'height','mm'],guide_axial_clearance_mm:[0,1,.1,'gap','mm']};
 const values={height:18,guide_axial_clearance_mm:.4};
 const queue=[];const result=(data,isPacket=false,parameters=values)=>({ok:true,status:200,json:async()=>data,arrayBuffer:async()=>isPacket?packet({parameters}):new ArrayBuffer(0)});
 globalThis.fetch=async(url,options)=>{
  if(url==='/api/info')return result({parameters:spec,default:values,presets:{narrow:values},preset_labels:{narrow:'narrow'}});
  if(url.startsWith('/api/base'))return result(null);
  return await new Promise(resolve=>queue.push({url,options,resolve}));
 };
 let source=await readFile(new URL('../experiments/petal_designer_ui/app.js',import.meta.url),'utf8');
 source=source.replace("import {InterfaceViewer} from './renderer.js';",'const InterfaceViewer=globalThis.FakeViewer;').replace("'./state.js'",JSON.stringify(new URL('../experiments/petal_designer_ui/state.js',import.meta.url).href));
 await import('data:text/javascript;base64,'+Buffer.from(source).toString('base64'));
 const tick=()=>new Promise(resolve=>setTimeout(resolve,0));await tick();
 queue.shift().resolve(result(null,true));await tick();
 const get=id=>document.getElementById(id);assert.equal(get('save-design').disabled,false);
 const saving=get('save-design').fire('click');assert.equal(get('save-design').disabled,true);
 get('param-height').value='20';get('param-height').fire('input');
 const save=queue.shift();assert.equal(JSON.parse(save.options.body).parameters.height,18);
 save.resolve(result({url:'/candidate',name:'candidate'}));await saving;assert.equal(get('save-design').disabled,true);
 // A new import cancels the edit debounce. Its delayed response must not replace a later edit.
 get('file-input').files=[{size:30,text:async()=>JSON.stringify({design_parameters:values})}];
 const importing=get('file-input').fire('change');await tick();const imported=queue.shift();
 assert.equal(get('file-input').value,'');get('param-height').value='22';get('param-height').fire('input');
 imported.resolve(result(null,true));await importing;assert.equal(Number(get('param-height-number').value),22);assert.equal(get('save-design').disabled,true);
 // Settle the pending latest preview and leave no timer running.
 await new Promise(resolve=>setTimeout(resolve,160));const latest=queue.shift();assert.equal(JSON.parse(latest.options.body).parameters.height,22);latest.resolve({ok:false,status:503,json:async()=>({error:'busy'})});await tick();
 get('param-height').value='23';get('param-height').fire('input');
 await new Promise(resolve=>setTimeout(resolve,280));assert.equal(queue.length,1);const retry=queue.shift();assert.equal(JSON.parse(retry.options.body).parameters.height,23);retry.resolve(result(null,true,{...values,height:23}));await tick();
 assert.equal(get('save-design').disabled,false);
 assert.equal(elements.has('param-radial_lead_width_mm'),false);
 assert.equal(elements.has('param-radial_crest_drop_mm'),false);
 // Nonzero legacy radial values reach server validation, rather than being silently discarded.
 get('file-input').files=[{size:30,text:async()=>JSON.stringify({design_parameters:{...values,radial_lead_width_mm:8,radial_crest_drop_mm:2}})}];
 const oldRadial=get('file-input').fire('change');await tick();const radialRequest=queue.shift();
 assert.equal(JSON.parse(radialRequest.options.body).parameters.radial_lead_width_mm,8);
 radialRequest.resolve({ok:false,status:400,json:async()=>({error:'径向导面功能已从交互工具移除'})});await oldRadial;
 assert.match(get('error').textContent,/功能已从交互工具移除/);assert.equal(get('save-design').disabled,true);
});
