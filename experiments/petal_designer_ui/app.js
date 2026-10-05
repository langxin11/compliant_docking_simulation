import {InterfaceViewer} from './renderer.js';
import {PreviewState,decodePacket} from './state.js';
const state=new PreviewState();
function syncSave(){$('save-design').disabled=!state.canSave;}
const $=id=>document.getElementById(id),NS='http://www.w3.org/2000/svg';
let info,values={},viewer,metadata,controller,timer;
const pose={opening_mm:14,x_mm:0,y_mm:0,yaw_deg:0,tilt_deg:0};
const poseSpec={opening_mm:[0,30,.25,'落座位置上方距离','mm'],x_mm:[-8,8,.25,'横向X偏差','mm'],y_mm:[-8,8,.25,'横向Y偏差','mm'],yaw_deg:[-20,20,.25,'绕轴偏差','°'],tilt_deg:[-8,8,.25,'倾斜偏差','°']};
function showError(message){$('error').textContent=message;$('error').hidden=false;}
function clearError(){$('error').hidden=true;}
async function response(url,options){const res=await fetch(url,options);if(!res.ok){const data=await res.json();const error=new Error(data.error||'请求失败，请重试。');error.status=res.status;throw error;}return res;}
function field(key,spec,parent,data,onChange,prefix){
  const [min,max,step,label,unit]=spec,wrap=document.createElement('div');wrap.className='field';
  const heading=document.createElement('div');heading.className='field-heading';const lab=document.createElement('label');lab.htmlFor=`${prefix}-${key}`;lab.textContent=label;
  const numeric=document.createElement('span');numeric.className='numeric';const number=document.createElement('input');number.type='number';number.id=`${prefix}-${key}-number`;number.min=min;number.max=max;number.step='any';number.value=data[key];number.setAttribute('aria-label',`${label}数值`);numeric.append(number,document.createTextNode(unit));heading.append(lab,numeric);
  const range=document.createElement('input');range.type='range';range.id=`${prefix}-${key}`;range.min=min;range.max=max;range.step=step;range.value=data[key];range.setAttribute('aria-label',label);wrap.append(heading,range);parent.append(wrap);
  range.addEventListener('input',()=>{number.value=range.value;data[key]=Number(range.value);onChange(key);});
  number.addEventListener('change',()=>{if(number.value===''||!Number.isFinite(Number(number.value))||Number(number.value)<min||Number(number.value)>max){showError(`${label}需在 ${min}–${max}${unit} 之间。`);number.value=data[key];return;}data[key]=Number(number.value);range.value=data[key];onChange(key);});
}
function syncFields(){for(const key of Object.keys(values)){for(const suffix of ['', '-number'])$(`param-${key}${suffix}`).value=values[key];}}
function selectedPreset(){const exact=name=>Object.keys(info.parameters).every(k=>values[k]===info.presets[name][k]);return Object.keys(info.presets).find(exact)||'custom';}
function changed(key){state.edit();syncSave();if(controller)controller.abort();clearError();syncFields();$('preset').value=selectedPreset();$('save-design').disabled=true;$('status').textContent='正在更新轮廓…';clearTimeout(timer);timer=setTimeout(update,140);}
async function update(attempt=0){
  const id=state.revision,snapshot={...values};if(controller)controller.abort();controller=new AbortController();$('save-design').disabled=true;
  try{const res=await response('/api/preview',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({parameters:snapshot}),signal:controller.signal});const packet=await res.arrayBuffer();if(!state.current(id))return;const decoded=decodePacket(packet);if(Object.keys(decoded.metadata.parameters).length!==Object.keys(snapshot).length||!Object.keys(snapshot).every(k=>decoded.metadata.parameters[k]===snapshot[k]))throw new Error('预览参数与请求不一致。');metadata=decoded.metadata;viewer.meta=metadata;viewer.setMesh('guide',decoded.vertices);viewer.draw();if(state.displayFailed)throw new Error('三维显示连接中断，请刷新页面。');state.accept(id,snapshot);
    $('separation').textContent=metadata.nominal_separation_mm.toFixed(2);$('guide-clearance').textContent=values.guide_axial_clearance_mm.toFixed(2);$('status').textContent=selectedPreset()==='narrow'?'当前窄平顶参数 · 几何预览':selectedPreset()==='selected'?'用户候选参数 · 几何预览':'自定义参数 · 待验证';
    $('warnings').replaceChildren();for(const text of metadata.warnings){const p=document.createElement('p');p.textContent=text;$('warnings').append(p);}drawPlots();syncSave();clearError();
  }catch(error){if(state.current(id)&&error.name!=='AbortError'){if(error.status===503&&attempt<2){timer=setTimeout(()=>{if(state.current(id))update(attempt+1);},250);return;}state.fail(id);syncSave();showError(error.message);$('status').textContent='参数未更新';}}
}
function svgNode(tag,attributes={},text){const el=document.createElementNS(NS,tag);for(const [k,v]of Object.entries(attributes))el.setAttribute(k,v);if(text!==undefined)el.textContent=text;return el;}
function plot(svg,x,series,xLabel){
  const w=svg.clientWidth;if(!w)return;const h=195,pad={l:42,r:14,t:8,b:39};svg.setAttribute('viewBox',`0 0 ${w} ${h}`);svg.replaceChildren();const xmin=Math.min(...x),xmax=Math.max(...x),ymax=Math.ceil(Math.max(...series.flatMap(s=>s.y),1)/5)*5;
  const px=v=>pad.l+(v-xmin)/(xmax-xmin)*(w-pad.l-pad.r),py=v=>h-pad.b-v/ymax*(h-pad.t-pad.b);
  for(let i=0;i<=3;i++){const y=ymax*i/3;svg.append(svgNode('line',{x1:pad.l,y1:py(y),x2:w-pad.r,y2:py(y),stroke:'#dbe3e8','stroke-width':1}));svg.append(svgNode('text',{x:pad.l-6,y:py(y)+4,'text-anchor':'end'},Number(y.toFixed(1)).toString()));}
  const nt=w<330?3:4;for(let i=0;i<=nt;i++){const xval=xmin+(xmax-xmin)*i/nt;svg.append(svgNode('text',{x:px(xval),y:h-pad.b+17,'text-anchor':i===0?'start':i===nt?'end':'middle'},Number(xval.toFixed(1)).toString()));}
  for(const s of series){const d=x.map((v,i)=>`${i?'L':'M'}${px(v).toFixed(2)},${py(s.y[i]).toFixed(2)}`).join(' ');svg.append(svgNode('path',{d,fill:'none',stroke:s.color,'stroke-width':2,...(s.dash?{'stroke-dasharray':'5 4'}:{})}));}
  svg.append(svgNode('text',{x:(pad.l+w-pad.r)/2,y:h-3,'text-anchor':'middle'},xLabel));svg.append(svgNode('text',{x:12,y:(pad.t+h-pad.b)/2,transform:`rotate(-90,12,${(pad.t+h-pad.b)/2})`,'text-anchor':'middle'},'高度 / mm'));
}
function drawPlots(){if(!metadata)return;const c=metadata.curve;plot($('angular-plot'),c.angles,[{y:c.baseline,color:'#85949e'},{y:c.heights,color:'#ad772e'}],'花瓣中心角 / °');}
async function start(){
  try{viewer=new InterfaceViewer($('preview'));viewer.pose=pose;$('preview').addEventListener('preview-error',e=>{state.displayFailed=true;syncSave();showError(e.detail);});
    info=await(await response('/api/info')).json();Object.assign(values,info.default);$('preset').replaceChildren();for(const key of Object.keys(info.presets)){const option=document.createElement('option');option.value=key;option.textContent=info.preset_labels[key];$('preset').append(option);}const custom=document.createElement('option');custom.value='custom';custom.textContent='自定义参数';custom.disabled=true;$('preset').append(custom);$('preset').value=selectedPreset();for(const [key,spec]of Object.entries(info.parameters))field(key,spec,$('geometry-controls'),values,changed,'param');for(const [key,spec]of Object.entries(poseSpec))field(key,spec,$('pose-fields'),pose,()=>viewer.draw(),'pose');syncFields();
    await Promise.all(['adapter','head_plate','stop_land'].map(async name=>{const bytes=await(await response(`/api/base/${name}`)).arrayBuffer();viewer.setMesh(name,new Float32Array(bytes));}));await update();
    $('preset').addEventListener('change',()=>{Object.assign(values,info.presets[$('preset').value]);syncFields();changed();});
    for(const button of document.querySelectorAll('[data-mode]'))button.addEventListener('click',()=>{viewer.mode=button.dataset.mode;for(const b of document.querySelectorAll('[data-mode]'))b.setAttribute('aria-pressed',b===button?'true':'false');$('pose-controls').open=viewer.mode!=='single';viewer.draw();});
    $('camera').addEventListener('change',()=>viewer.camera($('camera').value));$('axes').addEventListener('change',()=>{viewer.axes=$('axes').checked;viewer.draw();});
    $('save-design').addEventListener('click',async()=>{const snapshot=state.snapshot(pose);if(!snapshot)return;const id=state.revision;state.savePending=true;syncSave();try{const saved=await(await response('/api/save',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(snapshot)})).json();$('saved').hidden=false;$('saved').replaceChildren(document.createTextNode('已保存待验证候选：'));const link=document.createElement('a');link.href=saved.url;link.textContent=saved.name;link.download=saved.name;$('saved').append(link);if(state.current(id))clearError();}catch(error){if(state.current(id))showError(error.message);}finally{state.savePending=false;syncSave();}});
    $('load-file').addEventListener('click',()=>$('file-input').click());$('file-input').addEventListener('change',async()=>{const file=$('file-input').files[0];if(!file)return;$('file-input').value='';const id=state.edit();syncSave();clearTimeout(timer);if(controller)controller.abort();try{if(file.size>65536)throw new Error('参数文件过大，请选择本工具保存的候选。');const text=await file.text();if(!state.current(id))return;const data=JSON.parse(text);const input=data.design_parameters||data.parameters||data;if(!input||typeof input!=='object'||Array.isArray(input))throw new Error('参数文件格式不正确。');const next={};for(const key of ['radial_lead_width_mm','radial_crest_drop_mm']){if(input[key]!==undefined)next[key]=input[key];}for(const key of Object.keys(info.parameters)){if(input[key]===undefined)throw new Error(`参数文件缺少 ${key}。`);next[key]=input[key];}const res=await response('/api/preview',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({parameters:next})});decodePacket(await res.arrayBuffer());if(!state.current(id))return;for(const key of Object.keys(info.parameters))values[key]=next[key];syncFields();$('preset').value=selectedPreset();changed();}catch(error){if(state.current(id)){state.fail(id);syncSave();showError(error.message);}}});
    new ResizeObserver(drawPlots).observe($('angular-plot'));
  }catch(error){showError(error.message);$('status').textContent='载入失败';}
}
start();
