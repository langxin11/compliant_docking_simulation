// Native WebGL viewer: actual generated guide meshes, original mounting meshes.
// All preview transforms use millimetres. No simulation state is changed.
export function multiply(a,b){const c=new Float32Array(16);for(let j=0;j<4;j++)for(let i=0;i<4;i++)for(let k=0;k<4;k++)c[j*4+i]+=a[k*4+i]*b[j*4+k];return c;}
export function identity(){return new Float32Array([1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]);}
export function translation(x,y,z){const m=identity();m[12]=x;m[13]=y;m[14]=z;return m;}
export function scale(x,y,z){const m=identity();m[0]=x;m[5]=y;m[10]=z;return m;}
export function rotateX(a){const c=Math.cos(a),s=Math.sin(a);return new Float32Array([1,0,0,0,0,c,s,0,0,-s,c,0,0,0,0,1]);}
export function rotateZ(a){const c=Math.cos(a),s=Math.sin(a);return new Float32Array([c,s,0,0,-s,c,0,0,0,0,1,0,0,0,0,1]);}
export function normalMatrix(m){const a=m[0],b=m[4],c=m[8],d=m[1],e=m[5],f=m[9],g=m[2],h=m[6],i=m[10];const det=a*(e*i-f*h)-b*(d*i-f*g)+c*(d*h-e*g);return new Float32Array([(e*i-f*h)/det,(c*h-b*i)/det,(b*f-c*e)/det,(f*g-d*i)/det,(a*i-c*g)/det,(c*d-a*f)/det,(d*h-e*g)/det,(b*g-a*h)/det,(a*e-b*d)/det]);}
function perspective(aspect){const f=1/Math.tan(Math.PI/7),near=1,far=3000;return new Float32Array([f/aspect,0,0,0,0,f,0,0,0,0,(far+near)/(near-far),-1,0,0,2*far*near/(near-far),0]);}
export function lookAt(eye,center){const norm=v=>{const n=Math.hypot(...v);return v.map(x=>x/n);};const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];const z=norm(eye.map((v,i)=>v-center[i]));const x=norm(cross([0,0,1],z)),y=cross(z,x);const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);return new Float32Array([x[0],y[0],z[0],0,x[1],y[1],z[1],0,x[2],y[2],z[2],0,-dot(x,eye),-dot(y,eye),-dot(z,eye),1]);}
const RAD=Math.PI/180;
export class InterfaceViewer{
  constructor(canvas){
    this.canvas=canvas;this.gl=canvas.getContext('webgl',{antialias:true,preserveDrawingBuffer:true});
    if(!this.gl)throw new Error('当前浏览器无法显示三维模型，请使用支持WebGL的浏览器。');
    this.meshes={};this.mode='pair';this.pose={opening_mm:14,x_mm:0,y_mm:0,yaw_deg:0,tilt_deg:0};this.axes=true;this.meta=null;this.azimuth=-55*RAD;this.elevation=24*RAD;this.distance=155;
    const gl=this.gl;
    const vertex=`attribute vec3 a_position;attribute vec3 a_normal;uniform mat4 u_model;uniform mat4 u_viewProjection;uniform mat3 u_normal;varying vec3 v_normal;varying vec3 v_world;void main(){vec4 world=u_model*vec4(a_position,1.);v_world=world.xyz;v_normal=u_normal*a_normal;gl_Position=u_viewProjection*world;}`;
    const fragment=`precision mediump float;varying vec3 v_normal;varying vec3 v_world;uniform vec3 u_color;uniform float u_clip;uniform float u_unlit;void main(){if(u_clip>0.5&&v_world.x>0.)discard;vec3 n=normalize(v_normal);if(!gl_FrontFacing)n=-n;float light=.38+.45*max(0.,dot(n,normalize(vec3(.5,-.6,1.))))+.17*max(0.,dot(n,normalize(vec3(-.8,.4,.5))));gl_FragColor=vec4(u_color*mix(light,1.,u_unlit),1.);}`;
    const shader=(type,src)=>{const s=gl.createShader(type);gl.shaderSource(s,src);gl.compileShader(s);if(!gl.getShaderParameter(s,gl.COMPILE_STATUS))throw new Error(gl.getShaderInfoLog(s));return s;};
    this.program=gl.createProgram();gl.attachShader(this.program,shader(gl.VERTEX_SHADER,vertex));gl.attachShader(this.program,shader(gl.FRAGMENT_SHADER,fragment));gl.linkProgram(this.program);if(!gl.getProgramParameter(this.program,gl.LINK_STATUS))throw new Error(gl.getProgramInfoLog(this.program));gl.useProgram(this.program);
    this.position=gl.getAttribLocation(this.program,'a_position');this.normal=gl.getAttribLocation(this.program,'a_normal');this.uniform={};for(const name of ['model','viewProjection','normal','color','clip','unlit'])this.uniform[name]=gl.getUniformLocation(this.program,`u_${name}`);
    gl.enable(gl.DEPTH_TEST);gl.disable(gl.CULL_FACE);gl.clearColor(.918,.941,.957,1);
    this.axisBuffers=[];for(let axis=0;axis<3;axis++){const data=new Float32Array(12);data[6+axis]=16;data[5]=data[11]=1;const buffer=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,buffer);gl.bufferData(gl.ARRAY_BUFFER,data,gl.STATIC_DRAW);this.axisBuffers.push({buffer,count:2});}
    let drag=null;
    canvas.addEventListener('pointerdown',e=>{drag=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId);});
    canvas.addEventListener('pointermove',e=>{if(!drag)return;this.azimuth-=(e.clientX-drag[0])*.008;this.elevation=Math.max(-1.45,Math.min(1.45,this.elevation+(e.clientY-drag[1])*.008));drag=[e.clientX,e.clientY];this.draw();});
    const stop=()=>{drag=null;};canvas.addEventListener('pointerup',stop);canvas.addEventListener('pointercancel',stop);
    canvas.addEventListener('wheel',e=>{e.preventDefault();this.distance=Math.max(90,Math.min(480,this.distance*Math.exp(e.deltaY*.001)));this.draw();},{passive:false});
    this.observer=new ResizeObserver(()=>this.draw());this.observer.observe(canvas);
    canvas.addEventListener('webglcontextlost',e=>{e.preventDefault();canvas.dispatchEvent(new CustomEvent('preview-error',{detail:'三维显示连接中断，请刷新页面。'}));});
  }
  setMesh(name,data){const gl=this.gl;if(this.meshes[name])gl.deleteBuffer(this.meshes[name].buffer);const buffer=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,buffer);gl.bufferData(gl.ARRAY_BUFFER,data,gl.STATIC_DRAW);this.meshes[name]={buffer,count:data.length/6};this.draw();}
  camera(name){if(name==='side'){this.azimuth=-90*RAD;this.elevation=3*RAD;}else if(name==='top'){this.azimuth=-90*RAD;this.elevation=89*RAD;}else{this.azimuth=-55*RAD;this.elevation=24*RAD;}this.draw();}
  drawMesh(mesh,model,color,lines=false){if(!mesh)return;const gl=this.gl;gl.bindBuffer(gl.ARRAY_BUFFER,mesh.buffer);gl.enableVertexAttribArray(this.position);gl.vertexAttribPointer(this.position,3,gl.FLOAT,false,24,0);gl.enableVertexAttribArray(this.normal);gl.vertexAttribPointer(this.normal,3,gl.FLOAT,false,24,12);gl.uniformMatrix4fv(this.uniform.model,false,model);gl.uniformMatrix3fv(this.uniform.normal,false,normalMatrix(model));gl.uniform3fv(this.uniform.color,color);gl.uniform1f(this.uniform.unlit,lines?1:0);gl.drawArrays(lines?gl.LINES:gl.TRIANGLES,0,mesh.count);}
  draw(){
    const gl=this.gl;if(!gl||gl.isContextLost())return;const dpr=Math.min(window.devicePixelRatio||1,2),w=Math.round(this.canvas.clientWidth*dpr),h=Math.round(this.canvas.clientHeight*dpr);if(!w||!h)return;if(this.canvas.width!==w||this.canvas.height!==h){this.canvas.width=w;this.canvas.height=h;}gl.viewport(0,0,w,h);gl.useProgram(this.program);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);if(!this.meta)return;
    const sep=this.meta.nominal_separation_mm+this.pose.opening_mm;
    const center=[0,0,this.mode==='single'?17:sep/2];const eye=[center[0]+this.distance*Math.cos(this.elevation)*Math.cos(this.azimuth),center[1]+this.distance*Math.cos(this.elevation)*Math.sin(this.azimuth),center[2]+this.distance*Math.sin(this.elevation)];gl.uniformMatrix4fv(this.uniform.viewProjection,false,multiply(perspective(w/h),lookAt(eye,center)));gl.uniform1f(this.uniform.clip,this.mode==='section'?1:0);
    const active=multiply(translation(this.pose.x_mm,this.pose.y_mm,sep),multiply(rotateZ((45+this.pose.yaw_deg)*RAD),multiply(rotateX(this.pose.tilt_deg*RAD),rotateX(Math.PI))));
    const drawSide=(parent,color)=>{
      this.drawMesh(this.meshes.adapter,parent,color.map(x=>x*.85));this.drawMesh(this.meshes.head_plate,parent,color);
      const zb=this.meta.base_height_mm;const stop=multiply(parent,multiply(translation(0,0,zb),multiply(scale(1,1,this.meta.stop_scale),translation(0,0,-zb))));this.drawMesh(this.meshes.stop_land,stop,color.map(x=>Math.min(1,x*1.07)));
      for(let k=0;k<4;k++)this.drawMesh(this.meshes.guide,multiply(parent,rotateZ(k*Math.PI/2)),color);
    };
    drawSide(identity(),[.40,.59,.71]);if(this.mode!=='single')drawSide(active,[.83,.64,.36]);
    if(this.axes){gl.uniform1f(this.uniform.clip,0);gl.disable(gl.DEPTH_TEST);const colors=[[.80,.25,.20],[.22,.62,.37],[.24,.40,.80]];for(let k=0;k<3;k++){this.drawMesh(this.axisBuffers[k],identity(),colors[k],true);if(this.mode!=='single')this.drawMesh(this.axisBuffers[k],active,colors[k],true);}gl.enable(gl.DEPTH_TEST);}
    const error=gl.getError();if(error!==gl.NO_ERROR)this.canvas.dispatchEvent(new CustomEvent('preview-error',{detail:`三维显示出错（${error}），请刷新页面。`}));
  }
}
