// One revision covers edits, imports, preview success and preview failure.
export class PreviewState {
  revision=0; displayed=-1; parameters=null; savePending=false; failed=false; displayFailed=false;
  edit(){this.revision++;this.failed=false;return this.revision;}
  current(id){return id===this.revision;}
  accept(id,parameters){if(!this.current(id))return false;this.displayed=id;this.parameters={...parameters};this.failed=false;return true;}
  fail(id){if(this.current(id))this.failed=true;}
  get canSave(){return this.parameters!==null&&this.displayed===this.revision&&!this.failed&&!this.displayFailed&&!this.savePending;}
  snapshot(pose){if(!this.canSave)return null;return {parameters:{...this.parameters},pose:{...pose}};}
}
export function decodePacket(packet){
  if(packet.byteLength<4)throw new Error('预览数据不完整。');
  const n=new DataView(packet).getUint32(0,true);
  if(!n||n%4||n>packet.byteLength-4)throw new Error('预览数据长度不正确。');
  const metadata=JSON.parse(new TextDecoder('utf-8',{fatal:true}).decode(new Uint8Array(packet,4,n)));
  const count=metadata.vertex_count;
  if(metadata.protocol!=='petal-preview/v1'||metadata.units!=='mm'||!Number.isSafeInteger(count)||count<=0||count>1000000||count%3||packet.byteLength!==4+n+count*24)throw new Error('预览数据格式不正确。');
  const finite=x=>typeof x==='number'&&Number.isFinite(x);
  if(!metadata.parameters||typeof metadata.parameters!=='object'||!Object.values(metadata.parameters).every(finite)||!finite(metadata.nominal_separation_mm)||!finite(metadata.base_height_mm)||!finite(metadata.stop_scale)||!Array.isArray(metadata.warnings)||!metadata.warnings.every(x=>typeof x==='string'))throw new Error('预览元数据不正确。');
  const c=metadata.curve;
  if(!c||!['angles','heights','baseline','radii','crest','valley'].every(k=>Array.isArray(c[k])&&c[k].length>=2&&c[k].every(finite))||c.angles.length!==c.heights.length||c.angles.length!==c.baseline.length||c.radii.length!==c.crest.length||c.radii.length!==c.valley.length)throw new Error('预览曲线不正确。');
  const vertices=new Float32Array(packet,4+n);
  if(!vertices.every(Number.isFinite))throw new Error('预览数据含非法坐标。');
  return {metadata,vertices};
}
