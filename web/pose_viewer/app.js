import * as THREE from 'three';
import {OrbitControls} from './lib/OrbitControls.js';
import {prepareFrames, timeLabel} from './motion.js';

const $ = id => document.getElementById(id);
const viewport = $('viewport');
let frames = [], rawFrames = [], index = 0, mode = 'replay', playing = false;
let run = '', activeSegment = -1, segmentIndices = [], playbackTime = 0;
let cloud = null, follow = false, loading = false, pollBusy = false, pollFailed = false;
let lastTrusted = null, lastLiveFrame = -1, loadGeneration = 0, liveActive = false;

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0c1520);
scene.fog = new THREE.FogExp2(0x0c1520, 0.013);
const camera = new THREE.PerspectiveCamera(48, 1, 0.01, 5000);
camera.position.set(3.5, 2.6, 4.5);
let renderer;
try {
  renderer = new THREE.WebGLRenderer({antialias: true, alpha: false});
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  viewport.appendChild(renderer.domElement);
} catch (error) {
  $('error').hidden = false;
  $('error').textContent = '浏览器无法启用 WebGL。请启用硬件加速后重新打开。';
  throw error;
}
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.minDistance = 0.08;
controls.maxDistance = 500;

// A proper 180-degree rotation about X: x right, y display-up, z display-back.
// The raw SLAM frame remains unchanged in numeric readouts; this is not gravity alignment.
const world = new THREE.Group();
world.rotation.x = Math.PI;
scene.add(world);
const gridCanvas = document.createElement('canvas');
gridCanvas.width = gridCanvas.height = 128;
const gridContext = gridCanvas.getContext('2d');
gridContext.fillStyle = '#101d2a'; gridContext.fillRect(0,0,128,128);
gridContext.strokeStyle = '#3d5369'; gridContext.lineWidth = 1.3;
gridContext.strokeRect(.5,.5,127,127);
const gridTexture = new THREE.CanvasTexture(gridCanvas);
gridTexture.wrapS = gridTexture.wrapT = THREE.RepeatWrapping;
gridTexture.repeat.set(40,40); gridTexture.colorSpace = THREE.SRGBColorSpace;
const grid = new THREE.Mesh(new THREE.PlaneGeometry(40,40),new THREE.MeshBasicMaterial({map:gridTexture,side:THREE.DoubleSide,transparent:true,opacity:.8,depthWrite:false}));
grid.rotation.x = -Math.PI/2;
grid.position.y = -.5;
scene.add(grid);
world.add(new THREE.AxesHelper(.5));
scene.add(new THREE.HemisphereLight(0xd6e9ff, 0x25333a, 2.7));
const light = new THREE.DirectionalLight(0xc6e4ff, 3);
light.position.set(4, 8, 3); scene.add(light);

const cameraRig = new THREE.Group();
const rigMaterial = new THREE.MeshStandardMaterial({color: 0xffc86c, emissive: 0xa96718, emissiveIntensity: .28, roughness: .4});
const body = new THREE.Mesh(new THREE.BoxGeometry(.10, .065, .055), rigMaterial);
cameraRig.add(body);
const lens = new THREE.Mesh(new THREE.CylinderGeometry(.025, .025, .03, 20), rigMaterial);
lens.rotation.x = Math.PI / 2; lens.position.z = .041; cameraRig.add(lens);
const corners = [[-.18,-.12,.38],[.18,-.12,.38],[.18,.12,.38],[-.18,.12,.38]];
const frustum = [];
for (let i=0;i<4;i++) frustum.push(0,0,0,...corners[i],...corners[i],...corners[(i+1)%4]);
const rigLines = new THREE.LineSegments(new THREE.BufferGeometry().setAttribute('position',new THREE.Float32BufferAttribute(frustum,3)), new THREE.LineBasicMaterial({color:0xffc86c,transparent:true,opacity:.8}));
cameraRig.add(rigLines);
cameraRig.add(new THREE.ArrowHelper(new THREE.Vector3(0,0,1),new THREE.Vector3(),.52,0xffc86c,.08,.045));
world.add(cameraRig); cameraRig.visible = false;
const origin = new THREE.Mesh(new THREE.SphereGeometry(.026,16,12),new THREE.MeshBasicMaterial({color:0x4ad8cb}));
world.add(origin); origin.visible = false;
const trail = new THREE.Line(new THREE.BufferGeometry(),new THREE.LineBasicMaterial({color:0x4ad8cb}));
const future = new THREE.Line(new THREE.BufferGeometry(),new THREE.LineBasicMaterial({color:0x344d61,transparent:true,opacity:.6}));
world.add(future,trail);

function errorMessage(text) { $('error').textContent = text; $('error').hidden = !text; }
async function api(path) {
  const response = await fetch(path, {cache:'no-store', signal:AbortSignal.timeout(10000)});
  const data = await response.json();
  if (!response.ok) throw new Error(data.error || `HTTP ${response.status}`);
  return data;
}
function shownPosition(p) { return new THREE.Vector3(p[0],-p[1],-p[2]); }
function rebuildSegment(segment) {
  activeSegment = segment;
  segmentIndices = frames.flatMap((f,i)=>f.valid && f.segment === segment ? [i] : []);
  const values = segmentIndices.flatMap(i=>frames[i].p);
  trail.geometry.dispose(); future.geometry.dispose();
  const geometry = new THREE.BufferGeometry().setAttribute('position',new THREE.Float32BufferAttribute(values,3));
  future.geometry = geometry;
  trail.geometry = geometry.clone();
  if (segmentIndices.length) {
    origin.position.fromArray(frames[segmentIndices[0]].p); origin.visible = true;
    const base = shownPosition(frames[segmentIndices[0]].p);
    grid.position.set(base.x,base.y-.5,base.z);
  }
  else origin.visible = false;
}
function fitView(top = false) {
  const positions = segmentIndices.length ? segmentIndices.map(i=>shownPosition(frames[i].p)) : [new THREE.Vector3()];
  const box = new THREE.Box3().setFromPoints(positions);
  if (cloud?.visible) box.union(new THREE.Box3().setFromObject(cloud));
  const center = box.getCenter(new THREE.Vector3());
  const span = Math.max(1.25, box.getSize(new THREE.Vector3()).length());
  controls.target.copy(center);
  camera.up.set(0,1,0);
  camera.position.copy(center).add(top ? new THREE.Vector3(.001,span*1.6,.001) : new THREE.Vector3(span*.95,span*.7,span*1.05));
  camera.lookAt(center); controls.update();
}
function setPlaying(value) { playing = value && frames.length>0 && mode==='replay'; $('play').textContent = playing ? 'Ⅱ' : '▶'; }
function setUnavailable(label, detail) {
  $('tracking').textContent = label; $('tracking').classList.add('bad');
  $('trackingDetail').textContent = detail;
  rigMaterial.color.setHex(0x79818b); rigLines.material.color.setHex(0x79818b);
}
function showFrame(nextIndex) {
  if (!frames.length) return;
  index = Math.max(0,Math.min(nextIndex,frames.length-1));
  const f = frames[index];
  $('seek').value = index;
  $('time').textContent = timeLabel(f.t-frames[0].t);
  $('duration').textContent = `/ ${timeLabel(frames.at(-1).t-frames[0].t)}`;
  $('frameLabel').textContent = `第 ${f.frame+1} 帧`;
  $('mapId').textContent = f.map_id == null || f.map_id < 0 ? '未记录' : `MAP ${f.map_id}`;
  if (!f.valid) {
    setUnavailable('跟踪失效', '摄像头停留在最后可信位置；此处不连接轨迹。');
    for (const id of ['px','py','pz','displacement','distance','angle']) $(id).textContent = '—';
    $('segmentId').textContent='已断开';
    return;
  }
  if (f.segment !== activeSegment) { lastTrusted=null; rebuildSegment(f.segment); fitView(); }
  const nextPosition = shownPosition(f.p);
  if (follow && lastTrusted) {
    const delta = nextPosition.clone().sub(shownPosition(lastTrusted.p));
    camera.position.add(delta); controls.target.add(delta);
  }
  cameraRig.position.fromArray(f.p);
  cameraRig.quaternion.fromArray(f.q).normalize();
  cameraRig.visible = true; lastTrusted = f;
  const drawn = segmentIndices.filter(i=>i<=index).length;
  trail.geometry.setDrawRange(0,drawn);
  $('tracking').textContent='定位正常'; $('tracking').classList.remove('bad');
  $('trackingDetail').textContent=mode==='live'?'实时摄像头位姿 · 约 10 Hz 刷新':'已有记录回放 · 非当前现场位置';
  rigMaterial.color.setHex(0xffc86c); rigLines.material.color.setHex(0xffc86c);
  $('displacement').textContent=f.displacement.toFixed(3);
  $('distance').innerHTML=`${f.distance.toFixed(3)} <small>m</small>`;
  $('angle').innerHTML=`${f.angle.toFixed(1)} <small>°</small>`;
  ['px','py','pz'].forEach((id,i)=>$(id).textContent=f.p[i].toFixed(3));
  $('segmentId').textContent=`${f.segment+1}`;
}
function updateFrames(data) {
  rawFrames = data.frames;
  frames = prepareFrames(rawFrames);
  $('seek').max=Math.max(0,frames.length-1);
  $('empty').hidden=frames.some(f=>f.valid);
  $('okRatio').textContent=frames.length ? `${(100*frames.filter(f=>f.valid).length/frames.length).toFixed(1)}%` : '—';
  $('segmentNote').textContent=data.map_ids_recorded
    ? '按地图编号、跟踪失效、时间中断与明显位置跳变划分片段；仅显示当前片段。'
    : '旧记录未保存地图编号。按跟踪失效与明显跳变断开，仅显示当前片段；不能确认跨段坐标一致。';
}
async function loadRun(name, preserveMode = false) {
  if (!name) return;
  const generation = ++loadGeneration;
  loading=true; setPlaying(false); errorMessage('');
  try {
    const data=await api(`/api/run?run=${encodeURIComponent(name)}`);
    if (generation!==loadGeneration) return;
    run=name; $('run').value=name; activeSegment=-1; lastTrusted=null; lastLiveFrame=-1;
    cameraRig.visible=false; origin.visible=false;
    if(cloud){world.remove(cloud);cloud.geometry.dispose();cloud.material.dispose();cloud=null;}
    $('cloud').checked=false; $('cloud').disabled=!data.cloud_available;
    updateFrames(data); rebuildSegment(-1); playbackTime=frames[0]?.t||0;
    if(!preserveMode) setMode('replay');
    showFrame(mode==='live'?frames.length-1:0); fitView();
    $('sceneSubtitle').textContent=`${name.replace('run_','')} · ${frames.length} 帧 · 摄像头位姿`;
  } catch(error){errorMessage(`无法读取记录：${error.message}`);}
  finally{if(generation===loadGeneration) loading=false;}
}
async function refreshRuns() {
  const data=await api('/api/runs');
  $('run').replaceChildren(...data.runs.map(name=>new Option(name.replace('run_',''),name)));
  if(data.runs.includes(run)) $('run').value=run;
  if(!data.runs.length){$('empty').hidden=false;$('run').add(new Option('暂无运行记录',''));}
  return data;
}
function setMode(value) {
  mode=value; setPlaying(false); liveActive=false;
  $('replayMode').classList.toggle('selected',value==='replay');
  $('liveMode').classList.toggle('selected',value==='live');
  $('modeBadge').textContent=value==='live'?'实时':'回放';
  $('play').disabled=value==='live'; $('seek').disabled=value==='live'; $('speed').disabled=value==='live';
  if(value==='live') {setUnavailable('等待实时数据','请启动 SLAM；页面不会自行开启摄像头。'); pollLive();}
  else showFrame(index);
}
async function pollLive() {
  if(mode!=='live'||pollBusy||loading) return;
  pollBusy=true;
  try {
    const data=await api('/api/live');
    if(mode!=='live') return;
    if(data.run && data.run!==run){await refreshRuns();await loadRun(data.run,true);}
    liveActive=Boolean(data.active); pollFailed=false;
    if(!data.available){setUnavailable('暂无实时数据',data.reason||'请先启动 SLAM。');return;}
    const s=data.telemetry;
    if(s.frame>lastLiveFrame && s.frame>=0 && s.p && s.q){
      lastLiveFrame=s.frame;
      if(!rawFrames.length||s.frame>rawFrames.at(-1).frame){
        const appended={frame:s.frame,t:s.timestamp_s,state:s.state,valid:s.valid,map_id:s.map_id,source_segment:s.source_segment,p:s.p,q:s.q};
        updateFrames({frames:[...rawFrames,appended],map_ids_recorded:true});
        // Include the newest vertex in the active line buffer.
        rebuildSegment(activeSegment);
      }
      showFrame(frames.length-1);
    }
    if(!data.active){setUnavailable(s.lifecycle==='finished'?'采集已结束':'数据已停止更新',`保留最后可信位置 · ${data.age_s.toFixed(1)} 秒未更新。`);}
  }catch(error){pollFailed=true;liveActive=false;setUnavailable('连接中断','保留最后可信位置，正在重试。');}
  finally{pollBusy=false;}
}

$('play').onclick=()=>{if(index>=frames.length-1){playbackTime=frames[0].t;showFrame(0);}setPlaying(!playing);};
$('seek').oninput=()=>{setPlaying(false);showFrame(Number($('seek').value));playbackTime=frames[index]?.t||0;};
$('run').onchange=()=>loadRun($('run').value);
$('refresh').onclick=async()=>{try{const data=await refreshRuns();await loadRun(run||data.latest);}catch(e){errorMessage(e.message);}};
$('replayMode').onclick=()=>setMode('replay');
$('liveMode').onclick=()=>setMode('live');
$('fit').onclick=()=>fitView(); $('top').onclick=()=>fitView(true);
$('follow').onclick=()=>{follow=!follow;$('follow').setAttribute('aria-pressed',String(follow));};
$('cloud').onchange=async()=>{
  const wanted=$('cloud').checked, cloudRun=run;
  if(cloud){cloud.visible=wanted;return;}
  if(!wanted)return;
  $('cloudNote').textContent='正在读取参考点云…';
  try{
    const data=await api(`/api/cloud?run=${encodeURIComponent(run)}`);
    if(run!==cloudRun)return;
    const geometry=new THREE.BufferGeometry().setAttribute('position',new THREE.Float32BufferAttribute(data.points.flat(),3));
    cloud=new THREE.Points(geometry,new THREE.PointsMaterial({color:0x7f96a9,size:.018,transparent:true,opacity:.55}));
    cloud.visible=$('cloud').checked;world.add(cloud);
    $('cloudNote').textContent=`${data.points.length.toLocaleString()} 个抽样点。${data.warning}`;
  }catch(error){$('cloud').checked=false;$('cloudNote').textContent=`点云读取失败：${error.message}`;}
};
new ResizeObserver(()=>{const w=viewport.clientWidth,h=viewport.clientHeight;camera.aspect=w/h;camera.updateProjectionMatrix();renderer.setSize(w,h);}).observe(viewport);
let previousTick=performance.now();
function animate(now){
  requestAnimationFrame(animate);
  const dt=Math.min((now-previousTick)/1000,.1);previousTick=now;
  if(playing&&frames.length&&!loading){
    playbackTime+=dt*Number($('speed').value);
    let next=index;
    while(next+1<frames.length&&frames[next+1].t<=playbackTime)next++;
    if(next!==index)showFrame(next);
    if(next===frames.length-1)setPlaying(false);
  }
  controls.update();renderer.render(scene,camera);
}
requestAnimationFrame(animate);
setInterval(pollLive,250);
try{const data=await refreshRuns();const requested=new URLSearchParams(location.search).get('run');if(data.latest)await loadRun(data.runs.includes(requested)?requested:data.latest);}
catch(error){errorMessage(`无法连接设备：${error.message}`);}
// Compact diagnostics for automated browser verification; contains no controls or secrets.
window.viewerDiagnostics=()=>({mode,run,frame:frames[index]?.frame,frames:frames.length,valid:frames[index]?.valid,segment:activeSegment,playing,liveActive,pollFailed,cameraVisible:cameraRig.visible,cloudPoints:cloud?.geometry.attributes.position.count||0,viewPosition:camera.position.toArray(),viewTarget:controls.target.toArray(),gridPosition:grid.position.toArray()});
