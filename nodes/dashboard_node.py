"""
dashboard_node.py - pi_robot live dashboard
Open: http://<PI_IP>:8080
"""
import json, queue, threading, time, logging, sys, os
from http.server import HTTPServer, BaseHTTPRequestHandler
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from core.base_node   import BaseNode
from core.message_bus import Topics

logger = logging.getLogger("dashboard_node")
PORT = 8080

_data = {
    "nodes": {n:"unknown" for n in ["camera_vision_node","decision_node",
        "command_node","serial_node","logger_node","manual_node","ps4_node","dashboard_node"]},
    "robot_state":"MANUAL","system_mode":"MANUAL","ctrl_mode":"BASE",
    "qr_detected":False,"qr_data":"",
    "cube_color":"UNKNOWN","cube_color_rgb":[120,120,120],"color_confidence":0.0,
    "bins":{"RED":0,"GREEN":0,"BLUE":0},
    "base":{"FL":0,"FR":0,"RL":0,"RR":0,"target_FL":0,"target_FR":0,"target_RL":0,"target_RR":0,"last_cmd":"","last_fb":"","speed_pct":0},
    "arm":{"j0":0.0,"j1":0.0,"j2":0.0,"j3":0.0,"gripper":90,"moving":False,"last_cmd":"","last_fb":""},
    "logs":[],"start_time":time.time(),
}
_data["nodes"]["dashboard_node"] = "running"
_lock = threading.Lock()

def _set(key, val):
    with _lock:
        keys = key.split(".")
        d = _data
        for k in keys[:-1]: d = d[k]
        d[keys[-1]] = val

def _log(msg):
    with _lock:
        _data["logs"].append({"t":time.strftime("%H:%M:%S"),"m":msg[:100]})
        if len(_data["logs"]) > 40: _data["logs"].pop(0)

HTML = """<!DOCTYPE html>
<html><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>pi_robot</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:Arial,sans-serif;background:#0a0c14;color:#e2e8f0;padding:10px}
h1{font-size:1.2rem;color:#4f8ef7;margin-bottom:10px;padding:8px;background:#111825;border-radius:8px}
.grid{display:grid;grid-template-columns:1fr 1fr 1fr;gap:10px}
.card{background:#111825;border:1px solid #1e2a3a;border-radius:10px;padding:12px}
.card h2{font-size:11px;color:#6b7a9a;text-transform:uppercase;letter-spacing:.1em;margin-bottom:10px;border-bottom:1px solid #1e2a3a;padding-bottom:6px}
.row{display:flex;justify-content:space-between;align-items:center;padding:4px 0;border-bottom:1px solid #161e2e;font-size:12px}
.row:last-child{border-bottom:none}
.dot{width:8px;height:8px;border-radius:50%;display:inline-block;margin-right:6px}
.g{background:#34d399}.r{background:#f87171}.y{background:#6b7280}
.badge{padding:2px 8px;border-radius:12px;font-size:11px;font-weight:600}
.b-manual{background:#0f2744;color:#60a5fa}
.b-auto{background:#072a1e;color:#34d399}
.b-base{background:#0f2744;color:#60a5fa}
.b-arm{background:#1a0f40;color:#c4b5fd}
.b-idle{background:#1a1f2e;color:#6b7280}
.b-run{background:#072a1e;color:#34d399}
.swatch{width:40px;height:40px;border-radius:8px;border:2px solid rgba(255,255,255,.1)}
.bar-wrap{height:8px;background:#1e2535;border-radius:4px;overflow:hidden;margin-top:3px}
.bar{height:100%;border-radius:4px;transition:width .3s}
.motor{display:grid;grid-template-columns:1fr 1fr;gap:6px}
.mbox{background:#161e2e;border-radius:6px;padding:8px;text-align:center}
.mlab{font-size:10px;color:#6b7a9a}
.mval{font-size:1.1rem;font-weight:700;color:#4f8ef7}
.jrow{display:flex;align-items:center;gap:6px;padding:3px 0;font-size:11px}
.jname{color:#6b7a9a;min-width:70px}
.jbg{flex:1;height:5px;background:#1e2535;border-radius:3px;overflow:hidden}
.jbar{height:100%;border-radius:3px;transition:width .3s}
.jval{min-width:38px;text-align:right;font-weight:600}
.grip{display:flex;align-items:center;gap:8px;margin-top:6px}
.grip-bg{flex:1;height:10px;background:#1e2535;border-radius:5px;overflow:hidden}
.grip-bar{height:100%;background:#34d399;border-radius:5px;transition:width .3s}
.log{background:#161e2e;border-radius:6px;padding:8px;height:120px;overflow-y:auto;font-family:monospace;font-size:10px}
.lline{padding:1px 0;color:#6b7a9a}
.lts{color:#4f8ef7;margin-right:4px}
.cmd{font-family:monospace;font-size:11px;padding:5px 7px;background:#161e2e;border-radius:5px;color:#fb923c;min-height:24px;word-break:break-all}
.steps{display:flex;flex-direction:column;gap:4px}
.step{display:flex;align-items:center;gap:6px;padding:4px 7px;border-radius:5px;font-size:11px;border:1px solid transparent;color:#6b7a9a}
.step.act{background:rgba(79,142,247,.15);border-color:rgba(79,142,247,.3);color:#e2e8f0}
.step.done{background:rgba(52,211,153,.07);border-color:rgba(52,211,153,.2)}
.sico{width:16px;height:16px;border-radius:50%;display:flex;align-items:center;justify-content:center;font-size:9px;flex-shrink:0}
.ia{background:#4f8ef7;color:#fff}.id{background:#34d399;color:#000}.ii{background:#1e2535}
.lbl{font-size:10px;color:#6b7a9a;margin-bottom:3px;margin-top:6px}
@media(max-width:768px){.grid{grid-template-columns:1fr}}
</style>
</head><body>
<h1>&#129302; pi_robot &nbsp;<span id="UP" style="font-size:11px;color:#6b7a9a;font-weight:400">0s</span></h1>
<div class="grid">

<!-- SECTION 1: ROBOT STATE -->
<div class="card">
<h2>&#9899; Robot State</h2>
<div class="row"><span>State</span><span class="badge b-idle" id="RS">MANUAL</span></div>
<div class="row"><span>System Mode</span><span class="badge b-manual" id="SM">MANUAL</span></div>
<div class="row"><span>PS4 Mode</span><span class="badge b-base" id="CM">BASE</span></div>

<div class="lbl">Cube Detection</div>
<div style="display:flex;align-items:center;gap:10px;padding:8px;background:#161e2e;border-radius:8px">
  <div class="swatch" id="CSW" style="background:#222"></div>
  <div>
    <div style="font-weight:700;font-size:1rem" id="CN">No cube</div>
    <div style="font-size:10px;color:#6b7a9a" id="CC">—</div>
  </div>
</div>

<div class="lbl">Bins</div>
<div style="display:grid;grid-template-columns:1fr 1fr 1fr;gap:6px">
  <div style="background:#1a0808;border:1px solid #7f1d1d;border-radius:7px;padding:8px;text-align:center">
    <div style="font-size:1.4rem;font-weight:700;color:#f87171" id="BR">0</div>
    <div style="font-size:10px;color:#6b7a9a">RED</div>
  </div>
  <div style="background:#071a0e;border:1px solid #14532d;border-radius:7px;padding:8px;text-align:center">
    <div style="font-size:1.4rem;font-weight:700;color:#34d399" id="BG">0</div>
    <div style="font-size:10px;color:#6b7a9a">GREEN</div>
  </div>
  <div style="background:#081020;border:1px solid #1e3a5f;border-radius:7px;padding:8px;text-align:center">
    <div style="font-size:1.4rem;font-weight:700;color:#60a5fa" id="BB">0</div>
    <div style="font-size:10px;color:#6b7a9a">BLUE</div>
  </div>
</div>

<div class="lbl">Auto Sequence</div>
<div class="steps" id="STEPS"></div>

<div class="lbl">Log</div>
<div class="log" id="LOG"></div>
</div>

<!-- SECTION 2: BASE -->
<div class="card">
<h2>&#9881; Base (Mecanum)</h2>
<div class="lbl">Last Command</div>
<div class="cmd" id="BCMD">—</div>
<div class="lbl">Serial Feedback</div>
<div class="cmd" style="color:#34d399" id="BFB">—</div>

<div class="lbl">Speed</div>
<div style="font-size:11px;color:#6b7a9a;margin-bottom:2px">Current: <span id="SPCT">0</span>%</div>
<div class="bar-wrap"><div class="bar" id="SBAR" style="width:0%;background:#34d399"></div></div>

<div class="lbl">Motors (PPS)</div>
<div class="motor" id="MOT"></div>

<div class="lbl">Direction</div>
<div style="display:grid;grid-template-columns:repeat(3,1fr);gap:5px;max-width:180px;margin:0 auto">
  <div></div>
  <div id="DF" style="padding:7px;text-align:center;font-size:16px;border-radius:6px;background:#161e2e;border:1px solid #1e2a3a;transition:all .2s">▲</div>
  <div></div>
  <div id="DL" style="padding:7px;text-align:center;font-size:16px;border-radius:6px;background:#161e2e;border:1px solid #1e2a3a;transition:all .2s">◄</div>
  <div id="DS" style="padding:7px;text-align:center;font-size:10px;font-weight:600;border-radius:6px;background:#161e2e;border:1px solid #1e2a3a;color:#6b7a9a;transition:all .2s">STOP</div>
  <div id="DR" style="padding:7px;text-align:center;font-size:16px;border-radius:6px;background:#161e2e;border:1px solid #1e2a3a;transition:all .2s">►</div>
  <div></div>
  <div id="DB" style="padding:7px;text-align:center;font-size:16px;border-radius:6px;background:#161e2e;border:1px solid #1e2a3a;transition:all .2s">▼</div>
  <div></div>
</div>
</div>

<!-- SECTION 3: ARM -->
<div class="card">
<h2>&#129302; Arm</h2>
<div class="lbl">Last Command</div>
<div class="cmd" style="color:#c4b5fd" id="ACMD">—</div>
<div class="lbl">Serial Feedback</div>
<div class="cmd" style="color:#a78bfa" id="AFB">—</div>

<div class="lbl">Joints</div>
<div id="JNT"></div>

<div class="lbl">Gripper</div>
<div class="grip">
  <span style="font-size:10px;color:#6b7a9a;min-width:50px">Position</span>
  <div class="grip-bg"><div class="grip-bar" id="GRB" style="width:50%"></div></div>
  <span style="font-weight:700;min-width:35px;text-align:right" id="GRV">90°</span>
</div>
<div style="display:flex;justify-content:space-between;font-size:9px;color:#6b7a9a;margin-top:2px">
  <span>0° Closed</span><span>180° Open</span>
</div>
<div style="font-size:10px;text-align:center;margin-top:6px;color:#6b7a9a" id="AMOV">idle</div>

<div class="lbl">Arm View</div>
<canvas id="ACV" width="210" height="180" style="display:block;margin:0 auto;background:#161e2e;border-radius:8px"></canvas>

<div class="lbl">Nodes</div>
<div id="NDS"></div>
</div>

</div>

<script>
const STEPS=[{id:"MANUAL",l:"Manual phase"},{id:"AUTO_X_GO",l:"X motion (RED)"},{id:"AUTO_X_PLACE",l:"Place RED"},{id:"AUTO_X_BACK",l:"Return from X"},{id:"AUTO_Y_GO",l:"Y motion (GREEN)"},{id:"AUTO_Y_PLACE",l:"Place GREEN"},{id:"AUTO_Y_BACK",l:"Return from Y"},{id:"AUTO_D_GO",l:"Diagonal (BLUE)"},{id:"AUTO_D_PLACE",l:"Place BLUE"},{id:"AUTO_D_BACK",l:"Return diag"},{id:"AUTO_DONE",l:"Complete!"}];
const JNTS=[{id:"j0",n:"J0 Base",c:"#4f8ef7"},{id:"j1",n:"J1 Shoulder",c:"#7c5cbf"},{id:"j2",n:"J2 Elbow",c:"#34d399"},{id:"j3",n:"J3 Wrist",c:"#fb923c"}];
const MTRS=["FL","FR","RL","RR"];
const NODES=["camera_vision_node","decision_node","command_node","serial_node","logger_node","manual_node","ps4_node","dashboard_node"];
const SC={MANUAL:"b-manual",IDLE:"b-idle",AUTO_X_GO:"b-auto",AUTO_X_PLACE:"b-auto",AUTO_X_BACK:"b-auto",AUTO_Y_GO:"b-auto",AUTO_Y_PLACE:"b-auto",AUTO_Y_BACK:"b-auto",AUTO_D_GO:"b-auto",AUTO_D_PLACE:"b-auto",AUTO_D_BACK:"b-auto",AUTO_DONE:"b-auto"};

// Build steps
const sc=document.getElementById("STEPS");
STEPS.forEach(s=>sc.innerHTML+=`<div class="step" id="ST_${s.id}"><div class="sico ii" id="SI_${s.id}">&#9679;</div><span>${s.l}</span></div>`);

// Build motors
const mc=document.getElementById("MOT");
MTRS.forEach(m=>mc.innerHTML+=`<div class="mbox"><div class="mlab">${m}</div><div class="mval" id="MV_${m}">0</div><div class="bar-wrap"><div class="bar" id="MB_${m}" style="width:0%;background:#4f8ef7"></div></div><div style="font-size:9px;color:#6b7a9a;margin-top:2px" id="MT_${m}">tgt:0</div></div>`);

// Build joints
const jc=document.getElementById("JNT");
JNTS.forEach(j=>jc.innerHTML+=`<div class="jrow"><span class="jname">${j.n}</span><div class="jbg"><div class="jbar" id="JB_${j.id}" style="width:50%;background:${j.c}"></div></div><span class="jval" id="JV_${j.id}">0°</span></div>`);

// Build nodes
const nc=document.getElementById("NDS");
NODES.forEach(n=>nc.innerHTML+=`<div class="row"><span><span class="dot y" id="ND_${n}"></span>${n.replace("_node","")}</span><span style="font-size:10px;color:#6b7a9a" id="NS_${n}">—</span></div>`);

let lastDir="";
function setDir(d){
  if(d===lastDir)return;lastDir=d;
  ["DF","DB","DL","DR","DS"].forEach(id=>{
    const el=document.getElementById(id);
    if(el){el.style.background="#161e2e";el.style.borderColor="#1e2a3a";el.style.color="#e2e8f0";}
  });
  const map={FWD:"DF",BWD:"DB",L:"DL",R:"DR",STP:"DS"};
  const el=document.getElementById(map[d]);
  if(el){el.style.background="rgba(79,142,247,.25)";el.style.borderColor="#4f8ef7";}
}

function drawArm(j0,j1,j2,j3){
  const cv=document.getElementById("ACV");
  const ctx=cv.getContext("2d");
  ctx.clearRect(0,0,cv.width,cv.height);
  const bx=cv.width/2,by=cv.height-15;
  // Base circle
  ctx.beginPath();ctx.arc(bx,by,14,0,Math.PI*2);
  ctx.strokeStyle="#4f8ef7";ctx.lineWidth=2;ctx.stroke();
  const baseRad=j0*Math.PI/180;
  ctx.beginPath();ctx.moveTo(bx,by);
  ctx.lineTo(bx+Math.sin(baseRad)*14,by-Math.cos(baseRad)*14);
  ctx.strokeStyle="#60a5fa";ctx.lineWidth=3;ctx.stroke();
  // Arm links
  let cx=bx,cy=by-14,ang=-Math.PI/2;
  const lens=[48,42,35],cols=["#7c5cbf","#34d399","#fb923c"],jnts=[j1,j2,j3];
  for(let i=0;i<3;i++){
    ang+=jnts[i]*Math.PI/180;
    const nx=cx+Math.cos(ang)*lens[i],ny=cy+Math.sin(ang)*lens[i];
    ctx.beginPath();ctx.moveTo(cx,cy);ctx.lineTo(nx,ny);
    ctx.strokeStyle=cols[i];ctx.lineWidth=i===0?5:i===1?4:3;ctx.lineCap="round";ctx.stroke();
    ctx.beginPath();ctx.arc(cx,cy,3,0,Math.PI*2);
    ctx.fillStyle=cols[i];ctx.fill();
    cx=nx;cy=ny;
  }
  ctx.beginPath();ctx.arc(cx,cy,4,0,Math.PI*2);ctx.fillStyle="#f87171";ctx.fill();
}

function upd(d){
  // Uptime
  const u=Math.floor(d.uptime||0);
  const h=Math.floor(u/3600),m=Math.floor((u%3600)/60),s=u%60;
  document.getElementById("UP").textContent=h>0?`${h}h ${m}m ${s}s`:m>0?`${m}m ${s}s`:`${s}s`;

  // State + modes
  const rs=document.getElementById("RS");rs.textContent=d.robot_state;rs.className="badge "+(SC[d.robot_state]||"b-idle");
  const sm=document.getElementById("SM");sm.textContent=d.system_mode;sm.className="badge "+(d.system_mode==="AUTO"?"b-auto":"b-manual");
  const cm=document.getElementById("CM");cm.textContent=d.ctrl_mode;cm.className="badge "+(d.ctrl_mode==="ARM"?"b-arm":"b-base");

  // Cube color
  const [r,g,b]=d.cube_color_rgb||[120,120,120];
  const hex=`rgb(${r},${g},${b})`;
  document.getElementById("CSW").style.background=hex;
  document.getElementById("CSW").style.boxShadow=d.cube_color!=="UNKNOWN"?`0 0 10px rgba(${r},${g},${b},.5)`:"none";
  const cn=document.getElementById("CN");cn.textContent=d.cube_color==="UNKNOWN"?"No cube":d.cube_color+" cube";cn.style.color=hex;
  document.getElementById("CC").textContent=d.cube_color!=="UNKNOWN"?"Conf: "+Math.round((d.color_confidence||0)*100)+"%":"—";

  // Bins
  document.getElementById("BR").textContent=d.bins.RED||0;
  document.getElementById("BG").textContent=d.bins.GREEN||0;
  document.getElementById("BB").textContent=d.bins.BLUE||0;

  // Steps
  const ci=STEPS.findIndex(s=>s.id===d.robot_state);
  STEPS.forEach((s,i)=>{
    const row=document.getElementById("ST_"+s.id);
    const ico=document.getElementById("SI_"+s.id);
    if(!row)return;
    if(s.id===d.robot_state){row.className="step act";ico.className="sico ia";ico.innerHTML="&#9654;";}
    else if(ci>-1&&i<ci){row.className="step done";ico.className="sico id";ico.innerHTML="&#10003;";}
    else{row.className="step";ico.className="sico ii";ico.innerHTML="&#9679;";}
  });

  // Log
  const la=document.getElementById("LOG");
  la.innerHTML=(d.logs||[]).slice(-20).map(l=>`<div class="lline"><span class="lts">${l.t}</span>${l.m}</div>`).join("");
  la.scrollTop=la.scrollHeight;

  // Base
  const base=d.base||{};
  document.getElementById("BCMD").textContent=base.last_cmd||"—";
  document.getElementById("BFB").textContent=base.last_fb||"—";
  const sp=base.speed_pct||0;
  document.getElementById("SPCT").textContent=sp;
  const sb=document.getElementById("SBAR");
  sb.style.width=sp+"%";sb.style.background=sp>70?"#f87171":sp>40?"#fbbf24":"#34d399";

  MTRS.forEach(m=>{
    const v=Math.round(base[m]||0),t=Math.round(base["target_"+m]||0);
    const p=Math.min(100,Math.abs(v)/15.4).toFixed(1);
    const el=document.getElementById("MV_"+m);
    if(el){el.textContent=v;el.style.color=v!==0?"#4f8ef7":"#4b5563";}
    const bar=document.getElementById("MB_"+m);if(bar)bar.style.width=p+"%";
    const te=document.getElementById("MT_"+m);if(te)te.textContent="tgt:"+t;
  });

  const cmd=(base.last_cmd||"").toUpperCase();
  if(cmd.includes("FORWARD"))setDir("FWD");
  else if(cmd.includes("BACKWARD"))setDir("BWD");
  else if(cmd.includes("LEFT"))setDir("L");
  else if(cmd.includes("RIGHT"))setDir("R");
  else if(cmd.includes("STOP"))setDir("STP");

  // Arm
  const arm=d.arm||{};
  document.getElementById("ACMD").textContent=arm.last_cmd||"—";
  document.getElementById("AFB").textContent=arm.last_fb||"—";
  JNTS.forEach(j=>{
    const deg=arm[j.id]||0;
    const bar=document.getElementById("JB_"+j.id);if(bar)bar.style.width=((deg+90)/180*100).toFixed(1)+"%";
    const val=document.getElementById("JV_"+j.id);if(val)val.textContent=deg.toFixed(1)+"°";
  });
  const g=arm.gripper||90;
  document.getElementById("GRB").style.width=(g/180*100).toFixed(1)+"%";
  document.getElementById("GRV").textContent=g+"°";
  const as=document.getElementById("AMOV");
  if(as){as.textContent=arm.moving?"● moving":"idle";as.style.color=arm.moving?"#4f8ef7":"#6b7a9a";}
  drawArm(arm.j0||0,arm.j1||0,arm.j2||0,arm.j3||0);

  // Nodes
  NODES.forEach(n=>{
    const st=d.nodes[n]||"unknown";
    const dot=document.getElementById("ND_"+n);
    const lbl=document.getElementById("NS_"+n);
    if(dot)dot.className="dot "+(st==="running"?"g":st==="error"?"r":"y");
    if(lbl)lbl.textContent=st;
  });
}

async function poll(){
  try{
    const r=await fetch("/state");
    if(r.ok){const d=await r.json();upd(d);}
  }catch(e){console.log("fetch error",e);}
}
setInterval(poll,500);
poll();
drawArm(0,0,0,0);
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path in ("/","/dashboard"):
            body=HTML.encode()
            self.send_response(200)
            self.send_header("Content-Type","text/html; charset=utf-8")
            self.send_header("Content-Length",str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path=="/state":
            with _lock:
                out=dict(_data)
                out["uptime"]=time.time()-_data["start_time"]
            body=json.dumps(out,default=str).encode()
            self.send_response(200)
            self.send_header("Content-Type","application/json")
            self.send_header("Content-Length",str(len(body)))
            self.send_header("Access-Control-Allow-Origin","*")
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_response(404)
            self.end_headers()
    def log_message(self,*a):pass


class DashboardNode(BaseNode):
    def __init__(self,**kwargs):
        super().__init__(name="dashboard_node",**kwargs)
        self._server=None
        self._fq=self._mq=self._aq=self._manq=self._lq=self._qrq=self._qlq=self._cq=None

    def setup(self):
        self._fq  =self.bus.subscribe(Topics.SERIAL_FEEDBACK,    maxsize=20)
        self._mq  =self.bus.subscribe(Topics.SYSTEM_MODE,        maxsize=10)
        self._aq  =self.bus.subscribe(Topics.DECISION_ACTION,    maxsize=20)
        self._manq=self.bus.subscribe(Topics.MANUAL_COMMAND,     maxsize=20)
        self._lq  =self.bus.subscribe(Topics.LOG_EVENT,          maxsize=50)
        self._qrq =self.bus.subscribe(Topics.VISION_QR_DETECTED, maxsize=10)
        self._qlq =self.bus.subscribe(Topics.VISION_QR_LOST,     maxsize=10)
        self._cq  =self.bus.subscribe(Topics.COMMAND_SERIAL,     maxsize=20)
        self._server=HTTPServer(("0.0.0.0",PORT),Handler)
        threading.Thread(target=self._server.serve_forever,daemon=True,name="dash").start()
        import socket
        try:
            s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
            s.connect(("8.8.8.8",80));ip=s.getsockname()[0];s.close()
        except:
            ip="192.168.137.240"
        logger.info(f"Dashboard → http://{ip}:{PORT}")
        print(f"\n[Dashboard] http://{ip}:{PORT}\n")

    def run(self):
        logger.info("dashboard_node running")
        _set("nodes.dashboard_node","running")
        while self.running:
            self._fb();self._mode();self._action()
            self._manual();self._log();self._qr();self._cmd()
            with _lock:_data["uptime"]=time.time()-_data["start_time"]
            time.sleep(0.1)

    def cleanup(self):
        if self._server:self._server.shutdown()

    def _drain(self,q):
        r=[]
        while True:
            try:r.append(q.get_nowait())
            except queue.Empty:break
        return r

    def _fb(self):
        for msg in self._drain(self._fq):
            p=msg["payload"];src=p.get("source","");raw=p.get("raw","")
            if src=="base":
                _set("base.last_fb",raw)
                if raw.startswith("spd:"):
                    try:
                        pts=raw.split("|");spd=pts[0][4:].split(",")
                        tgt=pts[1][4:].split(",") if len(pts)>1 else [0]*4
                        with _lock:
                            for i,k in enumerate(["FL","FR","RL","RR"]):
                                _data["base"][k]=int(spd[i]) if i<len(spd) else 0
                                _data["base"]["target_"+k]=int(tgt[i]) if i<len(tgt) else 0
                            avg=sum(abs(_data["base"][k]) for k in ["FL","FR","RL","RR"])/4
                            _data["base"]["speed_pct"]=int(min(100,avg/15.4))
                    except:pass
            elif src=="arm":
                _set("arm.last_fb",raw)
                if raw.startswith("STATE"):
                    try:
                        for pt in raw.split():
                            if pt.startswith("deg="):
                                degs=pt[4:].split(",")
                                with _lock:
                                    for i,k in enumerate(["j0","j1","j2","j3"]):
                                        _data["arm"][k]=float(degs[i]) if i<len(degs) else 0.0
                            elif pt.startswith("moving="):_set("arm.moving",pt[7:]=="1")
                            elif pt.startswith("gripper="):_set("arm.gripper",int(pt[8:]))
                    except:pass

    def _mode(self):
        for msg in self._drain(self._mq):
            mode=msg["payload"].get("mode","MANUAL").upper()
            _set("system_mode",mode);_log(f"Mode → {mode}")

    def _action(self):
        for msg in self._drain(self._aq):
            p=msg["payload"]
            state=p.get("state","");color=p.get("color","UNKNOWN");cmd=p.get("command","")
            if state:_set("robot_state",state)
            if color and color!="UNKNOWN":_set("cube_color",color)
            if cmd=="ARM:PLACE" and color in("RED","GREEN","BLUE"):
                with _lock:_data["bins"][color]=_data["bins"].get(color,0)+1

    def _manual(self):
        for msg in self._drain(self._manq):
            cmd=msg["payload"].get("command","")
            if cmd.startswith("BASE:"):_set("ctrl_mode","BASE");_set("base.last_cmd",cmd)
            elif cmd.startswith("ARM:"):_set("ctrl_mode","ARM");_set("arm.last_cmd",cmd)
            if cmd.startswith("ARM:RAW:JOINT"):
                try:
                    toks=cmd[8:].split()
                    if len(toks)==3:
                        ji=int(toks[1]);jd=float(toks[2])
                        if ji<4:_set(f"arm.{['j0','j1','j2','j3'][ji]}",jd)
                except:pass
            elif cmd.startswith("ARM:RAW:GRIPPER"):
                try:_set("arm.gripper",int(cmd.split()[-1]))
                except:pass

    def _log(self):
        for msg in self._drain(self._lq):
            t=msg["payload"].get("message","")
            if t:_log(t)

    def _qr(self):
        for msg in self._drain(self._qrq):
            p=msg["payload"]
            _set("qr_detected",True);_set("qr_data",p.get("qr_data",""))
            _set("cube_color",p.get("color","UNKNOWN"))
            _set("cube_color_rgb",p.get("color_rgb",[120,120,120]))
            _set("color_confidence",p.get("confidence",0.0))
        for _ in self._drain(self._qlq):
            _set("qr_detected",False);_set("qr_data","")

    def _cmd(self):
        for msg in self._drain(self._cq):
            raw=msg["payload"].get("command_str","").strip()
            if not raw:continue
            clean=raw.replace("\n"," | ")
            if "BASE:" in raw:_set("base.last_cmd",clean)
            elif "ARM:" in raw:_set("arm.last_cmd",clean)

    def update_node_status(self,name,status):
        _set(f"nodes.{name}",status)
