#!/usr/bin/env python3
"""The room-mapper live camera/map page.

It serves the live camera, SLAM map, status, object JSON, and the autonomous
mapper's pause/resume safety controls.
"""

import json
import math
import os
import socket
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np


PAGE = """<!doctype html><html><head><meta charset='utf-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<title>JetHexa room mapper</title><style>
:root{color-scheme:dark;--bg:#080d12;--panel:#101820;--panel-2:#15212b;--line:#263746;--text:#eef5f8;--muted:#91a6b5;--cyan:#38d6e8;--green:#35d07f;--red:#ff5d68;--amber:#f6ad3c;--radius:16px;--shadow:0 18px 45px rgba(0,0,0,.32)}
*{box-sizing:border-box}html{background:var(--bg)}body{min-height:100vh;margin:0;color:var(--text);font-family:Inter,ui-sans-serif,system-ui,-apple-system,"Segoe UI",sans-serif;background:radial-gradient(circle at 15% -10%,rgba(56,214,232,.15),transparent 30rem),linear-gradient(180deg,#0b1219,var(--bg) 45%)}
.shell{width:min(1180px,calc(100% - 32px));margin:auto;padding:28px 0 38px}.topbar{display:flex;align-items:center;justify-content:space-between;gap:20px;margin-bottom:20px}.brand{display:flex;align-items:center;gap:13px}.mark{display:grid;place-items:center;width:44px;height:44px;border:1px solid rgba(56,214,232,.42);border-radius:13px;background:rgba(56,214,232,.09);color:var(--cyan);font-weight:900;letter-spacing:-2px;box-shadow:0 0 30px rgba(56,214,232,.12)}h1{margin:0;font-size:clamp(1.25rem,3vw,1.75rem);letter-spacing:-.035em}.eyebrow{margin:3px 0 0;color:var(--muted);font:600 .72rem/1.2 ui-monospace,SFMono-Regular,Consolas,monospace;letter-spacing:.12em;text-transform:uppercase}.system-pills{display:flex;flex-wrap:wrap;justify-content:flex-end;gap:8px}.live,.battery{display:flex;align-items:center;gap:8px;padding:8px 12px;border:1px solid rgba(53,208,127,.25);border-radius:999px;background:rgba(53,208,127,.08);color:#9af1bf;font-size:.78rem;font-weight:800;letter-spacing:.08em}.live:before{content:"";width:8px;height:8px;border-radius:50%;background:var(--green);box-shadow:0 0 0 5px rgba(53,208,127,.1)}.battery{min-width:112px;justify-content:center;border-color:#355064;background:rgba(21,33,43,.8);color:#c9d7de}.battery.low{border-color:rgba(255,93,104,.55);background:rgba(255,93,104,.12);color:#ff9ca4}.battery.medium{border-color:rgba(246,173,60,.5);color:#ffd99c}.battery.charging{border-color:rgba(56,214,232,.5);color:#8becf5}
.grid{display:grid;grid-template-columns:minmax(0,1.52fr) minmax(300px,1fr);gap:18px}.card{overflow:hidden;border:1px solid var(--line);border-radius:var(--radius);background:linear-gradient(145deg,rgba(21,33,43,.96),rgba(14,23,31,.96));box-shadow:var(--shadow)}.card-head{display:flex;align-items:center;justify-content:space-between;padding:14px 16px;border-bottom:1px solid var(--line)}h2{margin:0;font-size:.78rem;letter-spacing:.1em;text-transform:uppercase}.tag{color:var(--muted);font:600 .7rem ui-monospace,SFMono-Regular,Consolas,monospace}.frame{position:relative;background:#030608}.frame:after{content:"";position:absolute;inset:0;pointer-events:none;box-shadow:inset 0 0 0 1px rgba(255,255,255,.03)}img{display:block;width:100%;height:auto;aspect-ratio:4/3;object-fit:contain;background:#030608}.map img{aspect-ratio:1/1}
.console{margin-top:18px;padding:16px 18px;border:1px solid var(--line);border-radius:var(--radius);background:rgba(7,12,17,.88);box-shadow:var(--shadow)}.console-head{display:flex;justify-content:space-between;align-items:center;margin-bottom:12px;color:var(--muted);font-size:.72rem;font-weight:800;letter-spacing:.1em;text-transform:uppercase}.console-head span:last-child{color:var(--cyan)}#status{min-height:9.5em;margin:0;overflow:auto;color:#c9d7de;font:500 .86rem/1.62 ui-monospace,SFMono-Regular,Consolas,monospace;white-space:pre-wrap}
.controls{position:sticky;bottom:12px;display:flex;flex-wrap:wrap;gap:10px;margin-top:18px;padding:12px;border:1px solid rgba(52,76,94,.88);border-radius:18px;background:rgba(12,19,26,.88);box-shadow:0 18px 50px rgba(0,0,0,.45);backdrop-filter:blur(14px)}button{min-height:46px;padding:0 20px;border:1px solid transparent;border-radius:12px;color:white;font:800 .8rem/1 inherit;letter-spacing:.07em;cursor:pointer;transition:transform .15s ease,filter .15s ease,box-shadow .15s ease}button:hover{filter:brightness(1.1);transform:translateY(-1px)}button:active{transform:translateY(1px)}button:focus-visible{outline:3px solid rgba(56,214,232,.34);outline-offset:2px}button:disabled{cursor:wait;filter:saturate(.35);opacity:.7;transform:none}.stop{background:#d63f4a;box-shadow:0 8px 20px rgba(214,63,74,.18)}.cancel-route{border-color:#9b6926;background:#76501d;color:#ffe0a8;box-shadow:0 8px 20px rgba(118,80,29,.2)}.go{background:#168a50;box-shadow:0 8px 20px rgba(22,138,80,.18)}.recover{border-color:#286f7b;background:#185664;color:#bdf8ff}.clear{margin-left:auto;border-color:#805c26;background:rgba(148,91,17,.42);color:#ffd99c}
.manual{display:grid;grid-template-columns:minmax(220px,320px) 1fr;gap:22px;align-items:center;margin-top:18px;padding:18px}.manual-copy h2{font-size:.9rem}.manual-copy p{margin:8px 0 0;color:var(--muted);font-size:.82rem;line-height:1.5}.drive-pad{display:grid;grid-template-columns:repeat(3,64px);grid-template-rows:repeat(3,54px);gap:8px;justify-content:center;touch-action:none;user-select:none}.drive{min-height:54px;padding:0;border-color:#355064;background:#1b2a36;color:#d9f8fb;font-size:1.15rem;box-shadow:inset 0 1px rgba(255,255,255,.05)}.drive:hover{background:#243b49}.drive.active{border-color:var(--cyan);background:#174854;box-shadow:0 0 0 3px rgba(56,214,232,.14)}.drive.turn-left{grid-column:1;grid-row:1}.drive.forward{grid-column:2;grid-row:1}.drive.turn-right{grid-column:3;grid-row:1}.drive.strafe-left{grid-column:1;grid-row:2}.drive-stop{grid-column:2;grid-row:2;border-color:#8d3c43;background:#622830;color:#fff}.drive.strafe-right{grid-column:3;grid-row:2}.drive.back{grid-column:2;grid-row:3}.key{display:block;margin-top:2px;color:var(--muted);font-size:.58rem;letter-spacing:.08em}
.speed-panel{margin-top:18px;padding:18px}.speed-grid{display:grid;grid-template-columns:1fr 1fr;gap:24px;margin-top:15px}.speed-control{display:grid;grid-template-columns:1fr auto;gap:8px 12px;align-items:center}.speed-control label{color:var(--muted);font-size:.78rem;font-weight:700;letter-spacing:.05em;text-transform:uppercase}.speed-control output{color:var(--cyan);font:800 .82rem ui-monospace,SFMono-Regular,Consolas,monospace}.speed-control input{grid-column:1/-1;width:100%;accent-color:var(--cyan);cursor:pointer}
.tabs{display:flex;gap:8px;margin:0 0 18px;padding:5px;border:1px solid var(--line);border-radius:14px;background:rgba(8,13,18,.7);width:max-content}.tab{min-height:38px;padding:0 18px;border-color:transparent;background:transparent;color:var(--muted)}.tab.active{background:var(--panel-2);color:var(--text);border-color:#355064}.pane[hidden]{display:none}.map img{cursor:crosshair;touch-action:none;user-select:none}.map img.dragging{cursor:grabbing}.map-tools{position:absolute;z-index:2;right:10px;bottom:10px;display:flex;gap:6px}.map-tools button{min-height:34px;padding:0 11px;border-color:#456275;background:rgba(12,25,34,.9);font-size:.68rem;box-shadow:0 3px 12px rgba(0,0,0,.35)}.map-tools .zoom{width:36px;padding:0;font-size:1.05rem}.map-hint{padding:9px 14px;border-top:1px solid var(--line);color:var(--muted);font-size:.75rem}.object-panel{padding:18px}.object-intro{display:flex;justify-content:space-between;gap:18px;align-items:flex-start;margin-bottom:16px}.object-intro p{margin:7px 0 0;color:var(--muted);font-size:.84rem}.object-count{padding:7px 10px;border:1px solid var(--line);border-radius:999px;color:var(--cyan);font-size:.72rem;font-weight:800}.object-list{display:grid;gap:10px}.object-row{display:grid;grid-template-columns:minmax(160px,1.2fr) minmax(120px,.8fr) auto;gap:14px;align-items:center;padding:14px;border:1px solid var(--line);border-radius:13px;background:rgba(7,12,17,.55)}.object-name{font-weight:800}.object-meta{margin-top:4px;color:var(--muted);font:500 .72rem/1.5 ui-monospace,SFMono-Regular,Consolas,monospace}.object-state{color:#9af1bf;font-size:.72rem;font-weight:800}.object-state.pending{color:#ffd99c}.object-actions{display:flex;flex-wrap:wrap;gap:7px}.object-actions button{min-height:38px;padding:0 12px;font-size:.68rem;background:#1b4f5a;border-color:#2b7180}.object-actions .rename{background:#263d55}.object-actions .remove{background:#5b2930;border-color:#813b45}.empty{padding:34px;text-align:center;border:1px dashed var(--line);border-radius:13px;color:var(--muted)}
@media(max-width:800px){.shell{width:min(100% - 20px,680px);padding-top:18px}.grid{grid-template-columns:1fr}.map{max-width:none}.topbar{align-items:flex-start}.controls{bottom:8px}.clear{margin-left:0}.object-row{grid-template-columns:1fr auto}.object-actions{grid-column:1/-1}}@media(max-width:600px){.manual{grid-template-columns:1fr}.manual-copy{text-align:center}.speed-grid{grid-template-columns:1fr}}@media(max-width:480px){.live{padding:7px 9px;font-size:.68rem}.controls button{flex:1;padding:0 12px}.clear{flex-basis:100%!important}.console{padding:14px}#status{font-size:.78rem}.tabs{width:100%}.tab{flex:1}.object-intro{flex-direction:column}.object-row{grid-template-columns:1fr}.object-actions,.object-state{grid-column:1}.object-actions button{flex:1}}
</style></head><body><main class='shell'><header class='topbar'><div class='brand'><div class='mark'>JH</div><div><h1>JetHexa Room Mapper</h1><p class='eyebrow'>Autonomous spatial intelligence</p></div></div><div class='system-pills'><div class='battery' id='battery' title='Waiting for battery telemetry'>BATTERY --%</div><div class='live'>SYSTEM LIVE</div></div></header><nav class='tabs' aria-label='Views'><button class='tab active' data-tab='dashboard'>DASHBOARD</button><button class='tab' data-tab='objects'>OBJECTS</button></nav><section class='pane' id='dashboard'>
<section class='grid'><article class='card cam'><div class='card-head'><h2>Camera + YOLO detections</h2><span class='tag'>RGB / LIVE</span></div><div class='frame'><img id='cam' src='/cam.jpg' alt='Live robot camera'></div></article>
<article class='card map'><div class='card-head'><h2>SLAM + landmarks</h2><span class='tag' id='map-mode'>ROBOT-CENTERED / 100%</span></div><div class='frame'><img id='map' src='/map.jpg' alt='Robot-centered clickable SLAM occupancy map'><div class='map-tools'><button class='zoom' id='zoom-out' aria-label='Zoom out'>−</button><button id='center-map'>CENTER</button><button class='zoom' id='zoom-in' aria-label='Zoom in'>+</button></div></div><div class='map-hint'>Black walls are LiDAR; magenta walls are temporary depth obstacles. Scroll or use +/− to zoom, drag to pan, Center to follow, or click to navigate.</div></article></section>
<section class='console'><div class='console-head'><span>Telemetry</span><span>● streaming</span></div><pre id='status'>connecting...</pre></section>
<section class='card speed-panel'><h2>Movement speed</h2><div class='speed-grid'><div class='speed-control'><label for='translation-speed'>Translation</label><output id='translation-value'>100%</output><input id='translation-speed' type='range' min='50' max='200' step='5' value='100'></div><div class='speed-control'><label for='rotation-speed'>Rotation</label><output id='rotation-value'>100%</output><input id='rotation-speed' type='range' min='50' max='200' step='5' value='100'></div></div></section>
<section class='card manual'><div class='manual-copy'><h2>Manual drive</h2><p>Available while autonomy is paused. Manual commands bypass map and obstacle gating; maintain direct sight of the robot. Releasing a key stops immediately. W/S move, A/D strafe, and Q/E rotate; arrow keys retain forward/back/rotate.</p></div><div class='drive-pad' aria-label='Manual drive controls'><button class='drive turn-left' data-drive='left' aria-label='Rotate left'>↶<span class='key'>Q / ←</span></button><button class='drive forward' data-drive='forward' aria-label='Move forward'>▲<span class='key'>W / ↑</span></button><button class='drive turn-right' data-drive='right' aria-label='Rotate right'>↷<span class='key'>E / →</span></button><button class='drive strafe-left' data-drive='strafe_left' aria-label='Strafe left'>◀<span class='key'>A</span></button><button class='drive drive-stop' data-stop aria-label='Stop'>■<span class='key'>STOP</span></button><button class='drive strafe-right' data-drive='strafe_right' aria-label='Strafe right'>▶<span class='key'>D</span></button><button class='drive back' data-drive='back' aria-label='Move backward'>▼<span class='key'>S / ↓</span></button></div></section>
<nav class='controls' aria-label='Robot controls'><button class='stop' onclick="post('/pause')">PAUSE AUTO</button><button class='cancel-route' onclick="cancelRoute()">CANCEL ROUTE</button><button class='go' onclick="post('/resume')">RESUME AUTO</button><button class='recover' onclick="recoverCamera(this)">RECOVER CAMERA</button><button class='clear' onclick="clearMap()">CLEAR MAP</button></nav></section><section class='pane card object-panel' id='objects' hidden><div class='object-intro'><div><h2>Detected objects</h2><p>Navigate to a safe stand-off point, rename a landmark, or remove it from this map.</p></div><span class='object-count' id='object-count'>0 OBJECTS</span></div><div class='object-list' id='object-list'><div class='empty'>Waiting for detected objects...</div></div></section>
<script>function fresh(id,ms){let x=document.getElementById(id),load=()=>x.src='/'+id+'.jpg?t='+Date.now();x.onload=x.onerror=()=>setTimeout(load,ms);load()}fresh('cam',150);
const mapImage=document.getElementById('map'),mapView={scale:50,dx:0,dy:0,rendered:null},MAP_BASE_SCALE=50,MAP_MIN_SCALE=15,MAP_MAX_SCALE=180;let mapTimer=null,mapDrag=null,mapDragged=false,mapObjectUrl=null,mapLoadSequence=0;
function mapUrl(){let q=new URLSearchParams({t:Date.now(),scale:mapView.scale.toFixed(3),dx:mapView.dx.toFixed(4),dy:mapView.dy.toFixed(4)});return '/map.jpg?'+q}
function updateMapMode(){let centered=Math.abs(mapView.dx)<.001&&Math.abs(mapView.dy)<.001;document.getElementById('map-mode').textContent=(centered?'ROBOT-CENTERED':'PANNED')+' / '+Math.round(mapView.scale/MAP_BASE_SCALE*100)+'%'}
async function loadMapNow(){if(mapTimer)clearTimeout(mapTimer);let sequence=++mapLoadSequence;updateMapMode();try{let response=await fetch(mapUrl());if(!response.ok)throw Error('HTTP '+response.status);let hx=response.headers.get('X-Map-Center-X'),hy=response.headers.get('X-Map-Center-Y'),hs=response.headers.get('X-Map-Scale');if(hx===null||hy===null||hs===null)throw Error('missing map coordinates');let rendered={cx:Number(hx),cy:Number(hy),scale:Number(hs)};if(!Number.isFinite(rendered.cx)||!Number.isFinite(rendered.cy)||!Number.isFinite(rendered.scale))throw Error('invalid map coordinates');let blob=await response.blob();if(sequence!==mapLoadSequence)return;let next=URL.createObjectURL(blob),previous=mapObjectUrl;mapImage.onload=()=>{if(sequence!==mapLoadSequence||mapImage.src!==next){URL.revokeObjectURL(next);return}mapObjectUrl=next;mapView.rendered=rendered;if(previous)URL.revokeObjectURL(previous);mapTimer=setTimeout(loadMapNow,800)};mapImage.onerror=()=>{URL.revokeObjectURL(next);mapTimer=setTimeout(loadMapNow,800)};mapView.rendered=null;mapImage.src=next}catch(e){show('Map unavailable: '+e.message);mapTimer=setTimeout(loadMapNow,800)}}loadMapNow();
function imagePoint(event){let r=mapImage.getBoundingClientRect();return{x:(event.clientX-r.left)*mapImage.naturalWidth/r.width,y:(event.clientY-r.top)*mapImage.naturalHeight/r.height}}
function setZoom(next,event){next=Math.max(MAP_MIN_SCALE,Math.min(MAP_MAX_SCALE,next));if(next===mapView.scale)return;if(event){let p=imagePoint(event),m=mapImage.naturalWidth/2;mapView.dx+=(m-p.y)*(1/mapView.scale-1/next);mapView.dy+=(m-p.x)*(1/mapView.scale-1/next)}mapView.scale=next;loadMapNow()}
document.getElementById('zoom-in').addEventListener('click',e=>{e.stopPropagation();setZoom(mapView.scale*1.3)});document.getElementById('zoom-out').addEventListener('click',e=>{e.stopPropagation();setZoom(mapView.scale/1.3)});document.getElementById('center-map').addEventListener('click',e=>{e.stopPropagation();mapView.scale=MAP_BASE_SCALE;mapView.dx=mapView.dy=0;loadMapNow()});
mapImage.addEventListener('wheel',e=>{e.preventDefault();setZoom(mapView.scale*(e.deltaY<0?1.18:1/1.18),e)},{passive:false});
mapImage.addEventListener('pointerdown',e=>{if(e.button!==0)return;mapDrag={id:e.pointerId,x:e.clientX,y:e.clientY,lastX:e.clientX,lastY:e.clientY};mapDragged=false;mapImage.setPointerCapture?.(e.pointerId)});
mapImage.addEventListener('pointermove',e=>{if(!mapDrag||mapDrag.id!==e.pointerId)return;let dx=e.clientX-mapDrag.lastX,dy=e.clientY-mapDrag.lastY,ratio=mapImage.naturalWidth/mapImage.getBoundingClientRect().width;if(Math.hypot(e.clientX-mapDrag.x,e.clientY-mapDrag.y)>5){mapDragged=true;mapImage.classList.add('dragging')}if(mapDragged){mapView.dx-=dy*ratio/mapView.scale;mapView.dy-=dx*ratio/mapView.scale}mapDrag.lastX=e.clientX;mapDrag.lastY=e.clientY});
function endMapDrag(e){if(!mapDrag||mapDrag.id!==e.pointerId)return;mapDrag=null;mapImage.classList.remove('dragging');if(mapDragged)loadMapNow()}mapImage.addEventListener('pointerup',endMapDrag);mapImage.addEventListener('pointercancel',endMapDrag);
function show(t){document.getElementById('status').textContent=t}
function refreshBattery(){fetch('/battery.json').then(r=>{if(!r.ok)throw Error('HTTP '+r.status);return r.json()}).then(b=>{let el=document.getElementById('battery');el.className='battery';if(!b.available||b.percentage===null){el.textContent='BATTERY --%';el.title=b.source||'Waiting for battery telemetry';return}let pct=Math.round(b.percentage);el.textContent=(b.charging?'CHARGING ':'BATTERY ')+pct+'%';if(b.charging)el.classList.add('charging');else if(pct<=20)el.classList.add('low');else if(pct<=40)el.classList.add('medium');el.title=(b.voltage===null?'':b.voltage.toFixed(2)+' V | ')+(b.source||'battery telemetry')}).catch(()=>{let el=document.getElementById('battery');el.className='battery';el.textContent='BATTERY --%'})}refreshBattery();setInterval(refreshBattery,2000);
function post(u){fetch(u,{method:'POST'}).then(async r=>{let t=await r.text();if(!r.ok)throw Error(t);show(t)}).catch(e=>show('Web request failed: '+e.message))}
async function recoverCamera(button){button.disabled=true;let label=button.textContent;button.textContent='RECOVERING...';show('Camera recovery running; this can take up to two minutes.');try{let r=await fetch('/recover-camera',{method:'POST'}),t=await r.text();if(!r.ok)throw Error(t);show(t)}catch(e){show('Camera recovery failed: '+e.message)}finally{button.disabled=false;button.textContent=label}}
const translationSpeed=document.getElementById('translation-speed'),rotationSpeed=document.getElementById('rotation-speed');let speedTimer=null;
function showSpeed(){document.getElementById('translation-value').textContent=translationSpeed.value+'%';document.getElementById('rotation-value').textContent=rotationSpeed.value+'%'}
function sendSpeed(){showSpeed();if(speedTimer)clearTimeout(speedTimer);speedTimer=setTimeout(()=>{let q=new URLSearchParams({translation:translationSpeed.value,rotation:rotationSpeed.value});post('/speed?'+q)},120)}
[translationSpeed,rotationSpeed].forEach(slider=>slider.addEventListener('input',sendSpeed));fetch('/speed.json').then(r=>r.json()).then(s=>{translationSpeed.value=s.translation;rotationSpeed.value=s.rotation;translationSpeed.min=rotationSpeed.min=s.minimum;translationSpeed.max=rotationSpeed.max=s.maximum;showSpeed()}).catch(()=>showSpeed());
function clearMap(){if(confirm('Pause the robot and permanently clear the current map and object landmarks?'))post('/clear-map')}
function cancelRoute(){if(confirm('Cancel the active route and leave autonomy paused?'))post('/cancel-route')}
document.querySelectorAll('[data-tab]').forEach(button=>button.addEventListener('click',()=>{document.querySelectorAll('[data-tab]').forEach(item=>item.classList.toggle('active',item===button));document.querySelectorAll('.pane').forEach(pane=>pane.hidden=pane.id!==button.dataset.tab)}));
mapImage.addEventListener('click',event=>{if(mapDragged){mapDragged=false;return}let p=imagePoint(event),v=mapView.rendered;if(!v){show('Map coordinates are not ready yet');return}let q=new URLSearchParams({px:p.x.toFixed(2),py:p.y.toFixed(2),scale:v.scale.toFixed(3),cx:v.cx.toFixed(6),cy:v.cy.toFixed(6)});post('/navigate?'+q)});
function objectAction(action,id,name){let url='/object/'+action+'?id='+encodeURIComponent(id);if(action==='rename'){let next=prompt('New object name:',name);if(next===null)return;url+='&name='+encodeURIComponent(next)}if(action==='remove'&&!confirm('Remove '+name+' from this map and suppress it at this location?'))return;post(url)}
function renderObjects(items){let list=document.getElementById('object-list'),count=document.getElementById('object-count');count.textContent=items.length+' OBJECT'+(items.length===1?'':'S');list.replaceChildren();if(!items.length){let empty=document.createElement('div');empty.className='empty';empty.textContent='No mapped detections yet.';list.appendChild(empty);return}items.forEach(item=>{let row=document.createElement('article');row.className='object-row';let info=document.createElement('div'),name=document.createElement('div'),meta=document.createElement('div');name.className='object-name';name.textContent=item.name||item.label;meta.className='object-meta';meta.textContent=item.label+' class | '+item.count+' observation'+(item.count===1?'':'s')+' | '+item.x.toFixed(2)+', '+item.y.toFixed(2)+' m';info.append(name,meta);let state=document.createElement('div');let confirmed=item.confirmed!==false;state.className='object-state'+(confirmed?'':' pending');state.textContent=confirmed?'CONFIRMED':'PENDING';let actions=document.createElement('div');actions.className='object-actions';[['navigate','NAVIGATE'],['rename','RENAME'],['remove','REMOVE']].forEach(pair=>{let button=document.createElement('button');button.className=pair[0];button.textContent=pair[1];button.addEventListener('click',()=>objectAction(pair[0],item.id,item.name||item.label));actions.appendChild(button)});row.append(info,state,actions);list.appendChild(row)})}
function refreshObjects(){fetch('/objects.json').then(r=>{if(!r.ok)throw Error('HTTP '+r.status);return r.json()}).then(renderObjects).catch(e=>{let list=document.getElementById('object-list');list.textContent='Object list unavailable: '+e.message})}refreshObjects();setInterval(refreshObjects,1500);
let driveTimer=null,driveCommand=null;
function drivePulse(command){fetch('/manual?command='+encodeURIComponent(command),{method:'POST'}).catch(e=>show('Manual control failed: '+e.message))}
function driveStart(command){if(driveCommand===command)return;if(driveTimer)clearInterval(driveTimer);if(driveCommand)document.querySelector(`[data-drive="${driveCommand}"]`)?.classList.remove('active');driveCommand=command;document.querySelector(`[data-drive="${command}"]`)?.classList.add('active');drivePulse(command);driveTimer=setInterval(()=>drivePulse(command),180)}
function driveStop(){if(driveTimer)clearInterval(driveTimer);driveTimer=null;let wasDriving=driveCommand;if(wasDriving)document.querySelector(`[data-drive="${wasDriving}"]`)?.classList.remove('active');driveCommand=null;if(wasDriving)drivePulse('stop')}
document.querySelectorAll('[data-drive]').forEach(b=>{b.addEventListener('pointerdown',e=>{e.preventDefault();b.setPointerCapture?.(e.pointerId);driveStart(b.dataset.drive)});b.addEventListener('pointerup',driveStop);b.addEventListener('pointercancel',driveStop);b.addEventListener('lostpointercapture',driveStop)});document.querySelector('[data-stop]').addEventListener('click',driveStop);
const driveKeys={w:'forward',arrowup:'forward',s:'back',arrowdown:'back',a:'strafe_left',d:'strafe_right',q:'left',arrowleft:'left',e:'right',arrowright:'right'};addEventListener('keydown',e=>{let c=driveKeys[e.key.toLowerCase()];if(c&&!e.repeat){e.preventDefault();driveStart(c)}});addEventListener('keyup',e=>{if(driveKeys[e.key.toLowerCase()]){e.preventDefault();driveStop()}});addEventListener('blur',driveStop);document.addEventListener('visibilitychange',()=>{if(document.hidden)driveStop()});
setInterval(()=>fetch('/status').then(r=>{if(!r.ok)throw Error('HTTP '+r.status);return r.text()}).then(show).catch(e=>show('Web server unavailable: '+e.message)),1000);</script></main></body></html>"""

MAP_SIZE, MAP_SCALE = 420, 50.0
DEPTH_OBSTACLE_COLOR = (180, 0, 220)  # BGR: magenta in the encoded map
CAMERA_RECOVERY_LOCK = threading.Lock()


def recover_camera():
    """Run the fixed RGB recovery helper and return its combined output."""
    if not CAMERA_RECOVERY_LOCK.acquire(False):
        return False, 'Camera recovery is already running', 409
    try:
        script = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              'recover_rgb_camera.sh')
        if not os.path.isfile(script):
            return False, 'Camera recovery script is missing', 500
        try:
            result = subprocess.run(
                ['/bin/bash', script], cwd=os.path.dirname(script),
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                universal_newlines=True, timeout=135)
        except subprocess.TimeoutExpired:
            return False, 'Camera recovery timed out after 135 seconds', 504
        output = (result.stdout or '').strip()
        if result.returncode:
            return False, output or 'Camera recovery script failed', 500
        return True, output or 'RGB recovery succeeded', 200
    finally:
        CAMERA_RECOVERY_LOCK.release()


def jpeg(img, quality=75):
    ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    return buf.tobytes() if ok else b''


def draw_camera(node):
    frame = node.camera
    if frame is None:
        img = np.zeros((480, 640, 3), np.uint8)
        cv2.putText(img, 'waiting for camera...', (170, 240), cv2.FONT_HERSHEY_SIMPLEX, .8, (255,255,255), 2)
        return img
    source_h, source_w = frame.shape[:2]
    img = cv2.resize(frame, (640, 480)).copy()
    v = node.view
    color = (40, 40, 235) if v.get('blocked') else (60, 200, 60)
    label = 'BLOCKED' if v.get('blocked') else 'clear'
    depth = v.get('depth', 99.0)
    depth_text = 'depth %.2f m (%s)' % (depth, v.get('depth_source', 'clear')) if depth < 99 else 'depth clear'
    if not node.use_depth:
        depth_text = 'depth disabled'
    elif time.time() - node.t_depth > 1.0:
        depth_text = 'depth stale / waiting'
    elif node.floor_profile is None:
        depth_text = 'learning floor'
    elif not node.depth_health.startswith('ready'):
        depth_text = 'depth unknown'
    cv2.putText(img, 'LiDAR %.2f m | %s - %s' % (v.get('front', 0), depth_text, label), (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX, .65, color, 2)
    if node.paused:
        label = 'AUTO PAUSED'
        font, scale, thickness = cv2.FONT_HERSHEY_SIMPLEX, .72, 2
        (text_w, text_h), baseline = cv2.getTextSize(label, font, scale, thickness)
        x, y = img.shape[1] - text_w - 18, 14 + text_h
        cv2.rectangle(img, (x - 8, 7), (img.shape[1] - 8, y + baseline + 7),
                      (28, 28, 155), -1)
        cv2.putText(img, label, (x, y), font, scale, (255,255,255), thickness,
                    cv2.LINE_AA)
    with node.object_lock:
        detections = [dict(d) for d in node.object_detections]
    sx, sy = 640.0 / source_w, 480.0 / source_h
    for detection in detections:
        x1, y1, x2, y2 = detection['box']
        p1, p2 = (int(x1*sx), int(y1*sy)), (int(x2*sx), int(y2*sy))
        mapped = detection.get('map_point') is not None
        box_color = (60, 210, 60) if mapped else (0, 180, 255)
        cv2.rectangle(img, p1, p2, box_color, 2)
        text = '%s %.0f%%' % (detection['label'], 100*detection['confidence'])
        if detection.get('depth') is not None:
            text += ' %.1fm' % detection['depth']
        cv2.putText(img, text, (p1[0], max(42, p1[1]-5)),
                    cv2.FONT_HERSHEY_SIMPLEX, .5, box_color, 2)
    return img


def to_screen(x, y, cx, cy, scale=MAP_SCALE):
    return (int(MAP_SIZE/2 - (y-cy)*scale), int(MAP_SIZE/2 - (x-cx)*scale))


def map_view_center(node):
    """Center the moving web viewport on the latest robot position."""
    pose = getattr(node, 'pose', None)
    if pose is not None:
        return float(pose[0]), float(pose[1])
    return getattr(node, 'web_map_center', None) or (0.0, 0.0)


def draw_map(node, scale=MAP_SCALE, dx=0.0, dy=0.0, center=None):
    e, pose = node.explorer, node.pose
    # The occupancy data remains in world coordinates while this viewport
    # pans, keeping the robot fixed at the visual center.
    if center is None:
        cx, cy = map_view_center(node)
        cx, cy = cx + dx, cy + dy
    else:
        cx, cy = float(center[0]), float(center[1])
    with node.map_lock:
        depth_mask_fn = getattr(e, 'depth_obstacle_mask', None)
        depth_mask = (depth_mask_fn() if callable(depth_mask_fn)
                      else np.zeros_like(e.state, dtype=bool))
        state = e.state.copy()
        slam_state = getattr(e, 'slam_state', state).copy()
        path = list(node.path)
    yy, xx = np.indices((MAP_SIZE, MAP_SIZE))
    wx = cx + (MAP_SIZE/2 - yy)/scale
    wy = cy + (MAP_SIZE/2 - xx)/scale
    raw_r = ((wy+e.half)/e.cell).astype(int)
    raw_c = ((wx+e.half)/e.cell).astype(int)
    inside = (raw_r >= 0) & (raw_r < e.n) & (raw_c >= 0) & (raw_c < e.n)
    r = np.clip(raw_r, 0, e.n-1)
    c = np.clip(raw_c, 0, e.n-1)
    img = np.full((MAP_SIZE, MAP_SIZE, 3), 90, np.uint8)
    img[inside & (state[r,c] == 1)] = (255,255,255)
    img[inside & (state[r,c] == 2)] = (0,0,0)
    depth_only = inside & depth_mask[r,c] & (slam_state[r,c] != 2)
    img[depth_only] = DEPTH_OBSTACLE_COLOR
    if path:
        pts = np.array([to_screen(x,y,cx,cy,scale) for x,y in path], np.int32)
        cv2.polylines(img, [pts], False, (230,140,40), 2)
    if pose:
        x,y,yaw = pose; center = to_screen(x,y,cx,cy,scale)
        touching = node.view.get('nearest', 99) < node.body_radius + .02
        outline = (40,40,235) if touching else (255,255,255)
        # A filled, asymmetric body marker makes rotation visible even when
        # the robot has barely translated.  The long nose points along SLAM's
        # world-frame yaw; the map itself remains fixed in the initial frame.
        nose = to_screen(x+.42*math.cos(yaw), y+.42*math.sin(yaw), cx, cy, scale)
        left = to_screen(x+.22*math.cos(yaw+2.35), y+.22*math.sin(yaw+2.35), cx, cy, scale)
        right = to_screen(x+.22*math.cos(yaw-2.35), y+.22*math.sin(yaw-2.35), cx, cy, scale)
        marker = np.array([nose, left, right], np.int32)
        cv2.fillConvexPoly(img, marker, (210,90,20))
        cv2.polylines(img, [marker], True, outline, 2)
        cv2.circle(img, center, max(1, int(round(node.body_radius * scale))),
                   (30, 210, 230), 1, cv2.LINE_AA)
        cv2.circle(img, center, 3, outline, -1)
        heading = 'Pose %.2f, %.2f m   heading %.1f deg' % (
            x, y, math.degrees(yaw))
        cv2.rectangle(img, (5, 5), (365, 28), (45,45,45), -1)
        cv2.putText(img, heading, (10, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    .48, (255,255,255), 1, cv2.LINE_AA)
    with node.object_lock:
        objects = node.object_tracker.snapshot(True)
    for obj in objects:
        point = to_screen(obj['x'], obj['y'], cx, cy)
        if 0 <= point[0] < MAP_SIZE and 0 <= point[1] < MAP_SIZE:
            cv2.circle(img, point, 6, (0, 165, 255), -1)
            label = '%s #%d' % (obj.get('name', obj['label']), obj['id'])
            cv2.putText(img, label, (point[0]+8, point[1]-5),
                        cv2.FONT_HERSHEY_SIMPLEX, .42, (0, 100, 220), 1)
    requested = getattr(node, 'user_goal', None)
    actual = getattr(node, 'nav_goal', None)
    if requested is not None:
        cv2.drawMarker(img, to_screen(requested[0], requested[1], cx, cy),
                       (210, 0, 210), cv2.MARKER_TILTED_CROSS, 16, 2)
    if actual is not None:
        cv2.drawMarker(img, to_screen(actual[0], actual[1], cx, cy),
                       (255, 200, 20), cv2.MARKER_CROSS, 18, 2)
    cv2.line(img, (10, MAP_SIZE-12), (60, MAP_SIZE-12), (0,0,0), 3)
    cv2.putText(img, '1 m', (12, MAP_SIZE-20), cv2.FONT_HERSHEY_SIMPLEX, .45, (255,255,255), 1)
    return img


class Server(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    # SO_REUSEADDR has different semantics on Windows and can let two servers
    # bind the same port. Use an exclusive bind there so fallback works.
    allow_reuse_address = not hasattr(socket, 'SO_EXCLUSIVEADDRUSE')

    def server_bind(self):
        if hasattr(socket, 'SO_EXCLUSIVEADDRUSE'):
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()


def start(node, port=8000):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass
        def reply(self, body, content_type='text/plain; charset=utf-8', status=200,
                  headers=None):
            if isinstance(body, str): body = body.encode()
            self.send_response(status); self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body))); self.send_header('Cache-Control', 'no-store')
            for name, value in (headers or {}).items():
                self.send_header(name, str(value))
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                # Browsers may cancel an old JPEG request when a refresh starts.
                pass
        def do_GET(self):
            path = urlparse(self.path).path
            if path == '/': self.reply(PAGE, 'text/html; charset=utf-8')
            elif path == '/cam.jpg': self.reply(jpeg(draw_camera(node)), 'image/jpeg')
            elif path == '/map.jpg':
                values = parse_qs(urlparse(self.path).query)
                try:
                    scale = float(values.get('scale', [MAP_SCALE])[0])
                    dx = float(values.get('dx', [0.0])[0])
                    dy = float(values.get('dy', [0.0])[0])
                except (TypeError, ValueError):
                    self.reply('Invalid map view', status=400); return
                if not (15.0 <= scale <= 180.0 and math.isfinite(dx) and
                        math.isfinite(dy) and abs(dx) <= 50.0 and abs(dy) <= 50.0):
                    self.reply('Invalid map view', status=400); return
                base_cx, base_cy = map_view_center(node)
                cx, cy = base_cx + dx, base_cy + dy
                headers = {'X-Map-Center-X': '%.9f' % cx,
                           'X-Map-Center-Y': '%.9f' % cy,
                           'X-Map-Scale': '%.9f' % scale}
                self.reply(jpeg(draw_map(node, scale, center=(cx, cy)), 85),
                           'image/jpeg', headers=headers)
            elif path == '/status': self.reply(node.web_status())
            elif path == '/battery.json':
                self.reply(json.dumps(node.web_battery()), 'application/json')
            elif path == '/speed.json':
                self.reply(json.dumps(node.web_speed()), 'application/json')
            elif path == '/objects.json':
                with node.object_lock:
                    objects = node.object_tracker.snapshot(False)
                    for item in objects:
                        item['confirmed'] = item['count'] >= node.object_tracker.min_observations
                self.reply(json.dumps(objects), 'application/json')
            else: self.send_error(404)
        def do_POST(self):
            u = urlparse(self.path)
            if u.path in ('/pause','/resume','/clear-map','/cancel-route'):
                command = {'/clear-map': 'clear_map', '/cancel-route': 'cancel_route'}.get(
                    u.path, u.path[1:])
                node.web_request((command,)); self.reply('Got it: ' + command)
            elif u.path == '/recover-camera':
                if not node.paused:
                    self.reply('Pause autonomy before recovering the camera', status=409)
                    return
                _, message, status = recover_camera()
                self.reply(message, status=status)
            elif u.path == '/manual':
                command = parse_qs(u.query).get('command', [''])[0]
                if command not in ('forward', 'back', 'strafe_left', 'strafe_right',
                                    'left', 'right', 'stop'):
                    self.reply('Invalid manual command', status=400)
                    return
                node.web_request(('manual', command))
                self.reply('Manual: ' + command)
            elif u.path == '/speed':
                values = parse_qs(u.query)
                try:
                    translation = float(values['translation'][0])
                    rotation = float(values['rotation'][0])
                except (KeyError, IndexError, TypeError, ValueError):
                    self.reply('Invalid speed', status=400); return
                speed = node.web_speed()
                if (not math.isfinite(translation) or not math.isfinite(rotation) or
                        not speed['minimum'] <= translation <= speed['maximum'] or
                        not speed['minimum'] <= rotation <= speed['maximum']):
                    self.reply('Speed must be between %.0f%% and %.0f%%' % (
                        speed['minimum'], speed['maximum']), status=400); return
                node.web_request(('speed', translation, rotation))
                self.reply('Speed: translation %.0f%%, rotation %.0f%%' % (
                    translation, rotation))
            elif u.path == '/navigate':
                values = parse_qs(u.query)
                try:
                    px, py = float(values['px'][0]), float(values['py'][0])
                    scale = float(values.get('scale', [MAP_SCALE])[0])
                    supplied_center = 'cx' in values or 'cy' in values
                    if supplied_center:
                        cx = float(values['cx'][0])
                        cy = float(values['cy'][0])
                    else:
                        dx = float(values.get('dx', [0.0])[0])
                        dy = float(values.get('dy', [0.0])[0])
                except (KeyError, IndexError, TypeError, ValueError):
                    self.reply('Invalid map position', status=400); return
                if not (math.isfinite(px) and math.isfinite(py) and
                        0 <= px <= MAP_SIZE and 0 <= py <= MAP_SIZE and
                         15.0 <= scale <= 180.0 and
                         (not supplied_center or (math.isfinite(cx) and math.isfinite(cy) and
                                                  abs(cx) <= 1000.0 and abs(cy) <= 1000.0)) and
                         (supplied_center or (math.isfinite(dx) and math.isfinite(dy) and
                                              abs(dx) <= 50.0 and abs(dy) <= 50.0))):
                    self.reply('Map position is outside the viewport', status=400); return
                if not supplied_center:
                    cx, cy = map_view_center(node)
                    cx, cy = cx + dx, cy + dy
                x = cx + (MAP_SIZE/2.0 - py) / scale
                y = cy + (MAP_SIZE/2.0 - px) / scale
                node.web_request(('navigate', x, y))
                self.reply('Navigation requested: %.2f, %.2f m' % (x, y))
            elif u.path in ('/object/navigate', '/object/remove', '/object/rename'):
                values = parse_qs(u.query)
                try:
                    object_id = int(values['id'][0])
                except (KeyError, IndexError, TypeError, ValueError):
                    self.reply('Invalid object id', status=400); return
                if object_id < 1:
                    self.reply('Invalid object id', status=400); return
                if u.path == '/object/navigate':
                    node.web_request(('navigate_object', object_id))
                    self.reply('Object navigation requested')
                elif u.path == '/object/remove':
                    node.web_request(('remove_object', object_id))
                    self.reply('Object removal requested')
                else:
                    name = values.get('name', [''])[0].strip()
                    if not name or len(name) > 40 or any(ord(ch) < 32 for ch in name):
                        self.reply('Name must be 1-40 printable characters', status=400); return
                    node.web_request(('rename_object', object_id, name))
                    self.reply('Object rename requested')
            else: self.send_error(404)
    if not isinstance(port, int) or not 0 <= port <= 65535:
        raise ValueError('port must be an integer between 0 and 65535')
    ports = (0,) if port == 0 else range(port, min(port + 10, 65536))
    for candidate in ports:
        try: server = Server(('0.0.0.0', candidate), Handler)
        except OSError: continue
        threading.Thread(target=server.serve_forever, daemon=True).start()
        return server.server_address[1]
    return None
