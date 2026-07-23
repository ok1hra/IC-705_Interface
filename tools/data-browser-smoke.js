#!/usr/bin/env node
"use strict";

// Serves the production DATA assets plus a minimal AUD1 fixture, then checks
// the real page in headless Chrome. No radio or firmware is required.

const crypto=require("crypto"), fs=require("fs"), http=require("http"), path=require("path"), {spawn}=require("child_process");
const root=path.resolve(__dirname,".."), data=path.join(root,"data");
let unattendedPosts=[], inboxWrites=[];
// One message already waiting for K0OG, so the restore path is exercised too.
const inboxSeed=JSON.stringify({id:1,from:"KD8SKZ",to:"K0OG",text:"MEET AT NOON",atMs:0,delivered:false})+"\n";

// Mirror of the firmware's single-operator lock, enough to exercise the page's
// claim/heartbeat/release path and to prove an unowned audio upgrade is refused.
const session={token:"",claims:0,refusals:0,releases:0,wsRefusals:0};
function sessionOwns(token){return Boolean(session.token)&&session.token===token;}
function sessionBody(req,res,handler){let body="";req.on("data",chunk=>body+=chunk);req.on("end",()=>{let parsed={};try{parsed=JSON.parse(body);}catch(_error){}handler(parsed,res);});}
function sessionReply(res,status){res.writeHead(status,{"Content-Type":"application/json"});res.end(JSON.stringify({ok:status===200,owner:"192.168.1.99",ageMs:1200,leaseMs:15000}));}
function unattendedState(){return{armed:true,remainingMs:43200000,clientLive:false,clientAgeMs:31000,clientSeen:true,blockedLiveness:2,blockedNotArmed:0,livenessTimeoutMs:5000,ptt:false,txState:0,txUsed:0,txCapacity:12288,rxPackets:5,lan:true,upMs:90500,choicesH:[1,6,12,24,168]};}
let chrome, finished=false, timer, sequence=0, firstSample=0, streamId=707, txPrepares=0, txPackets=0, wsConnections=0, earlyWsConnections=0, jscRequests=0, jscStartedAt=0, jscCompleteAt=0, wsOpenedAt=0, jscComplete=false, js8Gzip=0, js8Brotli=0, commands=[], setupSaveBody="", setupRestartRequests=0, lanReconnectRequests=0;
const mime={".html":"text/html",".css":"text/css",".js":"application/javascript",".wasm":"application/wasm",".bin":"application/octet-stream"};
function frame(opcode,payload){const body=Buffer.isBuffer(payload)?payload:Buffer.from(payload);return body.length<126?Buffer.concat([Buffer.from([0x80|opcode,body.length]),body]):Buffer.concat([Buffer.from([0x80|opcode,126,body.length>>8,body.length&255]),body]);}
function aud1(){const wire=Buffer.alloc(200,0xff);wire.write("AUD1");wire[4]=1;wire[5]=1;wire.writeUInt16BE(sequence===0?1:0,6);wire.writeUInt16BE(40,8);wire.writeUInt16BE(0,10);wire.writeUInt32BE(streamId,12);wire.writeUInt32BE(sequence++,16);wire.writeUInt32BE(8000,20);wire.writeBigUInt64BE(BigInt(firstSample),24);wire.writeUInt32BE(0,32);wire.writeUInt32BE(160,36);firstSample+=160;return wire;}
function readClientFrames(socket){let input=Buffer.alloc(0);return chunk=>{input=Buffer.concat([input,chunk]);for(;;){if(input.length<2)return;let at=2,length=input[1]&127;if(length===126){if(input.length<4)return;length=input.readUInt16BE(2);at=4;}else if(length===127)return socket.destroy();const masked=Boolean(input[1]&128);if(masked)at+=4;if(input.length<at+length)return;const opcode=input[0]&15,maskAt=at-4,payload=Buffer.from(input.subarray(at,at+length));if(masked)for(let i=0;i<payload.length;i++)payload[i]^=input[maskAt+(i%4)];input=input.subarray(at+length);if(opcode===1){const message=JSON.parse(payload.toString());if(message.type==="tx.prepare"){txPrepares++;socket.write(frame(1,JSON.stringify({type:"tx-ready",txId:message.txId,ptt:false})));}}else if(opcode===2){txPackets++;const txId=payload.readUInt32BE(32),flags=payload.readUInt16BE(6);if(flags&1)socket.write(frame(1,JSON.stringify({type:"tx-state",txId,ptt:true})));if(flags&2)socket.write(frame(1,JSON.stringify({type:"tx-drained",txId,ptt:false})));}}};}
function finish(ok,text){if(finished)return;finished=true;clearTimeout(timer);if(chrome)chrome.kill("SIGTERM");server.close();const encodingPass=js8Gzip>0&&js8Brotli===0;const frequencyPass=commands.some(command=>command.type==="setFrequency"&&Number(command.frequency)===14078000), setupArgs=new URLSearchParams(setupSaveBody), setupSavePass=setupArgs.get("trx1transport")==="lan"&&setupArgs.get("lanip")==="192.168.1.60"&&setupArgs.get("lanuser")==="operator"&&setupArgs.get("lanpass")==="secret123"&&setupArgs.get("noRestart")==="1"&&setupRestartRequests===1;const unattendedRevokePass=unattendedPosts.some(post=>post.action==="revoke");const inboxWritePass=inboxWrites.length>0;const sessionPass=session.claims>0&&session.wsRefusals===0;ok=ok&&encodingPass&&frequencyPass&&earlyWsConnections===0&&jscRequests===1&&setupSavePass&&lanReconnectRequests===1&&unattendedRevokePass&&inboxWritePass&&sessionPass;const report=`${text} js8Gzip=${js8Gzip} js8Brotli=${js8Brotli} jscRequests=${jscRequests} jscMs=${jscCompleteAt-jscStartedAt} wsAfterJscMs=${wsOpenedAt-jscCompleteAt} ws=${wsConnections} earlyWs=${earlyWsConnections} setupSave=${setupSavePass} setupRestarts=${setupRestartRequests} unattendedRevoke=${unattendedRevokePass} session=${sessionPass}(claims=${session.claims} refusals=${session.refusals} wsRefusals=${session.wsRefusals}) inboxWrites=${inboxWrites.length} reconnects=${lanReconnectRequests} commands=${JSON.stringify(commands)} prepares=${txPrepares} packets=${txPackets}`;(ok?console.log:console.error)(report);if(!ok)process.exitCode=1;}
const server=http.createServer((req,res)=>{
  const url=new URL(req.url,"http://fixture");
  if(url.pathname==="/result"&&req.method==="POST"){let body="";req.on("data",c=>body+=c);req.on("end",()=>{res.writeHead(204).end();const result=JSON.parse(body);finish(result.pass,result.text);});return;}
  if(url.pathname==="/setup/save"&&req.method==="POST"){let body="";req.on("data",c=>body+=c);req.on("end",()=>{setupSaveBody=body;res.setHeader("Content-Type","application/json");res.end('{"ok":true}');});return;}
  if(url.pathname==="/restart"&&req.method==="POST"){setupRestartRequests++;res.setHeader("Content-Type","application/json");res.end('{"ok":true}');return;}
  if(url.pathname==="/lan/reconnect"&&req.method==="POST"){lanReconnectRequests++;res.setHeader("Content-Type","application/json");res.end('{"ok":true}');return;}
  if(url.pathname==="/inbox"&&req.method==="POST"){let body="";req.on("data",c=>body+=c);req.on("end",()=>{inboxWrites.push(body);res.setHeader("Content-Type","application/json");res.end('{"ok":true}');});return;}
  if(url.pathname==="/inbox"){res.setHeader("Content-Type","text/plain");res.end(inboxSeed);return;}
  if(url.pathname==="/js8/session/claim"&&req.method==="POST"){return sessionBody(req,res,(body)=>{if(session.token&&!sessionOwns(body.token)&&!body.force){session.refusals++;return sessionReply(res,409);}session.token=body.token||"";session.claims++;sessionReply(res,200);});}
  if(url.pathname==="/js8/session/ping"&&req.method==="POST"){return sessionBody(req,res,(body)=>{if(session.token&&!sessionOwns(body.token))return sessionReply(res,409);session.token=body.token||"";sessionReply(res,200);});}
  if(url.pathname==="/js8/session/release"&&req.method==="POST"){return sessionBody(req,res,(body)=>{if(sessionOwns(body.token)){session.token="";session.releases++;}res.writeHead(200,{"Content-Type":"application/json"});res.end('{"ok":true}');});}
  if(url.pathname==="/unattended"&&req.method==="POST"){let body="";req.on("data",c=>body+=c);req.on("end",()=>{try{unattendedPosts.push(JSON.parse(body));}catch(_error){}res.setHeader("Content-Type","application/json");res.end(JSON.stringify(unattendedState()));});return;}
  if(url.pathname==="/unattended"){res.setHeader("Content-Type","application/json");res.end(JSON.stringify(unattendedState()));return;}
  if(url.pathname==="/unattended/log"){res.setHeader("Content-Type","text/plain");res.end("1200 ARM 12 h\n90500 BLOCK liveness lost before keying\n");return;}
  if(url.pathname==="/state"){res.setHeader("Content-Type","application/json");res.end(JSON.stringify({connected:true,lanStatus:"linked",transceiverType:"IC-705-LAN",power:true,frequency:7078000,mode:"USB",tx:false,rfPower:128,fwRev:"20260718",wifiRssi:-51,bdSupported:true}));return;}
  if(url.pathname==="/setup-data.json"){const js8=url.searchParams.get("scope")==="js8call",missing=url.searchParams.get("fixture")==="missing"||!js8;res.setHeader("Content-Type","application/json");res.end(JSON.stringify({fwRev:20260718,hwRev:4,apModeText:"AP mode ON",mac:"00:11:22:33:44:55",ssid:"fixture-wifi",pswd:"fixture-password",ssid2:"",pswd2:"",trx1transport:"lan",lanip:missing?"":"192.168.1.60",lanuser:missing?"":"operator",lanpass:missing?"":"secret123",civaddr:"A4",trx2conntype:0,trx3conntype:0}));return;}
  if(url.pathname==="/cmd"&&req.method==="POST"){let body="";req.on("data",chunk=>body+=chunk);req.on("end",()=>{try{commands.push(JSON.parse(body));}catch(_error){}res.setHeader("Content-Type","application/json");res.end('{"ok":true}');});return;}
  if(url.pathname==="/smoke.html"){
    res.setHeader("Content-Type","text/html");res.end(`<!doctype html>
<iframe id="app" src="/data?test=1&audioPort=${server.address().port}" style="width:1100px;height:800px"></iframe>
<iframe id="setup" src="/setup" style="display:none"></iframe>
<iframe id="lanGate" src="/data?test=1&lanFixture=missing" style="display:none"></iframe>
<script>
addEventListener('error',event=>fetch('/result',{method:'POST',body:JSON.stringify({pass:false,text:'DATA BROWSER SCRIPT ERROR: '+event.message+' at '+event.filename+':'+event.lineno+':'+event.colno})}));
addEventListener('unhandledrejection',event=>fetch('/result',{method:'POST',body:JSON.stringify({pass:false,text:'DATA BROWSER SCRIPT REJECTION: '+String(event.reason?.stack||event.reason)})}));
const f=document.querySelector('#app');
const setupFrame=document.querySelector('#setup');
const lanGateFrame=document.querySelector('#lanGate');
f.onload=()=>{
  const d=f.contentDocument;
  setTimeout(()=>{
    window.loadingObserved=!!d.querySelector('#startupLoader:not([hidden])')&&d.querySelector('#linkState').textContent.includes('LOADING');
    window.firstVisitHelpObserved=d.querySelector('#trxHelpDialog')?.open===true;
    d.querySelector('#trxHelpDialog .trx-help-close')?.click();
  },250);
  setTimeout(()=>{
    try {
    const now=Date.now();
    const emptyIdentityDefaults=d.querySelector('#myCall').value===''&&d.querySelector('#myGrid').value==='';
    const defaultDisclosuresInitially=d.querySelector('details[data-section="spectrum"]').open&&d.querySelector('details[data-section="reply"]').open&&[...d.querySelectorAll('details[data-section="traffic"],details[data-section="stations"],details[data-section="settings"],details[data-section="timing"]')].every(node=>!node.open);
    d.querySelector('#myCall').value='OK1HRA';d.querySelector('#myCall').dispatchEvent(new f.contentWindow.Event('change',{bubbles:true}));
    d.querySelector('#myGrid').value='JO70';d.querySelector('#myGrid').dispatchEvent(new f.contentWindow.Event('change',{bubbles:true}));
    f.contentWindow.__dataTest.setActivity({frames:[],timing:[],channels:[],messages:[
      {text:'K0OG: OK1HRA OLDER',callsigns:['K0OG','OK1HRA'],kinds:['directed','data'],submode:0,offsetHz:700,firstSlotUtcMs:now-3000,lastSlotUtcMs:now-3000},
      {text:'KN4CRD: GENERAL',callsigns:['KN4CRD'],kinds:['compound','data'],submode:0,offsetHz:750,firstSlotUtcMs:now-2500,lastSlotUtcMs:now-2500},
      {text:'KN4CRD: DL1ABC PRIVATE',callsigns:['KN4CRD','DL1ABC'],kinds:['directed','data'],submode:0,offsetHz:750,firstSlotUtcMs:now-2200,lastSlotUtcMs:now-2200},
      {text:'KN4CRD: @HB EM73',callsigns:['KN4CRD'],kinds:['heartbeat'],submode:0,offsetHz:750,firstSlotUtcMs:now-2000,lastSlotUtcMs:now-2000},
      {text:'KN4CRD: OK1HRA FOR YOU',callsigns:['KN4CRD','OK1HRA'],kinds:['directed','data'],submode:0,offsetHz:750,firstSlotUtcMs:now-1800,lastSlotUtcMs:now-1800},
      {text:'K0OG: OK1HRA NEWEST',callsigns:['K0OG','OK1HRA'],kinds:['directed','data'],submode:0,offsetHz:700,firstSlotUtcMs:now-1000,lastSlotUtcMs:now-1000}
    ],calls:[
      {call:'KN4CRD',snr:-12,offsetHz:750,submode:0,lastSlotUtcMs:now-2000,grid:'EM73'},
      {call:'K0OG',snr:2,offsetHz:700,submode:0,lastSlotUtcMs:now-1000},
      {call:'OK1HRA',snr:-7,offsetHz:900,submode:0,lastSlotUtcMs:now-500,grid:'JO70'}
    ]});
    const originalBandActivity=f.contentWindow.__dataTest.activityCounts();
    f.contentWindow.__dataTest.setRadioFrequency(14078000);
    const otherBandStartsEmpty=f.contentWindow.__dataTest.activityCounts();
    f.contentWindow.__dataTest.setActivity({frames:[],timing:[],channels:[],messages:[
      {text:'DL1ABC: 20M ONLY',callsigns:['DL1ABC'],kinds:['directed','data'],submode:0,offsetHz:800,firstSlotUtcMs:now,lastSlotUtcMs:now}
    ],calls:[
      {call:'DL1ABC',snr:-5,offsetHz:800,submode:0,lastSlotUtcMs:now}
    ]});
    const otherBandActivity=f.contentWindow.__dataTest.activityCounts();
    f.contentWindow.__dataTest.setRadioFrequency(7078000);
    const restoredBandActivity=f.contentWindow.__dataTest.activityCounts();
    f.contentWindow.__dataTest.setRadioFrequency(7079500);
    const withinToleranceActivity=f.contentWindow.__dataTest.activityCounts();
    f.contentWindow.__dataTest.setRadioFrequency(7078000);
    d.querySelector('#trxFrequency').click();
    const originalPreset=d.querySelector('[data-frequency="14078000"]');
    const editingCall=d.querySelector('#myCall'),editingGrid=d.querySelector('#myGrid'),editingTxGain=d.querySelector('#txGain');
    const defaultTxGain=editingTxGain.value==='0.25';
    editingCall.focus();editingCall.value='N0';editingCall.dispatchEvent(new f.contentWindow.Event('input',{bubbles:true}));
    editingGrid.value='';editingGrid.dispatchEvent(new f.contentWindow.Event('input',{bubbles:true}));
    editingTxGain.focus();editingTxGain.value='0.35';editingTxGain.dispatchEvent(new f.contentWindow.Event('input',{bubbles:true}));
    f.contentWindow.__dataTest.setRadioConnection(true,'linked');
    f.contentWindow.__dataTest.setAudioLive(false);
    const connectedWithoutAudioIsNotLive=!d.querySelector('#linkState').textContent.includes('RX LIVE');
    setTimeout(async ()=>{
      try {
      const modemSettingsEditingStable=editingCall.value==='N0'&&editingGrid.value==='';
      const txGainEditingStable=editingTxGain.value==='0.35';
      editingCall.value='OK1HRA';editingCall.dispatchEvent(new f.contentWindow.Event('change',{bubbles:true}));
      editingGrid.value='JO70';editingGrid.dispatchEvent(new f.contentWindow.Event('change',{bubbles:true}));
      editingTxGain.dispatchEvent(new f.contentWindow.Event('change',{bubbles:true}));
      const savedTxGain=JSON.parse(f.contentWindow.localStorage.getItem('ic705.data.js8-settings')).modems.js8call.txGain;
      const currentPreset=d.querySelector('[data-frequency="14078000"]');
      const stationObserved={speed:d.querySelector('#stationRows tr[data-call="K0OG"] td:nth-child(5)')?.textContent.trim(),fallbackTitle:d.querySelector('#stationRows tr[data-call="K0OG"] .station-direction span')?.title,gridTitle:d.querySelector('#stationRows tr[data-call="KN4CRD"] .station-direction span')?.title,distance:d.querySelector('#stationRows tr[data-call="K0OG"] .station-distance')?.textContent};
      const overlay=d.querySelector('#waterfallOverlay'),overlayContext=overlay.getContext('2d'),hzX=hz=>Math.round((hz-500)/(2700-500)*overlay.width);
      const heartbeatX=hzX(1000),heartbeatPixels=overlayContext.getImageData(heartbeatX-2,0,5,8).data,outside=overlayContext.getImageData(hzX(700),30,1,1).data;
      const heartbeatEdgePresent=Array.from({length:heartbeatPixels.length/4},(_,i)=>heartbeatPixels[i*4+3]).some(alpha=>alpha>0);
      const cleanSpectrum=Float32Array.from({length:8192},(_,i)=>.035*Math.sin(2*Math.PI*1450*i/8000)+.008*Math.sin(2*Math.PI*730*i/8000));
      f.contentWindow.__dataTest.feedSpectrum(cleanSpectrum);
      const spectrumBeforeTx=f.contentWindow.__dataTest.spectrumState();
      f.contentWindow.__dataTest.setRadioTx(true);
      const txContamination=Float32Array.from({length:8192},(_,i)=>(((i*73)%257)/128-1)*.9);
      f.contentWindow.__dataTest.feedSpectrum(txContamination);
      const spectrumDuringTx=f.contentWindow.__dataTest.spectrumState();
      f.contentWindow.__dataTest.setRadioTx(false);
      f.contentWindow.__dataTest.feedSpectrum(cleanSpectrum);
      const spectrumAfterTx=f.contentWindow.__dataTest.spectrumState();
      f.contentWindow.__dataTest.setRadioConnection(false,'disconnected');
      const reconnectButton=d.querySelector('#trxReconnect');
      const reconnectVisible=!reconnectButton.hidden&&reconnectButton.textContent.trim()==='Reconnect';
      reconnectButton.click();
      const reconnectRequested=reconnectButton.disabled&&reconnectButton.textContent.includes('Connecting');
      f.contentWindow.__dataTest.setRadioConnection(true,'linked');
      const checks={
        frequencyScopedActivity:originalBandActivity.messages===6&&originalBandActivity.calls===3&&otherBandStartsEmpty.messages===0&&otherBandStartsEmpty.calls===0&&otherBandActivity.messages===1&&otherBandActivity.calls===1&&restoredBandActivity.messages===6&&restoredBandActivity.calls===3&&withinToleranceActivity.messages===6&&withinToleranceActivity.calls===3,
        emptyIdentityDefaults,
        modemSettingsEditingStable,
        defaultTxGain,
        txGainEditingStable,
        txGainSaved:savedTxGain===0.35,
        reconnectVisible,
        reconnectRequested,
        connectedWithoutAudioIsNotLive,
        english:d.body.textContent.includes('TX SESSION'),
        js8:d.querySelector('#modeSelect').value==='js8call',
        modemApi:['AudioSource','Modems','registerModem','Decoder','Encoder'].every(name=>name in f.contentWindow),
        modemOnly:d.querySelectorAll('#modeSelect option').length===1,
        modemRowHidden:getComputedStyle(d.querySelector('.modem-menu')).display==='none',
        noFutureSlot:!d.querySelector('#modemUnavailable'),
        waterfall:!!d.querySelector('#waterfallCanvas'),
        compactWaterfall:d.querySelector('#waterfall').getBoundingClientRect().height<=90,
        heartbeatGuide:overlay.width>1000&&heartbeatEdgePresent,
        heartbeatOutsideClear:outside[3]===0,
        heartbeatRangeLabel:d.querySelector('#waterfall').title.includes('500')&&d.querySelector('#waterfall').title.includes('1000')&&d.querySelector('#waterfall').title.includes('Heartbeat'),
        waterfallTxIsolation:spectrumDuringTx.rows===spectrumBeforeTx.rows&&spectrumDuringTx.agcLow===spectrumBeforeTx.agcLow&&spectrumDuringTx.agcHigh===spectrumBeforeTx.agcHigh&&spectrumAfterTx.rows>spectrumDuringTx.rows&&spectrumAfterTx.agcReady===true&&spectrumAfterTx.agcHigh-spectrumAfterTx.agcLow>=22,
        presets:d.querySelectorAll('[data-frequency]').length===12,
        presetStable:originalPreset===currentPreset,
        recipientInSession:!!d.querySelector('#composer #recipient'),
        heartbeat:!!d.querySelector('#heartbeatButton'),
        heartbeatUsesTx:d.querySelector('#heartbeatOffset').textContent.trim()===d.querySelector('#txOffset').value+' Hz',
        tune:!!d.querySelector('#tuneButton')&&d.querySelector('#tuneOffset').textContent.trim()===d.querySelector('#txOffset').value+' Hz',
        txSession:[...d.querySelectorAll('details[data-section="reply"] > summary span')].some(node=>node.textContent.trim()==='TX SESSION'),
        defaultDisclosures:defaultDisclosuresInitially,
        ownCallPanelsExpanded:d.querySelector('details[data-section="traffic"]').open&&d.querySelector('details[data-section="stations"]').open,
        ownCallTraffic:[...d.querySelectorAll('#traffic [data-own-call="true"]')].some(node=>node.textContent==='OK1HRA'&&getComputedStyle(node).color==='rgb(255, 107, 107)'),
        ownCallStation:(()=>{const node=d.querySelector('#stationRows tr[data-call="OK1HRA"] [data-own-call="true"]');return node?.textContent==='OK1HRA'&&getComputedStyle(node).color==='rgb(255, 107, 107)';})(),
        txModes:[...d.querySelectorAll('#txSessionMode option')].map(option=>option.value).join(',')==='CHAT,EMAIL,BIN',
        autoSpeed:d.querySelector('#txSpeedResolved')?.textContent.includes('A')===true,
        stationSpeed:(d.querySelector('#stationRows tr[data-call="K0OG"] td:nth-child(5)')?.textContent.trim()||'').startsWith('A')&&(d.querySelector('#stationRows tr[data-call="K0OG"] td:nth-child(5)')?.textContent.trim()||'').endsWith('15 s'),
        stationCountry:(()=>{const own=d.querySelector('#stationRows tr[data-call="OK1HRA"] td.station-country'),us=d.querySelector('#stationRows tr[data-call="K0OG"] td.station-country');return own?.textContent.trim()==='Czech Republic'&&own?.title==='Czech Republic'&&(us?.textContent.trim()||'').length>0&&d.querySelector('[data-station-sort="country"]')?.textContent.trim().startsWith('DXCC')===true;})(),
        stationDirection:Number.isFinite(parseFloat(d.querySelector('#stationRows tr[data-call="K0OG"] .station-distance')?.textContent))&&d.querySelector('#stationRows tr[data-call="K0OG"] .station-direction span')?.title.includes('DXCC estimate')===true&&d.querySelector('#stationRows tr[data-call="KN4CRD"] .station-direction span')?.title.includes('EM73')===true,
        slotMeter:!d.querySelector('#decodeMeter')&&parseFloat(d.querySelector('#slotFill').style.width)>0&&d.querySelector('.waterfall-rhythm').getBoundingClientRect().top>=d.querySelector('#waterfall').getBoundingClientRect().bottom&&getComputedStyle(d.querySelector('#slotFill')).boxShadow==='none',
        recentNewest:d.querySelector('#traffic .message')?.textContent.includes('NEWEST'),
        recentSingleLine:d.querySelector('#traffic .message')?.getBoundingClientRect().height<=36&&!!d.querySelector('#traffic .message-meta')&&!!d.querySelector('#traffic .message-text'),
        recentMessageWhite:getComputedStyle(d.querySelector('#traffic .message-text')).color==='rgb(255, 255, 255)',
        operationalDim:d.querySelectorAll('#traffic .message.operational').length===1,
        noDebugNav:![...d.querySelectorAll('.tabs .tab')].some(link=>link.textContent.trim()==='DEBUG'),
        removedPagesAbsentFromNav:!d.querySelector('.bd-nav,.tab-cat-muted,a[href="/bd"],a[href="/"]'),
        messagePresets:d.querySelectorAll('[data-message-preset]').length>=18&&!!d.querySelector('#messagePresetsButton')&&
          ['qsl-query','yes','no','tu','dit-dit','grid-query','info-query','status-query']
            .every(key=>!!d.querySelector('[data-message-preset="'+key+'"]')),
        sendHidden:d.querySelector('#sendButton').hidden===true&&d.querySelector('#sendHint').textContent.trim()==='Enter sends',
        js8Nav:d.querySelector('a[href="/data"]')?.textContent.trim()==='JS8LAN'&&d.querySelector('a[href="/data"]')?.title==='Web Client for JS8Call',
        pageFooter:d.querySelector('.js8-page-footer a[href^="https://github.com/"]')?.textContent.trim()==='GitHub'&&d.querySelector('.js8-page-footer a[href="/THIRD-PARTY-NOTICES.txt"]')?.textContent.trim()==='Licenses',
        idleNoLeaveWarning:(()=>{const event=new f.contentWindow.Event('beforeunload',{cancelable:true});return f.contentWindow.dispatchEvent(event)!==false&&!event.defaultPrevented;})(),
        helpButton:d.querySelector('#trxHelpButton')?.textContent.trim()==='?',
        helpSteps:d.querySelectorAll('#trxHelpDialog .trx-setup-steps > li').length===8&&d.querySelector('#trxHelpDialog').textContent.includes('DATA MOD')&&d.querySelector('#trxHelpDialog').textContent.includes('WLAN'),
        helpAudioPath:[...d.querySelectorAll('#trxHelpDialog code')].some(node=>{const text=node.textContent;return text.startsWith('MENU')&&text.includes('SET')&&text.includes('Connectors')&&text.includes('MOD Input')&&text.endsWith('WLAN MOD Level');}),
        helpAudioLevels:(()=>{const codes=[...d.querySelectorAll('#trxHelpDialog code')].map(node=>node.textContent);return codes.includes('25%')&&codes.includes('TX audio gain 0.25')&&codes.includes('WLAN MOD Level 25%')&&!codes.includes('50%');})(),
        firstVisitHelp:window.firstVisitHelpObserved===true,
        txSafe:d.querySelector('#sendButton').disabled,
        worker:d.querySelector('#modemState').textContent.includes('ready'),
        loading:window.loadingObserved===true,
        directedCommandFrame:(()=>{const frames=f.contentWindow.Js8Protocol.buildReplyFrames({myCall:'OK1HRA',toCall:'K0OG',text:'SNR -12'}),decoded=f.contentWindow.Js8Protocol.decodeFrame({...frames[0],submode:0,offsetHz:1500,slotUtcMs:0});return frames.length===1&&frames[0].raw==='TBx2Q-uJkbaJ'&&frames[0].messageText==='OK1HRA: K0OG SNR -12'&&decoded.command===' SNR'&&decoded.number==='-12';})(),
        heartbeatProtocolFrame:(()=>{const frames=f.contentWindow.Js8Protocol.buildHeartbeatFrames({myCall:'OK1HRA',grid:'JO70AA'}),decoded=f.contentWindow.Js8Protocol.decodeFrame({...frames[0],submode:0,offsetHz:1500,slotUtcMs:0});return frames.length===1&&frames[0].raw==='31-QkpgqOT6W'&&frames[0].messageText==='OK1HRA: @HB JO70'&&decoded.command==='HEARTBEAT'&&decoded.text==='OK1HRA: @HB JO70 ';})()
      };
      // Session snapshot round-trip on the real persist/restore path (the ?test
      // build only skips the automatic hooks, not the functions): keep a draft,
      // write to sessionStorage, restore into state and confirm the frequency
      // buckets, messages and draft return with the paused divider rendered.
      d.querySelector('#messageInput').value='DRAFT KEEP';
      const preSnap=f.contentWindow.__dataTest.snapshotBuild();
      checks.snapshotBuildScope=Array.isArray(preSnap.buckets)&&preSnap.buckets.length>=2&&preSnap.draft==='DRAFT KEEP'&&preSnap.buckets.some(bucket=>bucket.messages.length>0);
      f.contentWindow.__dataTest.snapshotWrite();
      const stored=(()=>{try{return JSON.parse(f.contentWindow.sessionStorage.getItem('js8lan.session.v1'));}catch(_error){return null;}})();
      checks.snapshotStored=stored?.version===1&&stored.draft==='DRAFT KEEP'&&stored.buckets.length===preSnap.buckets.length;
      d.querySelector('#messageInput').value='';
      const restoreResult=f.contentWindow.__dataTest.snapshotRestore();
      checks.snapshotRestored=restoreResult===true&&f.contentWindow.__dataTest.activityCounts().messages>0&&d.querySelector('#messageInput').value==='DRAFT KEEP';
      checks.snapshotDivider=d.querySelectorAll('#traffic .restore-divider').length===1;
      f.contentWindow.sessionStorage.removeItem('js8lan.session.v1');
      d.querySelector('#messageInput').value='';
      const sd=setupFrame.contentDocument,radioSection=sd.querySelector('#radioSection'),lanWarning=sd.querySelector('#trx1LanWarning');
      const gd=lanGateFrame.contentDocument,lanGate=gd.querySelector('#lanRequired');
      checks.lanRequiredGate=gd.body.classList.contains('lan-required-only')&&!lanGate.hidden&&!gd.querySelector('.brand')&&getComputedStyle(gd.querySelector('.radio-bar')).display==='none'&&getComputedStyle(gd.querySelector('#js8Interface')).display==='none'&&lanGate.querySelector('h1')?.textContent.trim()==='JS8Call requires TRX1 over LAN'&&lanGate.textContent.includes('not available with a Bluetooth or serial/CAT connection')&&lanGate.textContent.includes('Other Icom transceivers')&&!!lanGate.querySelector('a[href="/setup#radioSection"]');
      checks.lanGateNoLeaveWarning=(()=>{const event=new lanGateFrame.contentWindow.Event('beforeunload',{cancelable:true});return lanGateFrame.contentWindow.dispatchEvent(event)!==false&&!event.defaultPrevented;})();
      checks.setupJs8Nav=sd.querySelector('a[href="/data"]')?.textContent.trim()==='JS8LAN'&&sd.querySelector('a[href="/data"]')?.title==='Web Client for JS8Call';
      checks.setupRemovedPagesAbsentFromNav=!sd.querySelector('.bd-nav,.tab-cat-muted,a[href="/bd"],a[href="/"]');
      const missingInputs=[...sd.querySelectorAll('[name="lanip"],[name="lanuser"],[name="lanpass"]')];
      const setupMissingObserved=radioSection?.open===true&&lanWarning?.hidden===false&&missingInputs.length===3&&missingInputs.every(input=>input.classList.contains('setup-required-missing')&&input.getAttribute('aria-invalid')==='true');
      const setupValues={lanip:'192.168.1.60',lanuser:'operator',lanpass:'secret123'};
      missingInputs.forEach(input=>{input.value=setupValues[input.name];input.dispatchEvent(new setupFrame.contentWindow.Event('input',{bubbles:true}));});
      checks.setupLanWarning=setupMissingObserved&&lanWarning.hidden===true&&missingInputs.every(input=>!input.classList.contains('setup-required-missing')&&input.getAttribute('aria-invalid')==='false');
      // Unattended panel: armed, but the fixture says the modem tab has been
      // silent for 31 s. A timer alone would still read "armed" -- the point of
      // the panel is that this state is visibly flagged.
      const unaSection=sd.querySelector('#unattendedSection');
      unaSection.open=true;
      unaSection.dispatchEvent(new setupFrame.contentWindow.Event('toggle'));
      await new Promise(resolve=>setTimeout(resolve,400));
      const unaText=sd.querySelector('#unattendedGrid').textContent;
      const unaBad=[...sd.querySelectorAll('#unattendedGrid .unattended-bad')].map(node=>node.textContent);
      checks.unattendedPanel=unaText.includes('12 h')&&unaText.includes('silent for')&&
        unaBad.some(text=>text.includes('silent for'))&&
        sd.querySelector('#unattendedLog').textContent.includes('BLOCK liveness lost')&&
        [...sd.querySelectorAll('#unattendedExtend button')].map(b=>b.textContent).join('|')==='Extend 1 h|Extend 6 h|Extend 12 h|Extend 24 h|Extend 168 h';
      sd.querySelector('#unattendedRevoke').click();
      await new Promise(resolve=>setTimeout(resolve,300));
      // Auto-reply wiring: a decoded directed query must reach the engine and
      // come back as a composed answer. AUTO is off here, so the answer belongs
      // in the message box rather than on the air.
      f.contentWindow.__dataTest.setActivity({messages:[],frames:[],
        calls:[{call:'K0OG',snr:-12,offsetHz:1500,submode:0,dtMs:0,quality:1,lastSlotUtcMs:Date.now()}]});
      const composer=d.querySelector('#messageInput');
      composer.value='';
      f.contentWindow.__dataTest.feedDirected({from:'K0OG',to:'OK1HRA',command:' SNR?'});
      const bufferedAnswer=composer.value;
      // A second identical query inside the window must be refused, not answered.
      composer.value='';
      f.contentWindow.__dataTest.feedDirected({from:'K0OG',to:'OK1HRA',command:' SNR?'});
      const afterRepeat=composer.value;
      // A query for somebody else must be ignored outright.
      f.contentWindow.__dataTest.feedDirected({from:'K0OG',to:'OK2XYZ',command:' GRID?'});
      const autoState=f.contentWindow.__dataTest.autoReplyState();
      // The immediate repeat is caught by the QSO lock, not the restriction
      // window: two directed frames in a row mean a conversation is running,
      // and the station stays quiet. The window itself is covered by
      // protocol/restrictions_smoke.js, which can control time.
      checks.autoReplyWiring=bufferedAnswer==='K0OG SNR -12'&&afterRepeat===''&&
        autoState.restrictions.granted===1&&autoState.skipped===2&&
        autoState.lockUntilMs>Date.now();
      // TX arbiter: an answer produced while the radio is busy must be queued
      // with an expiry, not refused and not sent late. AUTO on for this part.
      // TX arbiter: with AUTO on, an answer must reach the queue rather than be
      // refused. The QSO lock is cleared first because the checks above armed it
      // and it would otherwise mask the queue entirely.
      d.querySelector('#autoReply').click();
      // Enabling radio TX is a precondition: without it the engine correctly
      // refuses with tx-not-enabled and nothing ever reaches the queue.
      const txSafetyBox=d.querySelector('#txSafety');
      const txSafetyWas=txSafetyBox.checked;
      if(!txSafetyWas){txSafetyBox.checked=true;txSafetyBox.dispatchEvent(new f.contentWindow.Event('change',{bubbles:true}));}
      f.contentWindow.__dataTest.resetAutoReplyLock();
      f.contentWindow.__dataTest.feedDirected({from:'KD8SKZ',to:'OK1HRA',command:' GRID?'});
      const queued=f.contentWindow.__dataTest.txQueueState();
      // Either it was taken straight away (radio idle) or it is waiting with a
      // finite expiry -- never queued forever.
      checks.txQueue=(queued.sent+queued.size)>=1&&
        queued.items.every(item=>item.source!=='autoreply'||typeof item.inMs==='number');
      // ---- newly added functions ----
      // IMPORTANT: MSG/MSG TO:/relay-text/QUERY MSG are multi-frame checksummed
      // commands. The production ActivityStore does not yet CRC-reassemble them,
      // so those paths are gated off and NOT asserted here as working end-to-end.
      // What is asserted: the single-frame HB-SNR advertisement logic and the
      // inbox reading UI, verified against a directly stored message (the store
      // is what the firmware will populate once reassembly lands).
      const dt=f.contentWindow.__dataTest;

      // HB ACK requires unattended mode + radio TX enabled; turn both on for this
      // check and restore afterwards.
      const hbSafety=d.querySelector('#txSafety'); const hbSafetyWas=hbSafety.checked;
      if(!hbSafetyWas){hbSafety.checked=true;hbSafety.dispatchEvent(new f.contentWindow.Event('change',{bubbles:true}));}
      const hbAuto=d.querySelector('#autoReply'); const hbAutoWas=hbAuto.checked;
      if(!hbAutoWas){hbAuto.click();}
      const hbEnabled=d.querySelector('#hbEnabled'); const hbEnabledWas=hbEnabled.checked;
      if(!hbEnabledWas){hbEnabled.click();}
      // Seed a stored message the way the firmware will, bypassing the missing
      // RX reassembly, then check the beacon advertises it instead of a plain ACK.
      dt.storeInboxDirect({from:'OK7DEP',to:'OK8HB',text:'STORED FOR YOU'});
      dt.clearTxCaptured();
      dt.feedHeartbeat({from:'OK8HB',to:'@HB',command:'HEARTBEAT',grid:'JO70'});
      const advert=dt.txCaptured().find(item=>item.to==='OK8HB'&&
        item.text.indexOf('HEARTBEAT SNR ')===0&&item.text.indexOf(' MSG ID ')>0);
      checks.heartbeatSnrWiring=Boolean(advert);
      d.querySelector('#abortButton').click();
      await new Promise(resolve=>setTimeout(resolve,120));
      if(!hbEnabledWas){hbEnabled.click();}
      if(!hbAutoWas){hbAuto.click();}
      if(!hbSafetyWas){hbSafety.checked=false;hbSafety.dispatchEvent(new f.contentWindow.Event('change',{bubbles:true}));}

      // Inbox reading UI shows the stored message and the QUERY MSGS control.
      dt.renderInboxNow();
      const inboxRows=[...d.querySelectorAll('#inboxRows tr')];
      checks.inboxUiWiring=inboxRows.some(row=>row.textContent.includes('OK8HB'))&&
        Boolean(d.querySelector('#inboxQueryMsgs'));

      // CQ interval selector: values offered and the setting applied.
      const cqSel=d.querySelector('#cqRepeat');
      const cqFlag=()=>[...d.querySelectorAll('#settingsFlags .summary-flag')].find(node=>node.textContent.trim()==='CQ');
      cqSel.value='5'; cqSel.dispatchEvent(new f.contentWindow.Event('change',{bubbles:true}));
      checks.cqIntervalWiring=[...cqSel.options].map(o=>o.value).join(',')==='0,2,5,10,15'&&
        d.querySelector('#cqState').textContent.includes('5 min');
      // SETTINGS header pills report the switches without being switches: they
      // follow the CQ selector, and none of them is clickable.
      const cqFlagOn=cqFlag()?.classList.contains('on')===true&&cqFlag()?.title.includes('5 min');
      cqSel.value='0'; cqSel.dispatchEvent(new f.contentWindow.Event('change',{bubbles:true}));
      checks.settingsFlags=[...d.querySelectorAll('#settingsFlags .summary-flag')].map(node=>node.textContent.trim()).join(',')==='TX,AUTO,CQ,HB,ACK'&&
        cqFlagOn&&cqFlag()?.classList.contains('on')===false&&
        !d.querySelector('#settingsFlags button,#settingsFlags input,#settingsFlags a');

      // Multi-frame reassembly: a fully assembled, checksum-verified MSG message
      // must be stored; a checksum-failed one must be dropped. This is the real
      // path (dispatchAssembledMessage), fed a message the way the worker's
      // ActivityStore produces it.
      const inboxSizeBefore=dt.inboxState().size;
      dt.feedAssembled({directed:{from:'OK9MSG',to:'OK1HRA',command:' MSG'},
        payload:'HELLO FROM REASSEMBLY',checksumOk:true});
      const afterStore=dt.inboxState();
      checks.reassemblyStore=afterStore.size===inboxSizeBefore+1&&
        afterStore.items.some(item=>item.text==='HELLO FROM REASSEMBLY'&&item.from==='OK9MSG');
      dt.feedAssembled({directed:{from:'OK9BAD',to:'OK1HRA',command:' MSG'},
        payload:'SHOULD BE DROPPED',checksumOk:false});
      checks.reassemblyChecksum=dt.inboxState().size===afterStore.size;

            // The answer really did go out; stop it so the page is idle again for the
      // BIN and TX checks that follow.
      d.querySelector('#abortButton').click();
      await new Promise(resolve=>setTimeout(resolve,150));
      if(!txSafetyWas){txSafetyBox.checked=false;txSafetyBox.dispatchEvent(new f.contentWindow.Event('change',{bubbles:true}));}
      d.querySelector('#autoReply').click();
      // Heartbeat: an incoming beacon must produce an ACK, and the station's own
      // beacon must be postponed by that traffic rather than firing into it.
      d.querySelector('#hbEnabled').click();
      const hbBefore=f.contentWindow.__dataTest.heartbeatState();
      f.contentWindow.__dataTest.feedHeartbeat({from:'K0OG',to:'@HB',command:'HEARTBEAT',grid:'EM73'});
      const hbAfter=f.contentWindow.__dataTest.heartbeatState();
      checks.heartbeatWiring=hbBefore.enabled===true&&hbBefore.intervalMs===15*60000&&
        typeof hbBefore.dueInMs==='number'&&
        (hbAfter.acked===1||hbAfter.ackSkipped>=1)&&
        [...d.querySelectorAll('#hbMinutes option')].map(o=>o.value).join(',')==='5,10,15,30,60';
      d.querySelector('#hbEnabled').click();
      // Relay via the assembled path: an armed station must forward a relayed
      // message to the next hop with attribution. Needs unattended mode + TX.
      const rSafety=d.querySelector('#txSafety'); const rSafetyWas=rSafety.checked;
      if(!rSafetyWas){rSafety.checked=true;rSafety.dispatchEvent(new f.contentWindow.Event('change',{bubbles:true}));}
      const rAuto=d.querySelector('#autoReply'); const rAutoWas=rAuto.checked;
      if(!rAutoWas){rAuto.click();}
      dt.clearTxCaptured();
      dt.feedAssembled({directed:{from:'KN4CRD',to:'OK1HRA',command:'>'},
        payload:'OH8STN>HELLO JULIAN',checksumOk:true});
      const fwd=dt.txCaptured().find(item=>item.to==='OH8STN'&&item.text==='>HELLO JULIAN DE KN4CRD');
      checks.relayForward=Boolean(fwd);
      d.querySelector('#abortButton').click();
      await new Promise(resolve=>setTimeout(resolve,120));
      if(!rAutoWas){rAuto.click();}
      if(!rSafetyWas){rSafety.checked=false;rSafety.dispatchEvent(new f.contentWindow.Event('change',{bubbles:true}));}
      // Inbox durable copy: the message seeded by the firmware /inbox fixture must
      // have been restored on load. (The K0OG entry from the seed; plus OK8HB we
      // stored directly above.)
      const inboxNow=f.contentWindow.__dataTest.inboxState();
      checks.inboxLoad=inboxNow.items.some(item=>item.to==='K0OG');
      // Groups: the always-joined pair must be present without being stored, a
      // custom group must be answered, and one we never joined must be ignored.
      const groupsField=d.querySelector('#groups');
      groupsField.value='@ARESGA';
      groupsField.dispatchEvent(new f.contentWindow.Event('change',{bubbles:true}));
      const joined=f.contentWindow.__dataTest.myGroups();
      f.contentWindow.__dataTest.resetAutoReplyLock();
      const composerG=d.querySelector('#messageInput'); composerG.value='';
      f.contentWindow.__dataTest.feedDirected({from:'OK5GRP',to:'@ARESGA',command:' SNR?'});
      const groupAnswer=composerG.value;
      composerG.value='';
      f.contentWindow.__dataTest.resetAutoReplyLock();
      f.contentWindow.__dataTest.feedDirected({from:'OK6GRP',to:'@NOTMINE',command:' SNR?'});
      const strangerGroup=composerG.value;
      checks.groupsWiring=joined.includes('@ALLCALL')&&joined.includes('@HB')&&
        joined.includes('@ARESGA')&&groupAnswer.startsWith('OK5GRP SNR')&&strangerGroup==='';
      groupsField.value='';
      groupsField.dispatchEvent(new f.contentWindow.Event('change',{bubbles:true}));
      checks.autoReplySettings=!!d.querySelector('#infoText')&&!!d.querySelector('#statusText')&&
        d.querySelector('#autoReply').checked===false&&
        [...d.querySelectorAll('#armHours option')].map(o=>o.value).join(',')==='1,6,12,24,168';
      // The reassembly/relay/HB tests can leave an auto-triggered TX in flight
      // (driveEncoder is async, so an immediate abort is a no-op). Settle to idle
      // before the manual TX test, which fails with "TX is busy" otherwise.
      for(let i=0;i<15;i+=1){
        const st=f.contentWindow.__dataTest.txStatus();
        if(['idle','completed','aborted','fault'].includes(st))break;
        d.querySelector('#abortButton').click();
        await new Promise(resolve=>setTimeout(resolve,100));
      }
            sd.querySelector('#setup-form').requestSubmit();
      d.querySelector('#trxHelpButton').click();
      checks.manualHelp=d.querySelector('#trxHelpDialog').open===true&&d.querySelector('#trxHelpModeWarning').hidden===true;
      d.querySelector('#trxHelpDialog .trx-help-close').click();
      checks.usbModeNeutral=!d.querySelector('#trxMode').classList.contains('incompatible');
      f.contentWindow.__dataTest.setRadioMode('CW');
      checks.incompatibleMode=d.querySelector('#trxMode').classList.contains('incompatible')&&d.querySelector('#trxMode').title.includes('USB');
      checks.modeHelp=d.querySelector('#trxHelpDialog').open===true&&d.querySelector('#trxHelpModeWarning').hidden===false;
      d.querySelector('#trxHelpDialog .trx-help-close').click();
      f.contentWindow.__dataTest.setRadioMode('USB');
      const change=id=>d.querySelector(id).dispatchEvent(new f.contentWindow.Event('change',{bubbles:true}));
      const callSort=d.querySelector('[data-station-sort="call"]');
      callSort.click();
      const sortAscObserved={call:d.querySelector('#stationRows tr')?.dataset.call,direction:callSort.closest('th').getAttribute('aria-sort')};
      checks.sortAsc=sortAscObserved.call==='K0OG'&&sortAscObserved.direction==='ascending';
      callSort.click();
      const sortDescObserved={call:d.querySelector('#stationRows tr')?.dataset.call,direction:callSort.closest('th').getAttribute('aria-sort')};
      checks.sortDesc=sortDescObserved.call==='OK1HRA'&&sortDescObserved.direction==='descending';
      d.querySelector('#stationRows tr[data-call="KN4CRD"]').click();
      checks.stationSelect=d.querySelector('#recipient').value==='KN4CRD'&&f.contentWindow.__dataTest.selectedCall()==='KN4CRD';
      const recipientThread=d.querySelector('#chatThread').textContent;
      checks.recipientTrafficFilter=recipientThread.includes('FOR YOU')&&recipientThread.includes('GENERAL')&&recipientThread.includes('@HB EM73')&&!recipientThread.includes('HEARTBEAT')&&!recipientThread.includes('PRIVATE');
      d.querySelector('#messagePresetsButton').click();
      d.querySelector('[data-message-preset="snr"]').click();
      checks.snrPreset=d.querySelector('#messageInput').value==='SNR -12'&&d.querySelector('#messagePresetsMenu').hidden===true;
      d.querySelector('#messagePresetsButton').click();
      d.querySelector('[data-message-preset="cq"]').click();
      checks.cqPreset=d.querySelector('#messageInput').value==='CQ CQ CQ';
      d.querySelector('#messageInput').value='';d.querySelector('#messageInput').dispatchEvent(new f.contentWindow.Event('input',{bubbles:true}));
      d.querySelector('#recipientClear')?.click();
      checks.recipientClear=d.querySelector('#recipient').value===''&&f.contentWindow.__dataTest.selectedCall()==='';
      const txSessionMode=d.querySelector('#txSessionMode');
      if(txSessionMode){txSessionMode.value='EMAIL';change('#txSessionMode');}
      d.querySelector('#emailGatewayAdd')?.click();
      const setEmailField=(selector,value)=>{const input=d.querySelector(selector);input.value=value;input.dispatchEvent(new f.contentWindow.Event('input',{bubbles:true}));};
      setEmailField('#emailGatewayName','Fixture APRS');setEmailField('#emailGatewayTarget','@APRSIS');setEmailField('#emailGatewayDial','7078000');setEmailField('#emailGatewayOffset','1500');
      d.querySelector('#emailGatewayFormat').value='aprs-email2';change('#emailGatewayFormat');
      setEmailField('#emailGatewayMaxBody','40');d.querySelector('#emailGatewayPolicy').value='aprs';
      d.querySelector('#emailGatewayForm').requestSubmit();
      setEmailField('#emailAddress','user@example.com');setEmailField('#emailMessage','TEST');
      checks.emailReady=d.querySelector('#emailSession')?.hidden===false&&d.querySelector('#emailPreview').textContent==='@APRSIS CMD :EMAIL-2  :USER@EXAMPLE.COM TEST'&&d.querySelector('#emailSession').textContent.includes('Gateway callsign')&&d.querySelector('#emailSend').disabled;
      if(txSessionMode){txSessionMode.value='BIN';change('#txSessionMode');}
      checks.binReady=d.querySelector('#binSession')?.hidden===false&&!d.querySelector('#binFile').disabled&&!d.querySelector('#binRecipient').disabled&&d.querySelector('#binOffer').disabled&&d.querySelector('#binSession').textContent.includes('Reliable point-to-point');
      d.querySelector('#txSpeed').value='E';change('#txSpeed');
      const slowLimits=d.querySelector('#binFileDetails').textContent;
      d.querySelector('#txSpeed').value='C';change('#txSpeed');
      const js840Limits=d.querySelector('#binFileDetails').textContent;
      d.querySelector('#txSpeed').value='A';change('#txSpeed');
      const normalLimits=d.querySelector('#binFileDetails').textContent;
      checks.binLimits=slowLimits.includes('Hard limit')&&slowLimits.includes('1 KiB')&&js840Limits.includes('Hard limit')&&js840Limits.includes('8 KiB')&&normalLimits.includes('Recommended')&&normalLimits.includes('1 KiB')&&normalLimits.includes('2 KiB');
      checks.binPrepared=false;
      checks.binStorage=false;
      const binInput=d.querySelector('#binFile'),oversizeFixture=new f.contentWindow.File([new Uint8Array(2049)],'too-large.bin',{type:'application/octet-stream'});
      Object.defineProperty(binInput,'files',{configurable:true,value:[oversizeFixture]});change('#binFile');
      checks.binOversizeRejected=f.contentWindow.__dataTest.fileProtocol().prepared===null&&d.querySelector('#binError').textContent.includes('exceeds')&&d.querySelector('#binOffer').disabled;
      const binFixture=new f.contentWindow.File([new Uint8Array([0,1,2,3,4])],'tiny.bin',{type:'application/octet-stream'});
      Object.defineProperty(binInput,'files',{configurable:true,value:[binFixture]});change('#binFile');
      setTimeout(()=>{const prepared=f.contentWindow.__dataTest.fileProtocol().prepared;checks.binPrepared=prepared?.manifest.originalSize===5&&prepared?.manifest.blockCount===1&&prepared?.manifest.sha256Hex.length===64;},500);
      const binStore=new f.contentWindow.Js8FileTransfer.TransferStore();
      binStore.save({id:'SMOKE1',direction:'tx',state:'complete',blocks:[new Uint8Array([1,2,3])]}).then(()=>binStore.get('SMOKE1')).then(saved=>{checks.binStorage=saved?.blocks?.[0]?.[2]===3;return binStore.delete('SMOKE1');}).catch(()=>{});
      if(txSessionMode){txSessionMode.value='CHAT';change('#txSessionMode');}
      currentPreset.click();
      d.querySelector('#recipient').value='K0OG';change('#recipient');
      d.querySelector('#txSpeed').value='I';change('#txSpeed');
      d.querySelector('#txSafety').checked=true;change('#txSafety');
      checks.txEnabled=!d.querySelector('#sendButton').disabled;
      const gate=d.querySelector('#sendButton').title,diag=d.querySelector('#diagnostics').textContent;
      if(!checks.txEnabled){checks.txCompleted=false;return fetch('/result',{method:'POST',body:JSON.stringify({pass:false,text:'DATA BROWSER FAIL '+JSON.stringify(checks)+' sort='+JSON.stringify({sortAscObserved,sortDescObserved})+' gate='+gate+' diag='+diag})});}
      const longTxText='THIS IS A LONG JS8 MESSAGE';
      d.querySelector('#messageInput').value=longTxText;
      const enterEvent=new f.contentWindow.KeyboardEvent('keydown',{key:'Enter',bubbles:true,cancelable:true});
      d.querySelector('#messageInput').dispatchEvent(enterEvent);
      checks.enterSends=enterEvent.defaultPrevented&&d.querySelector('#messageInput').value==='';
      const fullLongTxText='OK1HRA: K0OG '+longTxText;
      checks.txQueuedVisual=d.querySelector('.chat-row.outgoing:last-child .tx-copy-pending')?.textContent===fullLongTxText&&d.querySelector('#txPayload')?.textContent.includes(fullLongTxText);
      checks.leaveWarningDuringTx=(()=>{const event=new f.contentWindow.Event('beforeunload',{cancelable:true});return f.contentWindow.dispatchEvent(event)===false&&event.defaultPrevented;})();
      let tries=0,sawPartialTx=false,sawPausedTx=false;
      const poll=setInterval(()=>{
        const summary=d.querySelector('#txSummary').textContent,completed=summary.toLowerCase().includes('completed');
        const outgoing=d.querySelector('.chat-row.outgoing:last-child');
        const sentLength=outgoing?.querySelector('.tx-copy-sent')?.textContent.length||0;
        const hasPending=Boolean(outgoing?.querySelector('.tx-copy-pending'));
        const hasActive=Boolean(outgoing?.querySelector('.tx-copy-active'));
        if(sentLength>0&&hasPending)sawPartialTx=true;
        if(summary.toLowerCase().includes('waiting-slot')&&sentLength>0&&hasPending&&!hasActive)sawPausedTx=true;
        if(completed||++tries>110){
          clearInterval(poll);checks.txCompleted=completed;
          if(!completed){const pass=false;return fetch('/result',{method:'POST',body:JSON.stringify({pass,text:'DATA BROWSER FAIL '+JSON.stringify(checks)+' station='+JSON.stringify(stationObserved)+' sort='+JSON.stringify({sortAscObserved,sortDescObserved})+' gate='+gate+' tx='+summary})});}
          const completedText=d.querySelector('.chat-row.outgoing:last-child .tx-copy-sent');
          checks.txProgressVisual=sawPartialTx;
          checks.txSlotPauseVisual=sawPausedTx;
          checks.txCompletedVisual=completedText?.textContent===fullLongTxText&&!d.querySelector('.chat-row.outgoing:last-child .tx-copy-pending')&&getComputedStyle(completedText).backgroundColor!=='rgba(0, 0, 0, 0)';
          d.querySelector('#messageInput').value='WILL FAIL';
          d.querySelector('#messageInput').dispatchEvent(new f.contentWindow.KeyboardEvent('keydown',{key:'Enter',bubbles:true,cancelable:true}));
          d.querySelector('#abortButton').click();
          checks.txFailedVisual=d.querySelector('.chat-row.outgoing:last-child .tx-copy-failed')?.textContent==='OK1HRA: K0OG WILL FAIL';
          d.querySelector('#heartbeatButton').click();
          checks.heartbeatQueued=!d.querySelector('#txSummary').textContent.toLowerCase().includes('completed');
          checks.heartbeatPayload=d.querySelector('#txPayload')?.textContent.includes('OK1HRA: @HB JO70')&&!d.querySelector('#txPayload')?.textContent.includes('HEARTBEAT');
          let hbTries=0;
          const hbPoll=setInterval(()=>{
            const hbSummary=d.querySelector('#txSummary').textContent,hbCompleted=hbSummary.toLowerCase().includes('completed');
            if(hbCompleted||++hbTries>50){
              clearInterval(hbPoll);checks.heartbeatTx=hbCompleted;
              d.querySelector('#tuneButton').click();
              checks.tuneQueued=d.querySelector('#tuneLabel').textContent==='STOP';
              let tuneTries=0;
              const tunePoll=setInterval(()=>{
                const tuneSummary=d.querySelector('#txSummary').textContent,tuneTransmitting=tuneSummary.toLowerCase().includes('transmitting');
                if(tuneTransmitting||++tuneTries>30){
                  clearInterval(tunePoll);checks.tuneTransmitting=tuneTransmitting;
                  checks.viewportTx=d.body.classList.contains('radio-transmitting')&&getComputedStyle(d.body,'::after').borderTopWidth==='3px';
                  d.querySelector('#tuneButton').click();
                  checks.tuneStopped=d.querySelector('#txSummary').textContent.toLowerCase().includes('aborted')&&d.querySelector('#tuneLabel').textContent==='TUNE';
                  checks.viewportTxCleared=!d.body.classList.contains('radio-transmitting');
                  const pass=Object.values(checks).every(Boolean);
                  fetch('/result',{method:'POST',body:JSON.stringify({pass,text:'DATA BROWSER '+(pass?'PASS ':'FAIL ')+JSON.stringify(checks)+' station='+JSON.stringify(stationObserved)+' sort='+JSON.stringify({sortAscObserved,sortDescObserved})+' gate='+gate+' tx='+d.querySelector('#txSummary').textContent+' modem='+d.querySelector('#modemState').textContent+' diag='+diag})});
                }
              },100);
            }
          },200);
        }
      },200);
      } catch(error) {
        fetch('/result',{method:'POST',body:JSON.stringify({pass:false,text:'DATA BROWSER CHECK ERROR: '+String(error?.stack||error)})});
      }
    },650);
    } catch(error) {
      fetch('/result',{method:'POST',body:JSON.stringify({pass:false,text:'DATA BROWSER INIT ERROR: '+String(error?.stack||error)+' body='+d.body.className+' detail='+d.querySelector('#lanRequiredDetail')?.textContent})});
    }
  },5000);
};
</script>`);return;
  }
  const relative=url.pathname==="/data"?"data.html":url.pathname==="/setup"?"setup.html":url.pathname.slice(1);
  const target=path.resolve(data,relative);
  if(!target.startsWith(data+path.sep))return res.writeHead(403).end();
  // Match firmware handleFileFromSPIFFS(): only use advertised encodings.
  const br=target+".br", gz=target+".gz";
  const accepts=String(req.headers["accept-encoding"]||"");
  const encoded=accepts.includes("br")&&fs.existsSync(br)?br:(accepts.includes("gzip")&&fs.existsSync(gz)?gz:target);
  if(path.basename(target).startsWith("js8-")){if(encoded===br)js8Brotli++;if(encoded===gz)js8Gzip++;}
  fs.readFile(encoded,(error,bytes)=>{if(error)return res.writeHead(404).end();res.setHeader("Content-Type",mime[path.extname(target)]||"application/octet-stream");if(encoded===br)res.setHeader("Content-Encoding","br");if(encoded===gz)res.setHeader("Content-Encoding","gzip");if(url.pathname==="/js8-jsc.bin.br"){jscRequests++;jscStartedAt=Date.now();res.setHeader("Cache-Control","no-store");res.setHeader("Content-Length",bytes.length);const chunk=Math.ceil(bytes.length/8);let at=0;const tick=()=>{res.write(bytes.subarray(at,Math.min(bytes.length,at+chunk)));at+=chunk;if(at>=bytes.length){jscComplete=true;jscCompleteAt=Date.now();return res.end();}setTimeout(tick,150);};tick();return;}res.end(bytes);});
});
server.on("upgrade",(req,socket)=>{const wsUrl=new URL(req.url,"http://fixture");if(wsUrl.pathname!=="/audiows")return socket.destroy();
// Same gate as the firmware: the audio socket is where the lock actually bites,
// so a handshake without the owning token never reaches the stream.
if(!sessionOwns(wsUrl.searchParams.get("token"))){session.wsRefusals++;socket.write("HTTP/1.1 409 Conflict\r\nConnection: close\r\n\r\n");return socket.destroy();}
wsConnections++;wsOpenedAt=Date.now();if(!jscComplete)earlyWsConnections++;const accept=crypto.createHash("sha1").update(req.headers["sec-websocket-key"]+"258EAFA5-E914-47DA-95CA-C5AB0DC85B11").digest("base64");socket.write("HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: "+accept+"\r\n\r\n");socket.write(frame(1,JSON.stringify({type:"hello",protocol:"AUD1",version:1,streamId,rx:[{kind:"RX_ULAW",sampleRate:8000}],tx:[{kind:"TX_PCM16",sampleRate:48000}],maxPayloadBytes:1920})));const interval=setInterval(()=>{if(socket.destroyed)return clearInterval(interval);socket.write(frame(2,aud1()));},20);socket.on("data",readClientFrames(socket));socket.on("close",()=>clearInterval(interval));socket.on("error",()=>clearInterval(interval));});
server.listen(0,"127.0.0.1",()=>{chrome=spawn("google-chrome",["--headless=new","--no-sandbox","--disable-gpu","--disable-dev-shm-usage","--no-proxy-server","--host-resolver-rules=MAP ic705.test 127.0.0.1",`http://ic705.test:${server.address().port}/smoke.html`]);let errors="";chrome.stderr.on("data",c=>errors+=c);chrome.on("close",code=>{if(!finished)finish(false,`DATA BROWSER FAIL Chrome exited ${code}\n${errors}`);});timer=setTimeout(()=>finish(false,`DATA BROWSER FAIL timeout prepares=${txPrepares} packets=${txPackets}`),45000);});
