const socket = window.socket || (window.socket = io());

const telemetryEl = document.getElementById('telemetry');
const rawEl = document.getElementById('raw');
const relaysEl = document.getElementById('relays');
const connStatusEl = document.getElementById('conn-status');
const controlsEl = document.getElementById('controls');
const valT1 = document.getElementById('val-t1');
const valH1 = document.getElementById('val-h1');
const valT2 = document.getElementById('val-t2');
const valH2 = document.getElementById('val-h2');

const ctxT1 = document.getElementById('chart-t1').getContext('2d');
const ctxH1 = document.getElementById('chart-h1').getContext('2d');
const ctxT2 = document.getElementById('chart-t2').getContext('2d');
const ctxH2 = document.getElementById('chart-h2').getContext('2d');

let relayStates = {};
const MAX_HISTORY = 100;

// Try to enable Supabase Realtime if the server provides anon config at runtime
async function initSupabaseRealtime() {
  try {
    const r = await fetch('/api/supabase-config');
    if (!r.ok) return;
    const cfg = await r.json();
    if (!cfg.SUPABASE_URL || !cfg.SUPABASE_ANON_KEY) return;
    // load UMD build of supabase-js dynamically
    await new Promise((resolve, reject) => {
      const s = document.createElement('script');
      s.src = 'https://cdn.jsdelivr.net/npm/@supabase/supabase-js/dist/umd/supabase.min.js';
      s.onload = resolve; s.onerror = reject; document.head.appendChild(s);
    });
    if (!window.supabase || !window.supabase.createClient) return;
    const { createClient } = window.supabase;
    const supabase = createClient(cfg.SUPABASE_URL, cfg.SUPABASE_ANON_KEY);
    // initial load: latest 50
    const { data: rows } = await supabase.from('telemetry').select('*').order('ts', { ascending: false }).limit(50);
    if (Array.isArray(rows)) {
      rows.reverse().forEach(it => {
        const ts = new Date(it.ts).toLocaleTimeString();
        if (it.t1 !== null) pushPoint(chartT1, ts, Number(it.t1));
        if (it.h1 !== null) pushPoint(chartH1, ts, Number(it.h1));
        if (it.t2 !== null) pushPoint(chartT2, ts, Number(it.t2));
        if (it.h2 !== null) pushPoint(chartH2, ts, Number(it.h2));
      });
    }
    // subscribe to new inserts
    supabase.channel('realtime-telemetry')
      .on('postgres_changes', { event: 'INSERT', schema: 'public', table: 'telemetry' }, payload => {
        const row = payload.new;
        const ts = new Date(row.ts).toLocaleTimeString();
        if (row.t1 !== null) { pushPoint(chartT1, ts, Number(row.t1)); valT1.textContent = Number(row.t1).toFixed(2) + ' °C'; }
        if (row.h1 !== null) { pushPoint(chartH1, ts, Number(row.h1)); valH1.textContent = Number(row.h1).toFixed(2) + ' %'; }
        if (row.t2 !== null) { pushPoint(chartT2, ts, Number(row.t2)); valT2.textContent = Number(row.t2).toFixed(2) + ' °C'; }
        if (row.h2 !== null) { pushPoint(chartH2, ts, Number(row.h2)); valH2.textContent = Number(row.h2).toFixed(2) + ' %'; }
      })
      .subscribe();
  } catch (err) {
    console.debug('Supabase realtime not available', err);
  }
}

// attempt to initialize Supabase realtime; non-blocking
initSupabaseRealtime();

function makeChart(ctx, label, color) {
  return new Chart(ctx, {
    type: 'line',
    data: { labels: [], datasets: [{ label, data: [], borderColor: color, backgroundColor: 'rgba(0,0,0,0)', tension: 0.15, pointRadius: 2, borderWidth: 2 }] },
    options: { scales: { x: { display: true }, y: { display: true } }, plugins: { legend: { display: false } }, responsive: true, maintainAspectRatio: false }
  });
}

const chartT1 = makeChart(ctxT1, 'Sensor1 T (°C)', 'rgb(255,99,71)');
const chartH1 = makeChart(ctxH1, 'Sensor1 H (%)', 'rgb(54,162,235)');
const chartT2 = makeChart(ctxT2, 'Sensor2 T (°C)', 'rgb(255,159,64)');
const chartH2 = makeChart(ctxH2, 'Sensor2 H (%)', 'rgb(75,192,192)');

// build control buttons for relays 1..6
for (let i=1;i<=6;i++) {
  const b = document.createElement('button');
  b.textContent = `Relay ${i}`;
  b.id = `btn-relay${i}`;
  b.style.margin = '4px';
  b.onclick = (() => {
    const idx = i;
    return () => {
      const current = relayStates[String(idx)] === '1' ? 1 : 0;
      const newVal = current ? 0 : 1;
      socket.emit('command', { relay: idx, value: newVal });
      // optimistic UI toggle while waiting ack
      setButtonState(idx, newVal ? 'on' : 'off');
    };
  })();
  controlsEl.appendChild(b);
}

socket.on('telemetry', (data) => {
  lastTelemetry = data;
  telemetryEl.textContent = JSON.stringify(data, null, 2);
  // update human-readable latest values
  if (data && typeof data.t1 !== 'undefined') valT1.textContent = Number(data.t1).toFixed(2) + ' °C';
  if (data && typeof data.h1 !== 'undefined') valH1.textContent = Number(data.h1).toFixed(2) + ' %';
  if (data && typeof data.t2 !== 'undefined') valT2.textContent = Number(data.t2).toFixed(2) + ' °C';
  if (data && typeof data.h2 !== 'undefined') valH2.textContent = Number(data.h2).toFixed(2) + ' %';
  rawEl.textContent = `telemetry: ${JSON.stringify(data)}` + '\n' + rawEl.textContent;
  // update charts for each sensor field if present
  const ts = new Date().toLocaleTimeString();
  if (data && typeof data.t1 !== 'undefined') {
    pushPoint(chartT1, ts, Number(data.t1));
  }
  if (data && typeof data.h1 !== 'undefined') {
    pushPoint(chartH1, ts, Number(data.h1));
  }
  if (data && typeof data.t2 !== 'undefined') {
    pushPoint(chartT2, ts, Number(data.t2));
  }
  if (data && typeof data.h2 !== 'undefined') {
    pushPoint(chartH2, ts, Number(data.h2));
  }
});

socket.on('relay-state', (obj) => {
  let relay = obj.relay;
  // accept either 'relay1' or '1'
  if (typeof relay === 'string' && relay.startsWith('relay')) relay = relay.substring('relay'.length);
  const state = String(obj.state);
  relayStates[String(relay)] = state;
  updateRelays();
  rawEl.textContent = `state ${relay}: ${state}` + '\n' + rawEl.textContent;
});

socket.on('mqtt', (m) => {
  rawEl.textContent = `${m.topic}: ${m.payload}` + '\n' + rawEl.textContent;
});

socket.on('connect', () => { connStatusEl.textContent = 'connected'; connStatusEl.style.color = 'green'; });
socket.on('disconnect', () => { connStatusEl.textContent = 'disconnected'; connStatusEl.style.color = 'red'; });
socket.on('command-ack', (a) => { rawEl.textContent = `command ack relay${a.relay}=${a.value}` + '\n' + rawEl.textContent; relayStates[String(a.relay)] = String(a.value); updateRelays(); });
socket.on('command-error', (e) => { rawEl.textContent = `command error ${e.error}` + '\n' + rawEl.textContent; });

function updateRelays() {
  relaysEl.innerHTML = '';
  // try to show relays 1..6
  for (let i=1;i<=6;i++) {
    const key = String(i);
    const state = relayStates[key] || '0';
    const d = document.createElement('div');
    d.className = 'relay ' + (state === '1' ? 'on' : 'off');
    d.textContent = `Relay ${i}: ${state === '1' ? 'ON' : 'OFF'}`;
    relaysEl.appendChild(d);
    // update control button visual
    setButtonState(i, state === '1' ? 'on' : 'off');
  }
}

function setButtonState(idx, state) {
  const b = document.getElementById(`btn-relay${idx}`);
  if (!b) return;
  if (state === 'on') {
    b.style.background = '#9f9';
  } else {
    b.style.background = '#f99';
  }
}

// helper to push a point into chart (keeps max points)
function pushPoint(chart, label, value) {
  chart.data.labels.push(label);
  chart.data.datasets[0].data.push(value);
  if (chart.data.labels.length > MAX_HISTORY) {
    chart.data.labels.shift();
    chart.data.datasets[0].data.shift();
  }
  chart.update();
}

// on load fetch history to populate charts
fetch('/history?n=100').then(r => r.json()).then(arr => {
  if (!Array.isArray(arr)) return;
  arr.forEach(it => {
    const ts = new Date(it.ts).toLocaleTimeString();
    if (typeof it.t1 !== 'undefined' && it.t1 !== null) pushPoint(chartT1, ts, Number(it.t1));
    if (typeof it.h1 !== 'undefined' && it.h1 !== null) pushPoint(chartH1, ts, Number(it.h1));
    if (typeof it.t2 !== 'undefined' && it.t2 !== null) pushPoint(chartT2, ts, Number(it.t2));
    if (typeof it.h2 !== 'undefined' && it.h2 !== null) pushPoint(chartH2, ts, Number(it.h2));
  });
}).catch(e=>{});

// --- Plans UI (localStorage-backed) ---------------------------------------------------------
const plansListEl = document.getElementById('plans-list');
const btnNewPlan = document.getElementById('btn-new-plan');
const planEditor = document.getElementById('plan-editor');
const planSim = document.getElementById('plan-sim');
const btnSavePlan = document.getElementById('btn-save-plan');
const btnCancelPlan = document.getElementById('btn-cancel-plan');
const planNameInput = document.getElementById('plan-name');
const planTypeInput = document.getElementById('plan-type');
const planActiveInput = document.getElementById('plan-active');
const daysGrid = document.getElementById('days-grid');
const simDaySelect = document.getElementById('sim-day');
const simTimeline = document.getElementById('sim-timeline');

let plans = []; // {id,name,type,active,days:[ {events: [ {id,h,m,duration,label} ] } x8 ]}
let editingPlanId = null;

function uid(prefix='p') { return prefix + '-' + Math.random().toString(36).slice(2,9); }

function loadPlans() {
  try { plans = JSON.parse(localStorage.getItem('plans_v1') || '[]'); } catch(e){ plans = []; }
  renderPlansList();
}

function savePlans() { localStorage.setItem('plans_v1', JSON.stringify(plans)); }

function renderPlansList() {
  plansListEl.innerHTML = '';
  if (plans.length === 0) {
    plansListEl.innerHTML = '<em style="color:var(--muted)">No hay planes. Crea uno nuevo.</em>';
    return;
  }
  const wrap = document.createElement('div'); wrap.className = 'plans-grid';
  plans.forEach(p => {
    const c = document.createElement('div'); c.className = 'plan-card';
    const h = document.createElement('h5'); h.textContent = p.name || '(sin nombre)'; c.appendChild(h);
    const meta = document.createElement('div'); meta.style.marginBottom='6px'; meta.innerHTML = `<strong>${p.type === 'riego' ? 'Riego 💧' : 'Iluminación 💡'}</strong> • ${p.active ? 'Activo' : 'Inactivo'}`;
    c.appendChild(meta);
    // visual 8-day summary
    const strip = document.createElement('div'); strip.className='day-strip';
    for (let i=0;i<8;i++){
      const chip = document.createElement('div'); chip.className='day-chip';
      const evcount = (p.days && p.days[i] && p.days[i].events) ? p.days[i].events.length : 0;
      chip.textContent = evcount>0?evcount:'';
      if (evcount>0) chip.classList.add('active');
      strip.appendChild(chip);
    }
    c.appendChild(strip);
    const actions = document.createElement('div'); actions.style.marginTop='8px';
    const ebtn = document.createElement('button'); ebtn.className='btn ghost'; ebtn.textContent='Editar'; ebtn.onclick = ()=>{ openEditor(p.id); };
    const dbtn = document.createElement('button'); dbtn.className='btn ghost'; dbtn.style.marginLeft='6px'; dbtn.textContent='Eliminar'; dbtn.onclick = ()=>{ if(confirm('Eliminar plan?')){ plans = plans.filter(x=>x.id!==p.id); savePlans(); renderPlansList(); } };
    const sbtn = document.createElement('button'); sbtn.className='btn'; sbtn.style.marginLeft='6px'; sbtn.textContent='Simular'; sbtn.onclick = ()=>{ openSim(p.id); };
    actions.appendChild(ebtn); actions.appendChild(dbtn); actions.appendChild(sbtn);
    c.appendChild(actions);
    wrap.appendChild(c);
  });
  plansListEl.appendChild(wrap);
}

function createEmptyPlan(){
  const days = Array.from({length:8}).map(()=>({events:[]}));
  return { id: uid('plan'), name:'Nuevo plan', type:'riego', active:true, days };
}

btnNewPlan.onclick = ()=>{ editingPlanId = null; showEditor(createEmptyPlan()); };

function showEditor(plan){
  planEditor.style.display='block'; planSim.style.display='none';
  planNameInput.value = plan.name||''; planTypeInput.value = plan.type||'riego'; planActiveInput.checked = !!plan.active;
  // build days grid
  daysGrid.innerHTML='';
  for(let d=0;d<8;d++){
    const col = document.createElement('div'); col.className='day-column'; col.dataset.day=d;
    const hdr = document.createElement('div'); hdr.style.marginBottom='6px'; hdr.innerHTML = `<strong>Día ${d}</strong> <button class="btn ghost" style="float:right" data-add>+ Evento</button>`;
    col.appendChild(hdr);
    const list = document.createElement('div'); list.className='events-list'; list.dataset.day=d;
    const events = (plan.days && plan.days[d] && plan.days[d].events) ? plan.days[d].events.slice() : [];
    events.forEach(ev=>{ appendEventElement(list, ev, plan); });
    col.appendChild(list);
    daysGrid.appendChild(col);
  }
  // wire add buttons
  daysGrid.querySelectorAll('[data-add]').forEach(b=>{
    b.onclick = (ev)=>{
      const day = Number(ev.target.closest('.day-column').dataset.day);
      const newEv = { id: uid('ev'), h:12, m:0, duration:60000, label:'' };
      const list = ev.target.closest('.day-column').querySelector('.events-list');
      appendEventElement(list, newEv, plan); list.scrollTop = list.scrollHeight; }
  });
  // store editing plan temporarily in DOM element
  planEditor.dataset.plan = JSON.stringify(plan);
}

function appendEventElement(list, ev, plan){
  const item = document.createElement('div'); item.className='event-item'; item.draggable=true; item.dataset.ev = JSON.stringify(ev);
  const txt = document.createElement('div'); txt.innerHTML = `<div class="event-meta">${(ev.label||'')} ${pad(ev.h)}:${pad(ev.m)} • ${msToFriendly(ev.duration)}</div>`;
  const controls = document.createElement('div');
  const edit = document.createElement('button'); edit.className='btn ghost'; edit.textContent='✎'; edit.onclick = ()=>{ editEvent(item, ev, plan); };
  const del = document.createElement('button'); del.className='btn ghost'; del.style.marginLeft='6px'; del.textContent='✖'; del.onclick = ()=>{ if(confirm('Eliminar evento?')) item.remove(); };
  controls.appendChild(edit); controls.appendChild(del);
  item.appendChild(txt); item.appendChild(controls);
  // drag handlers
  item.addEventListener('dragstart', (e)=>{ e.dataTransfer.setData('text/plain', JSON.stringify(ev)); item.style.opacity='0.5'; });
  item.addEventListener('dragend', ()=>{ item.style.opacity='1'; });
  list.appendChild(item);
  // allow drops on list to reorder
  list.addEventListener('dragover', (e)=>{ e.preventDefault(); });
  list.addEventListener('drop', (e)=>{ e.preventDefault(); const data = e.dataTransfer.getData('text/plain'); try{ const dropped = JSON.parse(data); const newItem = document.createElement('div'); /* simple: append as new */ appendEventElement(list, dropped, plan); }catch(err){} });
}

function editEvent(item, ev, plan){
  const h = prompt('Hora (HH:MM)', pad(ev.h)+':'+pad(ev.m)); if(!h) return; const parts = h.split(':'); ev.h = Number(parts[0]||0); ev.m = Number(parts[1]||0);
  const dur = prompt('Duración en segundos', String(Math.floor(ev.duration/1000))); if(!dur) return; ev.duration = Number(dur)*1000;
  const lbl = prompt('Etiqueta (opcional)', ev.label||''); ev.label = lbl||'';
  // update display
  item.querySelector('.event-meta').textContent = `${ev.label||''} ${pad(ev.h)}:${pad(ev.m)} • ${msToFriendly(ev.duration)}`;
}

function pad(n){ return (n<10? '0'+n : ''+n); }
function msToFriendly(ms){ if(ms%3600000===0) return (ms/3600000)+' h'; if(ms%60000===0) return (ms/60000)+' min'; if(ms%1000===0) return (ms/1000)+' s'; return ms+' ms'; }

btnCancelPlan.onclick = ()=>{ planEditor.style.display='none'; }

btnSavePlan.onclick = ()=>{
  const raw = planEditor.dataset.plan ? JSON.parse(planEditor.dataset.plan) : createEmptyPlan();
  raw.name = planNameInput.value || raw.name;
  raw.type = planTypeInput.value;
  raw.active = planActiveInput.checked;
  // gather events from DOM
  raw.days = Array.from(daysGrid.querySelectorAll('.events-list')).map(list=>{
    const evs = Array.from(list.children).map(ch=>JSON.parse(ch.dataset.ev));
    return { events: evs };
  });
  if (!raw.id) raw.id = uid('plan');
  // replace or push
  const idx = plans.findIndex(x=>x.id===raw.id);
  if (idx>=0) plans[idx]=raw; else plans.push(raw);
  savePlans(); renderPlansList(); planEditor.style.display='none';
}

function openEditor(planId){
  const p = plans.find(x=>x.id===planId); if(!p) return; editingPlanId = planId; showEditor(JSON.parse(JSON.stringify(p)));
}

function openSim(planId){
  const p = plans.find(x=>x.id===planId); if(!p) return; planEditor.style.display='none'; planSim.style.display='block'; simDaySelect.innerHTML='';
  for(let d=0;d<8;d++){ const o=document.createElement('option'); o.value=d; o.textContent='Día '+d; simDaySelect.appendChild(o); }
  simDaySelect.onchange = ()=>{ renderSim(p, Number(simDaySelect.value)); };
  renderSim(p, 0);
}

function renderSim(plan, day){
  simTimeline.innerHTML='';
  const events = (plan.days && plan.days[day] && plan.days[day].events) ? plan.days[day].events : [];
  // draw 24h timeline width
  const W = simTimeline.clientWidth || 800;
  events.forEach(ev=>{
    const start = (ev.h*3600 + ev.m*60) * 1000;
    const left = (start / (24*3600*1000)) * 100;
    const width = Math.max(1, (ev.duration / (24*3600*1000)) * 100);
    const block = document.createElement('div'); block.className='sim-block '+(plan.type==='riego'?'mint':'amber');
    block.style.left = left + '%'; block.style.width = width + '%'; block.title = `${pad(ev.h)}:${pad(ev.m)} • ${msToFriendly(ev.duration)}`;
    block.textContent = `${pad(ev.h)}:${pad(ev.m)}`;
    block.onmouseover = (e)=>{ const t = document.createElement('div'); t.style.position='absolute'; t.style.left=e.clientX+'px'; };
    simTimeline.appendChild(block);
  });
}

// initialize
loadPlans();
