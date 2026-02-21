const socket = io();

const telemetryEl = document.getElementById('telemetry');
const rawEl = document.getElementById('raw');
const relaysEl = document.getElementById('relays');
const connStatusEl = document.getElementById('conn-status');
const controlsEl = document.getElementById('controls');

const ctxT1 = document.getElementById('chart-t1').getContext('2d');
const ctxH1 = document.getElementById('chart-h1').getContext('2d');
const ctxT2 = document.getElementById('chart-t2').getContext('2d');
const ctxH2 = document.getElementById('chart-h2').getContext('2d');

let relayStates = {};
const MAX_HISTORY = 100;

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
