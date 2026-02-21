const express = require('express');
const http = require('http');
const { Server } = require('socket.io');
const mqtt = require('mqtt');
const fs = require('fs');
const path = require('path');
const sqlite3 = require('sqlite3').verbose();

const app = express();
const server = http.createServer(app);
const io = new Server(server);

// Config via env
const MQTT_BROKER = process.env.MQTT_BROKER || 'mqtt://test.mosquitto.org:1883';
const MQTT_TELEMETRY_TOPIC = process.env.MQTT_TELEMETRY_TOPIC || 'esp32s3/telemetry';
const MQTT_STATE_TOPIC = process.env.MQTT_STATE_TOPIC || 'esp32s3/state/#';

app.use(express.static('public'));
app.use(express.json());

// ensure data directory exists
const DATA_DIR = path.join(__dirname, 'data');
if (!fs.existsSync(DATA_DIR)) fs.mkdirSync(DATA_DIR, { recursive: true });

// SQLite DB
const DB_FILE = path.join(DATA_DIR, 'telemetry.db');
const db = new sqlite3.Database(DB_FILE);
db.serialize(() => {
  db.run(`CREATE TABLE IF NOT EXISTS telemetry (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    ts INTEGER,
    t1 REAL,
    h1 REAL,
    t2 REAL,
    h2 REAL
  )`);
});

// HTTP endpoint to fetch last N telemetry records from DB
app.get('/history', (req, res) => {
  const n = parseInt(req.query.n || '50', 10);
  db.all('SELECT ts, t1, h1, t2, h2 FROM telemetry ORDER BY ts DESC LIMIT ?', [n], (err, rows) => {
    if (err) {
      console.error('DB history error', err);
      return res.json([]);
    }
    // return chronological order
    res.json(rows.reverse());
  });
});

io.on('connection', (socket) => {
  console.log('Web client connected');
  socket.on('disconnect', () => console.log('Web client disconnected'));
  // Accept command events from web client and publish to MQTT
  socket.on('command', (cmd) => {
    try {
      const relay = cmd.relay;
      const value = String(cmd.value);
      const topic = `esp32s3/command/relay${relay}`;
      console.log('Publishing command to MQTT', topic, value);
      // publish and acknowledge on publish callback
      client.publish(topic, value, { qos: 0 }, (err) => {
        if (err) {
          console.error('Publish error', err);
          socket.emit('command-error', { error: err.toString() });
        } else {
          socket.emit('command-ack', { relay, value });
        }
      });
    } catch (e) {
      console.error('Failed to publish command', e);
      socket.emit('command-error', { error: e.toString() });
    }
  });
});

// Connect to MQTT
console.log('Connecting to MQTT broker', MQTT_BROKER);
const client = mqtt.connect(MQTT_BROKER);

client.on('connect', () => {
  console.log('Connected to MQTT broker');
  client.subscribe(MQTT_TELEMETRY_TOPIC, { qos: 0 }, (err) => {
    if (err) console.error('Subscribe telemetry error', err);
  });
  client.subscribe(MQTT_STATE_TOPIC, { qos: 0 }, (err) => {
    if (err) console.error('Subscribe state error', err);
  });
});

client.on('message', (topic, message) => {
  const payload = message.toString();
  console.log('MQTT', topic, payload);
  if (topic === MQTT_TELEMETRY_TOPIC) {
    // try to parse JSON
    let obj = payload;
    try { obj = JSON.parse(payload); } catch(e) { obj = payload; }
    // insert into sqlite DB with timestamp
    try {
      const ts = Date.now();
      const t1 = (typeof obj === 'object' && typeof obj.t1 !== 'undefined') ? Number(obj.t1) : null;
      const h1 = (typeof obj === 'object' && typeof obj.h1 !== 'undefined') ? Number(obj.h1) : null;
      const t2 = (typeof obj === 'object' && typeof obj.t2 !== 'undefined') ? Number(obj.t2) : null;
      const h2 = (typeof obj === 'object' && typeof obj.h2 !== 'undefined') ? Number(obj.h2) : null;
      db.run('INSERT INTO telemetry (ts,t1,h1,t2,h2) VALUES (?,?,?,?,?)', [ts,t1,h1,t2,h2]);
    } catch (e) { console.error('Failed to insert telemetry', e); }
    io.emit('telemetry', obj);
  } else if (topic.startsWith('esp32s3/state/')) {
    const parts = topic.split('/');
    let relay = parts[parts.length-1];
    // normalize relay id: 'relay1' -> '1'
    if (typeof relay === 'string' && relay.startsWith('relay')) {
      relay = relay.substring('relay'.length);
    }
    io.emit('relay-state', { relay, state: payload });
  } else {
    // other topics
    io.emit('mqtt', { topic, payload });
  }
});

// Debug/test endpoint to publish a relay command via HTTP
app.post('/api/command', (req, res) => {
  try {
    const relay = req.body.relay;
    const value = String(req.body.value);
    if (typeof relay === 'undefined' || typeof value === 'undefined') return res.status(400).json({ error: 'relay and value required' });
    const topic = `esp32s3/command/relay${relay}`;
    console.log('HTTP test publish', topic, value);
    client.publish(topic, value, { qos: 0 }, (err) => {
      if (err) {
        console.error('HTTP publish error', err);
        return res.status(500).json({ error: err.toString() });
      }
      return res.json({ ok: true, topic, value });
    });
  } catch (e) {
    console.error('HTTP /api/command error', e);
    res.status(500).json({ error: e.toString() });
  }
});

const PORT = process.env.PORT || 3000;
server.listen(PORT, () => {
  console.log(`Server running on http://localhost:${PORT}`);
});
