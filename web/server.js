const express = require('express');
const http = require('http');
const { Server } = require('socket.io');
const mqtt = require('mqtt');
const fs = require('fs');
const path = require('path');
const sqlite3 = require('sqlite3').verbose();
const { Pool } = require('pg');

const DATABASE_URL = process.env.DATABASE_URL || '';
let usingPg = false;
let pgPool = null;

const app = express();
const server = http.createServer(app);
const io = new Server(server, {
  cors: {
    origin: process.env.FRONTEND_URL || '*',
    methods: ['GET', 'POST']
  }
});
// store last-known relay states so new web clients can be initialized
const lastRelayStates = {};
// pending file responses map: key = `${deviceId}:${reqId}` => { resolve, timer }
const pendingFileResponses = {};

// Normalize human-friendly states to device values for publishing
function normalizeStateForDevice(val) {
  if (val === null || val === undefined) return '';
  const s = String(val).trim();
  if (s === '') return s;
  const lower = s.toLowerCase();
  if (lower === 'on' || lower === 'true') return '1';
  if (lower === 'off' || lower === 'false') return '0';
  // numeric strings (1,0, etc.) pass through
  if (/^[+-]?\d+(?:\.\d+)?$/.test(s)) return s;
  return s;
}

// Config via env
const MQTT_BROKER = process.env.MQTT_BROKER || 'mqtt://test.mosquitto.org:1883';
const MQTT_TELEMETRY_TOPIC = process.env.MQTT_TELEMETRY_TOPIC || 'esp32s3/telemetry';
const MQTT_STATE_TOPIC = process.env.MQTT_STATE_TOPIC || 'esp32s3/state/#';
const MQTT_FILE_RESPONSE_TOPIC = 'esp32s3/file/response/#';
// When set, forward frontend API calls to Supabase Edge Functions base URL
// Example: https://xyz.supabase.co/functions/v1
const SUPABASE_FUNCTIONS_BASE = process.env.SUPABASE_FUNCTIONS_BASE || '';

app.use(express.static('public'));
app.use(express.json());

// ensure data directory exists
const DATA_DIR = path.join(__dirname, 'data');
if (!fs.existsSync(DATA_DIR)) fs.mkdirSync(DATA_DIR, { recursive: true });

// Database initialization: Postgres via DATABASE_URL (Supabase) OR SQLite fallback
if (DATABASE_URL) {
  usingPg = true;
  pgPool = new Pool({ connectionString: DATABASE_URL, ssl: { rejectUnauthorized: false } });
  (async () => {
    try {
      // Create tables if they don't exist
      await pgPool.query(`CREATE TABLE IF NOT EXISTS telemetry (
        id SERIAL PRIMARY KEY,
        ts BIGINT,
        device_ts BIGINT,
        t1 REAL,
        h1 REAL,
        t2 REAL,
        h2 REAL
      )`);
      await pgPool.query(`CREATE TABLE IF NOT EXISTS schedules (
        id SERIAL PRIMARY KEY,
        event_ts BIGINT,
        relay INTEGER,
        duration INTEGER,
        repeat8 INTEGER DEFAULT 0,
        created_ts BIGINT
      )`);
      console.log('Connected to Postgres and ensured tables exist');
    } catch (e) {
      console.error('Failed to prepare Postgres schema', e);
    }
  })();
} else {
  // SQLite DB
  const DB_FILE = path.join(DATA_DIR, 'telemetry.db');
  var db = new sqlite3.Database(DB_FILE);
  db.serialize(() => {
    db.run(`CREATE TABLE IF NOT EXISTS telemetry (
      id INTEGER PRIMARY KEY AUTOINCREMENT,
      ts INTEGER,
      device_ts INTEGER,
      t1 REAL,
      h1 REAL,
      t2 REAL,
      h2 REAL
    )`);
    db.run(`CREATE TABLE IF NOT EXISTS schedules (
      id INTEGER PRIMARY KEY AUTOINCREMENT,
      event_ts INTEGER,
      relay INTEGER,
      duration INTEGER,
      repeat8 INTEGER DEFAULT 0,
      created_ts INTEGER
    )`);
    // Persist relay states for SQLite fallback
    db.run(`CREATE TABLE IF NOT EXISTS relay_states (
      id INTEGER PRIMARY KEY AUTOINCREMENT,
      relay INTEGER NOT NULL,
      state TEXT NOT NULL,
      updated_ts INTEGER
    )`);
    // Ensure legacy DBs have the device_ts column
    db.all(`PRAGMA table_info(telemetry)`, (err, rows) => {
      if (err) {
        console.error('PRAGMA table_info error', err);
        return;
      }
      const hasDeviceTs = rows && rows.some(r => r.name === 'device_ts');
      if (!hasDeviceTs) {
        console.log('Altering telemetry table to add device_ts column');
        db.run('ALTER TABLE telemetry ADD COLUMN device_ts INTEGER', (e) => { if (e) console.error('Failed to add device_ts column', e); });
      }
    });
    // Ensure schedules table has repeat8 column for 8-day repeating schedules
    db.all(`PRAGMA table_info(schedules)`, (err2, rows2) => {
      if (err2) { console.error('PRAGMA table_info(schedules) error', err2); return; }
      const hasRepeat8 = rows2 && rows2.some(r => r.name === 'repeat8');
      if (!hasRepeat8) {
        console.log('Altering schedules table to add repeat8 column');
        db.run('ALTER TABLE schedules ADD COLUMN repeat8 INTEGER DEFAULT 0', (e) => { if (e) console.error('Failed to add repeat8 column', e); });
      }
    });
  });
}

// HTTP endpoint to fetch last N telemetry records from DB
app.get('/history', (req, res) => {
  const n = parseInt(req.query.n || '50', 10);
  if (usingPg) {
    (async () => {
      try {
        const r = await pgPool.query('SELECT ts, t1, h1, t2, h2 FROM telemetry ORDER BY ts DESC LIMIT $1', [n]);
        return res.json(r.rows.reverse());
      } catch (e) {
        console.error('PG history error', e);
        return res.json([]);
      }
    })();
  } else {
    db.all('SELECT ts, t1, h1, t2, h2 FROM telemetry ORDER BY ts DESC LIMIT ?', [n], (err, rows) => {
      if (err) {
        console.error('DB history error', err);
        return res.json([]);
      }
      // return chronological order
      res.json(rows.reverse());
    });
  }
});

// Request a file from a device over MQTT. Returns JSON with content_base64 and content_type.
app.post('/api/device_file', async (req, res) => {
  try {
    const { device_id, path } = req.body;
    if (!device_id || !path) return res.status(400).json({ error: 'device_id and path required' });
    const reqId = `${Date.now().toString(36)}-${Math.random().toString(36).slice(2,8)}`;
    const topicReq = `esp32s3/file/request/${device_id}`;
    const payload = JSON.stringify({ path, req_id: reqId });
    const key = `${device_id}:${reqId}`;
    // publish request
    client.publish(topicReq, payload, { qos: 0 }, (err) => {
      if (err) console.error('Publish file request error', err);
    });
    // wait for response (with timeout)
    const result = await new Promise((resolve, reject) => {
      const timer = setTimeout(() => { delete pendingFileResponses[key]; reject(new Error('timeout')); }, 10000);
      pendingFileResponses[key] = { resolve, timer };
    });
    return res.json(result);
  } catch (e) {
    console.error('POST /api/device_file error', e);
    return res.status(500).json({ error: e.toString() });
  }
});

// Schedule endpoints: accept array of events {ts, relay, duration}
app.post('/api/schedule', (req, res) => {
  // If SUPABASE_FUNCTIONS_BASE is configured, proxy the request to Supabase Edge Function
  if (SUPABASE_FUNCTIONS_BASE) {
    (async () => {
      try {
        const target = `${SUPABASE_FUNCTIONS_BASE}/schedules`;
        const fetchRes = await fetch(target, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(req.body)
        });
        const text = await fetchRes.text();
        // After persisting to Supabase, also publish to MQTT so devices get the update
        try {
          const events = req.body;
          if (Array.isArray(events)) {
            let payload = '';
            events.forEach(ev => {
              const ets = Number(ev.ts || ev.t || 0);
              const relay = Number(ev.relay || 1);
              const dur = Number(ev.duration || ev.d || 0);
              const repeat8 = ev.repeat8 ? 1 : 0;
              if (ets > 0) payload += `${ets},${relay},${dur},${repeat8}\n`;
            });
            if (payload.length > 0) {
              client.publish('esp32s3/schedule', payload, { qos: 0 }, (err) => {
                if (err) console.error('Failed to publish schedule payload:', err);
                else console.log('Published schedule payload to esp32s3/schedule (proxied)');
              });
            }
          }
        } catch (e) { console.error('Post-proxy MQTT publish error', e); }
        res.status(fetchRes.status).type(fetchRes.headers.get('content-type') || 'application/json').send(text);
      } catch (err) {
        console.error('Proxy to Supabase /schedules failed', err);
        res.status(502).json({ error: 'Bad Gateway' });
      }
    })();
    return;
  }

  try {
    const events = req.body; // expect array
    if (!Array.isArray(events)) return res.status(400).json({ error: 'expect array of events' });
    // store into DB for persistence
    const now = Date.now();
    if (usingPg) {
      (async () => {
        const client = await pgPool.connect();
        try {
          await client.query('BEGIN');
          for (const ev of events) {
            const ets = Number(ev.ts || ev.t || 0);
            const relay = Number(ev.relay || 1);
            const dur = Number(ev.duration || ev.d || 0);
            const repeat8 = ev.repeat8 ? 1 : 0;
            if (ets > 0) await client.query('INSERT INTO schedules (event_ts, relay, duration, repeat8, created_ts) VALUES ($1,$2,$3,$4,$5)', [ets, relay, dur, repeat8, now]);
          }
          await client.query('COMMIT');
        } catch (e) {
          await client.query('ROLLBACK');
          console.error('PG insert schedules error', e);
        } finally {
          client.release();
        }
      })();
    } else {
      const stmt = db.prepare('INSERT INTO schedules (event_ts, relay, duration, repeat8, created_ts) VALUES (?,?,?,?,?)');
      events.forEach(ev => {
        const ets = Number(ev.ts || ev.t || 0);
        const relay = Number(ev.relay || 1);
        const dur = Number(ev.duration || ev.d || 0);
        const repeat8 = ev.repeat8 ? 1 : 0;
        if (ets > 0) stmt.run(ets, relay, dur, repeat8, now);
      });
      stmt.finalize();
    }
    // publish schedule to MQTT as CSV lines: ts,relay,duration\n...
    let payload = '';
    events.forEach(ev => {
      const ets = Number(ev.ts || ev.t || 0);
      const relay = Number(ev.relay || 1);
      const dur = Number(ev.duration || ev.d || 0);
      const repeat8 = ev.repeat8 ? 1 : 0;
      if (ets > 0) payload += `${ets},${relay},${dur},${repeat8}\n`;
    });
    if (payload.length > 0) {
      console.log('Publishing schedule to esp32s3/schedule, payload length=', payload.length);
      // Log a shortened preview to avoid huge logs
      console.log('Schedule payload preview:', payload.split('\n').slice(0,5).join('\n'));
      client.publish('esp32s3/schedule', payload, { qos: 0 }, (err) => {
        if (err) {
          console.error('Failed to publish schedule payload:', err);
        } else {
          console.log('Published schedule payload to esp32s3/schedule');
        }
      });
    }
    return res.json({ ok: true });
  } catch (e) {
    console.error('POST /api/schedule error', e);
    return res.status(500).json({ error: e.toString() });
  }
});

// Return upcoming scheduled events from DB
app.get('/api/schedule', (req, res) => {
  if (SUPABASE_FUNCTIONS_BASE) {
    (async () => {
      try {
        const target = `${SUPABASE_FUNCTIONS_BASE}/schedules`;
        const fetchRes = await fetch(target, { method: 'GET', headers: { Accept: 'application/json' } });
        const json = await fetchRes.json();
        return res.status(fetchRes.status).json(json);
      } catch (err) {
        console.error('Proxy GET /api/schedule to Supabase failed', err);
        return res.status(502).json({ error: 'Bad Gateway' });
      }
    })();
    return;
  }
  if (usingPg) {
    (async () => {
      try {
        const r = await pgPool.query('SELECT id,event_ts as ts,relay,duration,repeat8,created_ts FROM schedules ORDER BY event_ts ASC LIMIT $1', [100]);
        return res.json(r.rows);
      } catch (e) {
        console.error('PG get schedules error', e);
        return res.status(500).json({ error: e.toString() });
      }
    })();
  } else {
    db.all('SELECT id,event_ts as ts,relay,duration,repeat8,created_ts FROM schedules ORDER BY event_ts ASC LIMIT ?', [100], (err, rows) => {
      if (err) return res.status(500).json({ error: err.toString() });
      res.json(rows);
    });
  }
});

// Device schedule CSV proxy endpoints (for the ESP device to GET/POST CSV)
app.get('/api/device_schedule', async (req, res) => {
  if (!SUPABASE_FUNCTIONS_BASE) return res.status(404).send('Not configured');
  try {
    const target = `${SUPABASE_FUNCTIONS_BASE}/device_schedule`;
    const fetchRes = await fetch(target, { method: 'GET', headers: { Accept: 'text/csv' } });
    const text = await fetchRes.text();
    res.status(fetchRes.status).type('text/csv').send(text);
  } catch (e) {
    console.error('Proxy GET /api/device_schedule failed', e);
    res.status(502).json({ error: 'Bad Gateway' });
  }
});

app.post('/api/device_schedule', express.text({ type: '*/*' }), async (req, res) => {
  if (!SUPABASE_FUNCTIONS_BASE) return res.status(404).send('Not configured');
  try {
    const target = `${SUPABASE_FUNCTIONS_BASE}/device_schedule`;
    const fetchRes = await fetch(target, { method: 'POST', headers: { 'Content-Type': 'text/plain' }, body: req.body });
    const text = await fetchRes.text();
    res.status(fetchRes.status).type(fetchRes.headers.get('content-type') || 'text/plain').send(text);
  } catch (e) {
    console.error('Proxy POST /api/device_schedule failed', e);
    res.status(502).json({ error: 'Bad Gateway' });
  }
});

io.on('connection', (socket) => {
  console.log('Web client connected');
  // initialize UI with last-known relay states
  Object.keys(lastRelayStates).forEach((relay) => {
    socket.emit('relay-state', { relay, state: lastRelayStates[relay] });
  });
  socket.on('disconnect', () => console.log('Web client disconnected'));
  // Accept command events from web client and publish to MQTT
  socket.on('command', (cmd) => {
    try {
      const relay = cmd.relay;
      const value = normalizeStateForDevice(cmd.value);
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
  // subscribe to device-published schedule loaded notifications
  client.subscribe('esp32s3/schedule/loaded', { qos: 0 }, (err) => {
    if (err) console.error('Subscribe schedule/loaded error', err);
  });
  // subscribe to file responses from devices
  client.subscribe(MQTT_FILE_RESPONSE_TOPIC, { qos: 0 }, (err) => {
    if (err) console.error('Subscribe file response error', err);
  });
});

client.on('message', (topic, message) => {
  const payload = message.toString();
  console.log(new Date().toISOString(), 'MQTT message ->', topic);
  try {
    // try to pretty-print JSON payloads
    const parsed = JSON.parse(payload);
    console.log('MQTT payload (json):', parsed);
  } catch (e) {
    console.log('MQTT payload (raw):', payload);
  }
  // File response handling: topic format esp32s3/file/response/<device_id>/<req_id>
  if (topic.startsWith('esp32s3/file/response/')) {
    // extract req_id
    const parts = topic.split('/');
    // parts[0]=esp32s3, [1]=file, [2]=response, [3]=device_id, [4]=req_id
    const deviceId = parts[3];
    const reqId = parts[4];
    try {
      const obj = JSON.parse(payload);
      const key = `${deviceId}:${reqId}`;
      if (pendingFileResponses[key]) {
        pendingFileResponses[key].resolve(obj);
        clearTimeout(pendingFileResponses[key].timer);
        delete pendingFileResponses[key];
      }
    } catch (e) {
      console.error('Invalid JSON in file response', e);
    }
    return;
  }
  if (topic === MQTT_TELEMETRY_TOPIC) {
    // try to parse JSON
    let obj = payload;
    try { obj = JSON.parse(payload); } catch(e) { obj = payload; }
    // insert into sqlite DB with timestamp
    try {
      const server_ts = Date.now();
      const device_ts = (typeof obj === 'object' && typeof obj.t !== 'undefined') ? Number(obj.t) : null;
      const t1 = (typeof obj === 'object' && typeof obj.t1 !== 'undefined') ? Number(obj.t1) : null;
      const h1 = (typeof obj === 'object' && typeof obj.h1 !== 'undefined') ? Number(obj.h1) : null;
      const t2 = (typeof obj === 'object' && typeof obj.t2 !== 'undefined') ? Number(obj.t2) : null;
      const h2 = (typeof obj === 'object' && typeof obj.h2 !== 'undefined') ? Number(obj.h2) : null;
      if (usingPg) {
        (async () => {
          try {
            await pgPool.query('INSERT INTO telemetry (ts,device_ts,t1,h1,t2,h2) VALUES ($1,$2,$3,$4,$5,$6)', [server_ts, device_ts, t1, h1, t2, h2]);
          } catch (e) { console.error('PG insert telemetry error', e); }
        })();
      } else {
        db.run('INSERT INTO telemetry (ts,device_ts,t1,h1,t2,h2) VALUES (?,?,?,?,?,?)', [server_ts, device_ts, t1,h1,t2,h2]);
      }
    } catch (e) { console.error('Failed to insert telemetry', e); }
    io.emit('telemetry', obj);
  } else if (topic.startsWith('esp32s3/state/')) {
    const parts = topic.split('/');
    let relay = parts[parts.length-1];
    // normalize relay id: 'relay1' -> '1'
    if (typeof relay === 'string' && relay.startsWith('relay')) {
      relay = relay.substring('relay'.length);
    }
    // normalize state, update in-memory last-known state and notify clients
    const stateText = String(payload);
    const publishVal = normalizeStateForDevice(stateText);
    lastRelayStates[String(relay)] = publishVal;
    io.emit('relay-state', { relay, state: publishVal });
    // Persist relay state into DB (Postgres or SQLite)
    try {
      const relayId = Number(relay);
      // const stateText already defined above
      const updatedTs = Date.now();
      if (!Number.isFinite(relayId)) {
        // ignore non-numeric relay identifiers
      } else if (usingPg) {
        (async () => {
          try {
            const sel = await pgPool.query('SELECT id FROM relay_states WHERE relay=$1', [relayId]);
            if (sel.rows.length > 0) {
              await pgPool.query('UPDATE relay_states SET state=$1, updated_ts=$2 WHERE relay=$3', [publishVal, updatedTs, relayId]);
            } else {
              await pgPool.query('INSERT INTO relay_states (relay, state, updated_ts) VALUES ($1,$2,$3)', [relayId, publishVal, updatedTs]);
            }
          } catch (e) {
            console.error('PG relay_states upsert error', e);
          }
        })();
      } else {
        db.get('SELECT id FROM relay_states WHERE relay=?', [relayId], (err, row) => {
          if (err) { console.error('SQLite relay_states select error', err); return; }
          if (row) {
            db.run('UPDATE relay_states SET state=?, updated_ts=? WHERE relay=?', [publishVal, updatedTs, relayId], (e) => { if (e) console.error('SQLite relay_states update error', e); });
          } else {
            db.run('INSERT INTO relay_states (relay, state, updated_ts) VALUES (?,?,?)', [relayId, publishVal, updatedTs], (e) => { if (e) console.error('SQLite relay_states insert error', e); });
          }
        });
      }
    } catch (e) {
      console.error('Failed to persist relay state', e);
    }
  } else {
    // special-case: device published its loaded schedule file -> sync DB
    if (topic === 'esp32s3/schedule/loaded') {
      const csv = payload;
      console.log('Received schedule/loaded from device, syncing DB');
      // parse CSV lines: ts,relay,duration,repeat8? and replace schedules table
      if (usingPg) {
        (async () => {
          const clientPg = await pgPool.connect();
          try {
            await clientPg.query('BEGIN');
            await clientPg.query('DELETE FROM schedules');
            const now = Date.now();
            const lines = csv.split(/\r?\n/);
            for (const ln of lines) {
              if (!ln || ln.trim().length === 0) continue;
              const parts = ln.split(',');
              if (parts.length < 3) continue;
              const ets = Number(parts[0] || 0);
              const relay = Number(parts[1] || 1);
              const dur = Number(parts[2] || 0);
              const repeat8 = parts.length >= 4 ? (Number(parts[3]) ? 1 : 0) : 0;
              await clientPg.query('INSERT INTO schedules (event_ts, relay, duration, repeat8, created_ts) VALUES ($1,$2,$3,$4,$5)', [ets, relay, dur, repeat8, now]);
            }
            await clientPg.query('COMMIT');
          } catch (e) {
            await clientPg.query('ROLLBACK');
            console.error('PG sync schedules error', e);
          } finally {
            clientPg.release();
          }
        })();
      } else {
        db.serialize(() => {
          db.run('DELETE FROM schedules', (err) => {
            if (err) console.error('Failed to clear schedules table', err);
          });
          const stmt = db.prepare('INSERT INTO schedules (event_ts, relay, duration, repeat8, created_ts) VALUES (?,?,?,?,?)');
          const now = Date.now();
          const lines = csv.split(/\r?\n/);
          lines.forEach((ln) => {
            if (!ln || ln.trim().length === 0) return;
            const parts = ln.split(',');
            if (parts.length < 3) return;
            const ets = Number(parts[0] || 0);
            const relay = Number(parts[1] || 1);
            const dur = Number(parts[2] || 0);
            const repeat8 = parts.length >= 4 ? (Number(parts[3]) ? 1 : 0) : 0;
            // store epoch ms if looks like large number (>1e12) else store as relative ts
            stmt.run(ets, relay, dur, repeat8, now);
          });
          stmt.finalize();
        });
      }
      // also emit mqtt event to web clients so UI can react if needed
      io.emit('mqtt', { topic, payload });
    } else {
      // other topics
      io.emit('mqtt', { topic, payload });
    }
  }
});

// Debug/test endpoint to publish a relay command via HTTP
app.post('/api/command', (req, res) => {
  try {
    const relay = req.body.relay;
    const rawValue = req.body.value;
    if (typeof relay === 'undefined' || typeof rawValue === 'undefined') return res.status(400).json({ error: 'relay and value required' });
    const value = normalizeStateForDevice(rawValue);
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

// Relay states API
app.get('/api/relay_states', (req, res) => {
  if (usingPg) {
    (async () => {
      try {
        const r = await pgPool.query('SELECT relay, state, updated_ts FROM relay_states ORDER BY relay');
        return res.json(r.rows);
      } catch (e) {
        console.error('PG get relay_states error', e);
        return res.status(500).json({ error: e.toString() });
      }
    })();
  } else {
    db.all('SELECT relay, state, updated_ts FROM relay_states ORDER BY relay', [], (err, rows) => {
      if (err) {
        console.error('SQLite get relay_states error', err);
        return res.status(500).json({ error: err.toString() });
      }
      res.json(rows);
    });
  }
});

app.get('/api/relay_states/:relay', (req, res) => {
  const relayId = Number(req.params.relay);
  if (!Number.isFinite(relayId)) return res.status(400).json({ error: 'invalid relay id' });
  if (usingPg) {
    (async () => {
      try {
        const r = await pgPool.query('SELECT relay, state, updated_ts FROM relay_states WHERE relay=$1 LIMIT 1', [relayId]);
        if (r.rows.length === 0) return res.status(404).json({});
        return res.json(r.rows[0]);
      } catch (e) {
        console.error('PG get relay_state error', e);
        return res.status(500).json({ error: e.toString() });
      }
    })();
  } else {
    db.get('SELECT relay, state, updated_ts FROM relay_states WHERE relay=? LIMIT 1', [relayId], (err, row) => {
      if (err) {
        console.error('SQLite get relay_state error', err);
        return res.status(500).json({ error: err.toString() });
      }
      if (!row) return res.status(404).json({});
      res.json(row);
    });
  }
});

// Create or update relay state (upsert-like)
app.post('/api/relay_states', (req, res) => {
  const relayId = Number(req.body.relay);
  const stateText = String(req.body.state || '');
  if (!Number.isFinite(relayId) || !stateText) return res.status(400).json({ error: 'relay and state required' });
  const publishVal = normalizeStateForDevice(stateText);
  const updatedTs = Date.now();
  if (usingPg) {
    (async () => {
      try {
        const sel = await pgPool.query('SELECT id FROM relay_states WHERE relay=$1', [relayId]);
        if (sel.rows.length > 0) {
          await pgPool.query('UPDATE relay_states SET state=$1, updated_ts=$2 WHERE relay=$3', [publishVal, updatedTs, relayId]);
        } else {
          await pgPool.query('INSERT INTO relay_states (relay, state, updated_ts) VALUES ($1,$2,$3)', [relayId, publishVal, updatedTs]);
        }
        lastRelayStates[String(relayId)] = publishVal;
        io.emit('relay-state', { relay: String(relayId), state: publishVal });
        // Publish MQTT command so device receives the change
        try {
          const topic = `esp32s3/command/relay${relayId}`;
          const publishVal = normalizeStateForDevice(stateText);
          client.publish(topic, publishVal, { qos: 0 }, (err) => { if (err) console.error('MQTT publish error on relay_states upsert', err); else console.log('Published MQTT command', topic, publishVal); });
        } catch (e) { console.error('Failed to publish MQTT command', e); }
        return res.json({ ok: true });
      } catch (e) {
        console.error('PG upsert relay_state error', e);
        return res.status(500).json({ error: e.toString() });
      }
    })();
  } else {
    db.get('SELECT id FROM relay_states WHERE relay=?', [relayId], (err, row) => {
      if (err) { console.error('SQLite select relay_states error', err); return res.status(500).json({ error: err.toString() }); }
      const finish = (e) => {
        if (e) { console.error('SQLite upsert relay_states error', e); return res.status(500).json({ error: e.toString() }); }
        lastRelayStates[String(relayId)] = publishVal;
        io.emit('relay-state', { relay: String(relayId), state: publishVal });
        try {
          const topic = `esp32s3/command/relay${relayId}`;
          client.publish(topic, publishVal, { qos: 0 }, (errPub) => { if (errPub) console.error('MQTT publish error on relay_states upsert', errPub); else console.log('Published MQTT command', topic, publishVal); });
        } catch (ePub) { console.error('Failed to publish MQTT command', ePub); }
        return res.json({ ok: true });
      };
      if (row) {
        db.run('UPDATE relay_states SET state=?, updated_ts=? WHERE relay=?', [publishVal, updatedTs, relayId], finish);
      } else {
        db.run('INSERT INTO relay_states (relay, state, updated_ts) VALUES (?,?,?)', [relayId, publishVal, updatedTs], finish);
      }
    });
  }
});

app.put('/api/relay_states/:relay', (req, res) => {
  const relayId = Number(req.params.relay);
  const stateText = String(req.body.state || '');
  if (!Number.isFinite(relayId) || !stateText) return res.status(400).json({ error: 'relay and state required' });
  req.body.relay = relayId; // reuse POST logic
  return app._router.handle(req, res, () => {});
});

const PORT = process.env.PORT || 3000;
server.listen(PORT, () => {
  console.log(`Server running on http://localhost:${PORT}`);
});
