const { Pool } = require('pg');

// Serverless telemetry ingestion for Vercel
// Expects env var DATABASE_URL (Postgres). If TELEMETRY_KEY is set, requires
// requests to include header `x-telemetry-key` with that value.

const DATABASE_URL = process.env.DATABASE_URL || null;
const TELEMETRY_KEY = process.env.TELEMETRY_KEY || null;

let pool = null;
if (DATABASE_URL) {
  pool = new Pool({ connectionString: DATABASE_URL, ssl: { rejectUnauthorized: false } });
}

function parseBody(raw) {
  if (!raw) return [];
  try {
    const parsed = JSON.parse(raw);
    if (Array.isArray(parsed)) return parsed;
    if (typeof parsed === 'object') return [parsed];
  } catch (e) {
    // try NDJSON
    const lines = raw.split(/\r?\n/).map(l => l.trim()).filter(l => l.length > 0);
    const out = [];
    for (const ln of lines) {
      try { out.push(JSON.parse(ln)); } catch (err) { /* skip */ }
    }
    return out;
  }
  return [];
}

module.exports = async (req, res) => {
  // Allow POST only
  if (req.method !== 'POST') return res.status(405).send('Method Not Allowed');

  if (TELEMETRY_KEY) {
    const keyHeader = req.headers['x-telemetry-key'] || req.headers['x-api-key'] || req.headers['authorization'];
    let key = null;
    if (keyHeader) {
      if (typeof keyHeader === 'string' && keyHeader.toLowerCase().startsWith('bearer ')) key = keyHeader.slice(7).trim();
      else if (Array.isArray(keyHeader)) key = keyHeader[0];
      else key = keyHeader;
    }
    if (!key || key !== TELEMETRY_KEY) return res.status(401).json({ error: 'unauthorized' });
  }

  if (!pool) return res.status(503).json({ error: 'DATABASE_URL not configured; cannot persist telemetry in serverless environment' });

  // read raw body text
  const raw = typeof req.body === 'string' ? req.body : (req.body && Object.keys(req.body).length ? JSON.stringify(req.body) : '');
  const entries = parseBody(raw);
  if (!entries || entries.length === 0) return res.status(400).json({ error: 'no valid telemetry entries' });

  const now = Date.now();
  const client = await pool.connect();
  try {
    await client.query('BEGIN');
    for (const e of entries) {
      const device_ts = (typeof e.t !== 'undefined') ? Number(e.t) : null;
      const t1 = (typeof e.t1 !== 'undefined') ? Number(e.t1) : null;
      const h1 = (typeof e.h1 !== 'undefined') ? Number(e.h1) : null;
      const t2 = (typeof e.t2 !== 'undefined') ? Number(e.t2) : null;
      const h2 = (typeof e.h2 !== 'undefined') ? Number(e.h2) : null;
      const dht1_ok = (typeof e.dht1_ok !== 'undefined') ? (e.dht1_ok ? 1 : 0) : null;
      const dht2_ok = (typeof e.dht2_ok !== 'undefined') ? (e.dht2_ok ? 1 : 0) : null;
      const hasSensor = [t1, h1, t2, h2].some(v => v !== null && Number.isFinite(v));
      if (!hasSensor) continue;
      await client.query('INSERT INTO telemetry (ts, device_ts, t1, h1, t2, h2, dht1_ok, dht2_ok) VALUES ($1,$2,$3,$4,$5,$6,$7,$8)', [now, device_ts, t1, h1, t2, h2, dht1_ok, dht2_ok]);
    }
    await client.query('COMMIT');
    return res.json({ ok: true, received: entries.length });
  } catch (err) {
    await client.query('ROLLBACK');
    console.error('Serverless telemetry insert error', err);
    return res.status(500).json({ error: String(err) });
  } finally {
    client.release();
  }
};
const { createClient } = require('@supabase/supabase-js');

// Use the SERVICE_ROLE key (server-side) configured in Vercel environment variables
const SUPABASE_URL = process.env.SUPABASE_URL;
const SUPABASE_SERVICE_ROLE_KEY = process.env.SUPABASE_SERVICE_ROLE_KEY;
const TELEMETRY_KEY = process.env.TELEMETRY_KEY || null; // simple shared secret

if (!SUPABASE_URL || !SUPABASE_SERVICE_ROLE_KEY) {
  console.warn('Supabase not configured for telemetry endpoint. Set SUPABASE_URL and SUPABASE_SERVICE_ROLE_KEY.');
}

const supabase = createClient(SUPABASE_URL || '', SUPABASE_SERVICE_ROLE_KEY || '');

module.exports = async (req, res) => {
  if (req.method !== 'POST') return res.status(405).json({ error: 'Method not allowed' });

  // simple auth: accept x-telemetry-key, x-api-key, or Authorization: Bearer <key>
  if (TELEMETRY_KEY) {
    const keyHeader = req.headers['x-telemetry-key'] || req.headers['x-api-key'] || req.headers['authorization'];
    let key = null;
    if (keyHeader) {
      if (typeof keyHeader === 'string' && keyHeader.toLowerCase().startsWith('bearer ')) {
        key = keyHeader.slice(7).trim();
      } else if (Array.isArray(keyHeader)) {
        key = keyHeader[0];
      } else {
        key = keyHeader;
      }
    }
    if (!key || key !== TELEMETRY_KEY) return res.status(401).json({ error: 'unauthorized' });
  }

  let body = req.body;
  if (!body || Object.keys(body).length === 0) {
    // try parse JSON body fallback
    try { body = JSON.parse(req.rawBody || '{}'); } catch (err) { body = {}; }
  }

  const row = {
    device_id: body.device_id || body.device || 'esp',
    t1: (typeof body.t1 !== 'undefined') ? body.t1 : null,
    h1: (typeof body.h1 !== 'undefined') ? body.h1 : null,
    t2: (typeof body.t2 !== 'undefined') ? body.t2 : null,
    h2: (typeof body.h2 !== 'undefined') ? body.h2 : null
  };

  try {
    const { error } = await supabase.from('telemetry').insert([row]);
    if (error) {
      console.error('Supabase insert error', error.message);
      return res.status(500).json({ error: error.message });
    }
    return res.status(200).json({ ok: true });
  } catch (err) {
    console.error('Telemetry endpoint error', err);
    return res.status(500).json({ error: String(err) });
  }
};
