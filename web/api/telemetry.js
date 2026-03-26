const { Pool } = require('pg');
const { createClient } = require('@supabase/supabase-js');

// Serverless telemetry ingestion for Vercel
// Supports two backends:
//  - Postgres via DATABASE_URL (preferred)
//  - Supabase via SUPABASE_URL + SUPABASE_SERVICE_ROLE_KEY

const DATABASE_URL = process.env.DATABASE_URL || null;
const SUPABASE_URL = process.env.SUPABASE_URL || null;
const SUPABASE_SERVICE_ROLE_KEY = process.env.SUPABASE_SERVICE_ROLE_KEY || null;
const TELEMETRY_KEY = process.env.TELEMETRY_KEY || null;

let pool = null;
if (DATABASE_URL) {
  pool = new Pool({ connectionString: DATABASE_URL, ssl: { rejectUnauthorized: false } });
}

let supabase = null;
if (SUPABASE_URL && SUPABASE_SERVICE_ROLE_KEY) {
  supabase = createClient(SUPABASE_URL, SUPABASE_SERVICE_ROLE_KEY);
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
      try { out.push(JSON.parse(ln)); } catch (err) { /* skip invalid lines */ }
    }
    return out;
  }
  return [];
}

async function insertToPostgres(entries) {
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
    return { ok: true, received: entries.length };
  } catch (err) {
    await client.query('ROLLBACK');
    throw err;
  } finally {
    client.release();
  }
}

async function insertToSupabase(entries) {
  // insert entries one by one using supabase service role (expects table telemetry schema)
  for (const e of entries) {
    const row = {
      device_id: e.device_id || e.device || 'esp',
      t1: (typeof e.t1 !== 'undefined') ? e.t1 : null,
      h1: (typeof e.h1 !== 'undefined') ? e.h1 : null,
      t2: (typeof e.t2 !== 'undefined') ? e.t2 : null,
      h2: (typeof e.h2 !== 'undefined') ? e.h2 : null
    };
    const { error } = await supabase.from('telemetry').insert([row]);
    if (error) throw error;
  }
  return { ok: true, received: entries.length };
}

module.exports = async (req, res) => {
  if (req.method !== 'POST') return res.status(405).json({ error: 'Method not allowed' });

  // auth by shared key if configured
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

  // read raw body text (support raw text, JSON parsed body, or NDJSON)
  const raw = typeof req.body === 'string' ? req.body : (req.body && Object.keys(req.body).length ? JSON.stringify(req.body) : (req.rawBody || ''));
  const entries = parseBody(raw);
  if (!entries || entries.length === 0) return res.status(400).json({ error: 'no valid telemetry entries' });

  try {
    if (pool) {
      const out = await insertToPostgres(entries);
      return res.json(out);
    }
    if (supabase) {
      const out = await insertToSupabase(entries);
      return res.json(out);
    }
    return res.status(503).json({ error: 'no persistence backend configured (DATABASE_URL or SUPABASE_URL + SERVICE_ROLE required)' });
  } catch (err) {
    console.error('Telemetry ingestion error', err);
    return res.status(500).json({ error: String(err) });
  }
};
