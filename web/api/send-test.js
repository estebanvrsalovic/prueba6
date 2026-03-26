const { createClient } = require('@supabase/supabase-js');

const SUPABASE_URL = process.env.SUPABASE_URL || '';
const SUPABASE_SERVICE_ROLE_KEY = process.env.SUPABASE_SERVICE_ROLE_KEY || '';
const TELEMETRY_KEY = process.env.TELEMETRY_KEY || null; // require this header to run

const supabase = createClient(SUPABASE_URL, SUPABASE_SERVICE_ROLE_KEY);

module.exports = async (req, res) => {
  if (req.method !== 'POST' && req.method !== 'GET') return res.status(405).json({ error: 'Method not allowed' });

  // protect endpoint with TELEMETRY_KEY if configured
  if (TELEMETRY_KEY) {
    const key = req.headers['x-api-key'] || req.query['api_key'];
    if (!key || key !== TELEMETRY_KEY) return res.status(401).json({ error: 'unauthorized' });
  }

  // allow overriding values via query or body
  const q = Object.assign({}, req.query || {});
  let body = req.body || {};
  if (Object.keys(body).length === 0 && q && Object.keys(q).length > 0) body = q;

  const now = new Date().toISOString();
  const row = {
    device: body.device || 'esp-test',
    t1: body.t1 ? Number(body.t1) : (20 + Math.random()*5).toFixed(2),
    h1: body.h1 ? Number(body.h1) : (40 + Math.random()*20).toFixed(2),
    t2: body.t2 ? Number(body.t2) : (19 + Math.random()*5).toFixed(2),
    h2: body.h2 ? Number(body.h2) : (38 + Math.random()*20).toFixed(2),
    raw: JSON.stringify(body),
    ts: now
  };

  try {
    const { error } = await supabase.from('telemetry').insert([row]);
    if (error) {
      console.error('send-test insert error', error.message);
      return res.status(500).json({ error: error.message });
    }
    return res.status(200).json({ ok: true, inserted: row });
  } catch (err) {
    console.error('send-test error', err);
    return res.status(500).json({ error: String(err) });
  }
};
