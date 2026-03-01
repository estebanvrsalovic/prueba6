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
