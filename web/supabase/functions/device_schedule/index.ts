// Supabase Edge Function: /functions/device_schedule
// - GET: returns schedule as CSV (epoch_ms,action,zone[,meta-json])
// - POST: accepts CSV text body and replaces schedules table

const SUPABASE_URL = Deno.env.get('SUPABASE_URL') || '';
const SUPABASE_KEY = Deno.env.get('SUPABASE_SERVICE_ROLE_KEY') || '';

function authHeaders() {
  return {
    apikey: SUPABASE_KEY,
    Authorization: `Bearer ${SUPABASE_KEY}`,
    'Content-Type': 'application/json'
  };
}

function rowsToCsv(rows: any[]) {
  // epoch_ms,action,zone,meta
  return rows
    .map(r => {
      const meta = r.meta ? JSON.stringify(r.meta) : '';
      return `${r.epoch_ms},${r.action},${r.zone ?? ''},${meta}`;
    })
    .join('\n') + '\n';
}

export default async (req: Request) => {
  if (!SUPABASE_URL || !SUPABASE_KEY) {
    return new Response('Missing SUPABASE_URL or SUPABASE_SERVICE_ROLE_KEY', { status: 500 });
  }

  const method = req.method.toUpperCase();

  if (method === 'GET') {
    const res = await fetch(`${SUPABASE_URL}/rest/v1/schedules?select=*&order=epoch_ms.asc`, { headers: authHeaders() });
    const rows = await res.json();
    return new Response(rowsToCsv(rows), { status: 200, headers: { 'Content-Type': 'text/csv' } });
  }

  if (method === 'POST') {
    const text = await req.text();
    if (!text) return new Response('Empty body', { status: 400 });

    const lines = text.split(/\r?\n/).map(l => l.trim()).filter(Boolean);
    const parsed = lines.map(line => {
      // epoch_ms,action,zone,meta
      const parts = line.split(',');
      const epoch_ms = Number(parts[0]);
      const action = parts[1] || '';
      const zone = parts[2] ? Number(parts[2]) : null;
      let meta = {};
      if (parts[3]) {
        try { meta = JSON.parse(parts.slice(3).join(',')); } catch (e) { meta = { raw: parts.slice(3).join(',') }; }
      }
      return { epoch_ms, action, zone, meta };
    });

    // Replace entire table
    await fetch(`${SUPABASE_URL}/rest/v1/schedules`, { method: 'DELETE', headers: authHeaders() });
    const insertRes = await fetch(`${SUPABASE_URL}/rest/v1/schedules`, {
      method: 'POST', headers: authHeaders(), body: JSON.stringify(parsed)
    });
    const body = await insertRes.text();
    return new Response(body, { status: insertRes.status, headers: { 'Content-Type': 'application/json' } });
  }

  return new Response('Method Not Allowed', { status: 405 });
};
