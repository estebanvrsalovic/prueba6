// Supabase Edge Function: /functions/schedules
// - GET: returns JSON array of schedules
// - POST: accepts JSON array of schedules and replaces the table contents

const SUPABASE_URL = Deno.env.get('SUPABASE_URL') || '';
const SUPABASE_KEY = Deno.env.get('SUPABASE_SERVICE_ROLE_KEY') || '';

function authHeaders() {
  return {
    apikey: SUPABASE_KEY,
    Authorization: `Bearer ${SUPABASE_KEY}`,
    'Content-Type': 'application/json'
  };
}

export default async (req: Request) => {
  if (!SUPABASE_URL || !SUPABASE_KEY) {
    return new Response('Missing SUPABASE_URL or SUPABASE_SERVICE_ROLE_KEY', { status: 500 });
  }

  const url = new URL(req.url);
  const method = req.method.toUpperCase();

  if (method === 'GET') {
    const res = await fetch(`${SUPABASE_URL}/rest/v1/schedules?select=*`, {
      headers: authHeaders()
    });
    const body = await res.text();
    return new Response(body, { status: res.status, headers: { 'Content-Type': 'application/json' } });
  }

  if (method === 'POST') {
    let payload;
    try {
      payload = await req.json();
    } catch (err) {
      return new Response('Invalid JSON payload', { status: 400 });
    }
    if (!Array.isArray(payload)) {
      return new Response('Expected JSON array', { status: 400 });
    }

    // Delete all existing rows, then insert the provided array
    await fetch(`${SUPABASE_URL}/rest/v1/schedules`, { method: 'DELETE', headers: authHeaders() });

    const insertRes = await fetch(`${SUPABASE_URL}/rest/v1/schedules`, {
      method: 'POST',
      headers: authHeaders(),
      body: JSON.stringify(payload)
    });
    const insertBody = await insertRes.text();
    const contentType = insertRes.headers.get('content-type') || 'application/json';
    return new Response(insertBody, { status: insertRes.status, headers: { 'Content-Type': contentType } });
  }

  return new Response('Method Not Allowed', { status: 405 });
};
