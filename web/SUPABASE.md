# Supabase serverless deployment (schedules)

This folder contains a minimal Supabase setup to make the project fully serverless:

- `web/supabase/init.sql` — SQL to create the `schedules` table in your Supabase Postgres.
- `web/supabase/functions/schedules` — Edge Function (GET/POST) returning/receiving JSON schedules.
- `web/supabase/functions/device_schedule` — Edge Function (GET returns CSV, POST accepts CSV) for the device.

Environment variables required in Supabase functions (set in the Supabase dashboard):

- `SUPABASE_URL` — example: https://xyzcompany.supabase.co
- `SUPABASE_SERVICE_ROLE_KEY` — service_role key (server-side secret)

Deployment steps (high level):

1. Create a Supabase project at https://app.supabase.com and note the project URL and service_role key.
2. In Supabase SQL Editor run `web/supabase/init.sql` to create the `schedules` table.
3. Deploy Edge Functions:
   - Install `supabase` CLI: https://supabase.com/docs/guides/cli
   - Copy the `web/supabase/functions/*` folders into your local `supabase/functions` folder or create them in-place.
   - Use `supabase login` and `supabase functions deploy schedules` and `supabase functions deploy device_schedule`.
4. Configure the two environment vars (`SUPABASE_URL`, `SUPABASE_SERVICE_ROLE_KEY`) in the Supabase Functions settings or via `supabase secrets set`.
5. Update your web UI and device firmware to call the new endpoints:
   - Schedules JSON API (edge function): `https://<project>.supabase.co/functions/v1/schedules`
   - Device CSV endpoint: `https://<project>.supabase.co/functions/v1/device_schedule`

Proxying via existing `web/server.js` (minimal frontend changes)

If you want to keep the current frontend unchanged, you can configure the existing `web/server.js` as a proxy to Supabase Edge Functions.

1. Set the env var `SUPABASE_FUNCTIONS_BASE` to your functions base URL, for example:

   - `SUPABASE_FUNCTIONS_BASE=https://<project>.supabase.co/functions/v1`

2. The server will proxy these endpoints when `SUPABASE_FUNCTIONS_BASE` is set:

   - `GET /api/schedule` -> proxied to `<base>/schedules` (returns JSON)
   - `POST /api/schedule` -> proxied to `<base>/schedules` (accepts JSON array). After persisting in Supabase the server will also publish the schedule CSV to MQTT topic `esp32s3/schedule` so connected devices receive the update.
   - `GET /api/device_schedule` -> proxied to `<base>/device_schedule` (returns CSV text)
   - `POST /api/device_schedule` -> proxied to `<base>/device_schedule` (accepts CSV text)

3. Start the server with the env var set (example):

```bash
SUPABASE_FUNCTIONS_BASE=https://xyz.supabase.co/functions/v1 npm start
```

This approach lets your frontend continue calling `/api/schedule` and device code continue using `/api/device_schedule` while storage and business logic move to Supabase.

Notes and integration suggestions:

- The `device_schedule` endpoint returns CSV; device can `GET` to download and `POST` to upload its `/schedule.csv` contents.
- The web UI should `GET` schedules (JSON) to display and `POST` them to replace the schedule set.
- For authentication, keep the service role key only in server-side functions or use row-level security + anon keys depending on your security model.

If you want, I can:

- Patch `web/server.js` to act as a thin proxy to these Supabase functions (to keep the current frontend unchanged), or
- Update the firmware `src/main.cpp` to call the CSV endpoints directly instead of using MQTT.

Tell me which follow-up you want and I will implement it next.
