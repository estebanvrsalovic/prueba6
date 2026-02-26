-- Supabase schema for schedules
-- Run this in your Supabase SQL editor (or via psql) to create the schedules table

CREATE TABLE IF NOT EXISTS schedules (
  id bigserial PRIMARY KEY,
  epoch_ms bigint NOT NULL,
  action text NOT NULL,
  zone integer,
  meta jsonb DEFAULT '{}'::jsonb
);
