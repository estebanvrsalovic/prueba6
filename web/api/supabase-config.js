// Returns runtime-safe Supabase public config for the frontend (anon key)
module.exports = (req, res) => {
  const SUPABASE_URL = process.env.SUPABASE_URL || '';
  const SUPABASE_ANON_KEY = process.env.SUPABASE_ANON_KEY || '';
  return res.status(200).json({ SUPABASE_URL, SUPABASE_ANON_KEY });
};
