// Wrapper for Vercel: forward to web/api/telemetry.js
const handler = require('../web/api/telemetry.js');

module.exports = async (req, res) => {
  return handler(req, res);
};
