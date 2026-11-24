// Minimal Node-RED settings for ColorMix.
module.exports = {
  uiPort: process.env.PORT || 1880,
  flowFile: "flows.json",
  credentialSecret: false,

  // Editor lives at /admin so the root URL can serve the web app.
  httpAdminRoot: "/admin",
  httpNodeRoot: "/",
  // Serves ../web (mounted at /web) -> http://localhost:1880/ and /scan.html
  httpStatic: "/web",

  // Allows opening the web pages from another origin (e.g. file:// or Live Server).
  httpNodeCors: { origin: "*", methods: "GET,POST,OPTIONS" },

  mqttReconnectTime: 5000,
  functionGlobalContext: {},
  exportGlobalContextKeys: false,
  logging: { console: { level: "info", metrics: false, audit: false } },
  editorTheme: { projects: { enabled: false } },
};
