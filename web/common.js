// Shared helpers for ColorMix pages (generator + scanner).
// NOTE: computeDose() is duplicated in the Node-RED flow ("Validate & hitung dosis").
// If you change the mixing formula, change it in both places.

const ColorMix = (() => {
  const clamp = (v) => Math.max(0, Math.min(255, Math.round(Number(v) || 0)));

  const rgbToHex = (r, g, b) =>
    "#" + [r, g, b].map((v) => clamp(v).toString(16).padStart(2, "0")).join("").toUpperCase();

  const hexToRgb = (hex) => {
    const m = /^#?([0-9a-f]{6})$/i.exec(hex.trim());
    if (!m) return null;
    const n = parseInt(m[1], 16);
    return { r: (n >> 16) & 255, g: (n >> 8) & 255, b: n & 255 };
  };

  // RGB -> base paints Red, Green, Blue, proportional to R:G:B.
  // Returns millilitres for each base paint that sum to `vol`.
  // RGB(0,0,0) has no ratio, so it mixes equal parts (gives a dark result).
  const computeDose = (r, g, b, vol) => {
    let sum = r + g + b;
    const parts = sum > 0 ? { red: r, green: g, blue: b } : { red: 1, green: 1, blue: 1 };
    if (sum === 0) sum = 3;
    const dose = {};
    for (const key in parts) dose[key] = Math.round((parts[key] / sum) * vol * 10) / 10;
    return dose;
  };

  // Validates the JSON object stored in the QR code. Returns error string or null.
  const validateOrder = (o) => {
    if (!o || typeof o !== "object") return "Bukan objek JSON";
    if (o.v !== 1) return "Versi format tidak dikenal (v harus 1)";
    if (typeof o.id !== "string" || !o.id) return "Field 'id' kosong";
    for (const k of ["r", "g", "b"]) {
      if (!Number.isInteger(o[k]) || o[k] < 0 || o[k] > 255) return `Nilai '${k}' harus integer 0-255`;
    }
    if (typeof o.vol !== "number" || o.vol <= 0 || o.vol > 5000) return "Volume (vol) harus 1-5000 ml";
    return null;
  };

  const PIGMENTS = [
    { key: "red", name: "Merah", color: "#E53935" },
    { key: "green", name: "Hijau", color: "#43A047" },
    { key: "blue", name: "Biru", color: "#1E88E5" },
  ];

  const renderMix = (el, dose) => {
    const total = Object.values(dose).reduce((a, b) => a + b, 0) || 1;
    el.innerHTML =
      `<div class="mix-bar">` +
      PIGMENTS.map((p) => `<div style="width:${(dose[p.key] / total) * 100}%;background:${p.color}"></div>`).join("") +
      `</div><div class="mix-legend">` +
      PIGMENTS.map((p) => `<span><i style="background:${p.color}"></i>${p.name} ${dose[p.key]} ml</span>`).join("") +
      `</div>`;
  };

  return { clamp, rgbToHex, hexToRgb, computeDose, validateOrder, renderMix, PIGMENTS };
})();
