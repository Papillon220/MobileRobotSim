/* ==========================================================================
   MobileRobotSim – Einstieg: Reiterverwaltung, helle/dunkle Darstellung
   ========================================================================== */
(function () {
  'use strict';
  const MRS = window.MRS;
  MRS.VERSION = 'V1.6'; // Programmversion (Kopfzeile und Demo-Anzeigen werden beim Start daraus gesetzt)
  const TABS = { plan: MRS.PlanningTab, teleop: MRS.TeleopTab, slam: MRS.SlamTab, follow: MRS.FollowTab, robkit: MRS.RobkitTab };
  let current = 'plan';

  // Datum der letzten Änderung: jüngster Zeitstempel von Seite, Skripten und Stylesheets (Server-Header „Last-Modified“);
  // ohne Webserver (file://) gilt das Änderungsdatum der HTML-Datei
  function showLastModified() {
    const set = (t) => document.querySelectorAll('.last-modified').forEach((el) => (el.textContent = new Date(t).toLocaleDateString('de-DE')));
    set(Date.parse(document.lastModified) || Date.now());
    if (!location.protocol.startsWith('http')) return;
    const urls = [location.href, ...[...document.scripts].map((s) => s.src), ...[...document.querySelectorAll('link[rel=stylesheet]')].map((l) => l.href)]
      .filter((u) => u && new URL(u).origin === location.origin);
    Promise.all(urls.map((u) => fetch(u, { method: 'HEAD', cache: 'no-store' }).then((r) => Date.parse(r.headers.get('Last-Modified')) || 0, () => 0)))
      .then((ts) => { const t = Math.max(...ts); if (t) set(t); });
  }

  function showTab(name) {
    current = name;
    document.querySelectorAll('.tab-btn').forEach((b) => b.classList.toggle('active', b.dataset.tab === name));
    // schmale Fenster: Reiterleiste scrollt, den gewählten Reiter in die Mitte holen
    const btn = document.querySelector('.tab-btn.active'), nav = btn.parentElement, br = btn.getBoundingClientRect(), nr = nav.getBoundingClientRect();
    nav.scrollLeft += br.left - nr.left - (nr.width - br.width) / 2;
    document.querySelectorAll('.tab').forEach((t) => t.classList.toggle('active', t.id === 'tab-' + name));
    MRS.TeleopTab.setActive(name === 'teleop');
    MRS.SlamTab.setActive(name === 'slam');
    MRS.FollowTab.setActive(name === 'follow');
    MRS.RobkitTab.setActive(name === 'robkit');
    if (name === 'plan') MRS.PlanningTab.onShow();
    try { localStorage.setItem('mrs-tab', name); } catch (e) { /* ignorieren */ }
  }

  // Darstellung: automatisch (wie das System), hell oder dunkel; Wahl wird im Browser gespeichert
  const THEMES = [['auto', '◐ Auto'], ['light', '☀ Hell'], ['dark', '☾ Dunkel']];
  const darkQuery = matchMedia('(prefers-color-scheme: dark)');
  function applyTheme(t) {
    if (t === 'auto') delete document.documentElement.dataset.theme; else document.documentElement.dataset.theme = t;
    document.getElementById('themeBtn').textContent = THEMES.find((x) => x[0] === t)[1];
    themeChanged();
  }
  // Farben der Zeichenflächen umstellen, zwischengespeicherte Kartenbilder neu erzeugen, sichtbaren Reiter neu zeichnen
  function themeChanged() {
    const t = document.documentElement.dataset.theme;
    MRS.setPalette(t ? t === 'dark' : darkQuery.matches);
    for (const tab of Object.values(TABS)) if (tab.onTheme) tab.onTheme();
    showTab(current);
  }

  window.addEventListener('DOMContentLoaded', () => {
    document.querySelectorAll('.version').forEach((el) => (el.textContent = MRS.VERSION));
    showLastModified();
    const t = document.documentElement.dataset.theme;
    MRS.setPalette(t ? t === 'dark' : darkQuery.matches); // Farben vor dem ersten Zeichnen
    MRS.PlanningTab.init();
    MRS.TeleopTab.init();
    MRS.SlamTab.init();
    MRS.FollowTab.init();
    MRS.RobkitTab.init();
    document.querySelectorAll('.tab-btn').forEach((b) => b.addEventListener('click', () => showTab(b.dataset.tab)));
    let tab = 'plan';
    try { tab = localStorage.getItem('mrs-tab') || 'plan'; } catch (e) { /* ignorieren */ }
    current = tab in TABS ? tab : 'plan';
    applyTheme(t || 'auto');
    document.getElementById('themeBtn').addEventListener('click', () => {
      const cur = document.documentElement.dataset.theme || 'auto';
      const next = THEMES[(THEMES.findIndex((x) => x[0] === cur) + 1) % THEMES.length][0];
      try { localStorage.setItem('mrs-theme', next); } catch (e) { /* ignorieren */ }
      applyTheme(next);
    });
    darkQuery.addEventListener('change', () => { if (!document.documentElement.dataset.theme) themeChanged(); });
  });
})();
