/* ==========================================================================
   MobileRobotSim – Einstieg: Reiterverwaltung
   ========================================================================== */
(function () {
  'use strict';
  const MRS = window.MRS;
  MRS.VERSION = 'V1.2'; // Programmversion (Kopfzeile und Demo-Anzeigen werden beim Start daraus gesetzt)

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
    document.querySelectorAll('.tab-btn').forEach((b) => b.classList.toggle('active', b.dataset.tab === name));
    document.querySelectorAll('.tab').forEach((t) => t.classList.toggle('active', t.id === 'tab-' + name));
    MRS.TeleopTab.setActive(name === 'teleop');
    MRS.SlamTab.setActive(name === 'slam');
    MRS.FollowTab.setActive(name === 'follow');
    if (name === 'plan') MRS.PlanningTab.onShow();
    try { localStorage.setItem('mrs-tab', name); } catch (e) { /* ignorieren */ }
  }

  window.addEventListener('DOMContentLoaded', () => {
    document.querySelectorAll('.version').forEach((el) => (el.textContent = MRS.VERSION));
    showLastModified();
    MRS.PlanningTab.init();
    MRS.TeleopTab.init();
    MRS.SlamTab.init();
    MRS.FollowTab.init();
    document.querySelectorAll('.tab-btn').forEach((b) => b.addEventListener('click', () => showTab(b.dataset.tab)));
    let tab = 'plan';
    try { tab = localStorage.getItem('mrs-tab') || 'plan'; } catch (e) { /* ignorieren */ }
    showTab(['plan', 'teleop', 'slam', 'follow'].includes(tab) ? tab : 'plan');
  });
})();
