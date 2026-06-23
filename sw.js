// sw.js — minimal service worker for PWA installability
// No caching — picar requires live connection to the rover
self.addEventListener('install', () => self.skipWaiting());
self.addEventListener('activate', e => e.waitUntil(self.clients.claim()));
self.addEventListener('fetch', e => e.respondWith(fetch(e.request)));
