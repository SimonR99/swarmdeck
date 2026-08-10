import { defineConfig } from 'vite';
import { svelte } from '@sveltejs/vite-plugin-svelte';
import tailwindcss from '@tailwindcss/vite';
import { fileURLToPath, URL } from 'node:url';

export default defineConfig({
  plugins: [svelte(), tailwindcss()],
  resolve: {
    alias: {
      $lib: fileURLToPath(new URL('./src/lib', import.meta.url))
    }
  },
  server: {
    port: 5173,
    // Vite rejects requests whose Host header it does not recognise, which a
    // tunnel's hostname never is — the dev server answers "Blocked request"
    // and the page never loads. Only the tunnel providers' own domains are
    // listed; `true` would disable the check outright.
    //
    // This affects `make ui` only. The Docker stack serves the built UI through
    // nginx, which has no such check, and `scripts/tunnel.sh` points there.
    allowedHosts: ['.ngrok-free.app', '.ngrok.app', '.ngrok.io', '.trycloudflare.com'],
    proxy: {
      '/api': 'http://localhost:8080',
      '/ws': { target: 'ws://localhost:8080', ws: true },
      // MediaMTX exposes WHEP as /<stream>/whep. Keep the UI's stable
      // /whep/<robot> route and perform that transport-specific rewrite here.
      '/whep': {
        target: process.env.MEDIAMTX_WHEP_URL ?? 'http://localhost:8891',
        rewrite: (path) => path.replace(/^\/whep\/([^/]+)$/, '/$1/whep')
      }
    }
  }
});
