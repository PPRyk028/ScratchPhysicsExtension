import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { buildAll } from './build.mjs';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);
const projectRoot = path.resolve(__dirname, '..');
const distDir = path.join(projectRoot, 'dist');
const srcDir = path.join(projectRoot, 'src');
const port = 8000;
const shouldWatch = process.argv.includes('--watch');

function contentTypeFor(filePath) {
  if (filePath.endsWith('.js')) {
    return 'application/javascript; charset=utf-8';
  }

  if (filePath.endsWith('.html')) {
    return 'text/html; charset=utf-8';
  }

  return 'text/plain; charset=utf-8';
}

function buildIndexHtml() {
  return `<!doctype html>
<html lang="en">
  <head>
    <meta charset="utf-8" />
    <title>Scratch 3D Extension Kit</title>
    <style>
      body {
        font-family: Consolas, "SFMono-Regular", monospace;
        margin: 32px;
        line-height: 1.5;
        background: #0f172a;
        color: #e2e8f0;
      }
      a {
        color: #7dd3fc;
      }
      code {
        background: rgba(148, 163, 184, 0.14);
        padding: 2px 6px;
        border-radius: 6px;
      }
    </style>
  </head>
  <body>
    <h1>Scratch 3D Extension Kit</h1>
    <p>Development server is running.</p>
    <ul>
      <li><a href="/turbowarp-unsandboxed.js">/turbowarp-unsandboxed.js</a></li>
      <li><a href="/gandi-normal-remote.js">/gandi-normal-remote.js</a></li>
      <li><a href="/gandi-approved.js">/gandi-approved.js</a></li>
    </ul>
    <p>TurboWarp local unsandboxed URL expects <code>http://localhost:8000/...</code>.</p>
  </body>
</html>`;
}

function serveFile(response, filePath) {
  if (!fs.existsSync(filePath) || !fs.statSync(filePath).isFile()) {
    response.writeHead(404, { 'content-type': 'text/plain; charset=utf-8' });
    response.end('Not found');
    return;
  }

  response.writeHead(200, { 'content-type': contentTypeFor(filePath) });
  response.end(fs.readFileSync(filePath));
}

function rebuild() {
  try {
    buildAll();
  } catch (error) {
    console.error('Build failed:', error);
  }
}

buildAll();

if (shouldWatch) {
  let timer = null;
  fs.watch(srcDir, { recursive: true }, () => {
    clearTimeout(timer);
    timer = setTimeout(rebuild, 60);
  });
}

const server = http.createServer((request, response) => {
  const pathname = new URL(request.url, `http://localhost:${port}`).pathname;
  if (pathname === '/' || pathname === '/index.html') {
    response.writeHead(200, { 'content-type': 'text/html; charset=utf-8' });
    response.end(buildIndexHtml());
    return;
  }

  serveFile(response, path.join(distDir, pathname.replace(/^\/+/, '')));
});

server.listen(port, '127.0.0.1', () => {
  console.log(`Serving dist on http://localhost:${port}/`);
  if (shouldWatch) {
    console.log('Watching src/ for changes...');
  }
});
