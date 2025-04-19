const http = require("http");
const https = require("https");
const fs = require("fs");
const path = require("path");
const url = require("url");
const WebSocket = require("ws");

// === 証明書 ===
const httpsOptions = {
  key: fs.readFileSync("/etc/letsencrypt/live/wrtc-pf.picam360.com/privkey.pem"),
  cert: fs.readFileSync("/etc/letsencrypt/live/wrtc-pf.picam360.com/fullchain.pem"),
};

const wwwRoot = path.join(__dirname, "www");
const peers = new Map();
const wss = new WebSocket.Server({ noServer: true });

// === 静的ファイルの共通リクエスト処理 ===
function handleHttpRequest(req, res) {
  const parsedUrl = url.parse(req.url);
  let pathname = decodeURIComponent(parsedUrl.pathname);
  let safePath = path.normalize(path.join(wwwRoot, pathname));

  if (!safePath.startsWith(wwwRoot)) {
    res.writeHead(403, { "Content-Type": "text/plain" });
    return res.end("403 Forbidden");
  }

  if (fs.existsSync(safePath) && fs.statSync(safePath).isDirectory()) {
    safePath = path.join(safePath, "index.html");
  }

  fs.readFile(safePath, (err, data) => {
    if (err) {
      res.writeHead(404, { "Content-Type": "text/plain" });
      return res.end("404 Not Found");
    }

    const ext = path.extname(safePath);
    const mime = {
      ".html": "text/html",
      ".js": "application/javascript",
      ".css": "text/css",
      ".png": "image/png",
      ".jpg": "image/jpeg",
      ".json": "application/json",
      ".txt": "text/plain",
    }[ext] || "application/octet-stream";

    res.writeHead(200, { "Content-Type": mime });
    res.end(data);
  });
}

// === WebSocket 接続処理 ===
wss.on("connection", (ws, request, clientId) => {
  console.log(`[+] New connection for ID: ${clientId}`);
  peers.set(clientId, ws);

  ws.on("message", (msg) => {
    try {
      const { type, dst, payload } = JSON.parse(msg);
      const src = clientId;
      if (dst && peers.has(dst)) {
        const target = peers.get(dst);
        target.send(JSON.stringify({ type, src, payload }));
        console.log(`[>] ${type}: ${src} → ${dst}`);
      }
    } catch (err) {
      console.error("[!] Error parsing message:", err);
    }
  });

  ws.on("close", () => {
    console.log(`[-] Connection closed: ${clientId}`);
    peers.delete(clientId);
    for (const [id, sock] of peers.entries()) {
      sock.send(JSON.stringify({ type: "leave", src: clientId }));
    }
  });

  ws.send(JSON.stringify({ type: "OPEN" }));
});

// === 共通 upgrade 処理 ===
function setupUpgrade(server) {
  server.on("upgrade", (request, socket, head) => {
    const { pathname, query } = url.parse(request.url, true);
    if (pathname === "/peerjs") {
      const clientId = query.id;
      if (!clientId) {
        socket.destroy();
        return;
      }
      wss.handleUpgrade(request, socket, head, (ws) => {
        wss.emit("connection", ws, request, clientId);
      });
    } else {
      socket.write(
        "HTTP/1.1 404 Not Found\r\n" +
        "Content-Type: text/plain\r\n" +
        "Content-Length: 13\r\n\r\n" +
        "404 Not Found"
      );
      socket.destroy();
    }
  });
}

// === HTTP サーバー ===
const httpServer = http.createServer(handleHttpRequest);
setupUpgrade(httpServer);
httpServer.listen(36080, () => {
  console.log("🚀 HTTP running on port 36080");
});

// === HTTPS サーバー ===
const httpsServer = https.createServer(httpsOptions, handleHttpRequest);
setupUpgrade(httpsServer);
httpsServer.listen(36443, () => {
  console.log("🔐 HTTPS running on port 36443");
});
