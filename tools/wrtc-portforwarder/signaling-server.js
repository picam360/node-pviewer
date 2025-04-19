const WebSocket = require("ws");
const url = require("url");

const PORT = 3000;
const peers = new Map();

const wss = new WebSocket.Server({ noServer: true });

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
        // 通知
        for (const [id, peerSocket] of peers.entries()) {
            peerSocket.send(JSON.stringify({ type: "leave", src: clientId }));
        }
    });

    ws.send(JSON.stringify({ type:"OPEN" }));
});

const http = require("http");
const server = http.createServer();

server.on("request", (req, res) => {
    const { pathname } = url.parse(req.url);

    if (pathname === "/") {
        res.writeHead(200, { "Content-Type": "text/plain" });
        res.end("OK");
    } else {
        res.writeHead(404, { "Content-Type": "text/plain" });
        res.end("404 Not Found");
    }
});

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
        socket.destroy();
    }
});

server.listen(PORT, () => {
    console.log(`🚀 PeerJS-compatible signaling server on ws://localhost:${PORT}/peerjs`);
});
