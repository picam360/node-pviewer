const { ipcRenderer } = require('electron');

window.addEventListener("DOMContentLoaded", () => {
    const savedIP = localStorage.getItem("lastIP");
    if (savedIP) {
        document.getElementById("ipInput").value = savedIP;
    }
});

document.getElementById('backupModeBtn').addEventListener('click', () => {
    document.getElementById('backupSection').style.display = 'block';
    document.getElementById('restoreSection').style.display = 'none';
});

document.getElementById('restoreModeBtn').addEventListener('click', () => {
    document.getElementById('backupSection').style.display = 'none';
    document.getElementById('restoreSection').style.display = 'block';
});

//<!-- バックアップ用セクション -->
document.getElementById('selectSyncListBtn').addEventListener('click', async () => {
    const filePath = await ipcRenderer.invoke('select-sync-list');

    if (filePath) {
        // ファイル読み込み
        const content = fs.readFileSync(filePath, 'utf-8');

        // 改行で分割し、空行を除去
        const folders = content
            .split(/\r?\n/)
            .map(line => line.trim())
            .filter(line => line.length > 0);

        // 表示リストを更新
        const ul = document.getElementById('folderListBkp');
        ul.innerHTML = ''; // 既存の内容をクリア

        folders.forEach(folder => {
            const li = document.createElement('li');
            li.textContent = folder;
            ul.appendChild(li);
        });
    }
});

document.getElementById("backupBtn").addEventListener("click", () => {
    const ipInput = document.getElementById("ipInput").value.trim();
    const liElements = document.querySelectorAll("#folderListBackup li");
    const folders = Array.from(liElements).map(li => li.textContent.trim());

    if (!ipInput || folders.length === 0) {
        alert("IPアドレスとフォルダを確認してください");
        return;
    }

    localStorage.setItem("lastIP", ipInput);
    ipcRenderer.send("start-backup", { ip: ipInput, folders });
});

ipcRenderer.on("backup-complete", (event, zipPath) => {
    alert(`バックアップ完了: ${zipPath}`);
});

ipcRenderer.on("backup-failed", (event, errorMsg) => {
    alert(`バックアップ失敗: ${errorMsg}`);
});

//<!-- リストア用セクション -->
let selectedZipPath = null;
document.getElementById('selectZipBtn').addEventListener('click', () => {
    ipcRenderer.send('open-zip-dialog');
});

ipcRenderer.on('zip-selected', (event, zipPath) => {
    selectedZipPath = zipPath;
    console.log("選択されたZIPパス:", zipPath);
});

ipcRenderer.on('folder-list', (event, folders, target) => {
    const ul = document.getElementById("folderList" + target);
    ul.innerHTML = '';
    folders.forEach(folder => {
        const li = document.createElement('li');
        li.textContent = folder;
        ul.appendChild(li);
    });

    // 共通セクションを表示
    const targetSection = document.getElementById("targetSection");
    targetSection.style.display = 'block';

    // ボタンの表示切り替え
    const backupBtn = document.getElementById("backupBtn");
    const restoreBtn = document.getElementById("restoreBtn");

    if (target === "Backup") {
        backupBtn.style.display = 'inline-block';
        restoreBtn.style.display = 'none';
    } else if (target === "Restore") {
        backupBtn.style.display = 'none';
        restoreBtn.style.display = 'inline-block';
    }
});

ipcRenderer.on('error', (event, message, target) => {
    const ul = document.getElementById("folderList" + target);
    ul.innerHTML = `<li style="color:red;">エラー: ${message}</li>`;
});

document.getElementById("restoreBtn").addEventListener("click", () => {
    const ip = document.getElementById("ipInput").value.trim();
    const zipPath = selectedZipPath; // ZIPパスはファイル選択時に保持しておく
    if (!ip || !zipPath) {
        alert("IPアドレスまたはZIPファイルが未選択です");
        return;
    }
    localStorage.setItem("lastIP", ip);
    ipcRenderer.send("start-restore", { ip, zipPath });
});

ipcRenderer.on("restore-complete", (event, count) => {
    alert(`リストア成功：${count}フォルダ`);
});

ipcRenderer.on("restore-failed", (event, error) => {
    alert("リストア失敗: " + error);
});