const { app, BrowserWindow, ipcMain, dialog } = require('electron');
const { NodeSSH } = require("node-ssh");
const path = require('path');
const fs = require('fs');
const os = require("os");
const { exec } = require('child_process');
const unzipper = require('unzipper'); // npm install unzipper
const archiver = require("archiver");
const AdmZip = require('adm-zip');

const yargs = require('yargs/yargs');
const { hideBin } = require('yargs/helpers');
const argv = yargs(hideBin(process.argv)).argv;
const query = argv.query || ''; // "param1=value1&param2=value2"
const devTools = argv.devTools || false;

let mainWindow;

function createWindow() {
    mainWindow = new BrowserWindow({
        width: 800,
        height: 600,
        webPreferences: {
            devTools: devTools,
            nodeIntegration: true,   // Electron 12以降は contextIsolation も false に必要
            contextIsolation: false,
        }
    });


    if (devTools) {
        mainWindow.webContents.openDevTools();
    }
    mainWindow.loadURL(`file://${__dirname}/index.html?${query}`);
}

app.whenReady().then(createWindow);

//<!-- バックアップ用セクション -->
ipcMain.handle('select-sync-list', async () => {
    const { canceled, filePaths } = await dialog.showOpenDialog({
        title: 'sync_list.txt を選択してください',
        properties: ['openFile'],
        filters: [{ name: 'Text Files', extensions: ['txt'] }],
        defaultPath: process.cwd(),
    });

    if (canceled || filePaths.length === 0) return;

    const filePath = filePaths[0];

    try {
        // ファイル読み込み
        const content = fs.readFileSync(filePath, 'utf-8');

        // 改行で分割し、空行を除去
        const folders = content
            .split(/\r?\n/)
            .map(line => line.trim())
            .filter(line => line.length > 0);

        mainWindow.webContents.send('folder-list', folders, 'Backup');
    } catch (err) {
        mainWindow.webContents.send('error', 'ZIP展開中にエラーが発生しました');
        console.error(err, 'Backup');
    }
});

function getTimestampedFilename(baseName = "backup_output") {
    const now = new Date();
    const yyyy = now.getFullYear();
    const mm = String(now.getMonth() + 1).padStart(2, "0");
    const dd = String(now.getDate()).padStart(2, "0");
    const hh = String(now.getHours()).padStart(2, "0");
    const mi = String(now.getMinutes()).padStart(2, "0");
    const ss = String(now.getSeconds()).padStart(2, "0");
    return `${baseName}_${yyyy}-${mm}-${dd}_${hh}-${mi}-${ss}.zip`;
}

ipcMain.on("start-backup", async (event, { ip, folders }) => {
    const defaultFileName = getTimestampedFilename(); // 例: backup_output_2025-05-31_17-41-22.zip

    const { canceled, filePath } = await dialog.showSaveDialog({
        title: "バックアップZIPの保存先を選択",
        defaultPath: path.join(process.cwd(), defaultFileName),
        filters: [{ name: "ZIPファイル", extensions: ["zip"] }]
    });

    if (canceled || !filePath) {
        event.sender.send("backup-failed", "保存先が選択されませんでした");
        return;
    }

    const ssh = new NodeSSH();
    const [userpass, host] = ip.split("@");
    const [username, password] = userpass.split(":");

    const tmpDir = fs.mkdtempSync(path.join(os.tmpdir(), "backup-"));

    try {
        await ssh.connect({
            host,
            username,
            password,
            tryKeyboard: true
        });

        for (const remotePath of folders) {
            const base = path.basename(remotePath);
            const localPath = path.join(tmpDir, base);
            if (!fs.existsSync(localPath)) {
                fs.mkdirSync(localPath, { recursive: true });
            }
            await ssh.getDirectory(localPath, remotePath, {
                recursive: true,
                concurrency: 5
            });
        }

        // sync_list.txt も保存
        fs.writeFileSync(path.join(tmpDir, "sync_list.txt"), folders.join("\n"), "utf-8");

        // ZIP化
        const output = fs.createWriteStream(filePath);
        const archive = archiver("zip", { zlib: { level: 9 } });

        archive.pipe(output);
        archive.directory(tmpDir, false);
        output.on('close', () => {
            console.log("ZIP 完了:", archive.pointer(), "bytes");
            event.sender.send("backup-complete", filePath);
        });
        await archive.finalize();
    } catch (err) {
        console.error("バックアップ失敗:", err);
        event.sender.send("backup-failed", err.message);
    } finally {
        ssh.dispose();
    }
});

//<!-- リストア用セクション -->
ipcMain.on('open-zip-dialog', async () => {
    const { canceled, filePaths } = await dialog.showOpenDialog({
        filters: [{ name: 'ZIP Files', extensions: ['zip'] }],
        properties: ['openFile'],
        defaultPath: process.cwd(),
    });

    if (canceled || filePaths.length === 0) return;

    const zipPath = filePaths[0];
    mainWindow.webContents.send('zip-selected', zipPath);

    try {
        const directory = await unzipper.Open.file(zipPath);
        const entry = directory.files.find(file => file.path === 'sync_list.txt');
        if (!entry) {
            mainWindow.webContents.send('error', 'sync_list.txt がZIP内に見つかりません');
            return;
        }

        const content = await entry.buffer();
        const folders = content.toString('utf-8').split('\n').map(line => line.trim()).filter(line => line.length > 0);
        mainWindow.webContents.send('folder-list', folders, 'Restore');
    } catch (err) {
        mainWindow.webContents.send('error', 'ZIP展開中にエラーが発生しました');
        console.error(err, 'Restore');
    }
});

function unzipToTemp(zipPath) {
    const tmpDir = fs.mkdtempSync(path.join(os.tmpdir(), "restore-"));
    const zip = new AdmZip(zipPath);
    zip.extractAllTo(tmpDir, true);
    return tmpDir;
}

function getTimestampSuffix() {
    const now = new Date();
    return now.toISOString().replace(/[-:T]/g, "").slice(0, 14); // 例: 20240531164512
}

ipcMain.on("start-restore", async (event, { ip, zipPath }) => {
    const ssh = new NodeSSH();
    const [userpass, host] = ip.split("@");
    const [username, password] = userpass.split(":");

    try {
        const tmpDir = unzipToTemp(zipPath);

        const syncListPath = path.join(tmpDir, "sync_list.txt");
        if (!fs.existsSync(syncListPath)) {
            throw new Error("sync_list.txt が ZIP 内にありません");
        }

        const folders = fs.readFileSync(syncListPath, "utf-8")
            .split(/\r?\n/)
            .map(line => line.trim())
            .filter(line => line.length > 0);

        await ssh.connect({ host, username, password });

        const timestamp = getTimestampSuffix();
        for (const folder of folders) {
            const backupPath = `${folder}_${timestamp}`;
            await ssh.execCommand(`if [ -d "${folder}" ]; then mv "${folder}" "${backupPath}"; fi`);
        }

        for (const folder of folders) {
            const base = path.basename(folder);
            const localFolder = path.join(tmpDir, base);
            if (fs.existsSync(localFolder)) {
                await ssh.putDirectory(localFolder, folder, { recursive: true });
            } else {
                console.warn(`ZIPに ${base} が見つかりません`);
            }
        }

        event.sender.send("restore-complete", folders.length);
    } catch (err) {
        event.sender.send("restore-failed", err.message);
    } finally {
        ssh.dispose();
    }
});