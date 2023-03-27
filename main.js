const { app, BrowserWindow } = require('electron');
const path = require('path');

const createWindow = () => {
    const win = new BrowserWindow({
        width: 800,
        height: 600,
        webPreferences: {
            //devTools: true,
            nodeIntegration: false,
            contextIsolation: false,
            enableRemoteModule: true,
            preload: path.join(__dirname, 'preload.js'),
        },
        hasShadow: false,
        transparent: true,
        backgroundColor: "#01000000",
        fullscreen: false,
        frame: false,
    });

    win.loadFile('www/index.html');
};

app.commandLine.appendSwitch('enable-transparent-visuals');
app.commandLine.appendSwitch('disable-gpu');

app.whenReady().then(() => {
    setTimeout(() => {
        createWindow();
    }, 300);
});