const { app, BrowserWindow } = require('electron');
const path = require('path');
const fs = require('fs');

const createWindow = () => {
    let appIcon;
    if (fs.existsSync(`${__dirname}/img/app.png`)) {
        appIcon = `${__dirname}/img/app.png`;
    } else if (fs.existsSync(`${__dirname}/img/icon.png`)) {
        appIcon = `${__dirname}/img/icon.png`;
    } else {
        appIcon = `${__dirname}/img/logo.png`;
    }
    console.log(appIcon);

    const win = new BrowserWindow({
        width: 800,
        height: 600,
        icon: appIcon,
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

    //win.webContents.openDevTools();

    win.loadFile('www/index.html');
};

app.commandLine.appendSwitch('enable-transparent-visuals');
app.commandLine.appendSwitch('disable-gpu');

app.whenReady().then(() => {
    setTimeout(() => {
        createWindow();
    }, 300);
});