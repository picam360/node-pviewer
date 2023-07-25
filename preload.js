const { contextBridge } = require('electron')

console.log("preload.js");

var m_pstcore = require('node-pstcore');
var electron_win = require('electron').remote.getCurrentWindow();

window.electron = {
    pstcore: m_pstcore,
    win : electron_win,
};

document.addEventListener("dblclick", (ev) => {
    if (ev.clientY < 50) { // title bar
        if (electron_win.isMaximized()) {
            electron_win.unmaximize();
        } else {
            electron_win.maximize();
        }
    }
});

electron_win.on('focus', function(ev) {
    if(!app.get_pst || !app.get_pst()){
        return;
    }
    // var now = Date.now();
    // if(!window.last_focus_date){
    // 	window.last_focus_date = now;
    // }
    // var diff = now - window.last_focus_date;
    // if(diff > 500){
    // 	window.last_focus_date = now;
    // 	m_pstcore.pstcore_set_param(app.get_pst(), "renderer", "win_focus", "1");
    // 	electron_win.show();
    // }
    m_pstcore.pstcore_set_param(app.get_pst(), "renderer", "win_focus", "1");
    electron_win.show();
});
window.addEventListener('resize', function(ev) {
    if(!app.get_pst || !app.get_pst()){
        return;
    }
    setTimeout(()=>{
        var is_fullscreen = (window.outerWidth == screen.width && window.outerHeight == screen.height);
        if(!is_fullscreen){
            console.log("resize", window.outerWidth, window.outerHeight);
            var value = window.outerWidth + "," + window.outerHeight;
            m_pstcore.pstcore_set_param(app.get_pst(), "renderer", "win_size", value);
        }
    }, 300);
});
setInterval(() => {
    if(!app.get_pst || !app.get_pst()){
        window.is_fullscreen = false;
        return;
    }
    var is_fullscreen = (window.outerWidth == screen.width && window.outerHeight == screen.height);
    if(is_fullscreen != window.is_fullscreen) {
        window.is_fullscreen = is_fullscreen;
        console.log("fullscreen", is_fullscreen);
        m_pstcore.pstcore_set_param(app.get_pst(), "renderer", "fullscreen", (is_fullscreen ? "1" : "0"));
        setTimeout(()=>{
            electron_win.show();
        }, 300);
    }
    if(!is_fullscreen){
        var dev_offset = electron_win.isDevToolsOpened() ? window.outerWidth : 0;
        m_pstcore.pstcore_set_param(app.get_pst(), "renderer", "win_pos", (window.screenX + dev_offset) + "," + window.screenY);
    }
    var win_focus = m_pstcore.pstcore_get_param(app.get_pst(), "renderer", "win_focus");
    if(parseInt(win_focus) && !electron_win.isFocused()){
        electron_win.show();
        //electron_win.focus();
        //console.log("focus req");
    }
    //console.log("focus "+ win_focus);
}, 200);