/*global cordova, module*/

module.exports = {
	init: function (successCallback, errorCallback) {
	    cordova.exec(successCallback, errorCallback, "CDVPstCore", "init_internal", []); // init is reserved
	},
	on_set_param: function (successCallback, errorCallback) {
	    cordova.exec(successCallback, errorCallback, "CDVPstCore", "on_set_param", []);
	},
	build_pvf_streamer: function (successCallback, errorCallback) {
	    cordova.exec(successCallback, errorCallback, "CDVPstCore", "build_pvf_streamer", []);
	},
	poll: function (successCallback, errorCallback) {
	    cordova.exec(successCallback, errorCallback, "CDVPstCore", "poll", []);
	},
	start_pstreamer: function (_this, successCallback, errorCallback) {
	    cordova.exec(successCallback, errorCallback, "CDVPstCore", "start_pstreamer", [_this]);
	},
	stop_pstreamer: function (_this, successCallback, errorCallback) {
	    cordova.exec(successCallback, errorCallback, "CDVPstCore", "stop_pstreamer", [_this]);
	},
	destroy_pstreamer: function (_this, successCallback, errorCallback) {
	    cordova.exec(successCallback, errorCallback, "CDVPstCore", "destroy_pstreamer", [_this]);
	},
	enqueue: function (_this, data, successCallback, errorCallback) {
	    cordova.exec(successCallback, errorCallback, "CDVPstCore", "enqueue", [_this, data]);
	},
	set_param: function (_this, pst_name, param, value, successCallback, errorCallback) {
	    cordova.exec(successCallback, errorCallback, "CDVPstCore", "set_param", [_this, pst_name, param, value]);
	},
};
