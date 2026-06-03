// streams/webrtc.js — WebRTC via MediaMTX (https://host:PORT/PATH/whep)
//
// MediaMTX owns the Pi camera. Node only tells the client where to find the
// WHEP endpoint. To change video params, this module patches mediamtx.yml and
// restarts the mediamtx service.
'use strict';

const fs   = require('fs');
const path = require('path');
const { execSync } = require('child_process');

// Read a single numeric value from mediamtx.yml by key.
function readYmlValue(content, key) {
  const m = content.match(new RegExp(`${key}:\\s*(\\S+)`));
  return m ? parseFloat(m[1]) : null;
}

// Replace a key's value in the YAML text. Works for numeric and string values.
function patchYml(content, key, value) {
  return content.replace(new RegExp(`(${key}:\\s*)\\S+`), `$1${value}`);
}

module.exports = function createWebRTCStream(config /*, streamServer not used */) {
  const PROTOCOL    = config.webrtc_protocol || 'https';
  const PORT        = config.webrtc_port     || 8889;
  const PATH_NAME   = config.webrtc_path     || 'cam';
  const YML_PATH    = config.mediamtx_yml    || path.join(__dirname, '..', 'mediamtx.yml');

  // Read current camera params from mediamtx.yml so getStreamConfig() returns
  // real values rather than hardcoded defaults.
  function readCurrentParams() {
    try {
      const content = fs.readFileSync(YML_PATH, 'utf8');
      return {
        width:   readYmlValue(content, 'rpiCameraWidth')   || 480,
        height:  readYmlValue(content, 'rpiCameraHeight')  || 360,
        fps:     readYmlValue(content, 'rpiCameraFPS')     || 20,
        bitrate: Math.round((readYmlValue(content, 'rpiCameraBitrate') || 350000) / 1000),
      };
    } catch (e) {
      console.error('WebRTC: failed to read mediamtx.yml:', e.message);
      return { width: 480, height: 360, fps: 20, bitrate: 350 };
    }
  }

  let params = readCurrentParams();
  console.log(`WebRTC: MediaMTX WHEP at ${PROTOCOL}://<host>:${PORT}/${PATH_NAME}/whep`);
  console.log(`WebRTC: camera ${params.width}×${params.height}@${params.fps}fps ${params.bitrate}kbps`);

  return {
    clientCount() { return 0; },
    stop() {},
    setParams(newParams) {
      try {
        let content = fs.readFileSync(YML_PATH, 'utf8');
        if (newParams.width   !== undefined) content = patchYml(content, 'rpiCameraWidth',   newParams.width);
        if (newParams.height  !== undefined) content = patchYml(content, 'rpiCameraHeight',  newParams.height);
        if (newParams.fps     !== undefined) content = patchYml(content, 'rpiCameraFPS',     newParams.fps);
        if (newParams.bitrate !== undefined) content = patchYml(content, 'rpiCameraBitrate', newParams.bitrate * 1000);
        fs.writeFileSync(YML_PATH, content);
        params = readCurrentParams();
        console.log('WebRTC: mediamtx.yml updated — restarting mediamtx…');
        execSync('systemctl restart mediamtx', { stdio: 'inherit' });
        console.log('WebRTC: mediamtx restarted');
      } catch (e) {
        console.error('WebRTC: setParams failed:', e.message);
      }
    },
    getStreamConfig() {
      return {
        codec:    'webrtc',
        protocol: PROTOCOL,
        port:     PORT,
        path:     PATH_NAME,
        width:    params.width,
        height:   params.height,
        fps:      params.fps,
        bitrate:  params.bitrate,
      };
    },
  };
};
