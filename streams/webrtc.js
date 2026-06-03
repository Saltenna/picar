// streams/webrtc.js — WebRTC via MediaMTX (https://host:PORT/PATH/whep)
//
// In WebRTC mode MediaMTX owns the Pi camera directly via its rpiCamera source.
// Node has no camera process to manage; it only tells the client where to find
// the WHEP endpoint so the browser can negotiate directly with MediaMTX.
'use strict';

module.exports = function createWebRTCStream(config /*, streamServer not used */) {
  const PROTOCOL = config.webrtc_protocol || 'https';
  const PORT     = config.webrtc_port     || 8889;
  const PATH     = config.webrtc_path     || 'cam';

  console.log(`WebRTC: MediaMTX WHEP at ${PROTOCOL}://<host>:${PORT}/${PATH}/whep`);

  return {
    clientCount() { return 0; },
    stop() {},
    getStreamConfig() {
      return { codec: 'webrtc', protocol: PROTOCOL, port: PORT, path: PATH };
    },
  };
};
