// streams/index.js — selects the active stream module based on config.stream_codec
'use strict';

module.exports = function createStream(config, streamServer) {
  const codec = (config.stream_codec || 'h264').toLowerCase();
  switch (codec) {
    case 'h264':   return require('./h264')(config, streamServer);
    case 'mjpeg':  return require('./mjpeg')(config, streamServer);
    case 'webrtc': return require('./webrtc')(config, streamServer);
    default:
      console.error(`Unknown stream_codec "${codec}" — defaulting to h264`);
      return require('./h264')(config, streamServer);
  }
};
