# Camera stream transport

The camera panel prefers MediaMTX WHEP/WebRTC because it has the lowest latency.
The WHEP offer travels through the UI at `/whep/<robot>`, but its media uses a
separate ICE socket. A browser on the LAN can reach production ICE port `8189`.
The isolated planning overlay uses `8190` for both UDP and TCP so it cannot
collide with production. Set `MEDIAMTX_TEST_WEBRTC_HOST` when the test browser
needs an ICE host other than `MEDIAMTX_WEBRTC_HOST` or the LAN default.

An HTTPS proxy, Cloudflare tunnel, or SSH port forward often carries HTTP while
blocking the separate ICE socket. The UI then falls back to the same H.264
stream through low-latency HLS at `/hls/<robot>/index.m3u8`. nginx proxies that
same-origin path to MediaMTX's internal port `8888`; the HLS port does not need
to be published. nginx keeps MediaMTX's cookie-check redirect beneath `/hls`
and relative to the browser's origin, preserving an HTTPS or forwarded host and
port. Browsers with Media Source Extensions prefer pinned `hls.js`; browsers
without MSE use the video element's native HLS support when available.

The panel reports **LIVE** only after the current robot's video element decodes
a frame. Robot changes and component teardown abort the WHEP request, close the
peer connection, destroy the HLS loader, clear the old media element, and fence
late callbacks. A stream that stops producing decoded frames is hidden after
three seconds. Its current decoder gets ten seconds to recover before the panel
rebuilds the transport with a retry delay capped at 30 seconds, so an old frame
is not presented as current location context.

The HLS path preserves H.264 and is a reachability fallback, not a JPEG camera
API. It has more latency than WebRTC. Check the on-screen frame rate and compare
visible motion before using it for time-sensitive driving.

## Validation

The isolated Bistro deployment decoded all four robot streams through nginx.
Chromium tests exercised a deliberately unanswered WHEP request and switching
between robots; old media was cleared before the new robot decoded frames.
The final build also played R3 through an SSH HTTP tunnel with normal browser
behavior: decoded frames advanced from 40 to 85 over sixteen one-second samples
without reconnecting. Switching to R1 cleared the previous source and then
decoded 320×240 video, advancing from 24 to 37 frames over four seconds.
These are playback and recovery checks, not a measured
end-to-end latency guarantee. The camera unit tests, Svelte checks, production
build, and deployment configuration tests also pass.
