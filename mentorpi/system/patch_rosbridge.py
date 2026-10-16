"""Patch rosbridge_server (1.3.2, Humble) inside a freshly started MentorPi container.

Bug: if a browser disconnects while its subscriptions are still being set up (page closed or
reloaded within a second or two, phone sleeping, Wi-Fi drop), some subscriptions survive the
cleanup. rosbridge then keeps sending to the closed socket forever, only logging
"WebSocketClosedError: Tried to write to a closed websocket" once a second, and its memory
grows until the kernel kills it (car 1, 2026-10-16: 6.6 GB in 9 minutes, rosbridge dead,
web UI disconnected).

Fix: on the first failed write to a closed socket, finish the client's protocol (unsubscribes
everything for that client), so nothing more is sent to it.

Second bug: no back-pressure. A client that stays connected but stops reading (slow link, tab in
the background, a stalled browser) makes rosbridge queue every message: +300 KB/s measured with the
web page's subscriptions, never freed (car 1: 2.5 GB within minutes, watchdog restart).
Fix: at most MAX_PENDING messages waiting per connection; newer messages are dropped while full
(live sensor data is stale anyway; the client gets fresh data once it catches up).

Run as root by run_container.sh: python3 patch_rosbridge.py [FILE]
"""
import sys

FILE = sys.argv[1] if len(sys.argv) > 1 else \
    "/opt/ros/humble/local/lib/python3.10/dist-packages/rosbridge_server/websocket_handler.py"
MARK = "# mentorpi patch: clean up after write to closed socket"
MAX_PENDING = 20

src = open(FILE).read()
if MARK in src:
    print("rosbridge already patched")
    sys.exit(0)

old_closed = '''            cls.node_handle.get_logger().warn(
                "WebSocketClosedError: Tried to write to a closed websocket",
                throttle_duration_sec=1.0,
            )'''
old_stream = '''            cls.node_handle.get_logger().warn(
                "StreamClosedError: Tried to write to a closed stream",
                throttle_duration_sec=1.0,
            )'''
cleanup = '''
            self._mentorpi_cleanup()'''
helper = f'''    def _mentorpi_cleanup(self):
        {MARK}
        if getattr(self, "_mentorpi_cleaned", False):
            return
        self._mentorpi_cleaned = True
        try:
            self.incoming_queue.finish()
            self.protocol.finish()
            self.__class__.node_handle.get_logger().warn(
                "Client gone: removed its remaining subscriptions (mentorpi patch)")
        except Exception as exc:  # never let cleanup take rosbridge down
            self.__class__.node_handle.get_logger().error(f"cleanup after closed socket failed: {{exc}}")

    async def prewrite_message(self, message, binary):'''

old_send = "        _io_loop.add_callback(partial(self.prewrite_message, message, binary))"
new_send = f"""        # mentorpi patch: back-pressure, drop while {MAX_PENDING} messages are waiting for this client
        if getattr(self, "_mentorpi_pending", 0) >= {MAX_PENDING}:
            self._mentorpi_dropped = getattr(self, "_mentorpi_dropped", 0) + 1
            if self._mentorpi_dropped % 500 == 1:
                self.__class__.node_handle.get_logger().warn(
                    f"slow client: dropped {{self._mentorpi_dropped}} messages so far (mentorpi patch)")
            return
        self._mentorpi_pending = getattr(self, "_mentorpi_pending", 0) + 1
        _io_loop.add_callback(partial(self._mentorpi_write, message, binary))

    async def _mentorpi_write(self, message, binary):
        try:
            await self.prewrite_message(message, binary)
        finally:
            self._mentorpi_pending -= 1"""

for old in (old_closed, old_stream, "    async def prewrite_message(self, message, binary):", old_send):
    if src.count(old) != 1:
        sys.exit(f"patch_rosbridge: rosbridge code changed, cannot apply patch:\n{old}")
src = src.replace(old_closed, old_closed + cleanup)
src = src.replace(old_stream, old_stream + cleanup)
src = src.replace("    async def prewrite_message(self, message, binary):", helper)
src = src.replace(old_send, new_send)
open(FILE, "w").write(src)
print(f"rosbridge patched: vanished clients cleaned up, at most {MAX_PENDING} queued messages per client")
