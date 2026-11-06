#!/usr/bin/env python3
"""HTTPS for the car's page on the local network (service mentorpi-lan-https, README "Local network").

On the tailnet, Tailscale serve gives the page HTTPS (https://<car>.<tailnet>.ts.net). On the local network the
page also works over plain http://<car ip>:8000, but browsers turn off the microphone (Talk) and gamepads on
plain-http pages that aren't localhost. This adds the same three HTTPS ports on the car's local addresses:

  443  -> 127.0.0.1:8000  the page
  8443 -> 127.0.0.1:9090  rosbridge (the page uses wss://<host>:8443 when it was opened over https)
  8444 -> 127.0.0.1:8080  video

It decrypts and forwards bytes, nothing else. The certificate is self-signed (made here once, for
<hostname>.local, <hostname> and the car's local IPs), so the browser warns the first time: accept it.
Tailscale's own addresses are left alone (tailscaled listens there); the address list is checked every
RESCAN_S, so a new Wi-Fi address is picked up without a restart. Python standard library + openssl only.
"""
import asyncio
import ipaddress
import os
import socket
import ssl
import subprocess
import sys

PORTS = {443: 8000, 8443: 9090, 8444: 8080}
RESCAN_S = 20
CERT_DIR = os.environ.get('LAN_HTTPS_DIR') or os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'state', 'lan_https')
SKIP_NETS = [ipaddress.ip_network(n) for n in ('127.0.0.0/8', '100.64.0.0/10', '169.254.0.0/16', '172.17.0.0/16',
                                               '::1/128', 'fd7a:115c:a1e0::/48', 'fe80::/10')]


def log(msg):
    print(msg, flush=True)


def local_addresses():
    """The car's local-network addresses (no loopback, Tailscale, Docker or link-local)."""
    out = subprocess.run(['ip', '-o', 'addr', 'show', 'up'], capture_output=True, text=True).stdout
    addrs = []
    for line in out.splitlines():
        f = line.split()
        if len(f) < 4 or f[2] not in ('inet', 'inet6') or f[1].startswith(('docker', 'veth', 'br-', 'tailscale')):
            continue
        a = ipaddress.ip_address(f[3].split('/')[0])
        if not any(a in n for n in SKIP_NETS):
            addrs.append(str(a))
    return addrs


def cert_names(path):
    try:
        out = subprocess.run(['openssl', 'x509', '-in', path, '-noout', '-ext', 'subjectAltName'],
                             capture_output=True, text=True).stdout
    except OSError:
        return set()
    return {s.strip().split(':', 1)[1] for s in out.replace('\n', ',').split(',') if ':' in s and s.strip().startswith(('DNS', 'IP'))}


def ensure_cert(addrs):
    """Self-signed certificate for this car; remade only when it lacks one of the current addresses (a remade
    certificate means the browser warns once more)."""
    os.makedirs(CERT_DIR, exist_ok=True)
    crt, key = os.path.join(CERT_DIR, 'cert.pem'), os.path.join(CERT_DIR, 'key.pem')
    host = socket.gethostname()
    have = cert_names(crt) if os.path.exists(crt) and os.path.exists(key) else set()
    want = {f'{host}.local', host, *addrs}
    if want <= have:
        return crt, key
    names = sorted(want | {n for n in have if n not in ('',)})       # keep old addresses: going back is quiet
    san = ','.join(('IP:' if _is_ip(n) else 'DNS:') + n for n in names)
    subprocess.run(['openssl', 'req', '-x509', '-newkey', 'ec', '-pkeyopt', 'ec_paramgen_curve:prime256v1', '-nodes',
                    '-days', '3650', '-subj', f'/CN={host}.local/O=MentorPi car', '-addext', f'subjectAltName={san}',
                    '-keyout', key, '-out', crt], check=True, capture_output=True)
    os.chmod(key, 0o600)
    log(f'certificate made for {", ".join(names)}')
    return crt, key


def _is_ip(n):
    try:
        ipaddress.ip_address(n)
        return True
    except ValueError:
        return False


async def pipe(reader, writer):
    try:
        while data := await reader.read(65536):
            writer.write(data)
            await writer.drain()
    except (ConnectionError, OSError, ssl.SSLError, asyncio.IncompleteReadError):
        pass
    finally:
        try:
            if writer.can_write_eof():
                writer.write_eof()
            else:
                writer.close()
        except (OSError, RuntimeError, ssl.SSLError):
            writer.close()


def handler(backend):
    async def handle(c_reader, c_writer):
        try:
            b_reader, b_writer = await asyncio.open_connection('127.0.0.1', backend)
        except OSError:
            c_writer.close()
            return
        try:
            await asyncio.gather(pipe(c_reader, b_writer), pipe(b_reader, c_writer))
        finally:
            for w in (b_writer, c_writer):
                w.close()
    return handle


async def main():
    listeners = {}        # (addr, port) -> server
    ctx = None
    cert_for = None
    while True:
        addrs = local_addresses()
        if set(addrs) != cert_for:
            crt, key = ensure_cert(addrs)
            ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            ctx.load_cert_chain(crt, key)
            ctx.set_alpn_protocols(['http/1.1'])        # bytes are forwarded as they are: no HTTP/2
            cert_for = set(addrs)
            for srv in listeners.values():              # reopen with the new certificate
                srv.close()
            listeners = {}
        for k in [k for k in listeners if k[0] not in addrs]:
            listeners.pop(k).close()
            log(f'stopped https on {k[0]}:{k[1]} (address gone)')
        for a in addrs:
            for port, backend in PORTS.items():
                if (a, port) in listeners:
                    continue
                try:
                    listeners[(a, port)] = await asyncio.start_server(handler(backend), a, port, ssl=ctx,
                                                                      ssl_handshake_timeout=10)
                    log(f'https://{"[" + a + "]" if ":" in a else a}:{port} -> 127.0.0.1:{backend}')
                except OSError as e:
                    log(f'could not listen on {a}:{port}: {e}')
        await asyncio.sleep(RESCAN_S)


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        sys.exit(0)
