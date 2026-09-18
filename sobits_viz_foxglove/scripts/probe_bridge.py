#!/usr/bin/env python3
# Copyright 2026 Team SOBITS
# SPDX-License-Identifier: BSD-3-Clause
"""
Check a running bridge without the Foxglove app.

Connects as a viewer would, lists the channels the bridge advertises and, given
a package:// URI, asks for it the way the 3D panel fetches a mesh:

    python3 scripts/probe_bridge.py ws://127.0.0.1:8765 package://<pkg>/meshes/<file>.stl
"""

import argparse
import asyncio
import json
import struct

import websockets

SUBPROTOCOLS = ['foxglove.sdk.v1', 'foxglove.websocket.v1']


async def probe(url: str, asset: str, listen: float) -> None:
    """Report the bridge's channels, and the asset it serves."""
    async with websockets.connect(url, subprotocols=SUBPROTOCOLS, max_size=None) as socket:
        channels, info = {}, None
        deadline = asyncio.get_event_loop().time() + listen
        while asyncio.get_event_loop().time() < deadline:
            try:
                message = await asyncio.wait_for(socket.recv(), timeout=0.5)
            except asyncio.TimeoutError:
                continue
            if isinstance(message, bytes):
                continue
            payload = json.loads(message)
            if payload.get('op') == 'serverInfo':
                info = payload
            elif payload.get('op') == 'advertise':
                for channel in payload['channels']:
                    channels[channel['id']] = (channel['topic'], channel['schemaName'])
            elif payload.get('op') == 'unadvertise':
                for channel_id in payload['channelIds']:
                    channels.pop(channel_id, None)

        print('capabilities:', info and info.get('capabilities'))
        print(f'channels ({len(channels)}):')
        for topic, schema in sorted(channels.values()):
            print(f'  {topic} | {schema}')

        if not asset:
            return
        await socket.send(json.dumps({'op': 'fetchAsset', 'uri': asset, 'requestId': 1}))
        while True:
            message = await asyncio.wait_for(socket.recv(), timeout=10)
            if isinstance(message, bytes) and message[0] == 0x04:
                request, status = struct.unpack_from('<IB', message, 1)
                length = struct.unpack_from('<I', message, 6)[0]
                error = message[10:10 + length].decode()
                data = len(message) - 10 - length
                print(f'fetchAsset {asset}: request {request}, status {status}, '
                      f'error {error!r}, {data} bytes')
                return


def main() -> None:
    """Probe the bridge the arguments name."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('url', nargs='?', default='ws://127.0.0.1:8765')
    parser.add_argument('asset', nargs='?', default=None,
                        help='a package:// URI from the relayed robot description')
    parser.add_argument('--listen', type=float, default=3.0,
                        help='seconds to collect channel advertisements')
    args = parser.parse_args()
    asyncio.run(probe(args.url, args.asset, args.listen))


if __name__ == '__main__':
    main()
