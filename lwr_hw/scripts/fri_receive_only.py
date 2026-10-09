#!/usr/bin/env python3
"""Receive and decode KUKA FRI measurements without sending any packets."""
import argparse
import datetime
import json
import math
import selectors
import socket
import struct
import time


def decode(data):
    # Layout from src/lwr_hw/include/fri/friComm.h (FRI v1.0).
    if len(data) != 916:
        return None
    seq, reflected, size, ident = struct.unpack_from('<4H', data)
    if size != 916 or ident != 0x2006:
        return None
    stamp, state, quality, msr_period, cmd_period = struct.unpack_from('<fHHff', data, 140)
    power, control, error, warning = struct.unpack_from('<4H', data, 180)
    positions = struct.unpack_from('<7f', data, 216)
    torques = struct.unpack_from('<7f', data, 444)
    external = struct.unpack_from('<7f', data, 472)
    return dict(sequence=seq, reflected_sequence=reflected, robot_timestamp_s=stamp,
                state={0: 'OFF', 1: 'MONITOR', 2: 'COMMAND'}.get(state, str(state)),
                quality={0: 'UNACCEPTABLE', 1: 'BAD', 2: 'OK', 3: 'PERFECT'}.get(quality, str(quality)),
                measurement_period_s=msr_period, command_period_s=cmd_period,
                power_bitfield=power, control=control, error_bitfield=error,
                warning_bitfield=warning, joint_position_rad=list(positions),
                joint_position_deg=[math.degrees(v) for v in positions],
                measured_joint_torque_nm=list(torques), external_joint_torque_nm=list(external))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bind', default='192.168.10.1')
    parser.add_argument('--ports', nargs='+', type=int, default=[49938, 49939])
    parser.add_argument('--seconds', type=float, default=10)
    parser.add_argument('--output', help='Optional JSON report path')
    args = parser.parse_args()
    if args.seconds <= 0 or not all(1 <= p <= 65535 for p in args.ports):
        parser.error('seconds must be positive and ports must be 1..65535')
    report = dict(started_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                  receive_only=True, transmitted_udp_packets=0, bind=args.bind, ports={})
    with selectors.DefaultSelector() as selector:
        sockets = []
        try:
            for port in dict.fromkeys(args.ports):
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sockets.append(sock)
                # No address reuse: never share a running driver's port.
                sock.bind((args.bind, port))
                sock.setblocking(False)
                selector.register(sock, selectors.EVENT_READ, port)
                report['ports'][str(port)] = dict(datagrams=0, valid_fri_packets=0,
                    sequence_gaps=0, last_sequence=None, first_receive_s=None,
                    last_receive_s=None, invalid_packets=0)
            start = time.monotonic()
            while time.monotonic() - start < args.seconds:
                for key, _ in selector.select(max(0, min(0.2, args.seconds - (time.monotonic() - start)))):
                    data, peer = key.fileobj.recvfrom(65535)
                    entry = report['ports'][str(key.data)]
                    entry['datagrams'] += 1
                    packet = decode(data)
                    if packet is None:
                        entry['invalid_packets'] += 1
                        continue
                    now = time.monotonic() - start
                    entry['valid_fri_packets'] += 1
                    if entry['first_receive_s'] is None:
                        entry['first_receive_s'] = now
                        entry['first_packet'] = packet
                    if entry['last_sequence'] is not None:
                        delta = (packet['sequence'] - entry['last_sequence']) % 65536
                        if 1 < delta < 32768:
                            entry['sequence_gaps'] += delta - 1
                    entry.update(last_sequence=packet['sequence'], last_receive_s=now,
                                 source=list(peer), last_packet=packet)
            report['duration_s'] = time.monotonic() - start
            for entry in report['ports'].values():
                span = (entry['last_receive_s'] or 0) - (entry['first_receive_s'] or 0)
                entry['observed_hz'] = (entry['valid_fri_packets'] - 1) / span if span > 0 else None
        finally:
            for sock in sockets:
                sock.close()
    rendered = json.dumps(report, indent=2, ensure_ascii=False)
    if args.output:
        with open(args.output, 'w', encoding='utf-8') as output:
            output.write(rendered + '\n')
    print(rendered)


if __name__ == '__main__':
    main()
