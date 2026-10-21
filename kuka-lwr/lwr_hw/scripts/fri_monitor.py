#!/usr/bin/env python3
"""Passive FRI connection check: receives the KRC measurement packets, never replies.

Run it while no lwr_hw is bound to the port (it needs the same UDP port). It prints
the packet rate and the interface state/quality, drives power, errors and joint angles
of the last packet, so the cable/IP/port/KRL setup can be checked without ROS.
"""
import argparse
import math
import socket
import struct
import sys
import time

# offsets in tFriMsrData (include/fri/friComm.h): head 8 B, krl 132 B, intf, robot, data
HEAD, INTF, STAT, ROBOT, MSR_JNT = 0, 140, 160, 180, 216
STATES = {0: 'OFF', 1: 'MONITOR', 2: 'COMMAND'}
QUALITIES = {0: 'UNACCEPTABLE', 1: 'BAD', 2: 'OK', 3: 'PERFECT'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=49938)
    parser.add_argument('--duration', type=float, default=3.0)
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(('0.0.0.0', args.port))
    except OSError as e:
        print(f'cannot bind UDP {args.port} ({e}); stop lwr_hw / ros2_control_node first')
        sys.exit(2)
    sock.settimeout(args.duration)
    count, last = 0, None
    t0 = time.time()
    try:
        while time.time() - t0 < args.duration:
            last = sock.recvfrom(4096)
            count += 1
    except socket.timeout:
        pass
    elapsed = time.time() - t0
    if last is None:
        print(f'no FRI packets on UDP {args.port} in {args.duration:.1f} s')
        sys.exit(1)

    data, addr = last
    _, _, size, datagram_id = struct.unpack_from('<4H', data, HEAD)
    _, state, quality, msr_t, cmd_t, _ = struct.unpack_from('<f2H3f', data, INTF)
    answer_rate, latency, _, miss_rate, miss_counter = struct.unpack_from('<4fI', data, STAT)
    power, control, error, warning = struct.unpack_from('<4H', data, ROBOT)
    q = struct.unpack_from('<7f', data, MSR_JNT)

    print(f'from {addr[0]}:{addr[1]}, {count} packets in {elapsed:.2f} s (~{count / elapsed:.0f} Hz), '
          f'size={size}, datagramId=0x{datagram_id:04x}')
    print(f'state={STATES.get(state, state)} quality={QUALITIES.get(quality, quality)} '
          f'msrSampleTime={msr_t:.3f} s cmdSampleTime={cmd_t:.3f} s')
    print(f'answerRate={answer_rate:.3f} latency={latency:.4f} missRate={miss_rate:.3f} missCounter={miss_counter}')
    print(f'power=0x{power:x} control={control} error={error} warning={warning}')
    print('msrJntPos [deg]:', [round(math.degrees(x), 2) for x in q])


if __name__ == '__main__':
    main()
