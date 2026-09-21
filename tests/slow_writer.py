"""Deliberately slow bounded-transport sink for the native writer resource test."""
import argparse
import json
from pathlib import Path
import struct
import sys
import time


def exact(size):
    data=bytearray()
    while len(data)<size:
        block=sys.stdin.buffer.read(size-len(data))
        if not block:raise EOFError('Missing explicit writer end record')
        data.extend(block)
    return data


parser=argparse.ArgumentParser();parser.add_argument('--output',required=True)
args=parser.parse_args();count=0
while True:
    size=struct.unpack('<I',exact(4))[0]
    if size==0:break
    header=json.loads(exact(size))
    assert header['sequence']==count
    payload=exact(header['size'])
    assert payload==bytes([count])*header['size']
    count+=1
    time.sleep(.02)
assert count==80
Path(args.output).write_text(json.dumps(dict(packets=count,explicit_end=True))+'\n')
