"""Robotiq register interface and configurable PLC signals, outside the RT loop."""
import struct
import time
from .modbus import ModbusTCP, integer


def endpoint(config):
    return ModbusTCP(**config)


class Robotiq:
    """2F register map via a transparent TCP/RTU gateway or verified controller.

    TCP transport is not a native port on every 2F-85. The gateway and register
    mapping must match the device's manual. No automatic fault reset on motion.
    """
    def __init__(self, config):
        self.bus = endpoint(config['endpoint'])
        self.read_kind = config['read_kind']
        if self.read_kind not in ('input', 'holding'):
            raise ValueError('Robotiq read_kind must be input or holding')
        self.input = integer(config['status_address'], 0, 65533)
        self.output = integer(config['command_address'], 0, 65533)

    def status(self):
        data = struct.pack('>3H', *self.bus.read(self.read_kind, self.input, 3))
        return dict(active=bool(data[0]&1), go=bool(data[0]&8),
                    activation=(data[0]>>4)&3, object=(data[0]>>6)&3,
                    fault=data[2], requested=data[3], position=data[4], current_raw=data[5])

    def command(self, action, position=0, speed=0, force=0):
        values = [integer(v, 0, 255) for v in (action, position, speed, force)]
        data = bytes([values[0], 0, 0, *values[1:]])
        self.bus.write_registers(self.output, struct.unpack('>3H', data))

    def wait(self, predicate, timeout_s, cancelled=lambda:False, activating=False):
        end = time.monotonic()+timeout_s
        while time.monotonic()<end:
            if cancelled():
                raise RuntimeError('Gripper action cancelled')
            s = self.status()
            if s['fault'] and not (activating and s['fault'] in (5, 7)):
                raise RuntimeError(f'Gripper fault {s["fault"]:#x}')
            if predicate(s):
                return s
            time.sleep(.02)
        raise TimeoutError('Gripper feedback timeout')

    def activate(self, timeout_s=5, cancelled=lambda:False):
        # Explicit CLI operation: activation itself moves the fingers.
        self.command(0)
        self.wait(lambda s:not s['active'] and s['activation']==0, timeout_s, cancelled, True)
        self.command(1)
        return self.wait(lambda s:s['active'] and s['activation']==3 and s['fault']==0,
                         timeout_s, cancelled, True)

    def move(self, position, speed, force, *, grasp=False, timeout_s=5, cancelled=lambda:False):
        integer(position, 0, 255); integer(speed, 0, 255); integer(force, 0, 255)
        s = self.status()
        if not s['active'] or s['activation']!=3 or s['fault']:
            raise RuntimeError('Gripper must be explicitly activated and free of faults')
        self.command(9, position, speed, force)
        def completed(s):
            if not s['active'] or s['activation']!=3:
                raise RuntimeError('Gripper lost activation')
            if not s['go'] or s['requested']!=position:
                return False
            if s['object']==3 and grasp:
                raise RuntimeError('Gripper reached requested position without confirmed object')
            return s['object']==2 if grasp else s['object']==3 and abs(s['position']-position)<=3
        return self.wait(completed, timeout_s, cancelled)

    def stop(self):
        # Keep activation; no emergency auto-release or automatic opening.
        self.command(1)


class PLC:
    def __init__(self, config):
        self.bus = endpoint(config['endpoint'])
        self.signals = config['signals']

    def read(self, name):
        row = self.signals[name]
        raw = self.bus.read(row['kind'], row['address'])[0]
        if row['kind'] in ('coil','discrete'):
            if type(row['active']) is not bool:
                raise ValueError('Bit signal active must be boolean')
            return raw == row['active']
        if row['kind'] not in ('input','holding'):
            raise ValueError('Unsupported PLC read kind')
        mask = integer(row['mask'], 1, 65535)
        active = integer(row['active'], 0, 65535)
        if active & ~mask:
            raise ValueError('PLC active value exceeds mask')
        return raw & mask == active

    def write(self, name, asserted):
        row = self.signals[name]
        if row['kind']!='coil' or type(row['active']) is not bool or type(asserted) is not bool:
            raise ValueError('Outputs require explicitly mapped boolean coils')
        self.bus.write_coil(row['address'], row['active'] if asserted else not row['active'])

    def ready(self):
        return self.read('cell_ready') and not self.read('cell_fault')

    def wait(self, signal, *, timeout_s=3, dwell_s=.1, cancelled=lambda:False, expected=True):
        if type(expected) is not bool:
            raise ValueError('Expected PLC state must be boolean')
        end = time.monotonic()+timeout_s
        since = None
        while time.monotonic()<end:
            if cancelled() or not self.ready():
                raise RuntimeError('PLC interlock lost')
            if self.read(signal)==expected:
                since = since if since is not None else time.monotonic()
                if time.monotonic()-since >= dwell_s:
                    return
            else:
                since = None
            time.sleep(.02)
        raise TimeoutError('PLC signal timeout: '+signal)
