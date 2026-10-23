"""Bounded Modbus TCP transactions. Addresses are zero-based PDU addresses."""
import socket
import ipaddress
import struct
import threading
import time


def integer(value, low, high):
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f'Expected integer in [{low}, {high}]')
    return value


class ModbusTCP:
    def __init__(self, host, port=502, unit=1, timeout_s=.2):
        if not isinstance(host, str) or not host.strip():
            raise ValueError('A device host is required')
        ipaddress.ip_address(host)  # No unbounded DNS resolution in the I/O path.
        self.host, self.port, self.unit = host, integer(port, 1, 65535), integer(unit, 0, 255)
        if not .01 <= timeout_s <= 2:
            raise ValueError('Modbus timeout must be between .01 and 2 seconds')
        self.timeout = timeout_s
        self.lock, self.transaction = threading.Lock(), 0

    def request(self, function, payload):
        with self.lock:
            self.transaction = (self.transaction + 1) % 65536
            tid = self.transaction
            deadline = time.monotonic() + self.timeout
            with socket.create_connection((self.host, self.port), self.timeout) as sock:
                def receive(count):
                    data = b''
                    while len(data) < count:
                        remaining = deadline - time.monotonic()
                        if remaining <= 0:
                            raise TimeoutError('Modbus transaction timed out')
                        sock.settimeout(remaining)
                        block = sock.recv(count - len(data))
                        if not block:
                            raise ConnectionError('Truncated Modbus response')
                        data += block
                    return data
                sock.settimeout(max(.001, deadline - time.monotonic()))
                sock.sendall(struct.pack('>HHHBB', tid, 0, len(payload)+2, self.unit, function)+payload)
                rt, protocol, length, unit = struct.unpack('>HHHB', receive(7))
                if (rt, protocol, unit) != (tid, 0, self.unit) or not 2 <= length <= 254:
                    raise RuntimeError('Invalid Modbus response header')
                body = receive(length-1)
                if body[0] == function | 128:
                    raise RuntimeError(f'Modbus device exception: {body[1:].hex()}')
                if body[0] != function:
                    raise RuntimeError('Wrong Modbus response function')
                return body[1:]

    def read(self, kind, address, count=1):
        function = {'coil':1, 'discrete':2, 'holding':3, 'input':4}[kind]
        integer(address, 0, 65535); integer(count, 1, 125 if function > 2 else 2000)
        if address+count > 65536:
            raise ValueError('Modbus address range overflow')
        result = self.request(function, struct.pack('>HH', address, count))
        size = count*2 if function > 2 else (count+7)//8
        if len(result) != size+1 or result[0] != size:
            raise RuntimeError('Invalid Modbus byte count')
        if function > 2:
            return list(struct.unpack('>'+str(count)+'H', result[1:]))
        return [bool(result[1+i//8] & (1 << (i%8))) for i in range(count)]

    def write_coil(self, address, value):
        integer(address, 0, 65535)
        if type(value) is not bool:
            raise ValueError('Coil values must be boolean')
        payload = struct.pack('>HH', address, 0xff00 if value else 0)
        if self.request(5, payload) != payload:
            raise RuntimeError('Coil write acknowledgement mismatch')

    def write_registers(self, address, values):
        integer(address, 0, 65535); integer(len(values), 1, 123)
        if address+len(values)>65536:
            raise ValueError('Modbus address range overflow')
        values = [integer(v, 0, 65535) for v in values]
        header = struct.pack('>HH', address, len(values))
        payload = header+bytes([len(values)*2])+struct.pack('>'+str(len(values))+'H', *values)
        if self.request(16, payload) != header:
            raise RuntimeError('Register write acknowledgement mismatch')
