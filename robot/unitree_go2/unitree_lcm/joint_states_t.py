"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

from io import BytesIO
import struct

class joint_states_t(object):

    __slots__ = ["tick", "q_front_left", "q_front_right", "q_rear_left", "q_rear_right"]

    __typenames__ = ["int64_t", "double", "double", "double", "double"]

    __dimensions__ = [None, [3], [3], [3], [3]]

    def __init__(self):
        self.tick = 0
        """ LCM Type: int64_t """
        self.q_front_left = [ 0.0 for dim0 in range(3) ]
        """ LCM Type: double[3] """
        self.q_front_right = [ 0.0 for dim0 in range(3) ]
        """ LCM Type: double[3] """
        self.q_rear_left = [ 0.0 for dim0 in range(3) ]
        """ LCM Type: double[3] """
        self.q_rear_right = [ 0.0 for dim0 in range(3) ]
        """ LCM Type: double[3] """

    def encode(self):
        buf = BytesIO()
        buf.write(joint_states_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">q", self.tick))
        buf.write(struct.pack('>3d', *self.q_front_left[:3]))
        buf.write(struct.pack('>3d', *self.q_front_right[:3]))
        buf.write(struct.pack('>3d', *self.q_rear_left[:3]))
        buf.write(struct.pack('>3d', *self.q_rear_right[:3]))

    @staticmethod
    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != joint_states_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return joint_states_t._decode_one(buf)

    @staticmethod
    def _decode_one(buf):
        self = joint_states_t()
        self.tick = struct.unpack(">q", buf.read(8))[0]
        self.q_front_left = struct.unpack('>3d', buf.read(24))
        self.q_front_right = struct.unpack('>3d', buf.read(24))
        self.q_rear_left = struct.unpack('>3d', buf.read(24))
        self.q_rear_right = struct.unpack('>3d', buf.read(24))
        return self

    @staticmethod
    def _get_hash_recursive(parents):
        if joint_states_t in parents: return 0
        tmphash = (0x1d53d664211e0282) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _packed_fingerprint = None

    @staticmethod
    def _get_packed_fingerprint():
        if joint_states_t._packed_fingerprint is None:
            joint_states_t._packed_fingerprint = struct.pack(">Q", joint_states_t._get_hash_recursive([]))
        return joint_states_t._packed_fingerprint

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", joint_states_t._get_packed_fingerprint())[0]

