"""
Modules for conversion between string hex and double(float in python)
Written by Ahn, Jeeho
"""

import struct
import numpy as np
import binascii


def float_to_hex(f):
    return hex(struct.unpack('<I', struct.pack('<f', f))[0])

def hex_to_float(f):
    #remove if 0x exists
    if(f[:2] == '0x'):
        f = f[2:]

    return struct.unpack('!f', bytes.fromhex(f))[0]


def double_to_hex(f)->str:
    """
    Convert python float(double) to string hex
    :param f: Input float
    :returns: hex string
    """
    return hex(struct.unpack('<Q', struct.pack('<d', f))[0])

def hex_to_double(f)->float:
    """
    Convert string hex to double(python float)
    :param f: Input hex string
    
    :returns: Reconstructed float
    """
    #remove if 0x exists
    if(f[:2] == '0x'):
        f = f[2:]

    return struct.unpack('!d', bytes.fromhex(f))[0]

def hex_to_ascii(f):
    #remove if 0x exists
    if(f[:2] == '0x'):
        f = f[2:]
    return binascii.unhexlify(bytes(f,'utf-8'))
def ascii_to_hex(f):
    return binascii.hexlify(bytes(f,'utf-8'))


def hexList_to_floatList(hexList_in:list)->list:
    out_list = []
    for n in range(len(hexList_in)):
        out_list.append(round(hex_to_float(hexList_in[n]),7))

    return out_list

def floatList_to_hexList(floatList_in:list)->list:
    out_list = []
    for n in range(len(floatList_in)):
        out_list.append(float_to_hex(floatList_in[n]))

    return out_list