import threading
import multiprocessing
import serial
import time
from spatialmath import *
import platform
import os
import re
from oclock import Timer, loop, interactiveloop
import numpy as np
import math, random
import roboticstoolbox as rp
from roboticstoolbox import trapezoidal
from roboticstoolbox import quintic
from spatialmath.base.argcheck import (
    isvector,
    getvector,
    # assertmatrix,
    getvector,
    isscalar,
)
import struct
import logging


# in big endian machines, first byte of binary representation of the multibyte data-type is stored first.
int_to_3_bytes = struct.Struct('>I').pack # BIG endian order

# Split data to 3 bytes
def split_2_3_bytes(var_in):
    y = int_to_3_bytes(var_in & 0xFFFFFF) # converts my int value to bytes array
    return y

# Splits byte to bitfield list
def split_2_bitfield(var_in):
    #return [var_in >> i & 1 for i in range(7,-1,-1)]
    return [(var_in >> i) & 1 for i in range(7, -1, -1)]

# Fuses 3 bytes to 1 signed int
def fuse_3_bytes(var_in):
    value = struct.unpack(">I", bytearray(var_in))[0] # converts bytes array to int

    # convert to negative number if it is negative
    if value >= 1<<23:
        value -= 1<<24

    return value

# Fuses 2 bytes to 1 signed int
def fuse_2_bytes(var_in):
    value = struct.unpack(">I", bytearray(var_in))[0] # converts bytes array to int

    # convert to negative number if it is negative
    if value >= 1<<15:
        value -= 1<<16

    return value

# Fuse bitfield list to byte
def fuse_bitfield_2_bytearray(var_in):
    number = 0
    for b in var_in:
        number = (2 * number) + b
    return bytes([number])




def extract_from_can_id(can_id):
    # Extracting ID2 (first 4 MSB)
    id2 = (can_id >> 7) & 0xF

    # Extracting CAN Command (next 6 bits)
    can_command = (can_id >> 1) & 0x3F

    # Extracting Error Bit (last bit)
    error_bit = can_id & 0x1

    return id2, can_command, error_bit


def combine_2_can_id(id2, can_command, error_bit):
    # Combine components into an 11-bit CAN ID
    can_id = 0

    # Add ID2 (first 4 MSB)
    can_id |= (id2 & 0xF) << 7

    # Add CAN Command (next 6 bits)
    can_id |= (can_command & 0x3F) << 1

    # Add Error Bit (last bit)
    can_id |= (error_bit & 0x1)

    return can_id


# Fuse bitfield list to byte
def fuse_bitfield_2_bytearray(var_in):
    number = 0
    for b in var_in:
        number = (2 * number) + b
    return bytes([number])


# Splits byte to bitfield list
def split_2_bitfield(var_in):
    # return [var_in >> i & 1 for i in range(7,-1,-1)]
    return [(var_in >> i) & 1 for i in range(7, -1, -1)]
