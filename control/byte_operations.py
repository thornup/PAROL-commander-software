
# Split data to 3 bytes
def Split_2_3_bytes(var_in):
    y = int_to_3_bytes(var_in & 0xFFFFFF) # converts my int value to bytes array
    return y

# Splits byte to bitfield list
def Split_2_bitfield(var_in):
    #return [var_in >> i & 1 for i in range(7,-1,-1)]
    return [(var_in >> i) & 1 for i in range(7, -1, -1)]

# Fuses 3 bytes to 1 signed int
def Fuse_3_bytes(var_in):
    value = struct.unpack(">I", bytearray(var_in))[0] # converts bytes array to int

    # convert to negative number if it is negative
    if value >= 1<<23:
        value -= 1<<24

    return value

# Fuses 2 bytes to 1 signed int
def Fuse_2_bytes(var_in):
    value = struct.unpack(">I", bytearray(var_in))[0] # converts bytes array to int

    # convert to negative number if it is negative
    if value >= 1<<15:
        value -= 1<<16

    return value

# Fuse bitfield list to byte
def Fuse_bitfield_2_bytearray(var_in):
    number = 0
    for b in var_in:
        number = (2 * number) + b
    return bytes([number])
