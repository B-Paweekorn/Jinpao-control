from ctypes import Union, Array, c_uint8, c_double

class byte_array(Array):
    _type_ = c_uint8
    _length_ = 8

class msgs_convert(Union):
    _fields_ = ("f64", c_double), ("u8", byte_array)

class mcu_msgs:
    def __init__(self, msg_size: int, header: int, carry_bit: int):
        self.buff = [0] * (msg_size * 8)
        self.header = header
        self.carry_bit = carry_bit

        self.state = "header"
        self.counter = 0
        self.checksum = 0
        self.status = False

    def encode_msg2buffer(self):
        tx_buff = []
        tx_buff.append(self.header)
        tx_buff.append(0xff - self.header)
        tx_buff.append(self.header)
        tx_buff.extend(self.buff)
        checksum = 0
        for i in range(len(self.buff)):
            checksum = (checksum + self.buff[i]) & 0xFF
        checksum = checksum // 256
        checksum = (checksum + self.carry_bit) & 0xFF
        checksum = ~checksum & 0xFF
        tx_buff.append(checksum)
        return tx_buff

    def decode_buffer2msg(self, val_byte: bytes):
        val = int.from_bytes(val_byte, "big")
        if self.state == "header":
            self.status = False
            if val == self.header: self.state = "header1"
        elif self.state == "header1":
            if val == 0xff - self.header: self.state = "header2"
            else: self.state = "header"
        elif self.state == "header2":
            if val == self.header: self.state = "recieve"; self.counter = 0; self.checksum = 0
            else: self.state = "header"
        elif self.state == "recieve":
            self.buff[self.counter] = val
            self.checksum = (self.checksum + val) & 0xFF
            self.counter = self.counter + 1
            if self.counter == len(self.buff): self.state = "end"
        elif self.state == "end":
            self.checksum = self.checksum // 256
            self.checksum = (self.checksum + self.carry_bit) & 0xFF
            self.checksum = ~self.checksum & 0xFF
            self.status = self.checksum == val
            self.state = "header"
    
    def get_msgs(self, index: int):
        msgs = msgs_convert()
        msgs.u8[:] = [self.buff[i] for i in range((index * 8), ((index + 1) * 8))]
        return float(msgs.f64)
    
    def assign_msgs(self, index: int, msg_f64: c_double):
        msgs = msgs_convert()
        msgs.f64 = msg_f64
        for i in range(8):
            self.buff[(8 * index) + i] = msgs.u8[i]