enum ParseState {
    Sync1,
    Sync2,
    Id1,
    Id2,
    Length,
    Data,
    Crc1,
    Crc2
}

const MAXLENGTH: usize = 32;
const SYNCBYTE1: u8 = 2;
const SYNCBYTE2: u8 = 3;

pub struct Packet {
    pub id: u16,
    pub length: u8,
    pub data: [u8; MAXLENGTH],
}

pub struct Parser {
    pub packet: Packet,
    state: ParseState,
    count: u8,
    crc_a: u8,
    crc_b: u8,
    rx_crc_a: u8,
    rx_crc_b: u8,
}

impl Parser {
    pub const fn new() -> Self {
        Parser{
            packet: Packet{id: 0, length: 0, data: [0; MAXLENGTH]},
            state: ParseState::Sync1,
            count: 0,
            crc_a: 0,
            crc_b: 0,
            rx_crc_a: 0,
            rx_crc_b: 0,
        }
    }

    pub fn reset(&mut self) {
        self.state = ParseState::Sync1;
        self.count = 0;
        self.crc_a = 0;
        self.crc_b = 0;
    }

    fn update_crc(&mut self, b: u8) {
        self.crc_a = self.crc_a.wrapping_add(b);
        self.crc_b = self.crc_b.wrapping_add(self.crc_a);
    }

    pub fn push_byte(&mut self, b: u8) -> bool {
        let mut packet_complete = false;

        match self.state {
            ParseState::Sync1 => {
                if b == SYNCBYTE1 {
                    self.state = ParseState::Sync2;
                    self.update_crc(b);
                }
            },
            ParseState::Sync2 => {
                if b == SYNCBYTE2 {
                    self.state = ParseState::Id1;
                    self.update_crc(b);
                } else {
                    self.reset(); // Abort
                }
            },
            ParseState::Id1 => {
                self.packet.id = u16::from(b);
                self.state = ParseState::Id2;
                self.update_crc(b);
            },
            ParseState::Id2 => {
                self.packet.id |= u16::from(b) << 8;
                self.state = ParseState::Length;
                self.update_crc(b);
            },
            ParseState::Length => {
                if usize::from(b) < MAXLENGTH {
                    self.packet.length = b;
                    self.count = 0;
                    if self.packet.length == 0 {
                        self.state = ParseState::Crc1;
                    } else {
                        self.state = ParseState::Data;
                    }
                    self.update_crc(b);
                } else {
                    self.reset() // Abort
                }
            },
            ParseState::Data => {
                self.packet.data[usize::from(self.count)] = b;
                self.count += 1;
                self.update_crc(b);
                if self.count == self.packet.length {
                    self.state = ParseState::Crc1;
                }
            },
            ParseState::Crc1 => {
                self.rx_crc_a = b;
                self.state = ParseState::Crc2;
            },
            ParseState::Crc2 => {
                self.rx_crc_b = b;

                if self.crc_a == self.rx_crc_a && self.crc_b == self.rx_crc_b {
                    packet_complete = true;
                }
                self.reset();
            }
        }
        return packet_complete;
    }
}

