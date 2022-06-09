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
    crc_b: u8
}

impl Parser {
    pub const fn new() -> Self {
        Parser{
            packet: Packet{id: 0, length: 0, data: [0; MAXLENGTH]},
            state: ParseState::Sync1,
            count: 0,
            crc_a: 0,
            crc_b: 0}
    }

    pub fn reset(&mut self) {
        self.state = ParseState::Sync1;
        self.count = 0;
    }

    pub fn push_byte(&mut self, b: u8) -> bool {
        let mut packet_complete = false;
        match self.state {
            ParseState::Sync1 => {
                if b == SYNCBYTE1 {
                    self.state = ParseState::Sync2;
                }
            },
            ParseState::Sync2 => {
                if b == SYNCBYTE2 {
                    self.state = ParseState::Id1;
                } else {
                    self.state = ParseState::Sync1; // Abort
                }
            },
            ParseState::Id1 => {
                self.packet.id = u16::from(b);
                self.state = ParseState::Id2;
            },
            ParseState::Id2 => {
                self.packet.id |= u16::from(b) << 8;
                self.state = ParseState::Length;
            },
            ParseState::Length => {
                if usize::from(b) < MAXLENGTH {
                    self.packet.length = b;
                    self.count = 0;
                    self.state = ParseState::Data;
                } else {
                    self.state = ParseState::Sync1; // Abort
                }
            },
            ParseState::Data => {
                self.packet.data[usize::from(self.count)] = b;
                self.count += 1;
                if self.count == self.packet.length {
                    self.state = ParseState::Crc1;
                }
            },
            ParseState::Crc1 => {
                self.crc_a = b;
                self.state = ParseState::Crc2;
            },
            ParseState::Crc2 => {
                self.crc_b = b;
                self.state = ParseState::Sync1;
                packet_complete = true;
            }
        }
        return packet_complete;
    }
}

