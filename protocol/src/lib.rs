use bytemuck::{Pod, Zeroable};

const PACKET_LEN: u32 = 496;

#[derive(Clone, Copy, Zeroable, Pod, Debug)]
#[repr(C)]
pub struct Control {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub throttle: f32,
}

/// The packet messaging protocol.
/// It is assumed that life is hard but not too hard. More precisely:
/// - Packets might be randomly missing, but that is rare.
/// - Packets will be delivered more or less in order. In particular, if new message is available and
///     previous one is not complete, the previous one is most likely no longer coming,
///     so all packets for it should be dropped.
#[derive(Clone, Copy, Zeroable, Pod)]
#[repr(C)]
pub struct Packet {
    message_id: u32,
    length: u32,
    packet_id: u32,
    /// Supposedly almost the biggest message size that can be sent without splitting
    buf: [u8; PACKET_LEN as usize],
}

impl Packet {
    pub fn new() -> Packet {
        Packet {
            message_id: 0,
            length: 0,
            packet_id: 0,
            buf: [0; PACKET_LEN as usize],
        }
    }
}

pub struct Packets(pub Vec<Packet>);
pub enum ParseError {
    NoFullMessage,
    DroppedPacket(u32),
}

impl Packets {
    pub fn build_packets(buf: &[u8], message_id: u32) -> Packets {
        let len = buf.len();

        let packets = buf
            .chunks(PACKET_LEN as usize)
            .enumerate()
            .map(|(i, x)| {
                let mut buf = [0; PACKET_LEN as usize];
                for i in 0..x.len().min(PACKET_LEN as usize) {
                    buf[i] = x[i];
                }

                Packet {
                    message_id,
                    length: len as u32,
                    packet_id: i as u32,
                    buf,
                }
            })
            .collect();

        Packets(packets)
    }

    /// Try to parse. Clear the parsed or impossible to parse payload.
    pub fn parse_and_clear(self: &mut Packets) -> Result<Vec<u8>, ParseError> {
        if self.0.is_empty() {
            return Err(ParseError::NoFullMessage);
        }
        // will not fail as we checked it
        let first = self.0.first().unwrap();
        // should work due to assumption about rough ordering
        let first_id = first.message_id;
        let length = first.length;
        if first_id != self.0.last().unwrap().message_id {
            self.0 = self
                .0
                .clone()
                .into_iter()
                .filter(|x| x.message_id != first_id)
                .collect();
            return Err(ParseError::DroppedPacket(first_id));
        }
        let len_to_parse = length.div_ceil(PACKET_LEN);
        if len_to_parse > self.0.len() as u32 {
            return Err(ParseError::NoFullMessage);
        }
        self.0.sort_by(|x, y| {
            if x.message_id != y.message_id {
                x.message_id.cmp(&y.message_id)
            } else {
                x.packet_id.cmp(&y.packet_id)
            }
        });
        let remainder = self.0.split_off(len_to_parse as usize);
        let mut output_vec: Vec<u8> = self.0.iter().flat_map(|x| x.buf).collect();
        self.0 = remainder;
        if first_id % 100 == 0 {
            println!("received {}", first_id);
        }
        Ok(output_vec)
    }
}
