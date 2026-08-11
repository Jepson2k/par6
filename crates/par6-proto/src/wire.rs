//! Minimal msgpack primitives used by the codec.
//!
//! Hand-rolled rather than serde-based: every protocol payload is a positional
//! array with an integer tag in slot 0 and heterogeneous fields after it, and
//! golden-vector byte equality across languages requires full control over the
//! encoding. Writers always emit the *smallest* representation (what msgspec
//! and ormsgpack emit on the Python side); floats are always 9-byte float64.

use crate::DecodeError;

// ---------------------------------------------------------------------------
// Writers (append to a caller-owned Vec<u8>; no other allocations)
// ---------------------------------------------------------------------------

/// Write an array header for `n` elements.
pub(crate) fn w_array(buf: &mut Vec<u8>, n: usize) {
    if n < 16 {
        buf.push(0x90 | n as u8);
    } else if n <= 0xFFFF {
        buf.push(0xDC);
        buf.extend_from_slice(&(n as u16).to_be_bytes());
    } else {
        buf.push(0xDD);
        buf.extend_from_slice(&(n as u32).to_be_bytes());
    }
}

/// Write an unsigned integer (smallest encoding).
pub(crate) fn w_uint(buf: &mut Vec<u8>, v: u64) {
    if v < 0x80 {
        buf.push(v as u8);
    } else if v <= 0xFF {
        buf.push(0xCC);
        buf.push(v as u8);
    } else if v <= 0xFFFF {
        buf.push(0xCD);
        buf.extend_from_slice(&(v as u16).to_be_bytes());
    } else if v <= 0xFFFF_FFFF {
        buf.push(0xCE);
        buf.extend_from_slice(&(v as u32).to_be_bytes());
    } else {
        buf.push(0xCF);
        buf.extend_from_slice(&v.to_be_bytes());
    }
}

/// Write a signed integer (non-negative values use the unsigned encodings).
pub(crate) fn w_int(buf: &mut Vec<u8>, v: i64) {
    if v >= 0 {
        w_uint(buf, v as u64);
    } else if v >= -32 {
        buf.push(v as u8); // negative fixint
    } else if v >= i64::from(i8::MIN) {
        buf.push(0xD0);
        buf.push(v as u8);
    } else if v >= i64::from(i16::MIN) {
        buf.push(0xD1);
        buf.extend_from_slice(&(v as i16).to_be_bytes());
    } else if v >= i64::from(i32::MIN) {
        buf.push(0xD2);
        buf.extend_from_slice(&(v as i32).to_be_bytes());
    } else {
        buf.push(0xD3);
        buf.extend_from_slice(&v.to_be_bytes());
    }
}

/// Write a float64 (always the full 9-byte encoding, never float32).
pub(crate) fn w_f64(buf: &mut Vec<u8>, v: f64) {
    buf.push(0xCB);
    buf.extend_from_slice(&v.to_bits().to_be_bytes());
}

/// Write a UTF-8 string.
pub(crate) fn w_str(buf: &mut Vec<u8>, s: &str) {
    let n = s.len();
    if n < 32 {
        buf.push(0xA0 | n as u8);
    } else if n <= 0xFF {
        buf.push(0xD9);
        buf.push(n as u8);
    } else if n <= 0xFFFF {
        buf.push(0xDA);
        buf.extend_from_slice(&(n as u16).to_be_bytes());
    } else {
        buf.push(0xDB);
        buf.extend_from_slice(&(n as u32).to_be_bytes());
    }
    buf.extend_from_slice(s.as_bytes());
}

/// Write a binary blob.
pub(crate) fn w_bin(buf: &mut Vec<u8>, b: &[u8]) {
    let n = b.len();
    if n <= 0xFF {
        buf.push(0xC4);
        buf.push(n as u8);
    } else if n <= 0xFFFF {
        buf.push(0xC5);
        buf.extend_from_slice(&(n as u16).to_be_bytes());
    } else {
        buf.push(0xC6);
        buf.extend_from_slice(&(n as u32).to_be_bytes());
    }
    buf.extend_from_slice(b);
}

/// Write a boolean.
pub(crate) fn w_bool(buf: &mut Vec<u8>, v: bool) {
    buf.push(if v { 0xC3 } else { 0xC2 });
}

/// Write nil (the single "unspecified" convention on the wire).
pub(crate) fn w_nil(buf: &mut Vec<u8>) {
    buf.push(0xC0);
}

// ---------------------------------------------------------------------------
// Reader
// ---------------------------------------------------------------------------

/// Positional msgpack reader over a byte slice.
///
/// Typed accessors are strict about the msgpack family (a float field must be
/// float64 on the wire, an unsigned field must use an unsigned encoding) so a
/// malformed datagram fails loudly instead of being coerced.
pub(crate) struct Reader<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    pub(crate) fn new(data: &'a [u8]) -> Self {
        Reader { data, pos: 0 }
    }

    fn type_err(&self, expected: &'static str, found: u8) -> DecodeError {
        DecodeError::Type {
            expected,
            found,
            pos: self.pos.saturating_sub(1),
        }
    }

    fn byte(&mut self) -> Result<u8, DecodeError> {
        let b = *self.data.get(self.pos).ok_or(DecodeError::Truncated)?;
        self.pos += 1;
        Ok(b)
    }

    fn take(&mut self, n: usize) -> Result<&'a [u8], DecodeError> {
        let end = self.pos.checked_add(n).ok_or(DecodeError::Truncated)?;
        let s = self.data.get(self.pos..end).ok_or(DecodeError::Truncated)?;
        self.pos = end;
        Ok(s)
    }

    fn be_u16(&mut self) -> Result<u16, DecodeError> {
        let b = self.take(2)?;
        Ok(u16::from_be_bytes([b[0], b[1]]))
    }

    fn be_u32(&mut self) -> Result<u32, DecodeError> {
        let b = self.take(4)?;
        Ok(u32::from_be_bytes([b[0], b[1], b[2], b[3]]))
    }

    fn be_u64(&mut self) -> Result<u64, DecodeError> {
        let b = self.take(8)?;
        let mut a = [0u8; 8];
        a.copy_from_slice(b);
        Ok(u64::from_be_bytes(a))
    }

    /// Read an array header, returning the element count.
    pub(crate) fn array_len(&mut self) -> Result<usize, DecodeError> {
        let m = self.byte()?;
        match m {
            0x90..=0x9F => Ok((m & 0x0F) as usize),
            0xDC => Ok(self.be_u16()? as usize),
            0xDD => Ok(self.be_u32()? as usize),
            _ => Err(self.type_err("array", m)),
        }
    }

    /// Read an unsigned integer (unsigned encodings only).
    pub(crate) fn uint(&mut self) -> Result<u64, DecodeError> {
        let m = self.byte()?;
        match m {
            0x00..=0x7F => Ok(u64::from(m)),
            0xCC => Ok(u64::from(self.byte()?)),
            0xCD => Ok(u64::from(self.be_u16()?)),
            0xCE => Ok(u64::from(self.be_u32()?)),
            0xCF => Ok(self.be_u64()?),
            _ => Err(self.type_err("unsigned int", m)),
        }
    }

    /// Read a signed integer (accepts unsigned encodings up to `i64::MAX`).
    pub(crate) fn int(&mut self) -> Result<i64, DecodeError> {
        let m = self.byte()?;
        match m {
            0x00..=0x7F => Ok(i64::from(m)),
            0xE0..=0xFF => Ok(i64::from(m as i8)),
            0xCC => Ok(i64::from(self.byte()?)),
            0xCD => Ok(i64::from(self.be_u16()?)),
            0xCE => Ok(i64::from(self.be_u32()?)),
            0xCF => {
                let v = self.be_u64()?;
                i64::try_from(v).map_err(|_| self.type_err("int (fits i64)", 0xCF))
            }
            0xD0 => Ok(i64::from(self.byte()? as i8)),
            0xD1 => Ok(i64::from(self.be_u16()? as i16)),
            0xD2 => Ok(i64::from(self.be_u32()? as i32)),
            0xD3 => Ok(self.be_u64()? as i64),
            _ => Err(self.type_err("int", m)),
        }
    }

    /// Read a float64 (strict: float32 or integer encodings are rejected).
    pub(crate) fn f64(&mut self) -> Result<f64, DecodeError> {
        let m = self.byte()?;
        if m != 0xCB {
            return Err(self.type_err("float64", m));
        }
        Ok(f64::from_bits(self.be_u64()?))
    }

    /// Read a UTF-8 string.
    pub(crate) fn str(&mut self) -> Result<&'a str, DecodeError> {
        let m = self.byte()?;
        let n = match m {
            0xA0..=0xBF => (m & 0x1F) as usize,
            0xD9 => self.byte()? as usize,
            0xDA => self.be_u16()? as usize,
            0xDB => self.be_u32()? as usize,
            _ => return Err(self.type_err("str", m)),
        };
        std::str::from_utf8(self.take(n)?).map_err(|_| DecodeError::Utf8)
    }

    /// Read a binary blob.
    pub(crate) fn bin(&mut self) -> Result<&'a [u8], DecodeError> {
        let m = self.byte()?;
        let n = match m {
            0xC4 => self.byte()? as usize,
            0xC5 => self.be_u16()? as usize,
            0xC6 => self.be_u32()? as usize,
            _ => return Err(self.type_err("bin", m)),
        };
        self.take(n)
    }

    /// Read a boolean.
    pub(crate) fn bool(&mut self) -> Result<bool, DecodeError> {
        let m = self.byte()?;
        match m {
            0xC2 => Ok(false),
            0xC3 => Ok(true),
            _ => Err(self.type_err("bool", m)),
        }
    }

    /// True if the next value is nil (does not consume).
    pub(crate) fn peek_nil(&self) -> bool {
        self.data.get(self.pos) == Some(&0xC0)
    }

    /// The next marker byte, without consuming it.
    pub(crate) fn peek_marker(&self) -> Result<u8, DecodeError> {
        self.data
            .get(self.pos)
            .copied()
            .ok_or(DecodeError::Truncated)
    }

    /// Consume a nil marker.
    pub(crate) fn nil(&mut self) -> Result<(), DecodeError> {
        let m = self.byte()?;
        if m != 0xC0 {
            return Err(self.type_err("nil", m));
        }
        Ok(())
    }

    /// Read `nil | float64`.
    pub(crate) fn opt_f64(&mut self) -> Result<Option<f64>, DecodeError> {
        if self.peek_nil() {
            self.nil()?;
            Ok(None)
        } else {
            Ok(Some(self.f64()?))
        }
    }

    /// Skip one value of any type (used for forward-compatible tails).
    pub(crate) fn skip_value(&mut self) -> Result<(), DecodeError> {
        let m = self.byte()?;
        match m {
            0x00..=0x7F | 0xE0..=0xFF | 0xC0 | 0xC2 | 0xC3 => {}
            0xA0..=0xBF => {
                let n = (m & 0x1F) as usize;
                self.take(n)?;
            }
            0x90..=0x9F => {
                for _ in 0..(m & 0x0F) {
                    self.skip_value()?;
                }
            }
            0x80..=0x8F => {
                for _ in 0..(2 * (m & 0x0F)) {
                    self.skip_value()?;
                }
            }
            0xCC | 0xD0 => {
                self.take(1)?;
            }
            0xCD | 0xD1 => {
                self.take(2)?;
            }
            0xCE | 0xD2 | 0xCA => {
                self.take(4)?;
            }
            0xCF | 0xD3 | 0xCB => {
                self.take(8)?;
            }
            0xC4 | 0xD9 => {
                let n = self.byte()? as usize;
                self.take(n)?;
            }
            0xC5 | 0xDA => {
                let n = self.be_u16()? as usize;
                self.take(n)?;
            }
            0xC6 | 0xDB => {
                let n = self.be_u32()? as usize;
                self.take(n)?;
            }
            0xDC => {
                let n = self.be_u16()?;
                for _ in 0..n {
                    self.skip_value()?;
                }
            }
            0xDD => {
                let n = self.be_u32()?;
                for _ in 0..n {
                    self.skip_value()?;
                }
            }
            0xDE => {
                let n = self.be_u16()?;
                for _ in 0..(2 * u32::from(n)) {
                    self.skip_value()?;
                }
            }
            0xDF => {
                let n = self.be_u32()?;
                for _ in 0..n {
                    self.skip_value()?;
                    self.skip_value()?;
                }
            }
            0xD4..=0xD8 => {
                // fixext 1/2/4/8/16 + type byte
                let n = 1usize << (m - 0xD4);
                self.take(n + 1)?;
            }
            0xC7 => {
                let n = self.byte()? as usize;
                self.take(n + 1)?;
            }
            0xC8 => {
                let n = self.be_u16()? as usize;
                self.take(n + 1)?;
            }
            0xC9 => {
                let n = self.be_u32()? as usize;
                self.take(n + 1)?;
            }
            0xC1 => return Err(self.type_err("value", m)),
        }
        Ok(())
    }

    /// Error unless the whole input has been consumed.
    pub(crate) fn finish(&self) -> Result<(), DecodeError> {
        if self.pos != self.data.len() {
            return Err(DecodeError::TrailingBytes);
        }
        Ok(())
    }
}
