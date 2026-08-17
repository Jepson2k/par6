//! The bus-ownership signal the vendor's CAN tools read.
//!
//! `can0` is a system-wide exclusive resource, and the vendor's firmware
//! flasher decides whether it may transmit by looking at two POSIX
//! shared-memory segments the runtime publishes:
//!
//! | segment | contents | meaning |
//! |---|---|---|
//! | `loop_tick` | one little-endian `f64` | advancing = a runtime is live and owns the bus |
//! | `robot_mode` | 4-byte LE length + UTF-8 | `FLASHING` = the bus is granted; anything else = keep off |
//!
//! Liveness is read FIRST by those tools, because the segments outlive
//! the process that wrote them: a stale `robot_mode` left behind by a
//! stopped runtime would otherwise read as a live grant.
//!
//! **A runtime that publishes neither is read as "no runtime, bus is
//! free"** — which is why par6d publishes them whether or not it has
//! anything to grant. Without this a flasher run against a live par6d
//! takes the recovery path and transmits into its traffic, which is the
//! two-transmitter corruption the whole arrangement exists to prevent.
//!
//! The tick value is the RT core's own tick counter, not a wall clock:
//! it advances only when the tick loop does, so an RT thread that has
//! stopped reads as stopped even though this writer is still running.

use std::fs::{File, OpenOptions};
use std::io::Result as IoResult;
use std::os::unix::fs::{FileExt, OpenOptionsExt};
use std::path::{Path, PathBuf};

use par6_rt::Mode;

/// Where the segments live. POSIX shared memory IS this directory on
/// Linux, so a plain file here is what `shared_memory.SharedMemory(name=…)`
/// opens.
const SHM_DIR: &str = "/dev/shm";

/// Override for [`SHM_DIR`].
///
/// The segments name a claim on ONE `can0`, so the default is the one
/// place every tool looks. A second runtime on the same box — a test
/// rig, a bench instance beside the real one — has to be pointed
/// somewhere else or it overwrites the claim of the runtime that
/// actually owns the bus.
pub(crate) const SHM_DIR_ENV: &str = "PAR6_SHM_DIR";

/// The directory the segments are published in.
pub(crate) fn shm_dir() -> PathBuf {
    std::env::var_os(SHM_DIR_ENV)
        .filter(|v| !v.is_empty())
        .map_or_else(|| PathBuf::from(SHM_DIR), PathBuf::from)
}

/// Fixed segment size for the mode string: a 4-byte length prefix plus
/// room for any mode name, zero-padded, matching the vendor's own
/// fixed-size string segments.
const MODE_SEGMENT_LEN: usize = 64;

/// World-readable: the tools that read these run as whoever the operator
/// is (often root under `sudo`), never as the `par6` service user.
const SEGMENT_MODE: u32 = 0o644;

/// Publishes the two segments. Dropping it removes them, so a stopped
/// par6d stops claiming the bus.
pub(crate) struct BusGrant {
    tick_path: PathBuf,
    mode_path: PathBuf,
    tick: File,
    mode: File,
    /// The last mode written, so an unchanged mode costs no write.
    last: Option<Mode>,
}

impl BusGrant {
    /// Create both segments under `dir`.
    ///
    /// Failure is logged by the caller and is not fatal: the signal is
    /// how OTHER tools stay off the bus, so a runtime that cannot
    /// publish it still drives the arm correctly — it just cannot be
    /// seen by a flasher, which is the state every par6d shipped in
    /// before this existed.
    pub(crate) fn create(dir: &Path) -> IoResult<Self> {
        let open = |name: &str, len: usize| -> IoResult<(PathBuf, File)> {
            let path = dir.join(name);
            let f = OpenOptions::new()
                .write(true)
                .create(true)
                .truncate(true)
                .mode(SEGMENT_MODE)
                .open(&path)?;
            f.set_len(len as u64)?;
            Ok((path, f))
        };
        let (tick_path, tick) = open("loop_tick", 8)?;
        let (mode_path, mode) = open("robot_mode", MODE_SEGMENT_LEN)?;
        Ok(Self {
            tick_path,
            mode_path,
            tick,
            mode,
            last: None,
        })
    }

    /// Publish one RT tick and the mode it was taken at.
    pub(crate) fn publish(&mut self, tick: u64, mode: Mode) -> IoResult<()> {
        // `f64` because that is what the vendor tools unpack; the cast
        // is exact for any tick count a control box will reach (2^53
        // ticks is 1.1 million years at 250 Hz).
        self.tick.write_all_at(&(tick as f64).to_le_bytes(), 0)?;
        if self.last != Some(mode) {
            let name = mode_name(mode).as_bytes();
            let mut buf = [0u8; MODE_SEGMENT_LEN];
            buf[..4].copy_from_slice(&(name.len() as u32).to_le_bytes());
            buf[4..4 + name.len()].copy_from_slice(name);
            self.mode.write_all_at(&buf, 0)?;
            self.last = Some(mode);
        }
        Ok(())
    }
}

impl Drop for BusGrant {
    fn drop(&mut self) {
        // A segment left behind reads as a live runtime for exactly as
        // long as its stale tick takes to be sampled twice, and the
        // tools check liveness before the mode precisely because that
        // window exists. Removing them closes it immediately.
        for path in [&self.tick_path, &self.mode_path] {
            if let Err(e) = std::fs::remove_file(path) {
                log::warn!("could not remove {}: {e}", path.display());
            }
        }
    }
}

/// The mode name as the vendor's tools spell it.
///
/// `FLASHING` is the one value with a meaning to them — it is the grant
/// — so it has to match exactly. The rest are named to match the vendor
/// mode they correspond to, because a refusal quotes the mode back at
/// the operator and "RTI is running in EXEC mode" has to name something
/// they can find on the screen in front of them.
fn mode_name(mode: Mode) -> &'static str {
    match mode {
        Mode::Booting => "BOOTING",
        Mode::Idle => "IDLE",
        Mode::ActiveError => "ACTIVE_ERROR",
        Mode::Homing => "HOMING",
        Mode::Jog => "JOG",
        // The vendor calls its streaming mode RTI, after the runtime
        // process that owns it.
        Mode::Stream => "RTI",
        Mode::Exec => "EXEC",
        Mode::HandGuiding => "HAND_GUIDING",
        Mode::Impedance => "IMPEDANCE",
        Mode::SafetyStop => "SAFETY_STOP",
        Mode::Flashing => "FLASHING",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Round-trip both segments the way the vendor tools read them.
    ///
    /// Their reader is three lines against a byte layout, so this
    /// re-implements those three lines rather than calling ours: a
    /// length prefix we write and read back with the same helper would
    /// agree with itself in any encoding, including one nothing else can
    /// parse.
    #[test]
    fn the_segments_read_back_the_way_the_vendor_tools_read_them() {
        let dir = std::env::temp_dir().join(format!("par6-grant-{}", std::process::id()));
        std::fs::create_dir_all(&dir).expect("scratch dir");
        let mut grant = BusGrant::create(&dir).expect("segments");

        let read_tick = || -> f64 {
            let raw = std::fs::read(dir.join("loop_tick")).expect("loop_tick");
            f64::from_le_bytes(raw[..8].try_into().expect("8 bytes"))
        };
        let read_mode = || -> String {
            let raw = std::fs::read(dir.join("robot_mode")).expect("robot_mode");
            let len = u32::from_le_bytes(raw[..4].try_into().expect("4 bytes")) as usize;
            assert!(
                len > 0 && len <= raw.len() - 4,
                "length {len} is unreadable"
            );
            String::from_utf8(raw[4..4 + len].to_vec()).expect("utf-8")
        };

        grant.publish(41, Mode::Exec).expect("publish");
        assert_eq!(read_tick(), 41.0);
        assert_eq!(read_mode(), "EXEC", "a mode with no grant in it");

        // Liveness is "advancing", which is what the tools sample twice.
        grant.publish(42, Mode::Exec).expect("publish");
        assert_eq!(read_tick(), 42.0);

        // The one value that grants the bus, spelled exactly.
        grant.publish(43, Mode::Flashing).expect("publish");
        assert_eq!(read_mode(), "FLASHING");

        // A shorter name must not leave the longer one's tail behind:
        // the length prefix would still say FLASHING is over, but a
        // reader that trusted the bytes would see "IDLEING".
        grant.publish(44, Mode::Idle).expect("publish");
        assert_eq!(read_mode(), "IDLE");

        drop(grant);
        assert!(
            !dir.join("loop_tick").exists() && !dir.join("robot_mode").exists(),
            "a stopped runtime must not leave a claim on the bus behind"
        );
        std::fs::remove_dir_all(&dir).expect("clean up");
    }

    /// The default is the one directory every tool that reads these
    /// looks in; the override exists for a second runtime, not as a
    /// place the shipped one might quietly end up.
    #[test]
    fn the_segments_default_to_the_directory_the_tools_read() {
        std::env::remove_var(SHM_DIR_ENV);
        assert_eq!(shm_dir(), Path::new("/dev/shm"));
        std::env::set_var(SHM_DIR_ENV, "");
        assert_eq!(shm_dir(), Path::new("/dev/shm"), "empty is not a directory");
        std::env::set_var(SHM_DIR_ENV, "/tmp/par6-elsewhere");
        assert_eq!(shm_dir(), Path::new("/tmp/par6-elsewhere"));
        std::env::remove_var(SHM_DIR_ENV);
    }
}
