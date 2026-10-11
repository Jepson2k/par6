//! The post-homing reference check against the real gravity model, end
//! to end on the torque-level sim plant: a sequence whose seeks all
//! latched is still refused when the pose they imply cannot be holding
//! the load the drives measure.
//!
//! The wrong reference is produced the way a false stall produces one —
//! by telling the runtime the endstop sits somewhere else (J1's
//! `home_offset_rad` shifted by half a radian) — so every seek, backoff
//! and positioning move runs exactly as in the passing case and only the
//! final hold differs.

use std::path::PathBuf;
use std::time::Duration;

use par6_client::{Client, ClientConfig, StatusTransport};
use par6_proto::{ErrorCode, HomingJointState, HomingPhase, Status};

mod common;
use common::{boot_for_client, free_udp_port, retimed_config};

/// A full sequence on the retimed sim takes ~45 s of wall clock.
const BUDGET: Duration = Duration::from_secs(120);
const J1_OFFSET: &str = "home_offset_rad = -2.5259796";

fn config(tag: &str, j1_offset: Option<&str>) -> PathBuf {
    let path = retimed_config(tag, 0.02);
    if let Some(shifted) = j1_offset {
        let text = std::fs::read_to_string(&path).expect("config");
        assert_eq!(text.matches(J1_OFFSET).count(), 1, "J1's offset is unique");
        std::fs::write(&path, text.replace(J1_OFFSET, shifted)).expect("patched config");
    }
    path
}

/// Home through the real client; the STATUS frame after the sequence ends.
fn home(config: PathBuf) -> Status {
    let status_port = free_udp_port();
    let daemon = boot_for_client(config, status_port).expect("daemon boots");
    let cmd = daemon.command_addr();
    let rt = tokio::runtime::Runtime::new().expect("runtime");
    rt.block_on(async {
        let client = Client::connect(ClientConfig {
            host: cmd.ip().to_string(),
            port: cmd.port(),
            status: StatusTransport::Unicast {
                host: "127.0.0.1".parse().unwrap(),
            },
            status_port,
            ..ClientConfig::default()
        })
        .await
        .expect("client connects");
        assert!(
            client.wait_status(|s| s.link_ok == 1, BUDGET).await,
            "the sim bus never came up"
        );
        client.reset().await.expect("reset");
        assert!(
            client.wait_status(|s| s.enabled, BUDGET).await,
            "reset enables"
        );
        client.home(false).await.expect("home accepted");
        assert!(
            client.wait_status(|s| s.homing.active, BUDGET).await,
            "homing starts"
        );
        assert!(
            client.wait_status(|s| !s.homing.active, BUDGET).await,
            "homing ends"
        );
        // The HOMING_FAILED warning is derived from the statuses on the
        // tick after the sequence ends; take a frame after that.
        tokio::time::sleep(Duration::from_millis(200)).await;
        let mut last = None;
        assert!(
            client
                .wait_status(
                    |s| {
                        last = Some(s.clone());
                        true
                    },
                    BUDGET
                )
                .await
        );
        last.expect("a status frame")
    })
}

fn homing_failed_for(s: &Status, joint: usize) -> bool {
    s.warnings.iter().any(|w| {
        w.code == ErrorCode::MotnHomingFailed as u16
            && w.cause.starts_with(&format!("Joint {joint}'"))
    })
}

#[test]
fn a_reference_the_real_gravity_model_contradicts_is_refused() {
    let nominal = home(config("homing-reference-nominal", None));
    assert!(nominal.homed, "the shipped sequence homes on the sim plant");
    assert!(
        nominal.warnings.is_empty() && nominal.error.is_none(),
        "clean home: {:?} {:?}",
        nominal.warnings,
        nominal.error
    );
    // Every seek latches as before; only the declared endstop moved, so
    // the ready pose the runtime drives to is half a radian from where
    // the model expects the shoulder load it then measures.
    let shifted = home(config(
        "homing-reference-shifted",
        Some("home_offset_rad = -2.0259796"),
    ));
    assert!(
        !shifted.homed,
        "an implausible reference must not count as homed"
    );
    assert_eq!(
        shifted.homing.joints[1],
        (HomingJointState::Failed as u8, HomingPhase::Finished as u8),
        "J1 is refused in its Finished phase, not in a seek: {:?}",
        shifted.homing.joints
    );
    assert!(
        homing_failed_for(&shifted, 1),
        "HOMING_FAILED names J1: {:?}",
        shifted.warnings
    );
    for (j, st) in shifted.homing.joints.iter().enumerate().take(6) {
        if j != 1 {
            assert_eq!(st.0, HomingJointState::Done as u8, "J{j} latched normally");
        }
    }
}
