//! Settings shared by the `source` and `collector` firmwares. Both boards must
//! agree on these, so they live in one place.

#![no_std]

use esp_csi_rs::{EmitterConfig, EspNowConfig, HtBandwidth, SimplexConfig, WifiSnifferConfig};
use esp_radio::esp_now::WifiPhyRate;
use esp_radio::wifi::{ControllerConfig, SecondaryChannel};

/// Primary channel. `>= 36` puts the C5 on 5 GHz.
///
/// 5 GHz in the EU (DE): use 36, 40, 44 or 48. Those are the non-DFS indoor
/// channels; 52+ need radar detection, which ESP-NOW does not do.
/// 2.4 GHz: 1..=13 (esp-csi's `csi_send`/`csi_recv` used 11).
pub const CHANNEL: u8 = 36;

/// `Some(..)` = HT40, `None` = HT20.
///
/// * 5 GHz: the driver derives the secondary from the primary (36/44 pair
///   upward, 40/48 downward); any `Some` works, `Above` matches 36.
/// * 2.4 GHz: `Above` for channels 1..=7, `Below` for 5..=13 (the 40 MHz block
///   must fit in the band, otherwise the radio silently falls back to HT20).
pub const HT40: Option<SecondaryChannel> = Some(SecondaryChannel::Above);

/// TX PHY rate forced on the source's unicast frames. MCS0 decodes at the
/// lowest SNR, which matters once a person walks through the link; the CSI
/// itself comes from the HT-LTF and does not depend on the MCS.
pub const PHY_RATE: WifiPhyRate = WifiPhyRate::RateMcs0Lgi;

/// Frames per second the source sends (≈ CSI samples/s at the collector).
/// `None` floods at whatever the link sustains.
pub const TRAFFIC_HZ: Option<u16> = Some(100);

/// Emitter sounding rate (frames/s) for the 1 x emitter : N x receiver setup.
/// Every receiver gets one CSI sample per emitted frame it decodes.
///
/// Default 100. Override at build time without editing this file:
/// `CSI_EMITTER_HZ=250 cargo emitter`. Only the emitter uses it, so a rate change
/// means reflashing the emitter alone; receivers follow whatever rate arrives.
/// Checked at compile time to be an integer in `1..=EMITTER_HZ_MAX`.
pub const EMITTER_HZ: u64 = match option_env!("CSI_EMITTER_HZ") {
    Some(s) => parse_hz(s),
    None => 100,
};

/// Upper bound for [`EMITTER_HZ`]: 500 µs period. Well past what a receiver can
/// print over USB as text (~1 KB per line), so it only bounds typos, not tests.
pub const EMITTER_HZ_MAX: u64 = 2000;

const _: () = assert!(
    EMITTER_HZ >= 1 && EMITTER_HZ <= EMITTER_HZ_MAX,
    "CSI_EMITTER_HZ must be in 1..=2000"
);

/// Compile-time decimal parse for `CSI_EMITTER_HZ` (no `str::parse` in const fn).
const fn parse_hz(s: &str) -> u64 {
    let b = s.as_bytes();
    assert!(!b.is_empty(), "CSI_EMITTER_HZ is empty");
    let mut v: u64 = 0;
    let mut i = 0;
    while i < b.len() {
        assert!(
            b[i].is_ascii_digit(),
            "CSI_EMITTER_HZ must be a plain integer (frames/s)"
        );
        v = v * 10 + (b[i] - b'0') as u64;
        assert!(v <= EMITTER_HZ_MAX, "CSI_EMITTER_HZ must be in 1..=2000");
        i += 1;
    }
    v
}

/// Receivers: `None` = lock onto the first csi-lab emitter heard (its frames
/// carry a marker). `Some(mac)` = only accept that MAC (printed by the emitter
/// at boot); use this if two emitters ever share the channel.
pub const EMITTER_MAC: Option<[u8; 6]> = None;

/// `source` only: `Some(mac)` = skip discovery and send straight to this MAC
/// (manual pairing). The target ACKs the frames at MAC level even if it runs
/// `receiver` instead of `collector`, so a promiscuous receiver can measure the
/// source's forced-PHY ESP-NOW frames. `None` = normal auto-pairing.
pub const SOURCE_PEER_MAC: Option<[u8; 6]> = None;

/// Regulatory domain. Decides which 5 GHz channels and TX powers are legal.
pub const COUNTRY: [u8; 2] = *b"DE";

/// Wi-Fi controller config shared by both firmwares.
pub fn controller_config() -> ControllerConfig {
    ControllerConfig::default().with_country_info(COUNTRY)
}

/// The source (flooding) end.
pub fn source_config() -> SimplexConfig {
    let mut cfg = EspNowConfig::default()
        .with_channel(CHANNEL)
        .with_phy_rate(PHY_RATE);
    if let Some(sec) = HT40 {
        cfg = cfg.with_ht40(sec);
    }
    if let Some(mac) = SOURCE_PEER_MAC {
        cfg = cfg.with_peer_mac(mac);
    }
    SimplexConfig::source(cfg)
}

/// The peer (collecting) end.
pub fn collector_config() -> SimplexConfig {
    let cfg = SimplexConfig::peer(CHANNEL);
    match HT40 {
        Some(sec) => cfg.with_ht40(sec),
        None => cfg,
    }
}

/// The emitter (dedicated sounding transmitter).
pub fn emitter_config() -> EmitterConfig {
    let bw = match HT40 {
        None => HtBandwidth::Ht20,
        Some(SecondaryChannel::Below) => HtBandwidth::Ht40Below,
        Some(_) => HtBandwidth::Ht40Above,
    };
    EmitterConfig::new(CHANNEL, bw)
        .with_period(embassy_time::Duration::from_micros(1_000_000 / EMITTER_HZ))
}

/// A receiver (promiscuous sniffer on the emitter's channel/bandwidth).
pub fn receiver_config() -> WifiSnifferConfig {
    let cfg = WifiSnifferConfig::default().with_channel(CHANNEL);
    match HT40 {
        Some(sec) => cfg.with_ht40(sec),
        None => cfg,
    }
}

/// Human-readable link description for the boot log.
pub fn link_label() -> (&'static str, &'static str) {
    (
        if CHANNEL >= 36 { "5 GHz" } else { "2.4 GHz" },
        if HT40.is_some() { "HT40" } else { "HT20" },
    )
}

/// Log a line through whichever path esp-csi-rs's logger is using, so status
/// lines never interleave mid-line with the CSI stream. (`esp_csi_rs::log_ln!`
/// checks *the calling crate's* cargo features and expands to nothing here.)
#[macro_export]
macro_rules! say {
    ($($arg:tt)*) => {{
        if esp_csi_rs::logging::logging::is_async_logging_active() {
            log::info!($($arg)*);
        } else {
            esp_println::println!($($arg)*);
        }
    }};
}
