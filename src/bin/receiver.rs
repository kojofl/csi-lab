//! CSI receiver for the 1 x emitter : N x receiver setup.
//!
//! Promiscuous sniffer on the emitter's channel/bandwidth. Locks onto the
//! emitter (auto via the frame marker, or `EMITTER_MAC`) and drops everything
//! else on the channel before any processing.
//!
//! Output (native USB port), esp-csi-rs `ArrayList` format, one line per frame:
//!   `[seq,rssi,rate,noise_floor,channel,local_timestamp,...,csi_len,[i8 data...],mac`
//! `seq` (802.11 sequence number, 12 bit, wraps at 4096) + `mac` identify the
//! emitted frame, so lines from different receivers join on `(mac, seq)`.
//! Every second also a `# rx ...` stats line; data lines start with `[`.

#![no_std]
#![no_main]

extern crate alloc;

use csi_lab::{CHANNEL, EMITTER_MAC, controller_config, link_label, receiver_config};
use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_csi_rs::config::CsiConfig;
use esp_csi_rs::csi::CSIDataPacket;
use esp_csi_rs::logging::logging::{LogMode, init_logger, log_csi};
use esp_csi_rs::{
    CSINode, NodeHardware, csi_peer_filter, csi_prelock_stats, set_csi_auto_lock, set_csi_callback,
    set_csi_peer_filter,
};
use esp_hal::clock::CpuClock;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::wifi::WifiController;
use portable_atomic::{AtomicI32, AtomicU32, Ordering};
use static_cell::StaticCell;
use {esp_backtrace as _, esp_println as _};

esp_bootloader_esp_idf::esp_app_desc!();

static WIFI_CONTROLLER: StaticCell<WifiController<'static>> = StaticCell::new();

static CSI_COUNT: AtomicU32 = AtomicU32::new(0);
static SUBCARRIERS: AtomicU32 = AtomicU32::new(0);
static LAST_RSSI: AtomicI32 = AtomicI32::new(0);
/// Frames the emitter sent that this receiver missed (from sequence gaps).
static LOST: AtomicU32 = AtomicU32::new(0);
/// Last sequence number seen; `u32::MAX` = none yet.
static LAST_SEQ: AtomicU32 = AtomicU32::new(u32::MAX);

/// HT-LTF only (HT20/HT40 PPDUs), no legacy L-LTF, no ACK dumps. Correct on the
/// C5 only thanks to the esp-radio patch.
fn csi_config() -> CsiConfig {
    CsiConfig {
        enable: 1,
        acquire_csi_legacy: 0,
        acquire_csi_force_lltf: false,
        acquire_csi_ht20: 1,
        acquire_csi_ht40: 1,
        acquire_csi_vht: false,
        val_scale_cfg: 0,
        dump_ack_en: 0,
        reserved: 0,
    }
}

/// Wi-Fi task context, only ever called for the locked emitter's frames.
fn on_csi(packet: &CSIDataPacket) {
    CSI_COUNT.fetch_add(1, Ordering::Relaxed);
    SUBCARRIERS.store(u32::from(packet.csi_data_len) / 2, Ordering::Relaxed);
    LAST_RSSI.store(packet.rssi, Ordering::Relaxed);

    let seq = u32::from(packet.sequence_number) & 0x0FFF;
    let last = LAST_SEQ.swap(seq, Ordering::Relaxed);
    if last != u32::MAX {
        let gap = seq.wrapping_sub(last) & 0x0FFF;
        // gap 0 = duplicate; > 1000 = emitter restarted, not a real loss.
        if (2..1000).contains(&gap) {
            LOST.fetch_add(gap - 1, Ordering::Relaxed);
        }
    }

    // A registered callback replaces esp-csi-rs's own CSI line output, so print it here.
    log_csi(packet.clone());
}

#[embassy_executor::task]
async fn stats_task() {
    let mut prev = 0u32;
    let mut prev_lost = 0u32;
    loop {
        Timer::after_secs(1).await;
        let total = CSI_COUNT.load(Ordering::Relaxed);
        let lost = LOST.load(Ordering::Relaxed);
        let got = total.wrapping_sub(prev);
        let missed = lost.wrapping_sub(prev_lost);
        let loss_pct = (100 * missed).checked_div(got + missed).unwrap_or(0);
        let m = csi_peer_filter();
        if m == [0; 6] {
            // `heard` = HT CSI reports from *any* transmitter so far. 0 = this
            // receiver decodes nothing on the channel; >0 but no lock = it hears
            // traffic, just not a csi-lab emitter frame.
            let (heard, last) = csi_prelock_stats();
            csi_lab::say!(
                "# rx searching for emitter on ch {}... heard={} last_from={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
                CHANNEL,
                heard,
                last[0],
                last[1],
                last[2],
                last[3],
                last[4],
                last[5]
            );
        } else {
            csi_lab::say!(
                "# rx rate={}/s loss={}% total={} subcarriers={} rssi={} emitter={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
                got,
                loss_pct,
                total,
                SUBCARRIERS.load(Ordering::Relaxed),
                LAST_RSSI.load(Ordering::Relaxed),
                m[0],
                m[1],
                m[2],
                m[3],
                m[4],
                m[5]
            );
        }
        prev = total;
        prev_lost = lost;
    }
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    init_logger(spawner, LogMode::ArrayList);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let (controller, mut interfaces) =
        esp_radio::wifi::new(peripherals.WIFI, controller_config()).expect("Wi-Fi init failed");
    let controller = WIFI_CONTROLLER.init(controller);

    // Source filter: fixed MAC, or learn it from the first marked emitter frame.
    match EMITTER_MAC {
        Some(mac) => set_csi_peer_filter(mac),
        None => set_csi_auto_lock(true),
    }

    let mut node = CSINode::sniffer(
        receiver_config(),
        Some(csi_config()),
        NodeHardware::new(&mut interfaces, controller),
    );

    set_csi_callback(on_csi);
    spawner.spawn(stats_task().unwrap());

    let (band, width) = link_label();
    csi_lab::say!("# csi-lab receiver: ch {} ({} {})", CHANNEL, band, width);
    node.run().await;

    loop {
        csi_lab::say!("# receiver stopped");
        Timer::after_secs(5).await;
    }
}
