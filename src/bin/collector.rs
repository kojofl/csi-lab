//! RX node (replaces esp-csi's `csi_recv`).
//!
//! esp-csi-rs ESP-NOW simplex **peer**: beacons ~1 Hz until a source finds it,
//! then goes receive-only and reports CSI for every frame from that source.
//!
//! Output on the USB-Serial-JTAG port:
//!   * one `CSI_DATA,...` line per frame in the ESP32-CSI-Tool CSV layout
//!     (header printed once at start; columns in the esp-csi-rs docs)
//!   * a `# stats ...` line every second (rate, subcarriers, RSSI) so you can
//!     see at a glance that the link is healthy. Parsers that look for lines
//!     starting with `CSI_DATA` skip it.

#![no_std]
#![no_main]

extern crate alloc;

use csi_lab::{CHANNEL, collector_config, controller_config, link_label};
use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_csi_rs::config::CsiConfig;
use esp_csi_rs::csi::CSIDataPacket;
use esp_csi_rs::logging::logging::{LogMode, init_logger};
use esp_csi_rs::{CSINode, NodeHardware, install_static_espnow_recv, set_csi_callback};
use esp_hal::clock::CpuClock;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::wifi::{Protocol, WifiController};
use portable_atomic::{AtomicI32, AtomicU32, Ordering};
use static_cell::StaticCell;
use {esp_backtrace as _, esp_println as _};

esp_bootloader_esp_idf::esp_app_desc!();

static WIFI_CONTROLLER: StaticCell<WifiController<'static>> = StaticCell::new();

static CSI_COUNT: AtomicU32 = AtomicU32::new(0);
static SUBCARRIERS: AtomicU32 = AtomicU32::new(0);
static LAST_RSSI: AtomicI32 = AtomicI32::new(0);

/// Same acquisition as `csi_recv`'s C5 branch: HT-LTF for HT20/HT40 PPDUs,
/// no legacy L-LTF, no ACK dumps. Only takes effect correctly because of the
/// esp-radio patch in `patches/esp-radio` (see README).
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

/// Runs in the Wi-Fi task for every CSI packet: keep it cheap.
fn on_csi(packet: &CSIDataPacket) {
    CSI_COUNT.fetch_add(1, Ordering::Relaxed);
    // Two i8 (imag, real) per subcarrier.
    SUBCARRIERS.store(u32::from(packet.csi_data_len) / 2, Ordering::Relaxed);
    LAST_RSSI.store(packet.rssi, Ordering::Relaxed);
}

#[embassy_executor::task]
async fn stats_task() {
    let mut previous = 0u32;
    loop {
        Timer::after_secs(1).await;
        let total = CSI_COUNT.load(Ordering::Relaxed);
        csi_lab::say!(
            "# stats rate={}/s total={} subcarriers={} rssi={} (HT40 >= ~114, HT20 ~56-64)",
            total.wrapping_sub(previous),
            total,
            SUBCARRIERS.load(Ordering::Relaxed),
            LAST_RSSI.load(Ordering::Relaxed),
        );
        previous = total;
    }
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    init_logger(spawner, LogMode::EspCsiTool);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let (controller, mut interfaces) =
        esp_radio::wifi::new(peripherals.WIFI, controller_config()).expect("Wi-Fi init failed");
    install_static_espnow_recv();
    let controller = WIFI_CONTROLLER.init(controller);

    let mut node = CSINode::esp_now_simplex(
        collector_config(),
        Some(csi_config()),
        None,
        NodeHardware::new(&mut interfaces, controller),
    );
    node.set_protocol(Protocol::N);

    set_csi_callback(on_csi);
    spawner.spawn(stats_task().unwrap());

    let (band, width) = link_label();
    csi_lab::say!(
        "# csi-lab collector: ch {} ({} {}), waiting for a source",
        CHANNEL,
        band,
        width
    );
    node.run().await;

    loop {
        csi_lab::say!("# collector stopped");
        Timer::after_secs(5).await;
    }
}
