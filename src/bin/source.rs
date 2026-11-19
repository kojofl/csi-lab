//! TX node (replaces esp-csi's `csi_send`).
//!
//! esp-csi-rs ESP-NOW simplex **source**: waits for a collector's discovery
//! beacon, registers it as a unicast peer with a forced PHY, then sends
//! `TRAFFIC_HZ` frames per second to it. Captures no CSI itself.
//!
//! One source can feed one collector in simplex mode. For several receivers
//! measuring the same transmitter, use the emitter + sniffer pairing instead.

#![no_std]
#![no_main]

extern crate alloc;

use csi_lab::{CHANNEL, PHY_RATE, TRAFFIC_HZ, controller_config, link_label, source_config};
use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_csi_rs::logging::logging::{LogMode, init_logger};
use esp_csi_rs::{
    CSINode, NodeHardware, SOURCE_PEER_REBUILDS, SOURCE_TX_FAILED, SOURCE_TX_OK,
    SOURCE_TX_TIMEOUT, install_static_espnow_recv,
};
use portable_atomic::Ordering;
use esp_hal::clock::CpuClock;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::wifi::{Protocol, WifiController};
use static_cell::StaticCell;
use {esp_backtrace as _, esp_println as _};

esp_bootloader_esp_idf::esp_app_desc!();

static WIFI_CONTROLLER: StaticCell<WifiController<'static>> = StaticCell::new();

/// Once a second: frames ACKed by the collector, plus anything that went wrong.
/// `ok` should sit near TRAFFIC_HZ. Rising `timeout` = driver lost send-complete
/// callbacks (the patched loop recovers); rising `failed` = no ACK from the collector.
#[embassy_executor::task]
async fn stats_task() {
    let mut prev_ok = 0u32;
    loop {
        Timer::after_secs(1).await;
        let ok = SOURCE_TX_OK.load(Ordering::Relaxed);
        csi_lab::say!(
            "# tx ok={}/s total={} failed={} timeout={} peer_rebuilds={}",
            ok.wrapping_sub(prev_ok),
            ok,
            SOURCE_TX_FAILED.load(Ordering::Relaxed),
            SOURCE_TX_TIMEOUT.load(Ordering::Relaxed),
            SOURCE_PEER_REBUILDS.load(Ordering::Relaxed),
        );
        prev_ok = ok;
    }
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    init_logger(spawner, LogMode::Text);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let (controller, mut interfaces) =
        esp_radio::wifi::new(peripherals.WIFI, controller_config()).expect("Wi-Fi init failed");
    // Swap esp-radio's heap-backed ESP-NOW RX queue for the crate's static pool
    // before any frames arrive (avoids C5 interrupt-watchdog trips at boot).
    install_static_espnow_recv();
    let controller = WIFI_CONTROLLER.init(controller);

    let mut node = CSINode::esp_now_simplex(
        source_config(),
        None,
        TRAFFIC_HZ,
        NodeHardware::new(&mut interfaces, controller),
    );
    node.set_protocol(Protocol::N);
    spawner.spawn(stats_task().unwrap());

    let (band, width) = link_label();
    csi_lab::say!(
        "csi-lab source: ch {} ({} {}), {:?}, {:?} Hz, waiting for a collector",
        CHANNEL,
        band,
        width,
        PHY_RATE,
        TRAFFIC_HZ
    );
    node.run().await;

    loop {
        csi_lab::say!("source stopped");
        Timer::after_secs(5).await;
    }
}
