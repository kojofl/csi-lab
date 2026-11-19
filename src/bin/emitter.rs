//! Dedicated sounding transmitter (esp-csi README §4.3 "Get CSI from specific devices").
//!
//! Injects one broadcast HT frame every `1/EMITTER_HZ` s on `CHANNEL` (HT20/HT40
//! per `HT40`) at MCS0, with the driver assigning a fresh 802.11 sequence number
//! to each. Every `receiver` in range measures CSI from the *same* frame, so the
//! host can join all receivers on `(emitter MAC, sequence number)`.
//!
//! Captures nothing. Prints its MAC at boot (set it as `EMITTER_MAC` on the
//! receivers if you ever run two emitters on one channel).

#![no_std]
#![no_main]

extern crate alloc;

use csi_lab::{CHANNEL, EMITTER_HZ, controller_config, emitter_config, link_label};
use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_csi_rs::logging::logging::{LogMode, init_logger};
use esp_csi_rs::{CSINode, EMITTER_TX_FAIL, EMITTER_TX_OK, NodeHardware};
use portable_atomic::Ordering;
use esp_hal::clock::CpuClock;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::wifi::WifiController;
use static_cell::StaticCell;
use {esp_backtrace as _, esp_println as _};

esp_bootloader_esp_idf::esp_app_desc!();

static WIFI_CONTROLLER: StaticCell<WifiController<'static>> = StaticCell::new();

/// Every 2 s: frames the driver accepted for injection, and rejections.
/// `ok` should grow by 2 x EMITTER_HZ per line; any `fail` means the radio
/// refused the forced PHY / injection on this channel.
#[embassy_executor::task]
async fn heartbeat_task() {
    let mut prev = 0u32;
    loop {
        Timer::after_secs(2).await;
        let ok = EMITTER_TX_OK.load(Ordering::Relaxed);
        csi_lab::say!(
            "# tx ok={}/s total={} fail={}",
            ok.wrapping_sub(prev) / 2,
            ok,
            EMITTER_TX_FAIL.load(Ordering::Relaxed)
        );
        prev = ok;
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
    let controller = WIFI_CONTROLLER.init(controller);

    let (band, width) = link_label();
    csi_lab::say!(
        "# csi-lab emitter: ch {} ({} {}), {} Hz",
        CHANNEL,
        band,
        width,
        EMITTER_HZ
    );
    spawner.spawn(heartbeat_task().unwrap());

    // Logs "Emitter running: ... src xx:xx:..." once the radio is up.
    let mut node = CSINode::emitter(emitter_config(), NodeHardware::new(&mut interfaces, controller));
    node.run().await;

    loop {
        csi_lab::say!("# emitter stopped");
        Timer::after_secs(5).await;
    }
}
