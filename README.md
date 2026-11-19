# csi-lab (ESP32-C5, esp-csi-rs)

Wi-Fi CSI lab firmware on [esp-csi-rs](https://github.com/csi-rs/esp-csi-rs) 0.11
(patched, see below). Main setup is esp-csi's "get CSI from specific devices":
**one dedicated emitter, any number of receivers**, all measuring the *same*
broadcast frames.

| Binary | Role |
|---|---|
| `emitter` | 1 board. Sends a broadcast HT frame (ESP-NOW, forced HT40/HT20 MCS0) every 1/`EMITTER_HZ` s (fixed-rate ticker), driver-assigned sequence numbers |
| `receiver` | N boards. Promiscuous on the emitter's channel/bandwidth, locks onto the emitter, prints CSI per frame |
| `source` / `collector` | 1:1 ESP-NOW pair, for quick two-board link tests |

## Run

```sh
rustup target add riscv32imac-unknown-none-elf
cargo install espflash --locked
ls -l /dev/serial/by-id/          # one Espressif USB JTAG entry per board

ESPFLASH_PORT=/dev/serial/by-id/<emitter>  cargo emitter
ESPFLASH_PORT=/dev/serial/by-id/<rx1>      cargo receiver   # repeat per receiver
```

Flash each receiver once; after that they just need USB power + a reader on
the port (they re-lock onto the emitter after any reboot). Use the **native USB
port** on every board.

Settings in `src/lib.rs` (all boards must run the same build, except `EMITTER_HZ`,
which only the emitter uses):

| Const | Default | Notes |
|---|---|---|
| `CHANNEL` | `36` | `>= 36` = 5 GHz. In DE/EU use 36/40/44/48 (non-DFS). 2.4 GHz: 1–13 |
| `HT40` | `Some(Above)` | `None` = HT20. 2.4 GHz: `Above` for ch 1–7, `Below` for 5–13 |
| `EMITTER_HZ` | `100` | sounding rate = CSI samples/s per receiver. Override per build: `CSI_EMITTER_HZ=250 cargo emitter` (1–2000) |
| `EMITTER_MAC` | `None` | `None` = auto-lock on the emitter's frame marker; `Some(mac)` to pin one |
| `COUNTRY` | `DE` | regulatory domain (esp-radio defaults to `CN`) |
| `PHY_RATE`, `TRAFFIC_HZ` | | only for the `source`/`collector` pair |

### Changing the sounding rate

Only the emitter needs reflashing; receivers keep running:

```sh
ESPFLASH_PORT=/dev/serial/by-id/<emitter> CSI_EMITTER_HZ=200 cargo emitter
```

The emitter confirms with `Emitter running: ... 5000 us period` and its `# tx ok=…/s`
line; each receiver's `# rx rate=…/s loss=…%` shows what actually arrives. Plain
`cargo emitter` goes back to 100 Hz (cargo rebuilds when the variable changes).
`loss` is counted from sequence gaps in the radio callback, before printing, so it
measures the air link. At high rates the USB text output (~1 KB/line) can become the
bottleneck instead: compare the receiver's `rate=` with the data lines the host
actually got.

## Receiver output

esp-csi-rs `ArrayList` format, one line per emitter frame:

```
[seq,rssi,rate,noise_floor,channel,local_timestamp,...,csi_len,[i8 imag/real pairs...],mac
```

`seq` is the 802.11 sequence number the emitter's driver assigned (12 bit,
wraps at 4096) and `mac` the emitter. **All receivers report the same `seq` for
the same frame**, so join the N streams on `(mac, seq)` (handle the wrap with
your host clock) instead of trusting N independent local timestamps.

Once a second each receiver also prints

```
# rx rate=99/s loss=1% total=… subcarriers=117 rssi=-52 emitter=10:bd:a3:…
```

`subcarriers >= ~114` = HT40, ~56–64 = HT20. `loss` is computed from sequence
gaps. Until the emitter is heard it prints `# rx searching for emitter...`.
Lines starting with `#` are status; data lines start with `[`.

Capture (one per receiver):

```sh
P=/dev/serial/by-id/<rx1>; stty -F $P raw -echo
cat $P | tee >(grep '^#' >&2) | grep '^\[' > rx1_$(date +%F_%H%M).txt
```

## The esp-radio patch (`patches/esp-radio`)

esp-radio 0.18.0 (which esp-csi-rs uses) builds the C5's
`wifi_csi_acquire_config_t` by passing fields positionally, but skips
`acquire_csi_force_lltf` and `acquire_csi_vht`. Every later field lands one or
two bits off. With esp-csi-rs's own C5 defaults that means **HT40 acquisition
off and force-L-LTF on**: you only ever see legacy CSI. The same bug is still in
esp-radio 1.0.0-beta.1.

`patches/esp-radio` is an unmodified copy of 0.18.0 except for that one block
(search for `PATCHED`), wired in through `[patch.crates-io]`. Remove both once
upstream is fixed. Worth filing on esp-rs/esp-hal and mentioning to the
esp-csi-rs maintainers, since their C5 numbers are affected.

## The esp-csi-rs patch (`patches/esp-csi-rs`)

Copy of esp-csi-rs 0.11.0; changes are marked `PATCHED`:

- **`node.rs`, run start:** for both ESP-NOW modes the node now selects the band
  from the channel (C5), enables 11a/11n on 5 GHz, and for HT40 sets the
  secondary channel *and* the 40 MHz interface bandwidth, all with ESP-NOW RX
  suspended and the C5 settle delay. Upstream only forced the source's per-peer
  TX PHY to HT40, so the peer stayed on a 20 MHz RX path and could not decode
  the frames, and the ESP-NOW modes never left 2.4 GHz.
- **`SimplexConfig::with_ht40`** so the peer end can be told about HT40.
- **`CSINode::esp_now_simplex(SimplexConfig, ..)`** constructor taking either end.
- **Source send timeout** (`peripheral/esp_now_fast.rs`): each send waits at most
  50 ms for the driver's completion callback (upstream waited forever, so one
  lost callback froze the source). 10 timeouts in a row rebuild the unicast peer.
  Counters are exported as `SOURCE_TX_OK/FAILED/TIMEOUT/PEER_REBUILDS` and
  printed by `source` as `# tx ...` once a second.
- **Collector re-pairing** (`central/esp_now_fast.rs`): no frame for 3 s puts the
  collector back into beaconing (upstream stayed RX-only forever), and it also
  re-latches onto a source that is already flooding it. Either board can now be
  reset on its own and the pair recovers.

Upstream notes ESP-NOW on 5 GHz as untested, so this is new ground: verify with
the `# stats` line on first run. Worth upstreaming together with the
esp-radio fix.

Emitter / receiver changes (also `PATCHED`):

- **Sniffer HT40 + 5 GHz** (`node.rs`): `WifiSnifferConfig::with_ht40` sets the
  secondary channel and the 40 MHz RX path, and 11a/n on 5 GHz. Upstream locked
  the sniffer to 20 MHz, so an HT40 emitter produced no HT CSI.
- **Emitter 40 MHz on 5 GHz** (`emitter/phy.rs`): upstream only widened 2.4 GHz.
- **C5 emitter sends over ESP-NOW** (`emitter/mod.rs`, `emitter/espnow.rs`): raw injection
  (`esp_wifi_80211_tx`) on the C5 always goes out as a 20 MHz PPDU, even with the interface at
  40 MHz and `esp_wifi_config_80211_tx(HT40)` returning OK, so receivers only got 57
  subcarriers. The C5 now uses the ESP-NOW broadcast transport (as esp-csi's `csi_send` does)
  with a per-peer HT40 MCS0 rate, and the same radio sequence as the ESP-NOW source
  (`bringup_c5`). Pitfall: `EspNow::set_channel` drops the HT40 secondary, after which the
  per-peer HT40 rate is rejected with `ESP_ERR_ESPNOW_ARG` and frames go out legacy.
- **Emitter fixed-rate ticker** (`emitter/mod.rs`): upstream slept a period
  *after* each injection, so the real rate sat below the setting and jittered.
- **Emitter marker + auto-lock** (`emitter/frame.rs`, `csi/delivery.rs`): frame
  body is `CSILAB01`; `set_csi_auto_lock(true)` installs the first marked
  frame's MAC as the existing peer filter, everything else on the channel is
  dropped before the packet copy.

## Known limits

- Emitter/receiver verified on two C5s on ch 36 HT40: 100 Hz, 117 subcarriers,
  ~0% loss. HT20 and 2.4 GHz with the ESP-NOW emitter are not re-tested yet.
- **No AGC/FFT gain compensation.**
