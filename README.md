<div align="center">

[![Build Status](https://github.com/hydra/vtx-tool/workflows/Rust/badge.svg)](https://github.com/hydra/vtx-tool/actions/workflows/rust.yml)

</div>

# VTX-Tool

RF PA calibration and VTX table tool for OSD/VTX firmware.

## What it is

`vtx-tool` is a desktop application for calibrating the RF power amplifier (PA) of a VTX (video
transmitter) against a reference RF power meter, and for building and maintaining the VTX's
power/frequency calibration table.

It connects to a VTX over a serial link (using MSP) and to an external RF power meter over a
second serial link, then drives both to sweep the PA through every configured combination of
frequency band and power level, recording the DAC/VBIAS setting needed to hit each target power.
The resulting table can be sent back to the VTX and stored in EEPROM.

It's primarily intended for use with OpenPixelOSD, but can be used with other VTX firmware that
implements the MSP commands that this tool uses and have the same behavior.

## Why it exists

The mapping from a VTX's internal DAC/VBIAS setting to its actual RF output power isn't fixed --
it depends on the PA hardware, the antenna path, the VTX RF BIAS control circuit and components, 
and unit-to-unit manufacturing tolerances. That mapping has to be measured on real hardware,
not assumed, or the VTX ends up transmitting at the wrong power (too high for regulatory limits
or interference with other pilots, or too low for usable range) and misreporting its power level
to the pilot. Measuring that mapping by hand -- one DAC value, one frequency, one power level at
a time, against a power meter -- is slow and error-prone. `vtx-tool` automates the sweep, so the
calibration table can be generated reliably and repeatably.

## VTX Tables

VTX tables define which frequencies the VTX can transmit on and at what power levels, usually
the VTX table is managed by a flight controller.  This tool responds to the requests for a VTX table in the same way that a 
flight controller would, so that the VTX can be easily tested without needing a flight controller.

Due to global radio frequency allocations and regulatory requirements, VTXs usually don't come with
a built-in VTX table, putting on onus on the user to use one that complies with local regulations.

## Features

* Automatic calibration sweeps across all configured power levels and frequencies.
* Manual calibration mode for hand-tuning individual DAC values, with live detector feedback.
* Live power, temperature and calibration status graphs during a sweep.
* VTX table editor -- bands, power levels.
* Support for external RF power meters (e.g. ImmersionRC).
* Persistent VTX table storage, with load/save support.
* MSP displayport OSD overlay while calibrating.
* Cross-platform desktop UI (built with `egui` / `eframe`).

## Running from CLI

```
VTX Tool - RF PA calibration + VTX table tool

Usage: vtx-tool.exe [OPTIONS]

Options:
      --vtx-port <VTX_PORT>
      --meter-port <METER_PORT>
      --meter-kind <METER_KIND>
      --attenuation <ATTENUATION>
      --log-level <LOG_LEVEL>      [default: debug]
      --vtx-table <VTX_TABLE>
  -h, --help                       Print help
```

If `--vtx-port`, `--meter-port` and `--meter-kind` are all supplied, `vtx-tool` connects to both
the VTX and the power meter automatically on startup instead of waiting for them to be connected
manually from the UI.

## Building from source

Requires a recent stable Rust toolchain, installed via [rustup](https://rustup.rs).

```
git clone https://github.com/hydra/vtx-tool.git
cd vtx-tool
cargo build --release
```

The built binary will be at `target/release/vtx-tool` (`vtx-tool.exe` on Windows).

To build and run in one step:

```
cargo run --release -- --help
```

Runs on Windows, Linux and macOS but only tested on Windows so far, create a pull request if you need
to make it run on other platforms.

## License

Available under APACHE *or* MIT licenses.

* [APACHE](LICENSE-APACHE)
* [MIT](LICENSE-MIT)

## Authors

* Dominic Clifton - Project founder, product owner, vibe coder and tweaker.
* Claude - Code author of most of the code.

## Notes on AI usage

- yes, Claude is AI and will have made mistakes, feel free to make human or AI generated PRs to fix or refactor things.
- For this tool, I (Dominic Clifton) value my sanity over code quality, egui layout and UI tinkering is best done by the
  AI as it's waaaay to time-consuming otherwise, and I only have so much time...
- Let it be known that I do not like the codebase, and there's no way I as a human would have written it like it is.

## Links

* Github: https://github.com/hydra/vtx-tool

## Contributing

If you'd like to contribute, please raise an issue or a PR on the GitHub issue tracker.
