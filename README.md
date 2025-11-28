# `tca8418-rs`

Embedded Rust crate for using the [Texas Instrument's TCA8418 I2C Controlled Keypad Scan IC](https://www.ti.com/lit/ds/symlink/tca8418.pdf?ts=1764249832240) with embedded-hal compatible frameworks. The crate primarily consumes `embedded-hal-async`'s `I2c` trait that can be used with I2C implementations from various Rust HALs.

See `examples/` for usage examples with Embassy.

## Features

- `defmt` - Enables debug and error logging over [`defmt`](https://docs.rs/defmt/latest/defmt/).

## TODO

- [] Add an implementation using the `embedded-hal` I2C blocking traits.
- [] Add a set of macros to allow using either `defmt` or `log` for logging depending on the enabled feature.
