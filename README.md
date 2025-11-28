# `tca8418-rs`

Crate for using the [TCA8418 I2C Controlled Keypad Scan IC](https://www.ti.com/lit/ds/symlink/tca8418.pdf?ts=1764249832240) with `embedded-hal` compatible frameworks.

This crate primarily consumes `embedded-hal-async`'s `I2c` trait that can be used with I2C implementations from various Rust HALs, or from a bus via [`embedded-hal-bus`](https://docs.rs/embedded-hal-bus/latest/embedded_hal_bus/).

See `examples/` for usage examples with Embassy.

## Features

- `defmt` - Enables debug and error logging over [`defmt`](https://docs.rs/defmt/latest/defmt/).

## TODO

- [x] Add an implementation using the `embedded-hal` I2C blocking traits.

