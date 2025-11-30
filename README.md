# `tca8418-rs`

Crate for using the [TCA8418 I2C Controlled Keypad Scan IC](https://www.ti.com/lit/ds/symlink/tca8418.pdf?ts=1764249832240) with `embedded-hal` compatible frameworks.

This crate consumes either `embedded-hal` or `embedded-hal-async`'s `I2c` traits that can be used with I2C implementations from various Rust HALs, or from a bus via [`embedded-hal-bus`](https://docs.rs/embedded-hal-bus/latest/embedded_hal_bus/).

See `examples/` for usage examples with Embassy.

## Features

- `defmt` - Enables debug and error logging over [`defmt`](https://docs.rs/defmt/latest/defmt/).

## Reset

I highly recommend connecting the reset line - if the TCA8418 isn't reset in tandem with the MCU, or the MCU tries to reconfigure the TCA8418 without a reset first, then there could be key events in the TCA8418 FIFO from before the MCU expects.

I ran into a problem with one project where the MCU would restart, but then, because the power to the TCA8418 wasn't also cycled, there would be a backlog of keys pressed before or during the restart that the MCU would suddenly try to process, leading to what looked like ghost inputs.
