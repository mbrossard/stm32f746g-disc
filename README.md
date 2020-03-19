# stm32f746g-disc

[![Build Status](https://travis-ci.org/mbrossard/stm32f746g-disc.svg?branch=master)](https://travis-ci.org/mbrossard/stm32f746g-disc)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

_stm32f746g-disc_ contains a basic board support package for the
[STM32F746G-DISCO](https://www.st.com/en/evaluation-tools/32f746gdiscovery.html)
microcontroller board to write firmwares using the Rust language. This board features
multiple user programmable LEDs, an LCD display with resistive touch layer,
64Mbit of memory and a user programmable USB connector, 2 microphones and an Ethernet
port. It also contains a (non-removable) capable ST-Link V2 debugging interface.

This repository is a fork of
[embed-rs/stm32f7-discovery](https://github.com/embed-rs/stm32f7-discovery)
I created to use stable rust. The original crate relies on many nightly options,
which is why many features have been disabled (like `ethernet` and `sd`) or
have be changes but untested (like `i2c`).

## Building

- Install the thumbv7em-none-eabihf target:
  ```
  $ rustup target add thumbv7em-none-eabihf
  ```
- Build the library and its examples
  ```
  $ cargo build --examples
  ```

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)

- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.
