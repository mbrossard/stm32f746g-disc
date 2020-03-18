//! Device library for the STM32F746NG discovery board.
//!
//! Most of the device specific code is based on the stm32f746ng [reference manual],
//! the [STM32CubeF7] package, and the [other stm32f746ng resources].
//!
//! [reference manual]: https://www.st.com/resource/en/reference_manual/dm00124865.pdf
//! [STM32CubeF7]: https://www.st.com/content/st_com/en/products/embedded-software/mcus-embedded-software/stm32-embedded-software/stm32cube-mcu-packages/stm32cubef7.html#getsoftware-scroll
//! [other stm32f746ng resources]: https://www.st.com/content/st_com/en/products/microcontrollers/stm32-32-bit-arm-cortex-mcus/stm32-high-performance-mcus/stm32f7-series/stm32f7x6/stm32f746ng.html#design-scroll

#![no_std]
#![warn(missing_docs)]

extern crate cortex_m_rt as rt;

#[macro_use]
pub mod lcd;
pub mod gpio;
pub mod i2c;
pub mod init;
pub mod random;
pub mod system_clock;
pub mod touch;
