use super::*;
use core::marker::PhantomData;
use stm32f7::stm32f7x6::{
    gpioa, gpiob, gpiod, GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH, GPIOI, GPIOJ,
    GPIOK,
};

/// Abstraction for a GPIO port that allows safe configuration of the port's pins.
pub struct GpioPort<T: RegisterBlockTrait> {
    pub(super) pin_in_use: [bool; 16],
    register_block: T,
}

/// Errors that can occur during pin initialization.
#[derive(Debug)]
pub enum Error {
    /// The specified GPIO pin is already in use.
    PinAlreadyInUse(PinNumber),
}

/// This trait allows generic functions that work on all three register block types.
pub trait RegisterBlockTrait {
    /// The IDR (input data register) type, returned by the `idr` function.
    type Idr: IdrTrait + 'static;

    /// The ODR (output data register) type, returned by the `odr` function.
    type Odr: OdrTrait + 'static;

    /// The BSRR (bit set and reset register) type, returned by the `bsrr` function.
    type Bsrr: BsrrTrait + 'static;

    /// Returns a static reference to the input data register.
    fn idr(&self) -> &'static Self::Idr;

    /// Returns a static reference to the output data register.
    fn odr(&self) -> &'static Self::Odr;

    /// Returns a static reference to the bit set and reset register.
    fn bsrr(&self) -> &'static Self::Bsrr;

    /// Set the mode register for the specified pins to the given `Mode`.
    fn set_mode(&mut self, pins: &[PinNumber], mode: Mode);

    /// Set the resistor register for the specified pins to the given `Resistor`.
    fn set_resistor(&mut self, pins: &[PinNumber], resistor: Resistor);

    /// Set the output type register for the specified pins to the given `OutputType`.
    fn set_out_type(&mut self, pins: &[PinNumber], out_type: OutputType);

    /// Set the output speed register for the specified pins to the given `OutputSpeed`.
    fn set_out_speed(&mut self, pins: &[PinNumber], out_speed: OutputSpeed);

    /// Set the alternate function register for the specified pins to the given `AlternateFunction`.
    fn set_alternate_fn(&mut self, pins: &[PinNumber], alternate_fn: AlternateFunction);
}

impl<T: RegisterBlockTrait> GpioPort<T> {
    /// Create a new GPIO port from the passed register block.
    pub fn new(register_block: T) -> Self {
        Self {
            register_block,
            pin_in_use: [false; 16],
        }
    }

    /// Initialize the specified pin as an input pin.
    pub fn to_input(&mut self, pin: PinNumber, resistor: Resistor) -> Result<impl InputPin, Error> {
        self.use_pin(pin)?;

        self.register_block.set_mode(&[pin], Mode::Input);
        self.register_block.set_resistor(&[pin], resistor);

        Ok(InputPinImpl {
            pin,
            input_data: ReadOnlyIdr(self.register_block.idr()),
        })
    }

    /// Initialize the specified pin as an output pin.
    pub fn to_output(
        &mut self,
        pin: PinNumber,
        out_type: OutputType,
        out_speed: OutputSpeed,
        resistor: Resistor,
    ) -> Result<impl OutputPin, Error> {
        self.use_pin(pin)?;

        self.register_block.set_mode(&[pin], Mode::Output);
        self.register_block.set_out_type(&[pin], out_type);
        self.register_block.set_out_speed(&[pin], out_speed);
        self.register_block.set_resistor(&[pin], resistor);

        let output_pin: OutputPinImpl<T::Odr, T::Bsrr> = OutputPinImpl {
            pin,
            output_data: ReadOnlyOdr(self.register_block.odr()),
            bit_set_reset: BsrrRef {
                register: self.register_block.bsrr() as *const _ as *mut _,
                phantom: PhantomData,
            },
        };
        Ok(output_pin)
    }

    /// Initialize the specified pin as an alternate function pin.
    pub fn to_alternate_function(
        &mut self,
        pin: PinNumber,
        alternate_fn: AlternateFunction,
        typ: OutputType,
        speed: OutputSpeed,
        resistor: Resistor,
    ) -> Result<(), Error> {
        self.to_alternate_function_all(&[pin], alternate_fn, typ, speed, resistor)
    }

    /// Initialize the specified pins as alternate function pins.
    pub fn to_alternate_function_all(
        &mut self,
        pins: &[PinNumber],
        alternate_fn: AlternateFunction,
        typ: OutputType,
        speed: OutputSpeed,
        resistor: Resistor,
    ) -> Result<(), Error> {
        self.use_pins(pins)?;

        self.register_block.set_mode(pins, Mode::Alternate);
        self.register_block.set_resistor(pins, resistor);
        self.register_block.set_out_type(pins, typ);
        self.register_block.set_out_speed(pins, speed);
        self.register_block.set_alternate_fn(pins, alternate_fn);

        Ok(())
    }

    fn use_pin(&mut self, pin: PinNumber) -> Result<(), Error> {
        if self.pin_in_use[pin as usize] {
            Err(Error::PinAlreadyInUse(pin))
        } else {
            self.pin_in_use[pin as usize] = true;
            Ok(())
        }
    }

    fn use_pins(&mut self, pins: &[PinNumber]) -> Result<(), Error> {
        // create a copy of the pin_in_use array since we only want to modify it in case of success
        let mut pin_in_use = self.pin_in_use;

        for &pin in pins {
            if pin_in_use[pin as usize] {
                return Err(Error::PinAlreadyInUse(pin));
            } else {
                pin_in_use[pin as usize] = true;
            }
        }

        // success => write back updated pin_in_use array
        self.pin_in_use = pin_in_use;

        Ok(())
    }
}

macro_rules! impl_register_block_trait {
    ($register_block:tt, $gpio:tt) => {
        impl RegisterBlockTrait for $register_block {
            type Idr = $gpio::IDR;
            type Odr = $gpio::ODR;
            type Bsrr = $gpio::BSRR;

            fn idr(&self) -> &'static Self::Idr {
                &unsafe { &*Self::ptr() }.idr
            }

            fn odr(&self) -> &'static Self::Odr {
                &unsafe { &*Self::ptr() }.odr
            }

            fn bsrr(&self) -> &'static Self::Bsrr {
                &unsafe { &*Self::ptr() }.bsrr
            }

            fn set_mode(&mut self, pins: &[PinNumber], mode: Mode) {
                use self::PinNumber::*;
                use stm32f7::stm32f7x6::$gpio::moder::MODER15_A;

                let variant = || match mode {
                    Mode::Input => MODER15_A::INPUT,
                    Mode::Output => MODER15_A::OUTPUT,
                    Mode::Alternate => MODER15_A::ALTERNATE,
                    Mode::Analog => MODER15_A::ANALOG,
                };

                self.moder.modify(|_, w| {
                    for pin in pins {
                        match pin {
                            Pin0 => w.moder0().variant(variant()),
                            Pin1 => w.moder1().variant(variant()),
                            Pin2 => w.moder2().variant(variant()),
                            Pin3 => w.moder3().variant(variant()),
                            Pin4 => w.moder4().variant(variant()),
                            Pin5 => w.moder5().variant(variant()),
                            Pin6 => w.moder6().variant(variant()),
                            Pin7 => w.moder7().variant(variant()),
                            Pin8 => w.moder8().variant(variant()),
                            Pin9 => w.moder9().variant(variant()),
                            Pin10 => w.moder10().variant(variant()),
                            Pin11 => w.moder11().variant(variant()),
                            Pin12 => w.moder12().variant(variant()),
                            Pin13 => w.moder13().variant(variant()),
                            Pin14 => w.moder14().variant(variant()),
                            Pin15 => w.moder15().variant(variant()),
                        };
                    }
                    w
                })
            }

            fn set_resistor(&mut self, pins: &[PinNumber], resistor: Resistor) {
                use self::PinNumber::*;
                use stm32f7::stm32f7x6::$gpio::pupdr::PUPDR15_A;

                let variant = || match resistor {
                    Resistor::NoPull => PUPDR15_A::FLOATING,
                    Resistor::PullUp => PUPDR15_A::PULLUP,
                    Resistor::PullDown => PUPDR15_A::PULLDOWN,
                };

                self.pupdr.modify(|_, w| {
                    for pin in pins {
                        match pin {
                            Pin0 => w.pupdr0().variant(variant()),
                            Pin1 => w.pupdr1().variant(variant()),
                            Pin2 => w.pupdr2().variant(variant()),
                            Pin3 => w.pupdr3().variant(variant()),
                            Pin4 => w.pupdr4().variant(variant()),
                            Pin5 => w.pupdr5().variant(variant()),
                            Pin6 => w.pupdr6().variant(variant()),
                            Pin7 => w.pupdr7().variant(variant()),
                            Pin8 => w.pupdr8().variant(variant()),
                            Pin9 => w.pupdr9().variant(variant()),
                            Pin10 => w.pupdr10().variant(variant()),
                            Pin11 => w.pupdr11().variant(variant()),
                            Pin12 => w.pupdr12().variant(variant()),
                            Pin13 => w.pupdr13().variant(variant()),
                            Pin14 => w.pupdr14().variant(variant()),
                            Pin15 => w.pupdr15().variant(variant()),
                        };
                    }
                    w
                });
            }

            fn set_out_type(&mut self, pins: &[PinNumber], out_type: OutputType) {
                use self::PinNumber::*;
                use stm32f7::stm32f7x6::$gpio::otyper::OT15_A;

                let variant = || match out_type {
                    OutputType::OpenDrain => OT15_A::OPENDRAIN,
                    OutputType::PushPull => OT15_A::PUSHPULL,
                };

                self.otyper.modify(|_, w| {
                    for pin in pins {
                        match pin {
                            Pin0 => w.ot0().variant(variant()),
                            Pin1 => w.ot1().variant(variant()),
                            Pin2 => w.ot2().variant(variant()),
                            Pin3 => w.ot3().variant(variant()),
                            Pin4 => w.ot4().variant(variant()),
                            Pin5 => w.ot5().variant(variant()),
                            Pin6 => w.ot6().variant(variant()),
                            Pin7 => w.ot7().variant(variant()),
                            Pin8 => w.ot8().variant(variant()),
                            Pin9 => w.ot9().variant(variant()),
                            Pin10 => w.ot10().variant(variant()),
                            Pin11 => w.ot11().variant(variant()),
                            Pin12 => w.ot12().variant(variant()),
                            Pin13 => w.ot13().variant(variant()),
                            Pin14 => w.ot14().variant(variant()),
                            Pin15 => w.ot15().variant(variant()),
                        };
                    }
                    w
                })
            }

            fn set_out_speed(&mut self, pins: &[PinNumber], out_speed: OutputSpeed) {
                use self::PinNumber::*;
                use stm32f7::stm32f7x6::$gpio::ospeedr::OSPEEDR15_A;

                let variant = || match out_speed {
                    OutputSpeed::Low => OSPEEDR15_A::LOWSPEED,
                    OutputSpeed::Medium => OSPEEDR15_A::MEDIUMSPEED,
                    OutputSpeed::High => OSPEEDR15_A::HIGHSPEED,
                    OutputSpeed::VeryHigh => OSPEEDR15_A::VERYHIGHSPEED,
                };

                self.ospeedr.modify(|_, w| {
                    for pin in pins {
                        match pin {
                            Pin0 => w.ospeedr0().variant(variant()),
                            Pin1 => w.ospeedr1().variant(variant()),
                            Pin2 => w.ospeedr2().variant(variant()),
                            Pin3 => w.ospeedr3().variant(variant()),
                            Pin4 => w.ospeedr4().variant(variant()),
                            Pin5 => w.ospeedr5().variant(variant()),
                            Pin6 => w.ospeedr6().variant(variant()),
                            Pin7 => w.ospeedr7().variant(variant()),
                            Pin8 => w.ospeedr8().variant(variant()),
                            Pin9 => w.ospeedr9().variant(variant()),
                            Pin10 => w.ospeedr10().variant(variant()),
                            Pin11 => w.ospeedr11().variant(variant()),
                            Pin12 => w.ospeedr12().variant(variant()),
                            Pin13 => w.ospeedr13().variant(variant()),
                            Pin14 => w.ospeedr14().variant(variant()),
                            Pin15 => w.ospeedr15().variant(variant()),
                        };
                    }
                    w
                })
            }

            fn set_alternate_fn(&mut self, pins: &[PinNumber], alternate_fn: AlternateFunction) {
                use self::PinNumber::*;
                use stm32f7::stm32f7x6::$gpio::afrh::AFRH15_A;
                use stm32f7::stm32f7x6::$gpio::afrl::AFRL7_A;

                let variant = || match alternate_fn {
                    AlternateFunction::AF0 => (AFRL7_A::AF0, AFRH15_A::AF0),
                    AlternateFunction::AF1 => (AFRL7_A::AF1, AFRH15_A::AF1),
                    AlternateFunction::AF2 => (AFRL7_A::AF2, AFRH15_A::AF2),
                    AlternateFunction::AF3 => (AFRL7_A::AF3, AFRH15_A::AF3),
                    AlternateFunction::AF4 => (AFRL7_A::AF4, AFRH15_A::AF4),
                    AlternateFunction::AF5 => (AFRL7_A::AF5, AFRH15_A::AF5),
                    AlternateFunction::AF6 => (AFRL7_A::AF6, AFRH15_A::AF6),
                    AlternateFunction::AF7 => (AFRL7_A::AF7, AFRH15_A::AF7),
                    AlternateFunction::AF8 => (AFRL7_A::AF8, AFRH15_A::AF8),
                    AlternateFunction::AF9 => (AFRL7_A::AF9, AFRH15_A::AF9),
                    AlternateFunction::AF10 => (AFRL7_A::AF10, AFRH15_A::AF10),
                    AlternateFunction::AF11 => (AFRL7_A::AF11, AFRH15_A::AF11),
                    AlternateFunction::AF12 => (AFRL7_A::AF12, AFRH15_A::AF12),
                    AlternateFunction::AF13 => (AFRL7_A::AF13, AFRH15_A::AF13),
                    AlternateFunction::AF14 => (AFRL7_A::AF14, AFRH15_A::AF14),
                    AlternateFunction::AF15 => (AFRL7_A::AF15, AFRH15_A::AF15),
                };

                self.afrh.modify(|_, wh| {
                    self.afrl.modify(|_, wl| {
                        for pin in pins {
                            match pin {
                                Pin0 => {
                                    wl.afrl0().variant(variant().0);
                                }
                                Pin1 => {
                                    wl.afrl1().variant(variant().0);
                                }
                                Pin2 => {
                                    wl.afrl2().variant(variant().0);
                                }
                                Pin3 => {
                                    wl.afrl3().variant(variant().0);
                                }
                                Pin4 => {
                                    wl.afrl4().variant(variant().0);
                                }
                                Pin5 => {
                                    wl.afrl5().variant(variant().0);
                                }
                                Pin6 => {
                                    wl.afrl6().variant(variant().0);
                                }
                                Pin7 => {
                                    wl.afrl7().variant(variant().0);
                                }
                                Pin8 => {
                                    wh.afrh8().variant(variant().1);
                                }
                                Pin9 => {
                                    wh.afrh9().variant(variant().1);
                                }
                                Pin10 => {
                                    wh.afrh10().variant(variant().1);
                                }
                                Pin11 => {
                                    wh.afrh11().variant(variant().1);
                                }
                                Pin12 => {
                                    wh.afrh12().variant(variant().1);
                                }
                                Pin13 => {
                                    wh.afrh13().variant(variant().1);
                                }
                                Pin14 => {
                                    wh.afrh14().variant(variant().1);
                                }
                                Pin15 => {
                                    wh.afrh15().variant(variant().1);
                                }
                            };
                        }
                        wl
                    });
                    wh
                })
            }
        }
    };
}

impl_register_block_trait!(GPIOA, gpioa);
impl_register_block_trait!(GPIOB, gpiob);
impl_register_block_trait!(GPIOC, gpiod);
impl_register_block_trait!(GPIOD, gpiod);
impl_register_block_trait!(GPIOE, gpiod);
impl_register_block_trait!(GPIOF, gpiod);
impl_register_block_trait!(GPIOG, gpiod);
impl_register_block_trait!(GPIOH, gpiod);
impl_register_block_trait!(GPIOI, gpiod);
impl_register_block_trait!(GPIOJ, gpiod);
impl_register_block_trait!(GPIOK, gpiod);
