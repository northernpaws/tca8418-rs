use concat_idents::concat_idents;
/// Helper that allows us to write registers as structs and read
/// them directly as their corrosponding binary representation.
use proc_bitfield::bitfield;

pub const RESERVED_ADDRESS: u8 = 0x00;

/// Configuration register (interrupt processor, interrupt enables).
pub const CONFIGURATION_ADDRESS: u8 = 0x01;

/// Interrupt status register.
pub const INT_STAT_ADDRESS: u8 = 0x02;

/// Key lock and event counter register.
pub const KEY_LOCK_EVENT_COUNTER_ADDRESS: u8 = 0x03;

/// Key event register A.
pub const KEY_EVENT_A_ADDRESS: u8 = 0x04;

/// Key event register B.
pub const KEY_EVENT_B_ADDRESS: u8 = 0x05;

/// Key event register C.
pub const KEY_EVENT_C_ADDRESS: u8 = 0x06;

/// Key event register D.
pub const KEY_EVENT_D_ADDRESS: u8 = 0x07;

/// Key event register E.
pub const KEY_EVENT_E_ADDRESS: u8 = 0x08;

/// Key event register F.
pub const KEY_EVENT_F_ADDRESS: u8 = 0x09;

/// Key event register G.
pub const KEY_EVENT_G_ADDRESS: u8 = 0x0A;

/// Key event register H.
pub const KEY_EVENT_H_ADDRESS: u8 = 0x0B;

/// Key event register I.
pub const KEY_EVENT_I_ADDRESS: u8 = 0x0C;

/// Key event register J.
pub const KEY_EVENT_J_ADDRESS: u8 = 0x0D;

/// Keypad lock 1 to lock 2 timer.
pub const KEYPAD_LOCK_TIMER_ADDRESS: u8 = 0x0E;

/// Unlock key 1.
pub const UNLOCK1_ADDRESS: u8 = 0x0F;

/// Unlock key 2.
pub const UNLOCK2_ADDRESS: u8 = 0x10;

/// GPIO interrupt status.
pub const GPIO_INTERRUPT_STATUS1_ADDRESS: u8 = 0x11;

/// GPIO interrupt status.
pub const GPIO_INTERRUPT_STATUS2_ADDRESS: u8 = 0x12;

/// GPIO interrupt status.
pub const GPIO_INTERRUPT_STATUS3_ADDRESS: u8 = 0x13;

/// GPIO data status (read twice to clear).
pub const GPIO_DATA_STATUS1_ADDRESS: u8 = 0x14;

/// GPIO data status (read twice to clear).
pub const GPIO_DATA_STATUS2_ADDRESS: u8 = 0x15;

/// GPIO data status (read twice to clear).
pub const GPIO_DATA_STATUS3_ADDRESS: u8 = 0x16;

/// GPIO data out.
pub const GPIO_DATA_OUT1_ADDRESS: u8 = 0x17;

/// GPIO data out.
pub const GPIO_DATA_OUT2_ADDRESS: u8 = 0x18;

/// GPIO data out.
pub const GPIO_DATA_OUT3_ADDRESS: u8 = 0x19;

/// GPIO interrupt enable.
pub const GPIO_INTERRUPT_ENABLE1_ADDRESS: u8 = 0x1A;

/// GPIO interrupt enable.
pub const GPIO_INTERRUPT_ENABLE2_ADDRESS: u8 = 0x1B;

/// GPIO interrupt enable.
pub const GPIO_INTERRUPT_ENABLE3_ADDRESS: u8 = 0x1C;

/// Keypad or GPIO selection.
///  0: GPIO
///  1: KP matrix
pub const KEYPAD_GPIO1_ADDRESS: u8 = 0x1D;

/// Keypad or GPIO selection.
///  0: GPIO
///  1: KP matrix
pub const KEYPAD_GPIO2_ADDRESS: u8 = 0x1E;

/// Keypad or GPIO selection.
///  0: GPIO
///  1: KP matrix
pub const KEYPAD_GPIO3_ADDRESS: u8 = 0x1F;

/// GPI event mode 1.
pub const GPI_EVENT_MODE1_ADDRESS: u8 = 0x20;

/// GPI event mode 2.
pub const GPI_EVENT_MODE2_ADDRESS: u8 = 0x21;

/// GPI event mode 3.
pub const GPI_EVENT_MODE3_ADDRESS: u8 = 0x22;

/// GPIO data direction.
///  0: input
///  1: output
pub const GPIO_DIRECTION1_ADDRESS: u8 = 0x23;

/// GPIO data direction.
///  0: input
///  1: output
pub const GPIO_DIRECTION2_ADDRESS: u8 = 0x24;

/// GPIO data direction.
///  0: input
///  1: output
pub const GPIO_DIRECTION3_ADDRESS: u8 = 0x25;

/// GPIO edge/level detect
///  0: falling/low
///  1: rising/high
pub const GPIO_INTERRUPT_LEVEL1_ADDRESS: u8 = 0x26;

/// GPIO edge/level detect
///  0: falling/low
///  1: rising/high
pub const GPIO_INTERRUPT_LEVEL2_ADDRESS: u8 = 0x27;

/// GPIO edge/level detect
///  0: falling/low
///  1: rising/high
pub const GPIO_INTERRUPT_LEVEL3_ADDRESS: u8 = 0x28;

/// Debounce disable
///  0: debounce enabled
///  1: debounce disabled
pub const DEBOUNCE_DISABLE1_ADDRESS: u8 = 0x29;

/// Debounce disable
///  0: debounce enabled
///  1: debounce disabled
pub const DEBOUNCE_DISABLE2_ADDRESS: u8 = 0x2A;

/// Debounce disable
///  0: debounce enabled
///  1: debounce disabled
pub const DEBOUNCE_DISABLE3_ADDRESS: u8 = 0x2B;

/// GPIO pull-up disable
///  0: pull-up enabled
///  1: pull-up disabled
pub const GPIO_PULL_UP_DISABLE1_ADDRESS: u8 = 0x2C;

/// GPIO pull-up disable
///  0: pull-up enabled
///  1: pull-up disabled
pub const GPIO_PULL_UP_DISABLE2_ADDRESS: u8 = 0x2D;

/// GPIO pull-up disable
///  0: pull-up enabled
///  1: pull-up disabled
pub const GPIO_PULL_UP_DISABLE3_ADDRESS: u8 = 0x2E;

pub const RESERVED2_ADDRESS: u8 = 0x2F;

/// Mapping of the TCA8418 register names to addresses using a conrete type.
///
/// The constants are coalesced into an enum to make it possible
/// to type the register addresses in some methods.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Address {
    Reserved = RESERVED_ADDRESS,

    /// Configuration register (interrupt processor, interrupt enables).
    Configuration = CONFIGURATION_ADDRESS,

    /// Interrupt status register.
    IntStat = INT_STAT_ADDRESS,

    /// Key lock and event counter register.
    KeyLockEventCounter = KEY_LOCK_EVENT_COUNTER_ADDRESS,

    /// Key event register A.
    KeyEventA = KEY_EVENT_A_ADDRESS,
    /// Key event register B.
    KeyEventB = KEY_EVENT_B_ADDRESS,
    /// Key event register C.
    KeyEventC = KEY_EVENT_C_ADDRESS,
    /// Key event register D.
    KeyEventD = KEY_EVENT_D_ADDRESS,
    /// Key event register E.
    KeyEventE = KEY_EVENT_E_ADDRESS,
    /// Key event register F.
    KeyEventF = KEY_EVENT_F_ADDRESS,
    /// Key event register G.
    KeyEventG = KEY_EVENT_G_ADDRESS,
    /// Key event register H.
    KeyEventH = KEY_EVENT_H_ADDRESS,
    /// Key event register I.
    KeyEventI = KEY_EVENT_I_ADDRESS,
    /// Key event register J.
    KeyEventJ = KEY_EVENT_J_ADDRESS,

    /// Keypad lock 1 to lock 2 timer.
    KeypadLockTimer = KEYPAD_LOCK_TIMER_ADDRESS,

    /// Unlock key 1.
    Unlock1 = UNLOCK1_ADDRESS,
    /// Unlock key 2.
    Unlock2 = UNLOCK2_ADDRESS,

    /// GPIO interrupt status.
    GPIOInterruptStatus1 = GPIO_INTERRUPT_STATUS1_ADDRESS,
    /// GPIO interrupt status.
    GPIOInterruptStatus2 = GPIO_INTERRUPT_STATUS2_ADDRESS,
    /// GPIO interrupt status.
    GPIOInterruptStatus3 = GPIO_INTERRUPT_STATUS3_ADDRESS,
    /// GPIO data status (read twice to clear).
    GPIODataStatus1 = GPIO_DATA_STATUS1_ADDRESS,
    /// GPIO data status (read twice to clear).
    GPIODataStatus2 = GPIO_DATA_STATUS2_ADDRESS,
    /// GPIO data status (read twice to clear).
    GPIODataStatus3 = GPIO_DATA_STATUS3_ADDRESS,

    /// GPIO data out.
    GPIODataOut1 = GPIO_DATA_OUT1_ADDRESS,
    /// GPIO data out.
    GPIODataOut2 = GPIO_DATA_OUT2_ADDRESS,
    /// GPIO data out.
    GPIODataOut3 = GPIO_DATA_OUT3_ADDRESS,

    /// GPIO interrupt enable.
    GPIOInterruptEnable1 = GPIO_INTERRUPT_ENABLE1_ADDRESS,
    /// GPIO interrupt enable.
    GPIOInterruptEnable2 = GPIO_INTERRUPT_ENABLE2_ADDRESS,
    /// GPIO interrupt enable.
    GPIOInterruptEnable3 = GPIO_INTERRUPT_ENABLE3_ADDRESS,

    /// Keypad or GPIO selection.
    ///  0: GPIO
    ///  1: KP matrix
    KeypadGPIO1 = KEYPAD_GPIO1_ADDRESS,
    /// Keypad or GPIO selection.
    ///  0: GPIO
    ///  1: KP matrix
    KeypadGPIO2 = KEYPAD_GPIO2_ADDRESS,
    /// Keypad or GPIO selection.
    ///  0: GPIO
    ///  1: KP matrix
    KeypadGPIO3 = KEYPAD_GPIO3_ADDRESS,

    /// GPI event mode 1.
    GPIEventMode1 = GPI_EVENT_MODE1_ADDRESS,
    /// GPI event mode 2.
    GPIEventMode2 = GPI_EVENT_MODE2_ADDRESS,
    /// GPI event mode 3.
    GPIEventMode3 = GPI_EVENT_MODE3_ADDRESS,

    /// GPIO data direction.
    ///  0: input
    ///  1: output
    GPIODirection1 = GPIO_DIRECTION1_ADDRESS,
    /// GPIO data direction.
    ///  0: input
    ///  1: output
    GPIODirection2 = GPIO_DIRECTION2_ADDRESS,
    /// GPIO data direction.
    ///  0: input
    ///  1: output
    GPIODirection3 = GPIO_DIRECTION3_ADDRESS,

    /// GPIO edge/level detect
    ///  0: falling/low
    ///  1: rising/high
    GPIOInterruptLevel1 = GPIO_INTERRUPT_LEVEL1_ADDRESS,
    /// GPIO edge/level detect
    ///  0: falling/low
    ///  1: rising/high
    GPIOInterruptLevel2 = GPIO_INTERRUPT_LEVEL2_ADDRESS,
    /// GPIO edge/level detect
    ///  0: falling/low
    ///  1: rising/high
    GPIOInterruptLevel3 = GPIO_INTERRUPT_LEVEL3_ADDRESS,

    /// Debounce disable
    ///  0: debounce enabled
    ///  1: debounce disabled
    DebounceDisable1 = DEBOUNCE_DISABLE1_ADDRESS,
    /// Debounce disable
    ///  0: debounce enabled
    ///  1: debounce disabled
    DebounceDisable2 = DEBOUNCE_DISABLE2_ADDRESS,
    /// Debounce disable
    ///  0: debounce enabled
    ///  1: debounce disabled
    DebounceDisable3 = DEBOUNCE_DISABLE3_ADDRESS,

    /// GPIO pull-up disable
    ///  0: pull-up enabled
    ///  1: pull-up disabled
    GPIOPullUpDisable1 = GPIO_PULL_UP_DISABLE1_ADDRESS,
    /// GPIO pull-up disable
    ///  0: pull-up enabled
    ///  1: pull-up disabled
    GPIOPullUpDisable2 = GPIO_PULL_UP_DISABLE2_ADDRESS,
    /// GPIO pull-up disable
    ///  0: pull-up enabled
    ///  1: pull-up disabled
    GPIOPullUpDisable3 = GPIO_PULL_UP_DISABLE3_ADDRESS,

    Reserved2 = RESERVED2_ADDRESS,
}

/// A trait representing a register.
///
/// All registers for the TCA8418 are 8 bit fields.
pub trait Register: Into<u8> + From<u8> {
    /// Constant containing the register address.
    ///
    /// Provided so that template parameters can use a trait
    /// implementation as a constant to extract the address.
    const ADDRESS: u8;

    /// Returns the address of the register.
    fn address() -> Address;
}

bitfield! {
    // Configuration Register (Address 0x01)
    #[derive(Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct Configuration(pub u8): Debug, FromStorage, IntoStorage, DerefStorage {
        /// Auto-increment for read and write operations.
        ///  0 = disabled
        ///  1 = enabled
        pub ai: bool @ 7,

        /// GPI event mode configuration
        ///  0 = GPI events are tracked when keypad is locked.
        ///  1 = GPI events are not tracked when keypad is locked.
        pub gpi_event_mode_configuration: bool @ 6,

        /// Overflow mode.
        /// 0 = disabled; Overflow data is lost.
        ///  1 = enabled; Overflow data shifts with last event pushing first event out.
        pub overflow_mode: bool @ 5,

        /// Interrupt configuration.
        ///   0 = Processor interrupt remains asserted (or low) if
        ///        host tries to clear interrupt while there is still
        ///        a pending key press, key release or GPI interrupt.
        ///   1 = processor interrupt is deasserted for 50 μs
        ///        and reassert with pending interrupts.
        pub interrupt_configuration: bool @ 4,

        /// Overflow interrupt enable.
        ///  0 = disabled; INT is not asserted if the FIFO overflows.
        ///  1 = enabled; INT becomes asserted if the FIFO overflows.
        pub overflow_interrupt_enable: bool @ 3,

        /// Keypad lock interrupt enable.
        ///  0 = disabled; INT is not asserted after a correct unlock key sequence.
        ///  1 = enabled; INT becomes asserted after a correct unlock key sequence.
        pub keypad_lock_interrupt_enable: bool @ 2,

        /// GPI interrupt enable to host processor.
        ///  0 = disabled; INT is not asserted for a change on a GPI.
        ///. 1 = enabled; INT becomes asserted for a change on a GPI.
        pub gpi_interrupt_enable: bool @ 1,

        /// Key events interrupt enable to host processor.
        ///  0 = disabled; INT is not asserted when a key event occurs.
        ///  1 = enabled; INT becomes asserted when a key event occurs.
        pub key_events_interrupt_enable: bool @ 0
    }
}

impl Register for Configuration {
    const ADDRESS: u8 = CONFIGURATION_ADDRESS;

    fn address() -> Address {
        Address::Configuration
    }
}

bitfield! {
    // INT_STAT (Address 0x02)
    #[derive(Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct InterruptStatus(pub u8): Debug, FromStorage, IntoStorage, DerefStorage {
        /// CTRL-ALT-DEL key sequence status.
        ///  0 = interrupt not detected,
        ///  1 = interrupt detected,
        ///
        /// Requires writing a 1 to clear interrupts.
        ///
        /// The following key press combinations will cause a false CAD_INT:
        /// • 1 + 11
        /// • 1 + 21
        /// • 21 + 1 + 11
        pub ctrl_alt_delete_interrupt_status: bool @ 4,

        /// Overflow interrupt status.
        ///  0 = interrupt not detected.
        ///  1 = interrupt detected.
        ///
        /// Requires writing a 1 to clear interrupts.
        pub overflow_interrupt_status: bool @ 3,

        /// Keypad lock interrupt status.
        ///
        /// This is the interrupt to the processor when
        /// the keypad lock sequence is started.
        ///
        ///  0 = interrupt not detected
        ///  1 = interrupt detected
        ///
        /// Requires writing a 1 to clear interrupts.
        pub keypad_lock_interrupt_status: bool @ 2,

        /// GPI interrupt status. Requires writing a 1 to clear interrupts.
        ///  0 = interrupt not detected
        ///  1 = interrupt detected
        ///
        /// Can be used to mask interrupts.
        pub gpi_interrupt_status: bool @ 1,

        /// Key events interrupt status.
        ///  0 = interrupt not detected
        ///  1 = interrupt detected
        ///
        /// Requires writing a 1 to clear interrupts.
        pub key_event_interrupt_status: bool @ 0,
    }
}

impl Register for InterruptStatus {
    const ADDRESS: u8 = INT_STAT_ADDRESS;

    fn address() -> Address {
        Address::IntStat
    }
}

bitfield! {
    // KEY_LCK_EC (Address 0x03)
    #[derive(Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct KeyLockAndEventCounter(pub u8): Debug, FromStorage, IntoStorage, DerefStorage {
        /// Key lock enable.
        ///  0 = disabled; Write a 0 to this bit to unlock the keypad manually.
        ///  1 = enabled; Write a 1 to this bit to lock the keypad.
        pub key_lock_enable: bool @ 6, // K_LCK_EN

        /// Keypad lock status.
        ///  0 = unlock (if LCK1 is 0 too).
        ///  1 = locked (if LCK1 is 1 too).
        pub keypad_lock_status_2: bool @ 5, // LCK2

        /// Keypad lock status.
        ///  0 = unlock (if LCK2 is 0 too).
        ///  1 = locked (if LCK2 is 1 too).
        pub keypad_lock_status_1: bool @ 4, // LCK1

        /// Key event count.
        pub key_event_count: u8 @ 0..=3, // KEC[3:0]
    }
}
impl Register for KeyLockAndEventCounter {
    const ADDRESS: u8 = KEY_LOCK_EVENT_COUNTER_ADDRESS;

    fn address() -> Address {
        Address::KeyLockEventCounter
    }
}

// Helper to write all the GPIOx registers that have the same field names.
macro_rules! key_event {
    ($name:ident, $addr:ident) => {
        bitfield! {
            // KEY_EVENT_A–J (Address 0x04–0x0D)
            //
            // All key event registers can be read, but for the purpose of
            // the FIFO, the user should only read KEY_EVENT_A register.
            //
            // Once all the events in the FIFO have been read, reading of
            // KEY_EVENT_A register will yield a zero value.
            #[derive(Clone, Copy, PartialEq, Eq)]
            #[cfg_attr(feature = "defmt", derive(defmt::Format))]
            pub struct $name(pub u8): Debug, FromStorage, IntoStorage, DerefStorage {

                /// KEA[7] indicate if a key press or key release has happened.
                ///
                /// A ‘0’ means a key release happened.
                /// A ‘1’ means a key has been pressed (which can be cleared on a read).
                pub key_pressed: bool @ 7,

                /// KEA[6:0] indicates the key # pressed or released.
                ///
                /// A value of 0 to 80 indicate which key has been
                /// pressed or released in a keypad matrix.
                ///
                /// Values of 97 to 114 are for GPI events.
                pub key_index: u8 @ 0..=6,
            }
        }

        impl Register for $name {
            concat_idents!(address_name = $addr, _ADDRESS {
                const ADDRESS: u8 = address_name;
            });

            fn address() -> Address {
                Address::$name
            }
        }
    }
}

key_event!(KeyEventA, KEY_EVENT_A);
key_event!(KeyEventB, KEY_EVENT_B);
key_event!(KeyEventC, KEY_EVENT_C);
key_event!(KeyEventD, KEY_EVENT_D);
key_event!(KeyEventE, KEY_EVENT_E);
key_event!(KeyEventF, KEY_EVENT_F);
key_event!(KeyEventG, KEY_EVENT_G);
key_event!(KeyEventH, KEY_EVENT_H);
key_event!(KeyEventI, KEY_EVENT_I);
key_event!(KeyEventJ, KEY_EVENT_J);

bitfield! {
    // KP_LCK_TIMER (Address 0x0E)
    #[derive(Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct KeypadLockTimer(pub u8): Debug, FromStorage, IntoStorage, DerefStorage {
        /// If the keypad lock interrupt mask timer is non-zero, a key event
        /// interrupt (K_INT) will be generated on any first key press.
        ///
        /// The second interrupt (K_LCK_IN) will only be generated when the
        /// correct unlock sequence has been completed. If either timer expires,
        /// the keylock state machine will reset.
        ///
        /// When the interrupt mask timer is disabled (‘0’), a key lock interrupt
        /// will trigger only when the correct unlock sequence is completed.
        ///
        /// The interrupt mask timer should be set for the time it takes for the LCD to dim or turn off.
        pub interrupt_mask_timer: u8 @ 3..=7, // KL[7:3]

        /// Lock1 to Lock2 timer must be non-zero for keylock to be enabled.
        ///
        /// The lock1 to lock2 bits ( KL[2:0] ) define the time
        /// in seconds the user has to press unlock key 2 after
        /// unlock key 1 before the key lock sequence times out.
        pub lock1_lock2_timer: u8 @ 0..=2, // KL[2:0]
    }
}

impl Register for KeypadLockTimer {
    const ADDRESS: u8 = KEYPAD_LOCK_TIMER_ADDRESS;

    fn address() -> Address {
        Address::KeypadLockTimer
    }
}

bitfield! {
    // UNLOCK1 (0x0F)
    #[derive(Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct Unlock1(pub u8): Debug, FromStorage, IntoStorage, DerefStorage {
        pub disabled: bool @ 7,

        /// Index of the key 1 for the unlock sequence.
        pub key_index: u8 @ 0..=7,
    }
}

impl Register for Unlock1 {
    const ADDRESS: u8 = UNLOCK1_ADDRESS;

    fn address() -> Address {
        Address::Unlock1
    }
}

bitfield! {
    // UNLOCK2 (0x10)
    #[derive(Clone, Copy, PartialEq, Eq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct Unlock2(pub u8): Debug, FromStorage, IntoStorage, DerefStorage {
        pub disabled: bool @ 7,

        /// Index of the key 1 for the unlock sequence.
        pub key_index: u8 @ 0..=7,
    }
}

impl Register for Unlock2 {
    const ADDRESS: u8 = UNLOCK2_ADDRESS;

    fn address() -> Address {
        Address::Unlock2
    }
}

// Helper to write all the GPIOx registers that have the same field names.
macro_rules! gpio_register {
    ($name1:ident, $name2:ident, $name3:ident, $addr_base:ident, $typename:ident) => {
        bitfield! {
            #[derive(Clone, Copy, PartialEq, Eq)]
            #[cfg_attr(feature = "defmt", derive(defmt::Format))]
            pub struct $name1(pub u8): Debug, FromStorage, IntoStorage, DerefStorage {
                pub row_7: bool [$typename] @ 7,
                pub row_6: bool [$typename] @ 6,
                pub row_5: bool [$typename] @ 5,
                pub row_4: bool [$typename] @ 4,
                pub row_3: bool [$typename] @ 3,
                pub row_2: bool [$typename] @ 2,
                pub row_1: bool [$typename] @ 1,
                pub row_0: bool [$typename] @ 0,
            }
        }

        impl Register for $name1 {
            concat_idents!(address_name = $addr_base, 1, _ADDRESS {
                const ADDRESS: u8 = address_name;
            });

            fn address() -> Address {
                Address::$name1
            }
        }

        bitfield! {
            #[derive(Clone, Copy, PartialEq, Eq)]
            #[cfg_attr(feature = "defmt", derive(defmt::Format))]
            pub struct $name2(pub u8): Debug, FromStorage, IntoStorage, DerefStorage {
                pub column_7: bool [$typename] @ 7,
                pub column_6: bool [$typename] @ 6,
                pub column_5: bool [$typename] @ 5,
                pub column_4: bool [$typename] @ 4,
                pub column_3: bool [$typename] @ 3,
                pub column_2: bool [$typename] @ 2,
                pub column_1: bool [$typename] @ 1,
                pub column_0: bool [$typename] @ 0,
            }
        }

        impl Register for $name2 {
            concat_idents!(address_name = $addr_base, 2, _ADDRESS {
                const ADDRESS: u8 = address_name;
            });

            fn address() -> Address {
                Address::$name2
            }
        }

        bitfield! {
            #[derive(Clone, Copy, PartialEq, Eq)]
            #[cfg_attr(feature = "defmt", derive(defmt::Format))]
            pub struct $name3(pub u8): Debug, FromStorage, IntoStorage, DerefStorage {
                pub column_9: bool [$typename] @ 1,
                pub column_8: bool [$typename] @ 0,
            }
        }

        impl Register for $name3 {
            concat_idents!(address_name = $addr_base, 3, _ADDRESS {
                const ADDRESS: u8 = address_name;
            });

            fn address() -> Address {
                Address::$name3
            }
        }
    };
}

// All these registers have 3 common sets of fields.
gpio_register!(
    GPIOInterruptStatus1,
    GPIOInterruptStatus2,
    GPIOInterruptStatus3,
    GPIO_INTERRUPT_STATUS,
    bool
);

gpio_register!(
    GPIODataStatus1,
    GPIODataStatus2,
    GPIODataStatus3,
    GPIO_DATA_STATUS,
    bool
);

gpio_register!(
    GPIODataOut1,
    GPIODataOut2,
    GPIODataOut3,
    GPIO_DATA_OUT,
    bool
);

/// A bit value of '0' in any of the unreserved bits disables the
/// corresponding pin's ability to generate an interrupt when the
/// state of the input changes. This is the default value.
///
/// A bit value of 1 in any of the unreserved bits enables the
/// corresponding pin's ability to generate an interrupt when
/// the state of the input changes.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GPIOInterruptEnable {
    Disabled = 0,
    Enabled = 1,
}

impl From<GPIOInterruptEnable> for bool {
    fn from(f: GPIOInterruptEnable) -> bool {
        match f {
            GPIOInterruptEnable::Disabled => false,
            GPIOInterruptEnable::Enabled => true,
        }
    }
}

impl From<bool> for GPIOInterruptEnable {
    fn from(b: bool) -> GPIOInterruptEnable {
        match b {
            false => GPIOInterruptEnable::Disabled,
            true => GPIOInterruptEnable::Enabled,
        }
    }
}

gpio_register!(
    GPIOInterruptEnable1,
    GPIOInterruptEnable2,
    GPIOInterruptEnable3,
    GPIO_INTERRUPT_ENABLE,
    GPIOInterruptEnable
);

/// A bit value of '0' in any of the unreserved bits puts the corresponding pin in GPIO mode.
///
/// A pin in GPIO mode can be configured as an input or an output in the GPIO_DIR1-3 registers.
///
/// This is the default value.
///
/// A 1 in any of these bits puts the pin in key scan mode and becomes part of the keypad array,
/// then it is configured as a row or column accordingly (this is not adjustable).
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GPIOMode {
    GPIOMode = 0,
    KeyScanMode = 1,
}

impl From<GPIOMode> for bool {
    fn from(f: GPIOMode) -> bool {
        match f {
            GPIOMode::GPIOMode => false,
            GPIOMode::KeyScanMode => true,
        }
    }
}

impl From<bool> for GPIOMode {
    fn from(b: bool) -> GPIOMode {
        match b {
            false => GPIOMode::GPIOMode,
            true => GPIOMode::KeyScanMode,
        }
    }
}

gpio_register!(KeypadGPIO1, KeypadGPIO2, KeypadGPIO3, KEYPAD_GPIO, GPIOMode);

/// A bit value of '0' in any of the unreserved bits indicates that
/// it is not part of the event FIFO. This is the default value.
///
/// A 1 in any of these bits means it is part of the event FIFO. When
/// the Event Mode register, then any key presses will be added to the
/// FIFO. Please see Key Event Table for more a pin is setup as a GPI
/// and has a value of 1 in information.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GPIEventMode {
    FIFODisabled = 0,
    FIFOEnabled = 1,
}

impl From<GPIEventMode> for bool {
    fn from(f: GPIEventMode) -> bool {
        match f {
            GPIEventMode::FIFODisabled => false,
            GPIEventMode::FIFOEnabled => true,
        }
    }
}

impl From<bool> for GPIEventMode {
    fn from(b: bool) -> GPIEventMode {
        match b {
            false => GPIEventMode::FIFODisabled,
            true => GPIEventMode::FIFOEnabled,
        }
    }
}

gpio_register!(
    GPIEventMode1,
    GPIEventMode2,
    GPIEventMode3,
    GPI_EVENT_MODE,
    GPIEventMode
);

/// A bit value of '0' in any of the unreserved bits sets the corresponding pin as an input.
///
/// This is the default value. A 1 in any of these bits sets the pin as an output.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GPIODirection {
    Input = 0,
    Output = 1,
}

impl From<GPIODirection> for bool {
    fn from(f: GPIODirection) -> bool {
        match f {
            GPIODirection::Input => false,
            GPIODirection::Output => true,
        }
    }
}

impl From<bool> for GPIODirection {
    fn from(b: bool) -> GPIODirection {
        match b {
            false => GPIODirection::Input,
            true => GPIODirection::Output,
        }
    }
}

gpio_register!(
    GPIODirection1,
    GPIODirection2,
    GPIODirection3,
    GPIO_DIRECTION,
    GPIODirection
);

/// A bit value of '0' indicates that interrupt will be triggered
/// on a high-to-low/low-level transition for the inputs in GPIO
/// mode. This is the default value.
///
/// A bit value of 1 indicates that interrupt will be triggered on
/// a low-to-high/high-level value for the inputs in GPIO mode.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GPIOInterruptLevel {
    HighToLow = 0,
    LowToHigh = 1,
}

impl From<GPIOInterruptLevel> for bool {
    fn from(f: GPIOInterruptLevel) -> bool {
        match f {
            GPIOInterruptLevel::HighToLow => false,
            GPIOInterruptLevel::LowToHigh => true,
        }
    }
}

impl From<bool> for GPIOInterruptLevel {
    fn from(b: bool) -> GPIOInterruptLevel {
        match b {
            false => GPIOInterruptLevel::HighToLow,
            true => GPIOInterruptLevel::LowToHigh,
        }
    }
}

gpio_register!(
    GPIOInterruptLevel1,
    GPIOInterruptLevel2,
    GPIOInterruptLevel3,
    GPIO_INTERRUPT_LEVEL,
    GPIOInterruptLevel
);

/// This is for pins configured as inputs. A bit value of ‘0’ in any of
/// the unreserved bits enables the debounce. This is the default value.
///
/// A bit value of ‘1’ disables the debounce.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GPIODebounce {
    Enabled = 0,
    Disabled = 1,
}

impl From<GPIODebounce> for bool {
    fn from(f: GPIODebounce) -> bool {
        match f {
            GPIODebounce::Enabled => false,
            GPIODebounce::Disabled => true,
        }
    }
}

impl From<bool> for GPIODebounce {
    fn from(b: bool) -> GPIODebounce {
        match b {
            false => GPIODebounce::Enabled,
            true => GPIODebounce::Disabled,
        }
    }
}

gpio_register!(
    DebounceDisable1,
    DebounceDisable2,
    DebounceDisable3,
    DEBOUNCE_DISABLE,
    GPIODebounce
);

/// This register enables or disables pull-up registers from inputs.
///
/// A bit value of '0' will enable the internal pull-up resistors. This is the default value.
///
/// A bit value of 1 will disable the internal pull-up resistors.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GPIOPullUp {
    Enabled = 0,
    Disabled = 1,
}

impl From<GPIOPullUp> for bool {
    fn from(f: GPIOPullUp) -> bool {
        match f {
            GPIOPullUp::Enabled => false,
            GPIOPullUp::Disabled => true,
        }
    }
}

impl From<bool> for GPIOPullUp {
    fn from(b: bool) -> GPIOPullUp {
        match b {
            false => GPIOPullUp::Enabled,
            true => GPIOPullUp::Disabled,
        }
    }
}

gpio_register!(
    GPIOPullUpDisable1,
    GPIOPullUpDisable2,
    GPIOPullUpDisable3,
    GPIO_PULL_UP_DISABLE,
    GPIOPullUp
);
