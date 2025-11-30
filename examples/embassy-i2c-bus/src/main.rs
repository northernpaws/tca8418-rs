#![no_std]
#![no_main]

use defmt::{error, info};

use assign_resources::assign_resources;

use embassy_time::Delay;
use grounded::uninit::GroundedArrayCell;
use static_cell::StaticCell;

use embassy_executor::Spawner;

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};

use embassy_stm32::{
    Config, Peri, bind_interrupts,
    exti::ExtiInput,
    gpio::{Output, Pull},
    i2c::{self, I2c},
    mode::Async,
    peripherals,
    time::Hertz,
};

use embassy_embedded_hal::shared_bus::asynch;

use panic_probe as _;
use tca8418_rs::{
    Tca8418,
    register::{KEYPAD_GPIO1_ADDRESS, KEYPAD_GPIO2_ADDRESS},
};

assign_resources! {
    i2c4: I2C4Resources {
        peri: I2C4 = I2C4Peri,
        scl: PB8,
        sda: PB9,

        tx_dma: BDMA_CH1,
        rx_dma: BDMA_CH2,
    }

    tca8418: TCa8418Resources {
        int: PB0,
        exti: EXTI0,
    }
}

bind_interrupts!(pub struct Irqs {
    I2C4_EV => i2c::EventInterruptHandler<peripherals::I2C4>;
    I2C4_ER => i2c::ErrorInterruptHandler<peripherals::I2C4>;
});

/// Due to a quirk of the I2C4 peripheral on the STM32H743ZI only being abing accessible by the BDMA, which in turn
/// can only access RAM in the D3 region, we need to allocate memory for the buffers specifically in that region.
///
/// For other I2C peripherals this can be omitted.
///
/// see: https://github.com/embassy-rs/embassy/blob/abcb6e607c4f13bf99c406fbb92480c32ebd0d4a/docs/pages/faq.adoc#stm32-bdma-only-working-out-of-some-ram-regions
///
/// Defined in memory.x
#[unsafe(link_section = ".ram_d3")]
static mut RAM_D3_BUF: GroundedArrayCell<u8, 4> = GroundedArrayCell::uninit();

static I2C_BUS: StaticCell<Mutex<CriticalSectionRawMutex, I2c<'static, Async, i2c::Master>>> =
    StaticCell::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Initialize the RCC and PLLs with the default
    // settings, typically around 48MHz CPU clock.
    info!("Initializing RCC and PLLs...");
    let peripherals = embassy_stm32::init(Config::default());

    let r = split_resources!(peripherals);

    // Configure and initialize the I2C2 peripheral.
    info!("Initializing I2C peripheral...");
    let mut i2c_config: i2c::Config = Default::default();
    i2c_config.frequency = Hertz::khz(100);
    let i2c4 = I2c::new(
        r.i2c4.peri,
        r.i2c4.scl,
        r.i2c4.sda,
        Irqs,
        r.i2c4.tx_dma,
        r.i2c4.rx_dma,
        i2c_config,
    );

    // Initialize the I2C bus mutex with the peripheral instance.
    info!("Initializing I2C bus...");
    let i2c_bus = I2C_BUS.init(Mutex::new(i2c4));

    // Initialize the EXTI pin used by the TCA8418's
    // interrupt pin to detect interrupt events.
    let mut int = ExtiInput::new(r.tca8418.int, r.tca8418.exti, Pull::Up);

    // The I2C4 peripherial on STM32H7 requires the BDMA, and in
    // turn the BDMA can only access data in the D3 section of RAM.
    //
    // TODO: Move this outside the driver struct for portability.
    //  Instead, pass the buffer references into the constructor.
    let (write_buf, write_read_buf, read_buf) = unsafe {
        let ram = &mut *core::ptr::addr_of_mut!(RAM_D3_BUF);
        ram.initialize_all_copied(0);
        (
            ram.get_subslice_mut_unchecked(0, 2), // 0..1
            ram.get_subslice_mut_unchecked(2, 1), // 2
            ram.get_subslice_mut_unchecked(3, 1), // 3
        )
    };

    // Create an device handle referencing the I2C bus, and use it to initialize the TCA8418 driver.
    let mut driver = Tca8418::new(
        asynch::i2c::I2cDevice::new(i2c_bus),
        write_buf,
        write_read_buf,
        read_buf,
        None::<Output>,
        Delay,
    );

    // Initialize the TCA8418 by resetting it and setting
    // some common default configuration register values.
    info!("Initializing TAC8418...");
    driver.init().await.unwrap();

    // Next, we need to configure the TCA8418's registers the enable
    // the row and column pins that we're using in the matrix.
    info!("Configuring TAC8418...");

    // Enable rows 0-3 in the button matrix.
    driver
        .write_register_raw(KEYPAD_GPIO1_ADDRESS, 0b00001111)
        .await
        .unwrap();

    // Enable columns 0-3 in the button matrix.
    driver
        .write_register_raw(KEYPAD_GPIO2_ADDRESS, 0b00001111)
        .await
        .unwrap();

    // Start a loop listening for interrupt events from the TCA8418 and logging them.
    loop {
        // If the interrupt pin is high at the start of the loop
        // then we need to immediately try to process the FIFO.
        if !int.is_low() {
            // Wait for the TCA8418 to raise an interrupt.
            int.wait_for_falling_edge().await;
        } else {
            info!("Interrupt line was already low, uncleared FIFO from previous loop?")
        }

        // Process the FIFO buffer in a loop until there are no events left.
        loop {
            // Poll the FIFO for the key event currently at it's head.
            let Ok(result) = driver.poll_fifo().await else {
                error!("I2C communication error polling TCA8418 for events!");
                break;
            };

            // If there was no event then the FIFO is empty.
            let Some(event) = result else {
                break;
            };

            // Otherwise, we can process the received event.
            let keycode = event.key_index();
            if event.key_pressed() {
                info!("Key {} pressed!", keycode);
            } else {
                info!("Key {} released!", keycode);
            }
        }

        info!("TCA8418 FIFO cleared!");

        driver.clear_all_interrupts().await.unwrap();
    }
}
