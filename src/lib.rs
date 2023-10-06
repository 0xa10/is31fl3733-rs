#![no_std]
#![feature(async_fn_in_trait)]

use embedded_hal_async::i2c::{Error, ErrorKind, ErrorType};

#[derive(Debug)]
pub enum IS31FL3733Error {
    I2CError(ErrorKind),
    StateError,
    DeviceError,
    OutOfSpaceError,
}

impl Error for IS31FL3733Error {
    fn kind(&self) -> ErrorKind {
        match self {
            IS31FL3733Error::I2CError(kind) => *kind,
            _ => ErrorKind::Other,
        }
    }
}

impl<BUS: embedded_hal_async::i2c::I2c> ErrorType for IS31FL3733<BUS> {
    type Error = IS31FL3733Error;
}

pub struct IS31FL3733<BUS: embedded_hal_async::i2c::I2c> {
    i2c: BUS,
    state: State,
    address: u8,
}

const COMMAND_REGISTER: u8 = 0xfd;
const COMMAND_WRITE_LOCK_REGISTER: u8 = 0xfe;
const COMMAND_WRITE_UNLOCK: u8 = 0xc5;
const INTERRUPT_MASK_REGISTER: u8 = 0xf0;
const INTERRUPT_STATUS_REGISTER: u8 = 0xf1;

struct PagedRegister {
    page: u8,
    register: u8,
}

const LED_CONTROL_REGISTER_BASE: PagedRegister = PagedRegister {
    page: 0x00,
    register: 0x00,
};
const LED_OPEN_REGISTER_BASE: PagedRegister = PagedRegister {
    page: 0x00,
    register: 0x18,
};
const LED_SHORT_REGISTER_BASE: PagedRegister = PagedRegister {
    page: 0x00,
    register: 0x30,
};

const PWM_REGISTER_BASE: PagedRegister = PagedRegister {
    page: 0x01,
    register: 0x00,
};

const AUTO_BREATH_MODE_REGISTER_BASE: PagedRegister = PagedRegister {
    page: 0x02,
    register: 0x00,
};

const CONFIGURATION_REGISTER: PagedRegister = PagedRegister {
    page: 0x03,
    register: 0x00,
};
const GCC_REGISTER: PagedRegister = PagedRegister {
    page: 0x03,
    register: 0x01,
};
const RESET_REGISTER: PagedRegister = PagedRegister {
    page: 0x03,
    register: 0x11,
};

const CONFIGURATION_SYNC_HIGH_IMPEDANCE: u8 = 0b0000_0000;
const CONFIGURATION_SYNC_HIGH_IMPEDANCE_ALTERNATE: u8 = 0b0110_0000;
const CONFIGURATION_SYNC_MASTER: u8 = 0b0010_0000;
const CONFIGURATION_SYNC_SLAVE: u8 = 0b0100_0000;
const CONFIGURATION_OSD_ENABLE: u8 = 0b0000_0100;
const CONFIGURATION_AUTO_BREATH_MODE_ENABLE: u8 = 0b0000_010;
const CONFIGURATION_SOFTWARE_SHUTDOWN_DISABLE: u8 = 0b0000_001;

pub struct State {
    page: u8,
    configuration_register: u8,
    leds_control: [u8; 0x18],
    leds_pwm: [u8; 192],
    global_current_control: u8,
}

impl Default for State {
    // This reflect the presumed default state after a reset
    fn default() -> Self {
        Self {
            page: 0,
            configuration_register: 0,
            leds_control: [0; 0x18],
            leds_pwm: [0; 192],
            global_current_control: 0,
        }
    }
}

const LED_COUNT: usize = 192;
impl<BUS: embedded_hal_async::i2c::I2c> IS31FL3733<BUS> {
    pub fn new(i2c: BUS, address: u8) -> Self {
        Self {
            i2c,
            address,
            state: State::default(),
        }
    }

    pub async fn init(&mut self) -> Result<(), IS31FL3733Error> {
        self.set_page(RESET_REGISTER.page).await?;
        let reset_result = self.read_register(RESET_REGISTER.register).await?;

        if reset_result != 0x00 {
            return Err(IS31FL3733Error::DeviceError);
        }
        self.set_configuration(CONFIGURATION_SOFTWARE_SHUTDOWN_DISABLE)
            .await?;
        self.set_page(self.state.page).await?;

        Ok(())
    }

    pub async fn set_global_current_control(&mut self, gcc: u8) -> Result<(), IS31FL3733Error> {
        if self.state.global_current_control != gcc {
            self.set_page(GCC_REGISTER.page).await?;
            self.write_register(GCC_REGISTER.register, gcc).await?;
            self.state.global_current_control = gcc;
        }
        Ok(())
    }

    pub async fn set_configuration(&mut self, configuration: u8) -> Result<(), IS31FL3733Error> {
        if self.state.configuration_register != configuration {
            self.set_page(CONFIGURATION_REGISTER.page).await?;
            self.write_register(CONFIGURATION_REGISTER.register, configuration)
                .await?;
            self.state.configuration_register = configuration;
        }
        Ok(())
    }

    pub async fn unlock(&mut self) -> Result<(), IS31FL3733Error> {
        self.write_register(COMMAND_WRITE_LOCK_REGISTER, COMMAND_WRITE_UNLOCK)
            .await?;
        Ok(())
    }

    pub async fn set_page(&mut self, page: u8) -> Result<(), IS31FL3733Error> {
        if page != self.state.page {
            self.unlock().await?;
            self.write_register(COMMAND_REGISTER, page).await?;

            self.state.page = page;
        }
        Ok(())
    }

    pub async fn set_led_brightness(
        &mut self,
        led: u8,
        brightness: u8,
    ) -> Result<(), IS31FL3733Error> {
        let old = self.state.leds_pwm[led as usize];

        if old != brightness {
            self.set_page(PWM_REGISTER_BASE.page).await?;
            self.write_register(PWM_REGISTER_BASE.register + led, brightness)
                .await?;

            self.state.leds_pwm[led as usize] = brightness;
        }

        Ok(())
    }

    pub async fn led_on(&mut self, led: u8) -> Result<(), IS31FL3733Error> {
        self.set_led(led, true).await?;
        Ok(())
    }

    pub async fn led_off(&mut self, led: u8) -> Result<(), IS31FL3733Error> {
        self.set_led(led, false).await?;
        Ok(())
    }

    pub async fn set_led(&mut self, led: u8, on: bool) -> Result<(), IS31FL3733Error> {
        let led_cell = self.state.leds_control[led as usize / 8];
        let led_bit = 1 << (led % 8);

        let new_led_cell = if on {
            led_cell | led_bit
        } else {
            led_cell & !led_bit
        };

        if new_led_cell != led_cell {
            self.set_page(LED_CONTROL_REGISTER_BASE.page).await?;
            self.write_register(
                LED_CONTROL_REGISTER_BASE.register + led as u8 / 8,
                new_led_cell,
            )
            .await?;

            self.state.leds_control[led as usize / 8] = new_led_cell;
        }

        Ok(())
    }

    pub async fn set_leds(&mut self, leds: &[u8; LED_COUNT / 8]) -> Result<(), IS31FL3733Error> {
        // TODO - figure out how to avoid this copy
        let old = self.state.leds_control;

        self.set_page(LED_CONTROL_REGISTER_BASE.page).await?;
        self.diff_write(&old, leds).await?;

        self.state.leds_control.copy_from_slice(leds);

        Ok(())
    }

    pub async fn set_pwm(&mut self, pwm: &[u8; LED_COUNT]) -> Result<(), IS31FL3733Error> {
        // TODO - figure out how to avoid this copy
        let old = self.state.leds_pwm;

        self.set_page(PWM_REGISTER_BASE.page).await?;
        self.diff_write(&old, pwm).await?;

        self.state.leds_pwm.copy_from_slice(pwm);

        Ok(())
    }

    async fn diff_write(&mut self, old: &[u8], new: &[u8]) -> Result<(), IS31FL3733Error> {
        let mut buffer: heapless::Vec<u8, LED_COUNT> = heapless::Vec::new();
        let mut cursor: u8 = 0;

        if old.len() != new.len() {
            return Err(IS31FL3733Error::StateError);
        }

        // The strategy here is to try and find "runs" in the buffer which
        // require changes, and write only those.
        // We are constrained by the maximum size of our buffer.
        for index in 0..old.len() {
            let old_value = old[index];
            let new_value = new[index];

            if old_value == new_value {
                // If the values match, we have either finished a run and need
                // to flush the buffered data.
                if buffer.len() > 0 {
                    // Flush buffer
                    self.write_buffer::<{ LED_COUNT + 1 }>(cursor, buffer.as_slice())
                        .await?;
                    buffer.clear();
                    // Advance cursor to current position
                    cursor = index as u8;
                }
                cursor = cursor.checked_add(1).ok_or(IS31FL3733Error::StateError)?;
            } else {
                buffer
                    .push(new_value)
                    .map_err(|_| IS31FL3733Error::StateError)?;
            }
        }

        // If any remainder is left in the buffer, flush it
        if buffer.len() > 0 {
            self.write_buffer::<{ LED_COUNT + 1 }>(cursor, buffer.as_slice())
                .await?;
        }

        Ok(())
    }

    pub async fn write_buffer<const N: usize>(
        &mut self,
        address: u8,
        data: &[u8],
    ) -> Result<(), IS31FL3733Error> {
        // N expresses the maximum write size
        if data.len() > N - 1 {
            return Err(IS31FL3733Error::OutOfSpaceError);
        }

        let mut buffer: heapless::Vec<u8, N> = heapless::Vec::new();
        buffer
            .push(address)
            .map_err(|_| IS31FL3733Error::OutOfSpaceError)?;
        buffer
            .extend_from_slice(data)
            .map_err(|_| IS31FL3733Error::OutOfSpaceError)?;

        self.i2c
            .write(self.address, &buffer.as_slice())
            .await
            .map_err(|e| IS31FL3733Error::I2CError(e.kind()))?;

        Ok(())
    }

    pub async fn write_register(&mut self, address: u8, value: u8) -> Result<(), IS31FL3733Error> {
        self.i2c
            .write(self.address, &[address, value])
            .await
            .map_err(|e| IS31FL3733Error::I2CError(e.kind()))?;
        Ok(())
    }

    pub async fn read_register(&mut self, address: u8) -> Result<u8, IS31FL3733Error> {
        let mut buffer = [0; 1];

        self.i2c
            .write_read(self.address, &[address], &mut buffer)
            .await
            .map_err(|e| IS31FL3733Error::I2CError(e.kind()))?;
        Ok(buffer[0])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Debug)]
    enum FakeI2cError {
        Error,
    }
    impl Error for FakeI2cError {
        fn kind(&self) -> ErrorKind {
            ErrorKind::Other
        }
    }

    struct FakeI2cBus<const N: usize, const M: usize> {
        pub write_data: heapless::Vec<u8, N>,
        pub read_data: heapless::Vec<u8, M>,
    }

    impl<const N: usize, const M: usize> ErrorType for FakeI2cBus<N, M> {
        type Error = FakeI2cError;
    }

    impl<const N: usize, const M: usize> FakeI2cBus<N, M> {
        pub fn new() -> Self {
            Self {
                write_data: heapless::Vec::new(),
                read_data: heapless::Vec::new(),
            }
        }

        pub fn new_with_read_data(read_data: &[u8]) -> Self {
            Self {
                write_data: heapless::Vec::new(),
                read_data: heapless::Vec::from_slice(read_data).unwrap(),
            }
        }

        pub fn into_write_slice(&self) -> &[u8] {
            self.write_data.as_slice()
        }

        pub fn into_read_slice(&self) -> &[u8] {
            self.read_data.as_slice()
        }
    }

    impl<const N: usize, const M: usize> embedded_hal_async::i2c::I2c for FakeI2cBus<N, M> {
        async fn transaction(
            &mut self,
            address: embedded_hal_async::i2c::SevenBitAddress,
            operations: &mut [embedded_hal_async::i2c::Operation<'_>],
        ) -> Result<(), Self::Error> {
            for operation in operations {
                match operation {
                    embedded_hal_async::i2c::Operation::Write(write) => {
                        self.write_data
                            .extend_from_slice(write)
                            .map_err(|_| FakeI2cError::Error)?;
                    }
                    embedded_hal_async::i2c::Operation::Read(read) => {
                        // Copy read.len() bytes from read_data to read and remove those bytes
                        for i in 0..read.len() {
                            read[i] = self.read_data.remove(i);
                        }
                    }
                }
            }
            Ok(())
        }
    }

    use async_test_lite::async_test;
    #[async_test]
    async fn init_test() {
        const EXPECTED_WRITE_DATA: [u8; 7] = [0xfe, 0xc5, 0xfd, 0x03, 0x11, 0x00, 0x01];
        const EXPECTED_READ_DATA: [u8; 1] = [0];

        let mut bus = FakeI2cBus::<32, 32>::new_with_read_data(&EXPECTED_READ_DATA);

        let mut is31fl3733 = IS31FL3733::new(&mut bus, 0x60);
        is31fl3733.init().await.unwrap();

        assert_eq!(bus.into_write_slice(), EXPECTED_WRITE_DATA);
    }

    #[async_test]
    async fn configuration_test() {
        const EXPECTED_WRITE_DATA: &[u8] = &[0xfe, 0xc5, 0xfd, 0x03, 0x00, 0xaa, 0x00, 0xab];

        let mut bus = FakeI2cBus::<32, 32>::new();

        let mut is31fl3733 = IS31FL3733::new(&mut bus, 0x60);
        is31fl3733.set_configuration(0xaa).await.unwrap();
        is31fl3733.set_configuration(0xaa).await.unwrap();
        is31fl3733.set_configuration(0xab).await.unwrap();

        assert_eq!(bus.into_write_slice(), EXPECTED_WRITE_DATA);
    }

    #[async_test]
    async fn global_current_control_test() {
        const EXPECTED_WRITE_DATA: &[u8] = &[0xfe, 0xc5, 0xfd, 0x03, 0x01, 0xaa, 0x01, 0xab];

        let mut bus = FakeI2cBus::<32, 32>::new();

        let mut is31fl3733 = IS31FL3733::new(&mut bus, 0x60);

        is31fl3733.set_global_current_control(0xaa).await.unwrap();
        is31fl3733.set_global_current_control(0xaa).await.unwrap();
        is31fl3733.set_global_current_control(0xab).await.unwrap();

        assert_eq!(bus.into_write_slice(), EXPECTED_WRITE_DATA);
    }

    #[async_test]
    async fn led_full_apply_test() {
        const EXPECTED_WRITE_DATA: &[u8] = &[
            0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0xaa, 0xaa,
            0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
            0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
        ];

        let mut bus = FakeI2cBus::<94, 32>::new();

        let mut is31fl3733 = IS31FL3733::new(&mut bus, 0x60);

        is31fl3733.set_leds(&[0xff; 0x18]).await.unwrap();
        is31fl3733.set_leds(&[0xaa; 0x18]).await.unwrap();
        assert_eq!(bus.into_write_slice(), EXPECTED_WRITE_DATA);
    }

    #[async_test]
    async fn led_striped_apply_test() {
        const EXPECTED_WRITE_DATA: &[u8] = &[
            0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01, 0xaa, 0xaa,
            0xaa, 0x08, 0xaa, 0x13, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
        ];

        let mut bus = FakeI2cBus::<94, 32>::new();

        let mut is31fl3733 = IS31FL3733::new(&mut bus, 0x60);

        let mut leds_state = [0xff; 0x18];

        is31fl3733.set_leds(&leds_state).await.unwrap();
        leds_state[1] = 0xaa;
        leds_state[2] = 0xaa;
        leds_state[3] = 0xaa;
        leds_state[8] = 0xaa;
        leds_state[19..].copy_from_slice(&[0xaa; 5]);
        is31fl3733.set_leds(&leds_state).await.unwrap();
        assert_eq!(bus.into_write_slice(), EXPECTED_WRITE_DATA);
    }
    #[async_test]
    async fn led_start_apply_test() {
        const EXPECTED_WRITE_DATA: &[u8] = &[
            0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0xaa, 0xaa,
        ];

        let mut bus = FakeI2cBus::<94, 32>::new();

        let mut is31fl3733 = IS31FL3733::new(&mut bus, 0x60);

        let mut leds_state = [0xff; 0x18];

        is31fl3733.set_leds(&leds_state).await.unwrap();
        leds_state[..2].copy_from_slice(&[0xaa; 2]);
        is31fl3733.set_leds(&leds_state).await.unwrap();
        assert_eq!(bus.into_write_slice(), EXPECTED_WRITE_DATA);
    }

    #[async_test]
    async fn led_middle_apply_test() {
        const EXPECTED_WRITE_DATA: &[u8] = &[
            0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x08, 0xaa, 0xaa,
        ];

        let mut bus = FakeI2cBus::<94, 32>::new();

        let mut is31fl3733 = IS31FL3733::new(&mut bus, 0x60);

        let mut leds_state = [0xff; 0x18];

        is31fl3733.set_leds(&leds_state).await.unwrap();
        is31fl3733.set_leds(&leds_state).await.unwrap();
        leds_state[8..10].copy_from_slice(&[0xaa; 2]);
        is31fl3733.set_leds(&leds_state).await.unwrap();
        assert_eq!(bus.into_write_slice(), EXPECTED_WRITE_DATA);
    }

    #[async_test]
    async fn led_control_test() {
        const EXPECTED_WRITE_DATA: &[u8] = &[
            0xfe, 0xc5, 0xfd, 0x02, 0xfe, 0xc5, 0xfd, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0xfe, 0x00, 0xff, 0x0a, 0xdf, 0x0a, 0xff,
        ];

        let mut bus = FakeI2cBus::<64, 32>::new();

        let mut is31fl3733 = IS31FL3733::new(&mut bus, 0x60);

        is31fl3733.set_page(2).await.unwrap();
        is31fl3733.set_leds(&[0xff; 0x18]).await.unwrap();
        is31fl3733.led_off(0).await.unwrap();
        is31fl3733.led_on(0).await.unwrap();

        is31fl3733.led_off(85).await.unwrap();
        is31fl3733.led_on(85).await.unwrap();

        assert_eq!(bus.into_write_slice(), EXPECTED_WRITE_DATA);
    }

    #[async_test]
    async fn led_pwm_test() {
        const EXPECTED_WRITE_DATA: &[u8] = &[
            0xfe, 0xc5, 0xfd, 0x01, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xbf, 0xaa,
        ];

        let mut bus = FakeI2cBus::<{ 192 * 2 }, 32>::new();

        let mut is31fl3733 = IS31FL3733::new(&mut bus, 0x60);

        is31fl3733.set_pwm(&[0xff; 192]).await.unwrap();
        is31fl3733.set_led_brightness(191, 0xaa).await.unwrap();

        assert_eq!(bus.into_write_slice(), EXPECTED_WRITE_DATA);
    }
}
