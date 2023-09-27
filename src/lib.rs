#![no_std]

use embedded_hal::i2c::I2c;
use embedded_hal::i2c::{Error, ErrorKind, ErrorType};


#[derive(Debug)]
pub enum IS31FL3733Error {
    I2CError(ErrorKind),
    StateError,
    DeviceError,
}

impl Error for IS31FL3733Error {
    fn kind(&self) -> ErrorKind {
        match self {
            IS31FL3733Error::I2CError(kind) => *kind,
            _ => ErrorKind::Other,
        }
    }
}

impl<BUS: I2c> ErrorType for IS31FL3733<BUS> {
    type Error = IS31FL3733Error;
}

pub struct IS31FL3733<BUS: I2c> {
    i2c: BUS,
    state: IS31FL3733State,
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

pub struct IS31FL3733State {
    page: u8,
    configuration_register: u8,
    leds_control: [u8; 0x18],
    leds_pwm: [u8; 192],
    global_current_control: u8,
}

impl Default for IS31FL3733State {
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

impl<BUS: I2c> IS31FL3733<BUS> {
    pub fn new(i2c: BUS, address: u8) -> Self {
        Self {
            i2c,
            address,
            state: IS31FL3733State::default(),
        }
    }

    pub fn init(&mut self) -> Result<(), IS31FL3733Error> {
        let reset_result = self.read_paged(RESET_REGISTER.page, RESET_REGISTER.register)?;
        if reset_result != 0x00 {
            return Err(IS31FL3733Error::DeviceError);
        }
        self.set_configuration(CONFIGURATION_SOFTWARE_SHUTDOWN_DISABLE)?;
        self.set_page(self.state.page)?;
        
        Ok(())
    }

    pub fn set_global_current_control(&mut self, gcc: u8) -> Result<(), IS31FL3733Error> {
        if self.state.global_current_control != gcc {
            self.write_paged(GCC_REGISTER.page, GCC_REGISTER.register, gcc)?;
            self.state.global_current_control = gcc;
        }
        Ok(())
    }

    pub fn set_configuration(&mut self, configuration: u8) -> Result<(), IS31FL3733Error> {
        if self.state.configuration_register != configuration {
            self.write_paged(
                CONFIGURATION_REGISTER.page,
                CONFIGURATION_REGISTER.register,
                configuration,
            )?;
            self.state.configuration_register = configuration;
        }
        Ok(())
    }

    pub fn unlock(&mut self) -> Result<(), IS31FL3733Error> {
        self.write_register(COMMAND_WRITE_LOCK_REGISTER, COMMAND_WRITE_UNLOCK)?;
        Ok(())
    }

    pub fn set_page(&mut self, page: u8) -> Result<(), IS31FL3733Error> {
        if page != self.state.page {
            self.unlock()?;
            self.write_register(COMMAND_REGISTER, page)?;

            self.state.page = page;
        }
        Ok(())
    }

    pub fn apply_pwm(&mut self, new_pwms: &[u8; 192]) -> Result<(), IS31FL3733Error> {
        Ok(())

    }

    pub fn set_pwm(&mut self, pwms: &[u8; 192]) -> Result<(), IS31FL3733Error> {
        let mut buffer: [u8; 193] = [0; 193];

        buffer[0] = PWM_REGISTER_BASE.register;
        buffer[1..].copy_from_slice(pwms);

        self.set_page(PWM_REGISTER_BASE.page)?;
        self.i2c
            .write(self.address, &buffer)
            .map_err(|e| IS31FL3733Error::I2CError(e.kind()))?;

        self.state.leds_pwm.copy_from_slice(pwms);

        Ok(())
    }

    pub fn apply_leds(&mut self, leds: &[u8; 0x18]) -> Result<(), IS31FL3733Error> {
        let mut buffer: heapless::Vec<u8, 25> = heapless::Vec::new();
        let mut cursor: u8 = LED_CONTROL_REGISTER_BASE.register;

        buffer.push(cursor).unwrap();

        self.set_page(LED_CONTROL_REGISTER_BASE.page)?;

        for i in 0..0x18 {
            let old = self.state.leds_control[i];
            let new = leds[i];

            if old == new {
                if buffer.len() > 1 {
                    // Flush buffer
                    buffer[0] = cursor;
                    self.i2c
                        .write(self.address, &buffer.as_slice())
                        .map_err(|e| IS31FL3733Error::I2CError(e.kind()))?;

                    cursor = cursor.checked_add(buffer.len() as u8).ok_or(IS31FL3733Error::StateError)?;
                    buffer.clear();
                    buffer.push(cursor)
                        .map_err(|_| IS31FL3733Error::StateError)?;
                } else {
                    cursor = cursor.checked_add(1).ok_or(IS31FL3733Error::StateError)?;
                }
            } else {
                buffer.push(new).map_err(|_| IS31FL3733Error::StateError)?;
            }
        }

        // If leftovers in buffer
        if buffer.len() > 1 {
            buffer[0] = cursor;
            self.i2c
                .write(self.address, &buffer.as_slice())
                .map_err(|e| IS31FL3733Error::I2CError(e.kind()))?;
        }

        Ok(())
    }

    pub fn set_leds(&mut self, leds: &[u8; 0x18]) -> Result<(), IS31FL3733Error> {
        // Non diffing implementation
        let mut buffer: [u8; 25] = [0; 25];
        buffer[0] = LED_CONTROL_REGISTER_BASE.register;
        buffer[1..].copy_from_slice(leds);

        self.set_page(LED_CONTROL_REGISTER_BASE.page)?;
        self.i2c
            .write(self.address, &buffer)
            .map_err(|e| IS31FL3733Error::I2CError(e.kind()))?;

        self.state.leds_control.copy_from_slice(leds);

        Ok(())
    }

    pub fn turn_on_led(&mut self, led: u8) -> Result<(), IS31FL3733Error> {
        let led_index = led / 8;
        let led_bit = led % 8;
        self.state.leds_control[led_index as usize] |= 1 << led_bit;
        self.write_paged(
            LED_CONTROL_REGISTER_BASE.page,
            LED_CONTROL_REGISTER_BASE.register + led_index,
            self.state.leds_control[led_index as usize],
        )?;

        Ok(())
    }

    pub fn turn_off_led(&mut self, led: u8) -> Result<(), IS31FL3733Error> {
        let led_index = led / 8;
        let led_bit = led % 8;
        self.state.leds_control[led_index as usize] &= !(1 << led_bit);
        self.write_paged(
            LED_CONTROL_REGISTER_BASE.page,
            LED_CONTROL_REGISTER_BASE.register + led_index,
            self.state.leds_control[led_index as usize],
        )?;

        Ok(())
    }

    pub fn set_led_pwm(&mut self, led: u8, pwm: u8) -> Result<(), IS31FL3733Error> {
        self.state.leds_pwm[led as usize] = pwm;
        self.write_paged(
            PWM_REGISTER_BASE.page,
            PWM_REGISTER_BASE.register + led,
            pwm,
        )?;
        Ok(())
    }

    pub fn read_paged(&mut self, page: u8, address: u8) -> Result<u8, IS31FL3733Error> {
        self.set_page(page)?;
        Ok(self.read_register(address)?)
    }

    pub fn write_paged(&mut self, page: u8, address: u8, value: u8) -> Result<(), IS31FL3733Error> {
        self.set_page(page)?;
        self.write_register(address, value)?;
        Ok(())
    }

    pub fn write_register(&mut self, address: u8, value: u8) -> Result<(), IS31FL3733Error> {
        self.i2c
            .write(self.address, &[address, value])
            .map_err(|e| IS31FL3733Error::I2CError(e.kind()))?;
        Ok(())
    }

    pub fn read_register(&mut self, address: u8) -> Result<u8, IS31FL3733Error> {
        let mut buffer = [0; 1];
        self.i2c
            .write_read(self.address, &[address], &mut buffer)
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

    impl<const N: usize, const M: usize> embedded_hal::i2c::I2c for FakeI2cBus<N, M> {
        fn transaction(
            &mut self,
            address: embedded_hal::i2c::SevenBitAddress,
            operations: &mut [embedded_hal::i2c::Operation],
        ) -> Result<(), Self::Error> {
            for operation in operations {
                match operation {
                    embedded_hal::i2c::Operation::Write(write) => {
                        self.write_data
                            .extend_from_slice(write)
                            .map_err(|_| FakeI2cError::Error)?;
                    }
                    embedded_hal::i2c::Operation::Read(read) => {
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

    #[test]
    fn init_test() {
        const EXPECTED_WRITE_DATA: [u8; 7] = [0xfe, 0xc5, 0xfd, 0x03, 0x11, 0x00, 0x01];
        const EXPECTED_READ_DATA: [u8; 1] = [0];

        let mut bus = FakeI2cBus::<32, 32>::new_with_read_data(&EXPECTED_READ_DATA);

        let mut is31fl3733 = IS31FL3733::new(&mut bus, 0x60);
        is31fl3733.init().unwrap();

        assert_eq!(bus.into_write_slice(), EXPECTED_WRITE_DATA);
    }

    #[test]
    fn configuration_test() {
        const EXPECTED_WRITE_DATA: &[u8] = &[0xfe, 0xc5, 0xfd, 0x03, 0x00, 0xaa, 0x00, 0xab];

        let mut bus = FakeI2cBus::<32, 32>::new();

        let mut is31fl3733 = IS31FL3733::new(&mut bus, 0x60);
        is31fl3733.set_configuration(0xaa).unwrap();
        is31fl3733.set_configuration(0xaa).unwrap();
        is31fl3733.set_configuration(0xab).unwrap();

        assert_eq!(bus.into_write_slice(), EXPECTED_WRITE_DATA);
    }

    #[test]
    fn global_current_control_test() {
        const EXPECTED_WRITE_DATA: &[u8] = &[0xfe, 0xc5, 0xfd, 0x03, 0x01, 0xaa, 0x01, 0xab];

        let mut bus = FakeI2cBus::<32, 32>::new();

        let mut is31fl3733 = IS31FL3733::new(&mut bus, 0x60);

        is31fl3733.set_global_current_control(0xaa).unwrap();
        is31fl3733.set_global_current_control(0xaa).unwrap();
        is31fl3733.set_global_current_control(0xab).unwrap();

        assert_eq!(bus.into_write_slice(), EXPECTED_WRITE_DATA);
    }

    #[test]
    fn led_full_apply_test() {
        const EXPECTED_WRITE_DATA: &[u8] = &[
            0x00,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
            0x00,
            0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 
        ];

        let mut bus = FakeI2cBus::<94, 32>::new();

        let mut is31fl3733 = IS31FL3733::new(&mut bus, 0x60);


        is31fl3733.set_leds(&[0xff; 0x18]).unwrap();
        is31fl3733.apply_leds(&[0xaa; 0x18]).unwrap();
        assert_eq!(bus.into_write_slice(), EXPECTED_WRITE_DATA);
    }

    #[test]
    fn led_striped_apply_test() {
        const EXPECTED_WRITE_DATA: &[u8] = &[
            0x00,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
            0x01,
            0xaa, 0xaa, 0xaa,
            0x08,
            0xaa,
            0x13,
            0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
        ];

        let mut bus = FakeI2cBus::<94, 32>::new();

        let mut is31fl3733 = IS31FL3733::new(&mut bus, 0x60);

        let mut leds_state = [0xff; 0x18];

        is31fl3733.set_leds(&leds_state).unwrap();
        leds_state[1] = 0xaa;
        leds_state[2] = 0xaa;
        leds_state[3] = 0xaa;
        leds_state[8] = 0xaa;
        leds_state[19..].copy_from_slice(&[0xaa; 5]);
        is31fl3733.apply_leds(&leds_state).unwrap();
        assert_eq!(bus.into_write_slice(), EXPECTED_WRITE_DATA);
    }
    #[test]
    fn led_start_apply_test() {
        const EXPECTED_WRITE_DATA: &[u8] = &[
            0x00,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
            0x00,
            0xaa, 0xaa,
        ];

        let mut bus = FakeI2cBus::<94, 32>::new();

        let mut is31fl3733 = IS31FL3733::new(&mut bus, 0x60);

        let mut leds_state = [0xff; 0x18];

        is31fl3733.set_leds(&leds_state).unwrap();
        leds_state[..2].copy_from_slice(&[0xaa; 2]);
        is31fl3733.apply_leds(&leds_state).unwrap();
        assert_eq!(bus.into_write_slice(), EXPECTED_WRITE_DATA);
    }

    #[test]
    fn led_middle_apply_test() {
        const EXPECTED_WRITE_DATA: &[u8] = &[
            0x00,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
            0x08, 0xaa, 0xaa

        ];

        let mut bus = FakeI2cBus::<94, 32>::new();

        let mut is31fl3733 = IS31FL3733::new(&mut bus, 0x60);

        let mut leds_state = [0xff; 0x18];

        is31fl3733.set_leds(&leds_state).unwrap();
        is31fl3733.apply_leds(&leds_state).unwrap();
        leds_state[8..10].copy_from_slice(&[0xaa; 2]);
        is31fl3733.apply_leds(&leds_state).unwrap();
        assert_eq!(bus.into_write_slice(), EXPECTED_WRITE_DATA);
    }

    #[test]
    fn led_control_test() {
        const EXPECTED_WRITE_DATA: &[u8] = &[
            0xfe, 0xc5, 0xfd, 0x02, 0xfe, 0xc5, 0xfd, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0xfe, 0x00, 0xff, 0x0a, 0xdf, 0x0a, 0xff,
        ];

        let mut bus = FakeI2cBus::<64, 32>::new();

        let mut is31fl3733 = IS31FL3733::new(&mut bus, 0x60);

        is31fl3733.set_page(2);
        is31fl3733.set_leds(&[0xff; 0x18]).unwrap();
        is31fl3733.turn_off_led(0).unwrap();
        is31fl3733.turn_on_led(0).unwrap();

        is31fl3733.turn_off_led(85).unwrap();
        is31fl3733.turn_on_led(85).unwrap();

        assert_eq!(bus.into_write_slice(), EXPECTED_WRITE_DATA);
    }

    #[test]
    fn led_pwm_test() {
        const EXPECTED_WRITE_DATA: &[u8] = &[
            0xfe, 0xc5, 0xfd, 0x01, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xbf, 0xaa
        ];

        let mut bus = FakeI2cBus::<{192*2}, 32>::new();

        let mut is31fl3733 = IS31FL3733::new(&mut bus, 0x60);

        is31fl3733.set_pwm(&[0xff; 192]).unwrap();
        is31fl3733.set_led_pwm(191, 0xaa).unwrap();

        assert_eq!(bus.into_write_slice(), EXPECTED_WRITE_DATA);
    }
}
