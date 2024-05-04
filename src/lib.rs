mod sys;

macro_rules! rtlsdr_result {
    ($ret:expr) => {
        unsafe {
            if $ret < 0 {
                Err($ret)
            } else {
                Ok($ret)
            }
        }
    };
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum RtlsdrError {
    LibusbError(LibusbError),
    Unspecified(i32),
}

impl From<i32> for RtlsdrError {
    fn from(err: i32) -> Self {
        match LibusbError::try_from(err) {
            Ok(e) => RtlsdrError::LibusbError(e),
            Err(e) => RtlsdrError::Unspecified(e),
        }
    }
}

#[repr(i32)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum LibusbError {
    /// Input/output error
    IoError = -1,
    /// Invalid parameter
    InvalidParam = -2,
    /// Access denied (insufficient permissions)
    AccessDenied = -3,
    /// No such device (it may have been disconnected)
    NoDevice = -4,
    /// Entity not found
    NoEntity = -5,
    /// Resource busy
    Busy = -6,
    /// Operation timed out
    Timeout = -7,
    /// Overflow
    Overflow = -8,
    /// Pipe error
    Pipe = -9,
    /// System call interrupted (perhaps due to signal)
    Interrupted = -10,
    /// Insufficient memory
    InsufficientMemory = -11,
    /// Operation not supported or unimplemented on this platform
    NotSupported = -12,
    /// Other error
    Other = -99,
}

impl TryFrom<i32> for LibusbError {
    type Error = i32;

    fn try_from(value: i32) -> Result<Self, Self::Error> {
        match value {
            -1 => Ok(LibusbError::IoError),
            -2 => Ok(LibusbError::InvalidParam),
            -3 => Ok(LibusbError::AccessDenied),
            -4 => Ok(LibusbError::NoDevice),
            -5 => Ok(LibusbError::NoEntity),
            -6 => Ok(LibusbError::Busy),
            -7 => Ok(LibusbError::Timeout),
            -8 => Ok(LibusbError::Overflow),
            -9 => Ok(LibusbError::Pipe),
            -10 => Ok(LibusbError::Interrupted),
            -11 => Ok(LibusbError::InsufficientMemory),
            -12 => Ok(LibusbError::NotSupported),
            -99 => Ok(LibusbError::Other),
            e => Err(e),
        }
    }
}

#[repr(i32)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum TunerType {
    /// Unknown tuner type
    Unknown = 0,
    /// Elonics E4000 tuner
    E4000 = 1,
    /// FC0012 tuner
    FC0012 = 2,
    /// FC0013 tuner
    FC0013 = 3,
    /// FC2580 tuner
    FC2580 = 4,
    /// Realtek 820T tuner
    R820T = 5,
    /// Realtek 828D tuner
    R828D = 6,
}

#[repr(i32)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Sideband {
    Lower = 0,
    Upper = 1,
}

#[repr(u32)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum DirectSampling {
    Disabled = 0,
    I = 1,
    Q = 2,
}

#[repr(u32)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum DirectSamplingThreshold {
    Disabled = 0,
    I = 1,
    Q = 2,
    IBelow = 3,
    QBelow = 4,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
/// Device struct
pub struct Device {
    index: u32,
    dev: *mut sys::rtlsdr_dev,
}

impl Device {
    /// Open device
    /// This may fail due to a libusb error or some other unspecified error
    pub fn open(&mut self) -> Result<(), RtlsdrError> {
        rtlsdr_result!(sys::rtlsdr_open(&mut self.dev, self.index))?;

        Ok(())
    }

    // Close device
    // This is called automatically when the Device is dropped
    // This will return an error if the device is not open or already closed
    pub fn close(&mut self) -> Result<(), String> {
        rtlsdr_result!(sys::rtlsdr_close(self.dev)).map_err(|e| match e {
            -1 => "Device was not opened or already closed".to_string(),
            _ => format!("Failed to close device: {}", e),
        })?;

        self.dev = std::ptr::null_mut();

        Ok(())
    }

    /// Get crystal oscillator frequencies used for the RTL2832 and the tuner IC
    /// Usually both ICs use the same clock.
    pub fn get_xtal_freq(&self) -> Result<(u32, u32), String> {
        let mut rtl_freq = 0;
        let mut tuner_freq = 0;

        rtlsdr_result!(sys::rtlsdr_get_xtal_freq(
            self.dev,
            &mut rtl_freq,
            &mut tuner_freq
        ))
        .map_err(|e| format!("Failed to get crystal frequency: {}", e))?;

        Ok((rtl_freq, tuner_freq))
    }

    /// Set crystal oscillator frequencies used for the RTL2832 and the tuner IC.
    /// Usually both ICs use the same clock.
    /// Changing the clock may make sense if you are applying an external clock to the tuner
    /// or to compensate the frequency (and samplerate) error caused by the original (cheap) crystal.
    /// NOTE: Call this function only if you fully understand the implications.
    pub fn set_xtal_freq(&mut self, rtl_freq: u32, tuner_freq: u32) -> Result<(), String> {
        rtlsdr_result!(sys::rtlsdr_set_xtal_freq(self.dev, rtl_freq, tuner_freq))
            .map_err(|e| format!("Failed to set crystal frequency: {}", e))?;

        Ok(())
    }

    /// Get USB device strings.
    /// @return (manufacturer, product, serial) strings
    pub fn get_usb_device_strings(&self) -> Result<(String, String, String), String> {
        let mut manufact = [0u8; 256];
        let mut product = [0u8; 256];
        let mut serial = [0u8; 256];

        rtlsdr_result!(sys::rtlsdr_get_usb_strings(
            self.dev,
            manufact.as_mut_ptr() as *mut i8,
            product.as_mut_ptr() as *mut i8,
            serial.as_mut_ptr() as *mut i8
        ))
        .map_err(|e| format!("Failed to get usb device strings: {}", e))?;

        let manufact = std::ffi::CStr::from_bytes_until_nul(&manufact)
            .map_err(|e| format!("Failed to get usb device strings: {}", e))?
            .to_str()
            .expect("Failed to convert usb device string to str")
            .to_string();
        let product = std::ffi::CStr::from_bytes_until_nul(&product)
            .map_err(|e| format!("Failed to get usb device strings: {}", e))?
            .to_str()
            .expect("Failed to convert usb device string to str")
            .to_string();
        let serial = std::ffi::CStr::from_bytes_until_nul(&serial)
            .map_err(|e| format!("Failed to get usb device strings: {}", e))?
            .to_str()
            .expect("Failed to convert usb device string to str")
            .to_owned();

        Ok((manufact, product, serial))
    }

    /// Read the device EEPROM
    pub fn read_eeprom(&self, offset: u8, len: u16) -> Result<Vec<u8>, String> {
        let mut buf = vec![0u8; len as usize];

        rtlsdr_result!(sys::rtlsdr_read_eeprom(
            self.dev,
            buf.as_mut_ptr(),
            offset,
            len
        ))
        .map_err(|e| match e {
            -1 => "Invalid Device".to_string(),
            -2 => "EEPROM size exceeded".to_string(),
            -3 => "No EEPROM found".to_string(),
            _ => format!("Failed to read EEPROM: {}", e),
        })?;

        Ok(buf)
    }

    /// Write the device EEPROM
    pub fn write_eeprom(&mut self, offset: u8, buf: &mut [u8]) -> Result<(), String> {
        rtlsdr_result!(sys::rtlsdr_write_eeprom(
            self.dev,
            buf.as_mut_ptr(),
            offset,
            buf.len() as u16
        ))
        .map_err(|e| match e {
            -1 => "Invalid Device".to_string(),
            -2 => "EEPROM size exceeded".to_string(),
            -3 => "No EEPROM found".to_string(),
            _ => format!("Failed to write EEPROM: {}", e),
        })?;

        Ok(())
    }

    /// Get actual frequency the device is tuned to in Hz
    pub fn get_center_freq(&self) -> Result<u32, String> {
        match unsafe { sys::rtlsdr_get_center_freq(self.dev) } {
            0 => Err("Failed to get center frequency".to_string()),
            freq => Ok(freq),
        }
    }

    /// Set the frequency the device is tuned to in Hz
    pub fn set_center_freq(&mut self, freq: u32) -> Result<(), String> {
        rtlsdr_result!(sys::rtlsdr_set_center_freq(self.dev, freq))
            .map_err(|e| format!("Failed to set center frequency: {}", e))?;

        Ok(())
    }

    /// Get actual frequency correction value of the device.
    /// @return correction value in parts per million (ppm)
    pub fn get_freq_correction(&self) -> i32 {
        unsafe { sys::rtlsdr_get_freq_correction(self.dev) }
    }

    /// Set frequency correction value for the device.
    /// @param ppm correction value in parts per million (ppm)
    pub fn set_freq_correction(&mut self, ppm: i32) -> Result<(), String> {
        rtlsdr_result!(sys::rtlsdr_set_freq_correction(self.dev, ppm))
            .map_err(|e| format!("Failed to set frequency correction: {}", e))?;

        Ok(())
    }

    /// Get the tuner type
    pub fn get_tuner_type(&self) -> TunerType {
        let tuner_type = unsafe { sys::rtlsdr_get_tuner_type(self.dev) };

        match tuner_type {
            0 => TunerType::Unknown,
            1 => TunerType::E4000,
            2 => TunerType::FC0012,
            3 => TunerType::FC0013,
            4 => TunerType::FC2580,
            5 => TunerType::R820T,
            6 => TunerType::R828D,
            _ => TunerType::Unknown,
        }
    }

    /// Get a list of gains supported by the tuner.
    /// Gain values in tenths of a dB, 115 means 11.5 dB
    pub fn get_tuner_gains(&self) -> Vec<i32> {
        let n = unsafe { sys::rtlsdr_get_tuner_gains(self.dev, std::ptr::null_mut()) };
        let mut gains = vec![0i32; n as usize];
        let n = unsafe { sys::rtlsdr_get_tuner_gains(self.dev, gains.as_mut_ptr()) };
        gains.truncate(n as usize);
        gains
    }

    /// Get actual (RF / HF) gain the device is configured to - excluding the IF gain.
    /// Gain in tenths of a dB, 115 means 11.5 dB.
    /// unfortunately it's impossible to distinguish error against 0 dB
    pub fn get_tuner_gain(&self) -> i32 {
        unsafe { sys::rtlsdr_get_tuner_gain(self.dev) }
    }

    /// Set the gain for the device.
    /// Manual gain mode must be enabled for this to work.
    /// Valid gain values may be queried with rtlsdr_get_tuner_gains function.
    /// Gain in tenths of a dB, 115 means 11.5 dB
    pub fn set_tuner_gain(&mut self, gain: i32) -> Result<(), String> {
        rtlsdr_result!(sys::rtlsdr_set_tuner_gain(self.dev, gain))
            .map_err(|e| format!("Failed to set tuner gain: {}", e))?;

        Ok(())
    }

    /// Set the bandwidth for the device.
    /// @param bw bandwidth in Hz. Zero means automatic BW selection.
    pub fn set_tuner_bandwidth(&mut self, bw: u32) -> Result<(), String> {
        rtlsdr_result!(sys::rtlsdr_set_tuner_bandwidth(self.dev, bw,))
            .map_err(|e| format!("Failed to set bandwidth: {}", e))?;

        Ok(())
    }

    /// Set the intermediate frequency gain for the device.
    /// @param stage intermediate frequency gain stage number (1 to 6 for E4000)
    /// @param gain in tenths of a dB, -30 means -3.0 dB.
    pub fn set_tuner_if_gain(&mut self, stage: i32, gain: i32) -> Result<(), String> {
        rtlsdr_result!(sys::rtlsdr_set_tuner_if_gain(self.dev, stage, gain))
            .map_err(|e| format!("Failed to set IF gain: {}", e))?;

        Ok(())
    }

    /// Set the gain mode (automatic/manual) for the device.
    /// Manual gain mode must be enabled for the gain setter function to work.
    pub fn set_tuner_gain_mode(&mut self, manual: bool) -> Result<(), String> {
        rtlsdr_result!(sys::rtlsdr_set_tuner_gain_mode(self.dev, manual as i32))
            .map_err(|e| format!("Failed to set tuner gain mode: {}", e))?;

        Ok(())
    }

    /// Get actual sample rate the device is configured to.
    /// @return sample rate in Hz
    pub fn get_sample_rate(&self) -> Result<u32, String> {
        match unsafe { sys::rtlsdr_get_sample_rate(self.dev) } {
            0 => Err("Failed to get sample rate".to_string()),
            rate => Ok(rate),
        }
    }

    /// Set the sample rate for the device.
    /// @param rate sample rate in Hz
    pub fn set_sample_rate(&mut self, rate: u32) -> Result<(), String> {
        rtlsdr_result!(sys::rtlsdr_set_sample_rate(self.dev, rate))
            .map_err(|e| format!("Failed to set sample rate: {}", e))?;

        Ok(())
    }

    /// Enable test mode that returns an 8 bit counter instead of the samples.
    /// The counter is generated inside the RTL2832.
    pub fn set_test_mode(&mut self, test_mode: bool) -> Result<(), String> {
        rtlsdr_result!(sys::rtlsdr_set_testmode(self.dev, test_mode as i32))
            .map_err(|e| format!("Failed to set test mode: {}", e))?;

        Ok(())
    }

    /// Enable or disable the internal digital AGC of the RTL2832.
    pub fn set_agc_mode(&mut self, enabled: bool) -> Result<(), String> {
        rtlsdr_result!(sys::rtlsdr_set_agc_mode(self.dev, enabled as i32))
            .map_err(|e| format!("Failed to set agc mode: {}", e))?;

        Ok(())
    }

    /// Get state of the direct sampling mode
    pub fn get_direct_sampling(&self) -> Result<DirectSampling, String> {
        rtlsdr_result!(sys::rtlsdr_get_direct_sampling(self.dev,))
            .map_err(|e| format!("Failed to get direct sampling mode: {}", e))
            .map(|mode| match mode {
                0 => DirectSampling::Disabled,
                1 => DirectSampling::I,
                2 => DirectSampling::Q,
                _ => DirectSampling::Disabled,
            })
    }

    /// Enable or disable the direct sampling mode.
    /// When enabled, the IF mode of the RTL2832 is activated, and set_center_freq() will control the IF-frequency of the DDC,
    /// which can be used to tune from 0 to 28.8 MHz (xtal frequency of the RTL2832).
    pub fn set_direct_sampling(&mut self, mode: DirectSampling) -> Result<(), String> {
        rtlsdr_result!(sys::rtlsdr_set_direct_sampling(self.dev, mode as i32))
            .map_err(|e| format!("Failed to set direct sampling mode: {}", e))?;

        Ok(())
    }

    /// Get state of the offset tuning mode
    pub fn get_offset_tuning(&self) -> Result<bool, String> {
        rtlsdr_result!(sys::rtlsdr_get_offset_tuning(self.dev))
            .map_err(|e| format!("Failed to get offset tuning mode: {}", e))
            .map(|mode| mode == 1)
    }

    /// Enable or disable offset tuning for zero-IF tuners, which allows to avoid problems caused by the DC offset of the ADCs and 1/f noise.
    pub fn set_offset_tuning(&mut self, enabled: bool) -> Result<(), String> {
        rtlsdr_result!(sys::rtlsdr_set_offset_tuning(self.dev, enabled as i32))
            .map_err(|e| format!("Failed to set offset tuning mode: {}", e))?;

        Ok(())
    }

    /// Reset buffer in RTL2832
    pub fn reset_buffer(&mut self) -> Result<(), String> {
        rtlsdr_result!(sys::rtlsdr_reset_buffer(self.dev))
            .map_err(|e| format!("Failed to reset buffer: {}", e))?;

        Ok(())
    }

    /// Read data synchronously
    pub fn read_sync(&self, buf: &mut [u8]) -> Result<i32, RtlsdrError> {
        let mut n_read = 0;

        rtlsdr_result!(sys::rtlsdr_read_sync(
            self.dev,
            buf.as_mut_ptr() as *mut std::ffi::c_void,
            buf.len() as i32,
            &mut n_read
        ))?;

        Ok(n_read)
    }

    /// Read samples from the device asynchronously.
    /// This function will block until it is being canceled using rtlsdr_cancel_async()
    /// NOTE: This function is deprecated and is subject for removal.
    /// @param cb callback function to return received samples
    /// @param ctx user specific context to pass via the callback function
    #[deprecated]
    pub fn wait_async<F>(&self, cb: F) -> Result<(), String>
    where
        F: FnMut(Vec<u8>),
    {
        self.read_async(cb, 0, 0)
    }

    /// Read samples from the device asynchronously.
    /// This function will block until it is being canceled using rtlsdr_cancel_async()
    /// @param cb callback function to return received samples
    /// @param buf_num optional buffer count, buf_num * buf_len = overall buffer size
    /// set to 0 for default buffer count (15)
    /// @param buf_len optional buffer length, must be multiple of 512,
    /// should be a multiple of 16384 (URB size), set to 0 for default buffer length (16 * 32 * 512)
    pub fn read_async<F>(&self, mut cb: F, buf_num: u32, buf_len: u32) -> Result<(), String>
    where
        F: FnMut(Vec<u8>),
    {
        unsafe extern "C" fn _cb<F>(buf: *mut u8, len: u32, ctx: *mut std::ffi::c_void)
        where
            F: FnMut(Vec<u8>),
        {
            let cb = &mut *(ctx as *mut F);
            let mut vec = Vec::with_capacity(len as usize);
            (0..len).for_each(|i| vec.push(*buf.offset(i as isize)));
            cb(vec);
        }

        rtlsdr_result!(sys::rtlsdr_read_async(
            self.dev,
            Some(_cb::<F>),
            &mut cb as *mut F as *mut std::ffi::c_void,
            buf_num,
            buf_len
        ))
        .map_err(|e| format!("Failed to wait async: {}", e))?;

        Ok(())
    }

    /// Cancel all pending asynchronous operations on the device.
    /// Due to incomplete concurrency implementation, this should only be called from within the callback function, so it is
    /// in the correct thread.
    pub fn cancel_async(&self) -> Result<(), String> {
        rtlsdr_result!(sys::rtlsdr_cancel_async(self.dev))
            .map_err(|e| format!("Failed to cancel async: {}", e))?;

        Ok(())
    }

    /// Enable or disable (the bias tee on) GPIO PIN 0 - if not reconfigured.
    /// See rtlsdr_set_opt_string() option 'T'.
    /// This works for rtl-sdr.com v3 dongles, see http://www.rtl-sdr.com/rtl-sdr-blog-v-3-dongles-user-guide/
    /// Note: rtlsdr_close() does not clear GPIO lines, so it leaves the (bias tee) line enabled if a client program
    /// doesn't explictly disable it.
    pub fn set_bias_tee(&mut self, on: bool) -> Result<(), String> {
        rtlsdr_result!(sys::rtlsdr_set_bias_tee(self.dev, on as i32)).map_err(|e| match e {
            -1 => "Device is not initialized".to_string(),
            _ => format!("Failed to set bias tee: {}", e),
        })?;

        Ok(())
    }

    /// Enable or disable (the bias tee on) the given GPIO pin.
    /// Note: rtlsdr_close() does not clear GPIO lines, so it leaves the (bias tee) lines enabled if a client program
    /// doesn't explictly disable it.
    /// @param gpio the gpio pin -- assuming this line is connected to Bias T.
    /// gpio needs to be in 0 .. 7. BUT pin 4 is connected to Tuner RESET.
    /// and for FC0012 is already connected/reserved pin 6 for switching V/U-HF.
    /// @param on: 1 for Bias T on. 0 for Bias T off.
    pub fn set_bias_tee_gpio(&mut self, gpio: i32, on: bool) -> Result<(), String> {
        rtlsdr_result!(sys::rtlsdr_set_bias_tee_gpio(self.dev, gpio, on as i32)).map_err(|e| {
            match e {
                -1 => "Device is not initialized".to_string(),
                _ => format!("Failed to set bias tee gpio: {}", e),
            }
        })?;

        Ok(())
    }
}

#[doc = "Get all available devices"]
pub fn get_devices() -> Vec<Device> {
    let n = unsafe { sys::rtlsdr_get_device_count() };

    (0..n)
        .map(|index| Device {
            index,
            dev: std::ptr::null_mut(),
        })
        .collect()
}
