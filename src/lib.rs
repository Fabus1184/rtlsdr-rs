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

#[derive(Debug, Clone, PartialEq, Eq)]
/// Device struct
pub struct Device {
    index: u32,
    dev: *mut sys::rtlsdr_dev,
}

impl Device {
    /// Open device
    /// This may fail due to a libusb error or some other unspecified error
    pub fn open(&mut self) -> Result<(), RtlsdrError> {
        rtlsdr_result!(sys::rtlsdr_open(&mut self.dev, self.index))
            .map_err(|e| Into::<RtlsdrError>::into(e))?;

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
            buf.as_mut_ptr() as *mut u8,
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
    pub fn get_center_freq(&self) -> Result<u64, String> {
        match unsafe { sys::rtlsdr_get_center_freq64(self.dev) } {
            0 => Err("Failed to get center frequency".to_string()),
            freq => Ok(freq),
        }
    }

    /// Set the frequency the device is tuned to in Hz
    pub fn set_center_freq(&mut self, freq: u64) -> Result<(), String> {
        rtlsdr_result!(sys::rtlsdr_set_center_freq64(self.dev, freq))
            .map_err(|e| format!("Failed to set center frequency: {}", e))?;

        Ok(())
    }

    /// Set harmonic reception - for R820T/2 tuner
    /// @param harmonic - receive n'th harmonic. 0 = default for disabling this
    pub fn set_harmonic_rx(&mut self, harmonic: i32) -> Result<(), String> {
        rtlsdr_result!(sys::rtlsdr_set_harmonic_rx(self.dev, harmonic))
            .map_err(|e| format!("Failed to set harmonic rx: {}", e))?;

        Ok(())
    }

    /// Check, if tuner PLL (frequency) is still locked.
    /// Tuner/PLL might loose lock (at high frequencies),
    /// e.g. for temperature reasons
    pub fn is_tuner_pll_locked(&self) -> Result<bool, String> {
        match unsafe { sys::rtlsdr_is_tuner_PLL_locked(self.dev) } {
            0 => Ok(true),
            1 => Ok(false),
            -2 => Err("Not supported for devices' tuner".to_string()),
            e => Err(format!("Failed to check if tuner PLL is locked: {}", e)),
        }
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

    /// Get the bandwidth for the device.
    /// Zero means automatic BW selection.
    pub fn get_bandwidth(&self) -> Result<u32, String> {
        let mut applied_bw = 0;
        rtlsdr_result!(sys::rtlsdr_set_and_get_tuner_bandwidth(
            self.dev,
            0,
            &mut applied_bw,
            0
        ))
        .map_err(|e| format!("Failed to get bandwidth: {}", e))?;

        Ok(applied_bw)
    }

    /// Set the bandwidth for the device.
    /// @param bw bandwidth in Hz. Zero means automatic BW selection.
    pub fn set_tuner_bandwidth(&mut self, bw: u32) -> Result<(), String> {
        rtlsdr_result!(sys::rtlsdr_set_tuner_bandwidth(self.dev, bw,))
            .map_err(|e| format!("Failed to set bandwidth: {}", e))?;

        Ok(())
    }

    /// Sets the center of the filtered tuner band(width)
    /// @param center in Hz.
    /// Zero means, that band center shall be at zero (=default).
    /// set if_band_center_freq = +samplerate/4 to have the filtered band centered at output's right half.
    pub fn set_tuner_band_center(&mut self, center: i32) -> Result<(), String> {
        rtlsdr_result!(sys::rtlsdr_set_tuner_band_center(self.dev, center))
            .map_err(|e| format!("Failed to set tuner band center: {}", e))?;

        Ok(())
    }

    /// Set the mixer sideband for the device.
    pub fn set_tuner_sideband(&mut self, sideband: Sideband) -> Result<(), String> {
        rtlsdr_result!(sys::rtlsdr_set_tuner_sideband(self.dev, sideband as i32))
            .map_err(|e| format!("Failed to set tuner mixer sideband: {}", e))?;

        Ok(())
    }

    /// Set LNA / Mixer / VGA Device Gain for R820T/2 device is configured to.
    /// @param lna_gain index in 0 .. 15: 0 == min;   see tuner_r82xx.c table r82xx_lna_gain_steps[]
    /// @param mixer_gain index in 0 .. 15: 0 == min; see tuner_r82xx.c table r82xx_mixer_gain_steps[]
    /// @param vga_gain index in 0 .. 15: 0 == -12 dB; 15 == 40.5 dB; => 3.5 dB/step;
    /// vga_gain index 16 activates AGC for VGA controlled from RTL2832
    /// see tuner_r82xx.c table r82xx_vga_gain_steps[]
    pub fn set_tuner_gain_ext(
        &self,
        lna_gain: i32,
        mixer_gain: i32,
        vga_gain: i32,
    ) -> Result<(), String> {
        rtlsdr_result!(sys::rtlsdr_set_tuner_gain_ext(
            self.dev, lna_gain, mixer_gain, vga_gain
        ))
        .map_err(|e| format!("Failed to set tuner gain ext: {}", e))?;

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

    /// Set the agc_variant for automatic gain mode for the device (only R820T/2).
    /// Automatic gain mode must be enabled for the gain setter function to work.
    /// @param if_mode
    /// 0: set automatic VGA, which is controlled from RTL2832
    /// -2500 .. +2500: set fixed IF gain in tenths of a dB, 115 means 11.5 dB. use -1 or +1 in case you neither want attenuation nor gain.
    /// this equals the VGA gain for R820T/2 tuner. exact values (R820T/2) are in range -47 .. 408 in tenth of a dB,
    /// giving -4.7 .. +40.8 dB. these exact values may slightly change with better measurements.
    /// 10000 .. 10015: IF gain == VGA index from parameter if_mode set if_mode by index: index := VGA_idx +10000
    /// 10016 .. 10031: same as 10000 .. 10015, but additionally set automatic VGA
    /// 10011:          for fixed VGA (=default) of -12 dB + 11 * 3.5 dB = 26.5 dB
    pub fn set_tuner_if_mode(&mut self, agc_mode: i32) -> Result<(), String> {
        rtlsdr_result!(sys::rtlsdr_set_tuner_if_mode(self.dev, agc_mode))
            .map_err(|e| format!("Failed to set tuner agc mode: {}", e))?;

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

    /// Set direct sampling mode with threshold
    /// @param mode static modes 0 .. 2 as in rtlsdr_set_direct_sampling().
    /// other modes do automatic switching
    /// @param freq_threshold direct sampling is used below this frequency, else quadrature mode through tuner
    ///  set 0 for using default setting per tuner - not fully implemented yet!
    pub fn set_ds_mode(
        &mut self,
        mode: DirectSamplingThreshold,
        freq_threshold: u32,
    ) -> Result<(), String> {
        rtlsdr_result!(sys::rtlsdr_set_ds_mode(
            self.dev,
            mode as u32,
            freq_threshold
        ))
        .map_err(|e| format!("Failed to set direct sampling mode with threshold: {}", e))?;

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

    /// Enable or disable frequency dithering for r820t tuners.
    /// Must be performed before freq_set().
    /// Fails for other tuners.
    pub fn set_dithering(&mut self, enabled: bool) -> Result<(), String> {
        rtlsdr_result!(sys::rtlsdr_set_dithering(self.dev, enabled as i32))
            .map_err(|e| format!("Failed to set frequency correction mode: {}", e))?;

        Ok(())
    }

    /// Reset buffer in RTL2832
    pub fn reset_buffer(&mut self) -> Result<(), String> {
        rtlsdr_result!(sys::rtlsdr_reset_buffer(self.dev))
            .map_err(|e| format!("Failed to reset buffer: {}", e))?;

        Ok(())
    }

    /// Read data synchronously
    /// librtlsdr states this errors "on error or error code from libusb"
    /// so we assume it's a libusb error
    pub fn read_sync(&self, buf: &mut [u8]) -> Result<i32, RtlsdrError> {
        let mut n_read = 0;

        rtlsdr_result!(sys::rtlsdr_read_sync(
            self.dev,
            buf.as_mut_ptr() as *mut std::ffi::c_void,
            buf.len() as i32,
            &mut n_read
        ))
        .map_err(|e| Into::<RtlsdrError>::into(e))?;

        Ok(n_read)
    }

    /// Read samples from the device asynchronously.
    /// This function will block until it is being canceled using rtlsdr_cancel_async()
    /// NOTE: This function is deprecated and is subject for removal.
    /// @param cb callback function to return received samples
    /// @param ctx user specific context to pass via the callback function
    #[deprecated]
    pub fn wait_async<F, U>(&mut self, cb: F, ctx: &mut U) -> Result<(), String>
    where
        F: FnMut(*mut u8, u32, &mut U),
    {
        self.read_async(cb, ctx, 0, 0)
    }

    /// Read samples from the device asynchronously.
    /// This function will block until it is being canceled using rtlsdr_cancel_async()
    /// @param cb callback function to return received samples
    /// @param ctx user specific context to pass via the callback function
    /// @param buf_num optional buffer count, buf_num * buf_len = overall buffer size
    /// set to 0 for default buffer count (15)
    /// @param buf_len optional buffer length, must be multiple of 512,
    /// should be a multiple of 16384 (URB size), set to 0 for default buffer length (16 * 32 * 512)
    pub fn read_async<F, U>(
        &mut self,
        cb: F,
        ctx: &mut U,
        buf_num: u32,
        buf_len: u32,
    ) -> Result<(), String>
    where
        F: FnMut(*mut u8, u32, &mut U),
    {
        unsafe extern "C" fn _cb<F, U>(buf: *mut u8, len: u32, ctx: *mut std::ffi::c_void)
        where
            F: FnMut(*mut u8, u32, &mut U),
        {
            let (cb, ctx) = &mut *(ctx as *mut (F, &mut U));
            cb(buf, len, *ctx);
        }

        let mut _ctx = (cb, ctx);

        rtlsdr_result!(sys::rtlsdr_read_async(
            self.dev,
            Some(_cb::<F, U>),
            &mut _ctx as *mut (F, &mut U) as *mut std::ffi::c_void,
            buf_num,
            buf_len
        ))
        .map_err(|e| format!("Failed to wait async: {}", e))?;

        Ok(())
    }

    /// Cancel all pending asynchronous operations on the device.
    /// Due to incomplete concurrency implementation, this should only be called from within the callback function, so it is
    /// in the correct thread.
    pub fn cancel_async(&mut self) -> Result<(), String> {
        rtlsdr_result!(sys::rtlsdr_cancel_async(self.dev))
            .map_err(|e| format!("Failed to cancel async: {}", e))?;

        Ok(())
    }

    /// Read from the remote control (RC) infrared (IR) sensor
    /// @param buf buffer to write IR signal (MSB=pulse/space, 7LSB=duration*20usec), recommended 128-bytes
    /// @param buf_len size of buf
    /// @return 0 if no signal, >0 number of bytes written into buf
    pub fn ir_query(&self, buf: &mut [u8]) -> Result<i32, String> {
        let n_read = rtlsdr_result!(sys::rtlsdr_ir_query(self.dev, buf.as_mut_ptr(), buf.len(),))
            .map_err(|e| format!("Failed to query IR: {}", e))?;

        Ok(n_read)
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

    /// Setup a GPIO pin as output.
    pub fn set_gpio_output(&mut self, gpio: u8) -> Result<(), String> {
        rtlsdr_result!(sys::rtlsdr_set_gpio_output(self.dev, gpio))
            .map_err(|e| format!("Failed to set gpio output: {}", e))?;

        Ok(())
    }

    /// Setup a GPIO pin as input.
    pub fn set_gpio_input(&mut self, gpio: u8) -> Result<(), String> {
        rtlsdr_result!(sys::rtlsdr_set_gpio_input(self.dev, gpio))
            .map_err(|e| format!("Failed to set gpio input: {}", e))?;

        Ok(())
    }

    /// Read a bit from a GPIO pin.
    pub fn get_gpio_bit(&self, gpio: u8) -> Result<i32, String> {
        let mut bit = 0;

        rtlsdr_result!(sys::rtlsdr_get_gpio_bit(self.dev, gpio, &mut bit))
            .map_err(|e| format!("Failed to read gpio: {}", e))?;

        Ok(bit)
    }

    /// Write a bit to a GPIO pin.
    pub fn set_gpio_bit(&mut self, gpio: u8, bit: i32) -> Result<(), String> {
        rtlsdr_result!(sys::rtlsdr_set_gpio_bit(self.dev, gpio, bit))
            .map_err(|e| format!("Failed to write gpio: {}", e))?;

        Ok(())
    }

    /// Read GPIO byte.
    pub fn get_gpio_byte(&self) -> Result<i32, String> {
        let mut byte = 0;

        rtlsdr_result!(sys::rtlsdr_get_gpio_byte(self.dev, &mut byte))
            .map_err(|e| format!("Failed to read gpio byte: {}", e))?;

        Ok(byte)
    }

    /// Write GPIO byte.
    pub fn set_gpio_byte(&mut self, byte: i32) -> Result<(), String> {
        rtlsdr_result!(sys::rtlsdr_set_gpio_byte(self.dev, byte))
            .map_err(|e| format!("Failed to write gpio byte: {}", e))?;

        Ok(())
    }

    /// Set GPIO status
    pub fn set_gpio_status(&mut self, status: i32) -> Result<(), String> {
        rtlsdr_result!(sys::rtlsdr_set_gpio_status(
            self.dev,
            &status as *const i32 as *mut i32
        ))
        .map_err(|e| format!("Failed to set gpio status: {}", e))?;

        Ok(())
    }

    /// Sets multiple options from a string encoded like "bw=300:agc=0:gain=27.3:dagc=0:T=1".
    /// this is a helper function, that programs don't need to implement every single option at the command line interface.
    /// Options are seperated by colon ':'.
    /// There mustn't be extra spaces between option name and '='.
    /// option 'f' set center frequency as in rtlsdr_set_center_freq()
    /// option 'bw' sets tuner bandwidth as in rtlsdr_set_tuner_bandwidth() - but value is in kHz.
    /// option 'agc' sets tuner gain mode as with rtlsdr_set_tuner_gain_mode():
    ///     '1' means manual gain mode shall be enabled.
    /// option 'gain' sets tuner gain as with rtlsdr_set_tuner_gain():
    ///     values in tenth dB.
    /// option 'dagc' or 'dgc' de/activates digital agc as with rtlsdr_set_agc_mode().
    ///     1 to enable.
    ///     0 to disable.
    /// option 'ds' set direct sampling as with rtlsdr_set_direct_sampling():
    ///     '0' to deactivate,
    ///     '1' or 'i' for I-ADC input,
    ///     '2' or 'q' for Q-ADC input
    /// option 't' or 'T' for enabling bias tee on GPIO PIN 0 as with rtlsdr_set_bias_tee():
    ///     '1' for Bias T on.
    ///     '0' for Bias T off.
    /// @param opts described option string
    /// @param verbose print parsed options to stderr
    pub fn set_opt_string(&mut self, opts: String, verbose: bool) -> Result<(), String> {
        let cstr = std::ffi::CString::new(opts).expect("Failed to convert opts to CString");

        rtlsdr_result!(sys::rtlsdr_set_opt_string(
            self.dev,
            cstr.as_ptr(),
            verbose as i32
        ))
        .map_err(|e| format!("Failed to set opt string: {}", e))?;

        Ok(())
    }

    /// Get opt help
    pub fn get_opt_help(long_info: bool) -> String {
        let ptr = unsafe { sys::rtlsdr_get_opt_help(long_info as i32) };
        let str = unsafe { std::ffi::CStr::from_ptr(ptr) };
        str.to_str()
            .expect("Failed to convert opt help from CStr")
            .to_owned()
    }

    /// Get tuner i2c register
    pub fn get_tuner_i2c_register(&self, data: &mut [u8]) -> Result<(), String> {
        rtlsdr_result!(sys::rtlsdr_get_tuner_i2c_register(
            self.dev,
            data.as_mut_ptr(),
            data.len() as i32
        ))
        .map_err(|e| match e {
            _ => format!("Failed to get tuner i2c register: {}", e),
        })?;

        Ok(())
    }

    /// Exposes/permits hacking of Tuner-specific I2C registers:
    /// set register once
    /// @param i2c_register  register address
    /// @param mask          8-bit bitmask, indicating which bits shall be set
    /// @param data          8-bit data, which shall be set
    pub fn set_tuner_i2c_register(
        &mut self,
        i2c_register: u32,
        mask: u8,
        data: u8,
    ) -> Result<(), String> {
        rtlsdr_result!(sys::rtlsdr_set_tuner_i2c_register(
            self.dev,
            i2c_register,
            mask as u32,
            data as u32
        ))
        .map_err(|e| match e {
            -1 => "Device is not initialized".to_string(),
            _ => format!("Failed to set tuner i2c register: {}", e),
        })?;

        Ok(())
    }

    /// Exposes/permits hacking of Tuner-specific I2C registers:
    /// set and keep register for future
    /// @param i2c_register  register address
    /// @param mask          8-bit bitmask, indicating which bits shall be set
    /// @param data          8-bit data, which shall be set; data in 0 .. 255 sets override; data > 255 clears override
    pub fn set_tuner_i2c_override(
        &mut self,
        i2c_register: u32,
        mask: u8,
        data: u8,
    ) -> Result<(), String> {
        rtlsdr_result!(sys::rtlsdr_set_tuner_i2c_override(
            self.dev,
            i2c_register,
            mask as u32,
            data as u32
        ))
        .map_err(|e| match e {
            -1 => "Device is not initialized".to_string(),
            _ => format!("Failed to set tuner i2c register persistent: {}", e),
        })?;

        Ok(())
    }

    /// Request version id string to identify source and date of library
    pub fn get_version_id() -> &'static str {
        let ptr = unsafe { sys::rtlsdr_get_ver_id() };
        let str = unsafe { std::ffi::CStr::from_ptr(ptr) };

        str.to_str()
            .expect("Failed to convert version id from CStr")
    }

    /// Request version numbers of library
    /// @return (major, minor)
    pub fn get_version() -> (u16, u16) {
        let ver = unsafe { sys::rtlsdr_get_version() };
        let major = (ver >> 16) & 0xFFFF;
        let minor = ver & 0xFFFF;

        (major as u16, minor as u16)
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
