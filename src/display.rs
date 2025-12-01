//! ILI9342C Display driver for M5Stack Core2 v1.1
//!
//! This module provides manual SPI-based display initialization and drawing
//! for the ILI9342C LCD controller (320x240 pixels).

use embassy_time::{Duration, Timer};
use embedded_hal_async::spi::SpiBus;
use esp_hal::gpio::Output;
use esp_hal::spi::master::SpiDmaBus;
use log::info;

/// Display dimensions
pub const LCD_WIDTH: u16 = 320;
pub const LCD_HEIGHT: u16 = 240;

/// ILI9342C Commands
mod cmd {
    pub const NOP: u8 = 0x00;
    pub const SWRESET: u8 = 0x01;
    pub const SLPOUT: u8 = 0x11;
    pub const INVON: u8 = 0x21;
    pub const DISPON: u8 = 0x29;
    pub const CASET: u8 = 0x2A;
    pub const RASET: u8 = 0x2B;
    pub const RAMWR: u8 = 0x2C;
    pub const MADCTL: u8 = 0x36;
    pub const COLMOD: u8 = 0x3A;

    // ILI9342C specific
    pub const SETEXTC: u8 = 0xC8;
    pub const PWCTR1: u8 = 0xC0;
    pub const PWCTR2: u8 = 0xC1;
    pub const VMCTR1: u8 = 0xC5;
    pub const GMCTRP1: u8 = 0xE0;
    pub const GMCTRN1: u8 = 0xE1;
    pub const DFUNCTR: u8 = 0xB6;
}

/// RGB565 color type
pub type Color = u16;

/// RGB565 color helpers
pub mod colors {
    use super::Color;

    pub const fn rgb565(r: u8, g: u8, b: u8) -> Color {
        ((r as u16 & 0xF8) << 8) | ((g as u16 & 0xFC) << 3) | (b as u16 >> 3)
    }

    pub const BLACK: Color = 0x0000;
    pub const WHITE: Color = 0xFFFF;
    pub const RED: Color = rgb565(255, 0, 0);
    pub const GREEN: Color = rgb565(0, 255, 0);
    pub const BLUE: Color = rgb565(0, 0, 255);
    pub const YELLOW: Color = rgb565(255, 255, 0);
    pub const CYAN: Color = rgb565(0, 255, 255);
    pub const MAGENTA: Color = rgb565(255, 0, 255);
}

/// ILI9342C Display driver
pub struct Display<'d, SPI, CS, DC> {
    spi: SPI,
    cs: CS,
    dc: DC,
    _phantom: core::marker::PhantomData<&'d ()>,
}

impl<'d, SPI, CS, DC> Display<'d, SPI, CS, DC>
where
    SPI: embedded_hal_async::spi::SpiDevice,
    CS: embedded_hal::digital::OutputPin,
    DC: embedded_hal::digital::OutputPin,
{
    /// Create a new display instance
    pub fn new(spi: SPI, cs: CS, dc: DC) -> Self {
        Self {
            spi,
            cs,
            dc,
            _phantom: core::marker::PhantomData,
        }
    }

    /// Send a command byte
    async fn cmd(&mut self, cmd: u8) -> Result<(), SPI::Error> {
        let _ = self.dc.set_low();
        let _ = self.cs.set_low();
        self.spi.write(&[cmd]).await?;
        let _ = self.cs.set_high();
        Ok(())
    }

    /// Send data bytes
    async fn data(&mut self, data: &[u8]) -> Result<(), SPI::Error> {
        let _ = self.dc.set_high();
        let _ = self.cs.set_low();
        self.spi.write(data).await?;
        let _ = self.cs.set_high();
        Ok(())
    }

    /// Send command followed by data
    async fn cmd_data(&mut self, cmd: u8, data: &[u8]) -> Result<(), SPI::Error> {
        self.cmd(cmd).await?;
        self.data(data).await
    }

    /// Initialize the display
    pub async fn init(&mut self) -> Result<(), SPI::Error> {
        info!("Initializing ILI9342C display...");

        // Software reset
        self.cmd(cmd::SWRESET).await?;
        Timer::after(Duration::from_millis(150)).await;

        // Exit sleep mode
        self.cmd(cmd::SLPOUT).await?;
        Timer::after(Duration::from_millis(150)).await;

        // Turn on extended command set
        self.cmd_data(cmd::SETEXTC, &[0xFF, 0x93, 0x42]).await?;

        // Power control 1
        self.cmd_data(cmd::PWCTR1, &[0x12, 0x12]).await?;

        // Power control 2
        self.cmd_data(cmd::PWCTR2, &[0x03]).await?;

        // VCOM control
        self.cmd_data(cmd::VMCTR1, &[0xF2]).await?;

        // Interface control
        self.cmd_data(0xB0, &[0xE0]).await?;

        // Interface mode control
        self.cmd_data(0xF6, &[0x01, 0x00, 0x00]).await?;

        // Positive gamma correction
        self.cmd_data(
            cmd::GMCTRP1,
            &[
                0x00, 0x0C, 0x11, 0x04, 0x11, 0x08, 0x37, 0x89, 0x4C, 0x06, 0x0C, 0x0A, 0x2E, 0x34,
                0x0F,
            ],
        )
        .await?;

        // Negative gamma correction
        self.cmd_data(
            cmd::GMCTRN1,
            &[
                0x00, 0x0B, 0x11, 0x05, 0x13, 0x09, 0x33, 0x67, 0x48, 0x07, 0x0E, 0x0B, 0x2E, 0x33,
                0x0F,
            ],
        )
        .await?;

        // Display function control
        self.cmd_data(cmd::DFUNCTR, &[0x08, 0x82, 0x1D, 0x04])
            .await?;

        // Memory access control - rotation for Core2 (landscape, RGB)
        // 0x08 = MV=0, MX=0, MY=0, RGB order
        self.cmd_data(cmd::MADCTL, &[0x08]).await?;

        // Pixel format - 16bit (RGB565)
        self.cmd_data(cmd::COLMOD, &[0x55]).await?;

        // Invert display (Core2 display needs inversion)
        self.cmd(cmd::INVON).await?;

        // Display on
        self.cmd(cmd::DISPON).await?;
        Timer::after(Duration::from_millis(50)).await;

        info!("Display initialized!");
        Ok(())
    }

    /// Set the drawing window
    async fn set_window(&mut self, x0: u16, y0: u16, x1: u16, y1: u16) -> Result<(), SPI::Error> {
        self.cmd_data(
            cmd::CASET,
            &[
                (x0 >> 8) as u8,
                x0 as u8,
                (x1 >> 8) as u8,
                x1 as u8,
            ],
        )
        .await?;

        self.cmd_data(
            cmd::RASET,
            &[
                (y0 >> 8) as u8,
                y0 as u8,
                (y1 >> 8) as u8,
                y1 as u8,
            ],
        )
        .await?;

        self.cmd(cmd::RAMWR).await
    }

    /// Fill a rectangle with a color
    pub async fn fill_rect(
        &mut self,
        x: u16,
        y: u16,
        w: u16,
        h: u16,
        color: Color,
    ) -> Result<(), SPI::Error> {
        self.set_window(x, y, x + w - 1, y + h - 1).await?;

        let _ = self.dc.set_high();
        let _ = self.cs.set_low();

        let color_bytes = [((color >> 8) as u8), (color as u8)];
        let pixels = (w as u32) * (h as u32);

        // Write in chunks to avoid stack overflow
        const CHUNK_SIZE: usize = 512;
        let mut buffer = [0u8; CHUNK_SIZE];

        // Fill buffer with color
        for i in 0..(CHUNK_SIZE / 2) {
            buffer[i * 2] = color_bytes[0];
            buffer[i * 2 + 1] = color_bytes[1];
        }

        let full_chunks = (pixels * 2) as usize / CHUNK_SIZE;
        let remainder = (pixels * 2) as usize % CHUNK_SIZE;

        for _ in 0..full_chunks {
            self.spi.write(&buffer).await?;
        }

        if remainder > 0 {
            self.spi.write(&buffer[..remainder]).await?;
        }

        let _ = self.cs.set_high();
        Ok(())
    }

    /// Fill the entire screen with a color
    pub async fn fill_screen(&mut self, color: Color) -> Result<(), SPI::Error> {
        self.fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, color).await
    }
}

/// Display driver using esp-hal's DMA SPI with manual CS/DC control
pub struct DisplayDma<'d> {
    spi: SpiDmaBus<'d, esp_hal::Async>,
    cs: Output<'d>,
    dc: Output<'d>,
}

impl<'d> DisplayDma<'d> {
    /// Create a new DMA-based display instance
    pub fn new(spi: SpiDmaBus<'d, esp_hal::Async>, cs: Output<'d>, dc: Output<'d>) -> Self {
        Self { spi, cs, dc }
    }

    /// Send a command byte
    async fn cmd(&mut self, cmd: u8) {
        self.dc.set_low();
        self.cs.set_low();
        let _ = SpiBus::write(&mut self.spi, &[cmd]).await;
        self.cs.set_high();
    }

    /// Send data bytes
    async fn data(&mut self, data: &[u8]) {
        self.dc.set_high();
        self.cs.set_low();
        let _ = SpiBus::write(&mut self.spi, data).await;
        self.cs.set_high();
    }

    /// Send command followed by data
    async fn cmd_data(&mut self, cmd: u8, data: &[u8]) {
        self.cmd(cmd).await;
        self.data(data).await;
    }

    /// Initialize the display
    pub async fn init(&mut self) {
        info!("Initializing ILI9342C display (DMA)...");

        // Software reset
        self.cmd(cmd::SWRESET).await;
        Timer::after(Duration::from_millis(150)).await;

        // Exit sleep mode
        self.cmd(cmd::SLPOUT).await;
        Timer::after(Duration::from_millis(150)).await;

        // Turn on extended command set
        self.cmd_data(cmd::SETEXTC, &[0xFF, 0x93, 0x42]).await;

        // Power control 1
        self.cmd_data(cmd::PWCTR1, &[0x12, 0x12]).await;

        // Power control 2
        self.cmd_data(cmd::PWCTR2, &[0x03]).await;

        // VCOM control
        self.cmd_data(cmd::VMCTR1, &[0xF2]).await;

        // Interface control
        self.cmd_data(0xB0, &[0xE0]).await;

        // Interface mode control
        self.cmd_data(0xF6, &[0x01, 0x00, 0x00]).await;

        // Positive gamma correction
        self.cmd_data(
            cmd::GMCTRP1,
            &[
                0x00, 0x0C, 0x11, 0x04, 0x11, 0x08, 0x37, 0x89, 0x4C, 0x06, 0x0C, 0x0A, 0x2E, 0x34,
                0x0F,
            ],
        )
        .await;

        // Negative gamma correction
        self.cmd_data(
            cmd::GMCTRN1,
            &[
                0x00, 0x0B, 0x11, 0x05, 0x13, 0x09, 0x33, 0x67, 0x48, 0x07, 0x0E, 0x0B, 0x2E, 0x33,
                0x0F,
            ],
        )
        .await;

        // Display function control
        self.cmd_data(cmd::DFUNCTR, &[0x08, 0x82, 0x1D, 0x04])
            .await;

        // Memory access control - rotation for Core2 (landscape, RGB)
        self.cmd_data(cmd::MADCTL, &[0x08]).await;

        // Pixel format - 16bit (RGB565)
        self.cmd_data(cmd::COLMOD, &[0x55]).await;

        // Invert display (Core2 display needs inversion)
        self.cmd(cmd::INVON).await;

        // Display on
        self.cmd(cmd::DISPON).await;
        Timer::after(Duration::from_millis(50)).await;

        info!("Display initialized!");
    }

    /// Set the drawing window
    async fn set_window(&mut self, x0: u16, y0: u16, x1: u16, y1: u16) {
        self.cmd_data(
            cmd::CASET,
            &[(x0 >> 8) as u8, x0 as u8, (x1 >> 8) as u8, x1 as u8],
        )
        .await;

        self.cmd_data(
            cmd::RASET,
            &[(y0 >> 8) as u8, y0 as u8, (y1 >> 8) as u8, y1 as u8],
        )
        .await;

        self.cmd(cmd::RAMWR).await;
    }

    /// Fill a rectangle with a color
    pub async fn fill_rect(&mut self, x: u16, y: u16, w: u16, h: u16, color: Color) {
        self.set_window(x, y, x + w - 1, y + h - 1).await;

        self.dc.set_high();
        self.cs.set_low();

        let color_bytes = [(color >> 8) as u8, color as u8];
        let pixels = (w as u32) * (h as u32);

        // Write in chunks to avoid stack overflow
        const CHUNK_SIZE: usize = 512;
        let mut buffer = [0u8; CHUNK_SIZE];

        // Fill buffer with color
        for i in 0..(CHUNK_SIZE / 2) {
            buffer[i * 2] = color_bytes[0];
            buffer[i * 2 + 1] = color_bytes[1];
        }

        let full_chunks = (pixels * 2) as usize / CHUNK_SIZE;
        let remainder = (pixels * 2) as usize % CHUNK_SIZE;

        for _ in 0..full_chunks {
            let _ = SpiBus::write(&mut self.spi, &buffer).await;
        }

        if remainder > 0 {
            let _ = SpiBus::write(&mut self.spi, &buffer[..remainder]).await;
        }

        self.cs.set_high();
    }

    /// Fill the entire screen with a color
    pub async fn fill_screen(&mut self, color: Color) {
        self.fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, color).await;
    }
}
