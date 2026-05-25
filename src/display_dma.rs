use core::fmt::Debug;
use core::ops::Range;

use embedded_hal::digital::OutputPin;
use embedded_hal::spi::{ErrorKind, ErrorType, Operation, SpiBus, SpiDevice};
use esp_hal::dma::DmaTxBuf;
use esp_hal::spi::master::{SpiDma, SpiDmaTransfer};
use esp_hal::{Blocking, spi};
use slint::platform::software_renderer::{LineBufferProvider, Rgb565PixelBE};

const RAM_WRITE: u8 = 0x2c;
const SET_COLUMN_ADDRESS: u8 = 0x2a;
const SET_PAGE_ADDRESS: u8 = 0x2b;
const BYTES_PER_PIXEL: usize = core::mem::size_of::<Rgb565PixelBE>();

#[derive(Debug)]
pub enum InitSpiDeviceError<SPI, CS> {
    Spi(SPI),
    Cs(CS),
}

impl<SPI, CS> embedded_hal::spi::Error for InitSpiDeviceError<SPI, CS>
where
    SPI: embedded_hal::spi::Error,
    CS: Debug,
{
    fn kind(&self) -> ErrorKind {
        match self {
            Self::Spi(error) => error.kind(),
            Self::Cs(_) => ErrorKind::Other,
        }
    }
}

pub struct InitSpiDevice<SPI, CS> {
    bus: SPI,
    cs: CS,
}

impl<SPI, CS> InitSpiDevice<SPI, CS>
where
    CS: OutputPin,
{
    pub fn new(bus: SPI, mut cs: CS) -> Result<Self, CS::Error> {
        cs.set_high()?;
        Ok(Self { bus, cs })
    }

    pub fn release(self) -> (SPI, CS) {
        (self.bus, self.cs)
    }
}

impl<SPI, CS> ErrorType for InitSpiDevice<SPI, CS>
where
    SPI: ErrorType,
    CS: OutputPin,
{
    type Error = InitSpiDeviceError<SPI::Error, CS::Error>;
}

impl<Word, SPI, CS> SpiDevice<Word> for InitSpiDevice<SPI, CS>
where
    Word: Copy + 'static,
    SPI: SpiBus<Word>,
    CS: OutputPin,
{
    fn transaction(&mut self, operations: &mut [Operation<'_, Word>]) -> Result<(), Self::Error> {
        self.cs.set_low().map_err(InitSpiDeviceError::Cs)?;

        let operation_result = 'operations: {
            for operation in operations {
                let result = match operation {
                    Operation::Read(buffer) => self.bus.read(buffer),
                    Operation::Write(buffer) => self.bus.write(buffer),
                    Operation::Transfer(read, write) => self.bus.transfer(read, write),
                    Operation::TransferInPlace(buffer) => self.bus.transfer_in_place(buffer),
                    Operation::DelayNs(_) => self.bus.flush(),
                };

                if let Err(error) = result {
                    break 'operations Err(error);
                }
            }

            self.bus.flush()
        };

        let cs_result = self.cs.set_high();

        operation_result.map_err(InitSpiDeviceError::Spi)?;
        cs_result.map_err(InitSpiDeviceError::Cs)
    }
}

#[derive(Debug)]
pub enum DmaLineDisplayError<CS, DC> {
    Spi(spi::Error),
    Cs(CS),
    Dc(DC),
    BufferTooSmall,
    InvalidRegion,
    MissingBuffer,
}

#[derive(Clone, Copy)]
struct ActiveTile {
    x: u16,
    y: u16,
    w: u16,
    lines: u16,
    next_y: u16,
}

impl ActiveTile {
    fn byte_len(self) -> usize {
        self.w as usize * self.lines as usize * BYTES_PER_PIXEL
    }
}

pub struct DmaLineDisplay<'d, CS, DC>
where
    CS: OutputPin,
    DC: OutputPin,
{
    spi: Option<SpiDma<'d, Blocking>>,
    render_buf: Option<DmaTxBuf>,
    spare_buf: Option<DmaTxBuf>,
    pending: Option<SpiDmaTransfer<'d, Blocking, DmaTxBuf>>,
    cs: CS,
    dc: DC,
    width: u16,
    height: u16,
    max_tile_lines: usize,
    active: Option<ActiveTile>,
    error: Option<DmaLineDisplayError<CS::Error, DC::Error>>,
}

impl<'d, CS, DC> DmaLineDisplay<'d, CS, DC>
where
    CS: OutputPin,
    DC: OutputPin,
{
    pub fn new(
        spi: SpiDma<'d, Blocking>,
        mut cs: CS,
        mut dc: DC,
        first_buf: DmaTxBuf,
        second_buf: DmaTxBuf,
        width: u16,
        height: u16,
        max_tile_lines: usize,
    ) -> Result<Self, DmaLineDisplayError<CS::Error, DC::Error>> {
        cs.set_high().map_err(DmaLineDisplayError::Cs)?;
        dc.set_high().map_err(DmaLineDisplayError::Dc)?;

        let min_capacity = first_buf.capacity().min(second_buf.capacity());
        let line_bytes = width as usize * BYTES_PER_PIXEL;
        let max_capacity_lines = min_capacity / line_bytes.max(1);
        let max_tile_lines = max_tile_lines.max(1).min(max_capacity_lines);
        if max_tile_lines == 0 {
            return Err(DmaLineDisplayError::BufferTooSmall);
        }

        Ok(Self {
            spi: Some(spi),
            render_buf: Some(first_buf),
            spare_buf: Some(second_buf),
            pending: None,
            cs,
            dc,
            width,
            height,
            max_tile_lines,
            active: None,
            error: None,
        })
    }

    pub fn finish_frame(&mut self) -> Result<(), DmaLineDisplayError<CS::Error, DC::Error>> {
        if let Some(error) = self.error.take() {
            let _ = self.wait_pending();
            self.active = None;
            return Err(error);
        }

        self.flush_active_tile()?;
        self.wait_pending()
    }

    fn set_error(&mut self, error: DmaLineDisplayError<CS::Error, DC::Error>) {
        if self.error.is_none() {
            self.error = Some(error);
        }
    }

    fn wait_pending(&mut self) -> Result<(), DmaLineDisplayError<CS::Error, DC::Error>> {
        if let Some(transfer) = self.pending.take() {
            let (spi, tx_buf) = transfer.wait();
            self.spi = Some(spi);
            self.cs.set_high().map_err(DmaLineDisplayError::Cs)?;
            self.spare_buf = Some(tx_buf);
        }

        Ok(())
    }

    fn write_with_spare_buf(
        &mut self,
        bytes: &[u8],
    ) -> Result<(), DmaLineDisplayError<CS::Error, DC::Error>> {
        if bytes.is_empty() {
            return Ok(());
        }

        let mut spi = self.spi.take().ok_or(DmaLineDisplayError::MissingBuffer)?;
        let mut tx_buf = self
            .spare_buf
            .take()
            .ok_or(DmaLineDisplayError::MissingBuffer)?;

        for chunk in bytes.chunks(tx_buf.capacity()) {
            tx_buf.fill(chunk);
            match spi.write(chunk.len(), tx_buf) {
                Ok(transfer) => {
                    let (returned_spi, returned_buf) = transfer.wait();
                    spi = returned_spi;
                    tx_buf = returned_buf;
                }
                Err((error, returned_spi, returned_buf)) => {
                    self.spi = Some(returned_spi);
                    self.spare_buf = Some(returned_buf);
                    return Err(DmaLineDisplayError::Spi(error));
                }
            }
        }

        self.spi = Some(spi);
        self.spare_buf = Some(tx_buf);
        Ok(())
    }

    fn write_command(
        &mut self,
        command: u8,
        args: &[u8],
    ) -> Result<(), DmaLineDisplayError<CS::Error, DC::Error>> {
        self.cs.set_low().map_err(DmaLineDisplayError::Cs)?;
        self.dc.set_low().map_err(DmaLineDisplayError::Dc)?;
        self.write_with_spare_buf(&[command])?;

        if !args.is_empty() {
            self.dc.set_high().map_err(DmaLineDisplayError::Dc)?;
            self.write_with_spare_buf(args)?;
        }

        self.cs.set_high().map_err(DmaLineDisplayError::Cs)
    }

    fn set_address_window(
        &mut self,
        x: u16,
        y: u16,
        w: u16,
        h: u16,
    ) -> Result<(), DmaLineDisplayError<CS::Error, DC::Error>> {
        if w == 0 || h == 0 {
            return Err(DmaLineDisplayError::InvalidRegion);
        }

        let ex = x
            .checked_add(w - 1)
            .ok_or(DmaLineDisplayError::InvalidRegion)?;
        let ey = y
            .checked_add(h - 1)
            .ok_or(DmaLineDisplayError::InvalidRegion)?;
        if ex >= self.width || ey >= self.height {
            return Err(DmaLineDisplayError::InvalidRegion);
        }

        let column_args = [(x >> 8) as u8, x as u8, (ex >> 8) as u8, ex as u8];
        let page_args = [(y >> 8) as u8, y as u8, (ey >> 8) as u8, ey as u8];

        self.write_command(SET_COLUMN_ADDRESS, &column_args)?;
        self.write_command(SET_PAGE_ADDRESS, &page_args)
    }

    fn flush_active_tile(&mut self) -> Result<(), DmaLineDisplayError<CS::Error, DC::Error>> {
        let Some(tile) = self.active.take() else {
            return Ok(());
        };
        if tile.lines == 0 {
            return Ok(());
        }

        self.wait_pending()?;
        self.set_address_window(tile.x, tile.y, tile.w, tile.lines)?;

        self.cs.set_low().map_err(DmaLineDisplayError::Cs)?;
        self.dc.set_low().map_err(DmaLineDisplayError::Dc)?;
        self.write_with_spare_buf(&[RAM_WRITE])?;
        self.dc.set_high().map_err(DmaLineDisplayError::Dc)?;

        let spi = self.spi.take().ok_or(DmaLineDisplayError::MissingBuffer)?;
        let mut render_buf = self
            .render_buf
            .take()
            .ok_or(DmaLineDisplayError::MissingBuffer)?;
        let byte_len = tile.byte_len();
        render_buf.set_length(byte_len);

        match spi.write(byte_len, render_buf) {
            Ok(transfer) => {
                self.pending = Some(transfer);
                self.render_buf = self.spare_buf.take();
                Ok(())
            }
            Err((error, returned_spi, returned_buf)) => {
                self.spi = Some(returned_spi);
                self.render_buf = Some(returned_buf);
                Err(DmaLineDisplayError::Spi(error))
            }
        }
    }

    fn can_extend_active_tile(&self, line: usize, range: &Range<usize>) -> bool {
        let Some(active) = self.active else {
            return false;
        };

        active.x as usize == range.start
            && active.w as usize == range.len()
            && active.next_y as usize == line
            && (active.lines as usize) < self.max_tile_lines
    }

    fn start_tile(
        &mut self,
        line: usize,
        range: &Range<usize>,
    ) -> Result<(), DmaLineDisplayError<CS::Error, DC::Error>> {
        if line >= self.height as usize || range.end > self.width as usize {
            return Err(DmaLineDisplayError::InvalidRegion);
        }

        let line_bytes = range.len() * BYTES_PER_PIXEL;
        let capacity = self
            .render_buf
            .as_ref()
            .ok_or(DmaLineDisplayError::MissingBuffer)?
            .capacity();
        if line_bytes > capacity {
            return Err(DmaLineDisplayError::BufferTooSmall);
        }

        self.active = Some(ActiveTile {
            x: range.start as u16,
            y: line as u16,
            w: range.len() as u16,
            lines: 0,
            next_y: line as u16,
        });
        Ok(())
    }
}

impl<CS, DC> LineBufferProvider for &mut DmaLineDisplay<'_, CS, DC>
where
    CS: OutputPin,
    DC: OutputPin,
{
    type TargetPixel = Rgb565PixelBE;

    fn process_line(
        &mut self,
        line: usize,
        range: Range<usize>,
        render_fn: impl FnOnce(&mut [Self::TargetPixel]),
    ) {
        if self.error.is_some() || range.is_empty() {
            return;
        }

        if !self.can_extend_active_tile(line, &range) {
            if let Err(error) = self.flush_active_tile() {
                self.set_error(error);
                return;
            }
            if let Err(error) = self.start_tile(line, &range) {
                self.set_error(error);
                return;
            }
        }

        let Some(active) = self.active.as_mut() else {
            self.set_error(DmaLineDisplayError::MissingBuffer);
            return;
        };
        let Some(render_buf) = self.render_buf.as_mut() else {
            self.set_error(DmaLineDisplayError::MissingBuffer);
            return;
        };

        let offset_pixels = active.lines as usize * active.w as usize;
        let end_pixels = offset_pixels + range.len();
        let end_bytes = end_pixels * BYTES_PER_PIXEL;
        if end_bytes > render_buf.capacity() {
            self.set_error(DmaLineDisplayError::BufferTooSmall);
            return;
        }

        let pixels = bytemuck::cast_slice_mut::<u8, Rgb565PixelBE>(
            &mut render_buf.as_mut_slice()[..end_bytes],
        );
        render_fn(&mut pixels[offset_pixels..end_pixels]);

        active.lines += 1;
        active.next_y = active.next_y.saturating_add(1);
    }
}
