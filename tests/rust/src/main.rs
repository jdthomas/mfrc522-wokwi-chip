use std::sync::Arc;

use anyhow::Result;
use esp_idf_hal::gpio::*;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use esp_idf_hal::spi::config;
use esp_idf_hal::spi::SpiDeviceDriver;
use esp_idf_hal::spi::SpiDriver;
use esp_idf_hal::spi::SpiDriverConfig;
use esp_idf_hal::spi::SPI2;
use esp_idf_sys as _;
use mfrc522::Mfrc522;

fn main() -> Result<()> {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take()?;

    let spi = peripherals.spi2;

    let sclk = peripherals.pins.gpio36;
    let sdi = peripherals.pins.gpio37; // SDI/MISO
    let sdo = peripherals.pins.gpio35; // SDO/MOSI
    let _rst = PinDriver::output(peripherals.pins.gpio40)?;

    let driver = Arc::new(SpiDriver::new::<SPI2>(
        spi,
        sclk,
        sdo,
        Some(sdi),
        &SpiDriverConfig::new(),
    )?);
    let rfid_driver = driver.clone();

    let config_1 = config::Config::new()
        .baudrate(26.MHz().into())
        .data_mode(config::MODE_3);

    let cs_rfid = peripherals.pins.gpio8;
    let rfid_device = SpiDeviceDriver::new(rfid_driver, Some(cs_rfid), &config_1)?;
    let itf = mfrc522::comm::blocking::spi::SpiInterface::new(rfid_device);
    let mut mfrc522 = Mfrc522::new(itf).init()?;
    let mfrc522_version = mfrc522.version()?;
    log::info!("VERSION: 0x{:x}", mfrc522_version);
    // assert!(mfrc522_version == 0x91 || mfrc522_version == 0x92);

    let mut card_uid = None;
    loop {
        log::info!("Ready: waiting for chip");
        loop {
            match mfrc522.wupa() {
                Ok(atqa) => {
                    log::info!("card detected");
                    let new_card_uid = match mfrc522.select(&atqa) {
                        Ok(ref _uid @ mfrc522::Uid::Single(ref inner)) => {
                            format_card(inner.as_bytes())
                        }
                        Ok(ref _uid @ mfrc522::Uid::Double(ref inner)) => {
                            format_card(inner.as_bytes())
                        }
                        Ok(ref _uid @ mfrc522::Uid::Triple(ref inner)) => {
                            format_card(inner.as_bytes())
                        }
                        Err(e) => {
                            log::error!("Select error: {e:?}");
                            continue;
                        }
                    };
                    if card_uid.as_ref() != Some(&new_card_uid) {
                        log::info!("Card changed: {}", new_card_uid);
                        card_uid = Some(new_card_uid);
                    }
                    break;
                }
                Err(e) => log::info!("wupa error {:?}", e),
            }

            std::thread::sleep(std::time::Duration::from_millis(100));
        }
        std::thread::sleep(std::time::Duration::from_millis(100));
    }

    // Ok(())
}

fn format_card(card: &[u8]) -> String {
    card.iter()
        .map(|b| format!("{:02x}", b))
        .collect::<Vec<_>>()
        .join(":")
}
