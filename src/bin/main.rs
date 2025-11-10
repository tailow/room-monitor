#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use embedded_dht_rs::dht11::Dht11;
use esp_hal::{
    clock::CpuClock,
    delay::Delay,
    gpio::{DriveMode, Level, Output, OutputConfig, Pull},
    main,
    time::Duration,
    timer::timg::TimerGroup,
};
use esp_radio::ble::controller::BleConnector;
use onewire::{self, DS18B20, DeviceSearch, OneWire};

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[unsafe(link_section = ".dram2_uninit")] size: 98767);
    // COEX needs more RAM - so we've added some more
    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);

    esp_rtos::start(timg0.timer0);

    let radio_init = esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller");
    let (mut _wifi_controller, _interfaces) =
        esp_radio::wifi::new(&radio_init, peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");
    let _connector = BleConnector::new(&radio_init, peripherals.BT, Default::default());

    let mut delay = Delay::new();

    let od_for_dht11 = Output::new(
        peripherals.GPIO4,
        Level::High,
        OutputConfig::default()
            .with_drive_mode(DriveMode::OpenDrain)
            .with_pull(Pull::None),
    )
    .into_flex();

    od_for_dht11.peripheral_input();

    let mut dht11 = Dht11::new(od_for_dht11, delay);

    let mut onewire_pin = Output::new(
        peripherals.GPIO16,
        Level::Low,
        OutputConfig::default().with_drive_mode(DriveMode::OpenDrain),
    )
    .into_flex();

    onewire_pin.set_input_enable(true);

    let mut wire = OneWire::new(onewire_pin, false);

    let mut first_iteration = true;

    'infinite: loop {
        // Just a little trick to prevent spam, in case we need to restart this loop
        if !first_iteration {
            delay.delay(Duration::from_secs(1)); // Prevent spam
        } else {
            first_iteration = false;
        }

        // Reset to test if wire is okay and if any sensor is connected
        if wire
            .reset(&mut delay)
            .inspect_err(|_| log::error!("Failed to reset wire"))
            .is_err()
        {
            continue 'infinite;
        }

        // Start searching for a sensor (we just care to get the first one)
        let mut search = DeviceSearch::new();
        let Ok(device) = wire
            .search_next(&mut search, &mut delay)
            .inspect_err(|_| log::error!("Failed to search for temperature sensor"))
        else {
            continue;
        };
        let Some(device) = device else {
            log::info!("No temperature sensor found");
            continue;
        };

        log::info!("Found temperature sensor: {:?}", device);

        // Construct the sensor driver
        let Ok(sensor) = DS18B20::new(device)
            .inspect_err(|_| log::error!("Failed to create temperature sensor"))
        else {
            continue;
        };

        'measure: loop {
            let Ok(resolution) = sensor
                .measure_temperature(&mut wire, &mut delay)
                .inspect_err(|_| log::error!("Failed to start temperature measurement"))
            else {
                continue 'measure;
            };

            delay.delay(Duration::from_millis(resolution.time_ms() as u64));

            // Retrieve the measured temperature from the sensor
            let Ok(raw_temperature) = sensor
                .read_temperature(&mut wire, &mut delay)
                .inspect_err(|_| log::error!("Failed to read temperature"))
            else {
                continue 'measure;
            };

            // Process and log the temperature
            let (integer, fraction) = onewire::ds18b20::split_temp(raw_temperature);
            let temperature = (integer as f32) + (fraction as f32) / 10000.0;

            log::info!("DS18B20 - Temperature: {} °C", temperature);

            match dht11.read() {
                Ok(sensor_reading) => log::info!(
                    "DHT 11 - Temperature: {} °C, humidity: {} %",
                    sensor_reading.temperature,
                    sensor_reading.humidity
                ),
                Err(error) => {
                    log::error!("An error occurred while trying to read sensor: {:?}", error)
                }
            }
        }
    }
}
