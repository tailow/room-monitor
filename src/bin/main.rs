#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

mod secrets;

use alloc::string::String;
use embassy_net::DhcpConfig;
use embedded_dht_rs::dht11::Dht11;
use esp_hal::{
    clock::CpuClock,
    delay::Delay,
    gpio::{DriveMode, Level, Output, OutputConfig, Pull},
    main,
    time::Duration,
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_radio::{ble::controller::BleConnector, wifi::ClientConfig};
use onewire::{self, DS18B20, DeviceSearch, OneWire};

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    let mut delay = Delay::new();

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

    let wifi_ssid = String::from(secrets::WIFI_SSID);
    let wifi_password = String::from(secrets::WIFI_PASS);

    let wifi_config = esp_radio::wifi::ModeConfig::Client(
        ClientConfig::default()
            .with_ssid(wifi_ssid)
            .with_password(wifi_password),
    );

    _wifi_controller
        .set_config(&wifi_config)
        .expect("Couldn't set wifi config");

    _wifi_controller
        .start()
        .expect("Coudln't start wifi controller");

    _wifi_controller
        .connect()
        .expect("Couldn't connect to wifi");

    loop {
        if _wifi_controller.is_connected().expect("Error") {
            break;
        }

        log::info!("Connecting to WiFi");

        delay.delay(Duration::from_millis(500));
    }

    log::info!("WiFi connected");

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
        peripherals.GPIO25,
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
            let Ok(_) = sensor
                .measure_temperature(&mut wire, &mut delay)
                .inspect_err(|_| log::error!("Failed to start temperature measurement"))
            else {
                continue 'measure;
            };

            delay.delay(Duration::from_millis(2000));

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

            match dht11.read() {
                Ok(sensor_reading) => println!(
                    "T1:{},T2:{:.4},H:{}",
                    sensor_reading.temperature, temperature, sensor_reading.humidity
                ),
                Err(error) => {
                    log::error!("An error occurred while trying to read sensor: {:?}", error)
                }
            }
        }
    }
}
