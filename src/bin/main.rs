#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

mod secrets;

use alloc::string::String;
use embassy_executor::Spawner;

use embassy_net::{Runner, StackResources};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};

use embassy_time::{Duration, Timer};

use embedded_dht_rs::dht11::Dht11;

use esp_hal::{
    clock::CpuClock,
    delay::Delay,
    gpio::{DriveMode, Level, Output, OutputConfig, Pull},
    rng::Rng,
    timer::timg::TimerGroup,
};

use esp_radio::{
    Controller,
    wifi::{ClientConfig, ModeConfig, WifiController, WifiDevice, WifiError, WifiEvent},
};
use esp_rtos::main;

use esp_println::println;

use onewire::{self, DS18B20, DeviceSearch, OneWire};

use static_cell::StaticCell;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

static STACK_RESOURCES: StaticCell<embassy_net::StackResources<3>> = StaticCell::new();
static WIFI_CONTROLLER: StaticCell<Controller<'static>> = StaticCell::new();
pub static STOP_WIFI_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

#[main]
async fn main(spawner: Spawner) -> ! {
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
    let radio_init: &'static mut _ = WIFI_CONTROLLER.init(radio_init);

    let (wifi_controller, interfaces) =
        esp_radio::wifi::new(radio_init, peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");

    let config = embassy_net::Config::dhcpv4(Default::default());

    let stack_resources: &'static mut _ = STACK_RESOURCES.init(StackResources::new());

    let (stack, runner) = embassy_net::new(
        interfaces.sta,
        config,
        stack_resources,
        Rng::new().random() as u64,
    );

    let wifi_ssid = String::from(secrets::WIFI_SSID);
    let wifi_password = String::from(secrets::WIFI_PASS);

    spawner.must_spawn(connection(wifi_controller, wifi_ssid, wifi_password));
    spawner.must_spawn(net_task(runner));

    loop {
        if stack.is_link_up() {
            break;
        }

        log::info!("Wait for network link");

        Timer::after_millis(500).await;
    }

    log::info!("Wait for IP address");

    loop {
        if let Some(config) = stack.config_v4() {
            log::info!("Connected to WiFi with IP address {}", config.address);

            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    let dht11_pin = Output::new(
        peripherals.GPIO4,
        Level::High,
        OutputConfig::default()
            .with_drive_mode(DriveMode::OpenDrain)
            .with_pull(Pull::None),
    )
    .into_flex();

    dht11_pin.peripheral_input();

    let mut dht11 = Dht11::new(dht11_pin, delay);

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
            Timer::after_millis(1000).await;
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

            Timer::after_millis(2000).await;

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

/// Task for WiFi connection
///
/// This will wrap [`connection_fallible()`] and trap any error.
#[embassy_executor::task]
async fn connection(controller: WifiController<'static>, ssid: String, password: String) {
    if let Err(error) = connection_fallible(controller, ssid, password).await {
        log::error!("Cannot connect to WiFi: {error:?}");
    }
}

/// Fallible task for WiFi connection
async fn connection_fallible(
    mut controller: WifiController<'static>,
    ssid: String,
    password: String,
) -> Result<(), Error> {
    log::info!("Start connection");
    log::info!("Device capabilities: {:?}", controller.capabilities());

    loop {
        if controller.is_connected().expect("Connection check error") {
            // wait until we're no longer connected
            controller.wait_for_event(WifiEvent::StaDisconnected).await;

            Timer::after(Duration::from_millis(5000)).await;
        }

        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = ModeConfig::Client(
                ClientConfig::default()
                    .with_ssid(ssid.clone())
                    .with_password(password.clone()),
            );

            controller
                .set_config(&client_config)
                .expect("Failed to set config");

            log::info!("Starting WiFi controller");

            controller.start_async().await?;

            log::info!("WiFi controller started");
        }

        log::info!("Connect to WiFi network");

        match controller.connect_async().await {
            Ok(()) => {
                log::info!("Connected to WiFi network");

                log::info!("Wait for request to stop wifi");

                STOP_WIFI_SIGNAL.wait().await;

                log::info!("Received signal to stop wifi");

                controller.stop_async().await?;
                break;
            }
            Err(error) => {
                log::error!("Failed to connect to WiFi network: {error:?}");

                Timer::after(Duration::from_millis(5000)).await;
            }
        }
    }

    log::info!("Leave connection task");

    Ok(())
}

/// Task for ongoing network processing
#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await;
}

/// Error within WiFi connection
#[derive(Debug)]
pub enum Error {
    /// Error during WiFi operation
    Wifi(#[expect(unused, reason = "Never read directly")] WifiError),
}

impl From<WifiError> for Error {
    fn from(error: WifiError) -> Self {
        Self::Wifi(error)
    }
}
