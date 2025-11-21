# ESP32 Room monitor

## Overview

A simple ESP32 room temperature and humidity monitor, which uses Wi-Fi to send the data to InfluxDB v2 over HTTP. Measurements are taken and stored at a one second interval. The buffered data is sent every minute. A watchdog restarts the device if it is stuck for more than 30 seconds.

## Hardware

- ESP32-DevKitC v4
- DS18B20 module (temperature)
- DHT11 module (temperature and humidity)

## Wiring

- DHT11 module connected to GPIO4, 5V and ground
- DS18B20 module connected to GPIO25, 5V and ground

## Crates

- embassy-net (Wi-Fi stack)
- reqwless (HTTP)
- embedded-dht-rs (DHT11)
- onewire (DS18B20)

## Usage

- Requires an existing InfluxDB v2 database
- Install necessary Rust toolchain for ESP32
- Rename src/bin/secrets_example to secrets and add tokens, etc. to mod.rs
- Run ``cargo run --release`` to flash and debug

## Issues

- Occasional sensor read errors when using Wi-Fi
- Repeating HTTP errors may cause device restart

## Images

![Screenshot](influxdb_screenshot.png)
*InfluxDB dashboard*

## License

[![MIT License](https://img.shields.io/badge/License-MIT-green.svg)](https://choosealicense.com/licenses/mit/)
