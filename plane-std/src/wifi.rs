use anyhow::{bail, Result};
use core::str;
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    hal::peripheral,
    wifi::{
        AccessPointConfiguration, AuthMethod, BlockingWifi, ClientConfiguration, Configuration,
        EspWifi,
    },
};
use log::info;

pub fn wifi(
    ssid: &str,
    pass: &str,
    modem: impl peripheral::Peripheral<P = esp_idf_svc::hal::modem::Modem> + 'static,
    sysloop: EspSystemEventLoop,
) -> Result<Box<EspWifi<'static>>> {
    let mut auth_method = AuthMethod::None;

    if ssid.is_empty() {
        bail!("Missing WiFi name")
    }
    if pass.is_empty() {
        auth_method = AuthMethod::None;
        info!("Wifi password is empty");
    }
    let mut esp_wifi = EspWifi::new(modem, sysloop.clone(), None)?;
    let mut wifi = BlockingWifi::wrap(&mut esp_wifi, sysloop)?;

    wifi.set_configuration(&Configuration::Client(ClientConfiguration::default()))?;
    info!("Starting wifi...");

    wifi.start()?;
    info!("Scanning...");
    let ap_infos = wifi.scan()?;
    let ours = ap_infos.into_iter().find(|a| a.ssid == ssid);
    let channel = if let Some(ours) = ours {
        info!(
            "Found configured access point {} on channel {}",
            ssid, ours.channel
        );
        Some(ours.channel)
    } else {
        info!(
            "Configured access point {} not found during scanning, will go with unknown channel",
            ssid
        );
        None
    };
    wifi.set_configuration(&Configuration::Client(ClientConfiguration {
        ssid: ssid.try_into().unwrap(),
        password: pass.try_into().unwrap(),
        channel,
        auth_method,
        ..Default::default()
    }))?;

    info!("Connecting wifi...");
    wifi.connect()?;

    info!("Waiting for DHCP lease...");
    wifi.wait_netif_up()?;

    let ip_info = wifi.wifi().sta_netif().get_ip_info()?;

    info!("Wifi DHCP info: {:?}", ip_info);

    Ok(Box::new(esp_wifi))
}

pub fn wifi_ap(
    ssid: &str,
    pass: &str,
    modem: impl peripheral::Peripheral<P = esp_idf_svc::hal::modem::Modem> + 'static,
    sysloop: EspSystemEventLoop,
) -> Result<Box<EspWifi<'static>>> {
    let mut auth_method = AuthMethod::WPA;

    if ssid.is_empty() {
        bail!("Missing WiFi name")
    }
    if pass.is_empty() {
        auth_method = AuthMethod::None;
        info!("Wifi password is empty");
    }
    let mut esp_wifi = EspWifi::new(modem, sysloop.clone(), None)?;
    let mut wifi = BlockingWifi::wrap(&mut esp_wifi, sysloop)?;

    wifi.set_configuration(&Configuration::AccessPoint(AccessPointConfiguration {
        ssid: ssid.try_into().unwrap(),
        password: pass.try_into().unwrap(),
        auth_method,
        max_connections: 1,
        ..Default::default()
    }))?;
    info!("Starting wifi...");

    wifi.start()?;
    // info!("Scanning...");
    // let ap_infos = wifi.scan()?;
    // let ours = ap_infos.into_iter().find(|a| a.ssid == ssid);
    // let channel = if let Some(ours) = ours {
    //     info!(
    //         "Found configured access point {} on channel {}",
    //         ssid, ours.channel
    //     );
    //     Some(ours.channel)
    // } else {
    //     info!(
    //         "Configured access point {} not found during scanning, will go with unknown channel",
    //         ssid
    //     );
    //     None
    // };
    // wifi.set_configuration(&Configuration::Client(ClientConfiguration {
    //     ssid: ssid.try_into().unwrap(),
    //     password: pass.try_into().unwrap(),
    //     channel,
    //     auth_method,
    //     ..Default::default()
    // }))?;

    // info!("Connecting wifi...");
    // wifi.connect()?;

    info!("Waiting for server to start...");
    wifi.wait_netif_up()?;

    // let ip_info = wifi.wifi().sta_netif().get_ip_info()?;

    // info!("Wifi DHCP info: {:?}", ip_info);

    Ok(Box::new(esp_wifi))
}
