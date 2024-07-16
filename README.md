# Nordic UART Service Project for BLE Communication between nRF52833 and ESP32

## Getting Started

### Prerequisites

- Download SEGGER from [here](https://www.segger.com/downloads/embedded-studio/)
- Follow this [video](https://www.youtube.com/watch?v=dChOb0Kd31A) to install the Espressif IDF VS Code Extension
- Download the nRF5 SDK compatible with your SEGGER version from [here](https://www.nordicsemi.com/Products/Development-software/nRF5-SDK)

### Setup

1. Navigate to your nRF5 SDK path and create a new folder named `myexp` or any name of your choice:
    ```sh
    cd /your_nrf_sdk_path/examples/myexp
    ```

    **Note:** This repository only includes the SoftDevice for nRF52833. For other PCA numbers, copy the SoftDevice files from the nRF5 SDK and modify the `main.c` functions accordingly.

2. Clone the repository:
    ```sh
    git clone https://github.com/Mio-Atse/nRF-to-ESP-BLE.git
    ```

## Flashing nRF52833 with SEGGER

1. Open SEGGER, select "Open Solution" and load the project file:
    ```sh
    myexp/nRF-to-ESP-BLE/pca10100/s113/ses/ble_app_uart_pca_10100_s113.emProject
    ```

2. Build the project and flash it to the nRF52833 using the "Connect J-Link" and "Download" options.

3. Use JlinkRTT Viewer to monitor the USB serial port and ensure the flashing process is successful.

## Flashing ESP32 with VSCode

1. Open the folder:
    ```sh
    myexp/nRF-to-ESP-BLE/spp_client
    ```

2. In the bottom section, select your ESP device and type. Then, run `>menuconfig` from the VSCode command palette.

3. Click on "Build, Flash and Monitor" from the menu.

4. If connected correctly, you should see the following log in the terminal:
    ```sh
    I (3411) NimBLE: Successfully subscribed to notifications
    ```

## Usage

- Press `1` or `2` to switch between data transmitted from nRF52833 to ESP32.
- Modify the `main.c` file on the nRF52833 according to your data transmission needs.
- The last data sent from nRF52833 can be saved to ESP Flash NVM and observed in terminal logs after a reset.
