# nRF IEEE 802.15.4 radio driver

The nRF IEEE 802.15.4 radio driver implements IEEE 802.15.4 PHY layer on Nordic Semiconductor nRF52840 SoC.

The architecture of nRF IEEE 802.15.4 radio driver is OS and IEEE 802.15.4 MAC layer independent.
It allows to use the driver in any IEEE 802.15.4 based stacks that implement protocols such as Thread, ZigBee or RF4CE.

What is more it was designed to work with multiprotocol applications. The driver allows to share RADIO peripheral with other PHY protocol drivers e.g. Bluetooth LE.

For more information and detailed description of the driver, please refer to [Wiki page](https://github.com/NordicSemiconductor/nRF-IEEE-802.15.4-radio-driver/wiki).

Note that *nRF-IEEE-802.15.4-radio-driver.packsc* file is project building description used for internal testing in Nordic Semiconductor. This file is NOT needed to build the driver with any other tool.
