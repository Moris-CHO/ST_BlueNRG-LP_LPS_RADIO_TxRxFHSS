## ST_BlueNRG-LP_LPS RADIO TX/RX example with FHSS
Example of RADIO RX/TX using FHSS (Frequency-hopping spread spectrum) on BlueNRG-LP_LPS. And, This example was implemented based on BlueNRG-LP_LPS DK 1.5.0. And, Comfimed based on this DK version. You can see RADIO TX/RX wtih simple FHSS(Frequency hopping).
(Download BlueNRG-LP_LPS DK: https://www.st.com/en/embedded-software/stsw-bnrglp-dk.html)


## Preparations
* BlueNRG-LP_LPS DK (recommended version is 1.5.0)
* Wise-Studio IDE Tool (https://www.st.com/en/embedded-software/stsw-wise-studio.html)
* 2x STEVAL-IDB011V1  or  2x STEVAL-IDB012V1

## How to compile it?
1. Download this repository
2. Copy it to "\BlueNRG-LP_LPS DK 1.5.0\Projects\Periph_Examples\MIX\RADIO"
3. You can compile it in RADIO_TxRxFHSS folder.

## Example description
* TX - TX device will send a packet with the below structure. TX device will go to next channel according to channel map after get ACK from RX device.
* RX - RX device will wait a packet from TX device at the start channel. RX device will go to next channel according to channel map after get a packet from TX device.
