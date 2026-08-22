# EJECT-Cumpump
Eject Cumpump which works together with the OSSM, controlled by M5 Remote or Xtoys WS

## Overview

The EJECT-Cumpump controls the pump mechanism, providing precise movement and control for the EJECT system. You can control the cumpump using https://github.com/MarcoB979/OSSM-M5-Remote-BLE-added
Communication now uses BLE only. Previous versions which used ESP_NOW will not work after updating your M5 firmware to the FW mentioned above

## Integration

This firmware is designed to work together with the M5 remote and OSSM. Please refer to the documentation in those repositories for integration instructions.

Now also it has XTOYS integration: you can use the https://xtoys.app/scripts/EJECT-cumpump script for this. Connect this to a custom toy you created which has a Generic Input/Ouput type. Do not forget to enter your WIFI credentials and Websocket details of the toy:

## Setup

Before building, copy the secrets template and fill in your own WiFi and XToys credentials:

```
cp src/secrets.h.example src/secrets.h
```

Then edit `src/secrets.h` with your WiFi SSID/password and your XToys connection details (Custom Toy, Private Webhook, or Shared Webhook — see the comments in the file). `src/secrets.h` is gitignored and never committed, so your credentials stay local.

## Credits

- Forked from [ortlof/EJECT-cum-tube-project](https://github.com/ortlof/EJECT-cum-tube-project)
- Developed and maintained by MarcoB979
