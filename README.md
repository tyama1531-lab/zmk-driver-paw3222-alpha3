[ English | [Japanese](README-j.md) ]

---

<div align="center">
    <h1>ZMK PAW3222 Driver</h1>
        <a href="https://badgen.net/badge/ZMK/v0.3.0/blue" alt="ZMK">
            <img src="https://badgen.net/badge/ZMK/v0.3.0/blue" />
        </a>
        <a href="https://badgen.net/badge/License/Apache-2.0">
            <img src="https://badgen.net/badge/License/Apache-2.0" />
        </a>
        <a href="https://deepwiki.com/nuovotaka/zmk-driver-paw3222-alpha" alt="Ask DeepWiki">
        <img src="https://deepwiki.com/badge.svg" />
    </a>
    <hr>
    <p align="center">
        This driver enables use of the PIXART PAW3222 optical sensor with the ZMK framework.
    </p>
</div>

---

## Features

- SPI communication with the PAW3222 sensor
- Supports cursor movement, vertical/horizontal scrolling, and snipe (precision) mode
- Layer-based input mode switching (move, scroll, snipe, scroll-snipe)
- High-precision scroll modes with configurable sensitivity
- Runtime CPI (resolution) adjustment
- Power management and low-power modes
- Optional power GPIO support
- **Toggle-Based Mode Switching:** Three independent toggle functions for flexible mode control:
  - **Move/Scroll Toggle:** Switch between cursor movement and scroll modes
  - **Normal/Snipe Toggle:** Enable/disable high-precision (snipe) mode for both cursor and scroll
  - **Vertical/Horizontal Toggle:** Switch scroll direction between vertical and horizontal
- **Behavior API Integration:** Implements Zephyr's behavior driver API for seamless key binding

---

## Overview

The PAW3222 is a low-power optical mouse sensor suitable for tracking applications such as mice and trackballs. This driver communicates with the PAW3222 sensor via SPI interface. It supports flexible configuration via devicetree and Kconfig, and enables advanced usage such as layer-based input mode switching and runtime configuration.

---

## Installation

- Add as a ZMK module in your `west.yml`:

```yaml
manifest:
  remotes:
    - name: zmkfirmware
      url-base: https://github.com/zmkfirmware
    - name: nuovotaka
      url-base: https://github.com/nuovotaka
  projects:
    - name: zmk
      remote: zmkfirmware
      revision: v0.3.0
      import: app/west.yml
    - name: zmk-driver-paw3222-alpha
      remote: nuovotaka
      revision: main
```

---

## Device Tree Configuration

Configure the sensor in your shield or board config file (`.overlay` or `.dtsi`):

<details>
<summary style="cursor:pointer; font-weight:bold;">Sample Code</summary>

```dts
&pinctrl {
    spi0_default: spi0_default {
        group1 {
            psels = <NRF_PSEL(SPIM_SCK, 0, 12)>,
                <NRF_PSEL(SPIM_MOSI, 1, 9)>,
                <NRF_PSEL(SPIM_MISO, 1, 9)>;
        };
    };

    spi0_sleep: spi0_sleep {
        group1 {
            psels = <NRF_PSEL(SPIM_SCK, 0, 12)>,
                <NRF_PSEL(SPIM_MOSI, 1, 9)>,
                <NRF_PSEL(SPIM_MISO, 1, 9)>;
            low-power-enable;
        };
    };
};

&spi0 {
    status = "okay";
    compatible = "nordic,nrf-spim";
    pinctrl-0 = <&spi0_default>;
    pinctrl-1 = <&spi0_sleep>;
    pinctrl-names = "default", "sleep";
    cs-gpios = <&gpio0 13 GPIO_ACTIVE_LOW>;

    trackball: trackball@0 {
        status = "okay";
        compatible = "pixart,paw3222";
        reg = <0>;
        spi-max-frequency = <2000000>;
        irq-gpios = <&gpio0 15 GPIO_ACTIVE_LOW>;

        /* Optional features */
        // rotation = <0>;  　   // default:0　(0, 90, 180, 270)
        // scroll-tick = <10>;  // default:10
        // snipe-divisor = <2>; // default:2 (configurable via Kconfig)
        // snipe-layers = <5>;
        // scroll-layers = <6>;
        // scroll-horizontal-layers = <7>;
        // scroll-snipe-layers = <8>
        // scroll-horizontal-snipe-layers = <9>;
        // scroll-snipe-divisor = <3>; // default:3 (configurable via Kconfig)
        // scroll-snipe-tick = <20>;   // default:20 (configurable via Kconfig)

        // Alternative: Use behavior-based switching instead of layers
        // switch-method = "toggle";   // "layer", "toggle"
    };
};
```

</details>

---

## Properties

| Property Name                  | Type          | Required | Description                                                                                                                                                          |
| ------------------------------ | ------------- | -------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| irq-gpios                      | phandle-array | Yes      | GPIO connected to the motion pin, active low.                                                                                                                        |
| power-gpios                    | phandle-array | No       | GPIO connected to the power control pin.                                                                                                                             |
| res-cpi                        | int           | No       | CPI resolution for the sensor (608-4826). Can also be changed at runtime using the `paw32xx_set_resolution()` API.                                                   |
| force-awake                    | boolean       | No       | Initialize the sensor in "force awake" mode. Can also be enabled/disabled at runtime via the `paw32xx_force_awake()` API.                                            |
| rotation                       | int           | No       | Physical rotation of the sensor in degrees. (0, 90, 180, 270). Used for scroll direction mapping. For cursor movement, use input-processors like `zip_xy_transform`. |
| scroll-tick                    | int           | No       | Threshold for scroll movement (delta value above which scroll is triggered). Used by normal scroll and horizontal scroll modes only.                                 |
| snipe-divisor                  | int           | No       | Divisor for cursor snipe mode sensitivity (higher values = lower sensitivity). Used by cursor snipe mode only, not scroll modes.                                     |
| snipe-layers                   | array         | No       | List of layer numbers to switch between using the snipe-layers feature.                                                                                              |
| scroll-layers                  | array         | No       | List of layer numbers to switch between using the scroll-layers feature.                                                                                             |
| scroll-horizontal-layers       | array         | No       | List of layer numbers to switch between using the horizontal scroll feature.                                                                                         |
| scroll-snipe-layers            | array         | No       | List of layer numbers to switch between using the high-precision vertical scroll feature.                                                                            |
| scroll-horizontal-snipe-layers | array         | No       | List of layer numbers to switch between using the high-precision horizontal scroll feature.                                                                          |
| scroll-snipe-divisor           | int           | No       | Divisor for scroll snipe mode sensitivity (higher values = lower sensitivity). Used by scroll snipe modes only.                                                      |
| scroll-snipe-tick              | int           | No       | Threshold for scroll movement in snipe mode (higher values = less sensitive scrolling). Used by scroll snipe modes only.                                             |

---

## Kconfig

Enable the module in your keyboard's `Kconfig.defconfig`:

```kconfig
if ZMK_KEYBOARD_YOUR_KEYBOARD

config ZMK_POINTING
    default y

config PAW3222
    default y

config PAW3222_BEHAVIOR
    default y

endif
```

Also, make sure to add the following line to your `.conf` file to enable input support:

```
CONFIG_INPUT=y
```

---

## Usage

- The driver automatically switches input mode (move, scroll, snipe) based on the active ZMK layer and your devicetree configuration.
- You can adjust CPI (resolution) at runtime using the API (see below).
- Use `rotation` to ensure scroll always works with y-axis movement regardless of sensor orientation. For cursor movement rotation, use ZMK input-processors like `zip_xy_transform`.
- Configure `scroll-tick` to tune scroll sensitivity.

---

## API Reference

### Change CPI (Resolution)

```c
int paw32xx_set_resolution(const struct device *dev, uint16_t res_cpi);
```

- Changes sensor resolution at runtime.
- Supported CPI range: 608-4826 (hardware limitation)
- CPI values are in steps of 38

### Force Awake Mode

```c
int paw32xx_force_awake(const struct device *dev, bool enable);
```

- Enables/disables "force awake" mode at runtime.

---

## Behavior-Based Mode Switching

Instead of using empty layers, you can use ZMK behaviors to switch input modes:

### Keymap Configuration

<details>
<summary style="cursor:pointer; font-weight:bold;">Sample Code</summary>

```dts
/ {
    behaviors {
        paw_mode: paw_mode {
            compatible = "paw32xx,mode";
            label = "PAW_MODE";
            #binding-cells = <1>;
        };
    };

    keymap {
        compatible = "zmk,keymap";

        default_layer {
            bindings = <
                // Toggle between move and scroll modes
                &paw_mode 0

                // Toggle between normal and snipe modes
                &paw_mode 1

                // Toggle between Vertical and Horizontal modes
                &paw_mode 2
            >;
        };
    };
};
```

</details>

### Device Tree Configuration

<details>
<summary style="cursor:pointer; font-weight:bold;">Sample Code</summary>

```dts
trackball: trackball@0 {
    compatible = "pixart,paw3222";
    // ... other properties ...

    // Use behavior-based switching
    switch-method = "toggle";

    // Layer-based properties are ignored when using behavior switching
};
```

</details>

### Complete Example

<details>
<summary style="cursor:pointer; font-weight:bold;">Sample Code</summary>

```dts
// In your .overlay file
&spi0 {
    trackball: trackball@0 {
        compatible = "pixart,paw3222";
        reg = <0>;
        spi-max-frequency = <2000000>;
        irq-gpios = <&gpio0 15 GPIO_ACTIVE_LOW>;

        // Use behavior-based switching
        switch-method = "toggle";

        // Sensitivity settings
        res-cpi = <1200>;
        scroll-tick = <10>;
        snipe-divisor = <2>;
        scroll-snipe-divisor = <3>;
        scroll-snipe-tick = <20>;
    };
};

// In your .keymap file
/ {
    behaviors {
        paw_mode: paw_mode {
            compatible = "paw32xx,mode";
            #binding-cells = <1>;
        };
    };

    keymap {
        compatible = "zmk,keymap";

        default_layer {
            bindings = <
                &kp Q &kp W &kp E &kp R &kp T
                &kp A &kp S &kp D &kp F &kp G
                &kp Z &kp X &paw_mode 0 &paw_mode 1 &paw_mode 2
                //           ↑Move/Scroll   ↑Normal/Snipe  ↑Vertical/Horizontal
            >;
        };
    };
};
```

</details>

## Mode Switching Functions

The driver provides three independent toggle functions that can be combined for flexible mode control:

1. **Move/Scroll Toggle (Parameter 0):**

   - Switches between cursor movement (MOVE/SNIPE) and scroll modes (SCROLL/SCROLL_SNIPE)
   - When in MOVE or SNIPE mode: switches to SCROLL mode
   - When in any SCROLL mode: switches back to MOVE mode

2. **Normal/Snipe Toggle (Parameter 1):**

   - Toggles high-precision (snipe) mode on/off for current operation type
   - MOVE ↔ SNIPE (for cursor movement)
   - SCROLL ↔ SCROLL_SNIPE (for vertical scrolling)
   - SCROLL_HORIZONTAL ↔ SCROLL_HORIZONTAL_SNIPE (for horizontal scrolling)

3. **Vertical/Horizontal Toggle (Parameter 2):**
   - Switches scroll direction between vertical and horizontal
   - Only works when already in a scroll mode (not MOVE/SNIPE)
   - SCROLL ↔ SCROLL_HORIZONTAL
   - SCROLL_SNIPE ↔ SCROLL_HORIZONTAL_SNIPE

### Mode Combinations

By combining these toggles, you can access all six available modes:

- **MOVE:** Default cursor movement
- **SNIPE:** High-precision cursor movement
- **SCROLL:** Vertical scrolling
- **SCROLL_SNIPE:** High-precision vertical scrolling
- **SCROLL_HORIZONTAL:** Horizontal scrolling
- **SCROLL_HORIZONTAL_SNIPE:** High-precision horizontal scrolling

## Usage

- The driver is activated via key bindings, with each binding parameter corresponding to a mode-switch function.
- Logging provides feedback for mode changes and errors.
- The device reference is set during initialization.

## Initialization

The driver is initialized automatically if the device tree is configured correctly and the `CONFIG_PAW3222_BEHAVIOR` option is enabled.

```
CONFIG_PAW3222_BEHAVIOR=y

```

---

## Split Keyboard Configuration

For split keyboards with separate MCUs, you can configure different trackball behaviors for left and right sides.

### Independent Trackball Configuration

Each side can have completely different settings by configuring separate PAW3222 instances:

#### Left Side Configuration (left.overlay)

<details>
<summary style="cursor:pointer; font-weight:bold;">Sample Code</summary>

```dts
&spi0 {
    trackball_left: trackball@0 {
        compatible = "pixart,paw3222";
        reg = <0>;
        spi-max-frequency = <2000000>;
        irq-gpios = <&gpio0 15 GPIO_ACTIVE_LOW>;

        // Left side: Optimized for cursor movement
        res-cpi = <800>;           // Lower sensitivity
        scroll-tick = <15>;        // Less sensitive scrolling
        snipe-divisor = <3>;       // High precision mode
        switch-method = "toggle";
    };
};
```

</details>

#### Right Side Configuration (right.overlay)

<details>
<summary style="cursor:pointer; font-weight:bold;">Sample Code</summary>

```dts
&spi0 {
    trackball_right: trackball@0 {
        compatible = "pixart,paw3222";
        reg = <0>;
        spi-max-frequency = <2000000>;
        irq-gpios = <&gpio0 15 GPIO_ACTIVE_LOW>;

        // Right side: Optimized for scrolling
        res-cpi = <1200>;          // Higher sensitivity
        scroll-tick = <8>;         // More sensitive scrolling
        snipe-divisor = <2>;       // Standard precision
        switch-method = "toggle";
    };
};
```

</details>

### Different Behavior Assignments

You can assign different mode switching behaviors to each side:

#### Left Side Keymap

<details>
<summary style="cursor:pointer; font-weight:bold;">Sample Code</summary>

```dts
/ {
    keymap {
        compatible = "zmk,keymap";

        default_layer {
            bindings = <
                &kp Q &kp W &kp E
                &kp A &paw_mode 0 &kp D  // Left: Move/Scroll toggle only
                &kp Z &kp X &kp C
            >;
        };
    };
};
```

</details>

#### Right Side Keymap

<details>
<summary style="cursor:pointer; font-weight:bold;">Sample Code</summary>

```dts
/ {
    keymap {
        compatible = "zmk,keymap";

        default_layer {
            bindings = <
                &kp R &kp T &kp Y
                &kp F &paw_mode 1 &kp H  // Right: Normal/Snipe toggle only
                &kp V &kp B &kp N
            >;
        };
    };
};
```

</details>

### Use Case Examples

1. **Specialized Roles:**

   - Left trackball: Cursor movement only (low sensitivity)
   - Right trackball: Scrolling only (high sensitivity)

2. **Complementary Functions:**

   - Left trackball: Coarse movement and vertical scrolling
   - Right trackball: Fine movement and horizontal scrolling

3. **Mode-Specific Optimization:**
   - Left trackball: Optimized for design work (high precision)
   - Right trackball: Optimized for browsing (fast scrolling)

### Configuration Tips

- Use different `res-cpi` values to optimize each side for its intended use
- Adjust `scroll-tick` independently for different scrolling behaviors
- Configure `snipe-divisor` based on precision requirements for each side
- Each MCU maintains its own state, allowing completely independent operation

---

## Troubleshooting

- If the sensor does not work, check SPI and GPIO wiring.
- Confirm `irq-gpios` and (if used) `power-gpios` are correct.
- Use Zephyr logging to check for errors at boot.
- Ensure the ZMK version matches the required version.

---

## License

```
SPDX-License-Identifier: Apache-2.0

Copyright 2024 Google LLC
Modifications Copyright 2025 sekigon-gonnoc
Modifications Copyright 2025 nuovotaka
```
