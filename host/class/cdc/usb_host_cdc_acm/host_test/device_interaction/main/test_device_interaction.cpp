/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <catch2/catch_test_macros.hpp>

#include "descriptors/cdc_descriptors.hpp"
#include "usb/cdc_acm_host.h"
#include "mock_add_usb_device.h"
#include "common_test_fixtures.hpp"
#include "usb_helpers.h"
#include "cdc_host_descriptor_parsing.h"

extern "C" {
#include "Mockusb_host.h"
}

/**
 * @brief Add mocked devices
 *
 * Mocked devices are defined by a device descriptor, a configuration descriptor and a device address
 */
static void _add_mocked_devices(void)
{
    // Init mocked devices list at the beginning of the test
    usb_host_mock_dev_list_init();

    // Fill mocked devices list

    // ASIX Electronics Corp. AX88772A Fast Ethernet (FS descriptor)
    REQUIRE(ESP_OK == usb_host_mock_add_device(0, (const usb_device_desc_t *)premium_cord_device_desc_fs,
            (const usb_config_desc_t *)premium_cord_config_desc_fs));

    // ASIX Electronics Corp. AX88772B (FS descriptor)
    REQUIRE(ESP_OK == usb_host_mock_add_device(1, (const usb_device_desc_t *)i_tec_device_desc_fs,
            (const usb_config_desc_t *)i_tec_config_desc_fs));

    // FTDI chip dual (FS descriptor)
    REQUIRE(ESP_OK == usb_host_mock_add_device(2, (const usb_device_desc_t *)ftdi_device_desc_fs_hs,
            (const usb_config_desc_t *)ftdi_config_desc_fs));

    // TTL232RG (FS descriptor)
    REQUIRE(ESP_OK == usb_host_mock_add_device(3, (const usb_device_desc_t *)ttl232_device_desc,
            (const usb_config_desc_t *)ttl232_config_desc));

    // CP210x (FS descriptor)
    REQUIRE(ESP_OK == usb_host_mock_add_device(4, (const usb_device_desc_t *)cp210x_device_desc,
            (const usb_config_desc_t *)cp210x_config_desc));
}

/**
 * @brief Submit mock transfers to opened mocked device
 *
 * This function will submit couple of transfers to the mocked opened device, to test interaction of CDC-ACM host driver
 * with the mocked USB Host stack and mocked USB device
 *
 * @param[in] dev CDC handle obtained from cdc_acm_host_open()
 */
static void _submit_mock_transfer(cdc_acm_dev_hdl_t *dev)
{
    const uint8_t tx_buf[] = "HELLO";
    // Submit transfer successfully
    REQUIRE(ESP_OK == test_cdc_acm_host_data_tx_blocking(*dev, tx_buf, sizeof(tx_buf), 200, MOCK_USB_TRANSFER_SUCCESS));
    // Submit transfer which will fail to submit
    REQUIRE(ESP_ERR_INVALID_RESPONSE == test_cdc_acm_host_data_tx_blocking(*dev, tx_buf, sizeof(tx_buf), 200, MOCK_USB_TRANSFER_ERROR));
    // Submit transfer which times out
    REQUIRE(ESP_ERR_TIMEOUT == test_cdc_acm_host_data_tx_blocking(*dev, tx_buf, sizeof(tx_buf), 200, MOCK_USB_TRANSFER_TIMEOUT));
}

SCENARIO("Interact with mocked USB devices")
{
    // We put the device adding to the SECTION, to run it just once, not repeatedly for all the following SECTIONs
    SECTION("Add mocked devices") {

        _add_mocked_devices();

        // Optionally, print all the devices
        //Susb_host_mock_print_mocked_devices(0xFF);
    }

    GIVEN("Mocked devices are added to the device list") {
        // Install CDC-ACM driver

        REQUIRE(ESP_OK == test_cdc_acm_host_install(nullptr));

        cdc_acm_dev_hdl_t dev = nullptr;
        const cdc_acm_host_device_config_t dev_config = {
            .connection_timeout_ms = 1000,
            .out_buffer_size = 100,
            .in_buffer_size = 100,
            .event_cb = nullptr,
            .data_cb = nullptr,
            .user_arg = nullptr,
        };

        SECTION("Interact with device: ASIX Electronics Corp. AX88772A Fast Ethernet") {

            // Define details of a device which will be opened
            const uint16_t vid = 0xB95, pid = 0x772A;
            const uint8_t device_address = 0, interface_index = 0;

            // Open a device
            REQUIRE(ESP_OK == test_cdc_acm_host_open(device_address, vid, pid, interface_index, &dev_config, &dev));
            REQUIRE(dev != nullptr);
            // Interact with the device - submit mocked transfers
            _submit_mock_transfer(&dev);
            // Close the device
            REQUIRE(ESP_OK == test_cdc_acm_host_close(&dev, interface_index));
        }

        SECTION("Interact with device: ASIX Electronics Corp. AX88772B") {

            // Define details of a device which will be opened
            const uint16_t vid = 0xB95, pid = 0x772B;
            const uint8_t device_address = 1, interface_index = 0;

            // Open a device
            REQUIRE(ESP_OK == test_cdc_acm_host_open(device_address, vid, pid, interface_index, &dev_config, &dev));
            REQUIRE(dev != nullptr);
            // Interact with the device - submit mocked transfers
            _submit_mock_transfer(&dev);
            // Close the device
            REQUIRE(ESP_OK == test_cdc_acm_host_close(&dev, interface_index));
        }

        SECTION("Interact with device: FTDI chip dual (FS descriptor)") {

            // Define details of a device which will be opened
            const uint16_t vid = 0x403, pid = 0x6010;
            const uint8_t device_address = 2, interface_index = 0;

            // Open a device
            REQUIRE(ESP_OK == test_cdc_acm_host_open(device_address, vid, pid, interface_index, &dev_config, &dev));
            REQUIRE(dev != nullptr);
            // Interact with the device - submit mocked transfers
            _submit_mock_transfer(&dev);
            // Close the device
            REQUIRE(ESP_OK == test_cdc_acm_host_close(&dev, interface_index));
        }

        SECTION("Interact with device: TTL232RG (FS descriptor)") {

            // Define details of a device which will be opened
            const uint16_t vid = 0x403, pid = 0x6001;
            const uint8_t device_address = 3, interface_index = 0;

            // Open a device
            REQUIRE(ESP_OK == test_cdc_acm_host_open(device_address, vid, pid, interface_index, &dev_config, &dev));
            REQUIRE(dev != nullptr);
            // Interact with the device - submit mocked transfers
            _submit_mock_transfer(&dev);
            // Close the device
            REQUIRE(ESP_OK == test_cdc_acm_host_close(&dev, interface_index));
        }

        SECTION("Interact with device: CP210x") {

            // Define details of a device which will be opened
            const uint16_t vid = 0x10C4, pid = 0xEA60;
            const uint8_t device_address = 4, interface_index = 0;

            // Open a device
            REQUIRE(ESP_OK == test_cdc_acm_host_open(device_address, vid, pid, interface_index, &dev_config, &dev));
            REQUIRE(dev != nullptr);
            // Interact with the device - submit mocked transfers
            _submit_mock_transfer(&dev);
            // Close the device
            REQUIRE(ESP_OK == test_cdc_acm_host_close(&dev, interface_index));
        }

        // Uninstall CDC-ACM driver
        REQUIRE(ESP_OK == test_cdc_acm_host_uninstall());
    }
}