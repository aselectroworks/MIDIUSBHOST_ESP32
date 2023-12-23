#include <stdlib.h>
#include <string.h>

#include "MIDIUSB_Defs.h"
#include "esp32-hal-log.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "usb/usb_host.h"

#define CLIENT_NUM_EVENT_MSG 5

#define ACTION_OPEN_DEV 0x01
#define ACTION_GET_DEV_INFO 0x02
#define ACTION_GET_DEV_DESC 0x04
#define ACTION_GET_CONFIG_DESC 0x08
#define ACTION_GET_STR_DESC 0x10
#define ACTION_CLOSE_DEV 0x20
#define ACTION_EXIT 0x40

#define CLASS_AUDIO 0x01

#define SUBCLASS_UNDEFINED 0x00
#define SUBCLASS_AUDIOCONTROL 0x01
#define SUBCLASS_AUDIOSTREAMING 0x02
#define SUBCLASS_MIDISTREAMING 0x03

#define AUDIO_CS_ENDPOINT 0x25

#define USB_MIDI_CS_EP_DESC_SIZE 5

typedef union {
    struct {
        uint8_t bLength;
        uint8_t bDescriptorType;
        uint8_t bDescriptorSubtype;
        uint8_t bNumEmbMIDIJack;
        uint16_t BaAssocJackID[];
    } USB_DESC_ATTR;
    uint8_t val[USB_MIDI_CS_EP_DESC_SIZE];
} usb_midi_cs_ep_desc_t;
ESP_STATIC_ASSERT(sizeof(usb_midi_cs_ep_desc_t) == USB_MIDI_CS_EP_DESC_SIZE, "Size of usb_midi_cs_ep_desc_t incorrect");

typedef struct {
    usb_host_client_handle_t client_hdl;
    uint8_t dev_addr;
    usb_device_handle_t dev_hdl;
    uint32_t actions;
} class_driver_t;

usb_transfer_t *MidiIn = NULL;
usb_transfer_t *MidiOut = NULL;

#define MIDI_BUFFER_SIZE 64
typedef struct {
    midiEventPacket_t midiEvent[MIDI_BUFFER_SIZE];
    uint16_t head;
    uint16_t tail;
} midi_ring_buffer_t;
midi_ring_buffer_t midi_rx_buffer = {{0, 0, 0, 0}, 0, 0};

static const char *TAG = "MIDICLASS";

static void midi_transfer_cb(usb_transfer_t *transfer) {
    //ESP_LOGI(TAG, "midi_transfer_cb");
    if (transfer->status == USB_TRANSFER_STATUS_COMPLETED && USB_EP_DESC_GET_EP_DIR(transfer)) {
        // MIDI IN Transfer
        uint8_t *const data = transfer->data_buffer;
        for (uint8_t i = 0; i < transfer->actual_num_bytes; i += 4) {
            if (data[i] + data[i + 1] + data[i + 2] + data[i + 3] == 0) break;
            ESP_LOGI(TAG, "MIDI IN : %02x %02x %02x %02x", data[i], data[i + 1], data[i + 2], data[i + 3]);
            // Send to MIDI In Buffer
            uint8_t head = (midi_rx_buffer.head + 1) % MIDI_BUFFER_SIZE;
            if (midi_rx_buffer.tail != head) {
                ESP_LOGI(TAG, "Buffer: head = %d, tail = %d", head, midi_rx_buffer.tail);
                midiEventPacket_t midi_data = {data[i], data[i + 1], data[i + 2], data[i + 3]};
                midi_rx_buffer.midiEvent[midi_rx_buffer.head] = midi_data;
                midi_rx_buffer.head = head;
            }
        }
        esp_err_t err = usb_host_transfer_submit(transfer);
        if (err != ESP_OK) {
            ESP_LOGI(TAG, "MIDI IN Transfer Submit : fail");
        }
    } else {
        // MIDI OUT Transfer
        ESP_LOGI(TAG, "MIDI OUT : status = %d", transfer->status);
    }
}

static void client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg) {
    class_driver_t *driver_obj = (class_driver_t *)arg;
    switch (event_msg->event) {
        case USB_HOST_CLIENT_EVENT_NEW_DEV:
            if (driver_obj->dev_addr == 0) {
                driver_obj->dev_addr = event_msg->new_dev.address;
                // Open the device next
                driver_obj->actions |= ACTION_OPEN_DEV;
            }
            break;
        case USB_HOST_CLIENT_EVENT_DEV_GONE:
            if (driver_obj->dev_hdl != NULL) {
                // Cancel any other actions and close the device next
                driver_obj->actions = ACTION_CLOSE_DEV;
            }
            break;
        default:
            // Should never occur
            abort();
    }
}

static void action_open_dev(class_driver_t *driver_obj) {
    assert(driver_obj->dev_addr != 0);
    ESP_LOGI(TAG, "Opening device at address %d", driver_obj->dev_addr);
    ESP_ERROR_CHECK(usb_host_device_open(driver_obj->client_hdl, driver_obj->dev_addr, &driver_obj->dev_hdl));
    // Get the device's information next
    driver_obj->actions &= ~ACTION_OPEN_DEV;
    driver_obj->actions |= ACTION_GET_DEV_INFO;
}

static void action_get_info(class_driver_t *driver_obj) {
    assert(driver_obj->dev_hdl != NULL);
    ESP_LOGI(TAG, "Getting device information");
    usb_device_info_t dev_info;
    ESP_ERROR_CHECK(usb_host_device_info(driver_obj->dev_hdl, &dev_info));
    ESP_LOGI(TAG, "\tDevice Bus Speed\t%s speed", (dev_info.speed == USB_SPEED_LOW) ? "Low" : "Full");
    ESP_LOGI(TAG, "\tDevice’s address\t%d", dev_info.dev_addr);
    ESP_LOGI(TAG, "\tbMaxPacketSize0 \t%d", dev_info.bMaxPacketSize0);
    ESP_LOGI(TAG, "\tbConfigurationValue\t%d", dev_info.bConfigurationValue);
    ESP_LOGI(TAG, "\tiManufacturer\t%s", dev_info.str_desc_manufacturer);
    ESP_LOGI(TAG, "\tiProduct     \t%s", dev_info.str_desc_product);
    ESP_LOGI(TAG, "\tiSerialNumber\t%s", dev_info.str_desc_serial_num);
    // Get the device descriptor next
    driver_obj->actions &= ~ACTION_GET_DEV_INFO;
    driver_obj->actions |= ACTION_GET_DEV_DESC;
}

static uint8_t action_get_dev_desc(class_driver_t *driver_obj) {
    assert(driver_obj->dev_hdl != NULL);
    ESP_LOGI(TAG, "Getting device descriptor");
    const usb_device_desc_t *dev_desc;
    ESP_ERROR_CHECK(usb_host_get_device_descriptor(driver_obj->dev_hdl, &dev_desc));
    // usb_print_device_descriptor(dev_desc);
    if (dev_desc != NULL) {
        ESP_LOGI(TAG, "*** Device descriptor ***");
        ESP_LOGI(TAG, "bLength %d", dev_desc->bLength);
        ESP_LOGI(TAG, "bDescriptorType %d", dev_desc->bDescriptorType);
        ESP_LOGI(TAG, "bcdUSB %d.%d0", ((dev_desc->bcdUSB >> 8) & 0xF), ((dev_desc->bcdUSB >> 4) & 0xF));
        ESP_LOGI(TAG, "bDeviceClass 0x%x", dev_desc->bDeviceClass);
        ESP_LOGI(TAG, "bDeviceSubClass 0x%x", dev_desc->bDeviceSubClass);
        ESP_LOGI(TAG, "bDeviceProtocol 0x%x", dev_desc->bDeviceProtocol);
        ESP_LOGI(TAG, "bMaxPacketSize0 %d", dev_desc->bMaxPacketSize0);
        ESP_LOGI(TAG, "idVendor 0x%x", dev_desc->idVendor);
        ESP_LOGI(TAG, "idProduct 0x%x", dev_desc->idProduct);
        ESP_LOGI(TAG, "bcdDevice %d.%d0", ((dev_desc->bcdDevice >> 8) & 0xF), ((dev_desc->bcdDevice >> 4) & 0xF));
        ESP_LOGI(TAG, "iManufacturer %d", dev_desc->iManufacturer);
        ESP_LOGI(TAG, "iProduct %d", dev_desc->iProduct);
        ESP_LOGI(TAG, "iSerialNumber %d", dev_desc->iSerialNumber);
        ESP_LOGI(TAG, "bNumConfigurations %d", dev_desc->bNumConfigurations);
    }
    // Get the device's config descriptor next
    driver_obj->actions &= ~ACTION_GET_DEV_DESC;
    driver_obj->actions |= ACTION_GET_CONFIG_DESC;

    return dev_desc->bNumConfigurations;
}

static void action_get_config_desc(class_driver_t *driver_obj, uint8_t num_configuraitons) {
    assert(driver_obj->dev_hdl != NULL);
    ESP_LOGI(TAG, "Getting config descriptor");
    const usb_config_desc_t *config_desc;
    ESP_ERROR_CHECK(usb_host_get_active_config_descriptor(driver_obj->dev_hdl, &config_desc));
    // usb_print_config_descriptor(config_desc, NULL);
    int offset_intf = 0;
    // num_configuraitons = 1;  // ESP-IDF support only first configuration
    for (uint8_t i = 0; i < num_configuraitons; i++) {
        // Get Each Configuration Descriptors
        ESP_LOGI(TAG, "Config descriptor %d", i);
        for (uint8_t j = 0; j < config_desc->bNumInterfaces; j++) {
            // Get Each Interface Descriptors
            ESP_LOGI(TAG, "Interface %d", j);
            int8_t alt_setting_num = usb_parse_interface_number_of_alternate(config_desc, i);
            for (int8_t k = -1; k < alt_setting_num; k++) {
                // Get Each Alternate Interfaces
                ESP_LOGI(TAG, "Alt %d", k + 1);
                int offset_alt = 0;
                const usb_intf_desc_t *intf_desc = usb_parse_interface_descriptor(config_desc, j, k + 1, &offset_alt);
                // usb_print_config_descriptor((const usb_config_desc_t *)intf_desc, NULL);
                if (intf_desc != NULL) {
                    // Search MIDI Descriptor
                    ESP_LOGI(TAG, "Search MIDI Descriptor");
                    // Check bInterfaceClass & bInterfaceSubClass
                    if (intf_desc->bInterfaceClass == CLASS_AUDIO && intf_desc->bInterfaceSubClass == SUBCLASS_MIDISTREAMING &&
                        intf_desc->bNumEndpoints != 0) {
                        ESP_LOGI(TAG, "Found AUDIO-MIDISTREAMING, bNumEndpoints=%d", intf_desc->bNumEndpoints);
                        // Clain Interface
                        usb_host_interface_claim(driver_obj->client_hdl, driver_obj->dev_hdl, intf_desc->bInterfaceNumber,
                                                 intf_desc->bAlternateSetting);
                        int offset_ep;
                        const usb_ep_desc_t *ep_desc = NULL;
                        for (uint8_t index = 0; index < intf_desc->bNumEndpoints; index++) {
                            offset_ep = offset_alt;
                            ep_desc = usb_parse_endpoint_descriptor_by_index(intf_desc, index, config_desc->wTotalLength, &offset_ep);
                            if (ep_desc == NULL) {
                                ESP_LOGI(TAG, "EP_DESC == NULL!!!");
                            }
                            ESP_LOGI(TAG, "NumEndpoint=%d, EndPoindAddress=0x%02x", index, ep_desc->bEndpointAddress);
                            // Check Endpoint
                            if (USB_EP_DESC_GET_XFERTYPE(ep_desc) != USB_BM_ATTRIBUTES_XFER_BULK) {
                                ESP_LOGI(TAG, "This Endpoint is not BULK transfer");
                            } else {
                                // usb_print_config_descriptor((const usb_config_desc_t *)ep_desc, NULL);
                                if (USB_EP_DESC_GET_EP_DIR(ep_desc)) {
                                    // IN Endpoint
                                    ESP_LOGI(TAG, "Found IN Endpoiint, bEndpointAddress=0x%02x", ep_desc->bEndpointAddress);
                                    // Allocate transfer
                                    esp_err_t err = usb_host_transfer_alloc(ep_desc->wMaxPacketSize, 0, &MidiIn);
                                    if (err != ESP_OK) {
                                        MidiIn = NULL;
                                        ESP_LOGI(TAG, "usb_host_transfer_alloc MidiIn fail: %x", err);
                                    } else {
                                        MidiIn->device_handle = driver_obj->dev_hdl;
                                        MidiIn->bEndpointAddress = ep_desc->bEndpointAddress;
                                        MidiIn->callback = midi_transfer_cb;
                                        // MidiIn->context = 0;
                                        MidiIn->num_bytes = ep_desc->wMaxPacketSize;
                                        err = usb_host_transfer_submit(MidiIn);
                                        if (err != ESP_OK) {
                                            ESP_LOGI(TAG, "usb_host_transfer_submit MidiIn fail: %x", err);
                                        }
                                    }
                                } else {
                                    // OUT Endpoint
                                    ESP_LOGI(TAG, "Found OUT Endpoiint, bEndpointAddress=0x%02x", ep_desc->bEndpointAddress);
                                    // Allocate transfer
                                    esp_err_t err = usb_host_transfer_alloc(ep_desc->wMaxPacketSize, 0, &MidiOut);
                                    if (err != ESP_OK) {
                                        MidiOut = NULL;
                                        ESP_LOGI(TAG, "usb_host_transfer_alloc MidiOut fail: %x", err);
                                    } else {
                                        MidiOut->device_handle = driver_obj->dev_hdl;
                                        MidiOut->bEndpointAddress = ep_desc->bEndpointAddress;
                                        MidiOut->callback = midi_transfer_cb;
                                        MidiOut->context = NULL;
                                    }
                                }
                            }
                            const usb_midi_cs_ep_desc_t *midi_cs_ep_desc = NULL;
                            int offset_midi_cs_ep = 0;
                            midi_cs_ep_desc = (usb_midi_cs_ep_desc_t *)usb_parse_next_descriptor_of_type(
                                (const usb_standard_desc_t *)ep_desc, config_desc->wTotalLength, AUDIO_CS_ENDPOINT, &offset_midi_cs_ep);
                            ESP_LOGI(TAG, "Number of Embedded MIDI Jack=%d", midi_cs_ep_desc->bNumEmbMIDIJack);
                            // MIDI Jack NumberとEndpoint
                            // Numberを整理する
                        }
                        break; 
                    } else {
                        ESP_LOGI(TAG, "Not Found AUDIO-MIDISTREAMING");
                    }
                }
            }
        }
        usb_parse_next_descriptor_of_type((const usb_standard_desc_t *)config_desc, config_desc->bLength, USB_B_DESCRIPTOR_TYPE_CONFIGURATION,
                                          &offset_intf);
    }

    // Get the device's string descriptors next
    driver_obj->actions &= ~ACTION_GET_CONFIG_DESC;
    driver_obj->actions |= ACTION_GET_STR_DESC;
}

static void action_get_str_desc(class_driver_t *driver_obj) {
    assert(driver_obj->dev_hdl != NULL);
    usb_device_info_t dev_info;
    ESP_ERROR_CHECK(usb_host_device_info(driver_obj->dev_hdl, &dev_info));
    if (dev_info.str_desc_manufacturer) {
        ESP_LOGI(TAG, "Getting Manufacturer string descriptor");
        usb_print_string_descriptor(dev_info.str_desc_manufacturer);
    }
    if (dev_info.str_desc_product) {
        ESP_LOGI(TAG, "Getting Product string descriptor");
        usb_print_string_descriptor(dev_info.str_desc_product);
    }
    if (dev_info.str_desc_serial_num) {
        ESP_LOGI(TAG, "Getting Serial Number string descriptor");
        usb_print_string_descriptor(dev_info.str_desc_serial_num);
    }
    // Nothing to do until the device disconnects
    driver_obj->actions &= ~ACTION_GET_STR_DESC;
}

static void aciton_close_dev(class_driver_t *driver_obj) {
    ESP_ERROR_CHECK(usb_host_device_close(driver_obj->client_hdl, driver_obj->dev_hdl));
    driver_obj->dev_hdl = NULL;
    driver_obj->dev_addr = 0;
    // We need to exit the event handler loop
    driver_obj->actions &= ~ACTION_CLOSE_DEV;
    driver_obj->actions |= ACTION_EXIT;
}

void midi_class_driver_task(void *arg) {
    SemaphoreHandle_t signaling_sem = (SemaphoreHandle_t)arg;
    class_driver_t driver_obj = {};

    // Wait until daemon task has installed USB Host Library
    xSemaphoreTake(signaling_sem, portMAX_DELAY);

    ESP_LOGI(TAG, "Registering Client");
    usb_host_client_config_t client_config = {
        .is_synchronous = false,  // Synchronous clients currently not
                                  // supported. Set this to false
        .max_num_event_msg = CLIENT_NUM_EVENT_MSG,
        .async =
            {
                .client_event_callback = client_event_cb,
                .callback_arg = (void *)&driver_obj,
            },
    };
    ESP_ERROR_CHECK(usb_host_client_register(&client_config, &driver_obj.client_hdl));

    uint8_t num_configuraitons = 0;
    while (1) {
        if (driver_obj.actions == 0) {
            usb_host_client_handle_events(driver_obj.client_hdl, portMAX_DELAY);
        } else {
            if (driver_obj.actions & ACTION_OPEN_DEV) {
                action_open_dev(&driver_obj);
            }
            if (driver_obj.actions & ACTION_GET_DEV_INFO) {
                action_get_info(&driver_obj);
            }
            if (driver_obj.actions & ACTION_GET_DEV_DESC) {
                num_configuraitons = action_get_dev_desc(&driver_obj);
                ESP_LOGI(TAG, "bConfiguraitons=%d", num_configuraitons);
            }
            if (driver_obj.actions & ACTION_GET_CONFIG_DESC) {
                action_get_config_desc(&driver_obj, num_configuraitons);
            }
            if (driver_obj.actions & ACTION_GET_STR_DESC) {
                action_get_str_desc(&driver_obj);
            }

            if (driver_obj.actions & ACTION_CLOSE_DEV) {
                MidiIn = NULL;
                MidiOut = NULL;
                for (uint8_t i = 0; i < 16; i++) {
                    usb_host_endpoint_flush(driver_obj.dev_hdl, i); usb_host_endpoint_flush(driver_obj.dev_hdl, i | 0x80);
                }
                for (uint8_t i = 0; i < 16; i++) {
                    usb_host_interface_release(driver_obj.client_hdl, driver_obj.dev_hdl, i);
                }
                aciton_close_dev(&driver_obj);
            }
            if (driver_obj.actions & ACTION_EXIT) {
                driver_obj.actions &= ~ACTION_EXIT;
                // break;
            }
        }
    }

    ESP_LOGI(TAG, "Deregistering Client");
    ESP_ERROR_CHECK(usb_host_client_deregister(driver_obj.client_hdl));

    // Wait to be deleted
    xSemaphoreGive(signaling_sem);
    vTaskSuspend(NULL);
}

midiEventPacket_t midi_read() {
    midiEventPacket_t c = {0, 0, 0, 0};
    // Read from MIDI IN Buffer
    if ((MIDI_BUFFER_SIZE + midi_rx_buffer.head - midi_rx_buffer.tail) % MIDI_BUFFER_SIZE > 0) {
        c = midi_rx_buffer.midiEvent[midi_rx_buffer.tail];
        ESP_LOGI(TAG, "midi_read(): %02x %02x %02x %02x", c.header, c.byte1, c.byte2, c.byte3);
    }

    if (midi_rx_buffer.head != midi_rx_buffer.tail) {
        midi_rx_buffer.tail = (uint32_t)(midi_rx_buffer.tail + 1) % MIDI_BUFFER_SIZE;
        ESP_LOGI(TAG, "Buffer: head = %d, tail = %d", midi_rx_buffer.head, midi_rx_buffer.tail);
    }

    return c;
}

esp_err_t midi_write(midiEventPacket_t event) {
    if (MidiOut == NULL) return ESP_FAIL;
    ESP_LOGI(TAG, "write_midi(): %02x %02x %02x %02x", event.header, event.byte1, event.byte2, event.byte3);
    MidiOut->num_bytes = 4;
    memcpy(MidiOut->data_buffer, (uint8_t *)&event, 4);
    esp_err_t err = usb_host_transfer_submit(MidiOut);
    if (err != ESP_OK) {
        ESP_LOGI("", "write_midi() fail: %x", err);
    }
    return err; 
}