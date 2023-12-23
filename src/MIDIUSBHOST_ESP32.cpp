#include "USB.h"
#if CONFIG_TINYUSB_MIDI_ENABLED

#include "MIDIUSBHOST_ESP32.h"

const TickType_t CLIENT_EVENT_TIMEOUT = 1;

// Interface counter
enum interface_count {
#if CFG_TUD_MIDI
    ITF_NUM_MIDI = 0,
    ITF_NUM_MIDI_STREAMING,
#endif
    ITF_COUNT
};

// USB Endpoint numbers
enum usb_endpoints {
    // Available USB Endpoints: 5 IN/OUT EPs and 1 IN EP
    EP_EMPTY = 0,
#if CFG_TUD_MIDI
    EPNUM_MIDI,
#endif
};

/** TinyUSB descriptors **/
#define TUSB_DESCRIPTOR_ITF_MIDI_LEN                                         \
    (TUD_MIDI_DESC_HEAD_LEN + TUD_MIDI_DESC_JACK_LEN * USB_MIDI_NUM_CABLES + \
     TUD_MIDI_DESC_EP_LEN(USB_MIDI_NUM_CABLES) * 2)
#define TUSB_DESCRIPTOR_TOTAL_LEN \
    (TUD_CONFIG_DESC_LEN + CFG_TUD_MIDI * TUSB_DESCRIPTOR_ITF_MIDI_LEN)

ESP_EVENT_DEFINE_BASE(ARDUINO_USB_MIDI_EVENTS);

MIDIUSBHOST MidiUSB;

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"
#include "usb/usb_host.h"

#define DAEMON_TASK_PRIORITY    2
#define CLASS_TASK_PRIORITY     3

extern void midi_class_driver_task(void *arg);
extern midiEventPacket_t midi_read(void); 
extern esp_err_t midi_write(midiEventPacket_t event); 

static const char *TAG = "DAEMON";

static void host_lib_daemon_task(void *arg)
{
    SemaphoreHandle_t signaling_sem = (SemaphoreHandle_t)arg;

    ESP_LOGI(TAG, "Installing USB Host Library");
    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    //Signal to the class driver task that the host library is installed
    xSemaphoreGive(signaling_sem);
    vTaskDelay(10); //Short delay to let client task spin up

    while(1) {
        uint32_t event_flags;
        ESP_ERROR_CHECK(usb_host_lib_handle_events(portMAX_DELAY, &event_flags));
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_LOGI(TAG, "USB Host Lib Event: USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS");
            //has_clients = false;
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGI(TAG, "USB Host Lib Event: USB_HOST_LIB_EVENT_FLAGS_ALL_FREE");
            //has_devices = false;
        }
    }
    ESP_LOGI(TAG, "No more clients and devices");

    //Uninstall the USB Host Library
    ESP_ERROR_CHECK(usb_host_uninstall());
    //Wait to be deleted
    xSemaphoreGive(signaling_sem);
    vTaskSuspend(NULL);
}

MIDIUSBHOST::MIDIUSBHOST() {

}

MIDIUSBHOST::~MIDIUSBHOST(){};

static TaskHandle_t daemon_task_hdl;
static TaskHandle_t class_driver_task_hdl;

void MIDIUSBHOST::begin() {
    SemaphoreHandle_t signaling_sem = xSemaphoreCreateBinary();


    //Create daemon task
    xTaskCreatePinnedToCore(host_lib_daemon_task,
                            "daemon",
                            4096,
                            (void *)signaling_sem,
                            DAEMON_TASK_PRIORITY,
                            &daemon_task_hdl,
                            0);
    //Create the class driver task
    xTaskCreatePinnedToCore(midi_class_driver_task,
                            "class",
                            4096,
                            (void *)signaling_sem,
                            CLASS_TASK_PRIORITY,
                            &class_driver_task_hdl,
                            0);

}

void MIDIUSBHOST::end() {
    //Delete the tasks
    vTaskDelete(class_driver_task_hdl);
    vTaskDelete(daemon_task_hdl);
}; 

midiEventPacket_t MIDIUSBHOST::read() {
    // read from ring buffer
    midiEventPacket_t data = midi_read();
    return data;
}
void MIDIUSBHOST::flush(void) {
    
}

esp_err_t MIDIUSBHOST::sendMIDI(midiEventPacket_t event) {
    // write midi
    return midi_write(event); 
}

#endif /* CONFIG_TINYUSB_MIDI_ENABLED */
