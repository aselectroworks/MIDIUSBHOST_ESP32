#define ESP32_USB_HOST
#include <MIDI.h>
#include <USB-MIDI.h>

USBMIDI_CREATE_INSTANCE(0, UsbMIDI)

void setup() {
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    MidiUSB.begin();
}

uint32_t cur_time = 0;
void loop() {
    midiEventPacket_t rx;
	rx = MidiUSB.read();
	if (rx.header != 0) {
		Serial.printf("Received: %02x-%02x-%02x-%02x\r\n", rx.header, rx.byte1, rx.byte2, rx.byte3); 
	}

    if (millis() - 2000 > cur_time) {
        cur_time = millis();
        midiEventPacket_t tx;
        tx.header = 0x09;
        tx.byte1 = 0x090;
        tx.byte2 = 0x48;
        tx.byte3 = 0x64;
        if(MidiUSB.sendMIDI(tx) == ESP_OK) {
			Serial.printf("Send: %02x-%02x-%02x-%02x\r\n", tx.header, tx.byte1, tx.byte2, tx.byte3); 
		}
    }
}
