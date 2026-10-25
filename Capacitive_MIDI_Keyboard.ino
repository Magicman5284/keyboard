/*
  Capacitive MIDI Keyboard
  Board: Waveshare ESP32-S3-Zero
  Touch controller: MPR121 on I2C (SDA GPIO1, SCL GPIO2)

  USB MIDI uses the ESP32-S3 native USB port via TinyUSB.
  Serial debugging is intentionally omitted because native USB must stay in
  USB-OTG/TinyUSB mode, and GPIO43/44 are used by two of the switches.

  Controls:
    Switch 1 (RX / GPIO44): Octave down
    Switch 2 (TX / GPIO43): Octave up
    Switch 3 (GPIO13): Sustain (MIDI CC64, held = on)
    Switch 4 (GPIO12): Toggle modulation (MIDI CC1, 0/127)
    Switch 5 (GPIO11): Previous program
    Switch 6 (GPIO10): Next program
    Switch 7 (GPIO9): Volume down (MIDI CC7)
    Switch 8 (GPIO8): Volume up (MIDI CC7)
    Switch 9 (GPIO5): Panic / all notes off

    Encoder A (GPIO6), B (GPIO7): Change selected MIDI CC value
    Encoder switch (GPIO4): Cycle encoder target: Filter (CC74),
                             Resonance (CC71), Reverb send (CC91)

 
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPR121.h>
#include <string.h>
#include "USB.h"
#include "esp32-hal-tinyusb.h"

// -------------------- Pin configuration --------------------
static constexpr uint8_t SDA_PIN = 1;
static constexpr uint8_t SCL_PIN = 2;
static constexpr uint8_t MPR121_ADDRESS = 0x5A;

static constexpr uint8_t BUTTON_PINS[9] = {
  44,  // Switch 1: RX label = GPIO44
  43,  // Switch 2: TX label = GPIO43
  13,  // Switch 3
  12,  // Switch 4
  11,  // Switch 5
  10,  // Switch 6
  9,   // Switch 7
  8,   // Switch 8
  5    // Switch 9
};

static constexpr uint8_t ENCODER_A_PIN = 6;
static constexpr uint8_t ENCODER_B_PIN = 7;
static constexpr uint8_t ENCODER_SWITCH_PIN = 4;

static constexpr uint32_t I2C_CLOCK_HZ = 400000;
static constexpr uint32_t DEBOUNCE_MS = 25;


extern "C" uint16_t tusb_midi_load_descriptor(uint8_t *dst, uint8_t *itf) {
  uint8_t stringIndex = tinyusb_add_string_descriptor("Capacitive MIDI Keyboard");
  uint8_t endpoint = tinyusb_get_free_duplex_endpoint();
  if (endpoint == 0) {
    return 0;
  }

  uint8_t descriptor[TUD_MIDI_DESC_LEN] = {
    TUD_MIDI_DESCRIPTOR(*itf, stringIndex, endpoint,
                        static_cast<uint8_t>(0x80 | endpoint), 64)
  };

  *itf += 1;
  memcpy(dst, descriptor, TUD_MIDI_DESC_LEN);
  return TUD_MIDI_DESC_LEN;
}


Adafruit_MPR121 cap = Adafruit_MPR121();
bool mpr121Ready = false;
uint16_t lastTouchMask = 0;
int16_t activeNotes[12];  // The exact MIDI pitch currently sounding per key; -1 = none


int baseNote = 60;        // Starts at C4; octave buttons adjust this by 12
static constexpr int MIN_BASE_NOTE = 24;  // C1
static constexpr int MAX_BASE_NOTE = 96;  // C7

// MIDI channel 1 is encoded as channel index 0 in MIDI status bytes.
static constexpr uint8_t MIDI_CHANNEL = 0;

// MIDI control state 
uint8_t currentProgram = 0;  // MIDI program numbers sent are 0-127
uint8_t volumeValue = 100;
bool modulationEnabled = false;
bool sustainEnabled = false;

// Encoder targets: filter/timbre (CC74), resonance (CC71), reverb send (CC91).
static constexpr uint8_t ENCODER_CC[3] = {74, 71, 91};
const char *const ENCODER_NAMES[3] = {"Filter (CC74)", "Resonance (CC71)", "Reverb send (CC91)"};
uint8_t encoderValues[3] = {64, 64, 32};
uint8_t encoderMode = 0;
bool midiControlsInitialized = false;

// Debounced button handling 
struct DebouncedInput {
  uint8_t pin;
  bool stableState;  // HIGH = released, LOW = pressed
  bool lastRawState;
  uint32_t lastRawChangeMs;
};

DebouncedInput buttons[9];
DebouncedInput encoderSwitch;

void initInput(DebouncedInput &input, uint8_t pin) {
  input.pin = pin;
  pinMode(pin, INPUT_PULLUP);
  bool initialState = digitalRead(pin);
  input.stableState = initialState;
  input.lastRawState = initialState;
  input.lastRawChangeMs = millis();
}


bool updateInput(DebouncedInput &input) {
  const uint32_t now = millis();
  const bool raw = digitalRead(input.pin);

  if (raw != input.lastRawState) {
    input.lastRawState = raw;
    input.lastRawChangeMs = now;
  }

  if ((now - input.lastRawChangeMs >= DEBOUNCE_MS) &&
      (input.stableState != input.lastRawState)) {
    input.stableState = input.lastRawState;
    return true;
  }
  return false;
}


void sendMidi(const uint8_t *message, uint32_t length) {
  if (tud_midi_mounted()) {
    tud_midi_stream_write(0, message, length);
  }
}

void sendNoteOn(uint8_t note, uint8_t velocity = 100) {
  const uint8_t message[3] = {
    static_cast<uint8_t>(0x90 | MIDI_CHANNEL), note, velocity
  };
  sendMidi(message, sizeof(message));
}

void sendNoteOff(uint8_t note) {
  const uint8_t message[3] = {
    static_cast<uint8_t>(0x80 | MIDI_CHANNEL), note, 0
  };
  sendMidi(message, sizeof(message));
}

void sendControlChange(uint8_t controller, uint8_t value) {
  const uint8_t message[3] = {
    static_cast<uint8_t>(0xB0 | MIDI_CHANNEL), controller,
    static_cast<uint8_t>(value > 127 ? 127 : value)
  };
  sendMidi(message, sizeof(message));
}

void sendProgramChange() {
  const uint8_t message[2] = {
    static_cast<uint8_t>(0xC0 | MIDI_CHANNEL), currentProgram
  };
  sendMidi(message, sizeof(message));
}

void changeOctave(int direction) {
  const int proposed = baseNote + (12 * direction);
  if (proposed >= MIN_BASE_NOTE && proposed <= MAX_BASE_NOTE) {
    baseNote = proposed;
  }
  // Notes already held keep their original pitch until released.
}

void changeVolume(int amount) {
  int nextValue = static_cast<int>(volumeValue) + amount;
  if (nextValue < 0) nextValue = 0;
  if (nextValue > 127) nextValue = 127;
  volumeValue = static_cast<uint8_t>(nextValue);
  sendControlChange(7, volumeValue);
}

void changeProgram(int direction) {
  int nextProgram = static_cast<int>(currentProgram) + direction;
  if (nextProgram < 0) nextProgram = 127;
  if (nextProgram > 127) nextProgram = 0;
  currentProgram = static_cast<uint8_t>(nextProgram);
  sendProgramChange();
}

void panicAllNotesOff() {
  // Explicitly release every note this keyboard is tracking.
  for (uint8_t i = 0; i < 12; ++i) {
    if (activeNotes[i] >= 0) {
      sendNoteOff(static_cast<uint8_t>(activeNotes[i]));
      activeNotes[i] = -1;
    }
  }

  // Also send standard safety controllers on MIDI channel 1.
  sendControlChange(64, 0);   // Sustain off
  sendControlChange(120, 0);  // All Sounds Off
  sendControlChange(123, 0);  // All Notes Off
  sustainEnabled = false;
}

void handleButtonTransition(uint8_t index, bool pressed) {
  switch (index) {
    case 0:  // Switch 1: octave down
      if (pressed) changeOctave(-1);
      break;

    case 1:  // Switch 2: octave up
      if (pressed) changeOctave(+1);
      break;

    case 2:  // Switch 3: momentary sustain
      sustainEnabled = pressed;
      sendControlChange(64, pressed ? 127 : 0);
      break;

    case 3:  // Switch 4: modulation toggle
      if (pressed) {
        modulationEnabled = !modulationEnabled;
        sendControlChange(1, modulationEnabled ? 127 : 0);
      }
      break;

    case 4:  // Switch 5: previous program
      if (pressed) changeProgram(-1);
      break;

    case 5:  // Switch 6: next program
      if (pressed) changeProgram(+1);
      break;

    case 6:  // Switch 7: volume down
      if (pressed) changeVolume(-8);
      break;

    case 7:  // Switch 8: volume up
      if (pressed) changeVolume(+8);
      break;

    case 8:  // Switch 9: panic
      if (pressed) panicAllNotesOff();
      break;
  }
}

void handleTouchChanges() {
  if (!mpr121Ready) return;

  const uint16_t currentMask = cap.touched() & 0x0FFF;

  for (uint8_t i = 0; i < 12; ++i) {
    const uint16_t bit = static_cast<uint16_t>(1U << i);
    const bool isTouched = (currentMask & bit) != 0;
    const bool wasTouched = (lastTouchMask & bit) != 0;

    if (isTouched && !wasTouched) {
      const int note = baseNote + i;
      if (note >= 0 && note <= 127) {
        activeNotes[i] = note;
        sendNoteOn(static_cast<uint8_t>(note));
      }
    } else if (!isTouched && wasTouched) {
      if (activeNotes[i] >= 0) {
        sendNoteOff(static_cast<uint8_t>(activeNotes[i]));
        activeNotes[i] = -1;
      }
    }
  }

  lastTouchMask = currentMask;
}

// -------------------- Rotary encoder --------------------
// Table-based quadrature decoding helps avoid missed transitions.
const int8_t ENCODER_TRANSITIONS[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};
uint8_t lastEncoderState = 0;
int8_t encoderAccumulator = 0;

void adjustEncoderValue(int direction) {
  int value = static_cast<int>(encoderValues[encoderMode]) + (direction * 2);
  if (value < 0) value = 0;
  if (value > 127) value = 127;
  encoderValues[encoderMode] = static_cast<uint8_t>(value);
  sendControlChange(ENCODER_CC[encoderMode], encoderValues[encoderMode]);
}

void pollEncoder() {
  const uint8_t currentState =
      (static_cast<uint8_t>(digitalRead(ENCODER_A_PIN)) << 1) |
       static_cast<uint8_t>(digitalRead(ENCODER_B_PIN));

  const uint8_t tableIndex = static_cast<uint8_t>((lastEncoderState << 2) | currentState);
  encoderAccumulator += ENCODER_TRANSITIONS[tableIndex];
  lastEncoderState = currentState;

  if (encoderAccumulator >= 4) {
    encoderAccumulator = 0;
    adjustEncoderValue(+1);
  } else if (encoderAccumulator <= -4) {
    encoderAccumulator = 0;
    adjustEncoderValue(-1);
  }
}

void handleEncoderSwitch() {
  encoderMode = static_cast<uint8_t>((encoderMode + 1) % 3);
  sendControlChange(ENCODER_CC[encoderMode], encoderValues[encoderMode]);
}

// Send the current controller values once the host has opened the USB MIDI port.
// If USB is unplugged/reconnected, send them again.
void serviceMidiConnection() {
  if (!tud_midi_mounted()) {
    midiControlsInitialized = false;
    return;
  }

  if (!midiControlsInitialized) {
    sendControlChange(7, volumeValue);
    sendControlChange(1, modulationEnabled ? 127 : 0);
    sendControlChange(64, sustainEnabled ? 127 : 0);
    for (uint8_t i = 0; i < 3; ++i) {
      sendControlChange(ENCODER_CC[i], encoderValues[i]);
    }
    midiControlsInitialized = true;
  }
}

// -------------------- Arduino setup and loop --------------------
void setup() {
  // The Arduino IDE must use native USB OTG/TinyUSB mode.
  for (uint8_t i = 0; i < 12; ++i) {
    activeNotes[i] = -1;
  }

  for (uint8_t i = 0; i < 9; ++i) {
    initInput(buttons[i], BUTTON_PINS[i]);
  }
  initInput(encoderSwitch, ENCODER_SWITCH_PIN);

  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  lastEncoderState =
      (static_cast<uint8_t>(digitalRead(ENCODER_A_PIN)) << 1) |
       static_cast<uint8_t>(digitalRead(ENCODER_B_PIN));

  // Enable a standard USB MIDI 1.0 interface.
  tinyusb_enable_interface(USB_INTERFACE_MIDI, TUD_MIDI_DESC_LEN,
                           tusb_midi_load_descriptor);
  USB.begin();
  delay(300);

  Wire.begin(SDA_PIN, SCL_PIN, I2C_CLOCK_HZ);
  if (cap.begin(MPR121_ADDRESS, &Wire)) {
    cap.setAutoconfig(true);
    mpr121Ready = true;
    lastTouchMask = cap.touched() & 0x0FFF;
  } else {
    mpr121Ready = false;
  }

}

void loop() {
  for (uint8_t i = 0; i < 9; ++i) {
    if (updateInput(buttons[i])) {
      const bool pressed = (buttons[i].stableState == LOW);
      handleButtonTransition(i, pressed);
    }
  }

  if (updateInput(encoderSwitch) && encoderSwitch.stableState == LOW) {
    handleEncoderSwitch();
  }

  pollEncoder();
  serviceMidiConnection();
  handleTouchChanges();
  delay(1);
}
