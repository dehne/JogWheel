/****
 * JogWheel V1.0, December 2020
 *
 * This is the firmware for a jog wheel based on a stepper motor used not as a
 * motor but as a sensor (and as the jog wheel's mechanical guts). A stepper 
 * motor like the one used here has magnets and coils that move relative to 
 * one another when the output shaft turns. It's this relative motion that 
 * produces the cogging -- the bumps in torque -- you feel when you turn an 
 * unpowered stepper. It's also the cogging that makes such motors attractive 
 * to use in a jog wheel. It gives the wheel a nice feel. 
 * 
 * The ends of the coils are exposed as the motor's leads. Because the motor 
 * is designed to spin, the coils and magnets are arranged so that the magnets 
 * pass the coils at different times as the shaft spins, this makes it 
 * possible to drive the motor by sequentially energizing and deenregizing the 
 * coils with the correct polarity and timing. Here, though, we take advantage 
 * of the fact that if you turn the stepper's shaft, it moves magnets past the 
 * coils. Doing that induces voltage pulses across the coils. By sensing the 
 * number and relative timing of the voltage pulses we can deduce how much the 
 * shaft has been turned, and in which direction.
 * 
 * This sketch is for a two-coil, bipolar stepper motor I extracated from a 
 * deceased printer. The ends of its two coils, A and B, are exposed as four 
 * wires, A-, A+, B- and B+. When the shaft is turned clockwise, a pulse first 
 * appears on coil A and then on B. When it's turned counterclockwise, the 
 * pulse on B preceeds the one on A. A bit of passive support circuitry cleans 
 * up the shape of the pulses, ensures the pulses don't go below about -0.6v 
 * and aren't bigger than about 4.5v. (Not surprisingly, the voltage pulses 
 * coming directly out of the motor go in both positive and negative 
 * directions and get many times bigger as you spin the shaft more quickly.)
 * 
 * The jogwheel plugs into a computer via USB where it appears as three 
 * devices, a keyboard, a mouse, and a serial port. To do this, it needs to 
 * run on an ATmega32U4.) Turning the wheel generates customizable keyboard and 
 * mouse events. More specifically, when you turn the wheel, the sketch 
 * generates repeating sequences of keystrokes and/or mouse events. It 
 * generates more repeats the more you turn the wheel. The keystrokes and/or 
 * mouse events that make up each sequence depends on two things. First 
 * jogwheel generates one sequence when you turn the wheel clockwise, and a 
 * different one when you turn it counterclockwise. Second, jogwheel supports 
 * seven programmable pairs of sequences called "configurations". You can 
 * change which configuration the wheel uses by clicking the buttons near the 
 * front of the device. The buttons may be clicked individually or in 
 * combination. So, for example, by clicking the right, middle or left buttons 
 * you can select which of three different configurations jogwheel uses. 
 * Simultaneously clicking the right and middle buttons selects a fourth 
 * configuration, and so on. Each configuration is associated with a color. 
 * The color indicated by an LED tells which configuration you have selected.
 * 
 * Exactly what keystroke and/or mouse events make up each of the sequences 
 * for a configuration is up to you. You can set the sequences and define 
 * which configuration(s) the sequences are to be used for using the 
 * jogwheel's command-line interface. The command-line interface is presented 
 * through the jogwheel's serial port. Connect a serial terminal (e.g., Putty) 
 * to the jogwheel's serial port -- 9600 baud, 8 bits per character, no parity,
 * 1 start bit -- and type "help" to get started.
 * 
 * Keystrokes can include any printable character as well as many non-printing 
 * keystrokes like "up-arrow" and "down-arrow". Mouse events can include mouse 
 * movement, button clicks, and wheel movements. In addition, the keystrokes 
 * and mouse movements can include modifier keys like ctrl, alt, shift and 
 * "gui". The "gui" modifier is os-dependent. On Mac-OS it's called "option" 
 * on Windows it's called the "Windows key". Various Linux systems also have a 
 * "gui" modifier key.
 * 
 ****
 *
 * Copyright (C) 2020 D.L. Ehnebuske
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 * 
 ****/

#include <Arduino.h>
#ifdef __AVR_ATmega32U4__
#include <Keyboard.h>
#include <Mouse.h>
#endif
#include <avr/eeprom.h>                     // Store and retrieve values in non-volatile eeprom
#include <util/atomic.h>                    // Atomic blocks
#include "UserInput.h"

/****
 * 
 * Compile time constants
 * 
 ****/

//#define DEBUG_ISR                       // Uncomment to enbale ISR debugging
//#define DEBUG_EEPROM                    // Uncomment to enable eeprom debugging
//#define DEBUG_PARSE                     // Uncomment to enable config spec parsing debugging
//#define DEBUG                           // Uncomment to enable general debugging output
//#define FACTORY_RESET                   // Uncomment to "factory reset," i.e., reinitialize EEPROM
#define D_STATE_SIZE        (16)        // Number of ISR states to record for debugging

// Hardware GPIO pin definitions
#define COIL_A_PIN          (A0)        // A+ goes here. (A- goes to GND.)
#define COIL_B_PIN          (A1)        // B+ goes here. (B- goes to GND.)
#define LED_R               (8)         // Where we attached the red part of the LED
#define LED_G               (7)         // Where we attached the green part of the LED
#define LED_B               (6)         // Where we attached the blue part of the LED
#define BUTTON_A            (3)         // Configuration selector switch A attaches here
#define BUTTON_B            (4)         // Configuration selector switch B attaches here
#define BUTTON_C            (5)         // Configuration selector switch C attaches here

// EEPROM related stuff
#define FINGERPRINT         (0xC29D)    // EEPROM "fingerprint" value for JogWheel
#define ENTRY_CW            (0)         // The clockwise entry in a config.entry[x] pair
#define ENTRY_CC            (1)         // The counterclockwise entry in a confiog.entry[x] pair
#define N_ELEMENTS(a)       ((uint16_t)(sizeof(a) / sizeof(a[0]))) // Number of elements in an array

// Configuration entry bits and masks
#define CE_TYPE_MASK        (0x8000)    // =0 ==> keyboard entry, =1 ==> Mouse entry
#define KB_CTRL_MASK        (0x0800)    // =1 ==> ctrl-key down in keyboard entry
#define KB_ALT_MASK         (0x0400)    // =1 ==> alt-key down in keyboard entry
#define KB_SHIFT_MASK       (0x0200)    // =1 ==> shift-key down in keyboard entry (OK but not necessary for ASCII chars)
#define KB_GUI_MASK         (0x0100)    // =1 ==> gui-key (e.g., IOS option) down in keyboard entry
#define KB_VALUE_MASK       (0x00FF)    // Keyboard entry value bits
#define ME_TYPE_MASK        (0x3000)    // Mouse entry type bits
#define ME_TYPE(e)          (((e) & ME_TYPE_MASK) >> 12)   // Mouse entry type
#define SET_ME_TYPE(e, v)   (e = ((e) & ~ME_TYPE_MASK) | (((v) << 12) & ME_TYPE_MASK))
#define ME_TYPE_WHEEL       (0)         // In type 0, value is signed amount to move wheel
#define ME_TYPE_X           (1)         // In type 1, value is signed x-distance to move mouse
#define ME_TYPE_Y           (2)         // In type 2, value is signed y-distance to move mouse
#define ME_TYPE_CLICK       (3)         // In type 3, value is which mouse button to click
#define ME_VALUE_MASK       (0x00FF)
#define ME_CTRL_MASK        (0x0800)    // =1 ==> ctrl-key down in mouse entry
#define ME_ALT_MASK         (0x0400)    // =1 ==> alt-key down in mouse entry
#define ME_SHIFT_MASK       (0x0200)    // =1 ==> shift-key down in mouse entry
#define ME_GUI_MASK         (0x0100)    // =1 ==> gui-key down in mouse entry
#define ME1_LEFT_MASK       (0x0400)    // =1 ==> left mouse button down during move
#define ME1_MID_MASK        (0x0200)    // =1 ==> middle mouse button down during move
#define ME1_RIGHT_MASK      (0x0100)    // =1 ==> right mouse button down during move
#define ME3_LEFT_MASK       (0x1)       // =1 ==> Left mouse button clicked
#define ME3_RIGHT_MASK      (0x2)       // =1 ==> Right mouse button clicked
#define ME3_MID_MASK        (0x4)       // =1 ==> Middle mouse button clicked

// Misc.
#define COIL_A              (0)         // Index value for global vars for coil A
#define COIL_B              (1)         // Index value for global vars for coil B
#define TRIGGER_A           (15)        // Rising trigger level for coil A
#define TRIGGER_B           (15)        // Rising trigger level for coil B
#define TRIGGER(c)          ((c) == 0 ? TRIGGER_A : TRIGGER_B)
#define RESET_A             (10)        // Falling reset level for coil A
#define RESET_B             (10)        // Falling reset level for coil B
#define RESET(c)            ((c) == 0 ? RESET_A : RESET_B)
#define MAX_PULSE_SEP       (40000)     // Maximum separation (μs) between A and B pulses we're sensitive to
#define DEBOUNCE_MILLIS     (10)        // millis() that must pass for us to believe a button has changed state
#define FINGER_MILLIS       (150)       // How long a button chord must persist for us to believe it
#define BANNER              (F("JogWheel v1.0"))

// Processor dependencies -- ATmega328P and ATmega32U4 supported
#if defined(__AVR_ATmega32U4__)
    #define TIMERx_COMPA_vect   TIMER3_COMPA_vect
    #define TCCRxA              TCCR3A
    #define TCCRxB              TCCR3B
    #define OCRxA               OCR3A
    #define OCIExA              OCIE3A
    #define WGMx1               WGM31
    #define CSx0                CS30
    #define CSx1                CS31
    #define TIMSKx              TIMSK3
#elif defined(__AVR_ATmega328P__)
    #define KEY_UP_ARROW        (0xDA)
    #define KEY_DOWN_ARROW      (0xD9)
    #define TIMERx_COMPA_vect   TIMER2_COMPA_vect
    #define TCCRxA              TCCR2A
    #define TCCRxB              TCCR2B
    #define OCRxA               OCR2A
    #define OCIExA              OCIE2A
    #define WGMx1               WGM21
    #define CSx0                CS20
    #define CSx1                CS21
    #define TIMSKx              TIMSK2
#else
    #warning Unsupported processor!
#endif

/****
 * 
 * Globals
 * 
 ****/

// Types
enum movement_t : byte {cw, none, cc};                              // Shaft movement
enum coilState_t : byte {low, rising, rose};                        // Timer 2 interrupt state machine states
struct headerBlock {                                                // The configuration header (from EEPROM)
    uint16_t fingerprint;                                           // Fingerprint to say this is ours
    uint8_t selection;                                              // The configuration selected (index into curconfig): 0..7
    uint8_t curConfig[7];                                           // Which configuration corresponds with which combo of button pushes
    uint16_t configPtr[8];                                          // Addresses in EEPROM of start of each configuraton 
};

struct configBlock {                                                // The shape of a configuration
    uint8_t nEntries;                                               // The number of entries in this configuration block 0..31
    uint16_t entry[32][2];                                          // Space for the max number of entries entry[?][0] is action for cw, [1] is cc
};
// Variables
const byte coilPin[2] = {COIL_A_PIN, COIL_B_PIN};                   // Coil index to pin map
const char* ledColor[7] = {"red    ", "green  ", "yellow ", "blue   ", 
                           "magenta", "cyan   ", "white  "};        // LED colors corresponding to header.selection
UserInput ui {Serial};                                              // Our user input object from library UserInput
movement_t movement = {none};                                       // What sort of movement was detected (if any)
                                                                    // Set to cc or cw by ISR, reset to none by loop()
headerBlock header;                                                 // Copy of header from EEPROM

#ifdef DEBUG_ISR
byte dStateIx = 0;                                                  // How many states recorded
coilState_t dState[D_STATE_SIZE][2];                                // The states recorded for debugging
int dCoilVal[D_STATE_SIZE][2];                                      // coilVal[] at each recorded state
unsigned long dRisingTimestamp[D_STATE_SIZE][2];                    // risingTimestamp at entry to state *rising*
movement_t dMovement[D_STATE_SIZE];                                 // movement at each recorded state
#endif

/****
 * 
 * Timer ISR. Which timer is used depends on the processor; see #define 
 * statements, above. Turned on in setup() to occur once every 256μs. 
 * 
 * Because this replaces the ISR Arduino uses for the tone() function, 
 * that function can't be used in this sketch.
 * 
 * The ISR implements two identical state machines, one for each coil. A 
 * coil's machine is in one of three states: *low* (its initial state), 
 * *rising*, or *rose*. 
 * 
 * In state *low*, the machine is waiting for the induced voltage on its coil 
 * to rise. If, while in state *low*, the voltage rises above TRIGGER(c) the 
 * machine for the coil enters state *rising*. (Where c is the coil index -- 0 
 * for coil A, 1 for coil B.) If the voltage has not risen, the machine stays 
 * in state *low*.
 * 
 * In state *rising* the machine sets movement to *cc* or *cw* if movement has 
 * been newly detected. Movement has been newly detected if movement is *none* 
 * and machine for the other coil has recently entered the *rising* state. In 
 * any event, the machine then changes state to *rose*. 
 * 
 * In state *rose* the machine is waiting for the induced voltage to drop 
 * below RESET(c), where c is the coil index. If it has, the machine switches 
 * to state *low*. Otherwise it remains in state "rose*."
 * 
 ****/
ISR(TIMERx_COMPA_vect) {
    static coilState_t state[2] = {low, low};                           // State of each coil's state machine
    static unsigned long risingTimestamp[2] = {0, 0};                   // micros() at the point the time *rising* was last entered
    int coilVal[2];

    #ifdef DEBUG_ISR
    coilState_t lastStateA = state[0];
    coilState_t lastStateB = state[1];
    #endif

    // Update state machine as needed
    for (byte c = 0; c < 2; c++) {
        coilVal[c] = analogRead(coilPin[c]);
        switch (state[c]) {
            case low:
                if (coilVal[c] > TRIGGER(c)) {
                    state[c] = rising;
                }
                break;
            case rising:
                if (movement == none) {
                    risingTimestamp[c] = micros();
                    if (risingTimestamp[c] - risingTimestamp[(c + 1) % 2] <= MAX_PULSE_SEP) {
                        movement = c == 0 ? cc : cw;
                    }
                }
                state[c] = rose;
                break;
            case rose:
                if (coilVal[c] < RESET(c)) {
                    state[c] = low;
                }
                break;
        }
    }
    #ifdef DEBUG_ISR
    if ((state[0] != lastStateA || state[1] != lastStateB) && dStateIx < D_STATE_SIZE) {
        for (byte c = 0; c < 2; c++) {
            dState[dStateIx][c] = state[c];
            dCoilVal[dStateIx][c] = coilVal[c];
            dRisingTimestamp[dStateIx][c] = risingTimestamp[c];
        }
        dMovement[dStateIx] = movement;
        dStateIx++;
    }
    #endif
}

/****
 * 
 * EEPROM helper functions
 * 
 * The EEPROM is used to hold the configuration data. This consists of a 
 * header block at location 0 and up to 8 configuration blocks (CBs) at 
 * locations specified in the header block. Unused configurations have 0 for 
 * their locations. 
 * 
 * A CB consists of a count of the entries in it and the entries themselves. 
 * In EEPROM, the CBs are kept packed together starting right after the 
 * header.
 * 
 * The helper functions can read and write the header and can read, remove, 
 * and add CBs, up to the maximum of 8 and the available space in the EEPROM, 
 * which is only 1024 bytes. The 0th CB describes the default configuration 
 * and cannot be removed.
 * 
 * The routines operate on the globals header and config. (Cheap and dirty, I 
 * know.)
 * 
 * There are also a few special helper functions. One to set which CB 
 * describes the current configuration, one to calculate how much free space 
 * there it in EEPROM and one to return the number of currently defined 
 * configs.
 * 
 * NB: Except for readHeader(), all the functions assume, without checking, 
 * that the header data has already been rertieved.
 * 
 ****/

// Write the header to EEPROM.
void writeHeader() {
    #ifdef DEBUG_EEPROM
    if (header.curConfig < 0 || (size_t)header.curConfig > N_CONFIGS){
        Serial.print(F("writeHeader - Bad header.curConfig: "));
        Serial.println(header.curConfig);
    }
    if ((size_t)header.configPtr[header.curConfig] < sizeof(header) || header.configPtr[header.curConfig] > 1021) {
        Serial.print(F("writeHeader - Bad header.configPtr["));
        Serial.print(header.curConfig);
        Serial.print(F("]: "));
        Serial.println(header.configPtr[header.curConfig]);
    }
    #endif
    eeprom_write_block((void*)&header, (void*)0, sizeof(header));
}

// Write config block n, n:[0..N_CONFIGS - 1]
void writeConfig(int8_t cbn, configBlock &cb) {
    if (cbn < 0 || (uint8_t)cbn >= N_ELEMENTS(header.configPtr)) {
        cb.nEntries = 0;
        #ifdef DEBUG_EEPROM
        Serial.print(F("writeConfig - Bad cbn: "));
        Serial.println(cbn);
        #endif
        return;
    }
    #ifdef DEBUG_EEPROM
    if ((size_t)header.configPtr[cbn] < sizeof(header) || header.configPtr[cbn] > 1021) {
        Serial.print(F("writeConfig - Bad header.configPtr["));
        Serial.print(cbn);
        Serial.print(F("]: "));
        Serial.println(header.configPtr[cbn], HEX);
    }
    if (cb.nEntries < 1 || (size_t)cb.nEntries > sizeof(cb.entry) / sizeof(cb.entry[0])) {
        Serial.print(F("writeConfig - Bad cb.nEntries: "));
        Serial.println(cb.nEntries);
    }
    Serial.print(F("writeConfig -- write parms: 0x"));
    Serial.print((uint16_t)&cb, HEX);
    Serial.print(F(", 0x"));
    Serial.print(header.configPtr[cbn], HEX);
    Serial.print(F(", 0x"));
    Serial.println(sizeof(cb.nEntries) + sizeof(cb.entry[0]) * cb.nEntries, HEX);
    #endif
    // Write config, including the entries (all cb.nEntries of them) for config number cbn
    eeprom_write_block((void*)&cb, (void*)header.configPtr[cbn], sizeof(cb.nEntries) + sizeof(cb.entry[0]) * cb.nEntries);
}

// Read config block n, n:[0..N_CONFIGS - 1] into passed cb. Assumes header is properly populated.
bool readConfig(int8_t cbn, configBlock &cb) {
    if (cbn < 0 || (uint8_t)cbn >= N_ELEMENTS(header.configPtr)) {
        cb.nEntries = 0;
        #ifdef DEBUG_EEPROM
        Serial.print(F("readConfig - Bad cbn: "));
        Serial.println(cbn);
        #endif
        return false;
    }
    #ifdef DEBUG_EEPROM
    if ((size_t)header.configPtr[cbn] < sizeof(header) || header.configPtr[cbn] > 1021) {
        Serial.print(F("readConfig - Bad header.configPtr["));
        Serial.print(cbn);
        Serial.print(F("]: "));
        Serial.println(header.configPtr[cbn], HEX);
    }
    #endif
    // Retrieve value of cb.nEntries for config number cbn
    eeprom_read_block((void*)&cb.nEntries, (const void*)header.configPtr[cbn], sizeof(cb.nEntries));
    #ifdef DEBUG_EEPROM
    if (cb.nEntries < 1 || (size_t)cb.nEntries > sizeof(cb.entry) / sizeof(cb.entry[0])) {
        Serial.print(F("readConfig - Bad cb.nEntries: "));
        Serial.println(cb.nEntries);
    }
    #endif
    // Retrieve the entries (all cb.nEntries of them) for config number cbn
    int16_t entryEepromAddr = header.configPtr[cbn] + sizeof(cb.nEntries);
    #ifdef DEBUG_EEPROM
    Serial.print(F("readConfig - Reading "));
    Serial.print(cb.nEntries);
    Serial.print(F(" entries for config "));
    Serial.print(cbn);
    Serial.print(F(" at EEPROM addr 0x"));
    Serial.print(header.configPtr[cbn], HEX);
    Serial.print(F(". Read parms: 0x"));
    Serial.print((uint16_t)&cb.entry, HEX);
    Serial.print(F(", 0x"));
    Serial.print((uint16_t)entryEepromAddr, HEX);
    Serial.print(F(", "));
    Serial.println(sizeof(cb.entry[0]) * cb.nEntries);
    delay(100);
    #endif
    eeprom_read_block((void*)&cb.entry, (const void*)entryEepromAddr, sizeof(cb.entry[0]) * cb.nEntries);
    return true;
}

// Read header block into header. If what we read doesn't have the right fingerprint, initialize to the
// starter configuration set.
bool readHeader() {
    eeprom_read_block((void*)&header, (const void*)0, sizeof(header));
    #ifdef FACTORY_RESET
    header.fingerprint = 0; // Force regen each time
    #endif
    if (header.fingerprint == FINGERPRINT) {    // It's ours; use it

        return true;
    } else {                                    // Not ours; synthesize the default
        configBlock cb;
        header.fingerprint = FINGERPRINT;
        header.selection = 1;
        for (uint8_t i = 0; i < N_ELEMENTS(header.curConfig); i++) {
            header.curConfig[i] = 0;            // Set all button combos to config 0
        }
        header.configPtr[0] = sizeof(header);   // Config 0 starts right after header
        for (byte i = 1; i < N_ELEMENTS(header.configPtr); i++) {
            header.configPtr[i] = 0;            // Rest are unused
        }
        cb.nEntries = 1;                        // Config 0 is k0xDA 0xD9
        cb.entry[0][ENTRY_CW] = KEY_UP_ARROW;
        cb.entry[0][ENTRY_CC] = KEY_DOWN_ARROW;
        writeHeader();                          // Initialize eeprom
        writeConfig(0, cb);
    }
    return false;
}

// Set the current config for button combination combo to configuration number cbn. 
// The result is that the header is updated in EEPROM (if needed).
// Returns true if succeeded, false if passed invalid combo or invalid or currently unused cbn.
bool setConfig(uint8_t combo, uint8_t cbn) {
    if (cbn >= N_ELEMENTS(header.configPtr) || combo >= N_ELEMENTS(header.curConfig) || header.configPtr[cbn] == 0) {
        return false;
    }
    header.curConfig[combo] = cbn;
    writeHeader();
    return true;
}

// Return the number of bytes of free space remain in EEPROM
size_t freeSpace() {
    size_t fs = 1024 - sizeof(header);
    configBlock cb;
    for (uint8_t cbn = 0; cbn < N_ELEMENTS(header.configPtr) && header.configPtr[cbn] != 0; cbn++) {
        readConfig(cbn, cb);
        fs -= cb.nEntries * sizeof(cb.entry[0]);
    }
    #ifdef DEBUG_EEPROM
    Serial.print(F("freeSpace -- Remaining: "));
    Serial.println(fs);
    #endif
    return fs;
}

// Return the current number of configs there are. Relies on header being set.
uint8_t nConfigs() {
    uint8_t answer = 1;
    while (answer < N_ELEMENTS(header.configPtr)) {
        if (header.configPtr[answer] == 0) {
            return answer;
        }
        answer++;
    }
    return answer;
}

// Remove config block cbn, compact the header's list of configs, the higher 
// numbered CBs and revise the combo to config map, as needed. If a combo is  
// using the config that's being removed, set the it to use the default 
// config. Success returns true; failure, false.
bool removeConfig(uint8_t cbn) {
    configBlock cb;
    if (cbn < 1 || cbn >= N_ELEMENTS(header.configPtr) || header.configPtr[cbn] == 0) {
        return false;
    }
    for (uint8_t combo = 0; combo < N_ELEMENTS(header.curConfig); combo++) {
        if (header.curConfig[combo] == cbn) {
            header.curConfig[combo] = 0;        // if combo uses config being removed, use default
        }
        if (header.curConfig[combo] > cbn) {
            header.curConfig[combo]--;          // Account for compaction, if needed
        }
    }
    readConfig(cbn, cb);
    size_t delta = cb.nEntries * sizeof(cb.entry[0]) + sizeof(cb.nEntries);
    for (uint8_t cbi = cbn + 1; cbi < N_ELEMENTS(header.configPtr); cbi++) {
        if (header.configPtr[cbi] == 0) {
            header.configPtr[cbi - 1] = 0;
            break;
        }
        readConfig(cbi, cb);
        uint16_t newPtr = header.configPtr[cbi] - delta;
        header.configPtr[cbi] = 0;
        writeConfig(cbi - 1, cb);
        header.configPtr[cbi - 1] = newPtr;
    }
    writeHeader();
    return true;
}


// Add config block to the end of the list of current configs. Returns true if 
// successful, false if it won't fit in what's left in EEPROM
bool addConfig(configBlock toAdd) {
    configBlock cb;
    if (toAdd.nEntries * sizeof(toAdd.entry[0]) > freeSpace() || header.configPtr[N_ELEMENTS(header.configPtr) - 1] != 0) {
        #ifdef DEBUG_EEPROM
        Serial.print(F("addConfig -- Can't fit new config into eeprom."));
        #endif
        return false;
    }
    int8_t cbn = 1;
    while (header.configPtr[cbn] != 0) {
        cbn++;
    }
    readConfig(cbn - 1, cb);
    header.configPtr[cbn] = header.configPtr[cbn - 1] + sizeof(cb.nEntries) + cb.nEntries * sizeof(cb.entry[0]);
    #ifdef DEBUG_EEPROM
    Serial.print(F("Adding new config at "));
    Serial.print(cbn);
    Serial.print(F(" at eeprom address 0x"));
    Serial.print(header.configPtr[cbn], HEX);
    Serial.print(F(" with size "));
    Serial.println(sizeof(toAdd.nEntries) + sizeof(toAdd.entry[0]) * toAdd.nEntries);
    #endif
    writeConfig(cbn, toAdd);
    writeHeader();
    return true;
}

/****
 * 
 * Misc helper functions
 * 
 ****/

// void printConfigK(uint entry[2]) Format and print both parts of a keystroke entry
void printConfigK(uint16_t entry[]) {
    for (uint8_t dir = 0; dir < 2; dir++) {
        if ((entry[dir] & KB_CTRL_MASK) != 0) {
            Serial.print(F("c"));
        }
        if ((entry[dir] & KB_ALT_MASK) != 0){
            Serial.print(F("a"));
        }
        if ((entry[dir] & KB_SHIFT_MASK) != 0) {
            Serial.print(F("s"));
        }
        if ((entry[dir] & KB_GUI_MASK) != 0) {
            Serial.print(F("g"));
        }
        char ks = entry[dir] & KB_VALUE_MASK;
        if (isPrintable(ks) && ks > ' ') {
            Serial.print(F("\'"));
            Serial.print(ks);
        } else {
            Serial.print(F("0x"));
            Serial.print((uint8_t)ks, HEX);
        }
        Serial.print(F(" "));
    }
}

// void printConfigM(uint16_t entryA[], entryB[]) Format and print both parts of a mouse move double entry
void printConfigM(uint16_t entryA[], uint16_t entryB[]) {
    for (uint8_t dir = 0; dir < 2; dir++) {
        if ((entryB[dir] & ME_CTRL_MASK) != 0) {
            Serial.print(F("c"));
        }
        if ((entryB[dir] & ME_ALT_MASK) != 0){
            Serial.print(F("a"));
        }
        if ((entryB[dir] & ME_SHIFT_MASK) != 0) {
            Serial.print(F("s"));
        }
        if ((entryB[dir] & ME_GUI_MASK) != 0) {
            Serial.print(F("g"));
        }
        if ((entryA[dir] & ME1_LEFT_MASK) != 0) {
            Serial.print(F("l"));
        }
        if ((entryA[dir] & ME1_MID_MASK) != 0){
            Serial.print(F("m"));
        }
        if ((entryA[dir] & ME1_RIGHT_MASK) != 0) {
            Serial.print(F("r"));
        }
        int8_t val = entryA[dir] & ME_VALUE_MASK;
        if (val >= 0) {
            Serial.print(F("+"));
        }
        Serial.print(val);
        val = entryB[dir] & ME_VALUE_MASK;
        if (val >= 0) {
            Serial.print(F("+"));
        }
        Serial.print(val);
        Serial.print(F(" "));
    }
}

// printConfigW(uint16_t entry[])  Format and print both parts of a wheel roll entry
void printConfigW(uint16_t entry[]) {
    for (uint8_t dir = 0; dir < 2; dir++) {
        if ((entry[dir] & ME_CTRL_MASK) != 0) {
            Serial.print(F("c"));
        }
        if ((entry[dir] & ME_ALT_MASK) != 0){
            Serial.print(F("a"));
        }
        if ((entry[dir] & ME_SHIFT_MASK) != 0) {
            Serial.print(F("s"));
        }
        if ((entry[dir] & ME_GUI_MASK) != 0) {
            Serial.print(F("g"));
        }
        int8_t val = entry[dir] & ME_VALUE_MASK;
        if (val >= 0) {
            Serial.print(F("+"));
        }
        Serial.print(val);
        Serial.print(F(" "));
    }
}

// printConfigC(uint16_t entry[])  Format and print both parts of a mouse click entry
void printConfigC(uint16_t entry[]) {
    for (uint8_t dir = 0; dir < 2; dir++) {
        if ((entry[dir] & ME_CTRL_MASK) != 0) {
            Serial.print(F("c"));
        }
        if ((entry[dir] & ME_ALT_MASK) != 0){
            Serial.print(F("a"));
        }
        if ((entry[dir] & ME_SHIFT_MASK) != 0) {
            Serial.print(F("s"));
        }
        if ((entry[dir] & ME_GUI_MASK) != 0) {
            Serial.print(F("g"));
        }
        if ((entry[dir] & ME3_LEFT_MASK) != 0){
            Serial.print(F("l"));
        }
        if ((entry[dir] & ME3_MID_MASK) != 0) {
            Serial.print(F("m"));
        }
        if ((entry[dir] & ME3_RIGHT_MASK) != 0) {
            Serial.print(F("r"));
        }
        Serial.print(F(" "));
    }
}

// parseK() -- Parse keyboard spec
uint16_t parseK(String spec) {
    String sp = spec;
    uint16_t answer = 0;        // Keyboard entry (no flags in most-significant nibble)
    char nc;
    bool bad = false;
    for (int8_t i = 0; i < 5; i++) {
        nc = sp.charAt(0);
        sp.remove(0, 1);
        if (nc == 'C' || nc == 'c') {
            answer |= KB_CTRL_MASK;
        } else if (nc == 'A' || nc == 'a') {
            answer |= KB_ALT_MASK;
        } else if (nc == 'S' || nc == 's') {
            answer |= KB_SHIFT_MASK;
        } else if (nc == 'G' || nc == 'g') {
            answer |= KB_GUI_MASK;
        } else {
            break;
        }
    }
    if (nc == '\'') {
        nc = sp.charAt(0);
        sp.remove(0, 1);
        if (!isPrintable(nc)) {
            bad = true;
            Serial.print(F("Invalid character value in"));
        }
        answer |= (nc & 0x7F);
        answer = answer & ~KB_SHIFT_MASK; // Clear shift mod for printable chars
    } else if (nc == '0') {
        nc = sp.charAt(0);
        sp.remove(0, 1);
        uint8_t val;
        if ((nc == 'x' || nc == 'X') && sp.length() == 2) {
            char hh = sp.charAt(0);
            char hl = sp.charAt(1);
            if (!isHexadecimalDigit(hh) || !isHexadecimalDigit(hl)) {
                bad = true;
                Serial.print(F("Invalid hex number in"));
                val = 0;
            } else {
                hh = hh <= '9' ? hh - '0' : hh < 'F' ? hh - 'A' + 10 : hh - 'a' + 10;
                hl = hl <= '9' ? hl - '0' : hl < 'F' ? hl - 'A' + 10 : hl - 'a' + 10;
                val = (hh << 4) + hl;
            }            
        } else {
            bad = true;
            Serial.print(F("Keystroke value not 0x0 to 0xFF in"));
        }
        answer |= val;
    } else {
        bad = true;
        Serial.print(F("Invalid"));
    }
    if (bad) {
        Serial.print(" keyboard spec: ");
        Serial.println(spec);
        return 0;
    }
    #ifdef DEBUG_PARSE
    Serial.print(F("parseK - answer: 0x"));
    Serial.print(answer, HEX);
    Serial.println(F(". Parsed successfully."));
    #endif
    return answer;
}

// parseM() -- Parse mouse movement spec
uint32_t parseM(String spec) {
    uint16_t answer[2] = {CE_TYPE_MASK, CE_TYPE_MASK};
    SET_ME_TYPE(answer[0], ME_TYPE_X);      // Type 1 mouse entries
    SET_ME_TYPE(answer[1], ME_TYPE_Y);
    String sp = spec;
    char nc;
    bool bad = false;
    for (int8_t i = 0; i < 8; i++) {
        nc = sp.charAt(0);
        sp.remove(0, 1);
        if (nc == 'C' || nc == 'c') {
            answer[1] |= ME_CTRL_MASK;
        } else if (nc == 'A' || nc == 'a') {
            answer[1] |= ME_ALT_MASK;
        } else if (nc == 'S' || nc == 's') {
            answer[1] |= ME_SHIFT_MASK;
        } else if (nc == 'G' || nc == 'g') {
            answer[1] |= ME_GUI_MASK;
        } else if (nc == 'L' || nc == 'l') {
            answer[0] |= ME1_LEFT_MASK;
        } else if (nc == 'M' || nc == 'm') {
            answer[0] |= ME1_MID_MASK;
        } else if (nc == 'R' || nc == 'r') {
            answer[0] |= ME1_RIGHT_MASK;
        } else {
            break;
        }
    }
    for (uint8_t ap = 0; ap < 2; ap++) {
        bool isPos;
        if (nc == '+') {
            isPos = true;
        } else if (nc == '-') {
            isPos = false;
        } else {
            bad = true;
            Serial.print(F("Invalid"));
        }
        nc = sp.charAt(0);
        sp.remove(0, 1);
        if (!isdigit(nc)) {
            bad = true;
            Serial.print(F("Distance missing in"));
        }
        int8_t val = 0;
        for (uint8_t digit = 0; !bad && digit < 3; digit++) {
            val = val * 10 + (nc - '0');
            nc = sp.charAt(0);
            sp.remove(0, 1);
            if (!isdigit(nc)) {
                break;
            }
        }
        if (!bad) {
            if (val > 255) {
                bad = true;
                Serial.print(F("Distance, "));
                Serial.print(val);
                Serial.print(F(", must be <= 255. in"));
            } else {
                answer[ap] |= (isPos ? val : -val) & ME_VALUE_MASK;
            }
        }
        if (bad) {
            Serial.print(" mouse spec: ");
            Serial.println(spec);
            return 0;
        }
    }
    #ifdef DEBUG_PARSE
    Serial.print(F("parseM - entry 0: 0x"));
    Serial.print(answer[0], HEX);
    Serial.print(F(" entry 1: 0x"));
    Serial.print(answer[1], HEX);
    Serial.println(F(". Parsed successfully."));
    #endif
    return ((uint32_t)answer[0] << 16) | answer[1];
}

// parseW() parse mouse wheel-roll spec
uint16_t parseW(String spec) {
    String sp = spec;
    uint16_t answer = CE_TYPE_MASK;     //Type 0 (wheel) mouse entry
    char nc;
    bool bad = false;
    for (int8_t i = 0; i < 5; i++) {
        nc = sp.charAt(0);
        sp.remove(0, 1);
        if (nc == 'C' || nc == 'c') {
            answer |= KB_CTRL_MASK;
        } else if (nc == 'A' || nc == 'a') {
            answer |= KB_ALT_MASK;
        } else if (nc == 'S' || nc == 's') {
            answer |= KB_SHIFT_MASK;
        } else if (nc == 'G' || nc == 'g') {
            answer |= KB_GUI_MASK;
        } else {
            break;
        }
    }
    bool isPos;
    if (nc == '+') {
        isPos = true;
    } else if (nc == '-') {
        isPos = false;
    } else {
        bad = true;
        Serial.print(F("Invalid"));
    }
    nc = sp.charAt(0);
    sp.remove(0, 1);
    if (!isdigit(nc)) {
        bad = true;
        Serial.print(F("Wheel amount missing in"));
    }
    int8_t val = 0;
    for (uint8_t digit = 0; !bad && digit < 3; digit++) {
        val = val * 10 + (nc - '0');
        nc = sp.charAt(0);
        sp.remove(0, 1);
        if (!isdigit(nc)) {
            break;
        }
    }
    if (!bad) {
        if (val > 255) {
            bad = true;
            Serial.print(F("Wheel amount, "));
            Serial.print(val);
            Serial.print(F(", must be <= 255. in"));
        } else {
            answer |= (isPos ? val : -val) & ME_VALUE_MASK;
        }
    }
    if (bad) {
        Serial.print(" wheel spec: ");
        Serial.println(spec);
        return 0;
    }
    #ifdef DEBUG_PARSE
    Serial.print(F("parseW - answer: 0x"));
    Serial.print(answer, HEX);
    Serial.println(F(". Parsed successfully."));
    #endif
    return answer;
}

// parseC() parse mouse wheel-roll spec
uint16_t parseC(String spec) {
    String sp = spec;
    bool bad = false;
    uint16_t answer = CE_TYPE_MASK;     //Type 2 (click) mouse entry
    SET_ME_TYPE(answer, ME_TYPE_CLICK);
    char nc;
    for (int8_t i = 0; i < 5; i++) {
        nc = sp.charAt(0);
        sp.remove(0, 1);
        if (nc == 'C' || nc == 'c') {
            answer |= KB_CTRL_MASK;
        } else if (nc == 'A' || nc == 'a') {
            answer |= KB_ALT_MASK;
        } else if (nc == 'S' || nc == 's') {
            answer |= KB_SHIFT_MASK;
        } else if (nc == 'G' || nc == 'g') {
            answer |= KB_GUI_MASK;
        } else {
            break;
        }
    }
    if ((uint8_t)nc == 0) {
        bad = true;
        Serial.print(F("Missing which button(s) to click"));
    } else {
        for (int8_t i = 0; i < 3 && (uint8_t)nc != 0; i++) {
            if (nc == 'L' || nc == 'l') {
                answer |= ME3_LEFT_MASK;
            } else if (nc == 'M' || nc == 'm') {
                answer |= ME3_MID_MASK;
            } else if (nc == 'R' || nc == 'r') {
                answer |= ME3_RIGHT_MASK;
            } else {
                bad = true;
                Serial.print(F("Invalid button name: "));
                Serial.print(nc);
                break;
            }
            nc = sp.charAt(0);
            sp.remove(0, 1);
        }
    }
    if (bad) {
        if (spec.length() != 0) {
            Serial.print(F(" in: "));
            Serial.println(spec);
        } else {
            Serial.println(F(" in empty <c-spec>"));
        }
        return 0;
    }
    #ifdef DEBUG_PARSE
    Serial.print(F("parseC - answer: 0x"));
    Serial.print(answer, HEX);
    Serial.println(F(". Parsed successfully."));
    #endif
    return answer;
}

// uint8_t toCbn(String token) Convert token to configuration block number. Returns N_CONFGS if toke is not a valid integer in the required range
uint8_t toCbn(String token) {
    uint8_t n = token.toInt();
    if (!isDigit(token.charAt(0)) || token.length() == 0 || n < 0 || n >= N_ELEMENTS(header.configPtr) || header.configPtr[n] == 0) {
        return N_ELEMENTS(header.configPtr);
    }
    return n;
}

/****
 * 
 * ui command handlers
 * 
 ****/

// Unknown
void onUnknown() {
    Serial.println(F("Unknown or unimplemented command."));
}

// help | h [new] Displays list of commands
void onHelp() {
    if (ui.getWord(1).equals("new")) {
        Serial.println(F("JogWheel new command help\n"
                         "To make a new configuration, type \"new <config>\" where\n"
                         "  <config> = <spec> ( <spec>)*\n"
                         "There can be up to 32 specs per configuration (16 for mouse moves), separated by whitespace.\n"
                         "  <spec> = (K|k)<k-spec> <k-spec> | (M|m)<m-spec> <m-spec> | (W|w)<w-spec> <w-spec> | (C|c)<c-spec> <c-spec>\n"
                         "The first <*-spec> in a pair tells what to do on a clockwise click of the jogwheel. The other does the same for counterclockwise.\n"
                         "K means the action is a keystroke, M means a mouse movement spec, W means a mouse wheel roll, and C means a mouse click.\n"
                         "  <k-spec> = <k-modifiers><keystroke>\n"
                         "  <m-spec> = <k-modifiers><m-modifiers><x-dist><y-dist>\n"
                         "  <w-spec> = <k-modifiers><m-modifiers><wheel-amt>\n"
                         "  <c-spec> = <k-modifiers><m-button>\n"
                         "  <k-modifiers> = [(c|C)][(a|A)][(s|S)][(g|G)]\n"
                         "  <m-modifiers> = [(l|L)][(m|M)][(r|R)]\n"
                         "  <keystroke> = \'<printable-char> | (0X|0x)<hex-digit><hex-digit>\n"
                         "  <x-dist> = <signed-num>\n"
                         "  <y-dist> = <signed-num>\n"
                         "  <m-button> = (l|L)|(m|M)|(r|R)\n"
                         "  <signed-num> = (+|-)[<dec-digit>][<dec-digit>]<dec-digit> (whose value must be -255..+255)\n"
                         "  <dec-digit> = (0..9)\n"
                         "  <hex-digit> = (0..9)|(A..F)|(a..f)\n"
                         "  <printable-char> = a printable ascii character, including \'\n"
                         "For example, \"k0xDA 0xD9\" is the default config."));
    } else {
        Serial.println(F("JogWheel command list:\n"
                         "  help [new]      Display this list of commands or the help for the new command\n"
                         "  h [new]         Same as help\n"
                         "  display         Display a list of the configurations\n"
                         "  d               Same as display\n"
                         "  new <config>    Specify a new configuration. (Type \"help new\" for help)\n"
                         "  n <config>      Same as new\n"
                         "  use <c> <n>     Use configuration <n> for button combo <c>. <c> = 0: all up .. c = 7: all down\n"
                         "  u <c> <n>       Same as use\n"
                         "  remove <n>      Remove configuration <n>, 1 <= <n> <= 7\n"
                         "  r <n>           Same as remove"));
    }
}

// display | d  Display the list of available configurations
void onDisplay() {
    Serial.println(F("Button combination to configuraton map"));
    Serial.println(F("Combo  Color   Config Number"));
    for (uint8_t i = 0; i < N_ELEMENTS(header.curConfig); i++){
        Serial.print(F("    "));
        Serial.print(i + 1);
        Serial.print(F("  "));
        Serial.print(ledColor[i]);
        Serial.print(F(" "));
        Serial.println(header.curConfig[i]);
    }
    Serial.println(F("Configuration number to <config> map"));
    Serial.println(F("Number  <config>"));
    for (uint8_t cbn = 0; cbn < N_ELEMENTS(header.configPtr) && header.configPtr[cbn] != 0; cbn++) {
        configBlock cb;
        Serial.print(F("     "));
        Serial.print(cbn);
        Serial.print(F("  "));
        readConfig(cbn, cb);
        for (int8_t enN = 0; enN < cb.nEntries; enN++) {
            uint16_t entryA[2] = {cb.entry[enN][0], cb.entry[enN][1]};
            uint16_t entryB[2];
            char eType;
            if ((entryA[0] & CE_TYPE_MASK) == 0) {
                eType = 'k';
            }else {
                uint8_t meType = ME_TYPE(entryA[0]);
                eType = meType == ME_TYPE_CLICK ? 'c' : meType == ME_TYPE_WHEEL ? 'w' : 'm';
            }
            Serial.print(eType);
            switch (eType) {
                case 'k':
                    printConfigK(entryA);
                    break;
                case 'm':
                    if (enN >= cb.nEntries) {
                        Serial.print("Invalid move entry! enN: ");
                        Serial.print(enN);
                        break;
                    }
                    enN++;
                    entryB[0] = cb.entry[enN][0];
                    entryB[1] = cb.entry[enN][1];
                    printConfigM(entryA, entryB);
                    break;
                case 'w':
                    printConfigW(entryA);
                    break;
                case 'c':
                    printConfigC(entryA);
                    break;
                default:
                    Serial.print(F("Unrecognized type of configuration entry: "));
                    Serial.println(eType);
            }
        }
        Serial.print(F("\n"));
    }
    Serial.print(F("There are "));
    Serial.print(freeSpace());
    Serial.println(F(" bytes free for configurations."));
}

// new | n <config> Add the new configuration <config> to the list of available configurations
void onNew() {
    configBlock cb;
    cb.nEntries = 0;
    bool bad = false;
    uint8_t eNum = 0;
    while (cb.nEntries < sizeof(cb.entry) / sizeof(cb.entry[0])) {
        String spec[2];
        spec[0] = ui.getWord(1 + 2*eNum);
        spec[1] = ui.getWord(2 + 2*eNum);
        eNum++;
        if (spec[0].length() == 0) {
            break;
        }
        if (spec[1].length() == 0) {
            bad = true;
            Serial.println(F("Missing last <spec-cc>."));
        }
        uint16_t entry = 0;
        uint32_t doubleEntry = 0;
        uint8_t st = spec[0].charAt(0);
        spec[0].remove(0, 1);
        if (st == 'M' || st == 'm') {
            if (cb.nEntries + 1 >= (uint8_t)(sizeof(cb.entry) / sizeof(cb.entry[0]))) {
                bad = true;
                Serial.println(F("Too many entries for a config."));
            }
            for (uint8_t dir = 0; dir < 2; dir++) {
                doubleEntry = parseM(spec[dir]);
                if (doubleEntry == 0) {
                    bad = true;
                } else {
                    #ifdef DEBUG
                    Serial.print(F("parseM -- double entry: 0x"));
                    Serial.print(doubleEntry, HEX);
                    Serial.print(F(", cb.nEntries: "));
                    Serial.print(cb.nEntries);
                    Serial.print(F(", dir: "));
                    Serial.println(dir);
                    #endif
                    cb.entry[cb.nEntries][dir] = doubleEntry >> 16;
                    cb.entry[cb.nEntries + 1][dir] = doubleEntry & 0xFFFF;
                }
            }
            if (!bad) {
                cb.nEntries += 2;
            }
        } else {
            for (uint8_t dir = 0; dir < 2; dir ++) {
                if (st == 'K' || st == 'k') {
                    entry = parseK(spec[dir]);
                } else if (st == 'W' || st == 'w') {
                    entry = parseW(spec[dir]);
                } else if (st == 'C' || st == 'c') {
                    entry = parseC(spec[dir]);
                } else {
                    Serial.print(F("Invalid <spec> type: \'"));
                    Serial.print((char)st);
                    Serial.println(F("\'. Must be \'k\', \'m\', \'w'\' or \'c\'."));
                }
                if (entry == 0) {
                    bad = true;
                } else {
                    cb.entry[cb.nEntries][dir] = entry;
                }
            }
            if (!bad) {
                cb.nEntries++;
            }
        }
        if (bad) {
            break;
        }
    }
    if (bad) {
        Serial.println(F("Could not add specification. Type \'help new\' for help."));
        return;
    }
    #ifdef DEBUG
    Serial.print(F(" Adding config:"));
    for (uint8_t e = 0; e < cb.nEntries; e++) {
        Serial.print(F(" "));
        Serial.print(cb.entry[e][0], HEX);
        Serial.print(F(" "));
        Serial.print(cb.entry[e][1], HEX);
    }
    Serial.print(F("\n"));
    #endif
    addConfig(cb);
}

// use | u <n> Use the configuration number <n>
void onUse() {
    int8_t combo = ui.getWord(1).toInt();
    uint8_t cbn = toCbn(ui.getWord(2));
    if (combo < 0 || (uint8_t)combo >= N_ELEMENTS(header.curConfig) || cbn == N_ELEMENTS(header.configPtr)) {
        Serial.print(F("To set which configuration to use, type \'use <combo> <n>\' where <combo> is the button combination to set and\n"
                       "<n> is the number of the configuration to use. Where 0 <= <combo> <= "));
        Serial.print(N_ELEMENTS(header.curConfig) - 1);
        Serial.print(F(" and currently, 0 <= <n> <= "));
        Serial.println(nConfigs() - 1);
        return;
    }
    header.curConfig[combo] = cbn;
    writeHeader();
}

// remove || r <n> Remove configuration number <n> from the list of configurations
void onRemove() {
    uint8_t n = toCbn(ui.getWord(1));
    if (n == 0 || n == N_ELEMENTS(header.configPtr)) {
        Serial.print(F("To remove a configuration, type \'remove <n>\' where <n> is the configuration number. Currently, 1 <= <n> <= "));
        Serial.println(nConfigs() - 1);
    }
    removeConfig(n);
}

/****
 * 
 * Arduino setup() function.
 * 
 ****/
void setup() {
    // Setup GPIO pins
    pinMode(COIL_A_PIN, INPUT);
    pinMode(COIL_B_PIN, INPUT);
    pinMode(LED_R, OUTPUT);
    pinMode(LED_G, OUTPUT);
    pinMode(LED_B, OUTPUT);
    pinMode(BUTTON_A, INPUT_PULLUP);
    pinMode(BUTTON_B, INPUT_PULLUP);
    pinMode(BUTTON_C, INPUT_PULLUP);
    Serial.begin(9600);
    while(!Serial && millis()<5000) {
        // Wait up to 5s to see if the serial monitor connects (needed for Leo-like boards)
    }
    
    #ifdef __AVR_ATmega32U4__
    // Initialize the Keyboard and Mouse libraries
    Keyboard.begin();
    Mouse.begin();
    #endif

    // Attach the command handlers for the ui
    ui.attachDefaultCmdHandler(onUnknown);
    bool succeeded = 
    ui.attachCmdHandler("help", onHelp) &&
    ui.attachCmdHandler("h", onHelp) &&
    ui.attachCmdHandler("display", onDisplay) &&
    ui.attachCmdHandler("d", onDisplay) &&
    ui.attachCmdHandler("new", onNew) &&
    ui.attachCmdHandler("n", onNew) &&
    ui.attachCmdHandler("use", onUse) &&
    ui.attachCmdHandler("u", onUse) &&
    ui.attachCmdHandler("remove", onRemove) &&
    ui.attachCmdHandler("r", onRemove);
    if (!succeeded) {
        Serial.println(F("Too many UI command handlers."));
    }

    // Initialize our configuration header
    readHeader();

    // Set up our Timer ISR -- which one depends on processor type
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        TCCRxA = 0x00;
        TCCRxB = 0x00;                  // Stop Timer
        OCRxA = 0xFF;                   // Set number (256) of 2μs clock ticks per interrupt == 1 every 512μs
        TCCRxA = _BV(WGMx1);            // WGMx[2:0] = 0x2 i.e., Set "Clear Timer on Compare" (CTC) mode
        TCCRxB = _BV(CSx1) | _BV(CSx0); // CSx[2:0] = 0x3  i.e., Clock/64 (From prescaler)
        TIMSKx |= _BV(OCIExA);          // Turn on OCRxA compare interrupts
    }

    // Hello world!
    Serial.println(BANNER);
    Serial.println(F("Ready. Type \"help\" for list of commands."));
}

/****
 * 
 * Arduino loop() function
 * 
 ****/
void loop() {
    // If the wheel moved, deal with it.
    if (movement != none) {
        #ifdef DEBUG
        #ifdef DEBUG_ISR
        Serial.println(movement == cw ? F("+") : F("-"));
        #else
        Serial.print(movement == cw ? F("+") : F("-"));
        #endif
        #endif
        uint8_t dir = movement == cw ? 0 : 1;
        configBlock cb;
        readConfig(header.curConfig[header.selection], cb);
        for (uint8_t e = 0; e < cb.nEntries; e++) {
            uint16_t entryA = cb.entry[e][dir];
            char eType;
            uint8_t m;
            uint16_t entryB;
            if ((entryA & CE_TYPE_MASK) == 0) {
                eType = 'k';
            } else {
                uint8_t meType = ME_TYPE(cb.entry[e][dir]);
                eType = meType == ME_TYPE_CLICK ? 'c' : meType == ME_TYPE_WHEEL ? 'w' : 'm';
            }
            #ifndef __AVR_ATmega32U4__
            Serial.print(eType);
            #endif
            switch (eType) {
                case 'k':
                    #ifdef __AVR_ATmega32U4__
                    if ((entryA & KB_CTRL_MASK) != 0) {
                        Keyboard.press(KEY_LEFT_CTRL);
                    }
                    if ((entryA & KB_ALT_MASK) != 0) {
                        Keyboard.press(KEY_LEFT_ALT);
                    }
                    if ((entryA & KB_SHIFT_MASK) != 0) {
                        Keyboard.press(KEY_LEFT_SHIFT);
                    }
                    if ((entryA & KB_GUI_MASK) != 0) {
                        Keyboard.press(KEY_LEFT_GUI);
                    }
                    Keyboard.press(entryA & KB_VALUE_MASK);
                    Keyboard.releaseAll();
                    #else
                    Serial.print(F("0x"));
                    Serial.print(entryA, HEX);
                    #endif
                    break;
                case 'm':
                    #ifdef __AVR_ATmega32U4__
                    m = (entryA & ME1_LEFT_MASK) != 0 ? MOUSE_LEFT : 0;
                    m |= (entryA & ME1_MID_MASK) != 0 ? MOUSE_MIDDLE : 0;
                    m |= (entryA & ME1_RIGHT_MASK) != 0 ? MOUSE_RIGHT : 0;
                    Mouse.press(m);
                    e++;
                    entryB = cb.entry[e][dir];
                    if ((entryB & ME_CTRL_MASK) != 0) {
                        Keyboard.press(KEY_LEFT_CTRL);
                    }
                    if ((entryB & ME_ALT_MASK) != 0) {
                        Keyboard.press(KEY_LEFT_ALT);
                    }
                    if ((entryB & ME_SHIFT_MASK) != 0) {
                        Keyboard.press(KEY_LEFT_SHIFT);
                    }
                    if ((entryB & ME_GUI_MASK) != 0) {
                        Keyboard.press(KEY_LEFT_GUI);
                    }
                    Mouse.move(entryA & ME_VALUE_MASK, entryB & ME_VALUE_MASK, 0);
                    Keyboard.releaseAll();
                    Mouse.release(MOUSE_LEFT | MOUSE_MIDDLE | MOUSE_RIGHT);
                    #else
                    Serial.print(F("0x"));
                    Serial.print(cb.entry[e][dir], HEX);
                    Serial.print(F(" 0x"));
                    e++;
                    Serial.print(cb.entry[e][dir], HEX);
                    #endif
                    break;
                case 'w':
                    #ifdef __AVR_ATmega32U4__
                    if ((entryA & ME_CTRL_MASK) != 0) {
                        Keyboard.press(KEY_LEFT_CTRL);
                    }
                    if ((entryA & ME_ALT_MASK) != 0) {
                        Keyboard.press(KEY_LEFT_ALT);
                    }
                    if ((entryA & ME_SHIFT_MASK) != 0) {
                        Keyboard.press(KEY_LEFT_SHIFT);
                    }
                    if ((entryA & ME_GUI_MASK) != 0) {
                        Keyboard.press(KEY_LEFT_GUI);
                    }
                    Mouse.move(0, 0, entryA & ME_VALUE_MASK);
                    Keyboard.releaseAll();
                    #else
                    Serial.print(F("0x"));
                    Serial.print(entryA, HEX);
                    #endif
                    break;
                case 'c':
                    #ifdef __AVR_ATmega32U4__
                    if ((entryA & ME_CTRL_MASK) != 0) {
                        Keyboard.press(KEY_LEFT_CTRL);
                    }
                    if ((entryA & ME_ALT_MASK) != 0) {
                        Keyboard.press(KEY_LEFT_ALT);
                    }
                    if ((entryA & ME_SHIFT_MASK) != 0) {
                        Keyboard.press(KEY_LEFT_SHIFT);
                    }
                    if ((entryA & ME_GUI_MASK) != 0) {
                        Keyboard.press(KEY_LEFT_GUI);
                    }
                    m = (entryA & ME3_LEFT_MASK) != 0 ? MOUSE_LEFT : 0;
                    m |= (entryA & ME3_MID_MASK) != 0 ? MOUSE_MIDDLE : 0;
                    m |= (entryA & ME3_RIGHT_MASK) != 0 ? MOUSE_RIGHT : 0;
                    Mouse.click(m);
                    Keyboard.releaseAll();
                    #else
                    Serial.print(F("0x"));
                    Serial.print(entryA, HEX);
                    #endif
                    break;
                default:
                    #ifndef __AVR_ATmega32U4__
                    Serial.print(F("loop() -- Unrecognized type of configuration entry: "));
                    Serial.println(eType);
                    #endif
                    break;
            }
            #ifndef __AVR_ATmega32U4__
            Serial.print(F(" "));
            #endif
        }
        #ifndef __AVR_ATmega32U4__
        Serial.print(F("\n"));
        #endif
        movement = none;
    }

    // Do button stuff
    static uint8_t button[3] = {0, 0, 0};
    static unsigned long buttonMillis[3] = {0, 0, 0};
    bool curButton[3] = {digitalRead(BUTTON_A) == LOW, digitalRead(BUTTON_B) == LOW, digitalRead(BUTTON_C) == LOW};
    unsigned long curMillis = millis();
    // Debounce the buttons
    for (byte i = 0; i < 3; i++) {
        if(curButton[i] != button[i]) {
            if (buttonMillis[i] == 0) {
                buttonMillis[i] = curMillis;
            } else if (curMillis - buttonMillis[i] > DEBOUNCE_MILLIS) {
                button[i] = curButton[i];
                buttonMillis[i] = 0;
            }
        } else {
            buttonMillis[i] = 0;
        }
    }

    // Determine what button chord the user intends, if any. Chords are entered by pressing a combination 
    // of the three buttons and then releasing them. The buttons that were down just before release is the chord
    // the user entered.
    static uint8_t pendingCombo = 0;
    static unsigned long pendingMillis = 0;
    uint8_t curCombo = button[2] << 2 | button[1] << 1 | button[0];
    if (pendingCombo == 0) {                    // If first time through
        pendingCombo = header.selection + 1;
        digitalWrite(LED_R, (pendingCombo & 1) != 0 ? HIGH : LOW);
        digitalWrite(LED_G, (pendingCombo & 2) != 0 ? HIGH : LOW);
        digitalWrite(LED_B, (pendingCombo & 4) != 0 ? HIGH : LOW);
    }
    if (curCombo != 0) {
        if (pendingCombo != curCombo) {
            if (pendingMillis == 0) {
                pendingMillis = curMillis;
            } else if (curMillis - pendingMillis > FINGER_MILLIS) {
                pendingCombo = curCombo;
                pendingMillis = 0;
                digitalWrite(LED_R, button[0] ? HIGH : LOW);
                digitalWrite(LED_G, button[1] ? HIGH : LOW);
                digitalWrite(LED_B, button[2] ? HIGH : LOW);
                #ifdef DEBUG
                Serial.print(F("Chord: "));
                Serial.println(pendingCombo);
                #endif
            }
        } else {
            pendingMillis = 0;
        }
    } else if (pendingCombo - 1 != header.selection) {
        header.selection = pendingCombo - 1;
        writeHeader();
        #ifdef DEBUG
        Serial.print(F("Selection set to "));
        Serial.print(ledColor[header.selection]);
        Serial.print(F(" ("));
        Serial.print(header.selection);
        Serial.println(F(")"));
        #endif
    }
      
    // Do UI stuff
    ui.run();

    #ifdef DEBUG_ISR
    // Dump collected ISR stats, if needed
    if (dStateIx >= D_STATE_SIZE) {
        Serial.println(F("Recorded state:"));
        for (byte ix = 0; ix < D_STATE_SIZE; ix++) {
            Serial.print(F("Sample "));
            Serial.print(ix);
            Serial.print(F(" moved: "));
            Serial.print(dMovement[ix] == none ? F("no ") : dMovement[ix] == cc ? F("cc ") : F("cw "));
            for (byte c = 0; c < 2; c++) {
                coilState_t s = dState[ix][c];
                Serial.print(F(" coil "));
                Serial.print(c == 0 ? F("A: ") : F("B: "));
                Serial.print(s == low ? F("  low") : s == rising ? F(" rising") : F(" rose"));
                Serial.print(F(", val: "));
                Serial.print(dCoilVal[ix][c]);
                Serial.print(F(", ts: "));
                Serial.print(dRisingTimestamp[ix][c]);
                Serial.print(c == 1 ? F("\n") : F(", "));
            }
        }
        dStateIx = 0;
    }
    #endif
}