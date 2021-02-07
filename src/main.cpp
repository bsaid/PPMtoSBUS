#include <Arduino.h>

#define VERSION V20210207_PPMtoSBUS

#define PPM_PIN PB5
#define NUM_PPM_CHANNELS 15

const uint8_t NUM_PWM_CHANNELS = 12;
const uint8_t pwmChannels[NUM_PWM_CHANNELS] = {PB12, PB13, PB14, PB15, PA0, PB8, PA6, PB1, PA7, PA4, PA2, PA3};

// Queue used between interrupts and parser in the main loop
class QueueFromInterrupt {
    static const uint16_t SIZE = 64;
    uint16_t firstP = 0;
    uint16_t lastP = 0;

    uint32_t time[SIZE];
    uint32_t regs[SIZE];
public:
    uint16_t maxSize = 0;

    bool push(uint32_t t, uint32_t r) {
        time[lastP] = t;
        regs[lastP] = r;
        lastP = (lastP+1)%SIZE;
        if(maxSize < size()) {
            maxSize = size();
        }
        return firstP == lastP;
    }

    uint16_t size() {
        return (lastP+SIZE - firstP)%SIZE;
    }
    
    bool empty() {
        return size() == 0;
    }

    uint32_t getTime() {
        return time[firstP];
    }

    uint32_t getRegs() {
        return regs[firstP];
    }

    bool pop() {
        if(firstP != lastP) {
            firstP = (firstP+1)%SIZE;
            return true;
        }
        return false;
    }
};
QueueFromInterrupt queueFromInterrupt;

// Global parsed values of RC channels
uint16_t pwmCh[NUM_PWM_CHANNELS];
uint16_t ppmCh[NUM_PPM_CHANNELS];
uint32_t lastChannelMicros[32];
uint32_t regsFlags = 0;

void rcInterrupt() {
    // Read all GPIOs on PAx and PBx pins
    uint32_t regs = GPIOA->regs->IDR;
    regs |= (GPIOB->regs->IDR)<<16;
    queueFromInterrupt.push(micros(), regs);
}

//TODO: Modify this function to your preferred output protocol
void printChannels() {
    static uint32_t lastPrintMillis = 0;
    if(millis() - 100 < lastPrintMillis) {
        return;
    }
    //if( !(regsFlags & (1<<PPM_PIN)) ) {
    //    return;
    //}
    if(Serial.availableForWrite() < NUM_PPM_CHANNELS) {
        return;
    }

    Serial.print("PPM: ");
    Serial.print(queueFromInterrupt.maxSize);
    Serial.print(", ");
    for(int i=0; i<NUM_PPM_CHANNELS; i++) {
        Serial.print(ppmCh[i]);
        Serial.print(" ");
    }
    Serial.print("| PWM: ");
    for(int i=0; i<NUM_PWM_CHANNELS; i++) {
        Serial.print(pwmCh[i]);
        Serial.print(" ");
    }
    Serial.println();

    regsFlags &= ~(1 << PPM_PIN);
    lastPrintMillis = millis();
}

bool isValidServoValue(uint16_t value) {
    return value > 800 && value < 2200;
}

void setup() {
    // Debug USB serial
    Serial.begin(115200);

    // Channels output serial
    Serial1.begin(115200);

    // Initialization of all PPM a PWM input pins
    pinMode(PPM_PIN, INPUT_PULLDOWN);
    for(int ch=0; ch<NUM_PWM_CHANNELS; ch++) {
        pinMode(pwmChannels[ch], INPUT_PULLDOWN);
    }

    // Initialization of interrupts
    attachInterrupt(PPM_PIN, rcInterrupt, CHANGE);
    for(int ch=0; ch<NUM_PWM_CHANNELS; ch++) {
        attachInterrupt(pwmChannels[ch], rcInterrupt, CHANGE);
    }

    // Information for the parser
    uint16_t tempPPMch[NUM_PPM_CHANNELS];
    uint8_t lastPPMregister = 0;
    uint32_t lastPPMmicros = 0;
    bool lastPPMstate = false;
    bool pwmLastStates[NUM_PWM_CHANNELS];
    uint32_t pwmLastMicros[NUM_PWM_CHANNELS];

    // Initialization of global registers to zero
    for(int i=0; i<NUM_PWM_CHANNELS; i++) {
        pwmCh[i] = 0;
        pwmLastMicros[i] = 0;
        pwmLastStates[i] = 0;
    }
    for(int i=0; i<NUM_PPM_CHANNELS; i++) {
        ppmCh[i] = 0;
        tempPPMch[i] = 0;
    }

    // Loop
    for(;;) {
        while(queueFromInterrupt.empty()) delay(1);
        uint32_t currentMicros = queueFromInterrupt.getTime();
        uint32_t currentRegs = queueFromInterrupt.getRegs();
        queueFromInterrupt.pop();

        // PPM read
        bool actPPMstate = (currentRegs & (1<<PPM_PIN));
        if(actPPMstate && !lastPPMstate) { // rising edge
            tempPPMch[lastPPMregister] = currentMicros - lastPPMmicros;
            lastPPMregister++;

            if(currentMicros - lastPPMmicros > 5000) {
                tempPPMch[NUM_PPM_CHANNELS-1] = lastPPMregister;
                lastPPMregister = 0;
                bool isValid = true;
                for(int i=0; i<NUM_PPM_CHANNELS; i++) {
                    if( !isValidServoValue(tempPPMch[i]) ) {
                        if(tempPPMch[i] > 5000)
                            break;
                        else
                            isValid = false;
                    }
                }
                if(isValid) {
                    for(int i=0; i<NUM_PPM_CHANNELS; i++) {
                        ppmCh[i] = tempPPMch[i];
                    }
                    regsFlags |= (1<<PPM_PIN);
                }
            }
            lastPPMmicros = currentMicros;
        }
        lastPPMstate = actPPMstate;

        // PWM read
        for(int ch=0; ch<NUM_PWM_CHANNELS; ch++) {
            bool state = currentRegs & (1<<pwmChannels[ch]);
            if(pwmLastStates[ch] != state) {
                if(state) { // rising edge
                    pwmLastMicros[ch] = currentMicros;
                } else { // falling edge
                    uint16_t value = currentMicros - pwmLastMicros[ch];
                    if(isValidServoValue(value)) {
                        pwmCh[ch] = value;
                    }
                }
                pwmLastStates[ch] = state;
            }
        }

        printChannels();
    }
}

void loop() {}
