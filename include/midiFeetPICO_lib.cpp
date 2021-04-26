// ignores most of the code
// cSpell:ignoreRegExp /(^(?!\s*(\/\/)|(\/\*)).*[;:)}=,{])/gm

// ignores any word in quotes
// cSpell:ignoreRegExp /\"\S*\"/g

//--- ignores HEX literals
// cSpell:ignoreRegExp /0x[A-Z]+/g

//--- ignores any preprocessor directive (i.e #define)
// cSpell:ignoreRegExp /(^#.*)/gm

/// words to ignore
// cSpell:ignore pico PSRAM btn btns spec'd dbgserPrintln dbgser Println gpio ADCs VBUS VSHUNT RSHUNT LSBs BVOLTAGERANGE BADCRES SADCRES CNVR

/// spell check extension defaults to checking each part of camel case words as separate words.

#include "midiFeetPICO_lib.h"
#include "hardware/i2c.h"
// #include "hardware/irq.h"
// #include "pico/binary_info.h"
// #include "pico/multicore.h"
#include "pico/stdlib.h"
#include <stdio.h>
// #include <string.h>

uint8_t InputPortManager::update() {
    // value to return
    uint8_t edgeEvent_R = 0;

    // check sense lines to see if a port has something plugged in or not.
    // if something was plugged in now and wasn't before, see if the mode for the
    // port is auto, and if so suss out the mode.
    for (uint8_t i = 0; i < 8; i++) {
        // read the input and send that value to the edgetracker. 
        senseEdgeTack[i] += gpio_get(8 + i);

        // if this particular edge tracker has hit the threshhold of an edge evnt...
        if (senseEdgeTack[i].getEdgeEvent()) {
            printf("\r\nline %d experienced an edge event.", i);
            printf("\r\ncombst & 0xf0 : %d", senseEdgeTack[i].combSt & 0xf0);
            // combSt & 0xf0 == 16 means plugged in
            // combSt & 0xf0 == 8 means unplugged
        }
    }

    // Read each of the channels of the ADCs.
    // Since update() gets called quite often, check to see if a conversion is ready. If not, move along...
    // This will keep update() from blocking longer than it takes to read or write to a register on the ADC.
    if (AD_conv1->conversionComplete() && AD_conv2->conversionComplete()) {
        // When a conversion is ready...
        // Get the conversion from the ADC, compare it to a threshhold, and add the value to edgetracker.
        int16_t tempVal = AD_conv1->getLastConversionResults();

        // Compare ADC value to threshhold and store a new value in the edge tracker. 
        tipRingEdgeTrack[loopTrack] += (tempVal < ADC_LOW_THRESHERHOLD);
        // if(loopTrack==0) printf("tempVAl: %x\n",tempVal);

        // Store the ADC value in the average-er
        this->expPedals[loopTrack] += tempVal;

        // Do the same for the opther ADC
        tempVal = AD_conv2->getLastConversionResults();
        tipRingEdgeTrack[loopTrack + 4] += (tempVal < ADC_LOW_THRESHERHOLD);
        this->expPedals[loopTrack + 4] += tempVal;

        loopTrack++;
        if (loopTrack > 3) {
            loopTrack = 0;
        }

        // Count up the number of edge events that occurred on the tips and rings so it can be returned.
        if (tipRingEdgeTrack[loopTrack].getEdgeEvent() || tipRingEdgeTrack[loopTrack + 4].getEdgeEvent())edgeEvent_R++;

        // Now that we have gotten a complete A->D conversion and stored the values, lets get the next channel's data.
        // Interleave reads/writes of the ADCs so that they perform their conversions (sort of) at the same time
        AD_conv1->readADC_SingleEnded_NON_BLOCKING(loopTrack);
        AD_conv2->readADC_SingleEnded_NON_BLOCKING(loopTrack);
    }
    // return count of inputs with edge event. 
    return edgeEvent_R;
}
MCP23017_Port_Expander::MCP23017_Port_Expander() { };
void MCP23017_Port_Expander::begin() {
    // set defaults!
    // all outputs on port A and B
    writeRegister(MCP23017_IODIRA, 0x00);
    writeRegister(MCP23017_IODIRB, 0x00);
    // Turn off interrupt triggers
    writeRegister(MCP23017_GPINTENA, 0x00);
    writeRegister(MCP23017_GPINTENB, 0x00);
    // Turn off pull up resistors
    writeRegister(MCP23017_GPPUA, 0x00);
    writeRegister(MCP23017_GPPUB, 0x00);
    uint8_t i = 0;

    // input port pullups
    for (; i < 8; i++) {
        digitalWrite(i, 1);
    }
    // digitalWrite(1, 0);

    // outputports
    for (; i < 16; i++) {
        digitalWrite(i, 0);
    }
}
void MCP23017_Port_Expander::pinMode(uint8_t p, uint8_t d) {
    updateRegisterBit(p, (d == 1), MCP23017_IODIRA, MCP23017_IODIRB);
}
void MCP23017_Port_Expander::writeGPIOAB(uint16_t ba) {
    uint8_t data[3] = { MCP23017_GPIOA, uint8_t(ba & 0x00ff), uint8_t(ba >> 8) };
    int er = i2c_write_blocking_until(i2c1, I2CADDR_PORT_EXPANDER, data, 3, false, make_timeout_time_ms(transferTimeout_ms));
}
void MCP23017_Port_Expander::digitalWrite(uint8_t pin, uint8_t d) {
    uint8_t gpio;
    uint8_t bit = bitForPin(pin);
    // read the current GPIO output latches
    uint8_t regAddr = regForPin(pin, MCP23017_OLATA, MCP23017_OLATB);
    gpio = readRegister(regAddr);
    // set the pin and direction
    bitWrite(gpio, bit, d);
    // write the new GPIO
    regAddr = regForPin(pin, MCP23017_GPIOA, MCP23017_GPIOB);
    writeRegister(regAddr, gpio);
    if (pin < 8)
        bitSet(portA_state, pin);
    else
        bitSet(portB_state, pin - 8);
}
uint8_t MCP23017_Port_Expander::bitForPin(uint8_t pin) { return pin % 8; }
uint8_t MCP23017_Port_Expander::regForPin(uint8_t pin, uint8_t portAaddr, uint8_t portBaddr) {
    return (pin < 8) ? portAaddr : portBaddr;
}
uint8_t MCP23017_Port_Expander::readRegister(uint8_t addr) {
      // printf("readReg\r\n");
    uint8_t data[1] = { addr };
    int er = i2c_write_blocking_until(i2c1, I2CADDR_PORT_EXPANDER, data, 1, false, make_timeout_time_ms(transferTimeout_ms));
    i2c_read_blocking_until(i2c1, I2CADDR_PORT_EXPANDER, data, 1, false, make_timeout_time_ms(transferTimeout_ms));
    return data[0];
}
void MCP23017_Port_Expander::writeRegister(uint8_t regAddr, uint8_t regValue) {
      // printf("writeReg\r\n");
      // Write the register
    uint8_t data[2] = { regAddr, regValue };
    int er = i2c_write_blocking_until(i2c1, I2CADDR_PORT_EXPANDER, data, 2, false, make_timeout_time_ms(transferTimeout_ms));
}
void MCP23017_Port_Expander::updateRegisterBit(uint8_t pin, uint8_t pValue, uint8_t portAaddr, uint8_t portBaddr) {
    uint8_t regValue;
    uint8_t regAddr = regForPin(pin, portAaddr, portBaddr);
    uint8_t bit = bitForPin(pin);
    regValue = readRegister(regAddr);
    // set the value for the particular bit
    bitWrite(regValue, bit, pValue);
    writeRegister(regAddr, regValue);
}
Adafruit_ADS1015::Adafruit_ADS1015(uint8_t i2c_addr) {
    m_bitShift = 4;
    m_gain = GAIN_ONE; /* +/- 6.144V range (limited to VDD +0.3V max!) */
    m_dataRate = RATE_ADS1015_3300SPS;
    address = i2c_addr;
    readADC_SingleEnded_NON_BLOCKING(0);
}
int16_t Adafruit_ADS1015::readADC_SingleEnded_BLOCKING(uint8_t channel) {
    if (channel > 3) {
        return 0;
    }
    uint16_t config =
        ADS1X15_REG_CONFIG_CQUE_NONE |   // Disable the comparator (default val)
        ADS1X15_REG_CONFIG_CLAT_NONLAT | // Non-latching (default val)
        ADS1X15_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
        ADS1X15_REG_CONFIG_CMODE_TRAD | // Traditional comparator (default val)
        ADS1X15_REG_CONFIG_MODE_SINGLE; // Single-shot mode (default)
    config |= m_gain;
    config |= m_dataRate;

    switch (channel) {
        case (0):
            config |= ADS1X15_REG_CONFIG_MUX_SINGLE_0;
            break;
        case (1):
            config |= ADS1X15_REG_CONFIG_MUX_SINGLE_1;
            break;
        case (2):
            config |= ADS1X15_REG_CONFIG_MUX_SINGLE_2;
            break;
        case (3):
            config |= ADS1X15_REG_CONFIG_MUX_SINGLE_3;
            break;
    }
    config |= ADS1X15_REG_CONFIG_OS_SINGLE;
    writeRegister(ADS1X15_REG_POINTER_CONFIG, config);
    while (!conversionComplete()) { }
    return getLastConversionResults();
}
void Adafruit_ADS1015::readADC_SingleEnded_NON_BLOCKING(uint8_t channel) {
    if (channel > 3) {
        return;
    }
    uint16_t config =
        ADS1X15_REG_CONFIG_CQUE_NONE |   // Disable the comparator (default val)
        ADS1X15_REG_CONFIG_CLAT_NONLAT | // Non-latching (default val)
        ADS1X15_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
        ADS1X15_REG_CONFIG_CMODE_TRAD | // Traditional comparator (default val)
        ADS1X15_REG_CONFIG_MODE_SINGLE; // Single-shot mode (default)
    config |= m_gain;
    config |= m_dataRate;

    switch (channel) {
        case (0):
            config |= ADS1X15_REG_CONFIG_MUX_SINGLE_0;
            break;
        case (1):
            config |= ADS1X15_REG_CONFIG_MUX_SINGLE_1;
            break;
        case (2):
            config |= ADS1X15_REG_CONFIG_MUX_SINGLE_2;
            break;
        case (3):
            config |= ADS1X15_REG_CONFIG_MUX_SINGLE_3;
            break;
    }
    config |= ADS1X15_REG_CONFIG_OS_SINGLE;
    writeRegister(ADS1X15_REG_POINTER_CONFIG, config);
}
int16_t Adafruit_ADS1015::getLastConversionResults() {
  // Read the conversion results
    uint16_t res = readRegister(ADS1X15_REG_POINTER_CONVERT);
    return (int16_t)res;
}
void Adafruit_ADS1015::setGain(adsGain_t gain) { m_gain = gain; }
adsGain_t Adafruit_ADS1015::getGain() { return m_gain; }
void Adafruit_ADS1015::setDataRate(uint16_t rate) { m_dataRate = rate; }
uint16_t Adafruit_ADS1015::getDataRate() { return m_dataRate; }
bool Adafruit_ADS1015::conversionComplete() {
    bool ready = (readRegister(ADS1X15_REG_POINTER_CONFIG) & 0x8000) != 0;
    if (ready) {
        mostRecentConversionChannel = readInProgressChannel;
        readingInProgress = false;
        // printf(".");
    }
    return ready;
}
bool Adafruit_ADS1015::readADC_SingleEnded_nonBlocking_Async(uint8_t channel) {
    if (conversionComplete()) {
        readingInProgress = true;
        readInProgressChannel = channel;
        readADC_SingleEnded_NON_BLOCKING(channel);
        return true;
    } else {
        return false;
    }
}
bool readingInProgress = false;
int readInProgressChannel = -1;
uint8_t mostRecentConversionChannel = 0;
void Adafruit_ADS1015::writeRegister(uint8_t reg, uint16_t value) {
    buffer[0] = reg;
    buffer[1] = value >> 8;
    buffer[2] = value & 0xFF;
    i2c_write_blocking_until(i2c1, address, buffer, 3, false, make_timeout_time_ms(100));
}
uint16_t Adafruit_ADS1015::readRegister(uint8_t reg) {
    buffer[0] = reg;
    i2c_write_blocking_until(i2c1, address, buffer, 1, false, make_timeout_time_ms(100));
    i2c_read_blocking_until(i2c1, address, buffer, 2, false, make_timeout_time_ms(100));
    return ((buffer[0] << 8) | buffer[1]);
}
i2c_slave_handler::i2c_slave_handler() { }
void i2c_slave_handler::readDataIn(uint8_t numBytes, bool flush) {
    if (numBytes == 0) { return; } // Reading 0 bytes in does nothing.
    while (i2c_get_read_available(i2c0) < numBytes) { } // expect data to show up in i2c HW buffer
    while (dataInLock) { } // wait for lock to be released. Should only be set by other core.
    this->dataInLock = true; // lock data_in
    i2c_read_raw_blocking(i2c0, this->data_in, numBytes);
    this->dataInLock = false; // unlock data_in
    if (flush) {
      // if there's still data in the HW buffer, flush it out. This will prevent
      // bad data from creeping in
      // in the event the master sent too many bytes.
        while (i2c_get_read_available(i2c0) > 0) {
            i2c_read_raw_blocking(i2c0, this->bufferFlush, 1);
        }
    }
}
TEENSY_I2C::TEENSY_I2C(i2c_slave_handler& handler) { this->handler = &handler; }
uint8_t TEENSY_I2C::getReadLength() {
      // command<TeensyCommands.numOf_T2P_commands?TeensyCommands.messageSize[command]:0;
    this->lastInCommand = handler->data_in[0];
    if (this->lastInCommand < numOf_T2P_commands) {
        return T2P_messageSize[this->lastInCommand];
    } else {
        return 0;
    }
}
OutputPortManager::OutputPortManager(MCP23017_Port_Expander& exp, uint8_t _id) :expander(&exp), id(_id) { }
void OutputPortManager::UpdateOutput() {
    printf("writing to port# %i\t value:%s\n", (this->id * 2) + 8, this->state.Tip_on_b ? "true" : "false");
    printf("writing to port# %i\t value:%s\n", (this->id * 2) + 9, this->state.Ring_on_b ? "true" : "false");
    expander->digitalWrite((this->id * 2) + 8, this->state.Tip_on_b ? 1 : 0);
    expander->digitalWrite((this->id * 2) + 9, this->state.Ring_on_b ? 1 : 0);
}
InputPortManager::InputPortManager(Adafruit_ADS1015& conv1, Adafruit_ADS1015& conv2) {
    this->AD_conv1 = &conv1;
    this->AD_conv2 = &conv2;
}
InputPortManager::Input_debounce_edge_state_tracker::Input_debounce_edge_state_tracker() { }
uint8_t InputPortManager::Input_debounce_edge_state_tracker::countBits() {
    uint32_t temp = this->trackLSB;
    uint8_t count = 0;
    
    for (uint8_t i = 0; i < 8; i++) {
        count += (temp & 1);
        temp = (temp >> 1);
    }
    // temp = this->trackMSB;
    // for (uint8_t i = 0; i < 32; i++) {
    //     count += (temp & 1);
    //     temp = (temp >> 1);
    // }
    return count;
}
void InputPortManager::Input_debounce_edge_state_tracker::updateEdgeTracker(bool val) {
    // this->trackMSB = (this->trackMSB << 1) + (this->trackLSB >> 31); // Shift trackMSB to the left and add the MSB of trackLSB to it
    this->trackLSB = (this->trackLSB << 1) + (val ? 1 : 0); // shift in the newest bit
    uint8_t numBitsSet = countBits();
    if (numBitsSet > this->highThresh) {
        if (combSt >= combinedState::state_CLOSED_unplugged) { // rising edge occurred
            combSt = combinedState::state_OPEN_pluggedIn | combinedState::risingEdgeEvent;
            edgeEvent = true;
        }
    } else if (numBitsSet < this->lowThresh) {
        if ((combSt & 0xf0) <=
            combinedState::state_OPEN_pluggedIn) { // falling edge occurred
            combSt = combinedState::state_CLOSED_unplugged | combinedState::fallingEdgeEvent;
            edgeEvent = true;
        }
    }
}
void InputPortManager::Input_debounce_edge_state_tracker::updateEdgeTracker(bool val, uint8_t mult) {
    if (mult > 16)
        mult = 16;
    for (uint8_t i = 0; i < mult; i++) {
        updateEdgeTracker(val);
    }
}
bool InputPortManager::Input_debounce_edge_state_tracker::getEdgeEvent() {
    if (this->edgeEvent) {
        this->edgeEvent = false;
        return true;
    }
    return false;
}
uint8_t InputPortManager::detect(uint8_t port) {
      //@todo Need to implement auto sensing.
    return 0;
}
bool InputPortManager::getInputState(uint8_t trackerNum) {
    return this->tipRingEdgeTrack[trackerNum].combSt;
}
Adafruit_INA219::Adafruit_INA219(uint8_t addr) {
    ina219_i2caddr = addr;
    ina219_currentDivider_mA = 0;
    ina219_powerMultiplier_mW = 0.0f;
}
void Adafruit_INA219::setCalibration_16V_400mA() {
    ina219_calValue = 8192;
    // Set multipliers to convert raw current/power values
    ina219_currentDivider_mA = 20; // Current LSB = 50uA per bit (1000/50 = 20)
    ina219_powerMultiplier_mW = 1.0f; // Power LSB = 1mW per bit
    // Set Calibration register to 'Cal' calculated above
    uint8_t data[3] = { INA219_REG_CALIBRATION, uint8_t(ina219_calValue & 0x00ff), uint8_t(ina219_calValue >> 8) };
    i2c_write_blocking_until(i2c1, this->ina219_i2caddr, data, 3, true, make_timeout_time_ms(5));
    // Set Config register to take into account the settings above
    uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
        INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_9BIT |
        INA219_CONFIG_SADCRES_9BIT_1S_84US |
        INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
    data[0] = INA219_REG_CALIBRATION;
    data[1] = config & 0xff;
    data[2] = config >> 8;
    _success = 2 == i2c_write_blocking_until(i2c1, this->ina219_i2caddr, data, 3, true, make_timeout_time_ms(5));
}
float Adafruit_INA219::getBusVoltage_V() {
    int16_t value = getBusVoltage_raw();
    return value * 0.001;
}
float Adafruit_INA219::getShuntVoltage_mV() {
    int16_t value;
    value = getShuntVoltage_raw();
    return value * 0.01;
}
float Adafruit_INA219::getCurrent_mA() {
    float valueDec = getCurrent_raw();
    valueDec /= ina219_currentDivider_mA;
    return valueDec;
}
float Adafruit_INA219::getPower_mW() {
    float valueDec = getPower_raw();
    valueDec *= ina219_powerMultiplier_mW;
    return valueDec;
}
bool Adafruit_INA219::success() { return this->_success; }
void Adafruit_INA219::init() {
      // Set chip to large range config values to start
    setCalibration_16V_400mA();
}
int16_t Adafruit_INA219::getBusVoltage_raw() {
    uint16_t value;
    uint8_t data[2];
    data[0] = INA219_REG_BUSVOLTAGE;
    i2c_write_blocking_until(i2c1, this->ina219_i2caddr, data, 1, true, make_timeout_time_ms(5));
    _success = 2 == i2c_read_blocking_until(i2c1, this->ina219_i2caddr, data, 2, false, make_timeout_time_ms(5));
    value = data[0] | ((uint16_t(data[1])) << 8);
    // Shift to the right 3 to drop CNVR and OVF and multiply by LSB
    return (int16_t)((value >> 3) * 4);
}
int16_t Adafruit_INA219::getShuntVoltage_raw() {
    uint16_t value;
    uint8_t data[2];
    data[0] = INA219_REG_SHUNTVOLTAGE;
    i2c_write_blocking_until(i2c1, this->ina219_i2caddr, data, 1, true, make_timeout_time_ms(5));
    _success = 2 == i2c_read_blocking_until(i2c1, this->ina219_i2caddr, data, 2, false, make_timeout_time_ms(5));
    value = data[0] | ((uint16_t(data[1])) << 8);
    return value;
}
int16_t Adafruit_INA219::getCurrent_raw() {
    uint16_t value;
    uint8_t data[3];
    // Sometimes a sharp load will reset the INA219, which will
    // reset the cal register, meaning CURRENT and POWER will
    // not be available ... avoid this by always setting a cal
    // value even if it's an unfortunate extra step
    data[0] = INA219_REG_CALIBRATION;
    data[1] = ina219_calValue & 0xff;
    data[2] = ina219_calValue >> 8;
    _success = 2 == i2c_write_blocking_until(i2c1, this->ina219_i2caddr, data, 3, true, make_timeout_time_ms(5));
    // Now we can safely read the CURRENT register!
    data[0] = INA219_REG_CURRENT;
    i2c_write_blocking_until(i2c1, this->ina219_i2caddr, data, 1, true, make_timeout_time_ms(5));
    _success = 2 == i2c_read_blocking_until(i2c1, this->ina219_i2caddr, data, 2, false, make_timeout_time_ms(5));
    value = data[0] | ((uint16_t(data[1])) << 8);
    return value;
}
int16_t Adafruit_INA219::getPower_raw() {
    uint16_t value;
    uint8_t data[2];
    // Sometimes a sharp load will reset the INA219, which will
    // reset the cal register, meaning CURRENT and POWER will
    // not be available ... avoid this by always setting a cal
    // value even if it's an unfortunate extra step
    data[0] = INA219_REG_CALIBRATION;
    data[1] = ina219_calValue & 0xff;
    data[2] = ina219_calValue >> 8;
    _success = 2 == i2c_write_blocking_until(i2c1, this->ina219_i2caddr, data, 3, true, make_timeout_time_ms(5));
    // Now we can safely read the POWER register!
    data[0] = INA219_REG_POWER;
    i2c_write_blocking_until(i2c1, this->ina219_i2caddr, data, 1, true, make_timeout_time_ms(5));
    _success = 2 == i2c_read_blocking_until(i2c1, this->ina219_i2caddr, data, 2, false, make_timeout_time_ms(5));
    value = data[0] | ((uint16_t(data[1])) << 8);
    return value;
}