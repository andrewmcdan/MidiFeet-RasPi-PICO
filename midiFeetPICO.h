#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>

#define I2CADDR_PORT_EXPANDER 0x20
#define I2CADDR_ADC1 0x48
#define I2CADDR_ADC2 0x49
#define I2CADDR_CURR_SENS1 0x40
#define I2CADDR_CURR_SENS2 0x41
#define I2CADDR_CURR_SENS3 0x44
#define I2CADDR_CURR_SENS4 0x45
#define EDGETRACK_HIGH_THRESH 48
#define EDGETRACK_LOW_THRESH 16
#define ALL_TOPS_SENSE_INPUT_MASK 0x55
#define ALL_BTNS_SENSE_INPUT_MASK 0xAA

/*#region bitRead, bitSet, BitClear, BitWrite*/
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue)                                         \
  ((bitvalue) ? bitSet((value), (bit)) : bitClear((value), (bit)))
/*#endregion*/

/*#region defines for ADS1015*/
/*=========================================================================*/
//  ADS1015 code ripped / ported from adafruit arduino library
/*=========================================================================*/
/*=========================================================================
    POINTER REGISTER
    -----------------------------------------------------------------------*/
#define ADS1X15_REG_POINTER_MASK (0x03)      ///< Point mask
#define ADS1X15_REG_POINTER_CONVERT (0x00)   ///< Conversion
#define ADS1X15_REG_POINTER_CONFIG (0x01)    ///< Configuration
#define ADS1X15_REG_POINTER_LOWTHRESH (0x02) ///< Low threshold
#define ADS1X15_REG_POINTER_HITHRESH (0x03)  ///< High threshold
/*=========================================================================*/
/*=========================================================================
    CONFIG REGISTER
    -----------------------------------------------------------------------*/
#define ADS1X15_REG_CONFIG_OS_MASK (0x8000) ///< OS Mask
#define ADS1X15_REG_CONFIG_OS_SINGLE                                           \
  (0x8000) ///< Write: Set to start a single-conversion
#define ADS1X15_REG_CONFIG_OS_BUSY                                             \
  (0x0000) ///< Read: Bit = 0 when conversion is in progress
#define ADS1X15_REG_CONFIG_OS_NOTBUSY                                          \
  (0x8000) ///< Read: Bit = 1 when device is not performing a conversion
#define ADS1X15_REG_CONFIG_MUX_MASK (0x7000) ///< Mux Mask
#define ADS1X15_REG_CONFIG_MUX_DIFF_0_1                                        \
  (0x0000) ///< Differential P = AIN0, N = AIN1 (default)
#define ADS1X15_REG_CONFIG_MUX_DIFF_0_3                                        \
  (0x1000) ///< Differential P = AIN0, N = AIN3
#define ADS1X15_REG_CONFIG_MUX_DIFF_1_3                                        \
  (0x2000) ///< Differential P = AIN1, N = AIN3
#define ADS1X15_REG_CONFIG_MUX_DIFF_2_3                                        \
  (0x3000) ///< Differential P = AIN2, N = AIN3
#define ADS1X15_REG_CONFIG_MUX_SINGLE_0 (0x4000) ///< Single-ended AIN0
#define ADS1X15_REG_CONFIG_MUX_SINGLE_1 (0x5000) ///< Single-ended AIN1
#define ADS1X15_REG_CONFIG_MUX_SINGLE_2 (0x6000) ///< Single-ended AIN2
#define ADS1X15_REG_CONFIG_MUX_SINGLE_3 (0x7000) ///< Single-ended AIN3
#define ADS1X15_REG_CONFIG_PGA_MASK (0x0E00)     ///< PGA Mask
#define ADS1X15_REG_CONFIG_PGA_6_144V (0x0000)   ///< +/-6.144V range = Gain 2/3
#define ADS1X15_REG_CONFIG_PGA_4_096V (0x0200)   ///< +/-4.096V range = Gain 1
#define ADS1X15_REG_CONFIG_PGA_2_048V                                          \
  (0x0400) ///< +/-2.048V range = Gain 2 (default)
#define ADS1X15_REG_CONFIG_PGA_1_024V (0x0600)  ///< +/-1.024V range = Gain 4
#define ADS1X15_REG_CONFIG_PGA_0_512V (0x0800)  ///< +/-0.512V range = Gain 8
#define ADS1X15_REG_CONFIG_PGA_0_256V (0x0A00)  ///< +/-0.256V range = Gain 16
#define ADS1X15_REG_CONFIG_MODE_MASK (0x0100)   ///< Mode Mask
#define ADS1X15_REG_CONFIG_MODE_CONTIN (0x0000) ///< Continuous conversion mode
#define ADS1X15_REG_CONFIG_MODE_SINGLE                                         \
  (0x0100) ///< Power-down single-shot mode (default)
#define ADS1X15_REG_CONFIG_RATE_MASK (0x00E0)  ///< Data Rate Mask
#define ADS1X15_REG_CONFIG_CMODE_MASK (0x0010) ///< CMode Mask
#define ADS1X15_REG_CONFIG_CMODE_TRAD                                          \
  (0x0000) ///< Traditional comparator with hysteresis (default)
#define ADS1X15_REG_CONFIG_CMODE_WINDOW (0x0010) ///< Window comparator
#define ADS1X15_REG_CONFIG_CPOL_MASK (0x0008)    ///< CPol Mask
#define ADS1X15_REG_CONFIG_CPOL_ACTVLOW                                        \
  (0x0000) ///< ALERT/RDY pin is low when active (default)
#define ADS1X15_REG_CONFIG_CPOL_ACTVHI                                         \
  (0x0008) ///< ALERT/RDY pin is high when active
#define ADS1X15_REG_CONFIG_CLAT_MASK                                           \
  (0x0004) ///< Determines if ALERT/RDY pin latches once asserted
#define ADS1X15_REG_CONFIG_CLAT_NONLAT                                         \
  (0x0000) ///< Non-latching comparator (default)
#define ADS1X15_REG_CONFIG_CLAT_LATCH (0x0004) ///< Latching comparator
#define ADS1X15_REG_CONFIG_CQUE_MASK (0x0003)  ///< CQue Mask
#define ADS1X15_REG_CONFIG_CQUE_1CONV                                          \
  (0x0000) ///< Assert ALERT/RDY after one conversions
#define ADS1X15_REG_CONFIG_CQUE_2CONV                                          \
  (0x0001) ///< Assert ALERT/RDY after two conversions
#define ADS1X15_REG_CONFIG_CQUE_4CONV                                          \
  (0x0002) ///< Assert ALERT/RDY after four conversions
#define ADS1X15_REG_CONFIG_CQUE_NONE                                           \
  (0x0003) ///< Disable the comparator and put ALERT/RDY in high state (default)
/*=========================================================================*/

/** Gain settings */
typedef enum {
    GAIN_TWOTHIRDS = ADS1X15_REG_CONFIG_PGA_6_144V,
    GAIN_ONE = ADS1X15_REG_CONFIG_PGA_4_096V,
    GAIN_TWO = ADS1X15_REG_CONFIG_PGA_2_048V,
    GAIN_FOUR = ADS1X15_REG_CONFIG_PGA_1_024V,
    GAIN_EIGHT = ADS1X15_REG_CONFIG_PGA_0_512V,
    GAIN_SIXTEEN = ADS1X15_REG_CONFIG_PGA_0_256V
} adsGain_t;

/**  data rates  */
#define RATE_ADS1015_128SPS (0x0000)  ///< 128 samples per second
#define RATE_ADS1015_250SPS (0x0020)  ///< 250 samples per second
#define RATE_ADS1015_490SPS (0x0040)  ///< 490 samples per second
#define RATE_ADS1015_920SPS (0x0060)  ///< 920 samples per second
#define RATE_ADS1015_1600SPS (0x0080) ///< 1600 samples per second (default)
#define RATE_ADS1015_2400SPS (0x00A0) ///< 2400 samples per second
#define RATE_ADS1015_3300SPS (0x00C0) ///< 3300 samples per second
#define RATE_ADS1115_8SPS (0x0000)   ///< 8 samples per second
#define RATE_ADS1115_16SPS (0x0020)  ///< 16 samples per second
#define RATE_ADS1115_32SPS (0x0040)  ///< 32 samples per second
#define RATE_ADS1115_64SPS (0x0060)  ///< 64 samples per second
#define RATE_ADS1115_128SPS (0x0080) ///< 128 samples per second (default)
#define RATE_ADS1115_250SPS (0x00A0) ///< 250 samples per second
#define RATE_ADS1115_475SPS (0x00C0) ///< 475 samples per second
#define RATE_ADS1115_860SPS (0x00E0) ///< 860 samples per second
/*#endregion*/

class Adafruit_ADS1015 {
protected:
  // Instance-specific properties
  //   Adafruit_I2CDevice *m_i2c_dev; ///< I2C bus device
    uint8_t m_bitShift;  ///< bit shift amount
    adsGain_t m_gain;    ///< ADC gain
    uint16_t m_dataRate; ///< Data rate

public:
    int16_t readADC_SingleEnded_BLOCKING(uint8_t channel) {
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

    void readADC_SingleEnded_NON_BLOCKING(uint8_t channel) {
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

    int16_t getLastConversionResults() {
      // Read the conversion results
        uint16_t res = readRegister(ADS1X15_REG_POINTER_CONVERT);
        return (int16_t)res;
    }
    void setGain(adsGain_t gain) { m_gain = gain; }
    adsGain_t getGain() { return m_gain; }
    void setDataRate(uint16_t rate) { m_dataRate = rate; }
    uint16_t getDataRate() { return m_dataRate; }
    Adafruit_ADS1015(uint8_t i2c_addr) {
        m_bitShift = 4;
        m_gain = GAIN_ONE; /* +/- 6.144V range (limited to VDD +0.3V max!) */
        m_dataRate = RATE_ADS1115_860SPS;
        address = i2c_addr;
        readADC_SingleEnded_NON_BLOCKING(0);
    }
    bool conversionComplete() {
        bool ready = (readRegister(ADS1X15_REG_POINTER_CONFIG) & 0x8000) != 0;
        if (ready) {
            mostRecentConversionChannel = readInProgressChannel;
            readingInProgress = false;
        }
        return ready;
    }

    bool readADC_SingleEnded_nonBlocking_Async(uint8_t channel) {
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

private:
    void writeRegister(uint8_t reg, uint16_t value) {
        buffer[0] = reg;
        buffer[1] = value >> 8;
        buffer[2] = value & 0xFF;
        i2c_write_blocking_until(i2c1, address, buffer, 3, false, make_timeout_time_ms(100));
    }
    uint16_t readRegister(uint8_t reg) {
        buffer[0] = reg;
        i2c_write_blocking_until(i2c1, address, buffer, 1, false, make_timeout_time_ms(100));
        i2c_read_blocking_until(i2c1, address, buffer, 2, false, make_timeout_time_ms(100));
        return ((buffer[0] << 8) | buffer[1]);
    }
    uint8_t buffer[3];
    uint8_t address = 0;
};

class countTo64 {
private:
    uint8_t value;
    uint8_t maxVal;

public:
    countTo64(void) { value = 0; }
    countTo64(int val) {
        maxVal = 64;
        value = val < maxVal ? val : 0;
    }
    countTo64(int val, int max) {
        maxVal = (max < 257) ? max : 256;
        value = val < maxVal ? val : 0;
    }
    operator int() const { return value; }
    countTo64& operator=(const countTo64& rhs) {
        value = rhs.value;
        return *this;
    }
    countTo64& operator=(const int& rhs) {
        value = rhs;
        return *this;
    }
    countTo64& operator-=(uint8_t val) {
        value = (value >= val) ? (value - val) : maxVal - (val - value);
        return *this;
    }
    countTo64& operator+=(uint8_t val) {
        value = ((value + val) < maxVal) ? (value + val) : val - (maxVal - value);
        return *this;
    }
    countTo64 operator-(int val) const {
        countTo64 r(*this);
        r.value = (r.value >= val) ? (r.value - val) : r.maxVal - (val - r.value);
        return r;
    }
    countTo64 operator-(unsigned int val) const {
        countTo64 r(*this);
        r.value = (r.value >= val) ? (r.value - val) : r.maxVal - (val - r.value);
        return r;
    }
    countTo64 operator-(long val) const {
        countTo64 r(*this);
        r.value = (r.value >= val) ? (r.value - val) : r.maxVal - (val - r.value);
        return r;
    }
    countTo64 operator-(uint8_t val) const {
        countTo64 r(*this);
        r.value = (r.value >= val) ? (r.value - val) : r.maxVal - (val - r.value);
        return r;
    }
    countTo64 operator+(int val) const {
        countTo64 r(*this);
        r.value = ((r.value + val) < r.maxVal) ? (r.value + val)
            : val - (r.maxVal - r.value);
        return r;
    }
    countTo64 operator+(unsigned int val) const {
        countTo64 r(*this);
        r.value = ((r.value + val) < r.maxVal) ? (r.value + val)
            : val - (r.maxVal - r.value);
        return r;
    }
    countTo64 operator+(long val) const {
        countTo64 r(*this);
        r.value = ((r.value + val) < r.maxVal) ? (r.value + val)
            : val - (r.maxVal - r.value);
        return r;
    }
    countTo64 operator+(uint8_t val) const {
        countTo64 r(*this);
        r.value = ((r.value + val) < r.maxVal) ? (r.value + val)
            : val - (r.maxVal - r.value);
        return r;
    }
    countTo64 operator++(int val) {
        value = value == (maxVal - 1) ? 0 : value + 1;
        return *this;
    }
    countTo64 operator--(int val) {
        value = value == 0 ? (maxVal - 1) : value - 1;
        return *this;
    }
    bool operator==(const countTo64& rhs) { return value == rhs.value; }
    bool operator==(const int& rhs) { return value == rhs; }
    bool operator==(const unsigned int& rhs) { return value == rhs; }
    bool operator>=(const countTo64& rhs) { return value >= rhs.value; }
    bool operator<=(const countTo64& rhs) { return value <= rhs.value; }
    bool operator>(const countTo64& rhs) { return value > rhs.value; }
    bool operator<(const countTo64& rhs) { return value < rhs.value; }
    int toInt() { return (int)value; }
};

class i2c_slave_handler {
public:
    i2c_slave_handler() { }
    uint8_t data_in[32];
    uint8_t data_out[32];
    uint8_t numBytesToWrite = 0;
    bool readyToWrite = false;
    bool dataInLock = false;
    ///\brief Blocks until numBytes number of bytes has been read from the i2c
    /// bus. Data the is read in form the bus
    /// is placed into data_in array. Take care to use lock if data is to be
    /// accessed by other core.
    void readDataIn(uint8_t numBytes, bool flush = false) {
        if(numBytes==0){ return; } // Reading 0 bytes in does nothing.
        while (i2c_get_read_available(i2c0) < numBytes) { } // expect data to show up in i2c HW buffer
        while (dataInLock) { } // wait for lock to be realesed. Should only be set by other core.
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

private:
    uint8_t bufferFlush[1];
    // bool newData = false;
};

struct TEENSY_I2C {
    enum T2P_COMMMANDS {
        OutputStateUpdate,              // single byte with bit encoded states.
        SetOutputMode,                  // 2 bytes, each nibble represents a port
        SetInputMode,                   // 2 bytes, each nibble reps a port
        SetInputUpdateRate,             // 2 byte
        RequestInputUpdate = 0b10000000 // no data bytes sent after command
    };

    uint8_t lastInCommand = 0xff;
    uint8_t numOf_T2P_commands = 4; // must reflect the data set in T2P_messageSize.
    uint8_t T2P_messageSize[4] = { 1, 2, 2, 2 }; // number of data bytes after the command
    enum P2T_REPONSE {
        Acknowledge, InputUpdate = 0b10000000,
    };

    i2c_slave_handler* handler;

    TEENSY_I2C(i2c_slave_handler& handler) { this->handler = &handler; }

    uint8_t getReadLength() {
      // command<TeensyCommands.numOf_T2P_commands?TeensyCommands.messageSize[command]:0;
        this->lastInCommand = handler->data_in[0];
        if (this->lastInCommand < numOf_T2P_commands) {
            return T2P_messageSize[this->lastInCommand];
        } else {
            return 0;
        }
    }
};

enum input_port_modes {
    SingleButton = 0x00,
    DualButton = 0x01,
    ExpPedalMinMax = 0x03,
    ExpPedalContinuous = 0x04,
    MultiButton = 0x05,
    AutoMode = 0x0f, // will try to automatically determine if device connected is exp pedal, single or dual button.
    Disabled = 0xff
};

struct InputPortState {
    input_port_modes mode[4] = {
        input_port_modes::Disabled,
        input_port_modes::Disabled,
        input_port_modes::Disabled,
        input_port_modes::Disabled,
    };
    uint16_t expValue[4] = { 0, 0, 0, 0 };
    bool tipOn[4];
    bool ringOn[4];
};

enum m_core_fifo_D_types {
    In_TR_state = 0x01, // 0xDDNNNNTR - DD = this d_type value, NNNN = ntohing, dont care, T = Tip states, R = ring states
    expVal = 0x02, // 0xDDBBVVVV - DD = this d_type value, BB = input port number, and VVVV = 16 value
    setInPortMode = 0x03, // 0xDDNN1234 - DD=this dtype, NN=dont care, 1234=nibble for each port
    setOutPortMode = 0x04, // 0xDDNN1234 - DD=this dtype, NN=dont care, 1234=nibble for each port
    setUpdateRate = 0x05, // 0xDDNNXXXX - DD=this data type, NN=dont care, XXXX=16 bit value
    Out_TR_state = 0x06, // 0xDDNNNNTR - DD = this d_type value, NNNN = ntohing, dont care, T = Tip states, R = ring states
    EdgeEvent = 0x07, // 0xDDBBSSTR - DD = this d_type, BB = input port num, SS = combined state value, TR = set one to 0b1111 to select which one had the event.
};

struct OutputPortManager {
    enum out_port_modes {
        SingleOutput = 0x00,
        DualOutput = 0x01,
        Disable = 0xff,
        TS_output = SingleOutput,
        TRS_output = DualOutput
    } mode;
    struct OutPortState {
        enum out_port_state {
            Tip_On = 0x0f,
            Ring_On = 0xf0,
            TR_On = 0xff,
            Off = 0x00,
        } currState;
        // overload " = " sign so that currState can be set more easily.
        OutPortState& operator=(const int& rhs) {
            currState = out_port_state(rhs);
            return *this;
        }
        OutPortState& operator=(const out_port_state& rhs) {
            currState = rhs;
            return *this;
        }
    } state;
};


#define NUM_ADC_VALUES_TO_AVERAGE 25
#define ADC_LOW_THRESHERHOLD 0x0300

// @label InputPortManager struct
struct InputPortManager {
private:
    struct Input_debounce_edge_state_tracker {
        private:
        bool edgeEvent = false;

        uint64_t eventTime;
        // when reading state from external GPIO, shift current state (as 0 or 1)
        // into this variable. When it reaches a threshhold,
        // you can interpret the input as being high or low.
        // Threshhold is how many bits are set or not. Decide how many this should
        // be based on how often the value is updated.
        uint32_t trackLSB = 0;
        uint32_t trackMSB = 0;
        

        uint8_t countBits() {
            uint32_t temp = this->trackLSB;
            uint8_t count = 0;
            for (uint8_t i = 0; i < 32; i++) {
                count += (temp & 1);
                temp = (temp >> 1);
            }
            temp = this->trackMSB;
            for (uint8_t i = 0; i < 32; i++) {
                count += (temp & 1);
                temp = (temp >> 1);
            }
            return count;
        }
        uint8_t highThresh = EDGETRACK_HIGH_THRESH;
        uint8_t lowThresh = EDGETRACK_LOW_THRESH;

        void updateEdgeTracker(bool val) {
            this->trackMSB = (this->trackMSB << 1) + (this->trackLSB >> 31); // Shift trackMSB to the left and add the MSB of trackLSB to it
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

        void updateEdgeTracker(bool val, uint8_t mult) {
            if (mult > 16)
                mult = 16;
            for (uint8_t i = 0; i < mult; i++) {
                updateEdgeTracker(val);
            }
        }
        
        public:
        enum combinedState {
            fallingEdgeEvent = 0x01,
            risingEdgeEvent = 0x02,
            state_OPEN_pluggedIn = 0x10,  // reads high, something is plugged in
            state_CLOSED_unplugged = 0x20 // reads low, nothing plugged in
        };
        ///\brief Combined state value. CombSt & 0xf0 returns high / low value, combSt & 0x0f returns edge event type.
        uint8_t combSt = 0x21;       

        ///\brief Use this method to capture the occurance of an edge event. Returns true if an unprocess edge has occcurred. 
        /// Calling this methos resets the edgeevent tracker.
        bool getEdgeEvent() {
            if (this->edgeEvent) {
                this->edgeEvent = false;
                return true;
            }
            return false;
        }

        // overload "+=" to make adding entries easier.
        Input_debounce_edge_state_tracker& operator+=(bool val) {
            updateEdgeTracker(val);
            return *this;
        }

        Input_debounce_edge_state_tracker() { }
    };
    Adafruit_ADS1015* AD_conv1;
    Adafruit_ADS1015* AD_conv2;
    uint8_t loopTrack = 0;
    uint64_t timeStart = 0;
    uint64_t timeCurrent = 0;
    uint8_t detect(uint8_t port) {
      //@todo Need to implement auto sensing.
        return 0;
    }
public:
    ///\brief Keeps track of the ADC values of each input channel (2 channels x 4 ports). Each is a running average
    /// of the last "NUM_ADC_VALUES_TO_AVERAGE" values returned by the ADC (default, 25 values averaged). 
    /// To get the average, use (uint16_t)=<InputPortManager>expPedals[channel]. To add a value, use 
    /// <InputPortManager>expPedals[channel]+=(uint16_t).
    struct ExtExpPedalState {
    private:
        int32_t vals[NUM_ADC_VALUES_TO_AVERAGE];
        uint8_t vals_i = 0;
        uint8_t vals_num = NUM_ADC_VALUES_TO_AVERAGE;
        int64_t avgAdder = 0;
        int32_t avg = 0;
    public:
        // allows InputPortManager::expState += (uint16_t value)
        // This permits adding to running average easily.
        // i.e. ins.expState[0] += 0x03c8; {OR} ins.expState[0] += (uint16_t)value
        ExtExpPedalState& operator+=(const int val) {
            // if (val > 0) {
                this->vals[this->vals_i] = val;
                this->vals_i++;
                if (this->vals_i >= this->vals_num) { vals_i = 0; }
                this->avgAdder = 0;
                for (uint8_t i = 0;i < this->vals_num;i++) {
                    this->avgAdder += this->vals[i];
                }
                this->avg = ((uint32_t)this->avgAdder / (int32_t)this->vals_num);
            // }
            return *this;
        }
        // allows (uint16_t) = InputPortManager::expState
        // Get the current running average by setting a uint16_t equal to expState.
        // i.e. uint16_t currentAvg = ins.expState[0];
        operator int32_t& () {
            return this->avg;
        }
    } expPedals[8]; // we have 8 to keep track of the high side reading as well as the pedal reading.
    // keeping track of each port's mode. Defualts to disabled.
    uint8_t modes[4] = {
        input_port_modes::Disabled,
        input_port_modes::Disabled,
        input_port_modes::Disabled,
        input_port_modes::Disabled
    };
    ///\brief Update the InputPortManager. Call this often to keep up to date data (as up to date as possible at least)
    /// in the various objects within InputPortManager. 
    void update() {
        uint8_t i;
        // check sense lines to see if a port has something plugged in or not.
        // if something was plugged in now and wasnt before, see if the mode for the
        // port is auto, and if so suss out the mode.

        for (i = 0; i < 8; i++) {
            senseEdgeTack[i] += gpio_get(8 + i);
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

            // Compare ADV value to threshhold and store a new value in the edge tracker. 
            tipRingEdgeTrack[loopTrack] += (tempVal < ADC_LOW_THRESHERHOLD);
            // if (tipRingEdgeTrack[loopTrack].getEdgeEvent()) printf("\r\nedge event on tip/ring %d", loopTrack);
            // if(loopTrack==0)printf("\nADC val: %d",tempVal);

            // Store the ADC value in the averager
            this->expPedals[loopTrack] += tempVal;

            // Do the same for the opther ADC
            tempVal = AD_conv2->getLastConversionResults();
            tipRingEdgeTrack[loopTrack + 4] += (tempVal < ADC_LOW_THRESHERHOLD);
            // if (tipRingEdgeTrack[loopTrack + 4].getEdgeEvent()) printf("\r\nedge event on tip/ring %d", loopTrack + 4);
            this->expPedals[loopTrack + 4] += tempVal;

            loopTrack++;
            if (loopTrack > 3) {
                loopTrack = 0;
            }
            // Interleve reads/writes of the ADCs so that they perform their conversions (sort of) at the same time
            AD_conv1->readADC_SingleEnded_NON_BLOCKING(loopTrack);
            AD_conv2->readADC_SingleEnded_NON_BLOCKING(loopTrack);
        }
    }
    InputPortManager(Adafruit_ADS1015& conv1, Adafruit_ADS1015& conv2) {
        this->AD_conv1 = &conv1;
        this->AD_conv2 = &conv2;
    }
    Input_debounce_edge_state_tracker senseEdgeTack[8] = {
        Input_debounce_edge_state_tracker(),
        Input_debounce_edge_state_tracker(),
        Input_debounce_edge_state_tracker(),
        Input_debounce_edge_state_tracker(),
        Input_debounce_edge_state_tracker(),
        Input_debounce_edge_state_tracker(),
        Input_debounce_edge_state_tracker(),
        Input_debounce_edge_state_tracker()
    };
    Input_debounce_edge_state_tracker tipRingEdgeTrack[8] = {
        Input_debounce_edge_state_tracker(),
        Input_debounce_edge_state_tracker(),
        Input_debounce_edge_state_tracker(),
        Input_debounce_edge_state_tracker(),
        Input_debounce_edge_state_tracker(),
        Input_debounce_edge_state_tracker(),
        Input_debounce_edge_state_tracker(),
        Input_debounce_edge_state_tracker()
    };
};

/*#region: defines*/
#define MCP23017_IODIRA 0x00   //!< I/O direction register A
#define MCP23017_IPOLA 0x02    //!< Input polarity port register A
#define MCP23017_GPINTENA 0x04 //!< Interrupt-on-change pins A
#define MCP23017_DEFVALA 0x06  //!< Default value register A
#define MCP23017_INTCONA 0x08  //!< Interrupt-on-change control register A
#define MCP23017_IOCONA 0x0A   //!< I/O expander configuration register A
#define MCP23017_GPPUA 0x0C    //!< GPIO pull-up resistor register A
#define MCP23017_INTFA 0x0E    //!< Interrupt flag register A
#define MCP23017_INTCAPA 0x10  //!< Interrupt captured value for port register A
#define MCP23017_GPIOA 0x12    //!< General purpose I/O port register A
#define MCP23017_OLATA 0x14    //!< Output latch register 0 A

#define MCP23017_IODIRB 0x01   //!< I/O direction register B
#define MCP23017_IPOLB 0x03    //!< Input polarity port register B
#define MCP23017_GPINTENB 0x05 //!< Interrupt-on-change pins B
#define MCP23017_DEFVALB 0x07  //!< Default value register B
#define MCP23017_INTCONB 0x09  //!< Interrupt-on-change control register B
#define MCP23017_IOCONB 0x0B   //!< I/O expander configuration register B
#define MCP23017_GPPUB 0x0D    //!< GPIO pull-up resistor register B
#define MCP23017_INTFB 0x0F    //!< Interrupt flag register B
#define MCP23017_INTCAPB 0x11  //!< Interrupt captured value for port register B
#define MCP23017_GPIOB 0x13    //!< General purpose I/O port register B
#define MCP23017_OLATB 0x15    //!< Output latch register 0 B

#define MCP23017_INT_ERR 255 //!< Interrupt error
/*#endregion*/

class MCP23017_Port_Expander {
public:
    uint16_t error = 0;
    MCP23017_Port_Expander() { }
    void begin() {
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
        digitalWrite(1, 0);

        // outputports
        for (; i < 16; i++) {
            digitalWrite(i, 0);
        }
        
    }
    void pinMode(uint8_t p, uint8_t d) {
        updateRegisterBit(p, (d == 1), MCP23017_IODIRA, MCP23017_IODIRB);
    }
    void writeGPIOAB(uint16_t ba) {
        uint8_t data[3] = { MCP23017_GPIOA, uint8_t(ba & 0x00ff), uint8_t(ba >> 8) };
        int er = i2c_write_blocking_until(i2c1, I2CADDR_PORT_EXPANDER, data, 3, false, make_timeout_time_ms(transferTimeout_ms));
    }
    void digitalWrite(uint8_t pin, uint8_t d) {
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

private:
    uint8_t portA_state = 0;
    uint8_t portB_state = 0;
    uint32_t transferTimeout_ms = 1000;
    uint8_t bitForPin(uint8_t pin) { return pin % 8; }
    uint8_t regForPin(uint8_t pin, uint8_t portAaddr, uint8_t portBaddr) {
        return (pin < 8) ? portAaddr : portBaddr;
    }
    uint8_t readRegister(uint8_t addr) {
      // printf("readReg\r\n");
        uint8_t data[1] = { addr };
        int er = i2c_write_blocking_until(i2c1, I2CADDR_PORT_EXPANDER, data, 1, false, make_timeout_time_ms(transferTimeout_ms));
        i2c_read_blocking_until(i2c1, I2CADDR_PORT_EXPANDER, data, 1, false, make_timeout_time_ms(transferTimeout_ms));
        return data[0];
    }
    void writeRegister(uint8_t regAddr, uint8_t regValue) {
      // printf("writeReg\r\n");
      // Write the register
        uint8_t data[2] = { regAddr, regValue };
        int er = i2c_write_blocking_until(i2c1, I2CADDR_PORT_EXPANDER, data, 2, false, make_timeout_time_ms(transferTimeout_ms));
    }
    void updateRegisterBit(uint8_t pin, uint8_t pValue, uint8_t portAaddr, uint8_t portBaddr) {
        uint8_t regValue;
        uint8_t regAddr = regForPin(pin, portAaddr, portBaddr);
        uint8_t bit = bitForPin(pin);
        regValue = readRegister(regAddr);
        // set the value for the particular bit
        bitWrite(regValue, bit, pValue);
        writeRegister(regAddr, regValue);
    }
};

/*#region defines and enums */
#define INA219_READ (0x01)
#define INA219_REG_CONFIG (0x00)
#define INA219_CONFIG_RESET (0x8000)              // Reset Bit
#define INA219_CONFIG_BVOLTAGERANGE_MASK (0x2000) // Bus Voltage Range Mask
#define INA219_CONFIG_GAIN_MASK (0x1800)          // Gain Mask
#define INA219_CONFIG_BADCRES_MASK (0x0780)
#define INA219_CONFIG_SADCRES_MASK                                             \
  (0x0078) // Shunt ADC Resolution and Averaging Mask
#define INA219_CONFIG_MODE_MASK (0x0007) // Operating Mode Mask
#define INA219_REG_SHUNTVOLTAGE (0x01)
#define INA219_REG_BUSVOLTAGE (0x02)
#define INA219_REG_POWER (0x03)
#define INA219_REG_CURRENT (0x04)
#define INA219_REG_CALIBRATION (0x05)
enum {
    INA219_CONFIG_BVOLTAGERANGE_16V = (0x0000), // 0-16V Range
    INA219_CONFIG_BVOLTAGERANGE_32V = (0x2000), // 0-32V Range
};
enum {
    INA219_CONFIG_GAIN_1_40MV = (0x0000),  // Gain 1, 40mV Range
    INA219_CONFIG_GAIN_2_80MV = (0x0800),  // Gain 2, 80mV Range
    INA219_CONFIG_GAIN_4_160MV = (0x1000), // Gain 4, 160mV Range
    INA219_CONFIG_GAIN_8_320MV = (0x1800), // Gain 8, 320mV Range
};
enum {
    INA219_CONFIG_BADCRES_9BIT = (0x0000),  // 9-bit bus res = 0..511
    INA219_CONFIG_BADCRES_10BIT = (0x0080), // 10-bit bus res = 0..1023
    INA219_CONFIG_BADCRES_11BIT = (0x0100), // 11-bit bus res = 0..2047
    INA219_CONFIG_BADCRES_12BIT = (0x0180), // 12-bit bus res = 0..4097
    INA219_CONFIG_BADCRES_12BIT_2S_1060US = (0x0480), // 2 x 12-bit bus samples averaged together
    INA219_CONFIG_BADCRES_12BIT_4S_2130US = (0x0500), // 4 x 12-bit bus samples averaged together
    INA219_CONFIG_BADCRES_12BIT_8S_4260US = (0x0580), // 8 x 12-bit bus samples averaged together
    INA219_CONFIG_BADCRES_12BIT_16S_8510US = (0x0600), // 16 x 12-bit bus samples averaged together
    INA219_CONFIG_BADCRES_12BIT_32S_17MS = (0x0680), // 32 x 12-bit bus samples averaged together
    INA219_CONFIG_BADCRES_12BIT_64S_34MS = (0x0700), // 64 x 12-bit bus samples averaged together
    INA219_CONFIG_BADCRES_12BIT_128S_69MS = (0x0780), // 128 x 12-bit bus samples averaged together
};
enum {
    INA219_CONFIG_SADCRES_9BIT_1S_84US = (0x0000),   // 1 x 9-bit shunt sample
    INA219_CONFIG_SADCRES_10BIT_1S_148US = (0x0008), // 1 x 10-bit shunt sample
    INA219_CONFIG_SADCRES_11BIT_1S_276US = (0x0010), // 1 x 11-bit shunt sample
    INA219_CONFIG_SADCRES_12BIT_1S_532US = (0x0018), // 1 x 12-bit shunt sample
    INA219_CONFIG_SADCRES_12BIT_2S_1060US = (0x0048), // 2 x 12-bit shunt samples averaged together
    INA219_CONFIG_SADCRES_12BIT_4S_2130US = (0x0050), // 4 x 12-bit shunt samples averaged together
    INA219_CONFIG_SADCRES_12BIT_8S_4260US = (0x0058), // 8 x 12-bit shunt samples averaged together
    INA219_CONFIG_SADCRES_12BIT_16S_8510US = (0x0060), // 16 x 12-bit shunt samples averaged together
    INA219_CONFIG_SADCRES_12BIT_32S_17MS = (0x0068), // 32 x 12-bit shunt samples averaged together
    INA219_CONFIG_SADCRES_12BIT_64S_34MS = (0x0070), // 64 x 12-bit shunt samples averaged together
    INA219_CONFIG_SADCRES_12BIT_128S_69MS = (0x0078), // 128 x 12-bit shunt samples averaged together
};
enum {
    INA219_CONFIG_MODE_POWERDOWN = 0x00,            /**< power down */
    INA219_CONFIG_MODE_SVOLT_TRIGGERED = 0x01,      /**< shunt voltage triggered */
    INA219_CONFIG_MODE_BVOLT_TRIGGERED = 0x02,      /**< bus voltage triggered */
    INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED = 0x03,  /**< shunt and bus voltage triggered */
    INA219_CONFIG_MODE_ADCOFF = 0x04,               /**< ADC off */
    INA219_CONFIG_MODE_SVOLT_CONTINUOUS = 0x05,     /**< shunt voltage continuous */
    INA219_CONFIG_MODE_BVOLT_CONTINUOUS = 0x06,     /**< bus voltage continuous */
    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS = 0x07, /**< shunt and bus voltage continuous */
};
/*#endregion*/

class Adafruit_INA219 {
public:
    Adafruit_INA219(uint8_t addr) {
        ina219_i2caddr = addr;
        ina219_currentDivider_mA = 0;
        ina219_powerMultiplier_mW = 0.0f;
    }
    void setCalibration_16V_400mA() {

      // Calibration which uses the highest precision for
      // current measurement (0.1mA), at the expense of
      // only supporting 16V at 400mA max.

      // VBUS_MAX = 16V
      // VSHUNT_MAX = 0.4          (Assumes Gain 1, 40mV)
      // RSHUNT = 1.0               (Resistor value in ohms) *****1.0

      // 1. Determine max possible current
      // MaxPossible_I = VSHUNT_MAX / RSHUNT
      // MaxPossible_I = 0.4A
      // ***MaxPossible_I = 0.4A

      // 2. Determine max expected current
      // MaxExpected_I = 0.4A

      // 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
      // MinimumLSB = MaxExpected_I/32767
      // MinimumLSB = 0.0000122              (12uA per bit)
      // MaximumLSB = MaxExpected_I/4096
      // MaximumLSB = 0.0000977              (98uA per bit)

      // 4. Choose an LSB between the min and max values
      //    (Preferrably a roundish number close to MinLSB)
      // CurrentLSB = 0.00005 (50uA per bit)

      // 5. Compute the calibration register
      // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
      // Cal = 8192 (0x2000)

        ina219_calValue = 8192;

        // 6. Calculate the power LSB
        // PowerLSB = 20 * CurrentLSB
        // PowerLSB = 0.001 (1mW per bit)

        // 7. Compute the maximum current and shunt voltage values before overflow
        //
        // Max_Current = Current_LSB * 32767
        // Max_Current = 1.63835A before overflow
        //
        // If Max_Current > Max_Possible_I then
        //    Max_Current_Before_Overflow = MaxPossible_I
        // Else
        //    Max_Current_Before_Overflow = Max_Current
        // End If
        //
        // Max_Current_Before_Overflow = MaxPossible_I
        // Max_Current_Before_Overflow = 0.4
        //
        // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
        // Max_ShuntVoltage = 0.04V
        //
        // If Max_ShuntVoltage >= VSHUNT_MAX
        //    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
        // Else
        //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
        // End If
        //
        // Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
        // Max_ShuntVoltage_Before_Overflow = 0.04V

        // 8. Compute the Maximum Power
        // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
        // MaximumPower = 0.4 * 16V
        // MaximumPower = 6.4W

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
    float getBusVoltage_V() {
        int16_t value = getBusVoltage_raw();
        return value * 0.001;
    }
    float getShuntVoltage_mV() {
        int16_t value;
        value = getShuntVoltage_raw();
        return value * 0.01;
    }
    float getCurrent_mA() {
        float valueDec = getCurrent_raw();
        valueDec /= ina219_currentDivider_mA;
        return valueDec;
    }
    float getPower_mW() {
        float valueDec = getPower_raw();
        valueDec *= ina219_powerMultiplier_mW;
        return valueDec;
    }
    bool success() { return _success; }

private:
    bool _success;
    uint8_t ina219_i2caddr = -1;
    uint32_t ina219_calValue;
    // The following multipliers are used to convert raw current and power
    // values to mA and mW, taking into account the current config settings
    uint32_t ina219_currentDivider_mA;
    float ina219_powerMultiplier_mW;

    void init() {
      // Set chip to large range config values to start
        setCalibration_16V_400mA();
    }
    int16_t getBusVoltage_raw() {
        uint16_t value;
        uint8_t data[2];
        data[0] = INA219_REG_BUSVOLTAGE;
        i2c_write_blocking_until(i2c1, this->ina219_i2caddr, data, 1, true, make_timeout_time_ms(5));
        _success = 2 == i2c_read_blocking_until(i2c1, this->ina219_i2caddr, data, 2, false, make_timeout_time_ms(5));
        value = data[0] | ((uint16_t(data[1])) << 8);
        // Shift to the right 3 to drop CNVR and OVF and multiply by LSB
        return (int16_t)((value >> 3) * 4);
    }
    int16_t getShuntVoltage_raw() {
        uint16_t value;
        uint8_t data[2];
        data[0] = INA219_REG_SHUNTVOLTAGE;
        i2c_write_blocking_until(i2c1, this->ina219_i2caddr, data, 1, true, make_timeout_time_ms(5));
        _success = 2 == i2c_read_blocking_until(i2c1, this->ina219_i2caddr, data, 2, false, make_timeout_time_ms(5));
        value = data[0] | ((uint16_t(data[1])) << 8);
        return value;
    }
    int16_t getCurrent_raw() {
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
    int16_t getPower_raw() {
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
};