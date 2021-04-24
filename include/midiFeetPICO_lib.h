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

#include "pico/stdlib.h"
#include <stdio.h>
#include <stdint.h>

/*#region #defines and ENUMS*/
#ifndef MIDFT_PICO_HEAD
#define MIDFT_PICO_HEAD
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
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet((value), (bit)) : bitClear((value), (bit)))
#define ADS1X15_REG_POINTER_MASK (0x03)      ///< Point mask
#define ADS1X15_REG_POINTER_CONVERT (0x00)   ///< Conversion
#define ADS1X15_REG_POINTER_CONFIG (0x01)    ///< Configuration
#define ADS1X15_REG_POINTER_LOWTHRESH (0x02) ///< Low threshold
#define ADS1X15_REG_POINTER_HITHRESH (0x03)  ///< High threshold
#define ADS1X15_REG_CONFIG_OS_MASK (0x8000) ///< OS Mask
#define ADS1X15_REG_CONFIG_OS_SINGLE (0x8000) ///< Write: Set to start a single-conversion
#define ADS1X15_REG_CONFIG_OS_BUSY (0x0000) ///< Read: Bit = 0 when conversion is in progress
#define ADS1X15_REG_CONFIG_OS_NOTBUSY (0x8000) ///< Read: Bit = 1 when device is not performing a conversion
#define ADS1X15_REG_CONFIG_MUX_MASK (0x7000) ///< Mux Mask
#define ADS1X15_REG_CONFIG_MUX_DIFF_0_1 (0x0000) ///< Differential P = AIN0, N = AIN1 (default)
#define ADS1X15_REG_CONFIG_MUX_DIFF_0_3 (0x1000) ///< Differential P = AIN0, N = AIN3
#define ADS1X15_REG_CONFIG_MUX_DIFF_1_3 (0x2000) ///< Differential P = AIN1, N = AIN3
#define ADS1X15_REG_CONFIG_MUX_DIFF_2_3 (0x3000) ///< Differential P = AIN2, N = AIN3
#define ADS1X15_REG_CONFIG_MUX_SINGLE_0 (0x4000) ///< Single-ended AIN0
#define ADS1X15_REG_CONFIG_MUX_SINGLE_1 (0x5000) ///< Single-ended AIN1
#define ADS1X15_REG_CONFIG_MUX_SINGLE_2 (0x6000) ///< Single-ended AIN2
#define ADS1X15_REG_CONFIG_MUX_SINGLE_3 (0x7000) ///< Single-ended AIN3
#define ADS1X15_REG_CONFIG_PGA_MASK (0x0E00)     ///< PGA Mask
#define ADS1X15_REG_CONFIG_PGA_6_144V (0x0000)   ///< +/-6.144V range = Gain 2/3
#define ADS1X15_REG_CONFIG_PGA_4_096V (0x0200)   ///< +/-4.096V range = Gain 1
#define ADS1X15_REG_CONFIG_PGA_2_048V (0x0400) ///< +/-2.048V range = Gain 2 (default)
#define ADS1X15_REG_CONFIG_PGA_1_024V (0x0600)  ///< +/-1.024V range = Gain 4
#define ADS1X15_REG_CONFIG_PGA_0_512V (0x0800)  ///< +/-0.512V range = Gain 8
#define ADS1X15_REG_CONFIG_PGA_0_256V (0x0A00)  ///< +/-0.256V range = Gain 16
#define ADS1X15_REG_CONFIG_MODE_MASK (0x0100)   ///< Mode Mask
#define ADS1X15_REG_CONFIG_MODE_CONTIN (0x0000) ///< Continuous conversion mode
#define ADS1X15_REG_CONFIG_MODE_SINGLE (0x0100) ///< Power-down single-shot mode (default)
#define ADS1X15_REG_CONFIG_RATE_MASK (0x00E0)  ///< Data Rate Mask
#define ADS1X15_REG_CONFIG_CMODE_MASK (0x0010) ///< CMode Mask
#define ADS1X15_REG_CONFIG_CMODE_TRAD (0x0000) ///< Traditional comparator with hysteresis (default)
#define ADS1X15_REG_CONFIG_CMODE_WINDOW (0x0010) ///< Window comparator
#define ADS1X15_REG_CONFIG_CPOL_MASK (0x0008)    ///< CPol Mask
#define ADS1X15_REG_CONFIG_CPOL_ACTVLOW (0x0000) ///< ALERT/RDY pin is low when active (default)
#define ADS1X15_REG_CONFIG_CPOL_ACTVHI (0x0008) ///< ALERT/RDY pin is high when active
#define ADS1X15_REG_CONFIG_CLAT_MASK (0x0004) ///< Determines if ALERT/RDY pin latches once asserted
#define ADS1X15_REG_CONFIG_CLAT_NONLAT (0x0000) ///< Non-latching comparator (default)
#define ADS1X15_REG_CONFIG_CLAT_LATCH (0x0004) ///< Latching comparator
#define ADS1X15_REG_CONFIG_CQUE_MASK (0x0003)  ///< CQue Mask
#define ADS1X15_REG_CONFIG_CQUE_1CONV (0x0000) ///< Assert ALERT/RDY after one conversions
#define ADS1X15_REG_CONFIG_CQUE_2CONV (0x0001) ///< Assert ALERT/RDY after two conversions
#define ADS1X15_REG_CONFIG_CQUE_4CONV (0x0002) ///< Assert ALERT/RDY after four conversions
#define ADS1X15_REG_CONFIG_CQUE_NONE (0x0003) ///< Disable the comparator and put ALERT/RDY in high state (default)
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
#define NUM_ADC_VALUES_TO_AVERAGE 25
#define ADC_LOW_THRESHERHOLD 0x0300
#define INA219_READ (0x01)
#define INA219_REG_CONFIG (0x00)
#define INA219_CONFIG_RESET (0x8000)              // Reset Bit
#define INA219_CONFIG_BVOLTAGERANGE_MASK (0x2000) // Bus Voltage Range Mask
#define INA219_CONFIG_GAIN_MASK (0x1800)          // Gain Mask
#define INA219_CONFIG_BADCRES_MASK (0x0780)
#define INA219_CONFIG_SADCRES_MASK (0x0078) // Shunt ADC Resolution and Averaging Mask
#define INA219_CONFIG_MODE_MASK (0x0007) // Operating Mode Mask
#define INA219_REG_SHUNTVOLTAGE (0x01)
#define INA219_REG_BUSVOLTAGE (0x02)
#define INA219_REG_POWER (0x03)
#define INA219_REG_CURRENT (0x04)
#define INA219_REG_CALIBRATION (0x05)
#define INA219_READ (0x01)
#define INA219_REG_CONFIG (0x00)
#define INA219_CONFIG_RESET (0x8000)              // Reset Bit
#define INA219_CONFIG_BVOLTAGERANGE_MASK (0x2000) // Bus Voltage Range Mask
#define INA219_CONFIG_GAIN_MASK (0x1800)          // Gain Mask
#define INA219_CONFIG_BADCRES_MASK (0x0780)
#define INA219_CONFIG_SADCRES_MASK (0x0078) // Shunt ADC Resolution and Averaging Mask
#define INA219_CONFIG_MODE_MASK (0x0007) // Operating Mode Mask
#define INA219_REG_SHUNTVOLTAGE (0x01)
#define INA219_REG_BUSVOLTAGE (0x02)
#define INA219_REG_POWER (0x03)
#define INA219_REG_CURRENT (0x04)
#define INA219_REG_CALIBRATION (0x05)
typedef enum {
    GAIN_TWOTHIRDS = ADS1X15_REG_CONFIG_PGA_6_144V,
    GAIN_ONE = ADS1X15_REG_CONFIG_PGA_4_096V,
    GAIN_TWO = ADS1X15_REG_CONFIG_PGA_2_048V,
    GAIN_FOUR = ADS1X15_REG_CONFIG_PGA_1_024V,
    GAIN_EIGHT = ADS1X15_REG_CONFIG_PGA_0_512V,
    GAIN_SIXTEEN = ADS1X15_REG_CONFIG_PGA_0_256V
} adsGain_t;
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
enum combinedState {
    fallingEdgeEvent = 0x01,
    risingEdgeEvent = 0x02,
    state_OPEN_pluggedIn = 0x10,  // reads high, something is plugged in
    state_CLOSED_unplugged = 0x20, // reads low, nothing plugged in
    state_OPEN_buttonPressed = state_OPEN_pluggedIn,
    state_CLOSED_buttonNotPressed = state_CLOSED_unplugged,
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
enum input_port_modes {
    SingleButton = 0x00,
    DualButton = 0x01,
    ExpPedalMinMax = 0x03,
    ExpPedalContinuous = 0x04,
    MultiButton = 0x05,
    AutoMode = 0x0f, // will try to automatically determine if device connected is exp pedal, single or dual button.
    Disabled = 0xff
};
/*#endregion*/


struct InputPortManager;
class MCP23017_Port_Expander;
class Adafruit_ADS1015;
class countTo64;
class i2c_slave_handler;
struct TEENSY_I2C;
struct InputPortState;
struct OutputPortManager;
class Adafruit_INA219;

struct InputPortManager {
private:
    struct Input_debounce_edge_state_tracker {
    private:
        bool edgeEvent = false;
        uint64_t eventTime;
        uint32_t trackLSB = 0;
        uint32_t trackMSB = 0;
        uint8_t countBits();
        uint8_t highThresh = EDGETRACK_HIGH_THRESH;
        uint8_t lowThresh = EDGETRACK_LOW_THRESH;
        void updateEdgeTracker(bool val);
        void updateEdgeTracker(bool val, uint8_t mult);
    public:
        uint8_t combSt = combinedState::state_CLOSED_unplugged | combinedState::fallingEdgeEvent;
        bool getEdgeEvent();
        Input_debounce_edge_state_tracker& operator+=(bool val) {
            updateEdgeTracker(val);
            return *this;
        }
        Input_debounce_edge_state_tracker();
    };
    Adafruit_ADS1015* AD_conv1;
    Adafruit_ADS1015* AD_conv2;
    uint8_t loopTrack = 0;
    uint64_t timeStart = 0;
    uint64_t timeCurrent = 0;
    uint8_t detect(uint8_t port);
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
    uint8_t modes[4] = {
        input_port_modes::DualButton,
        input_port_modes::DualButton,
        input_port_modes::DualButton,
        input_port_modes::DualButton
    };
    uint8_t update();
    InputPortManager(Adafruit_ADS1015& conv1, Adafruit_ADS1015& conv2);
    bool getInputState(uint8_t trackerNum);
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

class MCP23017_Port_Expander {
public:
    uint16_t error = 0;
    MCP23017_Port_Expander();
    void begin();
    void pinMode(uint8_t p, uint8_t d);
    void writeGPIOAB(uint16_t ba);
    void digitalWrite(uint8_t pin, uint8_t d);

private:
    uint8_t portA_state = 0;
    uint8_t portB_state = 0;
    uint32_t transferTimeout_ms = 1000;
    uint8_t bitForPin(uint8_t pin);
    uint8_t regForPin(uint8_t pin, uint8_t portAaddr, uint8_t portBaddr);
    uint8_t readRegister(uint8_t addr);
    void writeRegister(uint8_t regAddr, uint8_t regValue);
    void updateRegisterBit(uint8_t pin, uint8_t pValue, uint8_t portAaddr, uint8_t portBaddr);
};

class Adafruit_ADS1015 {
protected:
    uint8_t m_bitShift;  ///< bit shift amount
    adsGain_t m_gain;    ///< ADC gain
    uint16_t m_dataRate; ///< Data rate

public:
    int16_t readADC_SingleEnded_BLOCKING(uint8_t channel);
    void readADC_SingleEnded_NON_BLOCKING(uint8_t channel);
    int16_t getLastConversionResults();
    void setGain(adsGain_t gain);
    adsGain_t getGain();
    void setDataRate(uint16_t rate);
    uint16_t getDataRate();
    Adafruit_ADS1015(uint8_t i2c_addr);
    bool conversionComplete();
    bool readADC_SingleEnded_nonBlocking_Async(uint8_t channel);
    bool readingInProgress = false;
    int readInProgressChannel = -1;
    uint8_t mostRecentConversionChannel = 0;

private:
    void writeRegister(uint8_t reg, uint16_t value);
    uint16_t readRegister(uint8_t reg);
    uint8_t buffer[3];
    uint8_t address = 0;
};


///\brief A type that can hold 64 unique values. Will rollover from 63 to 0. 
///\param val Init value. Must be less than 64 (or "max" if set)
///\param max Used to set the rollover value to an arbitrary value. 1 to 256.
class countTo64 {
    private:
    uint8_t value;
    uint8_t maxVal;
    public:
    countTo64(void)                                     { value = 0; }
    countTo64(int val)                                  { maxVal = 64; value = val<maxVal?val:0; }
    countTo64(int val, int max)                         { maxVal = (max<257)?max:256; value = val<maxVal?val:0; }
    operator int () const                               { return value; }
    countTo64 & operator = (const countTo64 &rhs)       { value = rhs.value; return *this; }
    countTo64 & operator = (const int &rhs)             { value = rhs; return *this; }
    countTo64 & operator -= (uint8_t val)               { value = (value>=val)?(value-val):maxVal-(val-value) ; return *this; }
	countTo64 & operator += (uint8_t val)               { value = ((value+val)<maxVal)?(value+val):val - (maxVal-value) ; return *this; }
	countTo64 operator - (int val) const                { countTo64 r(*this); r.value = (r.value>=val)?(r.value-val):r.maxVal-(val-r.value); return r; }
	countTo64 operator - (unsigned int val) const       { countTo64 r(*this); r.value = (r.value>=val)?(r.value-val):r.maxVal-(val-r.value); return r; }
	countTo64 operator - (long val) const               { countTo64 r(*this); r.value = (r.value>=val)?(r.value-val):r.maxVal-(val-r.value); return r; }
	countTo64 operator - (uint8_t val) const            { countTo64 r(*this); r.value = (r.value>=val)?(r.value-val):r.maxVal-(val-r.value); return r; }
	countTo64 operator + (int val) const                { countTo64 r(*this); r.value = ((r.value+val)<r.maxVal)?(r.value+val):val - (r.maxVal-r.value) ; return r; }
	countTo64 operator + (unsigned int val) const       { countTo64 r(*this); r.value = ((r.value+val)<r.maxVal)?(r.value+val):val - (r.maxVal-r.value) ; return r; }
	countTo64 operator + (long val) const               { countTo64 r(*this); r.value = ((r.value+val)<r.maxVal)?(r.value+val):val - (r.maxVal-r.value) ; return r; }
	countTo64 operator + (uint8_t val) const            { countTo64 r(*this); r.value = ((r.value+val)<r.maxVal)?(r.value+val):val - (r.maxVal-r.value) ; return r; }
    countTo64 operator++ (int val)                      { value = value==(maxVal-1)?0:value+1; return *this; }
    countTo64 operator-- (int val)                      { value = value==0?(maxVal-1):value-1; return *this; }
    bool operator==(const countTo64 &rhs)               { return value == rhs.value; }
    bool operator==(const int& rhs)                     { return value == rhs; }
    bool operator==(const unsigned int& rhs)            { return value == rhs; }
    bool operator>=(const countTo64 &rhs)               { return value >= rhs.value; }
    bool operator<=(const countTo64 &rhs)               { return value <= rhs.value; }
    bool operator>(const countTo64 &rhs)                { return value > rhs.value; }
    bool operator<(const countTo64 &rhs)                { return value < rhs.value; }
};

class i2c_slave_handler {
public:
    i2c_slave_handler();
    uint8_t data_in[32];
    uint8_t data_out[32];
    uint8_t numBytesToWrite = 0;
    bool readyToWrite = false;
    bool dataInLock = false;
    enum Edge_Event_FLAG : uint8_t {
        edge_detected = 0b00010000,
        edge_not_detected = 0
    }edgeEvent_F = edge_not_detected;
    void readDataIn(uint8_t numBytes, bool flush = false);

private:
    uint8_t bufferFlush[1];
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
    TEENSY_I2C(i2c_slave_handler& handler);
    uint8_t getReadLength();
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


struct OutputPortManager {
private:
    uint8_t id;
    MCP23017_Port_Expander* expander;
public:
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
        };
        bool Tip_on_b = false;
        bool Ring_on_b = false;
        uint8_t  currState = out_port_state::Off;
        OutPortState& operator=(const int& rhs);
        OutPortState& operator=(const out_port_state& rhs);
    } state;

    OutputPortManager(MCP23017_Port_Expander& exp, uint8_t _id);
    void UpdateOutput();
};




class Adafruit_INA219 {
public:
    Adafruit_INA219(uint8_t addr);
    void setCalibration_16V_400mA();
    float getBusVoltage_V();
    float getShuntVoltage_mV();
    float getCurrent_mA();
    float getPower_mW();
    bool success();

private:
    bool _success;
    uint8_t ina219_i2caddr = -1;
    uint32_t ina219_calValue;
    uint32_t ina219_currentDivider_mA;
    float ina219_powerMultiplier_mW;
    void init();
    int16_t getBusVoltage_raw();
    int16_t getShuntVoltage_raw();
    int16_t getCurrent_raw();
    int16_t getPower_raw();
};

#endif