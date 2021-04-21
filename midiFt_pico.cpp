#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"
#include "midiFeetPICO.h"
#define I2C0_PORT i2c0
#define I2C1_PORT i2c1

i2c_slave_handler slaveHandler = i2c_slave_handler(); // global to pass data into handler function.

// Gets called by timer to poll the i2c hardware for write requests from the Teensy
bool check_i2c_slave_request(repeating_timer_t* t) {
    // check HW register to see if the master has addressed this device.
    if (I2C0_PORT->hw->clr_rd_req && slaveHandler.readyToWrite) {
        // HW will hold the bus until the register is reset.
        hw_clear_bits(I2C_IC_RAW_INTR_STAT_RD_REQ_RESET, I2C0_PORT->hw->clr_rd_req);

        if (slaveHandler.numBytesToWrite > 0) {
            i2c_write_raw_blocking(I2C0_PORT, slaveHandler.data_out, slaveHandler.numBytesToWrite);
        } else {
            i2c_write_raw_blocking(I2C0_PORT, 0, 1);
        }
        slaveHandler.readyToWrite = false; // reset flag
    }
    return true;
}

void core1_entry_i2cSlave() {
    puts("Core #2 started. Setting up timer.");
    // setup timer to call handler repeatedly. Will wait 100us from completion of last call to next call.
    repeating_timer_t timer;
    add_repeating_timer_us(100, check_i2c_slave_request, NULL, &timer);

    TEENSY_I2C TeensyCommands = TEENSY_I2C(slaveHandler);
    InputPortState inState;
    puts("Entering loop...");
    while (1) {
        if (i2c_get_read_available(I2C0_PORT) > 0) {
            // puts("got data on i2c");
            slaveHandler.readDataIn(1); // read one byte from the bus. This first byte sent is the command.
            // The number of bytes to read is inferred from the command. Once these bytes have been read, flush the HW buffer.
            slaveHandler.readDataIn(TeensyCommands.getReadLength(), true);
            // puts("doing something with data");
            // do something with the data
            // @label  Teensy2Pico commands switch/case
            switch (TeensyCommands.lastInCommand) {
                case TEENSY_I2C::T2P_COMMMANDS::RequestInputUpdate... 255:
                    {
                        uint8_t portsRequested = TeensyCommands.lastInCommand & 0x0f; // bits 0-3 indicate which ports we want to get info for.
                        slaveHandler.data_out[0] = TEENSY_I2C::P2T_REPONSE::InputUpdate | portsRequested;
                        uint8_t numBytes = 1;
                        for (uint8_t i = 0; i < 4; i++) {
                            // if this iteration corresponds to a requested port...
                            if ((portsRequested >> i) & 1) {
                                // add data to data_out based on port mode
                                switch (inState.mode[i]) {
                                    case input_port_modes::ExpPedalMinMax... input_port_modes::ExpPedalContinuous:
                                        {
                                            slaveHandler.data_out[numBytes] = inState.expValue[i] & 0xff;
                                            slaveHandler.data_out[numBytes + 1] = (inState.expValue[i] >> 8) & 0xff;
                                            numBytes += 2;
                                            break;
                                        }
                                    case input_port_modes::SingleButton... input_port_modes::DualButton:
                                        {
                                            
                                            uint8_t u = 0;
                                            for (; u < 4; u++) {
                                                slaveHandler.data_out[numBytes] |= ((inState.tipOn[u] ? 1 : 0) << u);
                                            }
                                            for (; u < 8; u++) {
                                                slaveHandler.data_out[numBytes] |= ((inState.ringOn[u] ? 1 : 0) << u);
                                            }
                                            slaveHandler.data_out[numBytes+1]=slaveHandler.data_out[numBytes];
                                            numBytes+=2;
                                            break;
                                        }
                                    case input_port_modes::MultiButton:
                                    {
                                        //@todo Not sure if this will be implememtned.
                                        break;
                                    }
                                    default:
                                        break;
                                }
                            }
                        }
                        // for(uint8_t i=numBytes;i<32;i++){
                        //     slaveHandler.data_out[i]=0;
                        // }
                        slaveHandler.numBytesToWrite = numBytes;
                        slaveHandler.readyToWrite = true;
                        break;
                    }
                case TEENSY_I2C::T2P_COMMMANDS::OutputStateUpdate:
                    {
                        // if the fifo is full, we need to wait until there's room for the data or it could be lost.
                        // puts("got output update. flushing fifo.");
                        while (!multicore_fifo_wready()) {}
                        // puts("sending to fifo.");
                        multicore_fifo_push_blocking(slaveHandler.data_in[0] | (m_core_fifo_D_types::Out_TR_state << 24));
                        slaveHandler.data_out[0] = TEENSY_I2C::P2T_REPONSE::Acknowledge;
                        slaveHandler.numBytesToWrite = 1;
                        slaveHandler.readyToWrite = true;
                        break;
                    }
                case TEENSY_I2C::T2P_COMMMANDS::SetInputMode:
                    {
                        uint16_t data16 = slaveHandler.data_in[0] | (slaveHandler.data_in[1] << 8);
                        // if the fifo is full, we need to wait until there's room for the data or it could be lost.
                        while (!multicore_fifo_wready()) {}
                        multicore_fifo_push_blocking(data16 | (m_core_fifo_D_types::setInPortMode << 24));
                        slaveHandler.data_out[0] = TEENSY_I2C::P2T_REPONSE::Acknowledge;
                        slaveHandler.numBytesToWrite = 1;
                        slaveHandler.readyToWrite = true;
                        break;
                    }
                case TEENSY_I2C::T2P_COMMMANDS::SetInputUpdateRate:
                    {
                        uint16_t data16 = slaveHandler.data_in[0] | (slaveHandler.data_in[1] << 8);
                        // if the fifo is full, we need to wait until there's room for the data or it could be lost.
                        while (!multicore_fifo_wready()) {}
                        multicore_fifo_push_blocking(data16 | (m_core_fifo_D_types::setUpdateRate << 24));
                        slaveHandler.data_out[0] = TEENSY_I2C::P2T_REPONSE::Acknowledge;
                        slaveHandler.numBytesToWrite = 1;
                        slaveHandler.readyToWrite = true;
                        break;
                    }
                case TEENSY_I2C::T2P_COMMMANDS::SetOutputMode:
                    {
                        uint16_t data16 = slaveHandler.data_in[0] | (slaveHandler.data_in[1] << 8);
                        // if the fifo is full, we need to wait until there's room for the data or it could be lost.
                        while (!multicore_fifo_wready()) {}
                        multicore_fifo_push_blocking(data16 | (m_core_fifo_D_types::setOutPortMode << 24));
                        slaveHandler.data_out[0] = TEENSY_I2C::P2T_REPONSE::Acknowledge;
                        slaveHandler.numBytesToWrite = 1;
                        slaveHandler.readyToWrite = true;
                        break;
                    }
                default:
                    break;
            }
        }
        // on every loop, check multicore fifo for data and update local InputPortState object
        // if fifo available
        //@label Core 1 FIFO handler
        while (multicore_fifo_rvalid()) {
            // read fifo into uint32
            uint32_t fifo_in = multicore_fifo_pop_blocking();
            uint8_t d_type = fifo_in >> 24;
            // switch/case to do something it
            switch (d_type) {
                case m_core_fifo_D_types::expVal:
                    {
                        uint8_t portNum = (fifo_in >> 16) & 0xff;
                        inState.expValue[portNum] = fifo_in & 0xffff;
                        break;
                    }
                case m_core_fifo_D_types::In_TR_state:
                    {
                        uint8_t data = fifo_in & 0xff;
                        for (uint8_t i = 0; i < 4; i++) {
                            inState.tipOn[i] = (1 == (data & 1));
                            data = data >> 1;
                        }
                        for (uint8_t i = 0; i < 4; i++) {
                            inState.ringOn[i] = (1 == (data & 1));
                            data = data >> 1;
                        }
                        break;
                    }
                default:
                    break;
            }
        } // loop through the entire available fifo. Other core may have stacked updates while this core was busy.
    }     // main loop for core 1
}

bool readADCandPrint(repeating_timer_t* t) {
    // printf("gpio8: %d\r\n\r\n",gpio_get(8));
    // printf("\r\n");

    return true;
}

int main() {
    stdio_init_all();
    i2c_init(I2C0_PORT, 100 * 1000);
    i2c_set_slave_mode(I2C0_PORT, true, 0x5a);
    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_set_function(5, GPIO_FUNC_I2C);
    gpio_pull_up(4);
    gpio_pull_up(5);
    i2c_init(I2C1_PORT, 400 * 1000);
    gpio_set_function(2, GPIO_FUNC_I2C);
    gpio_set_function(3, GPIO_FUNC_I2C);
    gpio_pull_up(2);
    gpio_pull_up(3);
    for (uint8_t i = 8; i < 16; i++) {
        gpio_init(i);
        gpio_set_dir(i, GPIO_IN);
        gpio_pull_up(i);
    }
    sleep_ms(3000);
    
    puts("starting second core\r\n");
    multicore_launch_core1(core1_entry_i2cSlave);
    // flush the fifo
    while (multicore_fifo_rvalid()) {
        multicore_fifo_pop_blocking();
        puts(".");
    }
    Adafruit_ADS1015 AD_converter1 = Adafruit_ADS1015(I2CADDR_ADC1);
    Adafruit_ADS1015 AD_converter2 = Adafruit_ADS1015(I2CADDR_ADC2);
    MCP23017_Port_Expander PortExpander = MCP23017_Port_Expander();
    Adafruit_INA219 currentSensors[4] = {
        Adafruit_INA219(I2CADDR_CURR_SENS1),
        Adafruit_INA219(I2CADDR_CURR_SENS2),
        Adafruit_INA219(I2CADDR_CURR_SENS3),
        Adafruit_INA219(I2CADDR_CURR_SENS4),
    };
    PortExpander.begin();
    OutputPortManager outs[4];
    InputPortManager ins = InputPortManager(AD_converter1, AD_converter2);
    repeating_timer_t timer;
    // add_repeating_timer_ms(1000, readADCandPrint, NULL, &timer);
    for (uint8_t i = 0; i < 4; i++) {
        currentSensors[i].setCalibration_16V_400mA();
    }
    countTo64 counting = countTo64(0, 100);
    while (1) {
        float current = currentSensors[0].getCurrent_mA();
        if (counting == 0) {
            printf("\r\ncurrent current in ma: %f",current);
            for( int i = 0; i < 8; i ++){
                printf("\ncurrent avergae for pedal%i: %i",i , ins.expPedals[i]);
            }
        }
        counting++;

        // Have the inputPortManager update all values
        ins.update(); // This updates the states for all 4 input ports

        // Do the things depending on the current mode
        for (uint8_t port_i = 0; port_i < 4; port_i++) {
            switch ( ins.modes[port_i] ) {
                case input_port_modes::Disabled:
                {
                    // Do nothing. the port is set to disabled.
                    // shoud propbably make sure everythign is set to zero or somehting
                    break;
                }
                case input_port_modes::SingleButton ... input_port_modes::DualButton:
                {
                    // Read tip state
                    // Read ring state
                    // send the update to the other core
                    break;
                }
                case input_port_modes::ExpPedalContinuous:
                {
                    // Update the other core with a ratio value everytime
                    break;
                }
                case input_port_modes::ExpPedalMinMax:
                {
                    // update the other core with 1.0 or 0.0 ratio value depending on wether the pedal has hit min/max threshhold.
                    break;
                }
                case input_port_modes::MultiButton:
                {
                    // @todo not sure if this will be implemented.
                    break;
                }
            }
        }

        outs[0].state = OutputPortManager::OutPortState::out_port_state::Off;

        // check the multicore fifo for data...
        while (multicore_fifo_rvalid()) {
            // read fifo into uint32
            uint32_t fifo_in = multicore_fifo_pop_blocking();
            uint8_t d_type = fifo_in >> 24;
            // switch/case to do something
            switch (d_type) {
                case m_core_fifo_D_types::setInPortMode:
                    {
                        // uint8_t 
                        // ins.modes[0] = newmode;
                        break;
                    }
                case m_core_fifo_D_types::setOutPortMode:
                    {
                        break;
                    }
                case m_core_fifo_D_types::setUpdateRate:
                    {
                        // @todo not sure if this is needed
                        break;
                    }
                default:
                    break;
            }
        } // loop through the entire available fifo. Other core may have stacked updates while this core was busy.
    }
}
/*


                                                            .-----.
                                                          ,' __/|_ `.
                                                         ,    {~)    :
                                        /:\              :   //~_)  _:__
                            __,---.__ /::::\             : ~~~/' \~~====
                           `-.__     \:::::/              :   ~;;~  .'
                             ;;:\--.__`--.--._             `._____.'
                           ,;;'` `    `--.__  `-._
                           `,  ,\       /,  `--.__;
                           <   (o) ___ (o)   >
                          <        \:/        >
                           <     ._,"._,     >"
                   _.---._  `-.    ~~~    .-'
                 .'._.--. `.   `~:~~~~~~:'
                 `-'     `. `.  :        :
                          :__: :________  :___
                      ;'xx:XXxxxxxxxxxx:xxxXXX:xx:
                    :::xx:XXXX:xxxxxxxxx:XXXXXX:xxx:.
                   ::xxx:XXX/X;xxxxxxxxxx:XXXXXX:xxx:.
           |||    ::xxx:XXX// xxxxxxxxxx// XXXXXX:xxx:.         []
         ||||||  ||xxxx:XX//   xxxxxxxx//   XXXXXX:xx||     .:||:|:||.
    ___    |||   ||xxx:XX//  0  xxxxxx// 0   XXXXX:xx||    .:||^:|:^||.
  ,'   `.. |||   `::xx:XXXXX:xxxx/ \xxxxx:XXXXXXXX:xx:'    ::|:::V:::|:
  |     || |||    `::xx:XXXXX:xxx___Xxxx:XXXXXXXX:xx:'     `::|UUUUU:|'
  |R.I.P|| |||     `::xx:XXXXXxxxxxxxxxxxXXXXXXX:x::'       `::|::::|'
  |     ||"""""      `:xx:XX \/  \/  \/ \/XXXXX:xx:'       """"""""""""
 """""""""             `.x:XXXXxxxxxxxxxxXXXXxx;''
""""""""""""""""""""""   ~~~~~~~~~~~~~~~~~~~~~~   """"""""""""""""""""""""


*/