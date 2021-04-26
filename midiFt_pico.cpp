// ignores most of the code
// cSpell:ignoreRegExp /(^(?!\s*(\/\/)|(\/\*)).*[;:)}=,{])/gm

// ignores any word in quotes
// cSpell:ignoreRegExp /\"\S*\"/g

//--- ignores HEX literals
// cSpell:ignoreRegExp /0x[A-Z]+/g

//--- ignores any preprocessor directive (i.e #define)
// cSpell:ignoreRegExp /(^#.*)/gm

/// words to ignore
// cSpell:ignore pico PSRAM btn btns spec'd dbgserPrintln dbgser Println gpio

/// spell check extension defaults to checking each part of camel case words as separate words.

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"
#include "include/midiFeetPICO_lib.h"
#define I2C0_PORT i2c0
#define I2C1_PORT i2c1

void core1_entry_i2cSlave() {
    puts("\nCore #2 started.\n");
    i2c_slave_handler slaveHandler = i2c_slave_handler(); // global to pass data into handler function.
    TEENSY_I2C TeensyCommands = TEENSY_I2C(slaveHandler);
    InputPortState inState;
    puts("Entering loop...");
    while (1) {
        if (I2C0_PORT->hw->clr_rd_req && slaveHandler.readyToWrite) {
            // HW will hold the bus until the register is reset.
            hw_clear_bits(I2C_IC_RAW_INTR_STAT_RD_REQ_RESET, I2C0_PORT->hw->clr_rd_req);
            if (slaveHandler.numBytesToWrite > 0) {
                i2c_write_raw_blocking(I2C0_PORT, slaveHandler.data_out, slaveHandler.numBytesToWrite);
            } else {
                i2c_write_raw_blocking(I2C0_PORT, 0, 1);
            }
            slaveHandler.readyToWrite = false; // reset flag
            slaveHandler.edgeEvent_F = i2c_slave_handler::edge_not_detected;
        }

        // uint64_t timeDiff = absolute_time_diff_us(timeOld,timeNew);
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
                        slaveHandler.data_out[0] = TEENSY_I2C::P2T_REPONSE::InputUpdate | portsRequested | slaveHandler.edgeEvent_F;
                        
                        // Set the second and third bytes to reflect the data in the state trackers for each input.
                        // This gets sent every time in order to ensure that the packet has an easily predictable structure.
                        uint8_t u = 0;
                        slaveHandler.data_out[1] = 0;
                        for (; u < 4; u++) if(inState.tipOn[u])slaveHandler.data_out[1] |= (1 << u);
                        for (; u < 8; u++) if(inState.ringOn[u-4])slaveHandler.data_out[1] |= (1 << u);
                        slaveHandler.data_out[2] = inState.combSt;

                        uint8_t numBytes = 3;
                        for (uint8_t i = 0; i < 4; i++) {
                            // if this iteration corresponds to a requested port...
                            if (((portsRequested >> i) & 1) == 1) {
                                // add data to data_out based on port mode
                                switch (inState.mode[i]) {
                                    case input_port_modes::ExpPedalMinMax... input_port_modes::ExpPedalContinuous:
                                        {
                                            slaveHandler.data_out[numBytes] = 0;
                                            slaveHandler.data_out[numBytes] = inState.expValue[i] & 0xff;
                                            slaveHandler.data_out[numBytes + 1] = (inState.expValue[i] >> 8) & 0xff;
                                            numBytes += 2;
                                            break;
                                        }
                                    case 255:// input_port_modes::SingleButton... input_port_modes::DualButton:
                                        {
                                            uint8_t u = 0;
                                            slaveHandler.data_out[numBytes] = 0;
                                            for (; u < 4; u++) {
                                                if(inState.tipOn[u])slaveHandler.data_out[numBytes] |= (1 << u);
                                                // slaveHandler.data_out[numBytes] |= ((inState.tipOn[u] ? 1 : 0) << u);
                                                // printf("\ntipOn: ");
                                                // printf((inState.tipOn[u] ? "true" : "false"));
                                                // printf("\n");
                                            }
                                            for (; u < 8; u++) {
                                                if(inState.ringOn[u-4])slaveHandler.data_out[numBytes] |= (1 << u);
                                                // slaveHandler.data_out[numBytes] |= ((inState.ringOn[u] ? 1 : 0) << (u + 4));
                                                // printf("\ntipRing: ");
                                                // printf((inState.tipOn[u] ? "true" : "false"));
                                                // printf("\n");
                                            }
                                            slaveHandler.data_out[numBytes + 1] = inState.combSt;
                                            numBytes += 2;

                                            break;
                                        }
                                    case input_port_modes::MultiButton:
                                        {
                                            slaveHandler.data_out[numBytes] = 0;
                                            //@todo Not sure if this will be implememtned.
                                            break;
                                        }
                                    default:
                                    {
                                        // slaveHandler.data_out[numBytes] = 0;
                                        // slaveHandler.data_out[numBytes + 1] = 0;
                                        // numBytes += 2;
                                    }
                                        break;
                                }
                            }
                        }
                        slaveHandler.numBytesToWrite = numBytes;
                        slaveHandler.readyToWrite = true;
                        break;
                    }
                case TEENSY_I2C::T2P_COMMMANDS::OutputStateUpdate:
                    {
                        // if the fifo is full, we need to wait until there's room for the data or it could be lost.
                        // puts("got output update. flushing fifo.");
                        while (!multicore_fifo_wready()) { }
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
                        while (!multicore_fifo_wready()) { }
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
                        while (!multicore_fifo_wready()) { }
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
                        while (!multicore_fifo_wready()) { }
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
                        uint8_t data = (fifo_in & 0xff);
                        uint8_t mask = ( fifo_in >> 8 ) & 0xff;
                        inState.combSt = (inState.combSt & ~mask) | data;
                        for (uint8_t i = 0; i < 4; i++) {
                            if((mask & 1) == 1){
                                inState.tipOn[i] = (1 == (data & 1));
                            }
                            data >>= 1;
                            mask >>= 1;
                        }
                        for (uint8_t i = 0; i < 4; i++) {
                            if((mask & 1) == 1){
                                inState.ringOn[i] = (1 == (data & 1));
                            }
                            data >>= 1;
                            mask >>= 1;
                        }
                        break;
                    }
                case m_core_fifo_D_types::EdgeEvent:
                    {
                        if ((fifo_in & 0xff) > 0) {
                            slaveHandler.edgeEvent_F = i2c_slave_handler::Edge_Event_FLAG::edge_detected;
                        } else {
                            slaveHandler.edgeEvent_F = i2c_slave_handler::Edge_Event_FLAG::edge_not_detected;
                        }
                    }
                default:
                    break;
            }
        } // loop through the entire available fifo. Other core may have stacked updates while this core was busy.
    }     // main loop for core 1
}

int main() {
    stdio_init_all();
    i2c_init(I2C0_PORT, 400 * 1000);
    i2c_set_slave_mode(I2C0_PORT, true, 0x5a);
    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_set_function(5, GPIO_FUNC_I2C);
    // gpio_pull_up(4);
    // gpio_pull_up(5);
    i2c_init(I2C1_PORT, 700 * 1000);
    gpio_set_function(2, GPIO_FUNC_I2C);
    gpio_set_function(3, GPIO_FUNC_I2C);
    gpio_pull_up(2);
    gpio_pull_up(3);
    for (uint8_t i = 8; i < 16; i++) {
        gpio_init(i);
        gpio_set_dir(i, GPIO_IN);
        gpio_pull_up(i);
    }
    sleep_ms(1000);

    puts("starting second core\r\n");
    multicore_launch_core1(core1_entry_i2cSlave);
    // flush the multicore fifo
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
    OutputPortManager outs[4] = {
        OutputPortManager(PortExpander,0),
        OutputPortManager(PortExpander,1),
        OutputPortManager(PortExpander,2),
        OutputPortManager(PortExpander,3),
    };
    InputPortManager ins = InputPortManager(AD_converter1, AD_converter2);
    repeating_timer_t timer;
    // add_repeating_timer_ms(1000, readADCandPrint, NULL, &timer);
    for (uint8_t i = 0; i < 4; i++) {
        currentSensors[i].setCalibration_16V_400mA();
    }
    countTo64 counting = countTo64(0, 100);
    countTo64 countedCounter;
    // absolute_time_t timeOld = get_absolute_time();
    // absolute_time_t timeNew = get_absolute_time();
    // uint64_t largestTime = 0;
    while (1) {

        //  Here for debugging purposes, this prints a dots several times a second.
        //  This way, it's always easy to tell if the loop is still running.
        if (counting == 0) {
            countedCounter++;
            if (countedCounter == 0) {
                printf("\n");
            }
            printf(".");
        }
        counting++;

        // Have the inputPortManager update all values. update() will return uint8_t based on number of inputs with edge event
        uint8_t edges = ins.update();
        if (edges > 0) { // This updates the states for all 4 input ports
            // printf("edges: %i\n",edges);
            while (!multicore_fifo_wready()) { }
            multicore_fifo_push_blocking((m_core_fifo_D_types::EdgeEvent << 24) | edges);
        }
        
        
        // }
        bool buttonsSent = false;
        // Do the things depending on the current mode
        for (uint8_t port_i = 0; port_i < 4; port_i++) {
            switch (ins.modes[port_i]) {
                case input_port_modes::Disabled:
                    {
                        // Do nothing. the port is set to disabled.
                        printf("port dsabled\n");
                        break;
                    }
                case input_port_modes::SingleButton ... input_port_modes::DualButton:
                    {
                        // Get combined state and
                        // send the update to the other core
                        
                        // printf("port buttons\n");
                        uint32_t fifo_out = 0;
                        if ((ins.tipRingEdgeTrack[port_i*2].combSt & 0xf0) == combinedState::state_OPEN_buttonPressed) {
                            fifo_out |= (1 << port_i);
                        }
                        if ((ins.tipRingEdgeTrack[(port_i*2) + 1].combSt & 0xf0) == combinedState::state_OPEN_buttonPressed) {
                            fifo_out |= (1 << (port_i + 4));
                        }

                        // create a mask for the ports we are actually sending data for. 
                        fifo_out |= (0x0100 << port_i);
                        fifo_out |= (0x0100 << (port_i + 4));

                        while (!multicore_fifo_wready()) { }
                        multicore_fifo_push_blocking((m_core_fifo_D_types::In_TR_state << 24) | fifo_out);

                        break;
                    }
                case input_port_modes::ExpPedalContinuous:
                    {
                        printf("port cont\n");
                        // calculate ratio for port. ratio is scaled up to avoid floating point math. 
                        // -1 at end to offset some error.
                        uint32_t ratio = (((uint32_t)ins.expPedals[(port_i * 2) + 1] * 10000) / ins.expPedals[(port_i * 2)]) - 1;
                        // Update the other core with a ratio value everytime
                        if (counting == 0)
                            // printf("\nratio: %d\n",ratio);
                            break;
                    }
                case input_port_modes::ExpPedalMinMax:
                    {
                        printf("port min max\n");
                        // update the other core with 1.0 or 0.0 ratio value depending on wether the pedal has hit min/max threshhold.
                        break;
                    }
                case input_port_modes::MultiButton:
                    {
                        printf("port multi\n");
                        // @todo not sure if this will be implemented.
                        break;
                    }
            }
        }

        // outs[0].state = OutputPortManager::OutPortState::out_port_state::Off;

        // check the multicore fifo for data...
        while (multicore_fifo_rvalid()) {
            // read fifo into uint32
            uint32_t fifo_in = multicore_fifo_pop_blocking();
            uint8_t d_type = fifo_in >> 24;
            // switch/case to do something
            // printf("d_type: %x", d_type);
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
                case m_core_fifo_D_types::Out_TR_state:
                    {
                        uint8_t TR_states = fifo_in & 0xff;
                        printf("\nTR update: %x\n",TR_states);
                        for(uint8_t i=0; i< 4; i++){
                            // outs[i].state = OutputPortManager::OutPortState::out_port_state::Off;
                            outs[i].state.Ring_on_b = (TR_states&1)==1;
                            outs[i].state.Tip_on_b = (TR_states>>4)&1==1;
                            TR_states >>= 1;
                            outs[i].UpdateOutput(); 
                        }
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