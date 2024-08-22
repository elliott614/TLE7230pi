/*****TO BUILD:  ***********************************************
g++ -pthread [main.cpp] -o [main] -lpigpiod_if2 -lrt -std=c++20


*
*****TO RUN: First make sure the pigpio daemon is running******
sudo pigpiod

Then:
./[object name]
 
 **A BATCH FILE/SCRIPT IS PROBABLY A GOOD IDEA

*/

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <unistd.h>
#include <cstring>
#include <bitset>
#include <pigpiod_if2.h>
#include <vector>
#include <exception>
#include <errno.h>
#include <algorithm>
#include <type_traits>
#include <numeric>

using namespace std;

class TLE7230
{
    private:

        // RTN value of spi_open or bb_spi_open [bit-banged] goes here
        // initialize to {-1,-1} as 0 is a valid value
        pair<int, int> spiHandles;
        static constexpr int BUFFERLEN = 4;
        char* buffer;
        int PI; //needed for pigpio. RTN value of pigpio_start goes here
        bool daisyChain; //if false, use chipselect0 vs. chipselect1 and tx 16 bits per frame. if true only one CSn and 32b/frame

        static constexpr int FLTN1   = 14; //GPIO14 is FLTn (not fault on IC1)
        static constexpr int FLTN2   = 15; //GPIO15
        static constexpr int RSTN    = 16; //GPIO16
        static constexpr char DIAGNOSIS_ONLY  = 0b00000000; //spi commands. e.g. READ_REGISTER | MAP reads MAP reg
        static constexpr char READ_REGISTER   = 0b01000000; //spi commands. e.g. READ_REGISTER | MAP reads MAP reg
        static constexpr char RESET_DEVICE    = 0b10000000; //spi commands. e.g. READ_REGISTER | MAP reads MAP reg
        static constexpr char WRITE_REGISTER  = 0b11000000; //spi commands. e.g. READ_REGISTER | MAP reads MAP reg
        static constexpr int MOSI = 10;
        static constexpr int MISO = 9;
        static constexpr int SCLK = 11;
        static constexpr int CSN1 = 8;
        static constexpr int CSN2 = 7;

        bool __externalPiGpioHandle;
        
        //SPI_MODE_1 (0,1)   CPOL = 0, CPHA = 1, Clock idle low, data is clocked in on falling edge, output data (change) on rising edge
        static constexpr int SPI_MODE = 1;

        //holds last-received diagnosis status bits received.
        //    0b8877_6655_4433_2211 where #s are CHx
        //    0b11 = normal, 0b10 = overcurrent/SCB, 0b01 = open load, 0b00 = short to ground
        pair<bitset<16>, bitset<16>> diagStatus = pair<bitset<16>, bitset<16>>(bitset<16>(0xFFFF), bitset<16>(0xFFFF));
        
        //public version uses copy constructor
        constexpr bitset<16> __diagStatus(int device)
        {
            return device == 2 ? diagStatus.second : diagStatus.first;
        }

        // (Daisy-Chain) Result of the read will be in buffer
        // @warning daisy chain only @throws runtime_error
        // @param addr1 address of register in MOSI-receiving device @param addr2 address of register in MISO-transmitting device
        int __readRegisters(char addr1, char addr2)
        {
            if(addr1 < 1 || addr1 > 8) throw runtime_error("invalid address1; __readRegisters\n");
            if(addr2 < 1 || addr2 > 8) throw runtime_error("invalid address2; __readRegisters\n");
            if(!daisyChain)            throw runtime_error("error: __readRegisters() daisy chain method used w/o daisy chain enabled\n");
            //buffer tx/rx big endian:  [3:LSB1][2:MSB1][1:LSB2][0:MSB2]---->
            buffer[0] = READ_REGISTER | addr2;
            buffer[2] = READ_REGISTER | addr1;
            int result = SpiWriteAndRead(spiChannel(1), buffer, 4);
                if(result < 0) return result;
            diagStatus.second = bitset<16>((buffer[0] << 8) | buffer[1]);
            diagStatus.first  = bitset<16>((buffer[2] << 8) | buffer[3]);
            //printf("diagnosis out: %#x %#x %#x %#x\n", buffer[0], buffer[1], buffer[2], buffer[3]);
            buffer[0] = DIAGNOSIS_ONLY;
            //buffer[1] = DIAGNOSIS_ONLY; //don't care
            return SpiWriteAndRead(spiChannel(1), buffer, 4); //read result returned on next SPI frame, send diagnosis cmd
        }

        // (Daisy-Chain) Result of the read will be in buffer
        // @warning daisy chain only @throws runtime_error
        // @param addr address of register
        int __readRegisters(char addr)
        {
            return __readRegisters(addr, addr);
        }

        // (Daisy-Chain) Result of the read will be in buffer
        // @warning daisy chain only @throws runtime_error
        // @param device 1 (MOSI-receiving device) or 2 (MISO-transmitting device) @param addr address to read
        int __readRegister(int device, char addr)
        {
            //cout<<"__readRegister()\n";
            if(addr < 1 || addr > 8) throw runtime_error("invalid address; __readRegister");
            if(daisyChain)
                throw runtime_error("error, __readRegister() non-daisy chain method used w/ daisy chain enabled; use __readRegisters()");
            buffer[0] = READ_REGISTER | addr;
            //buffer[1] = READ_REGISTER | addr; //don't care
            //printf("SPI_OUT to device %d: %#x %#x\n", device, buffer[0], buffer[1]);
            int result = SpiWriteAndRead(spiChannel(device), buffer, 2);
            //cout<<"SpiWriteAndRead() rtn: "<<result<<'\n';
            if(result < 0) return result;
            (device == 1 ? diagStatus.first : diagStatus.second) = bitset<16>((buffer[0] << 8) | buffer[1]);
            //printf("diagnostics shifted out during read command: %#x %#x\n", buffer[0], buffer[1]);
            //cout<<"sending diagnosis command to get result of read\n";
            buffer[0] = DIAGNOSIS_ONLY;
            //buffer[1] = DIAGNOSIS_ONLY; //don't care
            //printf("SPI_OUT to device %d: %#x %#x\n", device, buffer[0], buffer[1]);
            result = SpiWriteAndRead(spiChannel(device), buffer, 2);
            //cout<<"SpiWriteAndRead() rtn: "<<result<<'\n';
            //printf("read result shifted out: %#x %#x\n", buffer[0], buffer[1]);
            return result;
        }

        //daisy-chained is always channel 0. Otherwise spiChannel(1) is 0, spiChannel(2) is 1.
        constexpr int spiChannel(const int device) {return !daisyChain && device == 2;}

        //wrapper around pigpio spi_xfer method to simplify calls, no need for multiple buffers
        int SpiWriteAndRead (int channel, char *Buffer, int Length)
        {
            int h = channel ? spiHandles.second : spiHandles.first;
            
            if(h < 0)
            {
                cerr<<"error: spi handle for channel "<<channel<<" doesn't exist\n";
                return -1;
            }
            return spi_xfer(PI, h, Buffer, Buffer, Length);
        }

//*******************************************************************************************************************
//*************************************PUBLIC************************************************************************
//*******************************************************************************************************************


    public:
        static constexpr char MAP = 1; //IN4 mapping register; MAP[relay-1] mapped to IN4 relay=1-8; default = 1<<(4-1)
        static constexpr char BOL = 2; //Boolean mode. BOL[relay-1] 0=OR 1=AND SPI w/ parallel inputs; default=0
        static constexpr char OVL = 3; //current overload, 0=current limit only, 1=latching shutdown; default=0
        static constexpr char OVT = 4; //overtemp, 0=restart after cooldown, 1=latching shutdown; default=0
        static constexpr char SLE = 5; //slew rate, 0=10us 1=50us; default=0
        static constexpr char STA = 6; //DMOS state, read only. Use CTL, not STA to read on/off state. Not sure what this is but it's not the same
        static constexpr char CTL = 7; //control ; default = 0
        static constexpr int RELAY_CH_OK        = 0b11; //normal function
        static constexpr int RELAY_CH_OVERLOAD  = 0b10; //overcurrent or SCB
        static constexpr int RELAY_CH_OPEN      = 0b01; //open load/open circuit
        static constexpr int RELAY_CH_SHORT2GND = 0b00; //short to ground



        // Class handles TLE7230 stuff for BOTH devices, so only create one.
        // If using this c++ class, compilation requires flags in:                                                                    "
        //              g++ -pthread [main.cpp] -o [main] -lpigpiod_if2 -lrt -std=c++20                                        "
        //
        //
        //  @attention ***TO RUN YOUR PROGRAM*** Must ensure sure pigpio daemon is running:
        //                                                                                  shell command => " sudo pigpiod " 
        //  @param daisyChain: Is the SPI in daisy-chain configuration with device 1 SPI-out => device 2 SPI-in ? (default: false)
        //  @param baud: frequency (Hz) of SCLK (default: 4194304)
        //  @param PI: (optional) if externally-managed piGPIO instance is being used, this is its handle. Otherwise pass nothing.
        //            default value indicates this module calls pigpio_start(...) in constructor and pigpio_stop(PI) in destructor.
        //  @throws runtime_error if pigpio handle cannot be acquired or spi port cannot be opened
        TLE7230(bool daisyChain = false, int baud = 4194304, int PI = 0x42) 
        : daisyChain(daisyChain), PI(PI)
        {
            __externalPiGpioHandle = (PI != 0x42);
            if(!__externalPiGpioHandle) this->PI = pigpio_start(NULL, NULL);
            if(this->PI < 0) throw runtime_error(__externalPiGpioHandle ? "invalid external piGPIO handle" : "could not start pigpio");
            spiHandles = pair<int, int>(spi_open(this->PI, spiChannel(1), baud, SPI_MODE), daisyChain ? -1 : spi_open(this->PI, spiChannel(2), baud, SPI_MODE));
            if(spiHandles.second < 0 && !daisyChain) throw runtime_error("could not open SPI port 2" + string(spiHandles.first < 0 ? " or 1" : ""));
            if(spiHandles.first < 0) throw runtime_error("could not open SPI port 1");
            //set RSTn to output
            set_mode(this->PI, RSTN, PI_OUTPUT);
            //set FLTnX to inputs
            set_mode(this->PI, FLTN1, PI_INPUT);
            set_mode(this->PI, FLTN2, PI_INPUT);
            //set_pull_up_down(pi, RSTN, PI_PUD_UP);
            set_pull_up_down(this->PI, FLTN1, PI_PUD_UP);
            set_pull_up_down(this->PI, FLTN2, PI_PUD_UP);
            set_pull_up_down(this->PI, RSTN, PI_PUD_DOWN);
            buffer = new char[BUFFERLEN];
        }


        ~TLE7230()
        {
            spi_close(PI, spiHandles.first);
            if(daisyChain) spi_close(PI, spiHandles.second);
            if(!__externalPiGpioHandle) pigpio_stop(PI);
            cout<<"~TLE7230() destructor called\n";
        }

        int getGpioHandle() {return PI;}
        int getFLTN1() {return gpio_read(PI, FLTN1);}
        int getFLTN2() {return gpio_read(PI, FLTN2);}
        int writeRSTn(bool level) {return gpio_write(PI, RSTN, level);}

        // Get Diagnosis Status for one TLE7230
        // @return std::bitset<16> with the TLE7230 object's internal diagnosis status object's current value
        // (i.e., copy constructs)
        // @param device 1 or 2
        constexpr bitset<16> getDiagStatus(int device) {return bitset<16>(device == 2 ? diagStatus.second : diagStatus.first);}

        // Get Diagnosis Status for both TLE7230s
        // @return <std::bitset<16>, std::bitset<16>> with the TLE7230 object's internal diagnosis status object's current value
        // (i.e., copy constructs)
        // @param device 1 or 2
        pair<bitset<16>, bitset<16>> getDiagStatus() {return pair<bitset<16>, bitset<16>>(diagStatus);}

        //get status for specified device (1 or 2) and relay
        int relayStatus(int device, int relay)
        {
            return __diagStatus(device)[relay*2-1]<<1 | __diagStatus(device)[relay*2-2]; //use private version
        }

        //prints contents of diagStatus to console in easy to read format
        void printDiagStatus()
        {
            for(int d = 1; d <= 2; ++d)
                for(int r = 1; r <= 8; ++r)
                    switch(relayStatus(d, r))
                    {
                        case RELAY_CH_OK       :
                            printf("DEVICE %d RELAY %d: NORMAL FUNCTION\n", d, r);        
                            break;
                        case RELAY_CH_OVERLOAD :
                            printf("DEVICE %d RELAY %d: SHORT CIRCUIT/OVERLOAD\n", d, r);
                            break;
                        case RELAY_CH_OPEN     :
                            printf("DEVICE %d RELAY %d: OPEN LOAD\n", d, r);              
                            break;
                        default                :
                            printf("DEVICE %d RELAY %d: SHORT TO GROUND\n", d, r);
                    }
        }


        // Sends a DIAGNOSIS_ONLY command to the TLE92104R
        // @buffer will contain the diagStatus afterward
        int updateDiagStatus()
        {
            //cout<<"updateDiagStatus()\n";
            if(!daisyChain)
            {
                buffer[0] = DIAGNOSIS_ONLY;
                int result = SpiWriteAndRead(spiChannel(1), buffer, 2);
                if(result < 0) return result;
                diagStatus.first = bitset<16>((buffer[0] << 8) | buffer[1]);
                //printf("DiagStatus 1: %#x %#x", buffer[0], buffer[1]);
                buffer[0] = 0x00;
                result = SpiWriteAndRead(spiChannel(2), buffer, 2);
                if(result < 0) return result;
                diagStatus.second = bitset<16>((buffer[0] << 8) | buffer[1]);
                //printf(" DiagStatus 2: %#x %#x\n", buffer[0], buffer[1]);
                return result;
            }
            buffer[0]  = DIAGNOSIS_ONLY;
            buffer[2]  = DIAGNOSIS_ONLY; //the other bytes are don't care
            int result = SpiWriteAndRead(spiChannel(1), buffer, 4);
            if(result < 0) return result;
            diagStatus.second = bitset<16>((buffer[0] << 8) | buffer[1]);
            diagStatus.first  = bitset<16>((buffer[2] << 8) | buffer[3]);
            //printf("DiagStatus 1: %#x %#x DiagStatus2: %#x %#x\n", buffer[2], buffer[3], buffer[0], buffer[1]);
            return result;
        }


        // @attention DAISY-CHAIN ONLY @throws runtime_error
        // @param addr1 address of register in MOSI-receiving device @param addr2 address of register in MISO-transmitting device
        // @return pair<vector<char>, vector<char>> where rtn.first = contents of register @ addr1 of MOSI-receiving device
        //           and rtn.second = contents of register @ addr2 of MISO-transmitting device.
        pair<char, char> readRegisters(char addr1, char addr2)
        {
            if(__readRegisters(addr1, addr2) < 0) throw runtime_error("spi communication failed");
            return pair<char, char>(buffer[3], buffer[1]);
        }

        // @attention DAISY-CHAIN ONLY @throws runtime_error
        // @param addr1 address of register in MOSI-receiving device @param addr2 address of register in MISO-transmitting device
        // @return pair<vector<char>, vector<char>> where rtn.first = contents of register @ addr1 of MOSI-receiving device
        //           and rtn.second = contents of register @ addr2 of MISO-transmitting device.
        pair<char, char> readRegisters(char addr)
        {
            return readRegisters(addr, addr);
        }

        // @param addr address of register @param device 1: SPI ch1 device / MOSI receiving device (daisy-chain)
        //                                      2: SPI ch2 device / MISO-transmitting device (daisy-chain).
        // @return contents
        // @throws runtime_error
        char readRegister(int device, char addr)
        {
            if((!daisyChain ? __readRegister(device, addr) : __readRegisters(addr)) < 0) throw runtime_error("spi communication failed' readRegister()");
            //for(int i = 0; i < BUFFERLEN; ++i) printf("Buffer[%d]: %#x\n", i, buffer[i]);
            return buffer[((daisyChain & device == 1) << daisyChain) + 1/*branchless!*/]; // [0:MSB2][1:LSB2][2:MSB1][3:LSB1]
        }

        // writes 2 registers, one for each device, 2 different addresses w/ 2 different data
        // @param addr1 address for SPI CH1 / MOSI-receiving device (depends if daisy-chained) @param addr2 address for SPI CH2 / MISO-transmitting device
        // @param data1 written to SPI CH1 / MOSI-receiving device (depends if daisy-chained) @param data2 written to SPI CH2 / MISO-transmitting device
        // @return pigpio's SPI R/W return value (last time called)
        // @throws runtime_error
        int writeRegisters(char addr1, char addr2, char data1, char data2)
        {
            if(addr1 < 1 || addr1 > 8) throw runtime_error(string("invalid address1").append(" and address 2", 14*(addr2 < 1 || addr2 > 8)) + "; writeRegisters()");
            if(addr2 < 1 || addr2 > 8) throw runtime_error("invalid address2; writeRegisters()\n");
            if(daisyChain)
            {
                //buffer (tx and rx):  [3:LSB1][2:MSB1][1:LSB2][0:MSB2]
                buffer[0] = WRITE_REGISTER | addr2;
                buffer[2] = WRITE_REGISTER | addr1;
                buffer[1] = data2;
                buffer[3] = data1;
                int result = SpiWriteAndRead(spiChannel(1), buffer, 4);
                if(result < 0) return result;
                diagStatus.second = bitset<16>((buffer[0] << 8) | buffer[1]);
                diagStatus.first  = bitset<16>((buffer[2] << 8) | buffer[3]);
                return result;
            }
            buffer[0] = WRITE_REGISTER | addr1;
            buffer[1] = data1;
            int result = SpiWriteAndRead(spiChannel(1), buffer, 2);
            if(result < 0) return result;
            diagStatus.first = bitset<16>((buffer[0] << 8) | buffer[1]);
            buffer[0] = WRITE_REGISTER | addr2;
            buffer[1] = data2;
            result = SpiWriteAndRead(spiChannel(2), buffer, 2);
            if(result < 0) return result;
            diagStatus.second = bitset<16>((buffer[0] << 8) | buffer[1]);
            return result;
        }

        // writes 2 registers, one for each device, 1 address w/ 2 different data
        // @param addr address
        // @param data1 written to SPI CH1 / MOSI-receiving device (depends if daisy-chained) @param data2 written to SPI CH2 / MISO-transmitting device
        // @return pigpio's SPI R/W return value (last time called)
        // @throws runtime_error
        int writeRegisters(char addr, char data1, char data2)
        {
            return writeRegisters(addr, addr, data1, data2);
        }

        // writes 2 registers, one for each device, 1 address, 1 data
        // @param addr address
        // @param data1 written to SPI CH1 / MOSI-receiving device (depends if daisy-chained) @param data2 written to SPI CH2 / MISO-transmitting device
        // @return pigpio's SPI R/W return value (last time called)
        // @throws runtime_error
        int writeRegisters(char addr, char data)
        {
            return writeRegisters(addr, data, data);
        }

        // This will only write the register for 1 device and if daisy chained send a get diagnosis commmand for the other.
        // @param device 1: SPI CH1 / MOSI-receiving device (depends if daisy-chained); 2: SPI CH2 / MISO-transmitting device
        // @param addr address to which to write data
        // @param data data to write to addr
        // @throws runtime_error
        // @return last SpiWriteAndRead rtn from pgiop
        int writeRegister(int device, char addr, char data)
        {
            //cout<<"writeRegister()\n";
            if(device < 1 || device > 2) throw runtime_error("invalid device; writeRegister()");
            if(addr < 1 || addr > 8)throw runtime_error("invalid address; writeRegister()");
            if(daisyChain)
            {
                //buffer tx/rx:  [3:LSB1][2:MSB1][1:LSB2][0:MSB2]
                buffer[0] = DIAGNOSIS_ONLY;
                buffer[2] = DIAGNOSIS_ONLY;
                buffer[(2-device)*2] = WRITE_REGISTER | addr;
                buffer[(2-device)*2+1] = data;
                int result = SpiWriteAndRead(spiChannel(1), buffer, 4);
                if(result < 0) return result;
                diagStatus.second = bitset<16>((buffer[0] << 8) | buffer[1]);
                diagStatus.first  = bitset<16>((buffer[2] << 8) | buffer[3]);
                return result;
            }
            buffer[0] = WRITE_REGISTER | addr;
            buffer[1] = data;
            //printf("SPI_OUT to device %d: %#x %#x\n", device, buffer[0], buffer[1]);
            int result = SpiWriteAndRead(spiChannel(device), buffer, 2);
            //cout<<"SpiWriteAndRead() rtn: "<<result<<'\n';
            if(result < 0) return result;
            (device == 1 ? diagStatus.first : diagStatus.second) = bitset<16>((buffer[0] << 8) | buffer[1]);
            //printf("diagnostics shifted out during write command: %#x %#x\n", buffer[0], buffer[1]);
            return result;
        }

        // Resets logic registers to defaults, one device only
        // @param device 1: SPI CH1 / MOSI-receiving device (depends if daisy-chained); 2: SPI CH2 / MISO-transmitting device
        // @throws runtime_error
        // @return last SpiWriteAndRead rtn from pgiop
        int resetRegisters(int device)
        {
            if(device < 1 || device > 2) throw runtime_error("invalid device; writeRegister()");
            if(daisyChain)
            {
                buffer[0] = DIAGNOSIS_ONLY; //don't reset device that isn't passed to method
                buffer[2] = DIAGNOSIS_ONLY;
                buffer[(2-device)*2] = RESET_DEVICE;
                int result = SpiWriteAndRead(spiChannel(device), buffer, 4);
                if(result < 0) return result;
                diagStatus.second = bitset<16>((buffer[0] << 8) | buffer[1]);
                diagStatus.first  = bitset<16>((buffer[2] << 8) | buffer[3]);
                return result;
            }
            buffer[0] = RESET_DEVICE;
            //buffer[1] = RESET_DEVICE; //don't care
            int result = SpiWriteAndRead(spiChannel(device), buffer, 2);
            if(result < 0) throw runtime_error("SPI communication failed");
            (device == 1 ? diagStatus.first : diagStatus.second) = bitset<16>((buffer[0] << 8) | buffer[1]);
            return result;
        }

        // Resets both devices' logic registers to defaults, one device only
        // @throws runtime_error
        // @return last SpiWriteAndRead rtn from pgiop
        int resetRegisters()
        {
            if(daisyChain)
            {
                buffer[0] = RESET_DEVICE; //don't reset device that isn't passed to method
                buffer[2] = RESET_DEVICE;
                int result = SpiWriteAndRead(spiChannel(1), buffer, 4);
                if(result < 0) throw runtime_error("SPI communication failed");
                diagStatus.second = bitset<16>((buffer[0] << 8) | buffer[1]);
                diagStatus.first  = bitset<16>((buffer[2] << 8) | buffer[3]);
                return result;
            }
            buffer[0] = RESET_DEVICE;
            //buffer[1] = RESET_DEVICE; //don't care
            int result = SpiWriteAndRead(spiChannel(1), buffer, 2);
            if(result < 0) throw runtime_error("SPI communication failed w/ device 1");
            diagStatus.first  = bitset<16>((buffer[2] << 8) | buffer[3]);
            buffer[0] = RESET_DEVICE;
            //buffer[1] = RESET_DEVICE; //don't care
            result = SpiWriteAndRead(spiChannel(2), buffer, 2);
            if(result < 0) throw runtime_error("SPI communication failed w/ device 2");
            diagStatus.second = bitset<16>((buffer[0] << 8) | buffer[1]);
            return result;
        }

        // @param device 1: SPI CH1 / MOSI-receiving device (depends if daisy-chained); 2: SPI CH2 / MISO-transmitting device
        // @param relay 1 to 8, the OUTx channel of TLE7230
        // @throws runtime_error
        // @return last SpiWriteAndRead rtn from pgiop
        int turnRelayOn(int device, int relay)
        {
            if(relay < 1 || relay > 8) throw runtime_error("invalid relay; turnRelayOn()");
            //cout<<"turnRelayOn()\n";
            int result = daisyChain ? __readRegisters(CTL) : __readRegister(device, CTL);
            if(result < 0) return result;
            //cout<<"CTL Register State Read\n";
            //daisychain buffer: [0: MSB2][1: LSB2][2: MSB1][3: LSB1]
            if(daisyChain) return writeRegister(device, CTL, buffer[(2-device)*2+1] | (0x01 << (relay-1)));
            return writeRegister(device, CTL, buffer[1] | (0x01 << (relay-1)));
        }

        // @param device 1: SPI CH1 / MOSI-receiving device (depends if daisy-chained); 2: SPI CH2 / MISO-transmitting device
        // @param relays bitset<8>/vector<bool>/string("[10101010]")/integer([0b01010101])
        //         or collection (e.g. vector<int>, set<int>, unordered_set<int>, list<int>, array<int>) of numbers 1 to 8 for output channel of device
        // @return pigpio SPI return value (if error, negative) @throws runime_exception
        // @attention string literal will be interpreted as a char[], use string("01010011") if not wanting to send e.g., [0x01][0x3][0x06] to turn on 1/3/6
        template <typename T> int turnRelaysOn(int device, T relays)
        {
            bitset<8> r;
            if constexpr (is_constructible_v<bitset<8>, T>) r = bitset<8>(relays);
            else if constexpr (is_convertible_v<T, vector<bool>>) ranges::for_each(relays, [&r](vector<bool>::reference rl){(r>>=1)[7] = rl;});
            else ranges::for_each(relays, [&r](int &rl){rl >= 1 && rl <= 8 ? r.set(rl-1) : throw runtime_error("invalid relay #");});
            int result = daisyChain ? __readRegisters(CTL) : __readRegister(device, CTL);
            if(result < 0) return result;
            if(daisyChain) return writeRegister(device, CTL, buffer[(2-device)*2+1] | r.to_ullong());
            return writeRegister(device, CTL, (buffer[1]) | r.to_ullong());
        }

        // @param device 1: SPI CH1 / MOSI-receiving device (depends if daisy-chained); 2: SPI CH2 / MISO-transmitting device
        // @param relay 1 to 8, the OUTx channel of TLE7230
        // @throws runtime_error
        // @return last SpiWriteAndRead rtn from pgiop
        int turnRelayOff(int device, int relay)
        {
            if(relay < 1 || relay > 8)
            {
                cerr << "invalid relay; turnRelayOff()\n";
                return -1;
            }
            //cout<<"turnRelayOff()\n";
            int result = daisyChain ? __readRegisters(CTL) : __readRegister(device, CTL);
            if(result < 0) return result;
            //cout<<"CTL Register State Read\n";
            //daisychain buffer out: [0: MSB2][1: LSB2][2: MSB1][3: LSB1]
            if(daisyChain) return writeRegister(device, CTL, buffer[(2-device)*2+1] & ~(0x01<<(relay-1)));
            return writeRegister(device, CTL, buffer[1] & ~(0x01<<(relay-1)));
        }

        // @param device 1: SPI CH1 / MOSI-receiving device (depends if daisy-chained); 2: SPI CH2 / MISO-transmitting device
        // @param relays bitset<8>/vector<bool>/string("[10101010]")/integer([0b01010101])
        //    or collection (e.g. vector<int>, set<int>, unordered_set<int>, list<int>, array<int>) of numbers 1 to 8 for output channel of device
        //   @attention string literal will be interpreted as a char[], use string("[0bxxxxxxxx]") if not wanting to send e.g., [0x01][0x3][0x06] to turn off 1/3/6
        // @return pigpio SPI return value (if error, negative) @throws runime_exception
        template <typename T> int turnRelaysOff(int device, T relays)
        {
            bitset<8> r;
            if constexpr (is_constructible_v<bitset<8>, T>) r = bitset<8>(relays).flip();
            else if constexpr (is_convertible_v<T, vector<bool>>) ranges::for_each(relays, [&r](vector<bool>::reference rl){(r>>=1)[7]=!rl;});
            else
            {
                r.set();
                std::ranges::for_each(relays, [&r](int &rl){rl >= 1 && rl <= 8 ? r.reset(rl-1) : throw runtime_error("invalid relay #");});
            }
            int result = daisyChain ? __readRegisters(CTL) : __readRegister(device, CTL);
            if(result < 0) return result;
            if(daisyChain) return writeRegister(device, CTL, (bitset<8>(buffer[(2-device)*2+1]) & r).to_ulong());
            return writeRegister(device, CTL, (bitset<8>(buffer[1]) & r).to_ullong());
        }

        //test script that reads the relevant GPIO pins, writes RST low/high, and then turns each relay on then off for 1 second
        int test()
        {
            cout<<"PI: "<<PI<<'\n';
            cout<<"CSn1 read: "<<gpio_read(PI, CSN1)<<" CSn2 read: "<<gpio_read(PI, CSN1);
            cout<<" FLTn1 read: "<<gpio_read(PI, FLTN1)<<" FLTn2 read: "<<gpio_read(PI, FLTN2)<<'\n';
            sleep(1);
            cout<<"write RSTN->low\n";
            cout<<"gpio_write rtn: "<<gpio_write(PI, RSTN, PI_OFF)<<'\n';
            cout<<"RSTN read: "<<gpio_read(PI, RSTN)<<'\n';
            sleep(1);
            cout<<"write RSTN->high\n";
            gpio_write(PI, RSTN, PI_ON);
            sleep(1);
            cout<<"RSTn read: "<<gpio_read(PI, RSTN)<<'\n';
        //turn each relay on + wait1s + turn off, one at a time
        for(int i = 1; i <= 2; ++i)
        {
            for(int j = 1; j <= 8; ++j)
            {
                cout<<"\n***********************turning on device "<<i<<" channel "<<j<<'\n';
                int result = turnRelayOn(i, j);
                if(result < 0)
                {
                    cerr<<"error: "<<result<<'\n';
                    return -1;
                };
                sleep(1);
                cout<<"slept 1sec\n";
                updateDiagStatus();
                cout<<"FLTn1: "<<gpio_read(PI, FLTN1)<<" FLTn2: "<<gpio_read(PI, FLTN2)<<'\n';
                cout<<"diagnostic status 1: "<<diagStatus.first<<'\n';
                cout<<"diagnostic status 2: "<<diagStatus.second<<'\n';
                cout<<"\n**********************turning off device "<<i<<" channel "<<j<<'\n';
                result = turnRelayOff(i, j);
                if(result < 0)
                {
                    cerr<<"error: "<<result<<'\n';
                    return -1;
                };
                sleep(1);
                cout<<"slept 1sec\n";
                updateDiagStatus();
                cout<<"diagnostic status 1: "<<diagStatus.first<<'\n';
                cout<<"diagnostic status 2: "<<diagStatus.second<<'\n';
                printDiagStatus();
            }
        }
        return 0;
    }
};