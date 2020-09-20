// PSoC libaray - MPU6050 module
// based on the MPU6050 I2C device libaray by Jeff Rowberg see contact below
// 17/09/2020 by Michael Stanley <cychu18@gmail.com>
// available at https://github.com/cychu18/PSoC_MPU6050
// The original I2C libaray functions are repalced with those in PSoC
// expected update: able to pass PSoC I2C module from main code and update the
// function call of respective I2C modules

/* ========================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
 * ========================================
*/
#include "MPU6050_PSoC.h"
//
int8_t readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout) {
    uint8_t b;
    uint8_t count = readByte(devAddr, regAddr, &b, timeout);
    *data = b & (1 << bitNum);
    return count;
}
int8_t readBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t *data, uint16_t timeout) {
    uint16_t b;
    uint8_t count = readWord(devAddr, regAddr, &b, timeout);
    *data = b & (1 << bitNum);
    return count;
}
int8_t readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout) {
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t count, b;
    if ((count = readByte(devAddr, regAddr, &b, timeout)) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *data = b;
    }
    return count;
}

int8_t readBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data, uint16_t timeout) {
    // 1101011001101001 read byte
    // fedcba9876543210 bit numbers
    //    xxx           args: bitStart=12, length=3
    //    010           masked
    //           -> 010 shifted
    uint8_t count;
    uint16_t w;
    if ((count = readWord(devAddr, regAddr, &w, timeout)) != 0) {
        uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        w &= mask;
        w >>= (bitStart - length + 1);
        *data = w;
    }
    return count;
}
int8_t readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t timeout) {
    return readBytes(devAddr, regAddr, 1, data, timeout);
}
int8_t readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data, uint16_t timeout) {
    return readWords(devAddr, regAddr, 1, data, timeout);
}
/************************************************************************
*Function:	int8_t readBytes()
*			
*Description:	Read Byte from the I2C. Ported from I2CDev.cpp for Arduino to PSoC  
*               the byte is loaded to a uint8_t data buffer
*
*Parameters:	
*Updates:	Date – 30.9.2019	Who - Michael Stanley	Reason – make life easier
************************************************************************/	
int8_t readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout) {
    int8_t count = 0;
    int8_t status;
    status = I2C_1_MasterSendStart(devAddr, 00);
    if(I2C_1_MSTR_NO_ERROR == status) 
    {
        status = I2C_1_MasterWriteByte(regAddr);
        I2C_1_MasterSendRestart(devAddr, 01); 
        
        for (uint8_t k = 0; k < length; k++){
            if (k==length-1){
                data[k]=I2C_1_MasterReadByte(I2C_1_NAK_DATA);
            }else{
                data[k]=I2C_1_MasterReadByte(I2C_1_ACK_DATA);
            }
            //increase the number of read Bytes
            count++;
        }
        I2C_1_MasterSendStop(); 
    }
    //
    return count;
}
/************************************************************************
*Function:	int8_t readWords()
*			
*Description:	Read Words (2 bytes from the I2C) Ported from I2CDev.cpp for Arduino to PSoC               
*               the byte is loaded to a uint16_t data buffer
*
*Parameters:	
*Updates:	Date – 30.9.2019	Who - Michael Stanley	Reason – make life easier
************************************************************************/	
int8_t readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data, uint16_t timeout) {
    int8_t count = 0;
    int8_t status;
    status = I2C_1_MasterSendStart(devAddr, 00);
    if(I2C_1_MSTR_NO_ERROR == status) 
    {
        status = I2C_1_MasterWriteByte(regAddr);
        I2C_1_MasterSendRestart(devAddr, 01); 
        
        for (uint8_t k = 0; k < length; k++){
            if (k==length-1){
                //first 8 bit
                data[k]=I2C_1_MasterReadByte(I2C_1_ACK_DATA)<<8; 
                //last 8 bits
                data[k]|=I2C_1_MasterReadByte(I2C_1_NAK_DATA);
            }else{
                data[k]=I2C_1_MasterReadByte(I2C_1_ACK_DATA)<<8;
                data[k]|=I2C_1_MasterReadByte(I2C_1_ACK_DATA);
            }
            //increase the number of read Words
            count++; 
        }
        I2C_1_MasterSendStop(); 
    }
    return count;
}

uint8_t writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    readByte(devAddr, regAddr, &b, 0);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return writeByte(devAddr, regAddr, b);
}
uint8_t writeBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data) {
    uint16_t w;
    readWord(devAddr, regAddr, &w, 0);
    w = (data != 0) ? (w | (1 << bitNum)) : (w & ~(1 << bitNum));
    return writeWord(devAddr, regAddr, w);
}
uint8_t writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b;
    if (readByte(devAddr, regAddr, &b,0) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        b &= ~(mask); // zero all important bits in existing byte
        b |= data; // combine data with existing byte
        return writeByte(devAddr, regAddr, b);
    } else {
        return 0;
    }
}
uint8_t writeBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data) {
    //              010 value to write
    // fedcba9876543210 bit numbers
    //    xxx           args: bitStart=12, length=3
    // 0001110000000000 mask word
    // 1010111110010110 original value (sample)
    // 1010001110010110 original & ~mask
    // 1010101110010110 masked | value
    uint16_t w;
    if (readWord(devAddr, regAddr, &w,0) != 0) {
        uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        w &= ~(mask); // zero all important bits in existing word
        w |= data; // combine data with existing word
        return writeWord(devAddr, regAddr, w);
    } else {
        return 0;
    }
}
uint8_t writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data) {
    return writeBytes(devAddr, regAddr, 1, &data);
}
uint8_t writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data) {
    uint8_t status = 0;
    
    status = I2C_1_MasterSendStart(devAddr, 00);
    if(I2C_1_MSTR_NO_ERROR == status) 
    {
        status = I2C_1_MasterWriteByte(regAddr);
        
        for (uint8_t k = 0; k < length; k++){
            status = I2C_1_MasterWriteByte(data[k]);
        }
        I2C_1_MasterSendStop(); 
    }
    return status == 0;
}
 uint8_t writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data) {
    return writeWords(devAddr, regAddr, 1, &data);
}
uint8_t writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t* data) {
    uint8_t status = 0;
    
    status = I2C_1_MasterSendStart(devAddr, 00);
    if(I2C_1_MSTR_NO_ERROR == status) 
    {
        status = I2C_1_MasterWriteByte(regAddr);
        
        for (uint8_t k = 0; k < length; k++){
            status = I2C_1_MasterWriteByte((uint8_t)data[k]>>8);
            status = I2C_1_MasterWriteByte((uint8_t)data[k]);
        }
        I2C_1_MasterSendStop(); 
    }
    return status == 0;
}

// IMU functioncs
void MPU_setSleep(uint8_t devAddr, uint8_t enabled){
    uint8_t status = 0;
    status = I2C_1_MasterSendStart(devAddr, 00);
     if(I2C_1_MSTR_NO_ERROR == status)
    {
        status = I2C_1_MasterWriteByte(0x6b);
        status = I2C_1_MasterSendRestart(devAddr, 01); 
        uint8_t data=I2C_1_MasterReadByte(I2C_1_NAK_DATA) ; 
        data = (enabled!=0)? (data | 0b01000000) :(data & 0b10111111);
        status = I2C_1_MasterWriteByte(data);
        I2C_1_MasterSendStop(); 
    }
}
int8_t getXGyroOffsetTC(uint8_t devAddr) {
    uint8_t buffer[10];
    readBits(devAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, buffer, 0);
    return buffer[0];
}
int8_t getYGyroOffsetTC(uint8_t devAddr) {
    uint8_t buffer[10];
    readBits(devAddr, MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, buffer,0);
    return buffer[0];
}
int8_t getZGyroOffsetTC(uint8_t devAddr) {
    uint8_t buffer[10];
    readBits(devAddr, MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, buffer,0);
    return buffer[0];
}

void setMemoryBank(uint8_t devAddr,uint8_t bank, uint8_t prefetchEnabled, uint8_t userBank) {
    bank &= 0x1F;
    if (userBank==1) bank |= 0x20;
    if (prefetchEnabled==1) bank |= 0x40;
    writeByte(devAddr, MPU6050_RA_BANK_SEL, bank);
}
void readMemoryBlock(uint8_t devAddr, uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address) {
    setMemoryBank(devAddr,bank,0,0);
    writeByte(devAddr, MPU6050_RA_MEM_START_ADDR, address);
    uint8_t chunkSize;
    for (uint16_t i = 0; i < dataSize;) {
        // determine correct chunk size according to bank position and data size
        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunkSize > dataSize) chunkSize = dataSize - i;

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address) chunkSize = 256 - address;

        // read the chunk of data as specified
        readBytes(devAddr, MPU6050_RA_MEM_R_W, chunkSize, data + i,0);
        
        // increase byte index by [chunkSize]
        i += chunkSize;

        // uint8_t automatically wraps to 0 at 256
        address += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < dataSize) {
            if (address == 0) bank++;
            setMemoryBank(devAddr,bank,0,0);
            writeByte(devAddr, MPU6050_RA_MEM_START_ADDR, address);
        }
    }
}
uint8_t writeMemoryBlock(uint8_t devAddr, const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, uint8_t verify, uint8_t useProgMem) {
    setMemoryBank(devAddr,bank,0,0);
    writeByte(devAddr, MPU6050_RA_MEM_START_ADDR, address);
    uint8_t chunkSize;
    uint8_t *verifyBuffer=0;
    uint8_t *progBuffer=0;
    uint16_t i;
    uint8_t j;
    
    if (verify==1) verifyBuffer = (uint8_t *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
    if (useProgMem==1) progBuffer = (uint8_t *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
    for (i = 0; i < dataSize;) {
        // determine correct chunk size according to bank position and data size
        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunkSize > dataSize) chunkSize = dataSize - i;

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address) chunkSize = 256 - address;
        
        if (useProgMem==1) {
            // write the chunk of data as specified
            for (j = 0; j < chunkSize; j++) progBuffer[j] = pgm_read_byte(data + i + j);
        } else {
            // write the chunk of data as specified
            progBuffer = (uint8_t *)data + i;
        }

        writeBytes(devAddr, MPU6050_RA_MEM_R_W, chunkSize, progBuffer);

        // verify data if needed
        if (verify==1 && verifyBuffer) {
            setMemoryBank(devAddr,bank,0,0);
            writeByte(devAddr, MPU6050_RA_MEM_START_ADDR, address);
            readBytes(devAddr, MPU6050_RA_MEM_R_W, chunkSize, verifyBuffer,0);
            if (memcmp(progBuffer, verifyBuffer, chunkSize) != 0) {

                free(verifyBuffer);
                if (useProgMem==1) free(progBuffer);
                return 0; // uh oh.
            }
        }

        // increase byte index by [chunkSize]
        i += chunkSize;

        // uint8_t automatically wraps to 0 at 256
        address += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < dataSize) {
            if (address == 0) bank++;
            setMemoryBank(devAddr,bank,0,0);
            writeByte(devAddr, MPU6050_RA_MEM_START_ADDR, address);
        }
    }
    if (verify==1) free(verifyBuffer);
    if (useProgMem==1) free(progBuffer);
    return 1;
}
uint8_t writeDMPConfigurationSet(uint8_t devAddr, const uint8_t *data, uint16_t dataSize, uint8_t useProgMem) {
    uint8_t *progBuffer = 0;
	uint8_t success, special;
    uint16_t i, j;
    if (useProgMem==1) {
        progBuffer = (uint8_t *)malloc(8); // assume 8-byte blocks, realloc later if necessary
    }

    // config set data is a long string of blocks with the following structure:
    // [bank] [offset] [length] [byte[0], byte[1], ..., byte[length]]
    uint8_t bank, offset, length;
    for (i = 0; i < dataSize;) {
        if (useProgMem==1) {
            bank = pgm_read_byte(data + i++);
            offset = pgm_read_byte(data + i++);
            length = pgm_read_byte(data + i++);
        } else {
            bank = data[i++];
            offset = data[i++];
            length = data[i++];
        }

        // write data or perform special action
        if (length > 0) {
            // regular block of data to write

            if (useProgMem==1) {
                if (sizeof(progBuffer) < length) progBuffer = (uint8_t *)realloc(progBuffer, length);
                for (j = 0; j < length; j++) progBuffer[j] = pgm_read_byte(data + i + j);
            } else {
                progBuffer = (uint8_t *)data + i;
            }
            success = writeMemoryBlock(devAddr, progBuffer, length, bank, offset, 1, 0);
            i += length;
        } else {
            // special instruction
            // NOTE: this kind of behavior (what and when to do certain things)
            // is totally undocumented. This code is in here based on observed
            // behavior only, and exactly why (or even whether) it has to be here
            // is anybody's guess for now.
            if (useProgMem==1) {
                special = pgm_read_byte(data + i++);
            } else {
                special = data[i++];
            }

            if (special == 0x01) {
                // enable DMP-related interrupts
                writeByte(devAddr, MPU6050_RA_INT_ENABLE, 0x32);  // single operation

                success = 1;
            } else {
                // unknown special command
                success = 0;
            }
        }
        
        if (success==0) {
            if (useProgMem==1) free(progBuffer);
            return 0; // uh oh
        }
    }
    if (useProgMem==1) free(progBuffer);
    return 1;
}
uint8_t writeProgMemoryBlock(uint8_t devAddr, const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, uint8_t verify) {
    return writeMemoryBlock(devAddr, data, dataSize, bank, address, verify, 1);
}
uint8_t writeProgDMPConfigurationSet(uint8_t devAddr, const uint8_t *data, uint16_t dataSize) {
    return writeDMPConfigurationSet(devAddr, data, dataSize, 1);
}
uint16_t getFIFOCount(uint8_t devAddr) {
    uint8_t buffer[10];
    readBytes(devAddr, MPU6050_RA_FIFO_COUNTH, 2, buffer,0);
    return (((uint16_t)buffer[0]) << 8) | buffer[1];
}
void getFIFOBytes(uint8_t devAddr, uint8_t *data, uint8_t length) {
    if(length > 0){
        readBytes(devAddr, MPU6050_RA_FIFO_R_W, length, data,0);
    } else {
    	*data = 0;
    }
}
uint8_t getIntStatus(uint8_t devAddr) {
    uint8_t buffer[8];
    readByte(devAddr, MPU6050_RA_INT_STATUS, buffer,0);
    return buffer[0];
}






/* [] END OF FILE */
