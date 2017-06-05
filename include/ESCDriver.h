#ifndef ESCDRIVER_H
#define ESCDRIVER_H


class ESCDriver
{
    public:
        /****** Value definition ******/
        // minimum
        const static int DELAY_TIME = 0x199;    // delay 10%
        const static int DC_MIN = 0x266;        // DELAY_TIME + duty_cycle
        const static int DC_MAX = 0x333;
        const static int DC_IDLE = 0x272;

        ESCDriver();

        void setModeNormal();
        void setModeSleep();
        void setFrequency(int frequency);
        void lock();
        void unLock();
        void setESC1(int dc);
        void setESC2(int dc);
        void setESC3(int dc);
        void setESC4(int dc);
        void setESCALL(int dc);
        void setESCALL(int esc1, int esc2, int esc3, int esc4);

        virtual ~ESCDriver();
    protected:
    private:
        // device address
        int i2cAddress = 0x40;
        int i2cHandler;

        /****** Address definition ******/
        // prescale
        const static int PRESCALE_REG_ADDR = 0xFE;

        // mode1
        const static int MODE1_REG_ADDR = 0x00;

        // ESCALL
        const static int ESCALL_ON_ADDR_L = 0XFA;
        const static int ESCALL_ON_ADDR_H = 0XFB;
        const static int ESCALL_OFF_ADDR_L = 0XFC;
        const static int ESCALL_OFF_ADDR_H = 0XFD;

        // ESC1
        const static int ESC1_ON_ADDR_L = 0x06;
        const static int ESC1_ON_ADDR_H = 0x07;
        const static int ESC1_OFF_ADDR_L = 0x08;
        const static int ESC1_OFF_ADDR_H = 0x09;
        //ESC2
        const static int ESC2_ON_ADDR_L = 0x0A;
        const static int ESC2_ON_ADDR_H = 0x0B;
        const static int ESC2_OFF_ADDR_L = 0x0C;
        const static int ESC2_OFF_ADDR_H = 0x0D;
        //ESC3
        const static int ESC3_ON_ADDR_L = 0x0E;
        const static int ESC3_ON_ADDR_H = 0x0F;
        const static int ESC3_OFF_ADDR_L = 0x10;
        const static int ESC3_OFF_ADDR_H = 0x11;
        //ESC4
        const static int ESC4_ON_ADDR_L = 0x12;
        const static int ESC4_ON_ADDR_H = 0x13;
        const static int ESC4_OFF_ADDR_L = 0x14;
        const static int ESC4_OFF_ADDR_H = 0x15;

        int getLow(int word){
            return word & 0x00FF;
        }

        int getHigh(int word){
            return word >> 8;
        }
};

#endif // ESCDRIVER_H
