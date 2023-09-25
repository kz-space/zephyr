/**************************************************************************//**
 * @file     type.h
 * @brief    Define types
 *           
 * @version  V1.0.0
 * @date     02. July 2021
 ******************************************************************************/
 
#ifndef TYPE_H
#define TYPE_H
//===========================================================================//
//===================== General Define ======================================//
//===========================================================================//
//-- Constant Define -------------------------------------------

//#undef ENABLE
//#undef DISABLE
//#define ENABLE                          1
//#define DISABLE                         0
//
//#undef  NULL
//#define NULL                            0
//#undef ON
//#undef OFF
//#define ON                              1
//#define OFF                             0
//
//#undef  TRUE
//#undef  FALSE
//#define TRUE                            1
//#define FALSE                           0

//-- Macro Define -----------------------------------------------
#define mCLRBIT(reg,bit_no)              ((reg) &= (~(0x01UL<<bit_no)))
#define mCLR2BIT(reg,bit_no)             ((reg) &= (~(0x03UL<<bit_no)))
#define mCLRBIT31(reg)                   mCLRBIT(reg,31)
#define mCLRBIT30(reg)                   mCLRBIT(reg,30)
#define mCLRBIT29(reg)                   mCLRBIT(reg,29)
#define mCLRBIT28(reg)                   mCLRBIT(reg,28)
#define mCLRBIT27(reg)                   mCLRBIT(reg,27)
#define mCLRBIT26(reg)                   mCLRBIT(reg,26)
#define mCLRBIT25(reg)                   mCLRBIT(reg,25)
#define mCLRBIT24(reg)                   mCLRBIT(reg,24)
#define mCLRBIT23(reg)                   mCLRBIT(reg,23)
#define mCLRBIT22(reg)                   mCLRBIT(reg,22)
#define mCLRBIT21(reg)                   mCLRBIT(reg,21)
#define mCLRBIT20(reg)                   mCLRBIT(reg,20)
#define mCLRBIT19(reg)                   mCLRBIT(reg,19)
#define mCLRBIT18(reg)                   mCLRBIT(reg,18)
#define mCLRBIT17(reg)                   mCLRBIT(reg,17)
#define mCLRBIT16(reg)                   mCLRBIT(reg,16)
#define mCLRBIT15(reg)                   mCLRBIT(reg,15)
#define mCLRBIT14(reg)                   mCLRBIT(reg,14)
#define mCLRBIT13(reg)                   mCLRBIT(reg,13)
#define mCLRBIT12(reg)                   mCLRBIT(reg,12)
#define mCLRBIT11(reg)                   mCLRBIT(reg,11)
#define mCLRBIT10(reg)                   mCLRBIT(reg,10)
#define mCLRBIT9(reg)                    mCLRBIT(reg,9)
#define mCLRBIT8(reg)                    mCLRBIT(reg,8)
#define mCLRBIT7(reg)                    mCLRBIT(reg,7)
#define mCLRBIT6(reg)                    mCLRBIT(reg,6)
#define mCLRBIT5(reg)                    mCLRBIT(reg,5)
#define mCLRBIT4(reg)                    mCLRBIT(reg,4)
#define mCLRBIT3(reg)                    mCLRBIT(reg,3)
#define mCLRBIT2(reg)                    mCLRBIT(reg,2)
#define mCLRBIT1(reg)                    mCLRBIT(reg,1)
#define mCLRBIT0(reg)                    mCLRBIT(reg,0)

#define mSETBIT(reg,bit_no)              ((reg) |= (0x01UL<<bit_no))
#define mSET2BIT(reg,bit_v,bit_no)       ((reg) |= ((bit_v&0x03UL)<<bit_no))
#define mSETBIT31(reg)                   mSETBIT(reg,31)
#define mSETBIT30(reg)                   mSETBIT(reg,30)
#define mSETBIT29(reg)                   mSETBIT(reg,29)
#define mSETBIT28(reg)                   mSETBIT(reg,28)
#define mSETBIT27(reg)                   mSETBIT(reg,27)
#define mSETBIT26(reg)                   mSETBIT(reg,26)
#define mSETBIT25(reg)                   mSETBIT(reg,25)
#define mSETBIT24(reg)                   mSETBIT(reg,24)
#define mSETBIT23(reg)                   mSETBIT(reg,23)
#define mSETBIT22(reg)                   mSETBIT(reg,22)
#define mSETBIT21(reg)                   mSETBIT(reg,21)
#define mSETBIT20(reg)                   mSETBIT(reg,20)
#define mSETBIT19(reg)                   mSETBIT(reg,19)
#define mSETBIT18(reg)                   mSETBIT(reg,18)
#define mSETBIT17(reg)                   mSETBIT(reg,17)
#define mSETBIT16(reg)                   mSETBIT(reg,16)
#define mSETBIT15(reg)                   mSETBIT(reg,15)
#define mSETBIT14(reg)                   mSETBIT(reg,14)
#define mSETBIT13(reg)                   mSETBIT(reg,13)
#define mSETBIT12(reg)                   mSETBIT(reg,12)
#define mSETBIT11(reg)                   mSETBIT(reg,11)
#define mSETBIT10(reg)                   mSETBIT(reg,10)
#define mSETBIT9(reg)                    mSETBIT(reg,9)
#define mSETBIT8(reg)                    mSETBIT(reg,8)
#define mSETBIT7(reg)                    mSETBIT(reg,7)
#define mSETBIT6(reg)                    mSETBIT(reg,6)
#define mSETBIT5(reg)                    mSETBIT(reg,5)
#define mSETBIT4(reg)                    mSETBIT(reg,4)
#define mSETBIT3(reg)                    mSETBIT(reg,3)
#define mSETBIT2(reg)                    mSETBIT(reg,2)
#define mSETBIT1(reg)                    mSETBIT(reg,1)
#define mSETBIT0(reg)                    mSETBIT(reg,0)

#define mCPLBIT(reg,bit_no)              ((reg) ^= (0x01UL<<bit_no))
#define mCPLBIT31(reg)                   mCPLBIT(reg,31)
#define mCPLBIT30(reg)                   mCPLBIT(reg,30)
#define mCPLBIT29(reg)                   mCPLBIT(reg,29)
#define mCPLBIT28(reg)                   mCPLBIT(reg,28)
#define mCPLBIT27(reg)                   mCPLBIT(reg,27)
#define mCPLBIT26(reg)                   mCPLBIT(reg,26)
#define mCPLBIT25(reg)                   mCPLBIT(reg,25)
#define mCPLBIT24(reg)                   mCPLBIT(reg,24)
#define mCPLBIT23(reg)                   mCPLBIT(reg,23)
#define mCPLBIT22(reg)                   mCPLBIT(reg,22)
#define mCPLBIT21(reg)                   mCPLBIT(reg,21)
#define mCPLBIT20(reg)                   mCPLBIT(reg,20)
#define mCPLBIT19(reg)                   mCPLBIT(reg,19)
#define mCPLBIT18(reg)                   mCPLBIT(reg,18)
#define mCPLBIT17(reg)                   mCPLBIT(reg,17)
#define mCPLBIT16(reg)                   mCPLBIT(reg,16)
#define mCPLBIT15(reg)                   mCPLBIT(reg,15)
#define mCPLBIT14(reg)                   mCPLBIT(reg,14)
#define mCPLBIT13(reg)                   mCPLBIT(reg,13)
#define mCPLBIT12(reg)                   mCPLBIT(reg,12)
#define mCPLBIT11(reg)                   mCPLBIT(reg,11)
#define mCPLBIT10(reg)                   mCPLBIT(reg,10)
#define mCPLBIT9(reg)                    mCPLBIT(reg,9)
#define mCPLBIT8(reg)                    mCPLBIT(reg,8)
#define mCPLBIT7(reg)                    mCPLBIT(reg,7)
#define mCPLBIT6(reg)                    mCPLBIT(reg,6)
#define mCPLBIT5(reg)                    mCPLBIT(reg,5)
#define mCPLBIT4(reg)                    mCPLBIT(reg,4)
#define mCPLBIT3(reg)                    mCPLBIT(reg,3)
#define mCPLBIT2(reg)                    mCPLBIT(reg,2)
#define mCPLBIT1(reg)                    mCPLBIT(reg,1)
#define mCPLBIT0(reg)                    mCPLBIT(reg,0)

#define mGETBIT(reg,bit_no)              (reg & (0x01UL<<bit_no))
#define mGET2BIT(reg,bit_no)             (reg & (0x03UL<<bit_no))
#define mGETBIT31(reg)                   mGETBIT(reg,31)
#define mGETBIT30(reg)                   mGETBIT(reg,30)
#define mGETBIT29(reg)                   mGETBIT(reg,29)
#define mGETBIT28(reg)                   mGETBIT(reg,28)
#define mGETBIT27(reg)                   mGETBIT(reg,27)
#define mGETBIT26(reg)                   mGETBIT(reg,26)
#define mGETBIT25(reg)                   mGETBIT(reg,25)
#define mGETBIT24(reg)                   mGETBIT(reg,24)
#define mGETBIT23(reg)                   mGETBIT(reg,23)
#define mGETBIT22(reg)                   mGETBIT(reg,22)
#define mGETBIT21(reg)                   mGETBIT(reg,21)
#define mGETBIT20(reg)                   mGETBIT(reg,20)
#define mGETBIT19(reg)                   mGETBIT(reg,19)
#define mGETBIT18(reg)                   mGETBIT(reg,18)
#define mGETBIT17(reg)                   mGETBIT(reg,17)
#define mGETBIT16(reg)                   mGETBIT(reg,16)
#define mGETBIT15(reg)                   mGETBIT(reg,15)
#define mGETBIT14(reg)                   mGETBIT(reg,14)
#define mGETBIT13(reg)                   mGETBIT(reg,13)
#define mGETBIT12(reg)                   mGETBIT(reg,12)
#define mGETBIT11(reg)                   mGETBIT(reg,11)
#define mGETBIT10(reg)                   mGETBIT(reg,10)
#define mGETBIT9(reg)                    mGETBIT(reg,9)
#define mGETBIT8(reg)                    mGETBIT(reg,8)
#define mGETBIT7(reg)                    mGETBIT(reg,7)
#define mGETBIT6(reg)                    mGETBIT(reg,6)
#define mGETBIT5(reg)                    mGETBIT(reg,5)
#define mGETBIT4(reg)                    mGETBIT(reg,4)
#define mGETBIT3(reg)                    mGETBIT(reg,3)
#define mGETBIT2(reg)                    mGETBIT(reg,2)
#define mGETBIT1(reg)                    mGETBIT(reg,1)
#define mGETBIT0(reg)                    mGETBIT(reg,0) 

#define mWRITE2BIT(reg,bit_v,bit_no)    do { mCLR2BIT(reg,bit_no);mSET2BIT(reg,bit_v,bit_no); } while(0)
#define mWRITEBIT(reg,bit_v,bit_no)     ((bit_v==0)?(mCLRBIT(reg,bit_no)):(mSETBIT(reg,bit_no)))
#define mREAD2BIT(reg,bit_no)           (mGET2BIT(reg,bit_no)>>bit_no)
#define mREADBIT(reg,bit_no)            (mGETBIT(reg,bit_no)>>bit_no)

#define mOR_DAT(reg,or_bit)             ((reg)|= or_bit)
#define mAND_DAT(reg,and_bit)           ((reg)&= and_bit)
#define mXOR_DAT(reg,xor_bit)           ((reg)^= xor_bit)
#define mSET_DAT(reg,dat)               ((reg)=dat)
#define mHIGH_BYTE(a)                   (a>>8)
#define mLOW_BYTE(a)                    (a&0xFF)

#define mSWAP_WORD_DAT(a)               (((a>>8)&0xFF)|((a<<8)&0xFF00))
#define mSWAP_DWORD_DATA(a)             ((a>65535)?(((a<<24)&0xFF000000)|((a<<8)&0x00FF0000)|((a>>8)&0x0000FF00)|((a>>24)&0x0000000FF)):\
                                                 ((((a>>8)&0xFF)|((a<<8)&0xFF00))*65536))
#define mMASK_SET_DAT(reg,mask_bit,set_bit)       ((reg)=(reg&mask_bit)|set_bit)

#define mMin(ValA, ValB)        (((ValA)<(ValB))?(ValA):(ValB))
#define mMax(ValA, ValB)        (((ValA)>(ValB))?(ValA):(ValA))

typedef void         (*FUNCTION_PTR_V_V)  (void);
typedef void         (*FUNCTION_PTR_V_B)  (unsigned char);
typedef void         (*FUNCTION_PTR_V_B_B)(unsigned char,unsigned char);

typedef unsigned char(*FUNCTION_PTR_B_B)  (unsigned char);

typedef unsigned int (*FUNCTION_PTR_W_V)  (void);
typedef unsigned int (*FUNCTION_PTR_W_B)  (unsigned char);

typedef void         (*FUNCTION_PTR_V_W_B)(unsigned short, unsigned char);

//
////-- Struction Define --------------------------------------------
//typedef union _BIT_8
//{
//    unsigned char Byte;
//    struct
//    {
//        unsigned char bit0 : 1;
//        unsigned char bit1 : 1;
//        unsigned char bit2 : 1;
//        unsigned char bit3 : 1;
//        unsigned char bit4 : 1;
//        unsigned char bit5 : 1;
//        unsigned char bit6 : 1;
//        unsigned char bit7 : 1;
//    }Bit;
//} __attribute__((packed)) BIT_8;
//
//typedef union _BIT_16
//{
//    unsigned short  Word;
//    unsigned char Byte[2];
//    struct
//    {
//        unsigned char bit0 : 1;
//        unsigned char bit1 : 1;
//        unsigned char bit2 : 1;
//        unsigned char bit3 : 1;
//        unsigned char bit4 : 1;
//        unsigned char bit5 : 1;
//        unsigned char bit6 : 1;
//        unsigned char bit7 : 1;
//        unsigned char bit8 : 1;
//        unsigned char bit9 : 1;
//        unsigned char bit10: 1;
//        unsigned char bit11: 1;
//        unsigned char bit12: 1;
//        unsigned char bit13: 1;
//        unsigned char bit14: 1;
//        unsigned char bit15: 1;
//    }Bit;
//} __attribute__((packed)) BIT_16;
//
//typedef unsigned char BOOL;
//typedef unsigned char BYTE;
//typedef unsigned char UINT8;
//typedef unsigned short UINT16;
//typedef unsigned long UINT32;

/*
typedef union _BYTE_2
{
    unsigned short Word;
    unsigned char Byte[2];
}BYTE_2;
*/
#endif  //TYPE_H
