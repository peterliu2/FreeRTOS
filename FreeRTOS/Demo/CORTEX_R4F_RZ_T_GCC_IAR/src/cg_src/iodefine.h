/********************************************************************************/
/*                                                                              */
/* Device     : RZ/T1                                                           */
/* File Name  : iodefine.h                                                      */
/* Abstract   : Definition of I/O Register.                                     */
/* History    : V0.8  (2015-02-23)  [Hardware Manual Revision : 0.8]            */
/* Note       : This is a typical example.                                      */
/*                                                                              */
/*  Copyright(c) 2015 Renesas Electronics Corp. ,All Rights Reserved.           */
/*                                                                              */
/********************************************************************************/
#ifndef __RZT1___IODEFINE_HEADER__
#define __RZT1___IODEFINE_HEADER__
struct st_bsc {
    union {
        unsigned long LONG;
        struct {
            unsigned long :9;
            unsigned long BSZ:2;
            unsigned long :1;
            unsigned long TYPE:3;
            unsigned long :1;
            unsigned long IWRRS:3;
            unsigned long IWRRD:3;
            unsigned long IWRWS:3;
            unsigned long IWRWD:3;
            unsigned long IWW:3;
            unsigned long :1;
        } BIT;
    } CS0BCR;
    union {
        unsigned long LONG;
        struct {
            unsigned long :9;
            unsigned long BSZ:2;
            unsigned long :1;
            unsigned long TYPE:3;
            unsigned long :1;
            unsigned long IWRRS:3;
            unsigned long IWRRD:3;
            unsigned long IWRWS:3;
            unsigned long IWRWD:3;
            unsigned long IWW:3;
            unsigned long :1;
        } BIT;
    } CS1BCR;
    union {
        unsigned long LONG;
        struct {
            unsigned long :9;
            unsigned long BSZ:2;
            unsigned long :1;
            unsigned long TYPE:3;
            unsigned long :1;
            unsigned long IWRRS:3;
            unsigned long IWRRD:3;
            unsigned long IWRWS:3;
            unsigned long IWRWD:3;
            unsigned long IWW:3;
            unsigned long :1;
        } BIT;
    } CS2BCR;
    union {
        unsigned long LONG;
        struct {
            unsigned long :9;
            unsigned long BSZ:2;
            unsigned long :1;
            unsigned long TYPE:3;
            unsigned long :1;
            unsigned long IWRRS:3;
            unsigned long IWRRD:3;
            unsigned long IWRWS:3;
            unsigned long IWRWD:3;
            unsigned long IWW:3;
            unsigned long :1;
        } BIT;
    } CS3BCR;
    union {
        unsigned long LONG;
        struct {
            unsigned long :9;
            unsigned long BSZ:2;
            unsigned long :1;
            unsigned long TYPE:3;
            unsigned long :1;
            unsigned long IWRRS:3;
            unsigned long IWRRD:3;
            unsigned long IWRWS:3;
            unsigned long IWRWD:3;
            unsigned long IWW:3;
            unsigned long :1;
        } BIT;
    } CS4BCR;
    union {
        unsigned long LONG;
        struct {
            unsigned long :9;
            unsigned long BSZ:2;
            unsigned long :1;
            unsigned long TYPE:3;
            unsigned long :1;
            unsigned long IWRRS:3;
            unsigned long IWRRD:3;
            unsigned long IWRWS:3;
            unsigned long IWRWD:3;
            unsigned long IWW:3;
            unsigned long :1;
        } BIT;
    } CS5BCR;
    char           wk0[12];
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long HW:2;
                unsigned long :4;
                unsigned long WM:1;
                unsigned long WR:4;
                unsigned long SW:2;
                unsigned long :7;
                unsigned long BAS:1;
                unsigned long :11;
            } BIT;
        } CS0WCR_0;
        union {
            unsigned long LONG;
            struct {
                unsigned long :6;
                unsigned long WM:1;
                unsigned long W:4;
                unsigned long :5;
                unsigned long BW:2;
                unsigned long :2;
                unsigned long BST:2;
                unsigned long :10;
            } BIT;
        } CS0WCR_1;
        union {
            unsigned long LONG;
            struct {
                unsigned long :6;
                unsigned long WM:1;
                unsigned long W:4;
                unsigned long :5;
                unsigned long BW:2;
                unsigned long :14;
            } BIT;
        } CS0WCR_2;
    } CS0WCR;
    union {
        unsigned long LONG;
        struct {
            unsigned long HW:2;
            unsigned long :4;
            unsigned long WM:1;
            unsigned long WR:4;
            unsigned long SW:2;
            unsigned long :3;
            unsigned long WW:3;
            unsigned long :1;
            unsigned long BAS:1;
            unsigned long :11;
        } BIT;
    } CS1WCR;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long :6;
                unsigned long WM:1;
                unsigned long WR:4;
                unsigned long :9;
                unsigned long BAS:1;
                unsigned long :11;
            } BIT;
        } CS2WCR_0;
        union {
            unsigned long LONG;
            struct {
                unsigned long :7;
                unsigned long A2CL:2;
                unsigned long :23;
            } BIT;
        } CS2WCR_1;
    } CS2WCR;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long :6;
                unsigned long WM:1;
                unsigned long WR:4;
                unsigned long :9;
                unsigned long BAS:1;
                unsigned long :11;
            } BIT;
        } CS3WCR_0;
        union {
            unsigned long LONG;
            struct {
                unsigned long WTRC:2;
                unsigned long :1;
                unsigned long TRWL:2;
                unsigned long :2;
                unsigned long A3CL:2;
                unsigned long :1;
                unsigned long WTRCD:2;
                unsigned long :1;
                unsigned long WTRP:2;
                unsigned long :17;
            } BIT;
        } CS3WCR_1;
    } CS3WCR;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long HW:2;
                unsigned long :4;
                unsigned long WM:1;
                unsigned long WR:4;
                unsigned long SW:2;
                unsigned long :3;
                unsigned long WW:3;
                unsigned long :1;
                unsigned long BAS:1;
                unsigned long :11;
            } BIT;
        } CS4WCR_0;
        union {
            unsigned long LONG;
            struct {
                unsigned long HW:2;
                unsigned long :4;
                unsigned long WM:1;
                unsigned long W:4;
                unsigned long SW:2;
                unsigned long :3;
                unsigned long BW:2;
                unsigned long :2;
                unsigned long BST:2;
                unsigned long :10;
            } BIT;
        } CS4WCR_1;
    } CS4WCR;
    union {
        unsigned long LONG;
        struct {
            unsigned long HW:2;
            unsigned long :4;
            unsigned long WM:1;
            unsigned long WR:4;
            unsigned long SW:2;
            unsigned long :3;
            unsigned long WW:3;
            unsigned long :1;
            unsigned long MPXWBAS:1;
            unsigned long SZSEL:1;
            unsigned long :10;
        } BIT;
    } CS5WCR;
    char           wk1[12];
    union {
        unsigned long LONG;
        struct {
            unsigned long A3COL:2;
            unsigned long :1;
            unsigned long A3ROW:2;
            unsigned long :3;
            unsigned long BACTV:1;
            unsigned long PDOWN:1;
            unsigned long RMODE:1;
            unsigned long RFSH:1;
            unsigned long :1;
            unsigned long DEEP:1;
            unsigned long :2;
            unsigned long A2COL:2;
            unsigned long :1;
            unsigned long A2ROW:2;
            unsigned long :11;
        } BIT;
    } SDCR;
    union {
        unsigned long LONG;
    } RTCSR;
    unsigned long  RTCNT;
    unsigned long  RTCOR;
    char           wk2[4];
    unsigned long  TOSCOR0;
    unsigned long  TOSCOR1;
    unsigned long  TOSCOR2;
    unsigned long  TOSCOR3;
    unsigned long  TOSCOR4;
    unsigned long  TOSCOR5;
    char           wk3[8];
    union {
        unsigned long LONG;
        struct {
            unsigned long CS0TOSTF:1;
            unsigned long CS1TOSTF:1;
            unsigned long CS2TOSTF:1;
            unsigned long CS3TOSTF:1;
            unsigned long CS4TOSTF:1;
            unsigned long CS5TOSTF:1;
            unsigned long :26;
        } BIT;
    } TOSTR;
    union {
        unsigned long LONG;
        struct {
            unsigned long CS0TOEN:1;
            unsigned long CS1TOEN:1;
            unsigned long CS2TOEN:1;
            unsigned long CS3TOEN:1;
            unsigned long CS4TOEN:1;
            unsigned long CS5TOEN:1;
            unsigned long :26;
        } BIT;
    } TOENR;
    char           wk4[2948];
    union {
        unsigned long LONG;
    } CKIOSET;
    char           wk5[236];
    union {
        unsigned char BYTE;
    } CKIOKEY;
};

struct st_clma0 {
    union {
        unsigned char BYTE;
    } CLMA0CTL0;
    char           wk0[7];
    union {
        unsigned short WORD;
        struct {
            unsigned short CLMAnCMPL:12;
            unsigned short :4;
        } BIT;
    } CLMA0CMPL;
    char           wk1[2];
    union {
        unsigned short WORD;
        struct {
            unsigned short CLMAnCMPH:12;
            unsigned short :4;
        } BIT;
    } CLMA0CMPH;
    char           wk2[2];
    union {
        unsigned char BYTE;
    } CLMA0PCMD;
    char           wk3[3];
    union {
        unsigned char BYTE;
        struct {
            unsigned char CLMAnPRERR:1;
            unsigned char :7;
        } BIT;
    } CLMA0PS;
};

struct st_clma1 {
    union {
        unsigned char BYTE;
    } CLMA1CTL0;
    char           wk0[7];
    union {
        unsigned short WORD;
        struct {
            unsigned short CLMAnCMPL:12;
            unsigned short :4;
        } BIT;
    } CLMA1CMPL;
    char           wk1[2];
    union {
        unsigned short WORD;
        struct {
            unsigned short CLMAnCMPH:12;
            unsigned short :4;
        } BIT;
    } CLMA1CMPH;
    char           wk2[2];
    union {
        unsigned char BYTE;
    } CLMA1PCMD;
    char           wk3[3];
    union {
        unsigned char BYTE;
        struct {
            unsigned char CLMAnPRERR:1;
            unsigned char :7;
        } BIT;
    } CLMA1PS;
};

struct st_clma2 {
    union {
        unsigned char BYTE;
    } CLMA2CTL0;
    char           wk0[7];
    union {
        unsigned short WORD;
        struct {
            unsigned short CLMAnCMPL:12;
            unsigned short :4;
        } BIT;
    } CLMA2CMPL;
    char           wk1[2];
    union {
        unsigned short WORD;
        struct {
            unsigned short CLMAnCMPH:12;
            unsigned short :4;
        } BIT;
    } CLMA2CMPH;
    char           wk2[2];
    union {
        unsigned char BYTE;
    } CLMA2PCMD;
    char           wk3[3];
    union {
        unsigned char BYTE;
        struct {
            unsigned char CLMAnPRERR:1;
            unsigned char :7;
        } BIT;
    } CLMA2PS;
};

struct st_cmt {
    union {
        unsigned short WORD;
        struct {
            unsigned short STR0:1;
            unsigned short STR1:1;
            unsigned short :14;
        } BIT;
    } CMSTR0;
    char           wk0[30];
    union {
        unsigned short WORD;
        struct {
            unsigned short STR2:1;
            unsigned short STR3:1;
            unsigned short :14;
        } BIT;
    } CMSTR1;
    char           wk1[30];
    union {
        unsigned short WORD;
        struct {
            unsigned short STR4:1;
            unsigned short STR5:1;
            unsigned short :14;
        } BIT;
    } CMSTR2;
};

struct st_cmt0 {
    union {
        unsigned short WORD;
        struct {
            unsigned short CKS:2;
            unsigned short :4;
            unsigned short CMIE:1;
            unsigned short :9;
        } BIT;
    } CMCR;
    unsigned short CMCNT;
    unsigned short CMCOR;
};

struct st_cmtw {
    union {
        unsigned long LONG;
        struct {
            unsigned long NF0EN:1;
            unsigned long NF1EN:1;
            unsigned long NFCS0:2;
            unsigned long :28;
        } BIT;
    } NFCR0;
    union {
        unsigned long LONG;
        struct {
            unsigned long NF2EN:1;
            unsigned long NF3EN:1;
            unsigned long NFCS1:2;
            unsigned long :28;
        } BIT;
    } NFCR1;
    char           wk0[8];
    union {
        unsigned long LONG;
        struct {
            unsigned long DMERSL:3;
            unsigned long :29;
        } BIT;
    } ECDMESLR;
};

struct st_cmtw0 {
    union {
        unsigned short WORD;
        struct {
            unsigned short STR:1;
            unsigned short :15;
        } BIT;
    } CMWSTR;
    char           wk0[2];
    union {
        unsigned short WORD;
        struct {
            unsigned short CKS:2;
            unsigned short :1;
            unsigned short CMWIE:1;
            unsigned short IC0IE:1;
            unsigned short IC1IE:1;
            unsigned short OC0IE:1;
            unsigned short OC1IE:1;
            unsigned short :1;
            unsigned short CMS:1;
            unsigned short :3;
            unsigned short CCLR:3;
        } BIT;
    } CMWCR;
    char           wk1[2];
    union {
        unsigned short WORD;
        struct {
            unsigned short IC0:2;
            unsigned short IC1:2;
            unsigned short IC0E:1;
            unsigned short IC1E:1;
            unsigned short :2;
            unsigned short OC0:2;
            unsigned short OC1:2;
            unsigned short OC0E:1;
            unsigned short OC1E:1;
            unsigned short :1;
            unsigned short CMWE:1;
        } BIT;
    } CMWIOR;
    char           wk2[6];
    unsigned long  CMWCNT;
    unsigned long  CMWCOR;
    unsigned long  CMWICR0;
    unsigned long  CMWICR1;
    unsigned long  CMWOCR0;
    unsigned long  CMWOCR1;
};

struct st_crc {
    union {
        unsigned long LONG;
        struct {
            unsigned long DCRA0CIN:32;
        } BIT;
    } CRCDIR;
    union {
        unsigned long LONG;
        struct {
            unsigned long DCRA0COUT:32;
        } BIT;
    } CRCDOR;
    char           wk0[24];
    union {
        unsigned char BYTE;
        struct {
            unsigned char DCRA0POL:2;
            unsigned char :2;
            unsigned char DCRA0ISZ:2;
            unsigned char :2;
        } BIT;
    } CRCCR;
};

struct st_dma0 {
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC0_N0SA_0_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC0_N0SA_0_W;
    } N0SA_0;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC0_N0DA_0;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC0_N0TB_0;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC0_N1SA_0_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC0_N1SA_0_W;
    } N1SA_0;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC0_N1DA_0;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC0_N1TB_0;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRSA:32;
        } BIT;
    } DMAC0_CRSA_0;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRDA:32;
        } BIT;
    } DMAC0_CRDA_0;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRTB:32;
        } BIT;
    } DMAC0_CRTB_0;
    union {
        unsigned long LONG;
        struct {
            unsigned long EN:1;
            unsigned long RQST:1;
            unsigned long TACT:1;
            unsigned long SUS:1;
            unsigned long ER:1;
            unsigned long END:1;
            unsigned long :1;
            unsigned long SR:1;
            unsigned long DL:1;
            unsigned long DW:1;
            unsigned long DER:1;
            unsigned long MODE:1;
            unsigned long :4;
            unsigned long INTM:1;
            unsigned long DMARQM:1;
            unsigned long SWPRQ:1;
            unsigned long :5;
            unsigned long DNUM:8;
        } BIT;
    } DMAC0_CHSTAT_0;
    union {
        unsigned long LONG;
        struct {
            unsigned long SETEN:1;
            unsigned long CLREN:1;
            unsigned long :1;
            unsigned long SWRST:1;
            unsigned long CLRRQ:1;
            unsigned long CLREND:1;
            unsigned long :1;
            unsigned long CLRDE:1;
            unsigned long SETSUS:1;
            unsigned long CLRSUS:1;
            unsigned long :2;
            unsigned long SETREN:1;
            unsigned long :1;
            unsigned long SETSSWPRQ:1;
            unsigned long :1;
            unsigned long SETINTM:1;
            unsigned long CLRINTM:1;
            unsigned long SETDMARQM:1;
            unsigned long CLRDMARQM:1;
            unsigned long :12;
        } BIT;
    } DMAC0_CHCTRL_0;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEL:3;
            unsigned long REQD:1;
            unsigned long LOEN:1;
            unsigned long HIEN:1;
            unsigned long LVL:1;
            unsigned long :1;
            unsigned long AM:3;
            unsigned long DRRP:1;
            unsigned long SDS:4;
            unsigned long DDS:4;
            unsigned long SAD:1;
            unsigned long DAD:1;
            unsigned long TM:1;
            unsigned long WONLY:1;
            unsigned long DEM:1;
            unsigned long :1;
            unsigned long DIM:1;
            unsigned long SBE:1;
            unsigned long RSEL:1;
            unsigned long RSW:1;
            unsigned long REN:1;
            unsigned long DMS:1;
        } BIT;
    } DMAC0_CHCFG_0;
    union {
        unsigned long LONG;
        struct {
            unsigned long ITVL:16;
            unsigned long :16;
        } BIT;
    } DMAC0_CHITVL_0;
    char           wk0[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long NXLA:32;
        } BIT;
    } DMAC0_NXLA_0;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRLA:32;
        } BIT;
    } DMAC0_CRLA_0;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC0_N0SA_1_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC0_N0SA_1_W;
    } N0SA_1;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC0_N0DA_1;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC0_N0TB_1;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC0_N1SA_1_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC0_N1SA_1_W;
    } N1SA_1;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC0_N1DA_1;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC0_N1TB_1;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRSA:32;
        } BIT;
    } DMAC0_CRSA_1;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRDA:32;
        } BIT;
    } DMAC0_CRDA_1;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRTB:32;
        } BIT;
    } DMAC0_CRTB_1;
    union {
        unsigned long LONG;
        struct {
            unsigned long EN:1;
            unsigned long RQST:1;
            unsigned long TACT:1;
            unsigned long SUS:1;
            unsigned long ER:1;
            unsigned long END:1;
            unsigned long :1;
            unsigned long SR:1;
            unsigned long DL:1;
            unsigned long DW:1;
            unsigned long DER:1;
            unsigned long MODE:1;
            unsigned long :4;
            unsigned long INTM:1;
            unsigned long DMARQM:1;
            unsigned long SWPRQ:1;
            unsigned long :5;
            unsigned long DNUM:8;
        } BIT;
    } DMAC0_CHSTAT_1;
    union {
        unsigned long LONG;
        struct {
            unsigned long SETEN:1;
            unsigned long CLREN:1;
            unsigned long :1;
            unsigned long SWRST:1;
            unsigned long CLRRQ:1;
            unsigned long CLREND:1;
            unsigned long :1;
            unsigned long CLRDE:1;
            unsigned long SETSUS:1;
            unsigned long CLRSUS:1;
            unsigned long :2;
            unsigned long SETREN:1;
            unsigned long :1;
            unsigned long SETSSWPRQ:1;
            unsigned long :1;
            unsigned long SETINTM:1;
            unsigned long CLRINTM:1;
            unsigned long SETDMARQM:1;
            unsigned long CLRDMARQM:1;
            unsigned long :12;
        } BIT;
    } DMAC0_CHCTRL_1;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEL:3;
            unsigned long REQD:1;
            unsigned long LOEN:1;
            unsigned long HIEN:1;
            unsigned long LVL:1;
            unsigned long :1;
            unsigned long AM:3;
            unsigned long DRRP:1;
            unsigned long SDS:4;
            unsigned long DDS:4;
            unsigned long SAD:1;
            unsigned long DAD:1;
            unsigned long TM:1;
            unsigned long WONLY:1;
            unsigned long DEM:1;
            unsigned long :1;
            unsigned long DIM:1;
            unsigned long SBE:1;
            unsigned long RSEL:1;
            unsigned long RSW:1;
            unsigned long REN:1;
            unsigned long DMS:1;
        } BIT;
    } DMAC0_CHCFG_1;
    union {
        unsigned long LONG;
        struct {
            unsigned long ITVL:16;
            unsigned long :16;
        } BIT;
    } DMAC0_CHITVL_1;
    char           wk1[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long NXLA:32;
        } BIT;
    } DMAC0_NXLA_1;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRLA:32;
        } BIT;
    } DMAC0_CRLA_1;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC0_N0SA_2_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC0_N0SA_2_W;
    } N0SA_2;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC0_N0DA_2;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC0_N0TB_2;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC0_N1SA_2_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC0_N1SA_2_W;
    } N1SA_2;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC0_N1DA_2;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC0_N1TB_2;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRSA:32;
        } BIT;
    } DMAC0_CRSA_2;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRDA:32;
        } BIT;
    } DMAC0_CRDA_2;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRTB:32;
        } BIT;
    } DMAC0_CRTB_2;
    union {
        unsigned long LONG;
        struct {
            unsigned long EN:1;
            unsigned long RQST:1;
            unsigned long TACT:1;
            unsigned long SUS:1;
            unsigned long ER:1;
            unsigned long END:1;
            unsigned long :1;
            unsigned long SR:1;
            unsigned long DL:1;
            unsigned long DW:1;
            unsigned long DER:1;
            unsigned long MODE:1;
            unsigned long :4;
            unsigned long INTM:1;
            unsigned long DMARQM:1;
            unsigned long SWPRQ:1;
            unsigned long :5;
            unsigned long DNUM:8;
        } BIT;
    } DMAC0_CHSTAT_2;
    union {
        unsigned long LONG;
        struct {
            unsigned long SETEN:1;
            unsigned long CLREN:1;
            unsigned long :1;
            unsigned long SWRST:1;
            unsigned long CLRRQ:1;
            unsigned long CLREND:1;
            unsigned long :1;
            unsigned long CLRDE:1;
            unsigned long SETSUS:1;
            unsigned long CLRSUS:1;
            unsigned long :2;
            unsigned long SETREN:1;
            unsigned long :1;
            unsigned long SETSSWPRQ:1;
            unsigned long :1;
            unsigned long SETINTM:1;
            unsigned long CLRINTM:1;
            unsigned long SETDMARQM:1;
            unsigned long CLRDMARQM:1;
            unsigned long :12;
        } BIT;
    } DMAC0_CHCTRL_2;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEL:3;
            unsigned long REQD:1;
            unsigned long LOEN:1;
            unsigned long HIEN:1;
            unsigned long LVL:1;
            unsigned long :1;
            unsigned long AM:3;
            unsigned long DRRP:1;
            unsigned long SDS:4;
            unsigned long DDS:4;
            unsigned long SAD:1;
            unsigned long DAD:1;
            unsigned long TM:1;
            unsigned long WONLY:1;
            unsigned long DEM:1;
            unsigned long :1;
            unsigned long DIM:1;
            unsigned long SBE:1;
            unsigned long RSEL:1;
            unsigned long RSW:1;
            unsigned long REN:1;
            unsigned long DMS:1;
        } BIT;
    } DMAC0_CHCFG_2;
    union {
        unsigned long LONG;
        struct {
            unsigned long ITVL:16;
            unsigned long :16;
        } BIT;
    } DMAC0_CHITVL_2;
    char           wk2[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long NXLA:32;
        } BIT;
    } DMAC0_NXLA_2;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRLA:32;
        } BIT;
    } DMAC0_CRLA_2;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC0_N0SA_3_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC0_N0SA_3_W;
    } N0SA_3;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC0_N0DA_3;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC0_N0TB_3;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC0_N1SA_3_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC0_N1SA_3_W;
    } N1SA_3;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC0_N1DA_3;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC0_N1TB_3;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRSA:32;
        } BIT;
    } DMAC0_CRSA_3;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRDA:32;
        } BIT;
    } DMAC0_CRDA_3;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRTB:32;
        } BIT;
    } DMAC0_CRTB_3;
    union {
        unsigned long LONG;
        struct {
            unsigned long EN:1;
            unsigned long RQST:1;
            unsigned long TACT:1;
            unsigned long SUS:1;
            unsigned long ER:1;
            unsigned long END:1;
            unsigned long :1;
            unsigned long SR:1;
            unsigned long DL:1;
            unsigned long DW:1;
            unsigned long DER:1;
            unsigned long MODE:1;
            unsigned long :4;
            unsigned long INTM:1;
            unsigned long DMARQM:1;
            unsigned long SWPRQ:1;
            unsigned long :5;
            unsigned long DNUM:8;
        } BIT;
    } DMAC0_CHSTAT_3;
    union {
        unsigned long LONG;
        struct {
            unsigned long SETEN:1;
            unsigned long CLREN:1;
            unsigned long :1;
            unsigned long SWRST:1;
            unsigned long CLRRQ:1;
            unsigned long CLREND:1;
            unsigned long :1;
            unsigned long CLRDE:1;
            unsigned long SETSUS:1;
            unsigned long CLRSUS:1;
            unsigned long :2;
            unsigned long SETREN:1;
            unsigned long :1;
            unsigned long SETSSWPRQ:1;
            unsigned long :1;
            unsigned long SETINTM:1;
            unsigned long CLRINTM:1;
            unsigned long SETDMARQM:1;
            unsigned long CLRDMARQM:1;
            unsigned long :12;
        } BIT;
    } DMAC0_CHCTRL_3;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEL:3;
            unsigned long REQD:1;
            unsigned long LOEN:1;
            unsigned long HIEN:1;
            unsigned long LVL:1;
            unsigned long :1;
            unsigned long AM:3;
            unsigned long DRRP:1;
            unsigned long SDS:4;
            unsigned long DDS:4;
            unsigned long SAD:1;
            unsigned long DAD:1;
            unsigned long TM:1;
            unsigned long WONLY:1;
            unsigned long DEM:1;
            unsigned long :1;
            unsigned long DIM:1;
            unsigned long SBE:1;
            unsigned long RSEL:1;
            unsigned long RSW:1;
            unsigned long REN:1;
            unsigned long DMS:1;
        } BIT;
    } DMAC0_CHCFG_3;
    union {
        unsigned long LONG;
        struct {
            unsigned long ITVL:16;
            unsigned long :16;
        } BIT;
    } DMAC0_CHITVL_3;
    char           wk3[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long NXLA:32;
        } BIT;
    } DMAC0_NXLA_3;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRLA:32;
        } BIT;
    } DMAC0_CRLA_3;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC0_N0SA_4_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC0_N0SA_4_W;
    } N0SA_4;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC0_N0DA_4;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC0_N0TB_4;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC0_N1SA_4_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC0_N1SA_4_W;
    } N1SA_4;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC0_N1DA_4;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC0_N1TB_4;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRSA:32;
        } BIT;
    } DMAC0_CRSA_4;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRDA:32;
        } BIT;
    } DMAC0_CRDA_4;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRTB:32;
        } BIT;
    } DMAC0_CRTB_4;
    union {
        unsigned long LONG;
        struct {
            unsigned long EN:1;
            unsigned long RQST:1;
            unsigned long TACT:1;
            unsigned long SUS:1;
            unsigned long ER:1;
            unsigned long END:1;
            unsigned long :1;
            unsigned long SR:1;
            unsigned long DL:1;
            unsigned long DW:1;
            unsigned long DER:1;
            unsigned long MODE:1;
            unsigned long :4;
            unsigned long INTM:1;
            unsigned long DMARQM:1;
            unsigned long SWPRQ:1;
            unsigned long :5;
            unsigned long DNUM:8;
        } BIT;
    } DMAC0_CHSTAT_4;
    union {
        unsigned long LONG;
        struct {
            unsigned long SETEN:1;
            unsigned long CLREN:1;
            unsigned long :1;
            unsigned long SWRST:1;
            unsigned long CLRRQ:1;
            unsigned long CLREND:1;
            unsigned long :1;
            unsigned long CLRDE:1;
            unsigned long SETSUS:1;
            unsigned long CLRSUS:1;
            unsigned long :2;
            unsigned long SETREN:1;
            unsigned long :1;
            unsigned long SETSSWPRQ:1;
            unsigned long :1;
            unsigned long SETINTM:1;
            unsigned long CLRINTM:1;
            unsigned long SETDMARQM:1;
            unsigned long CLRDMARQM:1;
            unsigned long :12;
        } BIT;
    } DMAC0_CHCTRL_4;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEL:3;
            unsigned long REQD:1;
            unsigned long LOEN:1;
            unsigned long HIEN:1;
            unsigned long LVL:1;
            unsigned long :1;
            unsigned long AM:3;
            unsigned long DRRP:1;
            unsigned long SDS:4;
            unsigned long DDS:4;
            unsigned long SAD:1;
            unsigned long DAD:1;
            unsigned long TM:1;
            unsigned long WONLY:1;
            unsigned long DEM:1;
            unsigned long :1;
            unsigned long DIM:1;
            unsigned long SBE:1;
            unsigned long RSEL:1;
            unsigned long RSW:1;
            unsigned long REN:1;
            unsigned long DMS:1;
        } BIT;
    } DMAC0_CHCFG_4;
    union {
        unsigned long LONG;
        struct {
            unsigned long ITVL:16;
            unsigned long :16;
        } BIT;
    } DMAC0_CHITVL_4;
    char           wk4[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long NXLA:32;
        } BIT;
    } DMAC0_NXLA_4;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRLA:32;
        } BIT;
    } DMAC0_CRLA_4;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC0_N0SA_5_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC0_N0SA_5_W;
    } N0SA_5;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC0_N0DA_5;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC0_N0TB_5;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC0_N1SA_5_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC0_N1SA_5_W;
    } N1SA_5;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC0_N1DA_5;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC0_N1TB_5;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRSA:32;
        } BIT;
    } DMAC0_CRSA_5;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRDA:32;
        } BIT;
    } DMAC0_CRDA_5;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRTB:32;
        } BIT;
    } DMAC0_CRTB_5;
    union {
        unsigned long LONG;
        struct {
            unsigned long EN:1;
            unsigned long RQST:1;
            unsigned long TACT:1;
            unsigned long SUS:1;
            unsigned long ER:1;
            unsigned long END:1;
            unsigned long :1;
            unsigned long SR:1;
            unsigned long DL:1;
            unsigned long DW:1;
            unsigned long DER:1;
            unsigned long MODE:1;
            unsigned long :4;
            unsigned long INTM:1;
            unsigned long DMARQM:1;
            unsigned long SWPRQ:1;
            unsigned long :5;
            unsigned long DNUM:8;
        } BIT;
    } DMAC0_CHSTAT_5;
    union {
        unsigned long LONG;
        struct {
            unsigned long SETEN:1;
            unsigned long CLREN:1;
            unsigned long :1;
            unsigned long SWRST:1;
            unsigned long CLRRQ:1;
            unsigned long CLREND:1;
            unsigned long :1;
            unsigned long CLRDE:1;
            unsigned long SETSUS:1;
            unsigned long CLRSUS:1;
            unsigned long :2;
            unsigned long SETREN:1;
            unsigned long :1;
            unsigned long SETSSWPRQ:1;
            unsigned long :1;
            unsigned long SETINTM:1;
            unsigned long CLRINTM:1;
            unsigned long SETDMARQM:1;
            unsigned long CLRDMARQM:1;
            unsigned long :12;
        } BIT;
    } DMAC0_CHCTRL_5;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEL:3;
            unsigned long REQD:1;
            unsigned long LOEN:1;
            unsigned long HIEN:1;
            unsigned long LVL:1;
            unsigned long :1;
            unsigned long AM:3;
            unsigned long DRRP:1;
            unsigned long SDS:4;
            unsigned long DDS:4;
            unsigned long SAD:1;
            unsigned long DAD:1;
            unsigned long TM:1;
            unsigned long WONLY:1;
            unsigned long DEM:1;
            unsigned long :1;
            unsigned long DIM:1;
            unsigned long SBE:1;
            unsigned long RSEL:1;
            unsigned long RSW:1;
            unsigned long REN:1;
            unsigned long DMS:1;
        } BIT;
    } DMAC0_CHCFG_5;
    union {
        unsigned long LONG;
        struct {
            unsigned long ITVL:16;
            unsigned long :16;
        } BIT;
    } DMAC0_CHITVL_5;
    char           wk5[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long NXLA:32;
        } BIT;
    } DMAC0_NXLA_5;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRLA:32;
        } BIT;
    } DMAC0_CRLA_5;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC0_N0SA_6_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC0_N0SA_6_W;
    } N0SA_6;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC0_N0DA_6;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC0_N0TB_6;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC0_N1SA_6_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC0_N1SA_6_W;
    } N1SA_6;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC0_N1DA_6;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC0_N1TB_6;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRSA:32;
        } BIT;
    } DMAC0_CRSA_6;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRDA:32;
        } BIT;
    } DMAC0_CRDA_6;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRTB:32;
        } BIT;
    } DMAC0_CRTB_6;
    union {
        unsigned long LONG;
        struct {
            unsigned long EN:1;
            unsigned long RQST:1;
            unsigned long TACT:1;
            unsigned long SUS:1;
            unsigned long ER:1;
            unsigned long END:1;
            unsigned long :1;
            unsigned long SR:1;
            unsigned long DL:1;
            unsigned long DW:1;
            unsigned long DER:1;
            unsigned long MODE:1;
            unsigned long :4;
            unsigned long INTM:1;
            unsigned long DMARQM:1;
            unsigned long SWPRQ:1;
            unsigned long :5;
            unsigned long DNUM:8;
        } BIT;
    } DMAC0_CHSTAT_6;
    union {
        unsigned long LONG;
        struct {
            unsigned long SETEN:1;
            unsigned long CLREN:1;
            unsigned long :1;
            unsigned long SWRST:1;
            unsigned long CLRRQ:1;
            unsigned long CLREND:1;
            unsigned long :1;
            unsigned long CLRDE:1;
            unsigned long SETSUS:1;
            unsigned long CLRSUS:1;
            unsigned long :2;
            unsigned long SETREN:1;
            unsigned long :1;
            unsigned long SETSSWPRQ:1;
            unsigned long :1;
            unsigned long SETINTM:1;
            unsigned long CLRINTM:1;
            unsigned long SETDMARQM:1;
            unsigned long CLRDMARQM:1;
            unsigned long :12;
        } BIT;
    } DMAC0_CHCTRL_6;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEL:3;
            unsigned long REQD:1;
            unsigned long LOEN:1;
            unsigned long HIEN:1;
            unsigned long LVL:1;
            unsigned long :1;
            unsigned long AM:3;
            unsigned long DRRP:1;
            unsigned long SDS:4;
            unsigned long DDS:4;
            unsigned long SAD:1;
            unsigned long DAD:1;
            unsigned long TM:1;
            unsigned long WONLY:1;
            unsigned long DEM:1;
            unsigned long :1;
            unsigned long DIM:1;
            unsigned long SBE:1;
            unsigned long RSEL:1;
            unsigned long RSW:1;
            unsigned long REN:1;
            unsigned long DMS:1;
        } BIT;
    } DMAC0_CHCFG_6;
    union {
        unsigned long LONG;
        struct {
            unsigned long ITVL:16;
            unsigned long :16;
        } BIT;
    } DMAC0_CHITVL_6;
    char           wk6[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long NXLA:32;
        } BIT;
    } DMAC0_NXLA_6;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRLA:32;
        } BIT;
    } DMAC0_CRLA_6;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC0_N0SA_7_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC0_N0SA_7_W;
    } N0SA_7;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC0_N0DA_7;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC0_N0TB_7;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC0_N1SA_7_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC0_N1SA_7_W;
    } N1SA_7;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC0_N1DA_7;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC0_N1TB_7;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRSA:32;
        } BIT;
    } DMAC0_CRSA_7;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRDA:32;
        } BIT;
    } DMAC0_CRDA_7;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRTB:32;
        } BIT;
    } DMAC0_CRTB_7;
    union {
        unsigned long LONG;
        struct {
            unsigned long EN:1;
            unsigned long RQST:1;
            unsigned long TACT:1;
            unsigned long SUS:1;
            unsigned long ER:1;
            unsigned long END:1;
            unsigned long :1;
            unsigned long SR:1;
            unsigned long DL:1;
            unsigned long DW:1;
            unsigned long DER:1;
            unsigned long MODE:1;
            unsigned long :4;
            unsigned long INTM:1;
            unsigned long DMARQM:1;
            unsigned long SWPRQ:1;
            unsigned long :5;
            unsigned long DNUM:8;
        } BIT;
    } DMAC0_CHSTAT_7;
    union {
        unsigned long LONG;
        struct {
            unsigned long SETEN:1;
            unsigned long CLREN:1;
            unsigned long :1;
            unsigned long SWRST:1;
            unsigned long CLRRQ:1;
            unsigned long CLREND:1;
            unsigned long :1;
            unsigned long CLRDE:1;
            unsigned long SETSUS:1;
            unsigned long CLRSUS:1;
            unsigned long :2;
            unsigned long SETREN:1;
            unsigned long :1;
            unsigned long SETSSWPRQ:1;
            unsigned long :1;
            unsigned long SETINTM:1;
            unsigned long CLRINTM:1;
            unsigned long SETDMARQM:1;
            unsigned long CLRDMARQM:1;
            unsigned long :12;
        } BIT;
    } DMAC0_CHCTRL_7;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEL:3;
            unsigned long REQD:1;
            unsigned long LOEN:1;
            unsigned long HIEN:1;
            unsigned long LVL:1;
            unsigned long :1;
            unsigned long AM:3;
            unsigned long DRRP:1;
            unsigned long SDS:4;
            unsigned long DDS:4;
            unsigned long SAD:1;
            unsigned long DAD:1;
            unsigned long TM:1;
            unsigned long WONLY:1;
            unsigned long DEM:1;
            unsigned long :1;
            unsigned long DIM:1;
            unsigned long SBE:1;
            unsigned long RSEL:1;
            unsigned long RSW:1;
            unsigned long REN:1;
            unsigned long DMS:1;
        } BIT;
    } DMAC0_CHCFG_7;
    union {
        unsigned long LONG;
        struct {
            unsigned long ITVL:16;
            unsigned long :16;
        } BIT;
    } DMAC0_CHITVL_7;
    char           wk7[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long NXLA:32;
        } BIT;
    } DMAC0_NXLA_7;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRLA:32;
        } BIT;
    } DMAC0_CRLA_7;
    union {
        unsigned long LONG;
        struct {
            unsigned long SCNT:32;
        } BIT;
    } DMAC0_SCNT_0;
    union {
        unsigned long LONG;
        struct {
            unsigned long SSKP:32;
        } BIT;
    } DMAC0_SSKP_0;
    union {
        unsigned long LONG;
        struct {
            unsigned long DCNT:32;
        } BIT;
    } DMAC0_DCNT_0;
    union {
        unsigned long LONG;
        struct {
            unsigned long DSKP:32;
        } BIT;
    } DMAC0_DSKP_0;
    char           wk8[16];
    union {
        unsigned long LONG;
        struct {
            unsigned long SCNT:32;
        } BIT;
    } DMAC0_SCNT_1;
    union {
        unsigned long LONG;
        struct {
            unsigned long SSKP:32;
        } BIT;
    } DMAC0_SSKP_1;
    union {
        unsigned long LONG;
        struct {
            unsigned long DCNT:32;
        } BIT;
    } DMAC0_DCNT_1;
    union {
        unsigned long LONG;
        struct {
            unsigned long DSKP:32;
        } BIT;
    } DMAC0_DSKP_1;
    char           wk9[16];
    union {
        unsigned long LONG;
        struct {
            unsigned long SCNT:32;
        } BIT;
    } DMAC0_SCNT_2;
    union {
        unsigned long LONG;
        struct {
            unsigned long SSKP:32;
        } BIT;
    } DMAC0_SSKP_2;
    union {
        unsigned long LONG;
        struct {
            unsigned long DCNT:32;
        } BIT;
    } DMAC0_DCNT_2;
    union {
        unsigned long LONG;
        struct {
            unsigned long DSKP:32;
        } BIT;
    } DMAC0_DSKP_2;
    char           wk10[16];
    union {
        unsigned long LONG;
        struct {
            unsigned long SCNT:32;
        } BIT;
    } DMAC0_SCNT_3;
    union {
        unsigned long LONG;
        struct {
            unsigned long SSKP:32;
        } BIT;
    } DMAC0_SSKP_3;
    union {
        unsigned long LONG;
        struct {
            unsigned long DCNT:32;
        } BIT;
    } DMAC0_DCNT_3;
    union {
        unsigned long LONG;
        struct {
            unsigned long DSKP:32;
        } BIT;
    } DMAC0_DSKP_3;
    char           wk11[16];
    union {
        unsigned long LONG;
        struct {
            unsigned long SCNT:32;
        } BIT;
    } DMAC0_SCNT_4;
    union {
        unsigned long LONG;
        struct {
            unsigned long SSKP:32;
        } BIT;
    } DMAC0_SSKP_4;
    union {
        unsigned long LONG;
        struct {
            unsigned long DCNT:32;
        } BIT;
    } DMAC0_DCNT_4;
    union {
        unsigned long LONG;
        struct {
            unsigned long DSKP:32;
        } BIT;
    } DMAC0_DSKP_4;
    char           wk12[16];
    union {
        unsigned long LONG;
        struct {
            unsigned long SCNT:32;
        } BIT;
    } DMAC0_SCNT_5;
    union {
        unsigned long LONG;
        struct {
            unsigned long SSKP:32;
        } BIT;
    } DMAC0_SSKP_5;
    union {
        unsigned long LONG;
        struct {
            unsigned long DCNT:32;
        } BIT;
    } DMAC0_DCNT_5;
    union {
        unsigned long LONG;
        struct {
            unsigned long DSKP:32;
        } BIT;
    } DMAC0_DSKP_5;
    char           wk13[16];
    union {
        unsigned long LONG;
        struct {
            unsigned long SCNT:32;
        } BIT;
    } DMAC0_SCNT_6;
    union {
        unsigned long LONG;
        struct {
            unsigned long SSKP:32;
        } BIT;
    } DMAC0_SSKP_6;
    union {
        unsigned long LONG;
        struct {
            unsigned long DCNT:32;
        } BIT;
    } DMAC0_DCNT_6;
    union {
        unsigned long LONG;
        struct {
            unsigned long DSKP:32;
        } BIT;
    } DMAC0_DSKP_6;
    char           wk14[16];
    union {
        unsigned long LONG;
        struct {
            unsigned long SCNT:32;
        } BIT;
    } DMAC0_SCNT_7;
    union {
        unsigned long LONG;
        struct {
            unsigned long SSKP:32;
        } BIT;
    } DMAC0_SSKP_7;
    union {
        unsigned long LONG;
        struct {
            unsigned long DCNT:32;
        } BIT;
    } DMAC0_DCNT_7;
    union {
        unsigned long LONG;
        struct {
            unsigned long DSKP:32;
        } BIT;
    } DMAC0_DSKP_7;
    char           wk15[16];
    union {
        unsigned long LONG;
        struct {
            unsigned long PR:1;
            unsigned long :31;
        } BIT;
    } DMAC0_DCTRL_A;
    union {
        unsigned long LONG;
        struct {
            unsigned long :8;
            unsigned long DITVL:8;
            unsigned long :16;
        } BIT;
    } DMAC0_DSCITVL_A;
    char           wk16[8];
    union {
        unsigned long LONG;
        struct {
            unsigned long EN08:1;
            unsigned long EN19:1;
            unsigned long EN210:1;
            unsigned long EN311:1;
            unsigned long EN412:1;
            unsigned long EN513:1;
            unsigned long EN614:1;
            unsigned long EN715:1;
            unsigned long :24;
        } BIT;
    } DMAC0_DST_EN_A;
    union {
        unsigned long LONG;
        struct {
            unsigned long ER08:1;
            unsigned long ER19:1;
            unsigned long ER210:1;
            unsigned long ER311:1;
            unsigned long ER412:1;
            unsigned long ER513:1;
            unsigned long ER614:1;
            unsigned long ER715:1;
            unsigned long :24;
        } BIT;
    } DMAC0_DST_ER_A;
    union {
        unsigned long LONG;
        struct {
            unsigned long END08:1;
            unsigned long END19:1;
            unsigned long END210:1;
            unsigned long END311:1;
            unsigned long END412:1;
            unsigned long END513:1;
            unsigned long END614:1;
            unsigned long END715:1;
            unsigned long :24;
        } BIT;
    } DMAC0_DST_END_A;
    char           wk17[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long SUS08:1;
            unsigned long SUS19:1;
            unsigned long SUS210:1;
            unsigned long SUS311:1;
            unsigned long SUS412:1;
            unsigned long SUS513:1;
            unsigned long SUS614:1;
            unsigned long SUS715:1;
            unsigned long :24;
        } BIT;
    } DMAC0_DST_SUS_A;
    char           wk18[220];
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC0_N0SA_8_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC0_N0SA_8_W;
    } N0SA_8;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC0_N0DA_8;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC0_N0TB_8;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC0_N1SA_8_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC0_N1SA_8_W;
    } N1SA_8;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC0_N1DA_8;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC0_N1TB_8;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRSA:32;
        } BIT;
    } DMAC0_CRSA_8;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRDA:32;
        } BIT;
    } DMAC0_CRDA_8;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRTB:32;
        } BIT;
    } DMAC0_CRTB_8;
    union {
        unsigned long LONG;
        struct {
            unsigned long EN:1;
            unsigned long RQST:1;
            unsigned long TACT:1;
            unsigned long SUS:1;
            unsigned long ER:1;
            unsigned long END:1;
            unsigned long :1;
            unsigned long SR:1;
            unsigned long DL:1;
            unsigned long DW:1;
            unsigned long DER:1;
            unsigned long MODE:1;
            unsigned long :4;
            unsigned long INTM:1;
            unsigned long DMARQM:1;
            unsigned long SWPRQ:1;
            unsigned long :5;
            unsigned long DNUM:8;
        } BIT;
    } DMAC0_CHSTAT_8;
    union {
        unsigned long LONG;
        struct {
            unsigned long SETEN:1;
            unsigned long CLREN:1;
            unsigned long :1;
            unsigned long SWRST:1;
            unsigned long CLRRQ:1;
            unsigned long CLREND:1;
            unsigned long :1;
            unsigned long CLRDE:1;
            unsigned long SETSUS:1;
            unsigned long CLRSUS:1;
            unsigned long :2;
            unsigned long SETREN:1;
            unsigned long :1;
            unsigned long SETSSWPRQ:1;
            unsigned long :1;
            unsigned long SETINTM:1;
            unsigned long CLRINTM:1;
            unsigned long SETDMARQM:1;
            unsigned long CLRDMARQM:1;
            unsigned long :12;
        } BIT;
    } DMAC0_CHCTRL_8;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEL:3;
            unsigned long REQD:1;
            unsigned long LOEN:1;
            unsigned long HIEN:1;
            unsigned long LVL:1;
            unsigned long :1;
            unsigned long AM:3;
            unsigned long DRRP:1;
            unsigned long SDS:4;
            unsigned long DDS:4;
            unsigned long SAD:1;
            unsigned long DAD:1;
            unsigned long TM:1;
            unsigned long WONLY:1;
            unsigned long DEM:1;
            unsigned long :1;
            unsigned long DIM:1;
            unsigned long SBE:1;
            unsigned long RSEL:1;
            unsigned long RSW:1;
            unsigned long REN:1;
            unsigned long DMS:1;
        } BIT;
    } DMAC0_CHCFG_8;
    union {
        unsigned long LONG;
        struct {
            unsigned long ITVL:16;
            unsigned long :16;
        } BIT;
    } DMAC0_CHITVL_8;
    char           wk19[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long NXLA:32;
        } BIT;
    } DMAC0_NXLA_8;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRLA:32;
        } BIT;
    } DMAC0_CRLA_8;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC0_N0SA_9_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC0_N0SA_9_W;
    } N0SA_9;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC0_N0DA_9;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC0_N0TB_9;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC0_N1SA_9_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC0_N1SA_9_W;
    } N1SA_9;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC0_N1DA_9;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC0_N1TB_9;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRSA:32;
        } BIT;
    } DMAC0_CRSA_9;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRDA:32;
        } BIT;
    } DMAC0_CRDA_9;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRTB:32;
        } BIT;
    } DMAC0_CRTB_9;
    union {
        unsigned long LONG;
        struct {
            unsigned long EN:1;
            unsigned long RQST:1;
            unsigned long TACT:1;
            unsigned long SUS:1;
            unsigned long ER:1;
            unsigned long END:1;
            unsigned long :1;
            unsigned long SR:1;
            unsigned long DL:1;
            unsigned long DW:1;
            unsigned long DER:1;
            unsigned long MODE:1;
            unsigned long :4;
            unsigned long INTM:1;
            unsigned long DMARQM:1;
            unsigned long SWPRQ:1;
            unsigned long :5;
            unsigned long DNUM:8;
        } BIT;
    } DMAC0_CHSTAT_9;
    union {
        unsigned long LONG;
        struct {
            unsigned long SETEN:1;
            unsigned long CLREN:1;
            unsigned long :1;
            unsigned long SWRST:1;
            unsigned long CLRRQ:1;
            unsigned long CLREND:1;
            unsigned long :1;
            unsigned long CLRDE:1;
            unsigned long SETSUS:1;
            unsigned long CLRSUS:1;
            unsigned long :2;
            unsigned long SETREN:1;
            unsigned long :1;
            unsigned long SETSSWPRQ:1;
            unsigned long :1;
            unsigned long SETINTM:1;
            unsigned long CLRINTM:1;
            unsigned long SETDMARQM:1;
            unsigned long CLRDMARQM:1;
            unsigned long :12;
        } BIT;
    } DMAC0_CHCTRL_9;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEL:3;
            unsigned long REQD:1;
            unsigned long LOEN:1;
            unsigned long HIEN:1;
            unsigned long LVL:1;
            unsigned long :1;
            unsigned long AM:3;
            unsigned long DRRP:1;
            unsigned long SDS:4;
            unsigned long DDS:4;
            unsigned long SAD:1;
            unsigned long DAD:1;
            unsigned long TM:1;
            unsigned long WONLY:1;
            unsigned long DEM:1;
            unsigned long :1;
            unsigned long DIM:1;
            unsigned long SBE:1;
            unsigned long RSEL:1;
            unsigned long RSW:1;
            unsigned long REN:1;
            unsigned long DMS:1;
        } BIT;
    } DMAC0_CHCFG_9;
    union {
        unsigned long LONG;
        struct {
            unsigned long ITVL:16;
            unsigned long :16;
        } BIT;
    } DMAC0_CHITVL_9;
    char           wk20[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long NXLA:32;
        } BIT;
    } DMAC0_NXLA_9;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRLA:32;
        } BIT;
    } DMAC0_CRLA_9;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC0_N0SA_10_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC0_N0SA_10_W;
    } N0SA_10;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC0_N0DA_10;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC0_N0TB_10;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC0_N1SA_10_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC0_N1SA_10_W;
    } N1SA_10;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC0_N1DA_10;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC0_N1TB_10;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRSA:32;
        } BIT;
    } DMAC0_CRSA_10;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRDA:32;
        } BIT;
    } DMAC0_CRDA_10;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRTB:32;
        } BIT;
    } DMAC0_CRTB_10;
    union {
        unsigned long LONG;
        struct {
            unsigned long EN:1;
            unsigned long RQST:1;
            unsigned long TACT:1;
            unsigned long SUS:1;
            unsigned long ER:1;
            unsigned long END:1;
            unsigned long :1;
            unsigned long SR:1;
            unsigned long DL:1;
            unsigned long DW:1;
            unsigned long DER:1;
            unsigned long MODE:1;
            unsigned long :4;
            unsigned long INTM:1;
            unsigned long DMARQM:1;
            unsigned long SWPRQ:1;
            unsigned long :5;
            unsigned long DNUM:8;
        } BIT;
    } DMAC0_CHSTAT_10;
    union {
        unsigned long LONG;
        struct {
            unsigned long SETEN:1;
            unsigned long CLREN:1;
            unsigned long :1;
            unsigned long SWRST:1;
            unsigned long CLRRQ:1;
            unsigned long CLREND:1;
            unsigned long :1;
            unsigned long CLRDE:1;
            unsigned long SETSUS:1;
            unsigned long CLRSUS:1;
            unsigned long :2;
            unsigned long SETREN:1;
            unsigned long :1;
            unsigned long SETSSWPRQ:1;
            unsigned long :1;
            unsigned long SETINTM:1;
            unsigned long CLRINTM:1;
            unsigned long SETDMARQM:1;
            unsigned long CLRDMARQM:1;
            unsigned long :12;
        } BIT;
    } DMAC0_CHCTRL_10;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEL:3;
            unsigned long REQD:1;
            unsigned long LOEN:1;
            unsigned long HIEN:1;
            unsigned long LVL:1;
            unsigned long :1;
            unsigned long AM:3;
            unsigned long DRRP:1;
            unsigned long SDS:4;
            unsigned long DDS:4;
            unsigned long SAD:1;
            unsigned long DAD:1;
            unsigned long TM:1;
            unsigned long WONLY:1;
            unsigned long DEM:1;
            unsigned long :1;
            unsigned long DIM:1;
            unsigned long SBE:1;
            unsigned long RSEL:1;
            unsigned long RSW:1;
            unsigned long REN:1;
            unsigned long DMS:1;
        } BIT;
    } DMAC0_CHCFG_10;
    union {
        unsigned long LONG;
        struct {
            unsigned long ITVL:16;
            unsigned long :16;
        } BIT;
    } DMAC0_CHITVL_10;
    char           wk21[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long NXLA:32;
        } BIT;
    } DMAC0_NXLA_10;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRLA:32;
        } BIT;
    } DMAC0_CRLA_10;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC0_N0SA_11_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC0_N0SA_11_W;
    } N0SA_11;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC0_N0DA_11;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC0_N0TB_11;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC0_N1SA_11_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC0_N1SA_11_W;
    } N1SA_11;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC0_N1DA_11;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC0_N1TB_11;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRSA:32;
        } BIT;
    } DMAC0_CRSA_11;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRDA:32;
        } BIT;
    } DMAC0_CRDA_11;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRTB:32;
        } BIT;
    } DMAC0_CRTB_11;
    union {
        unsigned long LONG;
        struct {
            unsigned long EN:1;
            unsigned long RQST:1;
            unsigned long TACT:1;
            unsigned long SUS:1;
            unsigned long ER:1;
            unsigned long END:1;
            unsigned long :1;
            unsigned long SR:1;
            unsigned long DL:1;
            unsigned long DW:1;
            unsigned long DER:1;
            unsigned long MODE:1;
            unsigned long :4;
            unsigned long INTM:1;
            unsigned long DMARQM:1;
            unsigned long SWPRQ:1;
            unsigned long :5;
            unsigned long DNUM:8;
        } BIT;
    } DMAC0_CHSTAT_11;
    union {
        unsigned long LONG;
        struct {
            unsigned long SETEN:1;
            unsigned long CLREN:1;
            unsigned long :1;
            unsigned long SWRST:1;
            unsigned long CLRRQ:1;
            unsigned long CLREND:1;
            unsigned long :1;
            unsigned long CLRDE:1;
            unsigned long SETSUS:1;
            unsigned long CLRSUS:1;
            unsigned long :2;
            unsigned long SETREN:1;
            unsigned long :1;
            unsigned long SETSSWPRQ:1;
            unsigned long :1;
            unsigned long SETINTM:1;
            unsigned long CLRINTM:1;
            unsigned long SETDMARQM:1;
            unsigned long CLRDMARQM:1;
            unsigned long :12;
        } BIT;
    } DMAC0_CHCTRL_11;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEL:3;
            unsigned long REQD:1;
            unsigned long LOEN:1;
            unsigned long HIEN:1;
            unsigned long LVL:1;
            unsigned long :1;
            unsigned long AM:3;
            unsigned long DRRP:1;
            unsigned long SDS:4;
            unsigned long DDS:4;
            unsigned long SAD:1;
            unsigned long DAD:1;
            unsigned long TM:1;
            unsigned long WONLY:1;
            unsigned long DEM:1;
            unsigned long :1;
            unsigned long DIM:1;
            unsigned long SBE:1;
            unsigned long RSEL:1;
            unsigned long RSW:1;
            unsigned long REN:1;
            unsigned long DMS:1;
        } BIT;
    } DMAC0_CHCFG_11;
    union {
        unsigned long LONG;
        struct {
            unsigned long ITVL:16;
            unsigned long :16;
        } BIT;
    } DMAC0_CHITVL_11;
    char           wk22[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long NXLA:32;
        } BIT;
    } DMAC0_NXLA_11;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRLA:32;
        } BIT;
    } DMAC0_CRLA_11;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC0_N0SA_12_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC0_N0SA_12_W;
    } N0SA_12;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC0_N0DA_12;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC0_N0TB_12;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC0_N1SA_12_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC0_N1SA_12_W;
    } N1SA_12;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC0_N1DA_12;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC0_N1TB_12;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRSA:32;
        } BIT;
    } DMAC0_CRSA_12;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRDA:32;
        } BIT;
    } DMAC0_CRDA_12;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRTB:32;
        } BIT;
    } DMAC0_CRTB_12;
    union {
        unsigned long LONG;
        struct {
            unsigned long EN:1;
            unsigned long RQST:1;
            unsigned long TACT:1;
            unsigned long SUS:1;
            unsigned long ER:1;
            unsigned long END:1;
            unsigned long :1;
            unsigned long SR:1;
            unsigned long DL:1;
            unsigned long DW:1;
            unsigned long DER:1;
            unsigned long MODE:1;
            unsigned long :4;
            unsigned long INTM:1;
            unsigned long DMARQM:1;
            unsigned long SWPRQ:1;
            unsigned long :5;
            unsigned long DNUM:8;
        } BIT;
    } DMAC0_CHSTAT_12;
    union {
        unsigned long LONG;
        struct {
            unsigned long SETEN:1;
            unsigned long CLREN:1;
            unsigned long :1;
            unsigned long SWRST:1;
            unsigned long CLRRQ:1;
            unsigned long CLREND:1;
            unsigned long :1;
            unsigned long CLRDE:1;
            unsigned long SETSUS:1;
            unsigned long CLRSUS:1;
            unsigned long :2;
            unsigned long SETREN:1;
            unsigned long :1;
            unsigned long SETSSWPRQ:1;
            unsigned long :1;
            unsigned long SETINTM:1;
            unsigned long CLRINTM:1;
            unsigned long SETDMARQM:1;
            unsigned long CLRDMARQM:1;
            unsigned long :12;
        } BIT;
    } DMAC0_CHCTRL_12;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEL:3;
            unsigned long REQD:1;
            unsigned long LOEN:1;
            unsigned long HIEN:1;
            unsigned long LVL:1;
            unsigned long :1;
            unsigned long AM:3;
            unsigned long DRRP:1;
            unsigned long SDS:4;
            unsigned long DDS:4;
            unsigned long SAD:1;
            unsigned long DAD:1;
            unsigned long TM:1;
            unsigned long WONLY:1;
            unsigned long DEM:1;
            unsigned long :1;
            unsigned long DIM:1;
            unsigned long SBE:1;
            unsigned long RSEL:1;
            unsigned long RSW:1;
            unsigned long REN:1;
            unsigned long DMS:1;
        } BIT;
    } DMAC0_CHCFG_12;
    union {
        unsigned long LONG;
        struct {
            unsigned long ITVL:16;
            unsigned long :16;
        } BIT;
    } DMAC0_CHITVL_12;
    char           wk23[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long NXLA:32;
        } BIT;
    } DMAC0_NXLA_12;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRLA:32;
        } BIT;
    } DMAC0_CRLA_12;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC0_N0SA_13_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC0_N0SA_13_W;
    } N0SA_13;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC0_N0DA_13;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC0_N0TB_13;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC0_N1SA_13_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC0_N1SA_13_W;
    } N1SA_13;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC0_N1DA_13;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC0_N1TB_13;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRSA:32;
        } BIT;
    } DMAC0_CRSA_13;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRDA:32;
        } BIT;
    } DMAC0_CRDA_13;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRTB:32;
        } BIT;
    } DMAC0_CRTB_13;
    union {
        unsigned long LONG;
        struct {
            unsigned long EN:1;
            unsigned long RQST:1;
            unsigned long TACT:1;
            unsigned long SUS:1;
            unsigned long ER:1;
            unsigned long END:1;
            unsigned long :1;
            unsigned long SR:1;
            unsigned long DL:1;
            unsigned long DW:1;
            unsigned long DER:1;
            unsigned long MODE:1;
            unsigned long :4;
            unsigned long INTM:1;
            unsigned long DMARQM:1;
            unsigned long SWPRQ:1;
            unsigned long :5;
            unsigned long DNUM:8;
        } BIT;
    } DMAC0_CHSTAT_13;
    union {
        unsigned long LONG;
        struct {
            unsigned long SETEN:1;
            unsigned long CLREN:1;
            unsigned long :1;
            unsigned long SWRST:1;
            unsigned long CLRRQ:1;
            unsigned long CLREND:1;
            unsigned long :1;
            unsigned long CLRDE:1;
            unsigned long SETSUS:1;
            unsigned long CLRSUS:1;
            unsigned long :2;
            unsigned long SETREN:1;
            unsigned long :1;
            unsigned long SETSSWPRQ:1;
            unsigned long :1;
            unsigned long SETINTM:1;
            unsigned long CLRINTM:1;
            unsigned long SETDMARQM:1;
            unsigned long CLRDMARQM:1;
            unsigned long :12;
        } BIT;
    } DMAC0_CHCTRL_13;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEL:3;
            unsigned long REQD:1;
            unsigned long LOEN:1;
            unsigned long HIEN:1;
            unsigned long LVL:1;
            unsigned long :1;
            unsigned long AM:3;
            unsigned long DRRP:1;
            unsigned long SDS:4;
            unsigned long DDS:4;
            unsigned long SAD:1;
            unsigned long DAD:1;
            unsigned long TM:1;
            unsigned long WONLY:1;
            unsigned long DEM:1;
            unsigned long :1;
            unsigned long DIM:1;
            unsigned long SBE:1;
            unsigned long RSEL:1;
            unsigned long RSW:1;
            unsigned long REN:1;
            unsigned long DMS:1;
        } BIT;
    } DMAC0_CHCFG_13;
    union {
        unsigned long LONG;
        struct {
            unsigned long ITVL:16;
            unsigned long :16;
        } BIT;
    } DMAC0_CHITVL_13;
    char           wk24[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long NXLA:32;
        } BIT;
    } DMAC0_NXLA_13;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRLA:32;
        } BIT;
    } DMAC0_CRLA_13;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC0_N0SA_14_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC0_N0SA_14_W;
    } N0SA_14;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC0_N0DA_14;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC0_N0TB_14;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC0_N1SA_14_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC0_N1SA_14_W;
    } N1SA_14;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC0_N1DA_14;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC0_N1TB_14;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRSA:32;
        } BIT;
    } DMAC0_CRSA_14;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRDA:32;
        } BIT;
    } DMAC0_CRDA_14;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRTB:32;
        } BIT;
    } DMAC0_CRTB_14;
    union {
        unsigned long LONG;
        struct {
            unsigned long EN:1;
            unsigned long RQST:1;
            unsigned long TACT:1;
            unsigned long SUS:1;
            unsigned long ER:1;
            unsigned long END:1;
            unsigned long :1;
            unsigned long SR:1;
            unsigned long DL:1;
            unsigned long DW:1;
            unsigned long DER:1;
            unsigned long MODE:1;
            unsigned long :4;
            unsigned long INTM:1;
            unsigned long DMARQM:1;
            unsigned long SWPRQ:1;
            unsigned long :5;
            unsigned long DNUM:8;
        } BIT;
    } DMAC0_CHSTAT_14;
    union {
        unsigned long LONG;
        struct {
            unsigned long SETEN:1;
            unsigned long CLREN:1;
            unsigned long :1;
            unsigned long SWRST:1;
            unsigned long CLRRQ:1;
            unsigned long CLREND:1;
            unsigned long :1;
            unsigned long CLRDE:1;
            unsigned long SETSUS:1;
            unsigned long CLRSUS:1;
            unsigned long :2;
            unsigned long SETREN:1;
            unsigned long :1;
            unsigned long SETSSWPRQ:1;
            unsigned long :1;
            unsigned long SETINTM:1;
            unsigned long CLRINTM:1;
            unsigned long SETDMARQM:1;
            unsigned long CLRDMARQM:1;
            unsigned long :12;
        } BIT;
    } DMAC0_CHCTRL_14;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEL:3;
            unsigned long REQD:1;
            unsigned long LOEN:1;
            unsigned long HIEN:1;
            unsigned long LVL:1;
            unsigned long :1;
            unsigned long AM:3;
            unsigned long DRRP:1;
            unsigned long SDS:4;
            unsigned long DDS:4;
            unsigned long SAD:1;
            unsigned long DAD:1;
            unsigned long TM:1;
            unsigned long WONLY:1;
            unsigned long DEM:1;
            unsigned long :1;
            unsigned long DIM:1;
            unsigned long SBE:1;
            unsigned long RSEL:1;
            unsigned long RSW:1;
            unsigned long REN:1;
            unsigned long DMS:1;
        } BIT;
    } DMAC0_CHCFG_14;
    union {
        unsigned long LONG;
        struct {
            unsigned long ITVL:16;
            unsigned long :16;
        } BIT;
    } DMAC0_CHITVL_14;
    char           wk25[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long NXLA:32;
        } BIT;
    } DMAC0_NXLA_14;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRLA:32;
        } BIT;
    } DMAC0_CRLA_14;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC0_N0SA_15_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC0_N0SA_15_W;
    } N0SA_15;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC0_N0DA_15;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC0_N0TB_15;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC0_N1SA_15_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC0_N1SA_15_W;
    } N1SA_15;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC0_N1DA_15;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC0_N1TB_15;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRSA:32;
        } BIT;
    } DMAC0_CRSA_15;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRDA:32;
        } BIT;
    } DMAC0_CRDA_15;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRTB:32;
        } BIT;
    } DMAC0_CRTB_15;
    union {
        unsigned long LONG;
        struct {
            unsigned long EN:1;
            unsigned long RQST:1;
            unsigned long TACT:1;
            unsigned long SUS:1;
            unsigned long ER:1;
            unsigned long END:1;
            unsigned long :1;
            unsigned long SR:1;
            unsigned long DL:1;
            unsigned long DW:1;
            unsigned long DER:1;
            unsigned long MODE:1;
            unsigned long :4;
            unsigned long INTM:1;
            unsigned long DMARQM:1;
            unsigned long SWPRQ:1;
            unsigned long :5;
            unsigned long DNUM:8;
        } BIT;
    } DMAC0_CHSTAT_15;
    union {
        unsigned long LONG;
        struct {
            unsigned long SETEN:1;
            unsigned long CLREN:1;
            unsigned long :1;
            unsigned long SWRST:1;
            unsigned long CLRRQ:1;
            unsigned long CLREND:1;
            unsigned long :1;
            unsigned long CLRDE:1;
            unsigned long SETSUS:1;
            unsigned long CLRSUS:1;
            unsigned long :2;
            unsigned long SETREN:1;
            unsigned long :1;
            unsigned long SETSSWPRQ:1;
            unsigned long :1;
            unsigned long SETINTM:1;
            unsigned long CLRINTM:1;
            unsigned long SETDMARQM:1;
            unsigned long CLRDMARQM:1;
            unsigned long :12;
        } BIT;
    } DMAC0_CHCTRL_15;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEL:3;
            unsigned long REQD:1;
            unsigned long LOEN:1;
            unsigned long HIEN:1;
            unsigned long LVL:1;
            unsigned long :1;
            unsigned long AM:3;
            unsigned long DRRP:1;
            unsigned long SDS:4;
            unsigned long DDS:4;
            unsigned long SAD:1;
            unsigned long DAD:1;
            unsigned long TM:1;
            unsigned long WONLY:1;
            unsigned long DEM:1;
            unsigned long :1;
            unsigned long DIM:1;
            unsigned long SBE:1;
            unsigned long RSEL:1;
            unsigned long RSW:1;
            unsigned long REN:1;
            unsigned long DMS:1;
        } BIT;
    } DMAC0_CHCFG_15;
    union {
        unsigned long LONG;
        struct {
            unsigned long ITVL:16;
            unsigned long :16;
        } BIT;
    } DMAC0_CHITVL_15;
    char           wk26[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long NXLA:32;
        } BIT;
    } DMAC0_NXLA_15;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRLA:32;
        } BIT;
    } DMAC0_CRLA_15;
    union {
        unsigned long LONG;
        struct {
            unsigned long SCNT:32;
        } BIT;
    } DMAC0_SCNT_8;
    union {
        unsigned long LONG;
        struct {
            unsigned long SSKP:32;
        } BIT;
    } DMAC0_SSKP_8;
    union {
        unsigned long LONG;
        struct {
            unsigned long DCNT:32;
        } BIT;
    } DMAC0_DCNT_8;
    union {
        unsigned long LONG;
        struct {
            unsigned long DSKP:32;
        } BIT;
    } DMAC0_DSKP_8;
    char           wk27[16];
    union {
        unsigned long LONG;
        struct {
            unsigned long SCNT:32;
        } BIT;
    } DMAC0_SCNT_9;
    union {
        unsigned long LONG;
        struct {
            unsigned long SSKP:32;
        } BIT;
    } DMAC0_SSKP_9;
    union {
        unsigned long LONG;
        struct {
            unsigned long DCNT:32;
        } BIT;
    } DMAC0_DCNT_9;
    union {
        unsigned long LONG;
        struct {
            unsigned long DSKP:32;
        } BIT;
    } DMAC0_DSKP_9;
    char           wk28[16];
    union {
        unsigned long LONG;
        struct {
            unsigned long SCNT:32;
        } BIT;
    } DMAC0_SCNT_10;
    union {
        unsigned long LONG;
        struct {
            unsigned long SSKP:32;
        } BIT;
    } DMAC0_SSKP_10;
    union {
        unsigned long LONG;
        struct {
            unsigned long DCNT:32;
        } BIT;
    } DMAC0_DCNT_10;
    union {
        unsigned long LONG;
        struct {
            unsigned long DSKP:32;
        } BIT;
    } DMAC0_DSKP_10;
    char           wk29[16];
    union {
        unsigned long LONG;
        struct {
            unsigned long SCNT:32;
        } BIT;
    } DMAC0_SCNT_11;
    union {
        unsigned long LONG;
        struct {
            unsigned long SSKP:32;
        } BIT;
    } DMAC0_SSKP_11;
    union {
        unsigned long LONG;
        struct {
            unsigned long DCNT:32;
        } BIT;
    } DMAC0_DCNT_11;
    union {
        unsigned long LONG;
        struct {
            unsigned long DSKP:32;
        } BIT;
    } DMAC0_DSKP_11;
    char           wk30[16];
    union {
        unsigned long LONG;
        struct {
            unsigned long SCNT:32;
        } BIT;
    } DMAC0_SCNT_12;
    union {
        unsigned long LONG;
        struct {
            unsigned long SSKP:32;
        } BIT;
    } DMAC0_SSKP_12;
    union {
        unsigned long LONG;
        struct {
            unsigned long DCNT:32;
        } BIT;
    } DMAC0_DCNT_12;
    union {
        unsigned long LONG;
        struct {
            unsigned long DSKP:32;
        } BIT;
    } DMAC0_DSKP_12;
    char           wk31[16];
    union {
        unsigned long LONG;
        struct {
            unsigned long SCNT:32;
        } BIT;
    } DMAC0_SCNT_13;
    union {
        unsigned long LONG;
        struct {
            unsigned long SSKP:32;
        } BIT;
    } DMAC0_SSKP_13;
    union {
        unsigned long LONG;
        struct {
            unsigned long DCNT:32;
        } BIT;
    } DMAC0_DCNT_13;
    union {
        unsigned long LONG;
        struct {
            unsigned long DSKP:32;
        } BIT;
    } DMAC0_DSKP_13;
    char           wk32[16];
    union {
        unsigned long LONG;
        struct {
            unsigned long SCNT:32;
        } BIT;
    } DMAC0_SCNT_14;
    union {
        unsigned long LONG;
        struct {
            unsigned long SSKP:32;
        } BIT;
    } DMAC0_SSKP_14;
    union {
        unsigned long LONG;
        struct {
            unsigned long DCNT:32;
        } BIT;
    } DMAC0_DCNT_14;
    union {
        unsigned long LONG;
        struct {
            unsigned long DSKP:32;
        } BIT;
    } DMAC0_DSKP_14;
    char           wk33[16];
    union {
        unsigned long LONG;
        struct {
            unsigned long SCNT:32;
        } BIT;
    } DMAC0_SCNT_15;
    union {
        unsigned long LONG;
        struct {
            unsigned long SSKP:32;
        } BIT;
    } DMAC0_SSKP_15;
    union {
        unsigned long LONG;
        struct {
            unsigned long DCNT:32;
        } BIT;
    } DMAC0_DCNT_15;
    union {
        unsigned long LONG;
        struct {
            unsigned long DSKP:32;
        } BIT;
    } DMAC0_DSKP_15;
    char           wk34[16];
    union {
        unsigned long LONG;
        struct {
            unsigned long PR:1;
            unsigned long :31;
        } BIT;
    } DMAC0_DCTRL_B;
    union {
        unsigned long LONG;
        struct {
            unsigned long :8;
            unsigned long DITVL:8;
            unsigned long :16;
        } BIT;
    } DMAC0_DSCITVL_B;
    char           wk35[8];
    union {
        unsigned long LONG;
        struct {
            unsigned long EN08:1;
            unsigned long EN19:1;
            unsigned long EN210:1;
            unsigned long EN311:1;
            unsigned long EN412:1;
            unsigned long EN513:1;
            unsigned long EN614:1;
            unsigned long EN715:1;
            unsigned long :24;
        } BIT;
    } DMAC0_DST_EN_B;
    union {
        unsigned long LONG;
        struct {
            unsigned long ER08:1;
            unsigned long ER19:1;
            unsigned long ER210:1;
            unsigned long ER311:1;
            unsigned long ER412:1;
            unsigned long ER513:1;
            unsigned long ER614:1;
            unsigned long ER715:1;
            unsigned long :24;
        } BIT;
    } DMAC0_DST_ER_B;
    union {
        unsigned long LONG;
        struct {
            unsigned long END08:1;
            unsigned long END19:1;
            unsigned long END210:1;
            unsigned long END311:1;
            unsigned long END412:1;
            unsigned long END513:1;
            unsigned long END614:1;
            unsigned long END715:1;
            unsigned long :24;
        } BIT;
    } DMAC0_DST_END_B;
    char           wk36[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long SUS08:1;
            unsigned long SUS19:1;
            unsigned long SUS210:1;
            unsigned long SUS311:1;
            unsigned long SUS412:1;
            unsigned long SUS513:1;
            unsigned long SUS614:1;
            unsigned long SUS715:1;
            unsigned long :24;
        } BIT;
    } DMAC0_DST_SUS_B;
    char           wk37[202972];
    union {
        unsigned long LONG;
        struct {
            unsigned long IFC0:8;
            unsigned long :24;
        } BIT;
    } DMA0SEL0;
    union {
        unsigned long LONG;
        struct {
            unsigned long IFC0:8;
            unsigned long :24;
        } BIT;
    } DMA0SEL1;
    union {
        unsigned long LONG;
        struct {
            unsigned long IFC0:8;
            unsigned long :24;
        } BIT;
    } DMA0SEL2;
    union {
        unsigned long LONG;
        struct {
            unsigned long IFC0:8;
            unsigned long :24;
        } BIT;
    } DMA0SEL3;
    union {
        unsigned long LONG;
        struct {
            unsigned long IFC0:8;
            unsigned long :24;
        } BIT;
    } DMA0SEL4;
    union {
        unsigned long LONG;
        struct {
            unsigned long IFC0:8;
            unsigned long :24;
        } BIT;
    } DMA0SEL5;
    union {
        unsigned long LONG;
        struct {
            unsigned long IFC0:8;
            unsigned long :24;
        } BIT;
    } DMA0SEL6;
    union {
        unsigned long LONG;
        struct {
            unsigned long IFC0:8;
            unsigned long :24;
        } BIT;
    } DMA0SEL7;
    union {
        unsigned long LONG;
        struct {
            unsigned long IFC0:8;
            unsigned long :24;
        } BIT;
    } DMA0SEL8;
    union {
        unsigned long LONG;
        struct {
            unsigned long IFC0:8;
            unsigned long :24;
        } BIT;
    } DMA0SEL9;
    union {
        unsigned long LONG;
        struct {
            unsigned long IFC0:8;
            unsigned long :24;
        } BIT;
    } DMA0SEL10;
    union {
        unsigned long LONG;
        struct {
            unsigned long IFC0:8;
            unsigned long :24;
        } BIT;
    } DMA0SEL11;
    union {
        unsigned long LONG;
        struct {
            unsigned long IFC0:8;
            unsigned long :24;
        } BIT;
    } DMA0SEL12;
    union {
        unsigned long LONG;
        struct {
            unsigned long IFC0:8;
            unsigned long :24;
        } BIT;
    } DMA0SEL13;
    union {
        unsigned long LONG;
        struct {
            unsigned long IFC0:8;
            unsigned long :24;
        } BIT;
    } DMA0SEL14;
    union {
        unsigned long LONG;
        struct {
            unsigned long IFC0:8;
            unsigned long :24;
        } BIT;
    } DMA0SEL15;
};

struct st_dma1 {
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC1_N0SA_0_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC1_N0SA_0_W;
    } N0SA_0;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC1_N0DA_0;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC1_N0TB_0;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC1_N1SA_0_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC1_N1SA_0_W;
    } N1SA_0;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC1_N1DA_0;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC1_N1TB_0;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRSA:32;
        } BIT;
    } DMAC1_CRSA_0;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRDA:32;
        } BIT;
    } DMAC1_CRDA_0;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRTB:32;
        } BIT;
    } DMAC1_CRTB_0;
    union {
        unsigned long LONG;
        struct {
            unsigned long EN:1;
            unsigned long RQST:1;
            unsigned long TACT:1;
            unsigned long SUS:1;
            unsigned long ER:1;
            unsigned long END:1;
            unsigned long :1;
            unsigned long SR:1;
            unsigned long DL:1;
            unsigned long DW:1;
            unsigned long DER:1;
            unsigned long MODE:1;
            unsigned long :4;
            unsigned long INTM:1;
            unsigned long DMARQM:1;
            unsigned long SWPRQ:1;
            unsigned long :5;
            unsigned long DNUM:8;
        } BIT;
    } DMAC1_CHSTAT_0;
    union {
        unsigned long LONG;
        struct {
            unsigned long SETEN:1;
            unsigned long CLREN:1;
            unsigned long :1;
            unsigned long SWRST:1;
            unsigned long CLRRQ:1;
            unsigned long CLREND:1;
            unsigned long :1;
            unsigned long CLRDE:1;
            unsigned long SETSUS:1;
            unsigned long CLRSUS:1;
            unsigned long :2;
            unsigned long SETREN:1;
            unsigned long :1;
            unsigned long SETSSWPRQ:1;
            unsigned long :1;
            unsigned long SETINTM:1;
            unsigned long CLRINTM:1;
            unsigned long SETDMARQM:1;
            unsigned long CLRDMARQM:1;
            unsigned long :12;
        } BIT;
    } DMAC1_CHCTRL_0;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEL:3;
            unsigned long REQD:1;
            unsigned long LOEN:1;
            unsigned long HIEN:1;
            unsigned long LVL:1;
            unsigned long :1;
            unsigned long AM:3;
            unsigned long DRRP:1;
            unsigned long SDS:4;
            unsigned long DDS:4;
            unsigned long SAD:1;
            unsigned long DAD:1;
            unsigned long TM:1;
            unsigned long WONLY:1;
            unsigned long DEM:1;
            unsigned long :1;
            unsigned long DIM:1;
            unsigned long SBE:1;
            unsigned long RSEL:1;
            unsigned long RSW:1;
            unsigned long REN:1;
            unsigned long DMS:1;
        } BIT;
    } DMAC1_CHCFG_0;
    union {
        unsigned long LONG;
        struct {
            unsigned long ITVL:16;
            unsigned long :16;
        } BIT;
    } DMAC1_CHITVL_0;
    char           wk0[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long NXLA:32;
        } BIT;
    } DMAC1_NXLA_0;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRLA:32;
        } BIT;
    } DMAC1_CRLA_0;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC1_N0SA_1_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC1_N0SA_1_W;
    } N0SA_1;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC1_N0DA_1;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC1_N0TB_1;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC1_N1SA_1_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC1_N1SA_1_W;
    } N1SA_1;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC1_N1DA_1;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC1_N1TB_1;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRSA:32;
        } BIT;
    } DMAC1_CRSA_1;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRDA:32;
        } BIT;
    } DMAC1_CRDA_1;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRTB:32;
        } BIT;
    } DMAC1_CRTB_1;
    union {
        unsigned long LONG;
        struct {
            unsigned long EN:1;
            unsigned long RQST:1;
            unsigned long TACT:1;
            unsigned long SUS:1;
            unsigned long ER:1;
            unsigned long END:1;
            unsigned long :1;
            unsigned long SR:1;
            unsigned long DL:1;
            unsigned long DW:1;
            unsigned long DER:1;
            unsigned long MODE:1;
            unsigned long :4;
            unsigned long INTM:1;
            unsigned long DMARQM:1;
            unsigned long SWPRQ:1;
            unsigned long :5;
            unsigned long DNUM:8;
        } BIT;
    } DMAC1_CHSTAT_1;
    union {
        unsigned long LONG;
        struct {
            unsigned long SETEN:1;
            unsigned long CLREN:1;
            unsigned long :1;
            unsigned long SWRST:1;
            unsigned long CLRRQ:1;
            unsigned long CLREND:1;
            unsigned long :1;
            unsigned long CLRDE:1;
            unsigned long SETSUS:1;
            unsigned long CLRSUS:1;
            unsigned long :2;
            unsigned long SETREN:1;
            unsigned long :1;
            unsigned long SETSSWPRQ:1;
            unsigned long :1;
            unsigned long SETINTM:1;
            unsigned long CLRINTM:1;
            unsigned long SETDMARQM:1;
            unsigned long CLRDMARQM:1;
            unsigned long :12;
        } BIT;
    } DMAC1_CHCTRL_1;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEL:3;
            unsigned long REQD:1;
            unsigned long LOEN:1;
            unsigned long HIEN:1;
            unsigned long LVL:1;
            unsigned long :1;
            unsigned long AM:3;
            unsigned long DRRP:1;
            unsigned long SDS:4;
            unsigned long DDS:4;
            unsigned long SAD:1;
            unsigned long DAD:1;
            unsigned long TM:1;
            unsigned long WONLY:1;
            unsigned long DEM:1;
            unsigned long :1;
            unsigned long DIM:1;
            unsigned long SBE:1;
            unsigned long RSEL:1;
            unsigned long RSW:1;
            unsigned long REN:1;
            unsigned long DMS:1;
        } BIT;
    } DMAC1_CHCFG_1;
    union {
        unsigned long LONG;
        struct {
            unsigned long ITVL:16;
            unsigned long :16;
        } BIT;
    } DMAC1_CHITVL_1;
    char           wk1[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long NXLA:32;
        } BIT;
    } DMAC1_NXLA_1;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRLA:32;
        } BIT;
    } DMAC1_CRLA_1;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC1_N0SA_2_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC1_N0SA_2_W;
    } N0SA_2;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC1_N0DA_2;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC1_N0TB_2;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC1_N1SA_2_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC1_N1SA_2_W;
    } N1SA_2;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC1_N1DA_2;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC1_N1TB_2;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRSA:32;
        } BIT;
    } DMAC1_CRSA_2;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRDA:32;
        } BIT;
    } DMAC1_CRDA_2;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRTB:32;
        } BIT;
    } DMAC1_CRTB_2;
    union {
        unsigned long LONG;
        struct {
            unsigned long EN:1;
            unsigned long RQST:1;
            unsigned long TACT:1;
            unsigned long SUS:1;
            unsigned long ER:1;
            unsigned long END:1;
            unsigned long :1;
            unsigned long SR:1;
            unsigned long DL:1;
            unsigned long DW:1;
            unsigned long DER:1;
            unsigned long MODE:1;
            unsigned long :4;
            unsigned long INTM:1;
            unsigned long DMARQM:1;
            unsigned long SWPRQ:1;
            unsigned long :5;
            unsigned long DNUM:8;
        } BIT;
    } DMAC1_CHSTAT_2;
    union {
        unsigned long LONG;
        struct {
            unsigned long SETEN:1;
            unsigned long CLREN:1;
            unsigned long :1;
            unsigned long SWRST:1;
            unsigned long CLRRQ:1;
            unsigned long CLREND:1;
            unsigned long :1;
            unsigned long CLRDE:1;
            unsigned long SETSUS:1;
            unsigned long CLRSUS:1;
            unsigned long :2;
            unsigned long SETREN:1;
            unsigned long :1;
            unsigned long SETSSWPRQ:1;
            unsigned long :1;
            unsigned long SETINTM:1;
            unsigned long CLRINTM:1;
            unsigned long SETDMARQM:1;
            unsigned long CLRDMARQM:1;
            unsigned long :12;
        } BIT;
    } DMAC1_CHCTRL_2;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEL:3;
            unsigned long REQD:1;
            unsigned long LOEN:1;
            unsigned long HIEN:1;
            unsigned long LVL:1;
            unsigned long :1;
            unsigned long AM:3;
            unsigned long DRRP:1;
            unsigned long SDS:4;
            unsigned long DDS:4;
            unsigned long SAD:1;
            unsigned long DAD:1;
            unsigned long TM:1;
            unsigned long WONLY:1;
            unsigned long DEM:1;
            unsigned long :1;
            unsigned long DIM:1;
            unsigned long SBE:1;
            unsigned long RSEL:1;
            unsigned long RSW:1;
            unsigned long REN:1;
            unsigned long DMS:1;
        } BIT;
    } DMAC1_CHCFG_2;
    union {
        unsigned long LONG;
        struct {
            unsigned long ITVL:16;
            unsigned long :16;
        } BIT;
    } DMAC1_CHITVL_2;
    char           wk2[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long NXLA:32;
        } BIT;
    } DMAC1_NXLA_2;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRLA:32;
        } BIT;
    } DMAC1_CRLA_2;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC1_N0SA_3_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC1_N0SA_3_W;
    } N0SA_3;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC1_N0DA_3;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC1_N0TB_3;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC1_N1SA_3_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC1_N1SA_3_W;
    } N1SA_3;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC1_N1DA_3;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC1_N1TB_3;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRSA:32;
        } BIT;
    } DMAC1_CRSA_3;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRDA:32;
        } BIT;
    } DMAC1_CRDA_3;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRTB:32;
        } BIT;
    } DMAC1_CRTB_3;
    union {
        unsigned long LONG;
        struct {
            unsigned long EN:1;
            unsigned long RQST:1;
            unsigned long TACT:1;
            unsigned long SUS:1;
            unsigned long ER:1;
            unsigned long END:1;
            unsigned long :1;
            unsigned long SR:1;
            unsigned long DL:1;
            unsigned long DW:1;
            unsigned long DER:1;
            unsigned long MODE:1;
            unsigned long :4;
            unsigned long INTM:1;
            unsigned long DMARQM:1;
            unsigned long SWPRQ:1;
            unsigned long :5;
            unsigned long DNUM:8;
        } BIT;
    } DMAC1_CHSTAT_3;
    union {
        unsigned long LONG;
        struct {
            unsigned long SETEN:1;
            unsigned long CLREN:1;
            unsigned long :1;
            unsigned long SWRST:1;
            unsigned long CLRRQ:1;
            unsigned long CLREND:1;
            unsigned long :1;
            unsigned long CLRDE:1;
            unsigned long SETSUS:1;
            unsigned long CLRSUS:1;
            unsigned long :2;
            unsigned long SETREN:1;
            unsigned long :1;
            unsigned long SETSSWPRQ:1;
            unsigned long :1;
            unsigned long SETINTM:1;
            unsigned long CLRINTM:1;
            unsigned long SETDMARQM:1;
            unsigned long CLRDMARQM:1;
            unsigned long :12;
        } BIT;
    } DMAC1_CHCTRL_3;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEL:3;
            unsigned long REQD:1;
            unsigned long LOEN:1;
            unsigned long HIEN:1;
            unsigned long LVL:1;
            unsigned long :1;
            unsigned long AM:3;
            unsigned long DRRP:1;
            unsigned long SDS:4;
            unsigned long DDS:4;
            unsigned long SAD:1;
            unsigned long DAD:1;
            unsigned long TM:1;
            unsigned long WONLY:1;
            unsigned long DEM:1;
            unsigned long :1;
            unsigned long DIM:1;
            unsigned long SBE:1;
            unsigned long RSEL:1;
            unsigned long RSW:1;
            unsigned long REN:1;
            unsigned long DMS:1;
        } BIT;
    } DMAC1_CHCFG_3;
    union {
        unsigned long LONG;
        struct {
            unsigned long ITVL:16;
            unsigned long :16;
        } BIT;
    } DMAC1_CHITVL_3;
    char           wk3[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long NXLA:32;
        } BIT;
    } DMAC1_NXLA_3;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRLA:32;
        } BIT;
    } DMAC1_CRLA_3;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC1_N0SA_4_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC1_N0SA_4_W;
    } N0SA_4;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC1_N0DA_4;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC1_N0TB_4;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC1_N1SA_4_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC1_N1SA_4_W;
    } N1SA_4;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC1_N1DA_4;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC1_N1TB_4;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRSA:32;
        } BIT;
    } DMAC1_CRSA_4;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRDA:32;
        } BIT;
    } DMAC1_CRDA_4;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRTB:32;
        } BIT;
    } DMAC1_CRTB_4;
    union {
        unsigned long LONG;
        struct {
            unsigned long EN:1;
            unsigned long RQST:1;
            unsigned long TACT:1;
            unsigned long SUS:1;
            unsigned long ER:1;
            unsigned long END:1;
            unsigned long :1;
            unsigned long SR:1;
            unsigned long DL:1;
            unsigned long DW:1;
            unsigned long DER:1;
            unsigned long MODE:1;
            unsigned long :4;
            unsigned long INTM:1;
            unsigned long DMARQM:1;
            unsigned long SWPRQ:1;
            unsigned long :5;
            unsigned long DNUM:8;
        } BIT;
    } DMAC1_CHSTAT_4;
    union {
        unsigned long LONG;
        struct {
            unsigned long SETEN:1;
            unsigned long CLREN:1;
            unsigned long :1;
            unsigned long SWRST:1;
            unsigned long CLRRQ:1;
            unsigned long CLREND:1;
            unsigned long :1;
            unsigned long CLRDE:1;
            unsigned long SETSUS:1;
            unsigned long CLRSUS:1;
            unsigned long :2;
            unsigned long SETREN:1;
            unsigned long :1;
            unsigned long SETSSWPRQ:1;
            unsigned long :1;
            unsigned long SETINTM:1;
            unsigned long CLRINTM:1;
            unsigned long SETDMARQM:1;
            unsigned long CLRDMARQM:1;
            unsigned long :12;
        } BIT;
    } DMAC1_CHCTRL_4;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEL:3;
            unsigned long REQD:1;
            unsigned long LOEN:1;
            unsigned long HIEN:1;
            unsigned long LVL:1;
            unsigned long :1;
            unsigned long AM:3;
            unsigned long DRRP:1;
            unsigned long SDS:4;
            unsigned long DDS:4;
            unsigned long SAD:1;
            unsigned long DAD:1;
            unsigned long TM:1;
            unsigned long WONLY:1;
            unsigned long DEM:1;
            unsigned long :1;
            unsigned long DIM:1;
            unsigned long SBE:1;
            unsigned long RSEL:1;
            unsigned long RSW:1;
            unsigned long REN:1;
            unsigned long DMS:1;
        } BIT;
    } DMAC1_CHCFG_4;
    union {
        unsigned long LONG;
        struct {
            unsigned long ITVL:16;
            unsigned long :16;
        } BIT;
    } DMAC1_CHITVL_4;
    char           wk4[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long NXLA:32;
        } BIT;
    } DMAC1_NXLA_4;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRLA:32;
        } BIT;
    } DMAC1_CRLA_4;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC1_N0SA_5_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC1_N0SA_5_W;
    } N0SA_5;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC1_N0DA_5;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC1_N0TB_5;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC1_N1SA_5_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC1_N1SA_5_W;
    } N1SA_5;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC1_N1DA_5;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC1_N1TB_5;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRSA:32;
        } BIT;
    } DMAC1_CRSA_5;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRDA:32;
        } BIT;
    } DMAC1_CRDA_5;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRTB:32;
        } BIT;
    } DMAC1_CRTB_5;
    union {
        unsigned long LONG;
        struct {
            unsigned long EN:1;
            unsigned long RQST:1;
            unsigned long TACT:1;
            unsigned long SUS:1;
            unsigned long ER:1;
            unsigned long END:1;
            unsigned long :1;
            unsigned long SR:1;
            unsigned long DL:1;
            unsigned long DW:1;
            unsigned long DER:1;
            unsigned long MODE:1;
            unsigned long :4;
            unsigned long INTM:1;
            unsigned long DMARQM:1;
            unsigned long SWPRQ:1;
            unsigned long :5;
            unsigned long DNUM:8;
        } BIT;
    } DMAC1_CHSTAT_5;
    union {
        unsigned long LONG;
        struct {
            unsigned long SETEN:1;
            unsigned long CLREN:1;
            unsigned long :1;
            unsigned long SWRST:1;
            unsigned long CLRRQ:1;
            unsigned long CLREND:1;
            unsigned long :1;
            unsigned long CLRDE:1;
            unsigned long SETSUS:1;
            unsigned long CLRSUS:1;
            unsigned long :2;
            unsigned long SETREN:1;
            unsigned long :1;
            unsigned long SETSSWPRQ:1;
            unsigned long :1;
            unsigned long SETINTM:1;
            unsigned long CLRINTM:1;
            unsigned long SETDMARQM:1;
            unsigned long CLRDMARQM:1;
            unsigned long :12;
        } BIT;
    } DMAC1_CHCTRL_5;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEL:3;
            unsigned long REQD:1;
            unsigned long LOEN:1;
            unsigned long HIEN:1;
            unsigned long LVL:1;
            unsigned long :1;
            unsigned long AM:3;
            unsigned long DRRP:1;
            unsigned long SDS:4;
            unsigned long DDS:4;
            unsigned long SAD:1;
            unsigned long DAD:1;
            unsigned long TM:1;
            unsigned long WONLY:1;
            unsigned long DEM:1;
            unsigned long :1;
            unsigned long DIM:1;
            unsigned long SBE:1;
            unsigned long RSEL:1;
            unsigned long RSW:1;
            unsigned long REN:1;
            unsigned long DMS:1;
        } BIT;
    } DMAC1_CHCFG_5;
    union {
        unsigned long LONG;
        struct {
            unsigned long ITVL:16;
            unsigned long :16;
        } BIT;
    } DMAC1_CHITVL_5;
    char           wk5[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long NXLA:32;
        } BIT;
    } DMAC1_NXLA_5;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRLA:32;
        } BIT;
    } DMAC1_CRLA_5;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC1_N0SA_6_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC1_N0SA_6_W;
    } N0SA_6;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC1_N0DA_6;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC1_N0TB_6;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC1_N1SA_6_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC1_N1SA_6_W;
    } N1SA_6;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC1_N1DA_6;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC1_N1TB_6;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRSA:32;
        } BIT;
    } DMAC1_CRSA_6;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRDA:32;
        } BIT;
    } DMAC1_CRDA_6;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRTB:32;
        } BIT;
    } DMAC1_CRTB_6;
    union {
        unsigned long LONG;
        struct {
            unsigned long EN:1;
            unsigned long RQST:1;
            unsigned long TACT:1;
            unsigned long SUS:1;
            unsigned long ER:1;
            unsigned long END:1;
            unsigned long :1;
            unsigned long SR:1;
            unsigned long DL:1;
            unsigned long DW:1;
            unsigned long DER:1;
            unsigned long MODE:1;
            unsigned long :4;
            unsigned long INTM:1;
            unsigned long DMARQM:1;
            unsigned long SWPRQ:1;
            unsigned long :5;
            unsigned long DNUM:8;
        } BIT;
    } DMAC1_CHSTAT_6;
    union {
        unsigned long LONG;
        struct {
            unsigned long SETEN:1;
            unsigned long CLREN:1;
            unsigned long :1;
            unsigned long SWRST:1;
            unsigned long CLRRQ:1;
            unsigned long CLREND:1;
            unsigned long :1;
            unsigned long CLRDE:1;
            unsigned long SETSUS:1;
            unsigned long CLRSUS:1;
            unsigned long :2;
            unsigned long SETREN:1;
            unsigned long :1;
            unsigned long SETSSWPRQ:1;
            unsigned long :1;
            unsigned long SETINTM:1;
            unsigned long CLRINTM:1;
            unsigned long SETDMARQM:1;
            unsigned long CLRDMARQM:1;
            unsigned long :12;
        } BIT;
    } DMAC1_CHCTRL_6;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEL:3;
            unsigned long REQD:1;
            unsigned long LOEN:1;
            unsigned long HIEN:1;
            unsigned long LVL:1;
            unsigned long :1;
            unsigned long AM:3;
            unsigned long DRRP:1;
            unsigned long SDS:4;
            unsigned long DDS:4;
            unsigned long SAD:1;
            unsigned long DAD:1;
            unsigned long TM:1;
            unsigned long WONLY:1;
            unsigned long DEM:1;
            unsigned long :1;
            unsigned long DIM:1;
            unsigned long SBE:1;
            unsigned long RSEL:1;
            unsigned long RSW:1;
            unsigned long REN:1;
            unsigned long DMS:1;
        } BIT;
    } DMAC1_CHCFG_6;
    union {
        unsigned long LONG;
        struct {
            unsigned long ITVL:16;
            unsigned long :16;
        } BIT;
    } DMAC1_CHITVL_6;
    char           wk6[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long NXLA:32;
        } BIT;
    } DMAC1_NXLA_6;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRLA:32;
        } BIT;
    } DMAC1_CRLA_6;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC1_N0SA_7_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC1_N0SA_7_W;
    } N0SA_7;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC1_N0DA_7;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC1_N0TB_7;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC1_N1SA_7_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC1_N1SA_7_W;
    } N1SA_7;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC1_N1DA_7;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC1_N1TB_7;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRSA:32;
        } BIT;
    } DMAC1_CRSA_7;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRDA:32;
        } BIT;
    } DMAC1_CRDA_7;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRTB:32;
        } BIT;
    } DMAC1_CRTB_7;
    union {
        unsigned long LONG;
        struct {
            unsigned long EN:1;
            unsigned long RQST:1;
            unsigned long TACT:1;
            unsigned long SUS:1;
            unsigned long ER:1;
            unsigned long END:1;
            unsigned long :1;
            unsigned long SR:1;
            unsigned long DL:1;
            unsigned long DW:1;
            unsigned long DER:1;
            unsigned long MODE:1;
            unsigned long :4;
            unsigned long INTM:1;
            unsigned long DMARQM:1;
            unsigned long SWPRQ:1;
            unsigned long :5;
            unsigned long DNUM:8;
        } BIT;
    } DMAC1_CHSTAT_7;
    union {
        unsigned long LONG;
        struct {
            unsigned long SETEN:1;
            unsigned long CLREN:1;
            unsigned long :1;
            unsigned long SWRST:1;
            unsigned long CLRRQ:1;
            unsigned long CLREND:1;
            unsigned long :1;
            unsigned long CLRDE:1;
            unsigned long SETSUS:1;
            unsigned long CLRSUS:1;
            unsigned long :2;
            unsigned long SETREN:1;
            unsigned long :1;
            unsigned long SETSSWPRQ:1;
            unsigned long :1;
            unsigned long SETINTM:1;
            unsigned long CLRINTM:1;
            unsigned long SETDMARQM:1;
            unsigned long CLRDMARQM:1;
            unsigned long :12;
        } BIT;
    } DMAC1_CHCTRL_7;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEL:3;
            unsigned long REQD:1;
            unsigned long LOEN:1;
            unsigned long HIEN:1;
            unsigned long LVL:1;
            unsigned long :1;
            unsigned long AM:3;
            unsigned long DRRP:1;
            unsigned long SDS:4;
            unsigned long DDS:4;
            unsigned long SAD:1;
            unsigned long DAD:1;
            unsigned long TM:1;
            unsigned long WONLY:1;
            unsigned long DEM:1;
            unsigned long :1;
            unsigned long DIM:1;
            unsigned long SBE:1;
            unsigned long RSEL:1;
            unsigned long RSW:1;
            unsigned long REN:1;
            unsigned long DMS:1;
        } BIT;
    } DMAC1_CHCFG_7;
    union {
        unsigned long LONG;
        struct {
            unsigned long ITVL:16;
            unsigned long :16;
        } BIT;
    } DMAC1_CHITVL_7;
    char           wk7[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long NXLA:32;
        } BIT;
    } DMAC1_NXLA_7;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRLA:32;
        } BIT;
    } DMAC1_CRLA_7;
    union {
        unsigned long LONG;
        struct {
            unsigned long SCNT:32;
        } BIT;
    } DMAC1_SCNT_0;
    union {
        unsigned long LONG;
        struct {
            unsigned long SSKP:32;
        } BIT;
    } DMAC1_SSKP_0;
    union {
        unsigned long LONG;
        struct {
            unsigned long DCNT:32;
        } BIT;
    } DMAC1_DCNT_0;
    union {
        unsigned long LONG;
        struct {
            unsigned long DSKP:32;
        } BIT;
    } DMAC1_DSKP_0;
    char           wk8[16];
    union {
        unsigned long LONG;
        struct {
            unsigned long SCNT:32;
        } BIT;
    } DMAC1_SCNT_1;
    union {
        unsigned long LONG;
        struct {
            unsigned long SSKP:32;
        } BIT;
    } DMAC1_SSKP_1;
    union {
        unsigned long LONG;
        struct {
            unsigned long DCNT:32;
        } BIT;
    } DMAC1_DCNT_1;
    union {
        unsigned long LONG;
        struct {
            unsigned long DSKP:32;
        } BIT;
    } DMAC1_DSKP_1;
    char           wk9[16];
    union {
        unsigned long LONG;
        struct {
            unsigned long SCNT:32;
        } BIT;
    } DMAC1_SCNT_2;
    union {
        unsigned long LONG;
        struct {
            unsigned long SSKP:32;
        } BIT;
    } DMAC1_SSKP_2;
    union {
        unsigned long LONG;
        struct {
            unsigned long DCNT:32;
        } BIT;
    } DMAC1_DCNT_2;
    union {
        unsigned long LONG;
        struct {
            unsigned long DSKP:32;
        } BIT;
    } DMAC1_DSKP_2;
    char           wk10[16];
    union {
        unsigned long LONG;
        struct {
            unsigned long SCNT:32;
        } BIT;
    } DMAC1_SCNT_3;
    union {
        unsigned long LONG;
        struct {
            unsigned long SSKP:32;
        } BIT;
    } DMAC1_SSKP_3;
    union {
        unsigned long LONG;
        struct {
            unsigned long DCNT:32;
        } BIT;
    } DMAC1_DCNT_3;
    union {
        unsigned long LONG;
        struct {
            unsigned long DSKP:32;
        } BIT;
    } DMAC1_DSKP_3;
    char           wk11[16];
    union {
        unsigned long LONG;
        struct {
            unsigned long SCNT:32;
        } BIT;
    } DMAC1_SCNT_4;
    union {
        unsigned long LONG;
        struct {
            unsigned long SSKP:32;
        } BIT;
    } DMAC1_SSKP_4;
    union {
        unsigned long LONG;
        struct {
            unsigned long DCNT:32;
        } BIT;
    } DMAC1_DCNT_4;
    union {
        unsigned long LONG;
        struct {
            unsigned long DSKP:32;
        } BIT;
    } DMAC1_DSKP_4;
    char           wk12[16];
    union {
        unsigned long LONG;
        struct {
            unsigned long SCNT:32;
        } BIT;
    } DMAC1_SCNT_5;
    union {
        unsigned long LONG;
        struct {
            unsigned long SSKP:32;
        } BIT;
    } DMAC1_SSKP_5;
    union {
        unsigned long LONG;
        struct {
            unsigned long DCNT:32;
        } BIT;
    } DMAC1_DCNT_5;
    union {
        unsigned long LONG;
        struct {
            unsigned long DSKP:32;
        } BIT;
    } DMAC1_DSKP_5;
    char           wk13[16];
    union {
        unsigned long LONG;
        struct {
            unsigned long SCNT:32;
        } BIT;
    } DMAC1_SCNT_6;
    union {
        unsigned long LONG;
        struct {
            unsigned long SSKP:32;
        } BIT;
    } DMAC1_SSKP_6;
    union {
        unsigned long LONG;
        struct {
            unsigned long DCNT:32;
        } BIT;
    } DMAC1_DCNT_6;
    union {
        unsigned long LONG;
        struct {
            unsigned long DSKP:32;
        } BIT;
    } DMAC1_DSKP_6;
    char           wk14[16];
    union {
        unsigned long LONG;
        struct {
            unsigned long SCNT:32;
        } BIT;
    } DMAC1_SCNT_7;
    union {
        unsigned long LONG;
        struct {
            unsigned long SSKP:32;
        } BIT;
    } DMAC1_SSKP_7;
    union {
        unsigned long LONG;
        struct {
            unsigned long DCNT:32;
        } BIT;
    } DMAC1_DCNT_7;
    union {
        unsigned long LONG;
        struct {
            unsigned long DSKP:32;
        } BIT;
    } DMAC1_DSKP_7;
    char           wk15[16];
    union {
        unsigned long LONG;
        struct {
            unsigned long PR:1;
            unsigned long :31;
        } BIT;
    } DMAC1_DCTRL_A;
    union {
        unsigned long LONG;
        struct {
            unsigned long :8;
            unsigned long DITVL:8;
            unsigned long :16;
        } BIT;
    } DMAC1_DSCITVL_A;
    char           wk16[8];
    union {
        unsigned long LONG;
        struct {
            unsigned long EN08:1;
            unsigned long EN19:1;
            unsigned long EN210:1;
            unsigned long EN311:1;
            unsigned long EN412:1;
            unsigned long EN513:1;
            unsigned long EN614:1;
            unsigned long EN715:1;
            unsigned long :24;
        } BIT;
    } DMAC1_DST_EN_A;
    union {
        unsigned long LONG;
        struct {
            unsigned long ER08:1;
            unsigned long ER19:1;
            unsigned long ER210:1;
            unsigned long ER311:1;
            unsigned long ER412:1;
            unsigned long ER513:1;
            unsigned long ER614:1;
            unsigned long ER715:1;
            unsigned long :24;
        } BIT;
    } DMAC1_DST_ER_A;
    union {
        unsigned long LONG;
        struct {
            unsigned long END08:1;
            unsigned long END19:1;
            unsigned long END210:1;
            unsigned long END311:1;
            unsigned long END412:1;
            unsigned long END513:1;
            unsigned long END614:1;
            unsigned long END715:1;
            unsigned long :24;
        } BIT;
    } DMAC1_DST_END_A;
    char           wk17[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long SUS08:1;
            unsigned long SUS19:1;
            unsigned long SUS210:1;
            unsigned long SUS311:1;
            unsigned long SUS412:1;
            unsigned long SUS513:1;
            unsigned long SUS614:1;
            unsigned long SUS715:1;
            unsigned long :24;
        } BIT;
    } DMAC1_DST_SUS_A;
    char           wk18[220];
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC1_N0SA_8_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC1_N0SA_8_W;
    } N0SA_8;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC1_N0DA_8;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC1_N0TB_8;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC1_N1SA_8_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC1_N1SA_8_W;
    } N1SA_8;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC1_N1DA_8;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC1_N1TB_8;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRSA:32;
        } BIT;
    } DMAC1_CRSA_8;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRDA:32;
        } BIT;
    } DMAC1_CRDA_8;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRTB:32;
        } BIT;
    } DMAC1_CRTB_8;
    union {
        unsigned long LONG;
        struct {
            unsigned long EN:1;
            unsigned long RQST:1;
            unsigned long TACT:1;
            unsigned long SUS:1;
            unsigned long ER:1;
            unsigned long END:1;
            unsigned long :1;
            unsigned long SR:1;
            unsigned long DL:1;
            unsigned long DW:1;
            unsigned long DER:1;
            unsigned long MODE:1;
            unsigned long :4;
            unsigned long INTM:1;
            unsigned long DMARQM:1;
            unsigned long SWPRQ:1;
            unsigned long :5;
            unsigned long DNUM:8;
        } BIT;
    } DMAC1_CHSTAT_8;
    union {
        unsigned long LONG;
        struct {
            unsigned long SETEN:1;
            unsigned long CLREN:1;
            unsigned long :1;
            unsigned long SWRST:1;
            unsigned long CLRRQ:1;
            unsigned long CLREND:1;
            unsigned long :1;
            unsigned long CLRDE:1;
            unsigned long SETSUS:1;
            unsigned long CLRSUS:1;
            unsigned long :2;
            unsigned long SETREN:1;
            unsigned long :1;
            unsigned long SETSSWPRQ:1;
            unsigned long :1;
            unsigned long SETINTM:1;
            unsigned long CLRINTM:1;
            unsigned long SETDMARQM:1;
            unsigned long CLRDMARQM:1;
            unsigned long :12;
        } BIT;
    } DMAC1_CHCTRL_8;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEL:3;
            unsigned long REQD:1;
            unsigned long LOEN:1;
            unsigned long HIEN:1;
            unsigned long LVL:1;
            unsigned long :1;
            unsigned long AM:3;
            unsigned long DRRP:1;
            unsigned long SDS:4;
            unsigned long DDS:4;
            unsigned long SAD:1;
            unsigned long DAD:1;
            unsigned long TM:1;
            unsigned long WONLY:1;
            unsigned long DEM:1;
            unsigned long :1;
            unsigned long DIM:1;
            unsigned long SBE:1;
            unsigned long RSEL:1;
            unsigned long RSW:1;
            unsigned long REN:1;
            unsigned long DMS:1;
        } BIT;
    } DMAC1_CHCFG_8;
    union {
        unsigned long LONG;
        struct {
            unsigned long ITVL:16;
            unsigned long :16;
        } BIT;
    } DMAC1_CHITVL_8;
    char           wk19[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long NXLA:32;
        } BIT;
    } DMAC1_NXLA_8;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRLA:32;
        } BIT;
    } DMAC1_CRLA_8;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC1_N0SA_9_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC1_N0SA_9_W;
    } N0SA_9;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC1_N0DA_9;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC1_N0TB_9;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC1_N1SA_9_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC1_N1SA_9_W;
    } N1SA_9;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC1_N1DA_9;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC1_N1TB_9;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRSA:32;
        } BIT;
    } DMAC1_CRSA_9;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRDA:32;
        } BIT;
    } DMAC1_CRDA_9;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRTB:32;
        } BIT;
    } DMAC1_CRTB_9;
    union {
        unsigned long LONG;
        struct {
            unsigned long EN:1;
            unsigned long RQST:1;
            unsigned long TACT:1;
            unsigned long SUS:1;
            unsigned long ER:1;
            unsigned long END:1;
            unsigned long :1;
            unsigned long SR:1;
            unsigned long DL:1;
            unsigned long DW:1;
            unsigned long DER:1;
            unsigned long MODE:1;
            unsigned long :4;
            unsigned long INTM:1;
            unsigned long DMARQM:1;
            unsigned long SWPRQ:1;
            unsigned long :5;
            unsigned long DNUM:8;
        } BIT;
    } DMAC1_CHSTAT_9;
    union {
        unsigned long LONG;
        struct {
            unsigned long SETEN:1;
            unsigned long CLREN:1;
            unsigned long :1;
            unsigned long SWRST:1;
            unsigned long CLRRQ:1;
            unsigned long CLREND:1;
            unsigned long :1;
            unsigned long CLRDE:1;
            unsigned long SETSUS:1;
            unsigned long CLRSUS:1;
            unsigned long :2;
            unsigned long SETREN:1;
            unsigned long :1;
            unsigned long SETSSWPRQ:1;
            unsigned long :1;
            unsigned long SETINTM:1;
            unsigned long CLRINTM:1;
            unsigned long SETDMARQM:1;
            unsigned long CLRDMARQM:1;
            unsigned long :12;
        } BIT;
    } DMAC1_CHCTRL_9;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEL:3;
            unsigned long REQD:1;
            unsigned long LOEN:1;
            unsigned long HIEN:1;
            unsigned long LVL:1;
            unsigned long :1;
            unsigned long AM:3;
            unsigned long DRRP:1;
            unsigned long SDS:4;
            unsigned long DDS:4;
            unsigned long SAD:1;
            unsigned long DAD:1;
            unsigned long TM:1;
            unsigned long WONLY:1;
            unsigned long DEM:1;
            unsigned long :1;
            unsigned long DIM:1;
            unsigned long SBE:1;
            unsigned long RSEL:1;
            unsigned long RSW:1;
            unsigned long REN:1;
            unsigned long DMS:1;
        } BIT;
    } DMAC1_CHCFG_9;
    union {
        unsigned long LONG;
        struct {
            unsigned long ITVL:16;
            unsigned long :16;
        } BIT;
    } DMAC1_CHITVL_9;
    char           wk20[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long NXLA:32;
        } BIT;
    } DMAC1_NXLA_9;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRLA:32;
        } BIT;
    } DMAC1_CRLA_9;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC1_N0SA_10_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC1_N0SA_10_W;
    } N0SA_10;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC1_N0DA_10;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC1_N0TB_10;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC1_N1SA_10_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC1_N1SA_10_W;
    } N1SA_10;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC1_N1DA_10;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC1_N1TB_10;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRSA:32;
        } BIT;
    } DMAC1_CRSA_10;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRDA:32;
        } BIT;
    } DMAC1_CRDA_10;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRTB:32;
        } BIT;
    } DMAC1_CRTB_10;
    union {
        unsigned long LONG;
        struct {
            unsigned long EN:1;
            unsigned long RQST:1;
            unsigned long TACT:1;
            unsigned long SUS:1;
            unsigned long ER:1;
            unsigned long END:1;
            unsigned long :1;
            unsigned long SR:1;
            unsigned long DL:1;
            unsigned long DW:1;
            unsigned long DER:1;
            unsigned long MODE:1;
            unsigned long :4;
            unsigned long INTM:1;
            unsigned long DMARQM:1;
            unsigned long SWPRQ:1;
            unsigned long :5;
            unsigned long DNUM:8;
        } BIT;
    } DMAC1_CHSTAT_10;
    union {
        unsigned long LONG;
        struct {
            unsigned long SETEN:1;
            unsigned long CLREN:1;
            unsigned long :1;
            unsigned long SWRST:1;
            unsigned long CLRRQ:1;
            unsigned long CLREND:1;
            unsigned long :1;
            unsigned long CLRDE:1;
            unsigned long SETSUS:1;
            unsigned long CLRSUS:1;
            unsigned long :2;
            unsigned long SETREN:1;
            unsigned long :1;
            unsigned long SETSSWPRQ:1;
            unsigned long :1;
            unsigned long SETINTM:1;
            unsigned long CLRINTM:1;
            unsigned long SETDMARQM:1;
            unsigned long CLRDMARQM:1;
            unsigned long :12;
        } BIT;
    } DMAC1_CHCTRL_10;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEL:3;
            unsigned long REQD:1;
            unsigned long LOEN:1;
            unsigned long HIEN:1;
            unsigned long LVL:1;
            unsigned long :1;
            unsigned long AM:3;
            unsigned long DRRP:1;
            unsigned long SDS:4;
            unsigned long DDS:4;
            unsigned long SAD:1;
            unsigned long DAD:1;
            unsigned long TM:1;
            unsigned long WONLY:1;
            unsigned long DEM:1;
            unsigned long :1;
            unsigned long DIM:1;
            unsigned long SBE:1;
            unsigned long RSEL:1;
            unsigned long RSW:1;
            unsigned long REN:1;
            unsigned long DMS:1;
        } BIT;
    } DMAC1_CHCFG_10;
    union {
        unsigned long LONG;
        struct {
            unsigned long ITVL:16;
            unsigned long :16;
        } BIT;
    } DMAC1_CHITVL_10;
    char           wk21[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long NXLA:32;
        } BIT;
    } DMAC1_NXLA_10;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRLA:32;
        } BIT;
    } DMAC1_CRLA_10;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC1_N0SA_11_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC1_N0SA_11_W;
    } N0SA_11;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC1_N0DA_11;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC1_N0TB_11;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC1_N1SA_11_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC1_N1SA_11_W;
    } N1SA_11;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC1_N1DA_11;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC1_N1TB_11;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRSA:32;
        } BIT;
    } DMAC1_CRSA_11;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRDA:32;
        } BIT;
    } DMAC1_CRDA_11;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRTB:32;
        } BIT;
    } DMAC1_CRTB_11;
    union {
        unsigned long LONG;
        struct {
            unsigned long EN:1;
            unsigned long RQST:1;
            unsigned long TACT:1;
            unsigned long SUS:1;
            unsigned long ER:1;
            unsigned long END:1;
            unsigned long :1;
            unsigned long SR:1;
            unsigned long DL:1;
            unsigned long DW:1;
            unsigned long DER:1;
            unsigned long MODE:1;
            unsigned long :4;
            unsigned long INTM:1;
            unsigned long DMARQM:1;
            unsigned long SWPRQ:1;
            unsigned long :5;
            unsigned long DNUM:8;
        } BIT;
    } DMAC1_CHSTAT_11;
    union {
        unsigned long LONG;
        struct {
            unsigned long SETEN:1;
            unsigned long CLREN:1;
            unsigned long :1;
            unsigned long SWRST:1;
            unsigned long CLRRQ:1;
            unsigned long CLREND:1;
            unsigned long :1;
            unsigned long CLRDE:1;
            unsigned long SETSUS:1;
            unsigned long CLRSUS:1;
            unsigned long :2;
            unsigned long SETREN:1;
            unsigned long :1;
            unsigned long SETSSWPRQ:1;
            unsigned long :1;
            unsigned long SETINTM:1;
            unsigned long CLRINTM:1;
            unsigned long SETDMARQM:1;
            unsigned long CLRDMARQM:1;
            unsigned long :12;
        } BIT;
    } DMAC1_CHCTRL_11;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEL:3;
            unsigned long REQD:1;
            unsigned long LOEN:1;
            unsigned long HIEN:1;
            unsigned long LVL:1;
            unsigned long :1;
            unsigned long AM:3;
            unsigned long DRRP:1;
            unsigned long SDS:4;
            unsigned long DDS:4;
            unsigned long SAD:1;
            unsigned long DAD:1;
            unsigned long TM:1;
            unsigned long WONLY:1;
            unsigned long DEM:1;
            unsigned long :1;
            unsigned long DIM:1;
            unsigned long SBE:1;
            unsigned long RSEL:1;
            unsigned long RSW:1;
            unsigned long REN:1;
            unsigned long DMS:1;
        } BIT;
    } DMAC1_CHCFG_11;
    union {
        unsigned long LONG;
        struct {
            unsigned long ITVL:16;
            unsigned long :16;
        } BIT;
    } DMAC1_CHITVL_11;
    char           wk22[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long NXLA:32;
        } BIT;
    } DMAC1_NXLA_11;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRLA:32;
        } BIT;
    } DMAC1_CRLA_11;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC1_N0SA_12_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC1_N0SA_12_W;
    } N0SA_12;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC1_N0DA_12;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC1_N0TB_12;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC1_N1SA_12_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC1_N1SA_12_W;
    } N1SA_12;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC1_N1DA_12;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC1_N1TB_12;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRSA:32;
        } BIT;
    } DMAC1_CRSA_12;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRDA:32;
        } BIT;
    } DMAC1_CRDA_12;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRTB:32;
        } BIT;
    } DMAC1_CRTB_12;
    union {
        unsigned long LONG;
        struct {
            unsigned long EN:1;
            unsigned long RQST:1;
            unsigned long TACT:1;
            unsigned long SUS:1;
            unsigned long ER:1;
            unsigned long END:1;
            unsigned long :1;
            unsigned long SR:1;
            unsigned long DL:1;
            unsigned long DW:1;
            unsigned long DER:1;
            unsigned long MODE:1;
            unsigned long :4;
            unsigned long INTM:1;
            unsigned long DMARQM:1;
            unsigned long SWPRQ:1;
            unsigned long :5;
            unsigned long DNUM:8;
        } BIT;
    } DMAC1_CHSTAT_12;
    union {
        unsigned long LONG;
        struct {
            unsigned long SETEN:1;
            unsigned long CLREN:1;
            unsigned long :1;
            unsigned long SWRST:1;
            unsigned long CLRRQ:1;
            unsigned long CLREND:1;
            unsigned long :1;
            unsigned long CLRDE:1;
            unsigned long SETSUS:1;
            unsigned long CLRSUS:1;
            unsigned long :2;
            unsigned long SETREN:1;
            unsigned long :1;
            unsigned long SETSSWPRQ:1;
            unsigned long :1;
            unsigned long SETINTM:1;
            unsigned long CLRINTM:1;
            unsigned long SETDMARQM:1;
            unsigned long CLRDMARQM:1;
            unsigned long :12;
        } BIT;
    } DMAC1_CHCTRL_12;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEL:3;
            unsigned long REQD:1;
            unsigned long LOEN:1;
            unsigned long HIEN:1;
            unsigned long LVL:1;
            unsigned long :1;
            unsigned long AM:3;
            unsigned long DRRP:1;
            unsigned long SDS:4;
            unsigned long DDS:4;
            unsigned long SAD:1;
            unsigned long DAD:1;
            unsigned long TM:1;
            unsigned long WONLY:1;
            unsigned long DEM:1;
            unsigned long :1;
            unsigned long DIM:1;
            unsigned long SBE:1;
            unsigned long RSEL:1;
            unsigned long RSW:1;
            unsigned long REN:1;
            unsigned long DMS:1;
        } BIT;
    } DMAC1_CHCFG_12;
    union {
        unsigned long LONG;
        struct {
            unsigned long ITVL:16;
            unsigned long :16;
        } BIT;
    } DMAC1_CHITVL_12;
    char           wk23[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long NXLA:32;
        } BIT;
    } DMAC1_NXLA_12;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRLA:32;
        } BIT;
    } DMAC1_CRLA_12;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC1_N0SA_13_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC1_N0SA_13_W;
    } N0SA_13;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC1_N0DA_13;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC1_N0TB_13;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC1_N1SA_13_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC1_N1SA_13_W;
    } N1SA_13;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC1_N1DA_13;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC1_N1TB_13;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRSA:32;
        } BIT;
    } DMAC1_CRSA_13;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRDA:32;
        } BIT;
    } DMAC1_CRDA_13;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRTB:32;
        } BIT;
    } DMAC1_CRTB_13;
    union {
        unsigned long LONG;
        struct {
            unsigned long EN:1;
            unsigned long RQST:1;
            unsigned long TACT:1;
            unsigned long SUS:1;
            unsigned long ER:1;
            unsigned long END:1;
            unsigned long :1;
            unsigned long SR:1;
            unsigned long DL:1;
            unsigned long DW:1;
            unsigned long DER:1;
            unsigned long MODE:1;
            unsigned long :4;
            unsigned long INTM:1;
            unsigned long DMARQM:1;
            unsigned long SWPRQ:1;
            unsigned long :5;
            unsigned long DNUM:8;
        } BIT;
    } DMAC1_CHSTAT_13;
    union {
        unsigned long LONG;
        struct {
            unsigned long SETEN:1;
            unsigned long CLREN:1;
            unsigned long :1;
            unsigned long SWRST:1;
            unsigned long CLRRQ:1;
            unsigned long CLREND:1;
            unsigned long :1;
            unsigned long CLRDE:1;
            unsigned long SETSUS:1;
            unsigned long CLRSUS:1;
            unsigned long :2;
            unsigned long SETREN:1;
            unsigned long :1;
            unsigned long SETSSWPRQ:1;
            unsigned long :1;
            unsigned long SETINTM:1;
            unsigned long CLRINTM:1;
            unsigned long SETDMARQM:1;
            unsigned long CLRDMARQM:1;
            unsigned long :12;
        } BIT;
    } DMAC1_CHCTRL_13;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEL:3;
            unsigned long REQD:1;
            unsigned long LOEN:1;
            unsigned long HIEN:1;
            unsigned long LVL:1;
            unsigned long :1;
            unsigned long AM:3;
            unsigned long DRRP:1;
            unsigned long SDS:4;
            unsigned long DDS:4;
            unsigned long SAD:1;
            unsigned long DAD:1;
            unsigned long TM:1;
            unsigned long WONLY:1;
            unsigned long DEM:1;
            unsigned long :1;
            unsigned long DIM:1;
            unsigned long SBE:1;
            unsigned long RSEL:1;
            unsigned long RSW:1;
            unsigned long REN:1;
            unsigned long DMS:1;
        } BIT;
    } DMAC1_CHCFG_13;
    union {
        unsigned long LONG;
        struct {
            unsigned long ITVL:16;
            unsigned long :16;
        } BIT;
    } DMAC1_CHITVL_13;
    char           wk24[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long NXLA:32;
        } BIT;
    } DMAC1_NXLA_13;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRLA:32;
        } BIT;
    } DMAC1_CRLA_13;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC1_N0SA_14_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC1_N0SA_14_W;
    } N0SA_14;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC1_N0DA_14;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC1_N0TB_14;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC1_N1SA_14_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC1_N1SA_14_W;
    } N1SA_14;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC1_N1DA_14;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC1_N1TB_14;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRSA:32;
        } BIT;
    } DMAC1_CRSA_14;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRDA:32;
        } BIT;
    } DMAC1_CRDA_14;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRTB:32;
        } BIT;
    } DMAC1_CRTB_14;
    union {
        unsigned long LONG;
        struct {
            unsigned long EN:1;
            unsigned long RQST:1;
            unsigned long TACT:1;
            unsigned long SUS:1;
            unsigned long ER:1;
            unsigned long END:1;
            unsigned long :1;
            unsigned long SR:1;
            unsigned long DL:1;
            unsigned long DW:1;
            unsigned long DER:1;
            unsigned long MODE:1;
            unsigned long :4;
            unsigned long INTM:1;
            unsigned long DMARQM:1;
            unsigned long SWPRQ:1;
            unsigned long :5;
            unsigned long DNUM:8;
        } BIT;
    } DMAC1_CHSTAT_14;
    union {
        unsigned long LONG;
        struct {
            unsigned long SETEN:1;
            unsigned long CLREN:1;
            unsigned long :1;
            unsigned long SWRST:1;
            unsigned long CLRRQ:1;
            unsigned long CLREND:1;
            unsigned long :1;
            unsigned long CLRDE:1;
            unsigned long SETSUS:1;
            unsigned long CLRSUS:1;
            unsigned long :2;
            unsigned long SETREN:1;
            unsigned long :1;
            unsigned long SETSSWPRQ:1;
            unsigned long :1;
            unsigned long SETINTM:1;
            unsigned long CLRINTM:1;
            unsigned long SETDMARQM:1;
            unsigned long CLRDMARQM:1;
            unsigned long :12;
        } BIT;
    } DMAC1_CHCTRL_14;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEL:3;
            unsigned long REQD:1;
            unsigned long LOEN:1;
            unsigned long HIEN:1;
            unsigned long LVL:1;
            unsigned long :1;
            unsigned long AM:3;
            unsigned long DRRP:1;
            unsigned long SDS:4;
            unsigned long DDS:4;
            unsigned long SAD:1;
            unsigned long DAD:1;
            unsigned long TM:1;
            unsigned long WONLY:1;
            unsigned long DEM:1;
            unsigned long :1;
            unsigned long DIM:1;
            unsigned long SBE:1;
            unsigned long RSEL:1;
            unsigned long RSW:1;
            unsigned long REN:1;
            unsigned long DMS:1;
        } BIT;
    } DMAC1_CHCFG_14;
    union {
        unsigned long LONG;
        struct {
            unsigned long ITVL:16;
            unsigned long :16;
        } BIT;
    } DMAC1_CHITVL_14;
    char           wk25[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long NXLA:32;
        } BIT;
    } DMAC1_NXLA_14;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRLA:32;
        } BIT;
    } DMAC1_CRLA_14;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC1_N0SA_15_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC1_N0SA_15_W;
    } N0SA_15;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC1_N0DA_15;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC1_N0TB_15;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SA:32;
            } BIT;
        } DMAC1_N1SA_15_N;
        union {
            unsigned long LONG;
            struct {
                unsigned long WD:32;
            } BIT;
        } DMAC1_N1SA_15_W;
    } N1SA_15;
    union {
        unsigned long LONG;
        struct {
            unsigned long DA:32;
        } BIT;
    } DMAC1_N1DA_15;
    union {
        unsigned long LONG;
        struct {
            unsigned long TB:32;
        } BIT;
    } DMAC1_N1TB_15;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRSA:32;
        } BIT;
    } DMAC1_CRSA_15;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRDA:32;
        } BIT;
    } DMAC1_CRDA_15;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRTB:32;
        } BIT;
    } DMAC1_CRTB_15;
    union {
        unsigned long LONG;
        struct {
            unsigned long EN:1;
            unsigned long RQST:1;
            unsigned long TACT:1;
            unsigned long SUS:1;
            unsigned long ER:1;
            unsigned long END:1;
            unsigned long :1;
            unsigned long SR:1;
            unsigned long DL:1;
            unsigned long DW:1;
            unsigned long DER:1;
            unsigned long MODE:1;
            unsigned long :4;
            unsigned long INTM:1;
            unsigned long DMARQM:1;
            unsigned long SWPRQ:1;
            unsigned long :5;
            unsigned long DNUM:8;
        } BIT;
    } DMAC1_CHSTAT_15;
    union {
        unsigned long LONG;
        struct {
            unsigned long SETEN:1;
            unsigned long CLREN:1;
            unsigned long :1;
            unsigned long SWRST:1;
            unsigned long CLRRQ:1;
            unsigned long CLREND:1;
            unsigned long :1;
            unsigned long CLRDE:1;
            unsigned long SETSUS:1;
            unsigned long CLRSUS:1;
            unsigned long :2;
            unsigned long SETREN:1;
            unsigned long :1;
            unsigned long SETSSWPRQ:1;
            unsigned long :1;
            unsigned long SETINTM:1;
            unsigned long CLRINTM:1;
            unsigned long SETDMARQM:1;
            unsigned long CLRDMARQM:1;
            unsigned long :12;
        } BIT;
    } DMAC1_CHCTRL_15;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEL:3;
            unsigned long REQD:1;
            unsigned long LOEN:1;
            unsigned long HIEN:1;
            unsigned long LVL:1;
            unsigned long :1;
            unsigned long AM:3;
            unsigned long DRRP:1;
            unsigned long SDS:4;
            unsigned long DDS:4;
            unsigned long SAD:1;
            unsigned long DAD:1;
            unsigned long TM:1;
            unsigned long WONLY:1;
            unsigned long DEM:1;
            unsigned long :1;
            unsigned long DIM:1;
            unsigned long SBE:1;
            unsigned long RSEL:1;
            unsigned long RSW:1;
            unsigned long REN:1;
            unsigned long DMS:1;
        } BIT;
    } DMAC1_CHCFG_15;
    union {
        unsigned long LONG;
        struct {
            unsigned long ITVL:16;
            unsigned long :16;
        } BIT;
    } DMAC1_CHITVL_15;
    char           wk26[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long NXLA:32;
        } BIT;
    } DMAC1_NXLA_15;
    union {
        unsigned long LONG;
        struct {
            unsigned long CRLA:32;
        } BIT;
    } DMAC1_CRLA_15;
    union {
        unsigned long LONG;
        struct {
            unsigned long SCNT:32;
        } BIT;
    } DMAC1_SCNT_8;
    union {
        unsigned long LONG;
        struct {
            unsigned long SSKP:32;
        } BIT;
    } DMAC1_SSKP_8;
    union {
        unsigned long LONG;
        struct {
            unsigned long DCNT:32;
        } BIT;
    } DMAC1_DCNT_8;
    union {
        unsigned long LONG;
        struct {
            unsigned long DSKP:32;
        } BIT;
    } DMAC1_DSKP_8;
    char           wk27[16];
    union {
        unsigned long LONG;
        struct {
            unsigned long SCNT:32;
        } BIT;
    } DMAC1_SCNT_9;
    union {
        unsigned long LONG;
        struct {
            unsigned long SSKP:32;
        } BIT;
    } DMAC1_SSKP_9;
    union {
        unsigned long LONG;
        struct {
            unsigned long DCNT:32;
        } BIT;
    } DMAC1_DCNT_9;
    union {
        unsigned long LONG;
        struct {
            unsigned long DSKP:32;
        } BIT;
    } DMAC1_DSKP_9;
    char           wk28[16];
    union {
        unsigned long LONG;
        struct {
            unsigned long SCNT:32;
        } BIT;
    } DMAC1_SCNT_10;
    union {
        unsigned long LONG;
        struct {
            unsigned long SSKP:32;
        } BIT;
    } DMAC1_SSKP_10;
    union {
        unsigned long LONG;
        struct {
            unsigned long DCNT:32;
        } BIT;
    } DMAC1_DCNT_10;
    union {
        unsigned long LONG;
        struct {
            unsigned long DSKP:32;
        } BIT;
    } DMAC1_DSKP_10;
    char           wk29[16];
    union {
        unsigned long LONG;
        struct {
            unsigned long SCNT:32;
        } BIT;
    } DMAC1_SCNT_11;
    union {
        unsigned long LONG;
        struct {
            unsigned long SSKP:32;
        } BIT;
    } DMAC1_SSKP_11;
    union {
        unsigned long LONG;
        struct {
            unsigned long DCNT:32;
        } BIT;
    } DMAC1_DCNT_11;
    union {
        unsigned long LONG;
        struct {
            unsigned long DSKP:32;
        } BIT;
    } DMAC1_DSKP_11;
    char           wk30[16];
    union {
        unsigned long LONG;
        struct {
            unsigned long SCNT:32;
        } BIT;
    } DMAC1_SCNT_12;
    union {
        unsigned long LONG;
        struct {
            unsigned long SSKP:32;
        } BIT;
    } DMAC1_SSKP_12;
    union {
        unsigned long LONG;
        struct {
            unsigned long DCNT:32;
        } BIT;
    } DMAC1_DCNT_12;
    union {
        unsigned long LONG;
        struct {
            unsigned long DSKP:32;
        } BIT;
    } DMAC1_DSKP_12;
    char           wk31[16];
    union {
        unsigned long LONG;
        struct {
            unsigned long SCNT:32;
        } BIT;
    } DMAC1_SCNT_13;
    union {
        unsigned long LONG;
        struct {
            unsigned long SSKP:32;
        } BIT;
    } DMAC1_SSKP_13;
    union {
        unsigned long LONG;
        struct {
            unsigned long DCNT:32;
        } BIT;
    } DMAC1_DCNT_13;
    union {
        unsigned long LONG;
        struct {
            unsigned long DSKP:32;
        } BIT;
    } DMAC1_DSKP_13;
    char           wk32[16];
    union {
        unsigned long LONG;
        struct {
            unsigned long SCNT:32;
        } BIT;
    } DMAC1_SCNT_14;
    union {
        unsigned long LONG;
        struct {
            unsigned long SSKP:32;
        } BIT;
    } DMAC1_SSKP_14;
    union {
        unsigned long LONG;
        struct {
            unsigned long DCNT:32;
        } BIT;
    } DMAC1_DCNT_14;
    union {
        unsigned long LONG;
        struct {
            unsigned long DSKP:32;
        } BIT;
    } DMAC1_DSKP_14;
    char           wk33[16];
    union {
        unsigned long LONG;
        struct {
            unsigned long SCNT:32;
        } BIT;
    } DMAC1_SCNT_15;
    union {
        unsigned long LONG;
        struct {
            unsigned long SSKP:32;
        } BIT;
    } DMAC1_SSKP_15;
    union {
        unsigned long LONG;
        struct {
            unsigned long DCNT:32;
        } BIT;
    } DMAC1_DCNT_15;
    union {
        unsigned long LONG;
        struct {
            unsigned long DSKP:32;
        } BIT;
    } DMAC1_DSKP_15;
    char           wk34[16];
    union {
        unsigned long LONG;
        struct {
            unsigned long PR:1;
            unsigned long :31;
        } BIT;
    } DMAC1_DCTRL_B;
    union {
        unsigned long LONG;
        struct {
            unsigned long :8;
            unsigned long DITVL:8;
            unsigned long :16;
        } BIT;
    } DMAC1_DSCITVL_B;
    char           wk35[8];
    union {
        unsigned long LONG;
        struct {
            unsigned long EN08:1;
            unsigned long EN19:1;
            unsigned long EN210:1;
            unsigned long EN311:1;
            unsigned long EN412:1;
            unsigned long EN513:1;
            unsigned long EN614:1;
            unsigned long EN715:1;
            unsigned long :24;
        } BIT;
    } DMAC1_DST_EN_B;
    union {
        unsigned long LONG;
        struct {
            unsigned long ER08:1;
            unsigned long ER19:1;
            unsigned long ER210:1;
            unsigned long ER311:1;
            unsigned long ER412:1;
            unsigned long ER513:1;
            unsigned long ER614:1;
            unsigned long ER715:1;
            unsigned long :24;
        } BIT;
    } DMAC1_DST_ER_B;
    union {
        unsigned long LONG;
        struct {
            unsigned long END08:1;
            unsigned long END19:1;
            unsigned long END210:1;
            unsigned long END311:1;
            unsigned long END412:1;
            unsigned long END513:1;
            unsigned long END614:1;
            unsigned long END715:1;
            unsigned long :24;
        } BIT;
    } DMAC1_DST_END_B;
    char           wk36[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long SUS08:1;
            unsigned long SUS19:1;
            unsigned long SUS210:1;
            unsigned long SUS311:1;
            unsigned long SUS412:1;
            unsigned long SUS513:1;
            unsigned long SUS614:1;
            unsigned long SUS715:1;
            unsigned long :24;
        } BIT;
    } DMAC1_DST_SUS_B;
    char           wk37[198940];
    union {
        unsigned long LONG;
        struct {
            unsigned long IFC1:8;
            unsigned long :24;
        } BIT;
    } DMA1SEL0;
    union {
        unsigned long LONG;
        struct {
            unsigned long IFC1:8;
            unsigned long :24;
        } BIT;
    } DMA1SEL1;
    union {
        unsigned long LONG;
        struct {
            unsigned long IFC1:8;
            unsigned long :24;
        } BIT;
    } DMA1SEL2;
    union {
        unsigned long LONG;
        struct {
            unsigned long IFC1:8;
            unsigned long :24;
        } BIT;
    } DMA1SEL3;
    union {
        unsigned long LONG;
        struct {
            unsigned long IFC1:8;
            unsigned long :24;
        } BIT;
    } DMA1SEL4;
    union {
        unsigned long LONG;
        struct {
            unsigned long IFC1:8;
            unsigned long :24;
        } BIT;
    } DMA1SEL5;
    union {
        unsigned long LONG;
        struct {
            unsigned long IFC1:8;
            unsigned long :24;
        } BIT;
    } DMA1SEL6;
    union {
        unsigned long LONG;
        struct {
            unsigned long IFC1:8;
            unsigned long :24;
        } BIT;
    } DMA1SEL7;
    union {
        unsigned long LONG;
        struct {
            unsigned long IFC1:8;
            unsigned long :24;
        } BIT;
    } DMA1SEL8;
    union {
        unsigned long LONG;
        struct {
            unsigned long IFC1:8;
            unsigned long :24;
        } BIT;
    } DMA1SEL9;
    union {
        unsigned long LONG;
        struct {
            unsigned long IFC1:8;
            unsigned long :24;
        } BIT;
    } DMA1SEL10;
    union {
        unsigned long LONG;
        struct {
            unsigned long IFC1:8;
            unsigned long :24;
        } BIT;
    } DMA1SEL11;
    union {
        unsigned long LONG;
        struct {
            unsigned long IFC1:8;
            unsigned long :24;
        } BIT;
    } DMA1SEL12;
    union {
        unsigned long LONG;
        struct {
            unsigned long IFC1:8;
            unsigned long :24;
        } BIT;
    } DMA1SEL13;
    union {
        unsigned long LONG;
        struct {
            unsigned long IFC1:8;
            unsigned long :24;
        } BIT;
    } DMA1SEL14;
    union {
        unsigned long LONG;
        struct {
            unsigned long IFC1:8;
            unsigned long :24;
        } BIT;
    } DMA1SEL15;
};

struct st_dmac {
    union {
        unsigned long LONG;
        struct {
            unsigned long :9;
            unsigned long DPRTY:2;
            unsigned long :13;
            unsigned long AL0:1;
            unsigned long AL1:1;
            unsigned long AL2:1;
            unsigned long :1;
            unsigned long TL0:1;
            unsigned long TL1:1;
            unsigned long TL2:1;
            unsigned long :1;
        } BIT;
    } CMNCR;
    char           wk0[598140];
    union {
        unsigned long LONG;
        struct {
            unsigned long DMREQ0:1;
            unsigned long DMREQ1:1;
            unsigned long :30;
        } BIT;
    } DMASTG;
};

struct st_doc {
    union {
        unsigned char BYTE;
        struct {
            unsigned char OMS:2;
            unsigned char DCSEL:1;
            unsigned char :1;
            unsigned char DOPCIE:1;
            unsigned char DOPCF:1;
            unsigned char DOPCFCL:1;
            unsigned char :1;
        } BIT;
    } DOCR;
    char           wk0[1];
    unsigned short DODIR;
    unsigned short DODSR;
};

struct st_dsmif {
    union {
        unsigned long LONG;
        struct {
            unsigned long ENABLE:1;
            unsigned long :7;
            unsigned long SINC1SEL:2;
            unsigned long :2;
            unsigned long WORD1GEN:3;
            unsigned long :1;
            unsigned long BITSHIFT1:4;
            unsigned long SINC2SEL:2;
            unsigned long :2;
            unsigned long WORD2GEN:3;
            unsigned long :1;
            unsigned long BITSHIFT2:4;
        } BIT;
    } UVWCTL;
    union {
        unsigned long LONG;
        struct {
            unsigned long ERUI:1;
            unsigned long ERVI:1;
            unsigned long ERWI:1;
            unsigned long :1;
            unsigned long ERUSC:1;
            unsigned long ERVSC:1;
            unsigned long ERWSC:1;
            unsigned long :1;
            unsigned long ERUVWIGND:1;
            unsigned long :23;
        } BIT;
    } UVWSTA;
    union {
        unsigned long LONG;
        struct {
            unsigned long CMPUVWIUNDER:16;
            unsigned long :16;
        } BIT;
    } UVWIUNCMP;
    union {
        unsigned long LONG;
        struct {
            unsigned long CMPUVWIOVER:16;
            unsigned long :16;
        } BIT;
    } UVWIOVCMP;
    union {
        unsigned long LONG;
        struct {
            unsigned long CMPUVWSCUNDER:13;
            unsigned long :19;
        } BIT;
    } UVWSCUNCMP;
    union {
        unsigned long LONG;
        struct {
            unsigned long CMPUVWSCOVER:13;
            unsigned long :19;
        } BIT;
    } UVWSCOVCMP;
    union {
        unsigned long LONG;
        struct {
            unsigned long CMPUVWIGNDUNDER:18;
            unsigned long :14;
        } BIT;
    } UVWIGUNCMP;
    union {
        unsigned long LONG;
        struct {
            unsigned long CMPUVWIGNDOVER:18;
            unsigned long :14;
        } BIT;
    } UVWIGOVCMP;
    union {
        unsigned long LONG;
        struct {
            unsigned long U1DATA:16;
            unsigned long :16;
        } BIT;
    } U1DATA;
    union {
        unsigned long LONG;
        struct {
            unsigned long U1CDATA:16;
            unsigned long :16;
        } BIT;
    } U1CDATA;
    union {
        unsigned long LONG;
        struct {
            unsigned long U1VDATA:16;
            unsigned long :16;
        } BIT;
    } U1VDATA;
    union {
        unsigned long LONG;
        struct {
            unsigned long U2DATA:16;
            unsigned long :16;
        } BIT;
    } U2DATA;
    union {
        unsigned long LONG;
        struct {
            unsigned long V1DATA:16;
            unsigned long :16;
        } BIT;
    } V1DATA;
    union {
        unsigned long LONG;
        struct {
            unsigned long V1CDATA:16;
            unsigned long :16;
        } BIT;
    } V1CDATA;
    union {
        unsigned long LONG;
        struct {
            unsigned long V1VDATA:16;
            unsigned long :16;
        } BIT;
    } V1VDATA;
    union {
        unsigned long LONG;
        struct {
            unsigned long V2DATA:16;
            unsigned long :16;
        } BIT;
    } V2DATA;
    union {
        unsigned long LONG;
        struct {
            unsigned long W1DATA:16;
            unsigned long :16;
        } BIT;
    } W1DATA;
    union {
        unsigned long LONG;
        struct {
            unsigned long W1CDATA:16;
            unsigned long :16;
        } BIT;
    } W1CDATA;
    union {
        unsigned long LONG;
        struct {
            unsigned long W1VDATA:16;
            unsigned long :16;
        } BIT;
    } W1VDATA;
    union {
        unsigned long LONG;
        struct {
            unsigned long W2DATA:16;
            unsigned long :16;
        } BIT;
    } W2DATA;
    char           wk0[48];
    union {
        unsigned long LONG;
        struct {
            unsigned long ENABLE:1;
            unsigned long :7;
            unsigned long SINC1SEL:2;
            unsigned long :2;
            unsigned long WORD1GEN:3;
            unsigned long :1;
            unsigned long BITSHIFT1:4;
            unsigned long SINC2SEL:2;
            unsigned long :2;
            unsigned long WORD2GEN:3;
            unsigned long :1;
            unsigned long BITSHIFT2:4;
        } BIT;
    } XYZCTL;
    union {
        unsigned long LONG;
        struct {
            unsigned long ERXI:1;
            unsigned long :3;
            unsigned long ERXSC:1;
            unsigned long :27;
        } BIT;
    } XYZSTA;
    union {
        unsigned long LONG;
        struct {
            unsigned long CMPXIUNDER:16;
            unsigned long :16;
        } BIT;
    } XYZIUNCMP;
    union {
        unsigned long LONG;
        struct {
            unsigned long CMPXIOVER:16;
            unsigned long :16;
        } BIT;
    } XYZIOVCMP;
    union {
        unsigned long LONG;
        struct {
            unsigned long CMPXSCUNDER:13;
            unsigned long :19;
        } BIT;
    } XYZSCUNCMP;
    union {
        unsigned long LONG;
        struct {
            unsigned long CMPXSCOVER:13;
            unsigned long :19;
        } BIT;
    } XYZSCOVCMP;
    char           wk1[8];
    union {
        unsigned long LONG;
        struct {
            unsigned long X1DATA:16;
            unsigned long :16;
        } BIT;
    } X1DATA;
    union {
        unsigned long LONG;
        struct {
            unsigned long X1CDATA:16;
            unsigned long :16;
        } BIT;
    } X1CDATA;
    union {
        unsigned long LONG;
        struct {
            unsigned long X1VDATA:16;
            unsigned long :16;
        } BIT;
    } X1VDATA;
    char           wk2[16];
    union {
        unsigned long LONG;
        struct {
            unsigned long X2DATA:16;
            unsigned long :16;
        } BIT;
    } X2DATA;
};

struct st_ecatc {
    union {
        unsigned long LONG;
        struct {
            unsigned long OADD0:1;
            unsigned long OADD1:1;
            unsigned long OADD2:1;
            unsigned long OADD3:1;
            unsigned long OADD4:1;
            unsigned long :27;
        } BIT;
    } CATOFFADD;
    union {
        unsigned long LONG;
        struct {
            unsigned long I2CSIZE:1;
            unsigned long :31;
        } BIT;
    } CATEMMD;
    char           wk0[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long TXSFT00:1;
            unsigned long TXSFT01:1;
            unsigned long TXSFT10:1;
            unsigned long TXSFT11:1;
            unsigned long :28;
        } BIT;
    } CATTXCSFT;
    char           wk1[69360];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TYPE:8;
        } BIT;
    } TYPE;
    union {
        unsigned char BYTE;
        struct {
            unsigned char REV:8;
        } BIT;
    } REVISION;
    union {
        unsigned short WORD;
        struct {
            unsigned short BUILD:16;
        } BIT;
    } BUILD;
    union {
        unsigned char BYTE;
        struct {
            unsigned char NUMFMMU:8;
        } BIT;
    } FMMU_NUM;
    union {
        unsigned char BYTE;
        struct {
            unsigned char NUMSYNC:8;
        } BIT;
    } SYNC_MANAGER;
    union {
        unsigned char BYTE;
        struct {
            unsigned char RAMSIZE:8;
        } BIT;
    } RAM_SIZE;
    union {
        unsigned char BYTE;
        struct {
            unsigned char P0:2;
            unsigned char P1:2;
            unsigned char P2:2;
            unsigned char P3:2;
        } BIT;
    } PORT_DESC;
    union {
        unsigned short WORD;
        struct {
            unsigned short FMMU:1;
            unsigned short :1;
            unsigned short DC:1;
            unsigned short DCWID:1;
            unsigned short :2;
            unsigned short LINKDECMII:1;
            unsigned short FCS:1;
            unsigned short DCSYNC:1;
            unsigned short LRW:1;
            unsigned short RWSUPP:1;
            unsigned short FSCONFIG:1;
            unsigned short :4;
        } BIT;
    } FEATURE;
    char           wk2[6];
    union {
        unsigned short WORD;
        struct {
            unsigned short NODADDR:16;
        } BIT;
    } STATION_ADR;
    union {
        unsigned short WORD;
        struct {
            unsigned short NODALIADDR:16;
        } BIT;
    } STATION_ALIAS;
    char           wk3[12];
    union {
        unsigned char BYTE;
        struct {
            unsigned char ENABLE:1;
            unsigned char :7;
        } BIT;
    } WR_REG_ENABLE;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PROTECT:1;
            unsigned char :7;
        } BIT;
    } WR_REG_PROTECT;
    char           wk4[14];
    union {
        unsigned char BYTE;
        struct {
            unsigned char ENABLE:1;
            unsigned char :7;
        } BIT;
    } ESC_WR_ENABLE;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PROTECT:1;
            unsigned char :7;
        } BIT;
    } ESC_WR_PROTECT;
    char           wk5[14];
    union {
        union {
            unsigned char BYTE;
        } ESC_RESET_ECAT_W;
        union {
            unsigned char BYTE;
        } ESC_RESET_ECAT_R;
    } RESET_ECAT;
    union {
        union {
            unsigned char BYTE;
        } ESC_RESET_PDI_W;
        union {
            unsigned char BYTE;
        } ESC_RESET_PDI_R;
    } RESET_PDI;
    char           wk6[190];
    union {
        unsigned long LONG;
        struct {
            unsigned long FWDRULE:1;
            unsigned long TEMPUSE:1;
            unsigned long :6;
            unsigned long LP0:2;
            unsigned long LP1:2;
            unsigned long LP2:2;
            unsigned long LP3:2;
            unsigned long RXFIFO:3;
            unsigned long :5;
            unsigned long STAALIAS:1;
            unsigned long :7;
        } BIT;
    } ESC_DL_CONTROL;
    char           wk7[4];
    union {
        unsigned short WORD;
        struct {
            unsigned short RWOFFSET:16;
        } BIT;
    } PHYSICAL_RW_OFFSET;
    char           wk8[6];
    union {
        unsigned short WORD;
        struct {
            unsigned short PDIOPE:1;
            unsigned short PDIWDST:1;
            unsigned short ENHLINKD:1;
            unsigned short :1;
            unsigned short PHYP0:1;
            unsigned short PHYP1:1;
            unsigned short PHYP2:1;
            unsigned short PHYP3:1;
            unsigned short LP0:1;
            unsigned short COMP0:1;
            unsigned short LP1:1;
            unsigned short COMP1:1;
            unsigned short LP2:1;
            unsigned short COMP2:1;
            unsigned short LP3:1;
            unsigned short COMP3:1;
        } BIT;
    } ESC_DL_STATUS;
    char           wk9[14];
    union {
        unsigned short WORD;
        struct {
            unsigned short INISTATE:4;
            unsigned short ERRINDACK:1;
            unsigned short :11;
        } BIT;
    } AL_CONTROL;
    char           wk10[14];
    union {
        unsigned short WORD;
        struct {
            unsigned short ACTSTATE:4;
            unsigned short ERR:1;
            unsigned short :11;
        } BIT;
    } AL_STATUS;
    char           wk11[2];
    union {
        unsigned short WORD;
        struct {
            unsigned short STATUSCODE:16;
        } BIT;
    } AL_STATUS_CODE;
    char           wk12[2];
    union {
        unsigned char BYTE;
        struct {
            unsigned char LEDCODE:4;
            unsigned char OVERRIDEEN:1;
            unsigned char :3;
        } BIT;
    } RUN_LED_OVERRIDE;
    union {
        unsigned char BYTE;
        struct {
            unsigned char LEDCODE:4;
            unsigned char OVERRIDEEN:1;
            unsigned char :3;
        } BIT;
    } ERR_LED_OVERRIDE;
    char           wk13[6];
    union {
        unsigned char BYTE;
        struct {
            unsigned char PDI:8;
        } BIT;
    } PDI_CONTROL;
    union {
        unsigned char BYTE;
        struct {
            unsigned char DEVEMU:1;
            unsigned char ENLALLP:1;
            unsigned char DCSYNC:1;
            unsigned char DCLATCH:1;
            unsigned char ENLP0:1;
            unsigned char ENLP1:1;
            unsigned char ENLP2:1;
            unsigned char ENLP3:1;
        } BIT;
    } ESC_CONFIG;
    char           wk14[14];
    union {
        unsigned char BYTE;
        struct {
            unsigned char ONCHIPBUSCLK:5;
            unsigned char ONCHIPBUS:3;
        } BIT;
    } PDI_CONFIG;
    union {
        unsigned char BYTE;
        struct {
            unsigned char SYNC0OUT:2;
            unsigned char SYNCLAT0:1;
            unsigned char SYNC0MAP:1;
            unsigned char :1;
            unsigned char SYNC1OUT:1;
            unsigned char SYNCLAT1:1;
            unsigned char SYNC1MAP:1;
        } BIT;
    } SYNC_LATCH_CONFIG;
    union {
        unsigned short WORD;
        struct {
            unsigned short DATABUSWID:1;
            unsigned short :15;
        } BIT;
    } EXT_PDI_CONFIG;
    char           wk15[172];
    union {
        unsigned short WORD;
        struct {
            unsigned short ECATEVMASK:16;
        } BIT;
    } ECAT_EVENT_MASK;
    char           wk16[2];
    union {
        unsigned long LONG;
        struct {
            unsigned long ALEVMASK:32;
        } BIT;
    } AL_EVENT_MASK;
    char           wk17[8];
    union {
        unsigned short WORD;
        struct {
            unsigned short DCLATCH:1;
            unsigned short :1;
            unsigned short DLSTA:1;
            unsigned short ALSTA:1;
            unsigned short SMSTA0:1;
            unsigned short SMSTA1:1;
            unsigned short SMSTA2:1;
            unsigned short SMSTA3:1;
            unsigned short SMSTA4:1;
            unsigned short SMSTA5:1;
            unsigned short SMSTA6:1;
            unsigned short SMSTA7:1;
            unsigned short :4;
        } BIT;
    } ECAT_EVENT_REQ;
    char           wk18[14];
    union {
        unsigned long LONG;
        struct {
            unsigned long ALCTRL:1;
            unsigned long DCLATCH:1;
            unsigned long DCSYNC0STA:1;
            unsigned long DCSYNC1STA:1;
            unsigned long SYNCACT:1;
            unsigned long :1;
            unsigned long WDPD:1;
            unsigned long :1;
            unsigned long SMINT0:1;
            unsigned long SMINT1:1;
            unsigned long SMINT2:1;
            unsigned long SMINT3:1;
            unsigned long SMINT4:1;
            unsigned long SMINT5:1;
            unsigned long SMINT6:1;
            unsigned long SMINT7:1;
            unsigned long :16;
        } BIT;
    } AL_EVENT_REQ;
    char           wk19[220];
    union {
        unsigned short WORD;
        struct {
            unsigned short RXERRCNT:16;
        } BIT;
    } RX_ERR_COUNT0;
    union {
        unsigned short WORD;
        struct {
            unsigned short RXERRCNT:16;
        } BIT;
    } RX_ERR_COUNT1;
    char           wk20[4];
    union {
        unsigned char BYTE;
        struct {
            unsigned char FWDERRCNT:8;
        } BIT;
    } FWD_RX_ERR_COUNT0;
    union {
        unsigned char BYTE;
        struct {
            unsigned char FWDERRCNT:8;
        } BIT;
    } FWD_RX_ERR_COUNT1;
    char           wk21[2];
    union {
        unsigned char BYTE;
        struct {
            unsigned char EPUERRCNT:8;
        } BIT;
    } ECAT_PROC_ERR_COUNT;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PDIERRCNT:8;
        } BIT;
    } PDI_ERR_COUNT;
    char           wk22[2];
    union {
        unsigned char BYTE;
        struct {
            unsigned char LOSTLINKCNT:8;
        } BIT;
    } LOST_LINK_COUNT0;
    union {
        unsigned char BYTE;
        struct {
            unsigned char LOSTLINKCNT:8;
        } BIT;
    } LOST_LINK_COUNT1;
    char           wk23[238];
    union {
        unsigned short WORD;
        struct {
            unsigned short WDDIV:16;
        } BIT;
    } WD_DIVIDE;
    char           wk24[14];
    union {
        unsigned short WORD;
        struct {
            unsigned short WDTIMPDI:16;
        } BIT;
    } WDT_PDI;
    char           wk25[14];
    union {
        unsigned short WORD;
        struct {
            unsigned short WDTIMPD:16;
        } BIT;
    } WDT_DATA;
    char           wk26[30];
    union {
        unsigned short WORD;
        struct {
            unsigned short WDSTAPD:1;
            unsigned short :15;
        } BIT;
    } WDS_DATA;
    union {
        unsigned char BYTE;
        struct {
            unsigned char WDCNTPD:8;
        } BIT;
    } WDC_DATA;
    union {
        unsigned char BYTE;
        struct {
            unsigned char WDCNTPDI:8;
        } BIT;
    } WDC_PDI;
    char           wk27[188];
    union {
        unsigned char BYTE;
        struct {
            unsigned char CTRLPDI:1;
            unsigned char FORCEECAT:1;
            unsigned char :6;
        } BIT;
    } EEP_CONF;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PDIACCESS:1;
            unsigned char :7;
        } BIT;
    } EEP_STATE;
    union {
        unsigned short WORD;
        struct {
            unsigned short ECATWREN:1;
            unsigned short :5;
            unsigned short READBYTE:1;
            unsigned short PROMSIZE:1;
            unsigned short COMMAND:3;
            unsigned short CKSUMERR:1;
            unsigned short LOADSTA:1;
            unsigned short ACKCMDERR:1;
            unsigned short WRENERR:1;
            unsigned short BUSY:1;
        } BIT;
    } EEP_CONT_STAT;
    union {
        unsigned long LONG;
        struct {
            unsigned long ADDRESS:32;
        } BIT;
    } EEP_ADR;
    union {
        unsigned long LONG;
        struct {
            unsigned long LODATA:16;
            unsigned long HIDATA:16;
        } BIT;
    } EEP_DATA;
    char           wk28[4];
    union {
        unsigned short WORD;
        struct {
            unsigned short WREN:1;
            unsigned short PDICTRL:1;
            unsigned short MILINK:1;
            unsigned short PHYOFFSET:5;
            unsigned short COMMAND:2;
            unsigned short :3;
            unsigned short READERR:1;
            unsigned short CMDERR:1;
            unsigned short BUSY:1;
        } BIT;
    } MII_CONT_STAT;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PHYADDR:5;
            unsigned char :3;
        } BIT;
    } PHY_ADR;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PHYREGADDR:5;
            unsigned char :3;
        } BIT;
    } PHY_REG_ADR;
    union {
        unsigned short WORD;
        struct {
            unsigned short PHYREGDATA:16;
        } BIT;
    } PHY_DATA;
    union {
        unsigned char BYTE;
        struct {
            unsigned char ACSMII:1;
            unsigned char :7;
        } BIT;
    } MII_ECAT_ACS_STAT;
    union {
        unsigned char BYTE;
        struct {
            unsigned char ACSMII:1;
            unsigned char FORPDI:1;
            unsigned char :6;
        } BIT;
    } MII_PDI_ACS_STAT;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PHYLINKSTA:1;
            unsigned char LINKSTA:1;
            unsigned char LINKSTAERR:1;
            unsigned char READERR:1;
            unsigned char LINKPARTERR:1;
            unsigned char PHYCONFIG:1;
            unsigned char :2;
        } BIT;
    } PHY_STATUS0;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PHYLINKSTA:1;
            unsigned char LINKSTA:1;
            unsigned char LINKSTAERR:1;
            unsigned char READERR:1;
            unsigned char LINKPARTERR:1;
            unsigned char PHYCONFIG:1;
            unsigned char :2;
        } BIT;
    } PHY_STATUS1;
    char           wk29[230];
    struct {
        union {
            unsigned long LONG;
            struct {
                unsigned long LSTAADR:32;
            } BIT;
        } L_START_ADR;
        union {
            unsigned short WORD;
            struct {
                unsigned short FMMULEN:16;
            } BIT;
        } LEN;
        union {
            unsigned char BYTE;
            struct {
                unsigned char LSTABIT:3;
                unsigned char :5;
            } BIT;
        } L_START_BIT;
        union {
            unsigned char BYTE;
            struct {
                unsigned char LSTABIT:3;
                unsigned char :5;
            } BIT;
        } L_STOP_BIT;
        union {
            unsigned short WORD;
            struct {
                unsigned short PHYSTAADR:16;
            } BIT;
        } P_START_ADR;
        union {
            unsigned char BYTE;
            struct {
                unsigned char PHYSTABIT:3;
                unsigned char :5;
            } BIT;
        } P_START_BIT;
        union {
            unsigned char BYTE;
            struct {
                unsigned char READ:1;
                unsigned char WRITE:1;
                unsigned char :6;
            } BIT;
        } TYPE;
        union {
            unsigned char BYTE;
            struct {
                unsigned char ACTIVATE:1;
                unsigned char :7;
            } BIT;
        } ACT;
        char           fmmu_wk[3];
    } FMMU[8];
    char           wk37[0x180];
    struct {
        union {
            unsigned short WORD;
            struct {
                unsigned short SMSTAADDR:16;
            } BIT;
        } P_START_ADR;
        union {
            unsigned short WORD;
            struct {
                unsigned short SMLEN:16;
            } BIT;
        } LEN;
        union {
            unsigned char BYTE;
            struct {
                unsigned char OPEMODE:2;
                unsigned char DIR:2;
                unsigned char IRQECAT:1;
                unsigned char IRQPDI:1;
                unsigned char WDTRGEN:1;
                unsigned char :1;
            } BIT;
        } CONTROL;
        union {
            unsigned char BYTE;
            struct {
                unsigned char INTWR:1;
                unsigned char INTRD:1;
                unsigned char :1;
                unsigned char MAILBOX:1;
                unsigned char BUFFERED:2;
                unsigned char RDBUF:1;
                unsigned char WRBUF:1;
            } BIT;
        } STATUS;
        union {
            unsigned char BYTE;
            struct {
                unsigned char SMEN:1;
                unsigned char REPEATREQ:1;
                unsigned char :4;
                unsigned char LATCHECAT:1;
                unsigned char LATCHPDI:1;
            } BIT;
        } ACT;
        union {
            unsigned char BYTE;
            struct {
                unsigned char DEACTIVE:1;
                unsigned char REPEATACK:1;
                unsigned char :6;
            } BIT;
        } PDI_CONT;
    } SM[8];
    char           wk38[192];
    union {
        unsigned long LONG;
        struct {
            unsigned long RCVTIME0:32;
        } BIT;
    } DC_RCV_TIME_PORT0;
    union {
        unsigned long LONG;
        struct {
            unsigned long RCVTIME1:32;
        } BIT;
    } DC_RCV_TIME_PORT1;
    char           wk39[8];
    union {
        unsigned long long LONGLONG;
    } DC_SYS_TIME;
    union {
        unsigned long long LONGLONG;
    } DC_RCV_TIME_UNIT;
    union {
        unsigned long long LONGLONG;
    } DC_SYS_TIME_OFFSET;
    union {
        unsigned long LONG;
        struct {
            unsigned long SYSTIMDLY:32;
        } BIT;
    } DC_SYS_TIME_DELAY;
    union {
        unsigned long LONG;
        struct {
            unsigned long LOCALCOPY:1;
            unsigned long DIFF:31;
        } BIT;
    } DC_SYS_TIME_DIFF;
    union {
        unsigned short WORD;
        struct {
            unsigned short :1;
            unsigned short SPDCNTSTRT:15;
        } BIT;
    } DC_SPEED_COUNT_START;
    union {
        unsigned short WORD;
        struct {
            unsigned short SPDCNTDIFF:16;
        } BIT;
    } DC_SPEED_COUNT_DIFF;
    union {
        unsigned char BYTE;
        struct {
            unsigned char :4;
            unsigned char SYSTIMDEP:4;
        } BIT;
    } DC_SYS_TIME_DIFF_FIL_DEPTH;
    union {
        unsigned char BYTE;
        struct {
            unsigned char :4;
            unsigned char CLKPERDEP:4;
        } BIT;
    } DC_SPEED_COUNT_FIL_DEPTH;
    char           wk40[74];
    union {
        unsigned char BYTE;
        struct {
            unsigned char :2;
            unsigned char LATCH1:1;
            unsigned char LATCH0:1;
            unsigned char :3;
            unsigned char SYNCOUT:1;
        } BIT;
    } DC_CYC_CONT;
    union {
        unsigned char BYTE;
        struct {
            unsigned char DBGPULSE:1;
            unsigned char NEARFUTURE:1;
            unsigned char STARTTIME:1;
            unsigned char EXTSTARTTIME:1;
            unsigned char AUTOACT:1;
            unsigned char SYNC1:1;
            unsigned char SYNC0:1;
            unsigned char SYNCACT:1;
        } BIT;
    } DC_ACT;
    union {
        unsigned short WORD;
        struct {
            unsigned short PULSELEN:16;
        } BIT;
    } DC_PULSE_LEN;
    union {
        unsigned char BYTE;
        struct {
            unsigned char :5;
            unsigned char STARTTIME:1;
            unsigned char SYNC1ACT:1;
            unsigned char SYNC0ACT:1;
        } BIT;
    } DC_ACT_STAT;
    char           wk41[9];
    union {
        unsigned char BYTE;
        struct {
            unsigned char :7;
            unsigned char SYNC0STA:1;
        } BIT;
    } DC_SYNC0_STAT;
    union {
        unsigned char BYTE;
        struct {
            unsigned char :7;
            unsigned char SYNC1STA:1;
        } BIT;
    } DC_SYNC1_STAT;
    union {
        unsigned long long LONGLONG;
    } DC_CYC_START_TIME;
    union {
        unsigned long long LONGLONG;
    } DC_NEXT_SYNC1_PULSE;
    union {
        unsigned long LONG;
        struct {
            unsigned long SYNC0CYC:32;
        } BIT;
    } DC_SYNC0_CYC_TIME;
    union {
        unsigned long LONG;
        struct {
            unsigned long SYNC1CYC:32;
        } BIT;
    } DC_SYNC1_CYC_TIME;
    union {
        unsigned char BYTE;
        struct {
            unsigned char :6;
            unsigned char NEGEDGE:1;
            unsigned char POSEDGE:1;
        } BIT;
    } DC_LATCH0_CONT;
    union {
        unsigned char BYTE;
        struct {
            unsigned char :6;
            unsigned char NEGEDGE:1;
            unsigned char POSEDGE:1;
        } BIT;
    } DC_LATCH1_CONT;
    char           wk42[4];
    union {
        unsigned char BYTE;
        struct {
            unsigned char :5;
            unsigned char PINSTATE:1;
            unsigned char EVENTNEG:1;
            unsigned char EVENTPOS:1;
        } BIT;
    } DC_LATCH0_STAT;
    union {
        unsigned char BYTE;
        struct {
            unsigned char :5;
            unsigned char PINSTATE:1;
            unsigned char EVENTNEG:1;
            unsigned char EVENTPOS:1;
        } BIT;
    } DC_LATCH1_STAT;
    union {
        unsigned long long LONGLONG;
    } DC_LATCH0_TIME_POS;
    union {
        unsigned long long LONGLONG;
    } DC_LATCH0_TIME_NEG;
    union {
        unsigned long long LONGLONG;
    } DC_LATCH1_TIME_POS;
    union {
        unsigned long long LONGLONG;
    } DC_LATCH1_TIME_NEG;
    char           wk43[32];
    union {
        unsigned long LONG;
        struct {
            unsigned long ECATCHANGE:32;
        } BIT;
    } DC_ECAT_CNG_EV_TIME;
    char           wk44[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long PDISTART:32;
        } BIT;
    } DC_PDI_START_EV_TIME;
    union {
        unsigned long LONG;
        struct {
            unsigned long PDICHANGE:32;
        } BIT;
    } DC_PDI_CNG_EV_TIME;
    char           wk45[1024];
    union {
        unsigned long long LONGLONG;
    } PRODUCT_ID;
    union {
        unsigned long long LONGLONG;
    } VENDOR_ID;
};

struct st_eccram {
    union {
        unsigned long LONG;
    } RAMPCMD;
    char           wk0[252];
    union {
        unsigned long LONG;
        struct {
            unsigned long :31;
            unsigned long ECC_ENABLE:1;
        } BIT;
    } RAMEDC;
    union {
        unsigned long LONG;
        struct {
            unsigned long :16;
            unsigned long DBE_DIST15:1;
            unsigned long DBE_DIST14:1;
            unsigned long DBE_DIST13:1;
            unsigned long DBE_DIST12:1;
            unsigned long DBE_DIST11:1;
            unsigned long DBE_DIST10:1;
            unsigned long DBE_DIST9:1;
            unsigned long DBE_DIST8:1;
            unsigned long DBE_DIST7:1;
            unsigned long DBE_DIST6:1;
            unsigned long DBE_DIST5:1;
            unsigned long DBE_DIST4:1;
            unsigned long DBE_DIST3:1;
            unsigned long DBE_DIST2:1;
            unsigned long DBE_DIST1:1;
            unsigned long DBE_DIST0:1;
        } BIT;
    } RAMEEC;
    union {
        unsigned long LONG;
        struct {
            unsigned long :16;
            unsigned long DBE_RAM15:1;
            unsigned long DBE_RAM14:1;
            unsigned long DBE_RAM13:1;
            unsigned long DBE_RAM12:1;
            unsigned long DBE_RAM11:1;
            unsigned long DBE_RAM10:1;
            unsigned long DBE_RAM9:1;
            unsigned long DBE_RAM8:1;
            unsigned long DBE_RAM7:1;
            unsigned long DBE_RAM6:1;
            unsigned long DBE_RAM5:1;
            unsigned long DBE_RAM4:1;
            unsigned long DBE_RAM3:1;
            unsigned long DBE_RAM2:1;
            unsigned long DBE_RAM1:1;
            unsigned long DBE_RAM0:1;
        } BIT;
    } RAMDBEST;
    union {
        unsigned long LONG;
        struct {
            unsigned long :12;
            unsigned long BANK:2;
            unsigned long ADDRESS:16;
            unsigned long :1;
            unsigned long LOCK:1;
        } BIT;
    } RAMDBEAD;
    union {
        unsigned long LONG;
        struct {
            unsigned long :28;
            unsigned long ERRCOUNT:4;
        } BIT;
    } RAMDBECNT;
};

struct st_ecm {
    union {
        unsigned char BYTE;
    } ECMEPCFG;
    char           wk0[3];
    union {
        unsigned long LONG;
    } ECMMICFG0;
    union {
        unsigned long LONG;
    } ECMMICFG1;
    union {
        unsigned long LONG;
    } ECMMICFG2;
    union {
        unsigned long LONG;
    } ECMNMICFG0;
    union {
        unsigned long LONG;
    } ECMNMICFG1;
    union {
        unsigned long LONG;
    } ECMNMICFG2;
    union {
        unsigned long LONG;
    } ECMIRCFG0;
    union {
        unsigned long LONG;
    } ECMIRCFG1;
    union {
        unsigned long LONG;
    } ECMIRCFG2;
    union {
        unsigned long LONG;
    } ECMEMK0;
    union {
        unsigned long LONG;
    } ECMEMK1;
    union {
        unsigned long LONG;
    } ECMEMK2;
    union {
        unsigned long LONG;
    } ECMESSTC0;
    union {
        unsigned long LONG;
    } ECMESSTC1;
    union {
        unsigned long LONG;
    } ECMESSTC2;
    union {
        unsigned long LONG;
    } ECMPCMD1;
    union {
        unsigned char BYTE;
        struct {
            unsigned char ECMPRERR:1;
            unsigned char :7;
        } BIT;
    } ECMPS;
    char           wk1[3];
    union {
        unsigned long LONG;
    } ECMPE0;
    union {
        unsigned long LONG;
    } ECMPE1;
    union {
        unsigned long LONG;
    } ECMPE2;
    union {
        unsigned char BYTE;
    } ECMDTMCTL;
    char           wk2[3];
    union {
        unsigned short WORD;
        struct {
            unsigned short ECMTDMR:16;
        } BIT;
    } ECMDTMR;
    char           wk3[2];
    union {
        unsigned long LONG;
    } ECMDTMCMP;
    union {
        unsigned long LONG;
    } ECMDTMCFG0;
    union {
        unsigned long LONG;
    } ECMDTMCFG1;
    union {
        unsigned long LONG;
    } ECMDTMCFG2;
    union {
        unsigned long LONG;
    } ECMDTMCFG3;
    union {
        unsigned long LONG;
    } ECMDTMCFG4;
    union {
        unsigned long LONG;
    } ECMDTMCFG5;
    union {
        unsigned long LONG;
    } ECMEOCCFG;
};

struct st_ecmc {
    union {
        unsigned char BYTE;
    } ECMCESET;
    char           wk0[3];
    union {
        unsigned char BYTE;
    } ECMCECLR;
    char           wk1[3];
    union {
        unsigned long LONG;
        struct {
            unsigned long ECMCSSE000:1;
            unsigned long ECMCSSE001:1;
            unsigned long ECMCSSE002:1;
            unsigned long :1;
            unsigned long ECMCSSE004:1;
            unsigned long ECMCSSE005:1;
            unsigned long ECMCSSE006:1;
            unsigned long ECMCSSE007:1;
            unsigned long ECMCSSE008:1;
            unsigned long ECMCSSE009:1;
            unsigned long ECMCSSE010:1;
            unsigned long ECMCSSE011:1;
            unsigned long ECMCSSE012:1;
            unsigned long ECMCSSE013:1;
            unsigned long ECMCSSE014:1;
            unsigned long ECMCSSE015:1;
            unsigned long ECMCSSE016:1;
            unsigned long ECMCSSE017:1;
            unsigned long ECMCSSE018:1;
            unsigned long ECMCSSE019:1;
            unsigned long ECMCSSE020:1;
            unsigned long ECMCSSE021:1;
            unsigned long ECMCSSE022:1;
            unsigned long ECMCSSE023:1;
            unsigned long ECMCSSE024:1;
            unsigned long ECMCSSE025:1;
            unsigned long ECMCSSE026:1;
            unsigned long ECMCSSE027:1;
            unsigned long ECMCSSE028:1;
            unsigned long :1;
            unsigned long ECMCSSE030:1;
            unsigned long ECMCSSE031:1;
        } BIT;
    } ECMCESSTR0;
    union {
        unsigned long LONG;
        struct {
            unsigned long ECMCSSE100:1;
            unsigned long ECMCSSE101:1;
            unsigned long ECMCSSE202:1;
            unsigned long :1;
            unsigned long ECMCSSE104:1;
            unsigned long ECMCSSE105:1;
            unsigned long ECMCSSE106:1;
            unsigned long ECMCSSE107:1;
            unsigned long ECMCSSE108:1;
            unsigned long :23;
        } BIT;
    } ECMCESSTR1;
    union {
        unsigned long LONG;
        struct {
            unsigned long :28;
            unsigned long ECMCSSE228:1;
            unsigned long ECMCSSE229:1;
            unsigned long ECMCSSE230:1;
            unsigned long ECMCSSE231:1;
        } BIT;
    } ECMCESSTR2;
    union {
        unsigned long LONG;
        struct {
            unsigned long ECMC0REG:8;
            unsigned long :24;
        } BIT;
    } ECMCPCMD0;
};

struct st_ecmm {
    union {
        unsigned char BYTE;
    } ECMMESET;
    char           wk0[3];
    union {
        unsigned char BYTE;
    } ECMMECLR;
    char           wk1[3];
    union {
        unsigned long LONG;
        struct {
            unsigned long ECMMSSE000:1;
            unsigned long ECMMSSE001:1;
            unsigned long ECMMSSE002:1;
            unsigned long :1;
            unsigned long ECMMSSE004:1;
            unsigned long ECMMSSE005:1;
            unsigned long ECMMSSE006:1;
            unsigned long ECMMSSE007:1;
            unsigned long ECMMSSE008:1;
            unsigned long ECMMSSE009:1;
            unsigned long ECMMSSE010:1;
            unsigned long ECMMSSE011:1;
            unsigned long ECMMSSE012:1;
            unsigned long ECMMSSE013:1;
            unsigned long ECMMSSE014:1;
            unsigned long ECMMSSE015:1;
            unsigned long ECMMSSE016:1;
            unsigned long ECMMSSE017:1;
            unsigned long ECMMSSE018:1;
            unsigned long ECMMSSE019:1;
            unsigned long ECMMSSE020:1;
            unsigned long ECMMSSE021:1;
            unsigned long ECMMSSE022:1;
            unsigned long ECMMSSE023:1;
            unsigned long ECMMSSE024:1;
            unsigned long ECMMSSE025:1;
            unsigned long ECMMSSE026:1;
            unsigned long ECMMSSE027:1;
            unsigned long ECMMSSE028:1;
            unsigned long :1;
            unsigned long ECMMSSE030:1;
            unsigned long ECMMSSE031:1;
        } BIT;
    } ECMMESSTR0;
    union {
        unsigned long LONG;
        struct {
            unsigned long ECMMSSE100:1;
            unsigned long ECMMSSE101:1;
            unsigned long ECMMSSE102:1;
            unsigned long :1;
            unsigned long ECMMSSE104:1;
            unsigned long ECMMSSE105:1;
            unsigned long ECMMSSE106:1;
            unsigned long ECMMSSE107:1;
            unsigned long ECMMSSE108:1;
            unsigned long :23;
        } BIT;
    } ECMMESSTR1;
    union {
        unsigned long LONG;
        struct {
            unsigned long :28;
            unsigned long ECMMSSE228:1;
            unsigned long ECMMSSE229:1;
            unsigned long ECMMSSE230:1;
            unsigned long ECMMSSE231:1;
        } BIT;
    } ECMMESSTR2;
    union {
        unsigned long LONG;
        struct {
            unsigned long ECMM0REG:8;
            unsigned long :24;
        } BIT;
    } ECMMPCMD0;
};

struct st_elc {
    union {
        unsigned char BYTE;
        struct {
            unsigned char :7;
            unsigned char ELCON:1;
        } BIT;
    } ELCR;
    union {
        unsigned char BYTE;
        struct {
            unsigned char ELS:8;
        } BIT;
    } ELSR0;
    char           wk0[2];
    union {
        unsigned char BYTE;
        struct {
            unsigned char ELS:8;
        } BIT;
    } ELSR3;
    union {
        unsigned char BYTE;
        struct {
            unsigned char ELS:8;
        } BIT;
    } ELSR4;
    char           wk1[2];
    union {
        unsigned char BYTE;
        struct {
            unsigned char ELS:8;
        } BIT;
    } ELSR7;
    char           wk2[2];
    union {
        unsigned char BYTE;
        struct {
            unsigned char ELS:8;
        } BIT;
    } ELSR10;
    union {
        unsigned char BYTE;
        struct {
            unsigned char ELS:8;
        } BIT;
    } ELSR11;
    union {
        unsigned char BYTE;
        struct {
            unsigned char ELS:8;
        } BIT;
    } ELSR12;
    union {
        unsigned char BYTE;
        struct {
            unsigned char ELS:8;
        } BIT;
    } ELSR13;
    char           wk3[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char ELS:8;
        } BIT;
    } ELSR15;
    char           wk4[2];
    union {
        unsigned char BYTE;
        struct {
            unsigned char ELS:8;
        } BIT;
    } ELSR18;
    union {
        unsigned char BYTE;
        struct {
            unsigned char ELS:8;
        } BIT;
    } ELSR19;
    union {
        unsigned char BYTE;
        struct {
            unsigned char ELS:8;
        } BIT;
    } ELSR20;
    union {
        unsigned char BYTE;
        struct {
            unsigned char ELS:8;
        } BIT;
    } ELSR21;
    union {
        unsigned char BYTE;
        struct {
            unsigned char ELS:8;
        } BIT;
    } ELSR22;
    union {
        unsigned char BYTE;
        struct {
            unsigned char ELS:8;
        } BIT;
    } ELSR23;
    union {
        unsigned char BYTE;
        struct {
            unsigned char ELS:8;
        } BIT;
    } ELSR24;
    union {
        unsigned char BYTE;
        struct {
            unsigned char ELS:8;
        } BIT;
    } ELSR25;
    union {
        unsigned char BYTE;
        struct {
            unsigned char ELS:8;
        } BIT;
    } ELSR26;
    union {
        unsigned char BYTE;
        struct {
            unsigned char ELS:8;
        } BIT;
    } ELSR27;
    char           wk5[2];
    union {
        unsigned char BYTE;
        struct {
            unsigned char MTU0MD:2;
            unsigned char :4;
            unsigned char MTU3MD:2;
        } BIT;
    } ELOPA;
    union {
        unsigned char BYTE;
        struct {
            unsigned char MTU4MD:2;
            unsigned char :6;
        } BIT;
    } ELOPB;
    union {
        unsigned char BYTE;
        struct {
            unsigned char :2;
            unsigned char CMT1MD:2;
            unsigned char :4;
        } BIT;
    } ELOPC;
    union {
        unsigned char BYTE;
        struct {
            unsigned char DSU0MD:2;
            unsigned char DSU1MD:2;
            unsigned char DSX0MD:2;
            unsigned char DSX1MD:2;
        } BIT;
    } ELOPD;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PGRn0:1;
            unsigned char PGRn1:1;
            unsigned char PGRn2:1;
            unsigned char PGRn3:1;
            unsigned char PGRn4:1;
            unsigned char PGRn5:1;
            unsigned char PGRn6:1;
            unsigned char PGRn7:1;
        } BIT;
    } PGR1;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PGRn0:1;
            unsigned char PGRn1:1;
            unsigned char PGRn2:1;
            unsigned char PGRn3:1;
            unsigned char PGRn4:1;
            unsigned char PGRn5:1;
            unsigned char PGRn6:1;
            unsigned char PGRn7:1;
        } BIT;
    } PGR2;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PGCIn:2;
            unsigned char PGCOVEn:1;
            unsigned char :1;
            unsigned char PGCOn:3;
            unsigned char :1;
        } BIT;
    } PGC1;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PGCIn:2;
            unsigned char PGCOVEn:1;
            unsigned char :1;
            unsigned char PGCOn:3;
            unsigned char :1;
        } BIT;
    } PGC2;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PDBFn0:1;
            unsigned char PDBFn1:1;
            unsigned char PDBFn2:1;
            unsigned char PDBFn3:1;
            unsigned char PDBFn4:1;
            unsigned char PDBFn5:1;
            unsigned char PDBFn6:1;
            unsigned char PDBFn7:1;
        } BIT;
    } PDBF1;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PDBFn0:1;
            unsigned char PDBFn1:1;
            unsigned char PDBFn2:1;
            unsigned char PDBFn3:1;
            unsigned char PDBFn4:1;
            unsigned char PDBFn5:1;
            unsigned char PDBFn6:1;
            unsigned char PDBFn7:1;
        } BIT;
    } PDBF2;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSBn:3;
            unsigned char PSPn:2;
            unsigned char PSMn:2;
            unsigned char :1;
        } BIT;
    } PEL0;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSBn:3;
            unsigned char PSPn:2;
            unsigned char PSMn:2;
            unsigned char :1;
        } BIT;
    } PEL1;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSBn:3;
            unsigned char PSPn:2;
            unsigned char PSMn:2;
            unsigned char :1;
        } BIT;
    } PEL2;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSBn:3;
            unsigned char PSPn:2;
            unsigned char PSMn:2;
            unsigned char :1;
        } BIT;
    } PEL3;
    union {
        unsigned char BYTE;
        struct {
            unsigned char SEG:1;
            unsigned char :5;
            unsigned char WE:1;
            unsigned char WI:1;
        } BIT;
    } ELSEGR;
    char           wk6[3];
    union {
        unsigned char BYTE;
        struct {
            unsigned char ELS:8;
        } BIT;
    } ELSR33;
    char           wk7[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char ELS:8;
        } BIT;
    } ELSR35;
    union {
        unsigned char BYTE;
        struct {
            unsigned char ELS:8;
        } BIT;
    } ELSR36;
    union {
        unsigned char BYTE;
        struct {
            unsigned char ELS:8;
        } BIT;
    } ELSR37;
    union {
        unsigned char BYTE;
        struct {
            unsigned char ELS:8;
        } BIT;
    } ELSR38;
    char           wk8[2];
    union {
        unsigned char BYTE;
        struct {
            unsigned char ELS:8;
        } BIT;
    } ELSR41;
    union {
        unsigned char BYTE;
        struct {
            unsigned char ELS:8;
        } BIT;
    } ELSR42;
    union {
        unsigned char BYTE;
        struct {
            unsigned char ELS:8;
        } BIT;
    } ELSR43;
    union {
        unsigned char BYTE;
        struct {
            unsigned char ELS:8;
        } BIT;
    } ELSR44;
    union {
        unsigned char BYTE;
        struct {
            unsigned char ELS:8;
        } BIT;
    } ELSR45;
    char           wk9[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TPU0MD:2;
            unsigned char TPU1MD:2;
            unsigned char TPU2MD:2;
            unsigned char TPU3MD:2;
        } BIT;
    } ELOPF;
    char           wk10[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char CMTW0MD:2;
            unsigned char :6;
        } BIT;
    } ELOPH;
    union {
        unsigned char BYTE;
        struct {
            unsigned char GPT0MD:3;
            unsigned char :1;
            unsigned char GPT1MD:3;
            unsigned char :1;
        } BIT;
    } ELOPI;
    union {
        unsigned char BYTE;
        struct {
            unsigned char GPT2MD:3;
            unsigned char :1;
            unsigned char GPT3MD:3;
            unsigned char :1;
        } BIT;
    } ELOPJ;
};

struct st_etherc {
    union {
        unsigned long LONG;
    } ETSPCMD;
    union {
        unsigned long LONG;
        struct {
            unsigned long MAC:3;
            unsigned long :29;
        } BIT;
    } MACSEL;
    union {
        unsigned long LONG;
        struct {
            unsigned long MODE:5;
            unsigned long :3;
            unsigned long FULLD:1;
            unsigned long :1;
            unsigned long RMII_CRS_MODE:1;
            unsigned long :21;
        } BIT;
    } MII_CTRL0;
    union {
        unsigned long LONG;
        struct {
            unsigned long MODE:5;
            unsigned long :3;
            unsigned long FULLD:1;
            unsigned long :1;
            unsigned long RMII_CRS_MODE:1;
            unsigned long :21;
        } BIT;
    } MII_CTRL1;
    union {
        unsigned long LONG;
        struct {
            unsigned long MODE:5;
            unsigned long :3;
            unsigned long FULLD:1;
            unsigned long :1;
            unsigned long RMII_CRS_MODE:1;
            unsigned long :21;
        } BIT;
    } MII_CTRL2;
    char           wk0[260];
    union {
        unsigned long LONG;
        struct {
            unsigned long CATRST:1;
            unsigned long SWRST:1;
            unsigned long PHYRST:1;
            unsigned long PHYRST2:1;
            unsigned long MIICRST:1;
            unsigned long :27;
        } BIT;
    } ETHSFTRST;
    char           wk1[196324];
    union {
        unsigned long LONG;
        struct {
            unsigned long SYSC:16;
            unsigned long :16;
        } BIT;
    } SYSC;
    union {
        unsigned long LONG;
        struct {
            unsigned long R4B:32;
        } BIT;
    } R4;
    union {
        unsigned long LONG;
        struct {
            unsigned long R5B:32;
        } BIT;
    } R5;
    union {
        unsigned long LONG;
        struct {
            unsigned long R6B:32;
        } BIT;
    } R6;
    union {
        unsigned long LONG;
        struct {
            unsigned long R7B:32;
        } BIT;
    } R7;
    char           wk2[12];
    union {
        unsigned long LONG;
        struct {
            unsigned long R0B:32;
        } BIT;
    } R0;
    union {
        unsigned long LONG;
        struct {
            unsigned long R1B:32;
        } BIT;
    } R1;
    char           wk3[4068];
    union {
        unsigned long LONG;
        struct {
            unsigned long TXID:32;
        } BIT;
    } GMAC_TXID;
    union {
        unsigned long LONG;
        struct {
            unsigned long FIFOUFLOW:1;
            unsigned long RETRYN:4;
            unsigned long LCOLLIS:1;
            unsigned long UNDERFW:1;
            unsigned long OVERFW:1;
            unsigned long CSERR:1;
            unsigned long MCOLLIS:1;
            unsigned long SCOLLIS:1;
            unsigned long TFAIL:1;
            unsigned long TABT:1;
            unsigned long TCMP:1;
            unsigned long :18;
        } BIT;
    } GMAC_TXRESULT;
    char           wk4[12];
    union {
        unsigned long LONG;
        struct {
            unsigned long :30;
            unsigned long DUPMODE:1;
            unsigned long ETHMODE:1;
        } BIT;
    } GMAC_MODE;
    union {
        unsigned long LONG;
        struct {
            unsigned long :9;
            unsigned long RRTTH:3;
            unsigned long RFULLTH:2;
            unsigned long REMPTH:2;
            unsigned long :12;
            unsigned long RAMASKEN:1;
            unsigned long SFRXFIFO:1;
            unsigned long MFILLTEREN:1;
            unsigned long AFILLTEREN:1;
        } BIT;
    } GMAC_RXMODE;
    union {
        unsigned long LONG;
        struct {
            unsigned long :6;
            unsigned long TRBMODE:2;
            unsigned long :1;
            unsigned long TFULLTH:2;
            unsigned long TEMPTH:3;
            unsigned long FSTTH:2;
            unsigned long :10;
            unsigned long SFOP:1;
            unsigned long RTRANSLC:1;
            unsigned long SPTXEN:1;
            unsigned long SF:1;
            unsigned long LPTXEN:1;
            unsigned long RTRANSDEN:1;
        } BIT;
    } GMAC_TXMODE;
    char           wk5[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long :13;
            unsigned long RXRST:1;
            unsigned long :1;
            unsigned long TXRST:1;
            unsigned long :15;
            unsigned long ALLRST:1;
        } BIT;
    } GMAC_RESET;
    char           wk6[76];
    union {
        unsigned long LONG;
        struct {
            unsigned long PPDATA1:32;
        } BIT;
    } GMAC_PAUSE1;
    union {
        unsigned long LONG;
        struct {
            unsigned long PPDATA2:32;
        } BIT;
    } GMAC_PAUSE2;
    union {
        unsigned long LONG;
        struct {
            unsigned long PPDATA3:32;
        } BIT;
    } GMAC_PAUSE3;
    union {
        unsigned long LONG;
        struct {
            unsigned long PPDATA4:32;
        } BIT;
    } GMAC_PAUSE4;
    union {
        unsigned long LONG;
        struct {
            unsigned long PPDATA5:32;
        } BIT;
    } GMAC_PAUSE5;
    char           wk7[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long :31;
            unsigned long PPRXEN:1;
        } BIT;
    } GMAC_FLWCTL;
    union {
        unsigned long LONG;
        struct {
            unsigned long :31;
            unsigned long PPR:1;
        } BIT;
    } GMAC_PAUSPKT;
    union {
        unsigned long LONG;
        struct {
            unsigned long DATA:16;
            unsigned long REGADDR:5;
            unsigned long PHYADDR:5;
            unsigned long RWDV:1;
            unsigned long :5;
        } BIT;
    } GMAC_MIIM;
    char           wk8[92];
    union {
        unsigned long LONG;
        struct {
            unsigned long MADDR1B:8;
            unsigned long MADDR2B:8;
            unsigned long MADDR3B:8;
            unsigned long MADDR4B:8;
        } BIT;
    } GMAC_ADR0A;
    union {
        unsigned long LONG;
        struct {
            unsigned long MADDR5B:8;
            unsigned long MADDR6B:8;
            unsigned long BITMSK:8;
            unsigned long :8;
        } BIT;
    } GMAC_ADR0B;
    union {
        unsigned long LONG;
        struct {
            unsigned long MADDR1B:8;
            unsigned long MADDR2B:8;
            unsigned long MADDR3B:8;
            unsigned long MADDR4B:8;
        } BIT;
    } GMAC_ADR1A;
    union {
        unsigned long LONG;
        struct {
            unsigned long MADDR5B:8;
            unsigned long MADDR6B:8;
            unsigned long BITMSK:8;
            unsigned long :8;
        } BIT;
    } GMAC_ADR1B;
    union {
        unsigned long LONG;
        struct {
            unsigned long MADDR1B:8;
            unsigned long MADDR2B:8;
            unsigned long MADDR3B:8;
            unsigned long MADDR4B:8;
        } BIT;
    } GMAC_ADR2A;
    union {
        unsigned long LONG;
        struct {
            unsigned long MADDR5B:8;
            unsigned long MADDR6B:8;
            unsigned long BITMSK:8;
            unsigned long :8;
        } BIT;
    } GMAC_ADR2B;
    union {
        unsigned long LONG;
        struct {
            unsigned long MADDR1B:8;
            unsigned long MADDR2B:8;
            unsigned long MADDR3B:8;
            unsigned long MADDR4B:8;
        } BIT;
    } GMAC_ADR3A;
    union {
        unsigned long LONG;
        struct {
            unsigned long MADDR5B:8;
            unsigned long MADDR6B:8;
            unsigned long BITMSK:8;
            unsigned long :8;
        } BIT;
    } GMAC_ADR3B;
    union {
        unsigned long LONG;
        struct {
            unsigned long MADDR1B:8;
            unsigned long MADDR2B:8;
            unsigned long MADDR3B:8;
            unsigned long MADDR4B:8;
        } BIT;
    } GMAC_ADR4A;
    union {
        unsigned long LONG;
        struct {
            unsigned long MADDR5B:8;
            unsigned long MADDR6B:8;
            unsigned long BITMSK:8;
            unsigned long :8;
        } BIT;
    } GMAC_ADR4B;
    union {
        unsigned long LONG;
        struct {
            unsigned long MADDR1B:8;
            unsigned long MADDR2B:8;
            unsigned long MADDR3B:8;
            unsigned long MADDR4B:8;
        } BIT;
    } GMAC_ADR5A;
    union {
        unsigned long LONG;
        struct {
            unsigned long MADDR5B:8;
            unsigned long MADDR6B:8;
            unsigned long BITMSK:8;
            unsigned long :8;
        } BIT;
    } GMAC_ADR5B;
    union {
        unsigned long LONG;
        struct {
            unsigned long MADDR1B:8;
            unsigned long MADDR2B:8;
            unsigned long MADDR3B:8;
            unsigned long MADDR4B:8;
        } BIT;
    } GMAC_ADR6A;
    union {
        unsigned long LONG;
        struct {
            unsigned long MADDR5B:8;
            unsigned long MADDR6B:8;
            unsigned long BITMSK:8;
            unsigned long :8;
        } BIT;
    } GMAC_ADR6B;
    union {
        unsigned long LONG;
        struct {
            unsigned long MADDR1B:8;
            unsigned long MADDR2B:8;
            unsigned long MADDR3B:8;
            unsigned long MADDR4B:8;
        } BIT;
    } GMAC_ADR7A;
    union {
        unsigned long LONG;
        struct {
            unsigned long MADDR5B:8;
            unsigned long MADDR6B:8;
            unsigned long BITMSK:8;
            unsigned long :8;
        } BIT;
    } GMAC_ADR7B;
    union {
        unsigned long LONG;
        struct {
            unsigned long MADDR1B:8;
            unsigned long MADDR2B:8;
            unsigned long MADDR3B:8;
            unsigned long MADDR4B:8;
        } BIT;
    } GMAC_ADR8A;
    union {
        unsigned long LONG;
        struct {
            unsigned long MADDR5B:8;
            unsigned long MADDR6B:8;
            unsigned long BITMSK:8;
            unsigned long :8;
        } BIT;
    } GMAC_ADR8B;
    union {
        unsigned long LONG;
        struct {
            unsigned long MADDR1B:8;
            unsigned long MADDR2B:8;
            unsigned long MADDR3B:8;
            unsigned long MADDR4B:8;
        } BIT;
    } GMAC_ADR9A;
    union {
        unsigned long LONG;
        struct {
            unsigned long MADDR5B:8;
            unsigned long MADDR6B:8;
            unsigned long BITMSK:8;
            unsigned long :8;
        } BIT;
    } GMAC_ADR9B;
    union {
        unsigned long LONG;
        struct {
            unsigned long MADDR1B:8;
            unsigned long MADDR2B:8;
            unsigned long MADDR3B:8;
            unsigned long MADDR4B:8;
        } BIT;
    } GMAC_ADR10A;
    union {
        unsigned long LONG;
        struct {
            unsigned long MADDR5B:8;
            unsigned long MADDR6B:8;
            unsigned long BITMSK:8;
            unsigned long :8;
        } BIT;
    } GMAC_ADR10B;
    union {
        unsigned long LONG;
        struct {
            unsigned long MADDR1B:8;
            unsigned long MADDR2B:8;
            unsigned long MADDR3B:8;
            unsigned long MADDR4B:8;
        } BIT;
    } GMAC_ADR11A;
    union {
        unsigned long LONG;
        struct {
            unsigned long MADDR5B:8;
            unsigned long MADDR6B:8;
            unsigned long BITMSK:8;
            unsigned long :8;
        } BIT;
    } GMAC_ADR11B;
    union {
        unsigned long LONG;
        struct {
            unsigned long MADDR1B:8;
            unsigned long MADDR2B:8;
            unsigned long MADDR3B:8;
            unsigned long MADDR4B:8;
        } BIT;
    } GMAC_ADR12A;
    union {
        unsigned long LONG;
        struct {
            unsigned long MADDR5B:8;
            unsigned long MADDR6B:8;
            unsigned long BITMSK:8;
            unsigned long :8;
        } BIT;
    } GMAC_ADR12B;
    union {
        unsigned long LONG;
        struct {
            unsigned long MADDR1B:8;
            unsigned long MADDR2B:8;
            unsigned long MADDR3B:8;
            unsigned long MADDR4B:8;
        } BIT;
    } GMAC_ADR13A;
    union {
        unsigned long LONG;
        struct {
            unsigned long MADDR5B:8;
            unsigned long MADDR6B:8;
            unsigned long BITMSK:8;
            unsigned long :8;
        } BIT;
    } GMAC_ADR13B;
    union {
        unsigned long LONG;
        struct {
            unsigned long MADDR1B:8;
            unsigned long MADDR2B:8;
            unsigned long MADDR3B:8;
            unsigned long MADDR4B:8;
        } BIT;
    } GMAC_ADR14A;
    union {
        unsigned long LONG;
        struct {
            unsigned long MADDR5B:8;
            unsigned long MADDR6B:8;
            unsigned long BITMSK:8;
            unsigned long :8;
        } BIT;
    } GMAC_ADR14B;
    union {
        unsigned long LONG;
        struct {
            unsigned long MADDR1B:8;
            unsigned long MADDR2B:8;
            unsigned long MADDR3B:8;
            unsigned long MADDR4B:8;
        } BIT;
    } GMAC_ADR15A;
    union {
        unsigned long LONG;
        struct {
            unsigned long MADDR5B:8;
            unsigned long MADDR6B:8;
            unsigned long BITMSK:8;
            unsigned long :8;
        } BIT;
    } GMAC_ADR15B;
    char           wk9[128];
    union {
        unsigned long LONG;
        struct {
            unsigned long :17;
            unsigned long RSW:12;
            unsigned long RRT:1;
            unsigned long REMP:1;
            unsigned long RFULL:1;
        } BIT;
    } GMAC_RXFIFO;
    union {
        unsigned long LONG;
        struct {
            unsigned long :24;
            unsigned long TRBFR:3;
            unsigned long TSTATUS:3;
            unsigned long TEMP:1;
            unsigned long TFULL:1;
        } BIT;
    } GMAC_TXFIFO;
    union {
        unsigned long LONG;
        struct {
            unsigned long RTCPIPEN:1;
            unsigned long TTCPIPEN:1;
            unsigned long RTCPIPACC:1;
            unsigned long :29;
        } BIT;
    } GMAC_ACC;
    char           wk10[20];
    union {
        unsigned long LONG;
        struct {
            unsigned long RMACEN:1;
            unsigned long :31;
        } BIT;
    } GMAC_RXMAC_ENA;
    union {
        unsigned long LONG;
        struct {
            unsigned long :31;
            unsigned long LPMEN:1;
        } BIT;
    } GMAC_LPI_MODE;
    union {
        unsigned long LONG;
        struct {
            unsigned long LPWTIME:16;
            unsigned long LPRDEF:16;
        } BIT;
    } GMAC_LPI_TIMING;
    char           wk11[3796];
    union {
        unsigned long LONG;
        struct {
            unsigned long ADDR:16;
            unsigned long WORD:12;
            unsigned long VALID:1;
            unsigned long :2;
            unsigned long NOEMP:1;
        } BIT;
    } BUFID;
    char           wk12[4092];
    union {
        unsigned long LONG;
    } SPCMD;
    char           wk13[12];
    union {
        unsigned long LONG;
        struct {
            unsigned long EMACRST:1;
            unsigned long :31;
        } BIT;
    } EMACRST;
};

struct st_ethersw {
    union {
        unsigned long LONG;
        struct {
            unsigned long SWLINK0:1;
            unsigned long SWLINK1:1;
            unsigned long CATLINK0:1;
            unsigned long CATLINK1:1;
            unsigned long :28;
        } BIT;
    } ETHPHYLNK;
    char           wk0[248];
    union {
        unsigned long LONG;
        struct {
            unsigned long SWTAGTYP:16;
            unsigned long :15;
            unsigned long SWTAGEN:1;
        } BIT;
    } ETHSWMTC;
    union {
        unsigned long LONG;
        struct {
            unsigned long :1;
            unsigned long P0HDMODE:1;
            unsigned long :1;
            unsigned long P1HDMODE:1;
            unsigned long :28;
        } BIT;
    } ETHSWMD;
    char           wk1[232];
    union {
        unsigned long LONG;
        struct {
            unsigned long OUTEN:1;
            unsigned long :31;
        } BIT;
    } SWTMEN;
    union {
        unsigned long LONG;
        struct {
            unsigned long TMSTSEC:32;
        } BIT;
    } SWTMSTSEC;
    union {
        unsigned long LONG;
        struct {
            unsigned long TMSTNS:32;
        } BIT;
    } SWTMSTNS;
    union {
        unsigned long LONG;
        struct {
            unsigned long TMPSEC:32;
        } BIT;
    } SWTMPSEC;
    union {
        unsigned long LONG;
        struct {
            unsigned long TMPNS:32;
        } BIT;
    } SWTMPNS;
    union {
        unsigned long LONG;
        struct {
            unsigned long TMWTH:16;
            unsigned long :16;
        } BIT;
    } SWTMWTH;
    char           wk2[20];
    union {
        unsigned long LONG;
        struct {
            unsigned long TMLATSEC:32;
        } BIT;
    } SWTMLATSEC;
    union {
        unsigned long LONG;
        struct {
            unsigned long TMLATNS:32;
        } BIT;
    } SWTMLATNS;
    char           wk3[3540];
    union {
        unsigned long LONG;
        struct {
            unsigned long P0ENA:1;
            unsigned long P1ENA:1;
            unsigned long P2ENA:1;
            unsigned long :29;
        } BIT;
    } PORT_ENA;
    union {
        unsigned long LONG;
        struct {
            unsigned long P0UCASTDM:1;
            unsigned long P1UCASTDM:1;
            unsigned long P2UCASTDM:1;
            unsigned long :29;
        } BIT;
    } UCAST_DEFAULT_MASK;
    char           wk4[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long P0BCASTDM:1;
            unsigned long P1BCASTDM:1;
            unsigned long P2BCASTDM:1;
            unsigned long :29;
        } BIT;
    } BCAST_DEFAULT_MASK;
    union {
        unsigned long LONG;
        struct {
            unsigned long P0MCASTDM:1;
            unsigned long P1MCASTDM:1;
            unsigned long P2MCASTDM:1;
            unsigned long :29;
        } BIT;
    } MCAST_DEFAULT_MASK;
    union {
        unsigned long LONG;
        struct {
            unsigned long P0BLOCKEN:1;
            unsigned long P1BLOCKEN:1;
            unsigned long P2BLOCKEN:1;
            unsigned long :13;
            unsigned long P0LEARNDIS:1;
            unsigned long P1LEARNDIS:1;
            unsigned long P2LEARNDIS:1;
            unsigned long :13;
        } BIT;
    } INPUT_LERAN_BLOCK;
    union {
        unsigned long LONG;
        struct {
            unsigned long PORT:2;
            unsigned long :3;
            unsigned long MSGTRANS:1;
            unsigned long ENABLE:1;
            unsigned long DISCARD:1;
            unsigned long :5;
            unsigned long PRIORITY:3;
            unsigned long P0PORTMASK:1;
            unsigned long P1PORTMASK:1;
            unsigned long :14;
        } BIT;
    } MGMT_CONFIG;
    union {
        unsigned long LONG;
        struct {
            unsigned long :31;
            unsigned long STATSRESET:1;
        } BIT;
    } MODE_CONFIG;
    char           wk5[12];
    union {
        unsigned long LONG;
        struct {
            unsigned long VLANTAGID:16;
            unsigned long :16;
        } BIT;
    } VLAN_TAG_ID;
    char           wk6[72];
    union {
        unsigned long LONG;
        struct {
            unsigned long BUSYINIT:1;
            unsigned long NOCELL:1;
            unsigned long MEMFULL:1;
            unsigned long MEMFULL_LT:1;
            unsigned long :2;
            unsigned long DEQUEGRANT:1;
            unsigned long :9;
            unsigned long CELLAVILABLE:16;
        } BIT;
    } OQMGR_STATUS;
    union {
        unsigned long LONG;
        struct {
            unsigned long MINCELLS:5;
            unsigned long :27;
        } BIT;
    } QMGR_MINCELLS;
    union {
        unsigned long LONG;
        struct {
            unsigned long STMINCELLS:5;
            unsigned long :27;
        } BIT;
    } QMGR_ST_MINCELLS;
    union {
        unsigned long LONG;
        struct {
            unsigned long P0CGS:1;
            unsigned long P1CGS:1;
            unsigned long P2CGS:1;
            unsigned long :29;
        } BIT;
    } QMGR_CGS_STAT;
    union {
        unsigned long LONG;
        struct {
            unsigned long P0TXFIFOST:1;
            unsigned long P1TXFIFOST:1;
            unsigned long P2TXFIFOST:1;
            unsigned long :13;
            unsigned long P0RXFIFOAV:1;
            unsigned long P1RXFIFOAV:1;
            unsigned long P2RXFIFOAV:1;
            unsigned long :13;
        } BIT;
    } QMGR_IFACE_STAT;
    union {
        unsigned long LONG;
        struct {
            unsigned long QUEUE0:5;
            unsigned long :3;
            unsigned long QUEUE1:5;
            unsigned long :3;
            unsigned long QUEUE2:5;
            unsigned long :3;
            unsigned long QUEUE3:5;
            unsigned long :3;
        } BIT;
    } QMGR_WEIGHTS;
    char           wk7[104];
    union {
        unsigned long LONG;
        struct {
            unsigned long PRIORITY0:3;
            unsigned long PRIORITY1:3;
            unsigned long PRIORITY2:3;
            unsigned long PRIORITY3:3;
            unsigned long PRIORITY4:3;
            unsigned long PRIORITY5:3;
            unsigned long PRIORITY6:3;
            unsigned long PRIORITY7:3;
            unsigned long :8;
        } BIT;
    } VLAN_PRIORITY[3];
    char           wk8[52];
    union {
        unsigned long LONG;
        struct {
            unsigned long ADDRESS:8;
            unsigned long IPV6SELECT:1;
            unsigned long PRIORITY:2;
            unsigned long :20;
            unsigned long READ:1;
        } BIT;
    } IP_PRIORITY0;
    union {
        unsigned long LONG;
        struct {
            unsigned long ADDRESS:8;
            unsigned long IPV6SELECT:1;
            unsigned long PRIORITY:2;
            unsigned long :20;
            unsigned long READ:1;
        } BIT;
    } IP_PRIORITY1;
    union {
        unsigned long LONG;
        struct {
            unsigned long ADDRESS:8;
            unsigned long IPV6SELECT:1;
            unsigned long PRIORITY:2;
            unsigned long :20;
            unsigned long READ:1;
        } BIT;
    } IP_PRIORITY2;
    char           wk9[52];
    union {
        unsigned long LONG;
        struct {
            unsigned long VLANEN:1;
            unsigned long IPEN:1;
            unsigned long :2;
            unsigned long DEFAULTPRI:3;
            unsigned long :25;
        } BIT;
    } PRIORITY_CFG[3];
    char           wk10[52];
    union {
        unsigned long LONG;
        struct {
            unsigned long HUBEN:1;
            unsigned long DIR0TO1EN:1;
            unsigned long DIR1TO0EN:1;
            unsigned long BROCAFILEN:1;
            unsigned long HUBIPG:4;
            unsigned long :24;
        } BIT;
    } HUB_CONTROL;
    union {
        unsigned long LONG;
        struct {
            unsigned long NUM1TO0:32;
        } BIT;
    } HUB_STATS;
    union {
        unsigned long LONG;
        struct {
            unsigned long MACADD1n:8;
            unsigned long MACADD2n:8;
            unsigned long MACADD3n:8;
            unsigned long MACADD4n:8;
        } BIT;
    } HUB_FLT_MAC0lo;
    union {
        unsigned long LONG;
        struct {
            unsigned long MACADD5n:8;
            unsigned long MACADD6n:8;
            unsigned long MASKCOMP:8;
            unsigned long FORCEFOW:1;
            unsigned long :7;
        } BIT;
    } HUB_FLT_MAC0hi;
    union {
        unsigned long LONG;
        struct {
            unsigned long MACADD1n:8;
            unsigned long MACADD2n:8;
            unsigned long MACADD3n:8;
            unsigned long MACADD4n:8;
        } BIT;
    } HUB_FLT_MAC1lo;
    union {
        unsigned long LONG;
        struct {
            unsigned long MACADD5n:8;
            unsigned long MACADD6n:8;
            unsigned long MASKCOMP:8;
            unsigned long FORCEFOW:1;
            unsigned long :7;
        } BIT;
    } HUB_FLT_MAC1hi;
    union {
        unsigned long LONG;
        struct {
            unsigned long MACADD1n:8;
            unsigned long MACADD2n:8;
            unsigned long MACADD3n:8;
            unsigned long MACADD4n:8;
        } BIT;
    } HUB_FLT_MAC2lo;
    union {
        unsigned long LONG;
        struct {
            unsigned long MACADD5n:8;
            unsigned long MACADD6n:8;
            unsigned long MASKCOMP:8;
            unsigned long FORCEFOW:1;
            unsigned long :7;
        } BIT;
    } HUB_FLT_MAC2hi;
    union {
        unsigned long LONG;
        struct {
            unsigned long MACADD1n:8;
            unsigned long MACADD2n:8;
            unsigned long MACADD3n:8;
            unsigned long MACADD4n:8;
        } BIT;
    } HUB_FLT_MAC3lo;
    union {
        unsigned long LONG;
        struct {
            unsigned long MACADD5n:8;
            unsigned long MACADD6n:8;
            unsigned long MASKCOMP:8;
            unsigned long FORCEFOW:1;
            unsigned long :7;
        } BIT;
    } HUB_FLT_MAC3hi;
    union {
        unsigned long LONG;
        struct {
            unsigned long MACADD1n:8;
            unsigned long MACADD2n:8;
            unsigned long MACADD3n:8;
            unsigned long MACADD4n:8;
        } BIT;
    } HUB_FLT_MAC4lo;
    union {
        unsigned long LONG;
        struct {
            unsigned long MACADD5n:8;
            unsigned long MACADD6n:8;
            unsigned long MASKCOMP:8;
            unsigned long FORCEFOW:1;
            unsigned long :7;
        } BIT;
    } HUB_FLT_MAC4hi;
    union {
        unsigned long LONG;
        struct {
            unsigned long MACADD1n:8;
            unsigned long MACADD2n:8;
            unsigned long MACADD3n:8;
            unsigned long MACADD4n:8;
        } BIT;
    } HUB_FLT_MAC5lo;
    union {
        unsigned long LONG;
        struct {
            unsigned long MACADD5n:8;
            unsigned long MACADD6n:8;
            unsigned long MASKCOMP:8;
            unsigned long FORCEFOW:1;
            unsigned long :7;
        } BIT;
    } HUB_FLT_MAC5hi;
    union {
        unsigned long LONG;
        struct {
            unsigned long MACADD1n:8;
            unsigned long MACADD2n:8;
            unsigned long MACADD3n:8;
            unsigned long MACADD4n:8;
        } BIT;
    } HUB_FLT_MAC6lo;
    union {
        unsigned long LONG;
        struct {
            unsigned long MACADD5n:8;
            unsigned long MACADD6n:8;
            unsigned long MASKCOMP:8;
            unsigned long FORCEFOW:1;
            unsigned long :7;
        } BIT;
    } HUB_FLT_MAC6hi;
    char           wk11[256];
    unsigned long  TOTAL_BYT_FRM;
    unsigned long  TOTAL_BYT_DISC;
    unsigned long  TOTAL_FRM;
    unsigned long  TOTAL_DISC;
    unsigned long  ODISC0;
    unsigned long  IDISC_BLOCKED0;
    unsigned long  ODISC1;
    unsigned long  IDISC_BLOCKED1;
    unsigned long  ODISC2;
    unsigned long  IDISC_BLOCKED2;
    char           wk12[472];
    union {
        unsigned long LONG;
        struct {
            unsigned long SRCADD1:8;
            unsigned long SRCADD2:8;
            unsigned long SRCADD3:8;
            unsigned long SRCADD4:8;
        } BIT;
    } LRN_REC_A;
    union {
        unsigned long LONG;
        struct {
            unsigned long SRCADD5:8;
            unsigned long SRCADD6:8;
            unsigned long HASH:8;
            unsigned long PORT:4;
            unsigned long :4;
        } BIT;
    } LRN_REC_B;
    union {
        unsigned long LONG;
        struct {
            unsigned long LERNAVAL:1;
            unsigned long :31;
        } BIT;
    } LRN_STATUS;
    char           wk13[0x4000-0x050C];
    char           ADR_TABLE[0x8000-0x4000];
    struct {
        char           mac_wk01[8];
        union {
            unsigned long LONG;
            struct {
                unsigned long TXENA:1;
                unsigned long RXENA:1;
                unsigned long :11;
                unsigned long SWRESET:1;
                unsigned long :9;
                unsigned long CNTRLREMEN:1;
                unsigned long NOLGTHCHK:1;
                unsigned long :1;
                unsigned long RXERRDISC:1;
                unsigned long :4;
                unsigned long CNTRESET:1;
            } BIT;
        } COMMAND_CONFIG;
        char           mac_wk02[8];
        union {
            unsigned long LONG;
            struct {
                unsigned long FRMLEN:14;
                unsigned long :18;
            } BIT;
        } FRM_LENGTH;
        char           mac_wk03[4];
        unsigned long  RX_SECTION_EMPTY;
        unsigned long  RX_SECTION_FULL;
        unsigned long  TX_SECTION_EMPTY;
        unsigned long  TX_SECTION_FULL;
        unsigned long  RX_ALMOST_EMPTY;
        unsigned long  RX_ALMOST_FULL;
        unsigned long  TX_ALMOST_EMPTY;
        unsigned long  TX_ALMOST_FULL;
        char           mac_wk04[28];
        union {
            unsigned long LONG;
            struct {
                unsigned long :8;
                unsigned long SPEEDP0:1;
                unsigned long :1;
                unsigned long HDPP0:1;
                unsigned long :1;
                unsigned long SPEEDP1:1;
                unsigned long :1;
                unsigned long HDPP1:1;
                unsigned long :17;
            } BIT;
        } MAC_STATUS;
        union {
            unsigned long LONG;
            struct {
                unsigned long TXIPGLEN:5;
                unsigned long :27;
            } BIT;
        } TX_IPG_LENGTH;
        char           mac_wk05[160];
        unsigned long  etherStatsOctets;
        unsigned long  OctetsOK;
        unsigned long  aAlignmentErrors;
        unsigned long  aPAUSEMACCtrlFrames;
        unsigned long  FramesOK;
        unsigned long  CRCErrors;
        unsigned long  VLANOK;
        unsigned long  ifInErrors;
        unsigned long  ifInUcastPkts;
        unsigned long  ifInMulticastPkts;
        unsigned long  ifInBroadcastPkts;
        unsigned long  etherStatsDropEvents;
        unsigned long  etherStatsPkts;
        unsigned long  etherStatsUndersizePkts;
        unsigned long  etherStatsPkts64Octets;
        unsigned long  etherStatsPkts65to127Octets;
        unsigned long  etherStatsPkts128to255Octets;
        unsigned long  etherStatsPkts256to511Octets;
        unsigned long  etherStatsPkts512to1023Octets;
        unsigned long  etherStatsPkts1024to1518Octets;
        unsigned long  etherStatsPkts1519toMax;
        unsigned long  etherStatsOversizePkts;
        unsigned long  etherStatsJabbers;
        unsigned long  etherStatsFragments;
        unsigned long  aMACControlFramesReceived;
        unsigned long  aFrameTooLong;
        char           mac_wk06[4];
        unsigned long  StackedVLANOK;
        char           mac_wk07[16];
        unsigned long  TXetherStatsOctets;
        unsigned long  TxOctetsOK;
        char           mac_wk08[4];
        unsigned long  TXaPAUSEMACCtrlFrames;
        unsigned long  TxFramesOK;
        unsigned long  TxCRCErrors;
        unsigned long  TxVLANOK;
        unsigned long  ifOutErrors;
        unsigned long  ifUcastPkts;
        unsigned long  ifMulticastPkts;
        unsigned long  ifBroadcastPkts;
        unsigned long  TXetherStatsDropEvents;
        unsigned long  TXetherStatsPkts;
        unsigned long  TXetherStatsUndersizePkts;
        unsigned long  TXetherStatsPkts64Octets;
        unsigned long  TXetherStatsPkts65to127Octets;
        unsigned long  TXetherStatsPkts128to255Octets;
        unsigned long  TXetherStatsPkts256to511Octets;
        unsigned long  TXetherStatsPkts512to1023Octets;
        unsigned long  TXetherStatsPkts1024to1518Octets;
        unsigned long  TXetherStatsPkts1519toMax;
        unsigned long  TXetherStatsOversizePkts;
        unsigned long  TXetherStatsJabbers;
        unsigned long  TXetherStatsFragments;
        unsigned long  aMACControlFrames;
        unsigned long  TXaFrameTooLong;
        char           mac_wk09[4];
        unsigned long  aMultipleCollisions;
        unsigned long  aSingleCollisions;
        unsigned long  aLateCollisions;
        unsigned long  aExcessCollisions;
        char           mac_wk10[0xA000-0x81FC];
    } MAC[2];
    char           wk32[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long IRQENA:1;
            unsigned long IRQEVTOFF:1;
            unsigned long IRQEVTPERD:1;
            unsigned long IRQTIMOVER:1;
            unsigned long IRQTEST:1;
            unsigned long :7;
            unsigned long IRQTXENAP0:1;
            unsigned long IRQTXENAP1:1;
            unsigned long :18;
        } BIT;
    } TSM_CONFIG;
    union {
        unsigned long LONG;
        struct {
            unsigned long IRQENA:1;
            unsigned long IRQEVTOFF:1;
            unsigned long IRQEVTPERD:1;
            unsigned long IRQTIMOVER:1;
            unsigned long IRQTEST:1;
            unsigned long :7;
            unsigned long IRQTXP0:1;
            unsigned long IRQTXP1:1;
            unsigned long :18;
        } BIT;
    } TSM_IRQ_STAT_ACK;
    char           wk33[20];
    union {
        unsigned long LONG;
        struct {
            unsigned long TSVALID:1;
            unsigned long TSOVR:1;
            unsigned long TSKEEP:1;
            unsigned long :29;
        } BIT;
    } PORT0_CTRL;
    union {
        unsigned long LONG;
        struct {
            unsigned long TSREG:32;
        } BIT;
    } PORT0_TIME;
    union {
        unsigned long LONG;
        struct {
            unsigned long TSVALID:1;
            unsigned long TSOVR:1;
            unsigned long TSKEEP:1;
            unsigned long :29;
        } BIT;
    } PORT1_CTRL;
    union {
        unsigned long LONG;
        struct {
            unsigned long TSREG:32;
        } BIT;
    } PORT1_TIME;
    char           wk34[240];
    union {
        unsigned long LONG;
        struct {
            unsigned long TMENA:1;
            unsigned long :1;
            unsigned long EVTOFFENA:1;
            unsigned long :1;
            unsigned long EVTPERIENA:1;
            unsigned long EVTPERIRST:1;
            unsigned long :3;
            unsigned long RST:1;
            unsigned long :1;
            unsigned long CAPTR:1;
            unsigned long PLUS1:1;
            unsigned long :19;
        } BIT;
    } ATIME_CTRL;
    union {
        unsigned long LONG;
        struct {
            unsigned long TMR:32;
        } BIT;
    } ATIME;
    union {
        unsigned long LONG;
        struct {
            unsigned long OFFSET:32;
        } BIT;
    } ATIME_OFFSET;
    union {
        unsigned long LONG;
        struct {
            unsigned long TIMPEREVET:32;
        } BIT;
    } ATIME_EVT_PERIOD;
    union {
        unsigned long LONG;
        struct {
            unsigned long DRIFCORVAL:31;
            unsigned long :1;
        } BIT;
    } ATIME_CORR;
    union {
        unsigned long LONG;
        struct {
            unsigned long CLKPERD:7;
            unsigned long :1;
            unsigned long CORRINC:7;
            unsigned long :1;
            unsigned long OFFSCORRINC:7;
            unsigned long :9;
        } BIT;
    } ATIME_INC;
    union {
        unsigned long LONG;
        struct {
            unsigned long SECTIM:32;
        } BIT;
    } ATIME_SEC;
    union {
        unsigned long LONG;
        struct {
            unsigned long OFFCOR:32;
        } BIT;
    } ATIME_CORR_OFFS;
    char           wk35[7872];
    union {
        unsigned long LONG;
        struct {
            unsigned long DLRENA:1;
            unsigned long :3;
            unsigned long BECTIMOUT:1;
            unsigned long :3;
            unsigned long CYCMCLK:8;
            unsigned long :16;
        } BIT;
    } DLR_CONTROL;
    union {
        unsigned long LONG;
        struct {
            unsigned long BEAREV0:1;
            unsigned long BEAREV1:1;
            unsigned long :6;
            unsigned long CURRSTA:8;
            unsigned long LINSTAP0:1;
            unsigned long LINSTAP1:1;
            unsigned long :6;
            unsigned long NETTOPGY:8;
        } BIT;
    } DLR_STATUS;
    union {
        unsigned long LONG;
        struct {
            unsigned long ETHTYPDLR:16;
            unsigned long :16;
        } BIT;
    } DLR_ETH_TYP;
    union {
        unsigned long LONG;
        struct {
            unsigned long IRQCHNGENA:1;
            unsigned long IRQFLUENA:1;
            unsigned long IRQSTOPP0:1;
            unsigned long IRQSTOPP1:1;
            unsigned long IRQBECTOUT0:1;
            unsigned long IRQBECTOUT1:1;
            unsigned long IRQSUPENA:1;
            unsigned long IRQLINKENA0:1;
            unsigned long IRQLINKENA1:1;
            unsigned long IRQSUPIGENA:1;
            unsigned long IRQIPADDREN:1;
            unsigned long IRQINVTMREN:1;
            unsigned long IRQBECENA0:1;
            unsigned long IRQBECENA1:1;
            unsigned long IRQFRMDSP0:1;
            unsigned long IRQFRMDSP1:1;
            unsigned long :14;
            unsigned long ATOMICOR:1;
            unsigned long ATOMICAND:1;
        } BIT;
    } DLR_IRQ_CTRL;
    union {
        unsigned long LONG;
        struct {
            unsigned long STACHANGE:1;
            unsigned long FLUEVENT:1;
            unsigned long STOPNBCHK0:1;
            unsigned long STOPNBCHK1:1;
            unsigned long BECTMRP0:1;
            unsigned long BECTMRP1:1;
            unsigned long SUPRCHAG:1;
            unsigned long LINKSTAP0:1;
            unsigned long LINKSTAP1:1;
            unsigned long SUPIGNBEC:1;
            unsigned long IPCHANEVET:1;
            unsigned long INVTMR:1;
            unsigned long BECFRAP0:1;
            unsigned long BECFRAP1:1;
            unsigned long FRMDISP0:1;
            unsigned long FRMDISP1:1;
            unsigned long :16;
        } BIT;
    } DLR_IRQ_STAT_ACK;
    union {
        unsigned long LONG;
        struct {
            unsigned long MACADD1:8;
            unsigned long MACADD2:8;
            unsigned long MACADD3:8;
            unsigned long MACADD4:8;
        } BIT;
    } LOC_MAClo;
    union {
        unsigned long LONG;
        struct {
            unsigned long MACADD5:8;
            unsigned long MACADD6:8;
            unsigned long :16;
        } BIT;
    } LOC_MAChi;
    char           wk36[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long MACADD1:8;
            unsigned long MACADD2:8;
            unsigned long MACADD3:8;
            unsigned long MACADD4:8;
        } BIT;
    } SUPR_MAClo;
    union {
        unsigned long LONG;
        struct {
            unsigned long MACADD5:8;
            unsigned long MACADD6:8;
            unsigned long SUPRPRE:8;
            unsigned long :8;
        } BIT;
    } SUPR_MAChi;
    union {
        unsigned long LONG;
        struct {
            unsigned long RINGSTATE:8;
            unsigned long VLANVALID:1;
            unsigned long :7;
            unsigned long VLANCI:16;
        } BIT;
    } STATE_VLAN;
    union {
        unsigned long LONG;
        struct {
            unsigned long BECTMOUT:32;
        } BIT;
    } BEC_TMOUT;
    union {
        unsigned long LONG;
        struct {
            unsigned long BECINTVAL:32;
        } BIT;
    } BEC_INTRVL;
    union {
        unsigned long LONG;
        struct {
            unsigned long SPVIP:32;
        } BIT;
    } SUPR_IPADR;
    union {
        unsigned long LONG;
        struct {
            unsigned long DLRRINGTPY:8;
            unsigned long DLRRINGVER:8;
            unsigned long SOURP:8;
            unsigned long :8;
        } BIT;
    } ETH_STYP_VER;
    union {
        unsigned long LONG;
        struct {
            unsigned long INVBECTMOUT:32;
        } BIT;
    } INV_TMOUT;
    unsigned long  SEQ_ID;
    char           wk37[28];
    unsigned long  RX_STAT0;
    unsigned long  RX_ERR_STAT0;
    unsigned long  TX_STAT0;
    char           wk38[4];
    unsigned long  RX_STAT1;
    unsigned long  RX_ERR_STAT1;
    unsigned long  TX_STAT1;
};

struct st_gpt {
    union {
        unsigned short WORD;
        struct {
            unsigned short CST0:1;
            unsigned short CST1:1;
            unsigned short CST2:1;
            unsigned short CST3:1;
            unsigned short :12;
        } BIT;
    } GTSTR;
    union {
        unsigned short WORD;
        struct {
            unsigned short NFA0EN:1;
            unsigned short NFB0EN:1;
            unsigned short NFA1EN:1;
            unsigned short NFB1EN:1;
            unsigned short NFA2EN:1;
            unsigned short NFB2EN:1;
            unsigned short NFA3EN:1;
            unsigned short NFB3EN:1;
            unsigned short NFCS0:2;
            unsigned short NFCS1:2;
            unsigned short NFCS2:2;
            unsigned short NFCS3:2;
        } BIT;
    } NFCR;
    union {
        unsigned short WORD;
        struct {
            unsigned short CSHW0:2;
            unsigned short CSHW1:2;
            unsigned short CSHW2:2;
            unsigned short CSHW3:2;
            unsigned short CPHW0:2;
            unsigned short CPHW1:2;
            unsigned short CPHW2:2;
            unsigned short CPHW3:2;
        } BIT;
    } GTHSCR;
    union {
        unsigned short WORD;
        struct {
            unsigned short CCHW0:2;
            unsigned short CCHW1:2;
            unsigned short CCHW2:2;
            unsigned short CCHW3:2;
            unsigned short CCSW0:1;
            unsigned short CCSW1:1;
            unsigned short CCSW2:1;
            unsigned short CCSW3:1;
            unsigned short :4;
        } BIT;
    } GTHCCR;
    union {
        unsigned short WORD;
        struct {
            unsigned short CSHSL0:4;
            unsigned short CSHSL1:4;
            unsigned short CSHSL2:4;
            unsigned short CSHSL3:4;
        } BIT;
    } GTHSSR;
    union {
        unsigned short WORD;
        struct {
            unsigned short CSHPL0:4;
            unsigned short CSHPL1:4;
            unsigned short CSHPL2:4;
            unsigned short CSHPL3:4;
        } BIT;
    } GTHPSR;
    union {
        unsigned short WORD;
        struct {
            unsigned short WP0:1;
            unsigned short WP1:1;
            unsigned short WP2:1;
            unsigned short WP3:1;
            unsigned short :12;
        } BIT;
    } GTWP;
    union {
        unsigned short WORD;
        struct {
            unsigned short SYNC0:2;
            unsigned short :2;
            unsigned short SYNC1:2;
            unsigned short :2;
            unsigned short SYNC2:2;
            unsigned short :2;
            unsigned short SYNC3:2;
            unsigned short :2;
        } BIT;
    } GTSYNC;
    union {
        unsigned short WORD;
        struct {
            unsigned short ETIPEN:1;
            unsigned short ETINEN:1;
            unsigned short :11;
            unsigned short GTENFCS:2;
            unsigned short GTETRGEN:1;
        } BIT;
    } GTETINT;
    char           wk0[2];
    union {
        unsigned short WORD;
        struct {
            unsigned short BD00:1;
            unsigned short BD01:1;
            unsigned short BD02:1;
            unsigned short BD03:1;
            unsigned short BD10:1;
            unsigned short BD11:1;
            unsigned short BD12:1;
            unsigned short BD13:1;
            unsigned short BD20:1;
            unsigned short BD21:1;
            unsigned short BD22:1;
            unsigned short BD23:1;
            unsigned short BD30:1;
            unsigned short BD31:1;
            unsigned short BD32:1;
            unsigned short BD33:1;
        } BIT;
    } GTBDR;
    char           wk1[2];
    union {
        unsigned short WORD;
        struct {
            unsigned short SWP0:1;
            unsigned short SWP1:1;
            unsigned short SWP2:1;
            unsigned short SWP3:1;
            unsigned short :12;
        } BIT;
    } GTSWP;
};

struct st_gpt0 {
    union {
        unsigned short WORD;
        struct {
            unsigned short GTIOA:6;
            unsigned short OADFLT:1;
            unsigned short OAHLD:1;
            unsigned short GTIOB:6;
            unsigned short OBDFLT:1;
            unsigned short OBHLD:1;
        } BIT;
    } GTIOR;
    union {
        unsigned short WORD;
        struct {
            unsigned short GTINTA:1;
            unsigned short GTINTB:1;
            unsigned short GTINTC:1;
            unsigned short GTINTD:1;
            unsigned short GTINTE:1;
            unsigned short GTINTF:1;
            unsigned short GTINTPR:2;
            unsigned short :3;
            unsigned short EINT:1;
            unsigned short ADTRAUEN:1;
            unsigned short ADTRADEN:1;
            unsigned short ADTRBUEN:1;
            unsigned short ADTRBDEN:1;
        } BIT;
    } GTINTAD;
    union {
        unsigned short WORD;
        struct {
            unsigned short MD:3;
            unsigned short :5;
            unsigned short TPCS:2;
            unsigned short :2;
            unsigned short CCLR:2;
            unsigned short :2;
        } BIT;
    } GTCR;
    union {
        unsigned short WORD;
        struct {
            unsigned short CCRA:2;
            unsigned short CCRB:2;
            unsigned short PR:2;
            unsigned short CCRSWT:1;
            unsigned short :1;
            unsigned short ADTTA:2;
            unsigned short ADTDA:1;
            unsigned short :1;
            unsigned short ADTTB:2;
            unsigned short ADTDB:1;
            unsigned short :1;
        } BIT;
    } GTBER;
    union {
        unsigned short WORD;
        struct {
            unsigned short UD:1;
            unsigned short UDF:1;
            unsigned short :14;
        } BIT;
    } GTUDC;
    union {
        unsigned short WORD;
        struct {
            unsigned short ITLA:1;
            unsigned short ITLB:1;
            unsigned short ITLC:1;
            unsigned short ITLD:1;
            unsigned short ITLE:1;
            unsigned short ITLF:1;
            unsigned short IVTC:2;
            unsigned short IVTT:3;
            unsigned short :1;
            unsigned short ADTAL:1;
            unsigned short :1;
            unsigned short ADTBL:1;
            unsigned short :1;
        } BIT;
    } GTITC;
    union {
        unsigned short WORD;
        struct {
            unsigned short :8;
            unsigned short ITCNT:3;
            unsigned short DTEF:1;
            unsigned short :3;
            unsigned short TUCF:1;
        } BIT;
    } GTST;
    unsigned short GTCNT;
    unsigned short GTCCRA;
    unsigned short GTCCRB;
    unsigned short GTCCRC;
    unsigned short GTCCRD;
    unsigned short GTCCRE;
    unsigned short GTCCRF;
    unsigned short GTPR;
    unsigned short GTPBR;
    unsigned short GTPDBR;
    char           wk0[2];
    unsigned short GTADTRA;
    unsigned short GTADTBRA;
    unsigned short GTADTDBRA;
    char           wk1[2];
    unsigned short GTADTRB;
    unsigned short GTADTBRB;
    unsigned short GTADTDBRB;
    char           wk2[2];
    union {
        unsigned short WORD;
        struct {
            unsigned short NEA:1;
            unsigned short NEB:1;
            unsigned short NVA:1;
            unsigned short NVB:1;
            unsigned short NFS:4;
            unsigned short NFV:1;
            unsigned short :3;
            unsigned short SWN:1;
            unsigned short :1;
            unsigned short OAE:1;
            unsigned short OBE:1;
        } BIT;
    } GTONCR;
    union {
        unsigned short WORD;
        struct {
            unsigned short TDE:1;
            unsigned short :3;
            unsigned short TDBUE:1;
            unsigned short TDBDE:1;
            unsigned short :2;
            unsigned short TDFER:1;
            unsigned short :7;
        } BIT;
    } GTDTCR;
    unsigned short GTDVU;
    unsigned short GTDVD;
    unsigned short GTDBU;
    unsigned short GTDBD;
    union {
        unsigned short WORD;
        struct {
            unsigned short SOS:2;
            unsigned short :14;
        } BIT;
    } GTSOS;
    union {
        unsigned short WORD;
        struct {
            unsigned short SOTR:1;
            unsigned short :15;
        } BIT;
    } GTSOTR;
};

struct st_icu {
    union {
        unsigned long LONG;
        struct {
            unsigned long :2;
            unsigned long IRQMD:2;
            unsigned long :28;
        } BIT;
    } IRQCR0;
    union {
        unsigned long LONG;
        struct {
            unsigned long :2;
            unsigned long IRQMD:2;
            unsigned long :28;
        } BIT;
    } IRQCR1;
    union {
        unsigned long LONG;
        struct {
            unsigned long :2;
            unsigned long IRQMD:2;
            unsigned long :28;
        } BIT;
    } IRQCR2;
    union {
        unsigned long LONG;
        struct {
            unsigned long :2;
            unsigned long IRQMD:2;
            unsigned long :28;
        } BIT;
    } IRQCR3;
    union {
        unsigned long LONG;
        struct {
            unsigned long :2;
            unsigned long IRQMD:2;
            unsigned long :28;
        } BIT;
    } IRQCR4;
    union {
        unsigned long LONG;
        struct {
            unsigned long :2;
            unsigned long IRQMD:2;
            unsigned long :28;
        } BIT;
    } IRQCR5;
    union {
        unsigned long LONG;
        struct {
            unsigned long :2;
            unsigned long IRQMD:2;
            unsigned long :28;
        } BIT;
    } IRQCR6;
    union {
        unsigned long LONG;
        struct {
            unsigned long :2;
            unsigned long IRQMD:2;
            unsigned long :28;
        } BIT;
    } IRQCR7;
    union {
        unsigned long LONG;
        struct {
            unsigned long :2;
            unsigned long IRQMD:2;
            unsigned long :28;
        } BIT;
    } IRQCR8;
    union {
        unsigned long LONG;
        struct {
            unsigned long :2;
            unsigned long IRQMD:2;
            unsigned long :28;
        } BIT;
    } IRQCR9;
    union {
        unsigned long LONG;
        struct {
            unsigned long :2;
            unsigned long IRQMD:2;
            unsigned long :28;
        } BIT;
    } IRQCR10;
    union {
        unsigned long LONG;
        struct {
            unsigned long :2;
            unsigned long IRQMD:2;
            unsigned long :28;
        } BIT;
    } IRQCR11;
    union {
        unsigned long LONG;
        struct {
            unsigned long :2;
            unsigned long IRQMD:2;
            unsigned long :28;
        } BIT;
    } IRQCR12;
    union {
        unsigned long LONG;
        struct {
            unsigned long :2;
            unsigned long IRQMD:2;
            unsigned long :28;
        } BIT;
    } IRQCR13;
    union {
        unsigned long LONG;
        struct {
            unsigned long :2;
            unsigned long IRQMD:2;
            unsigned long :28;
        } BIT;
    } IRQCR14;
    union {
        unsigned long LONG;
        struct {
            unsigned long :2;
            unsigned long IRQMD:2;
            unsigned long :28;
        } BIT;
    } IRQCR15;
    union {
        unsigned long LONG;
        struct {
            unsigned long FLTEN0:1;
            unsigned long FLTEN1:1;
            unsigned long FLTEN2:1;
            unsigned long FLTEN3:1;
            unsigned long FLTEN4:1;
            unsigned long FLTEN5:1;
            unsigned long FLTEN6:1;
            unsigned long FLTEN7:1;
            unsigned long FLTEN8:1;
            unsigned long FLTEN9:1;
            unsigned long FLTEN10:1;
            unsigned long FLTEN11:1;
            unsigned long FLTEN12:1;
            unsigned long FLTEN13:1;
            unsigned long FLTEN14:1;
            unsigned long FLTEN15:1;
            unsigned long :16;
        } BIT;
    } IRQFLTE;
    union {
        unsigned long LONG;
        struct {
            unsigned long FCLKSEL0:2;
            unsigned long FCLKSEL1:2;
            unsigned long FCLKSEL2:2;
            unsigned long FCLKSEL3:2;
            unsigned long FCLKSEL4:2;
            unsigned long FCLKSEL5:2;
            unsigned long FCLKSEL6:2;
            unsigned long FCLKSEL7:2;
            unsigned long FCLKSEL8:2;
            unsigned long FCLKSEL9:2;
            unsigned long FCLKSEL10:2;
            unsigned long FCLKSEL11:2;
            unsigned long FCLKSEL12:2;
            unsigned long FCLKSEL13:2;
            unsigned long FCLKSEL14:2;
            unsigned long FCLKSEL15:2;
        } BIT;
    } IRQFLTC;
    union {
        unsigned long LONG;
        struct {
            unsigned long NMIST:1;
            unsigned long ECMST:1;
            unsigned long :30;
        } BIT;
    } NMISR;
    union {
        unsigned long LONG;
        struct {
            unsigned long NMICLR:1;
            unsigned long ECMCLR:1;
            unsigned long :30;
        } BIT;
    } NMICLR;
    union {
        unsigned long LONG;
        struct {
            unsigned long :3;
            unsigned long NMIMD:1;
            unsigned long :28;
        } BIT;
    } NMICR;
    union {
        unsigned long LONG;
        struct {
            unsigned long NFLTEN:1;
            unsigned long :31;
        } BIT;
    } NMIFLTE;
    union {
        unsigned long LONG;
        struct {
            unsigned long NFCLKSEL:2;
            unsigned long :30;
        } BIT;
    } NMIFLTC;
    union {
        unsigned long LONG;
        struct {
            unsigned long :2;
            unsigned long EPHYMD:2;
            unsigned long :28;
        } BIT;
    } EPHYCR0;
    union {
        unsigned long LONG;
        struct {
            unsigned long :2;
            unsigned long EPHYMD:2;
            unsigned long :28;
        } BIT;
    } EPHYCR1;
    union {
        unsigned long LONG;
        struct {
            unsigned long :2;
            unsigned long EPHYMD:2;
            unsigned long :28;
        } BIT;
    } EPHYCR2;
    union {
        unsigned long LONG;
        struct {
            unsigned long EFLTEN0:1;
            unsigned long EFLTEN1:1;
            unsigned long EFLTEN2:1;
            unsigned long :29;
        } BIT;
    } EPHYFLTE;
    union {
        unsigned long LONG;
        struct {
            unsigned long EFCLKSEL0:2;
            unsigned long EFCLKSEL1:2;
            unsigned long EFCLKSEL2:2;
            unsigned long :26;
        } BIT;
    } EPHYFLTC;
    union {
        unsigned long LONG;
        struct {
            unsigned long DFLTEN0:1;
            unsigned long DFLTEN1:1;
            unsigned long DFLTEN2:1;
            unsigned long :29;
        } BIT;
    } DREQFLTE;
    union {
        unsigned long LONG;
        struct {
            unsigned long DFCLKSEL0:2;
            unsigned long DFCLKSEL1:2;
            unsigned long DFCLKSEL2:2;
            unsigned long :26;
        } BIT;
    } DREQFLTC;
    char           wk0[24];
    union {
        unsigned long LONG;
        struct {
            unsigned long CM3INT:1;
            unsigned long :15;
            unsigned long CR4INT:1;
            unsigned long :15;
        } BIT;
    } CPUINT;
};

struct st_iwdt {
    union {
        unsigned char BYTE;
        struct {
            unsigned char REFRESH:8;
        } BIT;
    } IWDTRR;
    char           wk0[1];
    union {
        unsigned short WORD;
        struct {
            unsigned short TOPS:2;
            unsigned short :2;
            unsigned short CKS:4;
            unsigned short RPES:2;
            unsigned short :2;
            unsigned short RPSS:2;
            unsigned short :2;
        } BIT;
    } IWDTCR;
    union {
        unsigned short WORD;
        struct {
            unsigned short CNTVAL:14;
            unsigned short UNDFF:1;
            unsigned short REFEF:1;
        } BIT;
    } IWDTSR;
    union {
        unsigned char BYTE;
        struct {
            unsigned char :7;
            unsigned char RSTIRQS:1;
        } BIT;
    } IWDTRCR;
};

struct st_mpc {
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :2;
        } BIT;
    } P00PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :2;
        } BIT;
    } P01PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :2;
        } BIT;
    } P02PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :2;
        } BIT;
    } P03PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :2;
        } BIT;
    } P04PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :2;
        } BIT;
    } P05PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :2;
        } BIT;
    } P06PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :2;
        } BIT;
    } P07PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P10PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P11PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P12PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P13PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P14PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P15PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P16PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P17PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P20PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P21PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P22PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P23PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P24PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P25PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P26PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P27PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P30PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P31PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P32PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P33PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P34PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P35PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P36PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P37PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P40PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P41PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P42PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P43PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P44PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P45PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P46PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P47PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P50PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P51PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P52PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P53PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P54PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P55PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P56PFS;
    char           wk0[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P60PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P61PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P62PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P63PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P64PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P65PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P66PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P67PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P70PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P71PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P72PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P73PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P74PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P75PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P76PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } P77PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char ASEL:1;
        } BIT;
    } P80PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char ASEL:1;
        } BIT;
    } P81PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char ASEL:1;
        } BIT;
    } P82PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char ASEL:1;
        } BIT;
    } P83PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char ASEL:1;
        } BIT;
    } P84PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char ASEL:1;
        } BIT;
    } P85PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char ASEL:1;
        } BIT;
    } P86PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char ASEL:1;
        } BIT;
    } P87PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char ASEL:1;
        } BIT;
    } P90PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char ASEL:1;
        } BIT;
    } P91PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char ASEL:1;
        } BIT;
    } P92PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char ASEL:1;
        } BIT;
    } P93PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char ASEL:1;
        } BIT;
    } P94PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char ASEL:1;
        } BIT;
    } P95PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char ASEL:1;
        } BIT;
    } P96PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char ASEL:1;
        } BIT;
    } P97PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PA0PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PA1PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PA2PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PA3PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PA4PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PA5PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PA6PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PA7PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PB0PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PB1PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PB2PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PB3PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PB4PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PB5PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PB6PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PB7PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PC0PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PC1PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PC2PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PC3PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PC4PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PC5PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PC6PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PC7PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :1;
            unsigned char ASEL:1;
        } BIT;
    } PD0PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :1;
            unsigned char ASEL:1;
        } BIT;
    } PD1PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :1;
            unsigned char ASEL:1;
        } BIT;
    } PD2PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :1;
            unsigned char ASEL:1;
        } BIT;
    } PD3PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :1;
            unsigned char ASEL:1;
        } BIT;
    } PD4PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :1;
            unsigned char ASEL:1;
        } BIT;
    } PD5PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :1;
            unsigned char ASEL:1;
        } BIT;
    } PD6PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :1;
            unsigned char ASEL:1;
        } BIT;
    } PD7PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PE0PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PE1PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PE2PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PE3PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PE4PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PE5PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PE6PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PE7PFS;
    char           wk1[5];
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PF5PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PF6PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PF7PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :2;
        } BIT;
    } PG0PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :2;
        } BIT;
    } PG1PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :2;
        } BIT;
    } PG2PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :2;
        } BIT;
    } PG3PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :2;
        } BIT;
    } PG4PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :2;
        } BIT;
    } PG5PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :2;
        } BIT;
    } PG6PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :2;
        } BIT;
    } PG7PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PH0PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PH1PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PH2PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PH3PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PH4PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PH5PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PH6PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PH7PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PJ0PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PJ1PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PJ2PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PJ3PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PJ4PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PJ5PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PJ6PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PJ7PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :2;
        } BIT;
    } PK0PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :2;
        } BIT;
    } PK1PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :2;
        } BIT;
    } PK2PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :2;
        } BIT;
    } PK3PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :2;
        } BIT;
    } PK4PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :2;
        } BIT;
    } PK5PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :2;
        } BIT;
    } PK6PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :2;
        } BIT;
    } PK7PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PL0PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PL1PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PL2PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PL3PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PL4PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PL5PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PL6PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PL7PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PM0PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PM1PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PM2PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PM3PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PM4PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PM5PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PM6PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PM7PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PN0PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PN1PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PN2PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PN3PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PN4PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PN5PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PN6PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PN7PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :2;
        } BIT;
    } PP0PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :2;
        } BIT;
    } PP1PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :2;
        } BIT;
    } PP2PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :2;
        } BIT;
    } PP3PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :2;
        } BIT;
    } PP4PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :2;
        } BIT;
    } PP5PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :2;
        } BIT;
    } PP6PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char :2;
        } BIT;
    } PP7PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PR0PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PR1PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PR2PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PR3PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PR4PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PR5PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PR6PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PR7PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PS0PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PS1PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PS2PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PS3PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PS4PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PS5PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PS6PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PS7PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PT0PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PT1PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PT2PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PT3PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PT4PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PT5PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PT6PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PT7PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PU0PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PU1PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PU2PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PU3PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PU4PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PU5PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PU6PFS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PSEL:6;
            unsigned char ISEL:1;
            unsigned char :1;
        } BIT;
    } PU7PFS;
    char           wk2[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char :6;
            unsigned char PFSWE:1;
            unsigned char B0WI:1;
        } BIT;
    } PWPR;
};

struct st_mtu {
    union {
        unsigned char BYTE;
        struct {
            unsigned char OE3B:1;
            unsigned char OE4A:1;
            unsigned char OE4B:1;
            unsigned char OE3D:1;
            unsigned char OE4C:1;
            unsigned char OE4D:1;
            unsigned char :2;
        } BIT;
    } TOERA;
    char           wk0[2];
    union {
        unsigned char BYTE;
        struct {
            unsigned char UF:1;
            unsigned char VF:1;
            unsigned char WF:1;
            unsigned char FB:1;
            unsigned char P:1;
            unsigned char N:1;
            unsigned char BDC:1;
            unsigned char :1;
        } BIT;
    } TGCRA;
    union {
        unsigned char BYTE;
        struct {
            unsigned char OLSP:1;
            unsigned char OLSN:1;
            unsigned char TOCS:1;
            unsigned char TOCL:1;
            unsigned char :2;
            unsigned char PSYE:1;
            unsigned char :1;
        } BIT;
    } TOCR1A;
    union {
        unsigned char BYTE;
        struct {
            unsigned char OLS1P:1;
            unsigned char OLS1N:1;
            unsigned char OLS2P:1;
            unsigned char OLS2N:1;
            unsigned char OLS3P:1;
            unsigned char OLS3N:1;
            unsigned char BF:2;
        } BIT;
    } TOCR2A;
    char           wk1[4];
    unsigned short TCDRA;
    unsigned short TDDRA;
    char           wk2[8];
    unsigned short TCNTSA;
    unsigned short TCBRA;
    char           wk3[12];
    union {
        unsigned char BYTE;
        struct {
            unsigned char T4VCOR:3;
            unsigned char T4VEN:1;
            unsigned char T3ACOR:3;
            unsigned char T3AEN:1;
        } BIT;
    } TITCR1A;
    union {
        unsigned char BYTE;
        struct {
            unsigned char T4VCNT:3;
            unsigned char :1;
            unsigned char T3ACNT:3;
            unsigned char :1;
        } BIT;
    } TITCNT1A;
    union {
        unsigned char BYTE;
        struct {
            unsigned char BTE:2;
            unsigned char :6;
        } BIT;
    } TBTERA;
    char           wk4[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TDER:1;
            unsigned char :7;
        } BIT;
    } TDERA;
    char           wk5[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char OLS1P:1;
            unsigned char OLS1N:1;
            unsigned char OLS2P:1;
            unsigned char OLS2N:1;
            unsigned char OLS3P:1;
            unsigned char OLS3N:1;
            unsigned char :2;
        } BIT;
    } TOLBRA;
    char           wk6[3];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TITM:1;
            unsigned char :7;
        } BIT;
    } TITMRA;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TRG4COR:3;
            unsigned char :5;
        } BIT;
    } TITCR2A;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TRG4CNT:3;
            unsigned char :5;
        } BIT;
    } TITCNT2A;
    char           wk7[35];
    union {
        unsigned char BYTE;
        struct {
            unsigned char WRE:1;
            unsigned char SCC:1;
            unsigned char :5;
            unsigned char CCE:1;
        } BIT;
    } TWCRA;
    char           wk8[15];
    union {
        unsigned char BYTE;
        struct {
            unsigned char DRS:1;
            unsigned char :7;
        } BIT;
    } TMDR2A;
    char           wk9[15];
    union {
        unsigned char BYTE;
        struct {
            unsigned char CST0:1;
            unsigned char CST1:1;
            unsigned char CST2:1;
            unsigned char CST8:1;
            unsigned char :2;
            unsigned char CST3:1;
            unsigned char CST4:1;
        } BIT;
    } TSTRA;
    union {
        unsigned char BYTE;
        struct {
            unsigned char SYNC0:1;
            unsigned char SYNC1:1;
            unsigned char SYNC2:1;
            unsigned char :3;
            unsigned char SYNC3:1;
            unsigned char SYNC4:1;
        } BIT;
    } TSYRA;
    union {
        unsigned char BYTE;
        struct {
            unsigned char SCH7:1;
            unsigned char SCH6:1;
            unsigned char :1;
            unsigned char SCH4:1;
            unsigned char SCH3:1;
            unsigned char SCH2:1;
            unsigned char SCH1:1;
            unsigned char SCH0:1;
        } BIT;
    } TCSYSTR;
    char           wk10[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char RWE:1;
            unsigned char :7;
        } BIT;
    } TRWERA;
    char           wk11[1925];
    union {
        unsigned char BYTE;
        struct {
            unsigned char OE6B:1;
            unsigned char OE7A:1;
            unsigned char OE7B:1;
            unsigned char OE6D:1;
            unsigned char OE7C:1;
            unsigned char OE7D:1;
            unsigned char :2;
        } BIT;
    } TOERB;
    char           wk12[3];
    union {
        unsigned char BYTE;
        struct {
            unsigned char OLSP:1;
            unsigned char OLSN:1;
            unsigned char TOCS:1;
            unsigned char TOCL:1;
            unsigned char :2;
            unsigned char PSYE:1;
            unsigned char :1;
        } BIT;
    } TOCR1B;
    union {
        unsigned char BYTE;
        struct {
            unsigned char OLS1P:1;
            unsigned char OLS1N:1;
            unsigned char OLS2P:1;
            unsigned char OLS2N:1;
            unsigned char OLS3P:1;
            unsigned char OLS3N:1;
            unsigned char BF:2;
        } BIT;
    } TOCR2B;
    char           wk13[4];
    unsigned short TCDRB;
    unsigned short TDDRB;
    char           wk14[8];
    unsigned short TCNTSB;
    unsigned short TCBRB;
    char           wk15[12];
    union {
        unsigned char BYTE;
        struct {
            unsigned char T7VCOR:3;
            unsigned char T7VEN:1;
            unsigned char T6ACOR:3;
            unsigned char T6AEN:1;
        } BIT;
    } TITCR1B;
    union {
        unsigned char BYTE;
        struct {
            unsigned char T7VCNT:3;
            unsigned char :1;
            unsigned char T6ACNT:3;
            unsigned char :1;
        } BIT;
    } TITCNT1B;
    union {
        unsigned char BYTE;
        struct {
            unsigned char BTE:2;
            unsigned char :6;
        } BIT;
    } TBTERB;
    char           wk16[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TDER:1;
            unsigned char :7;
        } BIT;
    } TDERB;
    char           wk17[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char OLS1P:1;
            unsigned char OLS1N:1;
            unsigned char OLS2P:1;
            unsigned char OLS2N:1;
            unsigned char OLS3P:1;
            unsigned char OLS3N:1;
            unsigned char :2;
        } BIT;
    } TOLBRB;
    char           wk18[3];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TITM:1;
            unsigned char :7;
        } BIT;
    } TITMRB;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TRG7COR:3;
            unsigned char :5;
        } BIT;
    } TITCR2B;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TRG7CNT:3;
            unsigned char :5;
        } BIT;
    } TITCNT2B;
    char           wk19[35];
    union {
        unsigned char BYTE;
        struct {
            unsigned char WRE:1;
            unsigned char SCC:1;
            unsigned char :5;
            unsigned char CCE:1;
        } BIT;
    } TWCRB;
    char           wk20[15];
    union {
        unsigned char BYTE;
        struct {
            unsigned char DRS:1;
            unsigned char :7;
        } BIT;
    } TMDR2B;
    char           wk21[15];
    union {
        unsigned char BYTE;
        struct {
            unsigned char :6;
            unsigned char CST6:1;
            unsigned char CST7:1;
        } BIT;
    } TSTRB;
    union {
        unsigned char BYTE;
        struct {
            unsigned char :6;
            unsigned char SYNC6:1;
            unsigned char SYNC7:1;
        } BIT;
    } TSYRB;
    char           wk22[2];
    union {
        unsigned char BYTE;
        struct {
            unsigned char RWE:1;
            unsigned char :7;
        } BIT;
    } TRWERB;
};

struct st_mtu0 {
    union {
        unsigned char BYTE;
        struct {
            unsigned char NFAEN:1;
            unsigned char NFBEN:1;
            unsigned char NFCEN:1;
            unsigned char NFDEN:1;
            unsigned char NFCS:2;
            unsigned char :2;
        } BIT;
    } NFCR0;
    char           wk0[8];
    union {
        unsigned char BYTE;
        struct {
            unsigned char NFAEN:1;
            unsigned char NFBEN:1;
            unsigned char NFCEN:1;
            unsigned char NFDEN:1;
            unsigned char NFCSC:2;
            unsigned char :2;
        } BIT;
    } NFCRC;
    char           wk1[102];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TPSC:3;
            unsigned char CKEG:2;
            unsigned char CCLR:3;
        } BIT;
    } TCR;
    union {
        unsigned char BYTE;
        struct {
            unsigned char MD:4;
            unsigned char BFA:1;
            unsigned char BFB:1;
            unsigned char BFE:1;
            unsigned char :1;
        } BIT;
    } TMDR1;
    union {
        unsigned char BYTE;
        struct {
            unsigned char IOA:4;
            unsigned char IOB:4;
        } BIT;
    } TIORH;
    union {
        unsigned char BYTE;
        struct {
            unsigned char IOC:4;
            unsigned char IOD:4;
        } BIT;
    } TIORL;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TGIEA:1;
            unsigned char TGIEB:1;
            unsigned char TGIEC:1;
            unsigned char TGIED:1;
            unsigned char TCIEV:1;
            unsigned char :2;
            unsigned char TTGE:1;
        } BIT;
    } TIER;
    char           wk2[1];
    unsigned short TCNT;
    unsigned short TGRA;
    unsigned short TGRB;
    unsigned short TGRC;
    unsigned short TGRD;
    char           wk3[16];
    unsigned short TGRE;
    unsigned short TGRF;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TGIEE:1;
            unsigned char TGIEF:1;
            unsigned char :5;
            unsigned char TTGE2:1;
        } BIT;
    } TIER2;
    char           wk4[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TTSA:1;
            unsigned char TTSB:1;
            unsigned char TTSE:1;
            unsigned char :5;
        } BIT;
    } TBTM;
    char           wk5[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TPSC2:3;
            unsigned char :5;
        } BIT;
    } TCR2;
};

struct st_mtu1 {
    char           wk0[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char NFAEN:1;
            unsigned char NFBEN:1;
            unsigned char NFCEN:1;
            unsigned char NFDEN:1;
            unsigned char NFCS:2;
            unsigned char :2;
        } BIT;
    } NFCR1;
    char           wk1[238];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TPSC:3;
            unsigned char CKEG:2;
            unsigned char CCLR:3;
        } BIT;
    } TCR;
    union {
        unsigned char BYTE;
        struct {
            unsigned char MD:4;
            unsigned char :4;
        } BIT;
    } TMDR1;
    union {
        unsigned char BYTE;
        struct {
            unsigned char IOA:4;
            unsigned char IOB:4;
        } BIT;
    } TIOR;
    char           wk2[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TGIEA:1;
            unsigned char TGIEB:1;
            unsigned char :2;
            unsigned char TCIEV:1;
            unsigned char TCIEU:1;
            unsigned char :1;
            unsigned char TTGE:1;
        } BIT;
    } TIER;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TGFA:1;
            unsigned char TGFB:1;
            unsigned char :2;
            unsigned char TCFV:1;
            unsigned char TCFU:1;
            unsigned char :1;
            unsigned char TCFD:1;
        } BIT;
    } TSR;
    unsigned short TCNT;
    unsigned short TGRA;
    unsigned short TGRB;
    char           wk3[4];
    union {
        unsigned char BYTE;
        struct {
            unsigned char I1AE:1;
            unsigned char I1BE:1;
            unsigned char I2AE:1;
            unsigned char I2BE:1;
            unsigned char :4;
        } BIT;
    } TICCR;
    union {
        unsigned char BYTE;
        struct {
            unsigned char LWA:1;
            unsigned char PHCKSEL:1;
            unsigned char :6;
        } BIT;
    } TMDR3;
    char           wk4[2];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TPSC2:3;
            unsigned char PCB:2;
            unsigned char :3;
        } BIT;
    } TCR2;
    char           wk5[11];
    unsigned long  TCNTLW;
    unsigned long  TGRALW;
    unsigned long  TGRBLW;
};

struct st_mtu2 {
    union {
        unsigned char BYTE;
        struct {
            unsigned char NFAEN:1;
            unsigned char NFBEN:1;
            unsigned char NFCEN:1;
            unsigned char NFDEN:1;
            unsigned char NFCS:2;
            unsigned char :2;
        } BIT;
    } NFCR2;
    char           wk0[365];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TPSC:3;
            unsigned char CKEG:2;
            unsigned char CCLR:3;
        } BIT;
    } TCR;
    union {
        unsigned char BYTE;
        struct {
            unsigned char MD:4;
            unsigned char :4;
        } BIT;
    } TMDR1;
    union {
        unsigned char BYTE;
        struct {
            unsigned char IOA:4;
            unsigned char IOB:4;
        } BIT;
    } TIOR;
    char           wk1[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TGIEA:1;
            unsigned char TGIEB:1;
            unsigned char :2;
            unsigned char TCIEV:1;
            unsigned char TCIEU:1;
            unsigned char :1;
            unsigned char TTGE:1;
        } BIT;
    } TIER;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TGFA:1;
            unsigned char TGFB:1;
            unsigned char :2;
            unsigned char TCFV:1;
            unsigned char TCFU:1;
            unsigned char :1;
            unsigned char TCFD:1;
        } BIT;
    } TSR;
    unsigned short TCNT;
    unsigned short TGRA;
    unsigned short TGRB;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TPSC2:3;
            unsigned char PCB:2;
            unsigned char :3;
        } BIT;
    } TCR2;
};

struct st_mtu3 {
    union {
        unsigned char BYTE;
        struct {
            unsigned char TPSC:3;
            unsigned char CKEG:2;
            unsigned char CCLR:3;
        } BIT;
    } TCR;
    char           wk0[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char MD:4;
            unsigned char BFA:1;
            unsigned char BFB:1;
            unsigned char :2;
        } BIT;
    } TMDR1;
    char           wk1[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char IOA:4;
            unsigned char IOB:4;
        } BIT;
    } TIORH;
    union {
        unsigned char BYTE;
        struct {
            unsigned char IOC:4;
            unsigned char IOD:4;
        } BIT;
    } TIORL;
    char           wk2[2];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TGIEA:1;
            unsigned char TGIEB:1;
            unsigned char TGIEC:1;
            unsigned char TGIED:1;
            unsigned char TCIEV:1;
            unsigned char :2;
            unsigned char TTGE:1;
        } BIT;
    } TIER;
    char           wk3[7];
    unsigned short TCNT;
    char           wk4[6];
    unsigned short TGRA;
    unsigned short TGRB;
    char           wk5[8];
    unsigned short TGRC;
    unsigned short TGRD;
    char           wk6[4];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TGFA:1;
            unsigned char TGFB:1;
            unsigned char TGFC:1;
            unsigned char TGFD:1;
            unsigned char TCFV:1;
            unsigned char :2;
            unsigned char TCFD:1;
        } BIT;
    } TSR;
    char           wk7[11];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TTSA:1;
            unsigned char TTSB:1;
            unsigned char :6;
        } BIT;
    } TBTM;
    char           wk8[19];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TPSC2:3;
            unsigned char :5;
        } BIT;
    } TCR2;
    char           wk9[37];
    unsigned short TGRE;
    char           wk10[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char NFAEN:1;
            unsigned char NFBEN:1;
            unsigned char NFCEN:1;
            unsigned char NFDEN:1;
            unsigned char NFCS:2;
            unsigned char :2;
        } BIT;
    } NFCR3;
};

struct st_mtu4 {
    char           wk0[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TPSC:3;
            unsigned char CKEG:2;
            unsigned char CCLR:3;
        } BIT;
    } TCR;
    char           wk1[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char MD:4;
            unsigned char BFA:1;
            unsigned char BFB:1;
            unsigned char :2;
        } BIT;
    } TMDR1;
    char           wk2[2];
    union {
        unsigned char BYTE;
        struct {
            unsigned char IOA:4;
            unsigned char IOB:4;
        } BIT;
    } TIORH;
    union {
        unsigned char BYTE;
        struct {
            unsigned char IOC:4;
            unsigned char IOD:4;
        } BIT;
    } TIORL;
    char           wk3[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TGIEA:1;
            unsigned char TGIEB:1;
            unsigned char TGIEC:1;
            unsigned char TGIED:1;
            unsigned char TCIEV:1;
            unsigned char :1;
            unsigned char TTGE2:1;
            unsigned char TTGE:1;
        } BIT;
    } TIER;
    char           wk4[8];
    unsigned short TCNT;
    char           wk5[8];
    unsigned short TGRA;
    unsigned short TGRB;
    char           wk6[8];
    unsigned short TGRC;
    unsigned short TGRD;
    char           wk7[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TGFA:1;
            unsigned char TGFB:1;
            unsigned char TGFC:1;
            unsigned char TGFD:1;
            unsigned char TCFV:1;
            unsigned char :2;
            unsigned char TCFD:1;
        } BIT;
    } TSR;
    char           wk8[11];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TTSA:1;
            unsigned char TTSB:1;
            unsigned char :6;
        } BIT;
    } TBTM;
    char           wk9[6];
    union {
        unsigned short WORD;
        struct {
            unsigned short ITB4VE:1;
            unsigned short ITB3AE:1;
            unsigned short ITA4VE:1;
            unsigned short ITA3AE:1;
            unsigned short DT4BE:1;
            unsigned short UT4BE:1;
            unsigned short DT4AE:1;
            unsigned short UT4AE:1;
            unsigned short :6;
            unsigned short BF:2;
        } BIT;
    } TADCR;
    char           wk10[2];
    unsigned short TADCORA;
    unsigned short TADCORB;
    unsigned short TADCOBRA;
    unsigned short TADCOBRB;
    char           wk11[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TPSC2:3;
            unsigned char :5;
        } BIT;
    } TCR2;
    char           wk12[38];
    unsigned short TGRE;
    unsigned short TGRF;
    char           wk13[28];
    union {
        unsigned char BYTE;
        struct {
            unsigned char NFAEN:1;
            unsigned char NFBEN:1;
            unsigned char NFCEN:1;
            unsigned char NFDEN:1;
            unsigned char NFCS:2;
            unsigned char :2;
        } BIT;
    } NFCR4;
};

struct st_mtu5 {
    char           wk0[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char NFUEN:1;
            unsigned char NFVEN:1;
            unsigned char NFWEN:1;
            unsigned char :1;
            unsigned char NFCS:2;
            unsigned char :2;
        } BIT;
    } NFCR5;
    char           wk1[490];
    unsigned short TCNTU;
    unsigned short TGRU;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TPSC:2;
            unsigned char :6;
        } BIT;
    } TCRU;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TPSC2:3;
            unsigned char CKEG:2;
            unsigned char :3;
        } BIT;
    } TCR2U;
    union {
        unsigned char BYTE;
        struct {
            unsigned char IOC:5;
            unsigned char :3;
        } BIT;
    } TIORU;
    char           wk2[9];
    unsigned short TCNTV;
    unsigned short TGRV;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TPSC:2;
            unsigned char :6;
        } BIT;
    } TCRV;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TPSC2:3;
            unsigned char CKEG:2;
            unsigned char :3;
        } BIT;
    } TCR2V;
    union {
        unsigned char BYTE;
        struct {
            unsigned char IOC:5;
            unsigned char :3;
        } BIT;
    } TIORV;
    char           wk3[9];
    unsigned short TCNTW;
    unsigned short TGRW;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TPSC:2;
            unsigned char :6;
        } BIT;
    } TCRW;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TPSC2:3;
            unsigned char CKEG:2;
            unsigned char :3;
        } BIT;
    } TCR2W;
    union {
        unsigned char BYTE;
        struct {
            unsigned char IOC:5;
            unsigned char :3;
        } BIT;
    } TIORW;
    char           wk4[11];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TGIE5W:1;
            unsigned char TGIE5V:1;
            unsigned char TGIE5U:1;
            unsigned char :5;
        } BIT;
    } TIER;
    char           wk5[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char CSTW5:1;
            unsigned char CSTV5:1;
            unsigned char CSTU5:1;
            unsigned char :5;
        } BIT;
    } TSTR;
    char           wk6[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char CMPCLR5W:1;
            unsigned char CMPCLR5V:1;
            unsigned char CMPCLR5U:1;
            unsigned char :5;
        } BIT;
    } TCNTCMPCLR;
};

struct st_mtu6 {
    union {
        unsigned char BYTE;
        struct {
            unsigned char TPSC:3;
            unsigned char CKEG:2;
            unsigned char CCLR:3;
        } BIT;
    } TCR;
    char           wk0[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char MD:4;
            unsigned char BFA:1;
            unsigned char BFB:1;
            unsigned char :2;
        } BIT;
    } TMDR1;
    char           wk1[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char IOA:4;
            unsigned char IOB:4;
        } BIT;
    } TIORH;
    union {
        unsigned char BYTE;
        struct {
            unsigned char IOC:4;
            unsigned char IOD:4;
        } BIT;
    } TIORL;
    char           wk2[2];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TGIEA:1;
            unsigned char TGIEB:1;
            unsigned char TGIEC:1;
            unsigned char TGIED:1;
            unsigned char TCIEV:1;
            unsigned char :2;
            unsigned char TTGE:1;
        } BIT;
    } TIER;
    char           wk3[7];
    unsigned short TCNT;
    char           wk4[6];
    unsigned short TGRA;
    unsigned short TGRB;
    char           wk5[8];
    unsigned short TGRC;
    unsigned short TGRD;
    char           wk6[4];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TGFA:1;
            unsigned char TGFB:1;
            unsigned char TGFC:1;
            unsigned char TGFD:1;
            unsigned char TCFV:1;
            unsigned char :2;
            unsigned char TCFD:1;
        } BIT;
    } TSR;
    char           wk7[11];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TTSA:1;
            unsigned char TTSB:1;
            unsigned char :6;
        } BIT;
    } TBTM;
    char           wk8[19];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TPSC2:3;
            unsigned char :5;
        } BIT;
    } TCR2;
    char           wk9[3];
    union {
        unsigned char BYTE;
        struct {
            unsigned char CE2B:1;
            unsigned char CE2A:1;
            unsigned char CE1B:1;
            unsigned char CE1A:1;
            unsigned char CE0D:1;
            unsigned char CE0C:1;
            unsigned char CE0B:1;
            unsigned char CE0A:1;
        } BIT;
    } TSYCR;
    char           wk10[33];
    unsigned short TGRE;
    char           wk11[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char NFAEN:1;
            unsigned char NFBEN:1;
            unsigned char NFCEN:1;
            unsigned char NFDEN:1;
            unsigned char NFCS:2;
            unsigned char :2;
        } BIT;
    } NFCR6;
};

struct st_mtu7 {
    char           wk0[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TPSC:3;
            unsigned char CKEG:2;
            unsigned char CCLR:3;
        } BIT;
    } TCR;
    char           wk1[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char MD:4;
            unsigned char BFA:1;
            unsigned char BFB:1;
            unsigned char :2;
        } BIT;
    } TMDR1;
    char           wk2[2];
    union {
        unsigned char BYTE;
        struct {
            unsigned char IOA:4;
            unsigned char IOB:4;
        } BIT;
    } TIORH;
    union {
        unsigned char BYTE;
        struct {
            unsigned char IOC:4;
            unsigned char IOD:4;
        } BIT;
    } TIORL;
    char           wk3[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TGIEA:1;
            unsigned char TGIEB:1;
            unsigned char TGIEC:1;
            unsigned char TGIED:1;
            unsigned char TCIEV:1;
            unsigned char :1;
            unsigned char TTGE2:1;
            unsigned char TTGE:1;
        } BIT;
    } TIER;
    char           wk4[8];
    unsigned short TCNT;
    char           wk5[8];
    unsigned short TGRA;
    unsigned short TGRB;
    char           wk6[8];
    unsigned short TGRC;
    unsigned short TGRD;
    char           wk7[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TGFA:1;
            unsigned char TGFB:1;
            unsigned char TGFC:1;
            unsigned char TGFD:1;
            unsigned char TCFV:1;
            unsigned char :2;
            unsigned char TCFD:1;
        } BIT;
    } TSR;
    char           wk8[11];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TTSA:1;
            unsigned char TTSB:1;
            unsigned char :6;
        } BIT;
    } TBTM;
    char           wk9[6];
    union {
        unsigned short WORD;
        struct {
            unsigned short ITB7VE:1;
            unsigned short ITB6AE:1;
            unsigned short ITA7VE:1;
            unsigned short ITA6AE:1;
            unsigned short DT7BE:1;
            unsigned short UT7BE:1;
            unsigned short DT7AE:1;
            unsigned short UT7AE:1;
            unsigned short :6;
            unsigned short BF:2;
        } BIT;
    } TADCR;
    char           wk10[2];
    unsigned short TADCORA;
    unsigned short TADCORB;
    unsigned short TADCOBRA;
    unsigned short TADCOBRB;
    char           wk11[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TPSC2:3;
            unsigned char :5;
        } BIT;
    } TCR2;
    char           wk12[38];
    unsigned short TGRE;
    unsigned short TGRF;
    char           wk13[28];
    union {
        unsigned char BYTE;
        struct {
            unsigned char NFAEN:1;
            unsigned char NFBEN:1;
            unsigned char NFCEN:1;
            unsigned char NFDEN:1;
            unsigned char NFCS:2;
            unsigned char :2;
        } BIT;
    } NFCR7;
};

struct st_mtu8 {
    union {
        unsigned char BYTE;
        struct {
            unsigned char NFAEN:1;
            unsigned char NFBEN:1;
            unsigned char NFCEN:1;
            unsigned char NFDEN:1;
            unsigned char NFCS:2;
            unsigned char :2;
        } BIT;
    } NFCR8;
    char           wk0[871];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TPSC:3;
            unsigned char CKEG:2;
            unsigned char CCLR:3;
        } BIT;
    } TCR;
    union {
        unsigned char BYTE;
        struct {
            unsigned char MD:4;
            unsigned char BFA:1;
            unsigned char BFB:1;
            unsigned char :2;
        } BIT;
    } TMDR1;
    union {
        unsigned char BYTE;
        struct {
            unsigned char IOA:4;
            unsigned char IOB:4;
        } BIT;
    } TIORH;
    union {
        unsigned char BYTE;
        struct {
            unsigned char IOC:4;
            unsigned char IOD:4;
        } BIT;
    } TIORL;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TGIEA:1;
            unsigned char TGIEB:1;
            unsigned char TGIEC:1;
            unsigned char TGIED:1;
            unsigned char TCIEV:1;
            unsigned char :3;
        } BIT;
    } TIER;
    char           wk1[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TPSC2:3;
            unsigned char :5;
        } BIT;
    } TCR2;
    char           wk2[1];
    unsigned long  TCNT;
    unsigned long  TGRA;
    unsigned long  TGRB;
    unsigned long  TGRC;
    unsigned long  TGRD;
};

struct st_poe {
    union {
        unsigned short WORD;
        struct {
            unsigned short POE0M:2;
            unsigned short :6;
            unsigned short PIE1:1;
            unsigned short :3;
            unsigned short POE0F:1;
            unsigned short :3;
        } BIT;
    } ICSR1;
    union {
        unsigned short WORD;
        struct {
            unsigned short :8;
            unsigned short OIE1:1;
            unsigned short OCE1:1;
            unsigned short :5;
            unsigned short OSF1:1;
        } BIT;
    } OCSR1;
    union {
        unsigned short WORD;
        struct {
            unsigned short POE4M:2;
            unsigned short :6;
            unsigned short PIE2:1;
            unsigned short :3;
            unsigned short POE4F:1;
            unsigned short :3;
        } BIT;
    } ICSR2;
    union {
        unsigned short WORD;
        struct {
            unsigned short :8;
            unsigned short OIE2:1;
            unsigned short OCE2:1;
            unsigned short :5;
            unsigned short OSF2:1;
        } BIT;
    } OCSR2;
    union {
        unsigned short WORD;
        struct {
            unsigned short POE8M:2;
            unsigned short :6;
            unsigned short PIE3:1;
            unsigned short POE8E:1;
            unsigned short :2;
            unsigned short POE8F:1;
            unsigned short :3;
        } BIT;
    } ICSR3;
    union {
        unsigned char BYTE;
        struct {
            unsigned char MTUCH34HIZ:1;
            unsigned char MTUCH67HIZ:1;
            unsigned char MTUCH0HIZ:1;
            unsigned char :1;
            unsigned char GPT3HIZ:1;
            unsigned char :3;
        } BIT;
    } SPOER;
    union {
        unsigned char BYTE;
        struct {
            unsigned char MTU0AZE:1;
            unsigned char MTU0BZE:1;
            unsigned char MTU0CZE:1;
            unsigned char MTU0DZE:1;
            unsigned char :4;
        } BIT;
    } POECR1;
    union {
        unsigned short WORD;
        struct {
            unsigned short MTU7BDZE:1;
            unsigned short MTU7ACZE:1;
            unsigned short MTU6BDZE:1;
            unsigned short :5;
            unsigned short MTU4BDZE:1;
            unsigned short MTU4ACZE:1;
            unsigned short MTU3BDZE:1;
            unsigned short :5;
        } BIT;
    } POECR2;
    union {
        unsigned short WORD;
        struct {
            unsigned short :9;
            unsigned short GPT3ABZE:1;
            unsigned short :6;
        } BIT;
    } POECR3;
    union {
        unsigned short WORD;
        struct {
            unsigned short :2;
            unsigned short IC2ADDMT34ZE:1;
            unsigned short IC3ADDMT34ZE:1;
            unsigned short IC4ADDMT34ZE:1;
            unsigned short IC5ADDMT34ZE:1;
            unsigned short :3;
            unsigned short IC1ADDMT67ZE:1;
            unsigned short :1;
            unsigned short IC3ADDMT67ZE:1;
            unsigned short IC4ADDMT67ZE:1;
            unsigned short IC5ADDMT67ZE:1;
            unsigned short :2;
        } BIT;
    } POECR4;
    union {
        unsigned short WORD;
        struct {
            unsigned short :1;
            unsigned short IC1ADDMT0ZE:1;
            unsigned short IC2ADDMT0ZE:1;
            unsigned short :1;
            unsigned short IC4ADDMT0ZE:1;
            unsigned short IC5ADDMT0ZE:1;
            unsigned short :10;
        } BIT;
    } POECR5;
    union {
        unsigned short WORD;
        struct {
            unsigned short :9;
            unsigned short IC1ADDGPT3ZE:1;
            unsigned short IC2ADDGPT3ZE:1;
            unsigned short IC3ADDGPT3ZE:1;
            unsigned short IC4ADDGPT3ZE:1;
            unsigned short :3;
        } BIT;
    } POECR6;
    union {
        unsigned short WORD;
        struct {
            unsigned short POE10M:2;
            unsigned short :6;
            unsigned short PIE4:1;
            unsigned short POE10E:1;
            unsigned short :2;
            unsigned short POE10F:1;
            unsigned short :3;
        } BIT;
    } ICSR4;
    union {
        unsigned short WORD;
        struct {
            unsigned short POE10M:2;
            unsigned short :6;
            unsigned short PIE5:1;
            unsigned short POE10E:1;
            unsigned short :2;
            unsigned short POE10F:1;
            unsigned short :3;
        } BIT;
    } ICSR5;
    union {
        unsigned short WORD;
        struct {
            unsigned short OLSG0A:1;
            unsigned short OLSG0B:1;
            unsigned short OLSG1A:1;
            unsigned short OLSG1B:1;
            unsigned short OLSG2A:1;
            unsigned short OLSG2B:1;
            unsigned short :1;
            unsigned short OLSEN:1;
            unsigned short :8;
        } BIT;
    } ALR1;
    union {
        unsigned short WORD;
        struct {
            unsigned short :9;
            unsigned short OSTSTE:1;
            unsigned short :2;
            unsigned short OSTSTF:1;
            unsigned short :3;
        } BIT;
    } ICSR6;
    char           wk0[5];
    union {
        unsigned char BYTE;
        struct {
            unsigned char G3ASEL:4;
            unsigned char G3BSEL:4;
        } BIT;
    } G3SELR;
    union {
        unsigned char BYTE;
        struct {
            unsigned char M0ASEL:4;
            unsigned char M0BSEL:4;
        } BIT;
    } M0SELR1;
    union {
        unsigned char BYTE;
        struct {
            unsigned char M0CSEL:4;
            unsigned char M0DSEL:4;
        } BIT;
    } M0SELR2;
    union {
        unsigned char BYTE;
        struct {
            unsigned char M3BSEL:4;
            unsigned char M3DSEL:4;
        } BIT;
    } M3SELR;
    union {
        unsigned char BYTE;
        struct {
            unsigned char M4ASEL:4;
            unsigned char M4CSEL:4;
        } BIT;
    } M4SELR1;
    union {
        unsigned char BYTE;
        struct {
            unsigned char M4BSEL:4;
            unsigned char M4DSEL:4;
        } BIT;
    } M4SELR2;
};

struct st_port0 {
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PDR;
    char           wk0[62];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PODR;
    char           wk1[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PIDR;
    char           wk2[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PMR;
    char           wk3[127];
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PCR;
};

struct st_port1 {
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PDR;
    char           wk0[61];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PODR;
    char           wk1[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PIDR;
    char           wk2[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PMR;
    char           wk3[128];
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PCR;
    char           wk4[62];
    union {
        unsigned short WORD;
        struct {
            unsigned char H;
            unsigned char L;
        } BYTE;
        struct {
            unsigned char B0:1;
            unsigned char :8;
            unsigned char :7;
        } BIT;
    } DSCR;
};

struct st_port2 {
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PDR;
    char           wk0[60];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PODR;
    char           wk1[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PIDR;
    char           wk2[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PMR;
    char           wk3[129];
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PCR;
};

struct st_port3 {
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PDR;
    char           wk0[59];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PODR;
    char           wk1[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PIDR;
    char           wk2[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PMR;
    char           wk3[130];
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PCR;
};

struct st_port4 {
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PDR;
    char           wk0[58];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PODR;
    char           wk1[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PIDR;
    char           wk2[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PMR;
    char           wk3[131];
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PCR;
};

struct st_port5 {
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PDR;
    char           wk0[57];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PODR;
    char           wk1[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PIDR;
    char           wk2[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PMR;
    char           wk3[132];
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PCR;
};

struct st_port6 {
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PDR;
    char           wk0[56];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PODR;
    char           wk1[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PIDR;
    char           wk2[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PMR;
    char           wk3[133];
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PCR;
};

struct st_port7 {
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PDR;
    char           wk0[55];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PODR;
    char           wk1[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PIDR;
    char           wk2[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PMR;
    char           wk3[134];
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PCR;
};

struct st_port8 {
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PDR;
    char           wk0[54];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PODR;
    char           wk1[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PIDR;
    char           wk2[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PMR;
    char           wk3[135];
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PCR;
};

struct st_port9 {
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PDR;
    char           wk0[53];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PODR;
    char           wk1[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PIDR;
    char           wk2[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PMR;
    char           wk3[136];
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PCR;
};

struct st_porta {
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PDR;
    char           wk0[52];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PODR;
    char           wk1[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PIDR;
    char           wk2[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PMR;
    char           wk3[137];
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PCR;
};

struct st_portb {
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PDR;
    char           wk0[51];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PODR;
    char           wk1[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PIDR;
    char           wk2[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PMR;
    char           wk3[138];
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PCR;
};

struct st_portc {
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PDR;
    char           wk0[50];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PODR;
    char           wk1[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PIDR;
    char           wk2[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PMR;
};

struct st_portd {
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PDR;
    char           wk0[49];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PODR;
    char           wk1[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PIDR;
    char           wk2[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PMR;
    char           wk3[140];
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PCR;
};

struct st_porte {
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PDR;
    char           wk0[48];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PODR;
    char           wk1[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PIDR;
    char           wk2[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PMR;
    char           wk3[141];
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PCR;
};

struct st_portf {
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PDR;
    char           wk0[47];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PODR;
    char           wk1[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PIDR;
    char           wk2[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PMR;
    char           wk3[142];
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PCR;
};

struct st_portg {
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PDR;
    char           wk0[46];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PODR;
    char           wk1[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PIDR;
    char           wk2[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PMR;
    char           wk3[143];
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PCR;
};

struct st_porth {
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PDR;
    char           wk0[45];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PODR;
    char           wk1[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PIDR;
    char           wk2[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PMR;
    char           wk3[144];
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PCR;
};

struct st_portj {
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PDR;
    char           wk0[44];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PODR;
    char           wk1[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PIDR;
    char           wk2[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PMR;
    char           wk3[145];
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PCR;
};

struct st_portk {
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PDR;
    char           wk0[43];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PODR;
    char           wk1[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PIDR;
    char           wk2[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PMR;
    char           wk3[146];
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PCR;
};

struct st_portl {
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PDR;
    char           wk0[42];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PODR;
    char           wk1[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PIDR;
    char           wk2[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PMR;
    char           wk3[147];
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PCR;
};

struct st_portm {
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PDR;
    char           wk0[41];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PODR;
    char           wk1[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PIDR;
    char           wk2[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PMR;
    char           wk3[148];
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PCR;
};

struct st_portn {
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PDR;
    char           wk0[40];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PODR;
    char           wk1[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PIDR;
    char           wk2[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PMR;
    char           wk3[149];
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PCR;
};

struct st_portp {
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PDR;
    char           wk0[39];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PODR;
    char           wk1[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PIDR;
    char           wk2[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PMR;
    char           wk3[150];
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PCR;
};

struct st_portr {
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PDR;
    char           wk0[38];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PODR;
    char           wk1[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PIDR;
    char           wk2[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PMR;
    char           wk3[151];
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PCR;
};

struct st_ports {
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PDR;
    char           wk0[37];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PODR;
    char           wk1[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PIDR;
    char           wk2[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PMR;
    char           wk3[152];
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PCR;
};

struct st_portt {
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PDR;
    char           wk0[36];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PODR;
    char           wk1[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PIDR;
    char           wk2[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PMR;
    char           wk3[153];
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PCR;
};

struct st_portu {
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PDR;
    char           wk0[35];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PODR;
    char           wk1[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PIDR;
    char           wk2[31];
    union {
        unsigned char BYTE;
        struct {
            unsigned char B0:1;
            unsigned char B1:1;
            unsigned char B2:1;
            unsigned char B3:1;
            unsigned char B4:1;
            unsigned char B5:1;
            unsigned char B6:1;
            unsigned char B7:1;
        } BIT;
    } PMR;
    char           wk3[154];
    union {
        unsigned short WORD;
        struct {
            unsigned short B0:2;
            unsigned short B1:2;
            unsigned short B2:2;
            unsigned short B3:2;
            unsigned short B4:2;
            unsigned short B5:2;
            unsigned short B6:2;
            unsigned short B7:2;
        } BIT;
    } PCR;
};

struct st_ppg0 {
    union {
        unsigned char BYTE;
        struct {
            unsigned char G0CMS:2;
            unsigned char G1CMS:2;
            unsigned char G2CMS:2;
            unsigned char G3CMS:2;
        } BIT;
    } PCR;
    union {
        unsigned char BYTE;
        struct {
            unsigned char G0NOV:1;
            unsigned char G1NOV:1;
            unsigned char G2NOV:1;
            unsigned char G3NOV:1;
            unsigned char G0INV:1;
            unsigned char G1INV:1;
            unsigned char G2INV:1;
            unsigned char G3INV:1;
        } BIT;
    } PMR;
    union {
        unsigned char BYTE;
        struct {
            unsigned char NDER8:1;
            unsigned char NDER9:1;
            unsigned char NDER10:1;
            unsigned char NDER11:1;
            unsigned char NDER12:1;
            unsigned char NDER13:1;
            unsigned char NDER14:1;
            unsigned char NDER15:1;
        } BIT;
    } NDERH;
    union {
        unsigned char BYTE;
        struct {
            unsigned char NDER0:1;
            unsigned char NDER1:1;
            unsigned char NDER2:1;
            unsigned char NDER3:1;
            unsigned char NDER4:1;
            unsigned char NDER5:1;
            unsigned char NDER6:1;
            unsigned char NDER7:1;
        } BIT;
    } NDERL;
    union {
        unsigned char BYTE;
        struct {
            unsigned char POD8:1;
            unsigned char POD9:1;
            unsigned char POD10:1;
            unsigned char POD11:1;
            unsigned char POD12:1;
            unsigned char POD13:1;
            unsigned char POD14:1;
            unsigned char POD15:1;
        } BIT;
    } PODRH;
    union {
        unsigned char BYTE;
        struct {
            unsigned char POD0:1;
            unsigned char POD1:1;
            unsigned char POD2:1;
            unsigned char POD3:1;
            unsigned char POD4:1;
            unsigned char POD5:1;
            unsigned char POD6:1;
            unsigned char POD7:1;
        } BIT;
    } PODRL;
    union {
        unsigned char BYTE;
        struct {
            unsigned char NDR8:1;
            unsigned char NDR9:1;
            unsigned char NDR10:1;
            unsigned char NDR11:1;
            unsigned char NDR12:1;
            unsigned char NDR13:1;
            unsigned char NDR14:1;
            unsigned char NDR15:1;
        } BIT;
    } NDRH;
    union {
        unsigned char BYTE;
        struct {
            unsigned char NDR0:1;
            unsigned char NDR1:1;
            unsigned char NDR2:1;
            unsigned char NDR3:1;
            unsigned char NDR4:1;
            unsigned char NDR5:1;
            unsigned char NDR6:1;
            unsigned char NDR7:1;
        } BIT;
    } NDRL;
    union {
        unsigned char BYTE;
        struct {
            unsigned char NDR8:1;
            unsigned char NDR9:1;
            unsigned char NDR10:1;
            unsigned char NDR11:1;
            unsigned char NDR12:1;
            unsigned char NDR13:1;
            unsigned char NDR14:1;
            unsigned char NDR15:1;
        } BIT;
    } NDRH2;
    union {
        unsigned char BYTE;
        struct {
            unsigned char NDR0:1;
            unsigned char NDR1:1;
            unsigned char NDR2:1;
            unsigned char NDR3:1;
            unsigned char NDR4:1;
            unsigned char NDR5:1;
            unsigned char NDR6:1;
            unsigned char NDR7:1;
        } BIT;
    } NDRL2;
};

struct st_ppg1 {
    union {
        unsigned char BYTE;
        struct {
            unsigned char G0CMS:2;
            unsigned char G1CMS:2;
            unsigned char G2CMS:2;
            unsigned char G3CMS:2;
        } BIT;
    } PCR;
    union {
        unsigned char BYTE;
        struct {
            unsigned char G0NOV:1;
            unsigned char G1NOV:1;
            unsigned char G2NOV:1;
            unsigned char G3NOV:1;
            unsigned char G0INV:1;
            unsigned char G1INV:1;
            unsigned char G2INV:1;
            unsigned char G3INV:1;
        } BIT;
    } PMR;
    union {
        unsigned char BYTE;
        struct {
            unsigned char NDER8:1;
            unsigned char NDER9:1;
            unsigned char NDER10:1;
            unsigned char NDER11:1;
            unsigned char NDER12:1;
            unsigned char NDER13:1;
            unsigned char NDER14:1;
            unsigned char NDER15:1;
        } BIT;
    } NDERH;
    union {
        unsigned char BYTE;
        struct {
            unsigned char NDER0:1;
            unsigned char NDER1:1;
            unsigned char NDER2:1;
            unsigned char NDER3:1;
            unsigned char NDER4:1;
            unsigned char NDER5:1;
            unsigned char NDER6:1;
            unsigned char NDER7:1;
        } BIT;
    } NDERL;
    union {
        unsigned char BYTE;
        struct {
            unsigned char POD8:1;
            unsigned char POD9:1;
            unsigned char POD10:1;
            unsigned char POD11:1;
            unsigned char POD12:1;
            unsigned char POD13:1;
            unsigned char POD14:1;
            unsigned char POD15:1;
        } BIT;
    } PODRH;
    union {
        unsigned char BYTE;
        struct {
            unsigned char POD0:1;
            unsigned char POD1:1;
            unsigned char POD2:1;
            unsigned char POD3:1;
            unsigned char POD4:1;
            unsigned char POD5:1;
            unsigned char POD6:1;
            unsigned char POD7:1;
        } BIT;
    } PODRL;
    union {
        unsigned char BYTE;
        struct {
            unsigned char NDR8:1;
            unsigned char NDR9:1;
            unsigned char NDR10:1;
            unsigned char NDR11:1;
            unsigned char NDR12:1;
            unsigned char NDR13:1;
            unsigned char NDR14:1;
            unsigned char NDR15:1;
        } BIT;
    } NDRH;
    union {
        unsigned char BYTE;
        struct {
            unsigned char NDR0:1;
            unsigned char NDR1:1;
            unsigned char NDR2:1;
            unsigned char NDR3:1;
            unsigned char NDR4:1;
            unsigned char NDR5:1;
            unsigned char NDR6:1;
            unsigned char NDR7:1;
        } BIT;
    } NDRL;
    union {
        unsigned char BYTE;
        struct {
            unsigned char NDR8:1;
            unsigned char NDR9:1;
            unsigned char NDR10:1;
            unsigned char NDR11:1;
            unsigned char NDR12:1;
            unsigned char NDR13:1;
            unsigned char NDR14:1;
            unsigned char NDR15:1;
        } BIT;
    } NDRH2;
    union {
        unsigned char BYTE;
        struct {
            unsigned char NDR0:1;
            unsigned char NDR1:1;
            unsigned char NDR2:1;
            unsigned char NDR3:1;
            unsigned char NDR4:1;
            unsigned char NDR5:1;
            unsigned char NDR6:1;
            unsigned char NDR7:1;
        } BIT;
    } NDRL2;
    union {
        unsigned char BYTE;
        struct {
            unsigned char PTRSL:1;
            unsigned char :7;
        } BIT;
    } PTRSLR;
};

struct st_riic {
    union {
        unsigned char BYTE;
        struct {
            unsigned char SDAI:1;
            unsigned char SCLI:1;
            unsigned char SDAO:1;
            unsigned char SCLO:1;
            unsigned char SOWP:1;
            unsigned char CLO:1;
            unsigned char IICRST:1;
            unsigned char ICE:1;
        } BIT;
    } ICCR1;
    union {
        unsigned char BYTE;
        struct {
            unsigned char :1;
            unsigned char ST:1;
            unsigned char RS:1;
            unsigned char SP:1;
            unsigned char :1;
            unsigned char TRS:1;
            unsigned char MST:1;
            unsigned char BBSY:1;
        } BIT;
    } ICCR2;
    union {
        unsigned char BYTE;
        struct {
            unsigned char BC:3;
            unsigned char BCWP:1;
            unsigned char CKS:3;
            unsigned char MTWP:1;
        } BIT;
    } ICMR1;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMOS:1;
            unsigned char TMOL:1;
            unsigned char TMOH:1;
            unsigned char :1;
            unsigned char SDDL:3;
            unsigned char DLCS:1;
        } BIT;
    } ICMR2;
    union {
        unsigned char BYTE;
        struct {
            unsigned char NF:2;
            unsigned char ACKBR:1;
            unsigned char ACKBT:1;
            unsigned char ACKWP:1;
            unsigned char RDRFS:1;
            unsigned char WAIT:1;
            unsigned char :1;
        } BIT;
    } ICMR3;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMOE:1;
            unsigned char MALE:1;
            unsigned char NALE:1;
            unsigned char SALE:1;
            unsigned char NACKE:1;
            unsigned char NFE:1;
            unsigned char SCLE:1;
            unsigned char :1;
        } BIT;
    } ICFER;
    union {
        unsigned char BYTE;
        struct {
            unsigned char SAR0E:1;
            unsigned char SAR1E:1;
            unsigned char SAR2E:1;
            unsigned char GCAE:1;
            unsigned char :1;
            unsigned char DIDE:1;
            unsigned char :2;
        } BIT;
    } ICSER;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMOIE:1;
            unsigned char ALIE:1;
            unsigned char STIE:1;
            unsigned char SPIE:1;
            unsigned char NAKIE:1;
            unsigned char RIE:1;
            unsigned char TEIE:1;
            unsigned char TIE:1;
        } BIT;
    } ICIER;
    union {
        unsigned char BYTE;
        struct {
            unsigned char AAS0:1;
            unsigned char AAS1:1;
            unsigned char AAS2:1;
            unsigned char GCA:1;
            unsigned char :1;
            unsigned char DID:1;
            unsigned char :2;
        } BIT;
    } ICSR1;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMOF:1;
            unsigned char AL:1;
            unsigned char START:1;
            unsigned char STOP:1;
            unsigned char NACKF:1;
            unsigned char RDRF:1;
            unsigned char TEND:1;
            unsigned char TDRE:1;
        } BIT;
    } ICSR2;
    union {
        unsigned char BYTE;
        struct {
            unsigned char SVA0:1;
            unsigned char SVA:7;
        } BIT;
    } ICSARL0;
    union {
        unsigned char BYTE;
        struct {
            unsigned char FS:1;
            unsigned char SVA:2;
            unsigned char :5;
        } BIT;
    } ICSARU0;
    union {
        unsigned char BYTE;
        struct {
            unsigned char SVA0:1;
            unsigned char SVA:7;
        } BIT;
    } ICSARL1;
    union {
        unsigned char BYTE;
        struct {
            unsigned char FS:1;
            unsigned char SVA:2;
            unsigned char :5;
        } BIT;
    } ICSARU1;
    union {
        unsigned char BYTE;
        struct {
            unsigned char SVA0:1;
            unsigned char SVA:7;
        } BIT;
    } ICSARL2;
    union {
        unsigned char BYTE;
        struct {
            unsigned char FS:1;
            unsigned char SVA:2;
            unsigned char :5;
        } BIT;
    } ICSARU2;
    union {
        unsigned char BYTE;
        struct {
            unsigned char BRL:5;
            unsigned char :3;
        } BIT;
    } ICBRL;
    union {
        unsigned char BYTE;
        struct {
            unsigned char BRH:5;
            unsigned char :3;
        } BIT;
    } ICBRH;
    unsigned char  ICDRT;
    unsigned char  ICDRR;
};

struct st_rscan {
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long BRP:10;
            unsigned long :6;
            unsigned long TSEG1:4;
            unsigned long TSEG2:3;
            unsigned long :1;
            unsigned long SJW:2;
            unsigned long :6;
        } BIT;
    } RSCAN0C0CFG;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char CHMDC:2;
            unsigned char CSLPR:1;
            unsigned char RTBO:1;
            unsigned char :4;
            unsigned char BEIE:1;
            unsigned char EWIE:1;
            unsigned char EPIE:1;
            unsigned char BOEIE:1;
            unsigned char BORIE:1;
            unsigned char OLIE:1;
            unsigned char BLIE:1;
            unsigned char ALIE:1;
            unsigned char TAIE:1;
            unsigned char :4;
            unsigned char BOM:2;
            unsigned char ERRD:1;
            unsigned char CTME:1;
            unsigned char CTMS:2;
            unsigned char :5;
        } BIT;
    } RSCAN0C0CTR;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char CRSTSTS:1;
            unsigned char CHLTSTS:1;
            unsigned char CSLPSTS:1;
            unsigned char EPSTS:1;
            unsigned char BOSTS:1;
            unsigned char TRMSTS:1;
            unsigned char RECSTS:1;
            unsigned char COMSTS:1;
            unsigned char :8;
            unsigned char REC:8;
            unsigned char TEC:8;
        } BIT;
    } RSCAN0C0STS;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long BEF:1;
            unsigned long EWF:1;
            unsigned long EPF:1;
            unsigned long BOEF:1;
            unsigned long BORF:1;
            unsigned long OVLF:1;
            unsigned long BLF:1;
            unsigned long ALF:1;
            unsigned long SERR:1;
            unsigned long FERR:1;
            unsigned long AERR:1;
            unsigned long CERR:1;
            unsigned long B1ERR:1;
            unsigned long B0ERR:1;
            unsigned long ADERR:1;
            unsigned long :1;
            unsigned long CRCREG:15;
            unsigned long :1;
        } BIT;
    } RSCAN0C0ERFL;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long BRP:10;
            unsigned long :6;
            unsigned long TSEG1:4;
            unsigned long TSEG2:3;
            unsigned long :1;
            unsigned long SJW:2;
            unsigned long :6;
        } BIT;
    } RSCAN0C1CFG;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char CHMDC:2;
            unsigned char CSLPR:1;
            unsigned char RTBO:1;
            unsigned char :4;
            unsigned char BEIE:1;
            unsigned char EWIE:1;
            unsigned char EPIE:1;
            unsigned char BOEIE:1;
            unsigned char BORIE:1;
            unsigned char OLIE:1;
            unsigned char BLIE:1;
            unsigned char ALIE:1;
            unsigned char TAIE:1;
            unsigned char :4;
            unsigned char BOM:2;
            unsigned char ERRD:1;
            unsigned char CTME:1;
            unsigned char CTMS:2;
            unsigned char :5;
        } BIT;
    } RSCAN0C1CTR;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char CRSTSTS:1;
            unsigned char CHLTSTS:1;
            unsigned char CSLPSTS:1;
            unsigned char EPSTS:1;
            unsigned char BOSTS:1;
            unsigned char TRMSTS:1;
            unsigned char RECSTS:1;
            unsigned char COMSTS:1;
            unsigned char :8;
            unsigned char REC:8;
            unsigned char TEC:8;
        } BIT;
    } RSCAN0C1STS;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long BEF:1;
            unsigned long EWF:1;
            unsigned long EPF:1;
            unsigned long BOEF:1;
            unsigned long BORF:1;
            unsigned long OVLF:1;
            unsigned long BLF:1;
            unsigned long ALF:1;
            unsigned long SERR:1;
            unsigned long FERR:1;
            unsigned long AERR:1;
            unsigned long CERR:1;
            unsigned long B1ERR:1;
            unsigned long B0ERR:1;
            unsigned long ADERR:1;
            unsigned long :1;
            unsigned long CRCREG:15;
            unsigned long :1;
        } BIT;
    } RSCAN0C1ERFL;
    char           wk0[100];
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long TPRI:1;
            unsigned long DCE:1;
            unsigned long DRE:1;
            unsigned long MME:1;
            unsigned long DCS:1;
            unsigned long :3;
            unsigned long TSP:4;
            unsigned long TSSS:1;
            unsigned long TSBTCS:3;
            unsigned long ITRCP:16;
        } BIT;
    } RSCAN0GCFG;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char GMDC:2;
            unsigned char GSLPR:1;
            unsigned char :5;
            unsigned char DEIE:1;
            unsigned char MEIE:1;
            unsigned char THLEIE:1;
            unsigned char :5;
            unsigned char TSRST:1;
            unsigned char :7;
            unsigned char :8;
        } BIT;
    } RSCAN0GCTR;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char GRSTSTS:1;
            unsigned char GHLTSTS:1;
            unsigned char GSLPSTS:1;
            unsigned char GRAMINIT:1;
            unsigned char :4;
            unsigned char :8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0GSTS;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char DEF:1;
            unsigned char MES:1;
            unsigned char THLES:1;
            unsigned char :5;
            unsigned char :8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0GERFL;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned long TS:16;
            unsigned long :16;
        } BIT;
    } RSCAN0GTSC;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char AFLPN:5;
            unsigned char :3;
            unsigned char AFLDAE:1;
            unsigned char :7;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0GAFLECTR;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char :8;
            unsigned char :8;
            unsigned char RNC1:8;
            unsigned char RNC0:8;
        } BIT;
    } RSCAN0GAFLCFG0;
    char           wk1[4];
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char NRXMB:8;
            unsigned char :8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0RMNB;
    union {
        unsigned long LONG;
        struct {
            unsigned short RMNSq_l;
            unsigned short RMNSq_h;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
    } RSCAN0RMND0;
    char           wk2[12];
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char RFE:1;
            unsigned char RFIE:1;
            unsigned char :6;
            unsigned char RFDC:3;
            unsigned char :1;
            unsigned char RFIM:1;
            unsigned char RFIGCV:3;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0RFCC0;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char RFE:1;
            unsigned char RFIE:1;
            unsigned char :6;
            unsigned char RFDC:3;
            unsigned char :1;
            unsigned char RFIM:1;
            unsigned char RFIGCV:3;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0RFCC1;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char RFE:1;
            unsigned char RFIE:1;
            unsigned char :6;
            unsigned char RFDC:3;
            unsigned char :1;
            unsigned char RFIM:1;
            unsigned char RFIGCV:3;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0RFCC2;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char RFE:1;
            unsigned char RFIE:1;
            unsigned char :6;
            unsigned char RFDC:3;
            unsigned char :1;
            unsigned char RFIM:1;
            unsigned char RFIGCV:3;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0RFCC3;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char RFE:1;
            unsigned char RFIE:1;
            unsigned char :6;
            unsigned char RFDC:3;
            unsigned char :1;
            unsigned char RFIM:1;
            unsigned char RFIGCV:3;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0RFCC4;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char RFE:1;
            unsigned char RFIE:1;
            unsigned char :6;
            unsigned char RFDC:3;
            unsigned char :1;
            unsigned char RFIM:1;
            unsigned char RFIGCV:3;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0RFCC5;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char RFE:1;
            unsigned char RFIE:1;
            unsigned char :6;
            unsigned char RFDC:3;
            unsigned char :1;
            unsigned char RFIM:1;
            unsigned char RFIGCV:3;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0RFCC6;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char RFE:1;
            unsigned char RFIE:1;
            unsigned char :6;
            unsigned char RFDC:3;
            unsigned char :1;
            unsigned char RFIM:1;
            unsigned char RFIGCV:3;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0RFCC7;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char RFEMP:1;
            unsigned char RFFLL:1;
            unsigned char RFMLT:1;
            unsigned char RFIF:1;
            unsigned char :4;
            unsigned char RFMC:8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0RFSTS0;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char RFEMP:1;
            unsigned char RFFLL:1;
            unsigned char RFMLT:1;
            unsigned char RFIF:1;
            unsigned char :4;
            unsigned char RFMC:8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0RFSTS1;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char RFEMP:1;
            unsigned char RFFLL:1;
            unsigned char RFMLT:1;
            unsigned char RFIF:1;
            unsigned char :4;
            unsigned char RFMC:8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0RFSTS2;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char RFEMP:1;
            unsigned char RFFLL:1;
            unsigned char RFMLT:1;
            unsigned char RFIF:1;
            unsigned char :4;
            unsigned char RFMC:8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0RFSTS3;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char RFEMP:1;
            unsigned char RFFLL:1;
            unsigned char RFMLT:1;
            unsigned char RFIF:1;
            unsigned char :4;
            unsigned char RFMC:8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0RFSTS4;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char RFEMP:1;
            unsigned char RFFLL:1;
            unsigned char RFMLT:1;
            unsigned char RFIF:1;
            unsigned char :4;
            unsigned char RFMC:8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0RFSTS5;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char RFEMP:1;
            unsigned char RFFLL:1;
            unsigned char RFMLT:1;
            unsigned char RFIF:1;
            unsigned char :4;
            unsigned char RFMC:8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0RFSTS6;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char RFEMP:1;
            unsigned char RFFLL:1;
            unsigned char RFMLT:1;
            unsigned char RFIF:1;
            unsigned char :4;
            unsigned char RFMC:8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0RFSTS7;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char RFPC:8;
            unsigned char :8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0RFPCTR0;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char RFPC:8;
            unsigned char :8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0RFPCTR1;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char RFPC:8;
            unsigned char :8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0RFPCTR2;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char RFPC:8;
            unsigned char :8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0RFPCTR3;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char RFPC:8;
            unsigned char :8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0RFPCTR4;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char RFPC:8;
            unsigned char :8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0RFPCTR5;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char RFPC:8;
            unsigned char :8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0RFPCTR6;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char RFPC:8;
            unsigned char :8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0RFPCTR7;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char CFE:1;
            unsigned char CFRXIE:1;
            unsigned char CFTXIE:1;
            unsigned char :5;
            unsigned char CFDC:3;
            unsigned char :1;
            unsigned char CFIM:1;
            unsigned char CFIGCV:3;
            unsigned char CFM:2;
            unsigned char CFITSS:1;
            unsigned char CFITR:1;
            unsigned char CFTML:4;
            unsigned char CFITT:8;
        } BIT;
    } RSCAN0CFCC0;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char CFE:1;
            unsigned char CFRXIE:1;
            unsigned char CFTXIE:1;
            unsigned char :5;
            unsigned char CFDC:3;
            unsigned char :1;
            unsigned char CFIM:1;
            unsigned char CFIGCV:3;
            unsigned char CFM:2;
            unsigned char CFITSS:1;
            unsigned char CFITR:1;
            unsigned char CFTML:4;
            unsigned char CFITT:8;
        } BIT;
    } RSCAN0CFCC1;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char CFE:1;
            unsigned char CFRXIE:1;
            unsigned char CFTXIE:1;
            unsigned char :5;
            unsigned char CFDC:3;
            unsigned char :1;
            unsigned char CFIM:1;
            unsigned char CFIGCV:3;
            unsigned char CFM:2;
            unsigned char CFITSS:1;
            unsigned char CFITR:1;
            unsigned char CFTML:4;
            unsigned char CFITT:8;
        } BIT;
    } RSCAN0CFCC2;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char CFE:1;
            unsigned char CFRXIE:1;
            unsigned char CFTXIE:1;
            unsigned char :5;
            unsigned char CFDC:3;
            unsigned char :1;
            unsigned char CFIM:1;
            unsigned char CFIGCV:3;
            unsigned char CFM:2;
            unsigned char CFITSS:1;
            unsigned char CFITR:1;
            unsigned char CFTML:4;
            unsigned char CFITT:8;
        } BIT;
    } RSCAN0CFCC3;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char CFE:1;
            unsigned char CFRXIE:1;
            unsigned char CFTXIE:1;
            unsigned char :5;
            unsigned char CFDC:3;
            unsigned char :1;
            unsigned char CFIM:1;
            unsigned char CFIGCV:3;
            unsigned char CFM:2;
            unsigned char CFITSS:1;
            unsigned char CFITR:1;
            unsigned char CFTML:4;
            unsigned char CFITT:8;
        } BIT;
    } RSCAN0CFCC4;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char CFE:1;
            unsigned char CFRXIE:1;
            unsigned char CFTXIE:1;
            unsigned char :5;
            unsigned char CFDC:3;
            unsigned char :1;
            unsigned char CFIM:1;
            unsigned char CFIGCV:3;
            unsigned char CFM:2;
            unsigned char CFITSS:1;
            unsigned char CFITR:1;
            unsigned char CFTML:4;
            unsigned char CFITT:8;
        } BIT;
    } RSCAN0CFCC5;
    char           wk3[72];
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char CFEMP:1;
            unsigned char CFFLL:1;
            unsigned char CFMLT:1;
            unsigned char CFRXIF:1;
            unsigned char CFTXIF:1;
            unsigned char :3;
            unsigned char CFMC:8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0CFSTS0;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char CFEMP:1;
            unsigned char CFFLL:1;
            unsigned char CFMLT:1;
            unsigned char CFRXIF:1;
            unsigned char CFTXIF:1;
            unsigned char :3;
            unsigned char CFMC:8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0CFSTS1;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char CFEMP:1;
            unsigned char CFFLL:1;
            unsigned char CFMLT:1;
            unsigned char CFRXIF:1;
            unsigned char CFTXIF:1;
            unsigned char :3;
            unsigned char CFMC:8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0CFSTS2;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char CFEMP:1;
            unsigned char CFFLL:1;
            unsigned char CFMLT:1;
            unsigned char CFRXIF:1;
            unsigned char CFTXIF:1;
            unsigned char :3;
            unsigned char CFMC:8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0CFSTS3;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char CFEMP:1;
            unsigned char CFFLL:1;
            unsigned char CFMLT:1;
            unsigned char CFRXIF:1;
            unsigned char CFTXIF:1;
            unsigned char :3;
            unsigned char CFMC:8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0CFSTS4;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char CFEMP:1;
            unsigned char CFFLL:1;
            unsigned char CFMLT:1;
            unsigned char CFRXIF:1;
            unsigned char CFTXIF:1;
            unsigned char :3;
            unsigned char CFMC:8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0CFSTS5;
    char           wk4[72];
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char CFPC:8;
            unsigned char :8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0CFPCTR0;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char CFPC:8;
            unsigned char :8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0CFPCTR1;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char CFPC:8;
            unsigned char :8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0CFPCTR2;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char CFPC:8;
            unsigned char :8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0CFPCTR3;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char CFPC:8;
            unsigned char :8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0CFPCTR4;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char CFPC:8;
            unsigned char :8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0CFPCTR5;
    char           wk5[72];
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char RF0EMP:1;
            unsigned char RF1EMP:1;
            unsigned char RF2EMP:1;
            unsigned char RF3EMP:1;
            unsigned char RF4EMP:1;
            unsigned char RF5EMP:1;
            unsigned char RF6EMP:1;
            unsigned char RF7EMP:1;
            unsigned char CF0EMP:1;
            unsigned char CF1EMP:1;
            unsigned char CF2EMP:1;
            unsigned char CF3EMP:1;
            unsigned char CF4EMP:1;
            unsigned char CF5EMP:1;
            unsigned char :2;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0FESTS;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char RF0FLL:1;
            unsigned char RF1FLL:1;
            unsigned char RF2FLL:1;
            unsigned char RF3FLL:1;
            unsigned char RF4FLL:1;
            unsigned char RF5FLL:1;
            unsigned char RF6FLL:1;
            unsigned char RF7FLL:1;
            unsigned char CF0FLL:1;
            unsigned char CF1FLL:1;
            unsigned char CF2FLL:1;
            unsigned char CF3FLL:1;
            unsigned char CF4FLL:1;
            unsigned char CF5FLL:1;
            unsigned char :2;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0FFSTS;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char RF0MLT:1;
            unsigned char RF1MLT:1;
            unsigned char RF2MLT:1;
            unsigned char RF3MLT:1;
            unsigned char RF4MLT:1;
            unsigned char RF5MLT:1;
            unsigned char RF6MLT:1;
            unsigned char RF7MLT:1;
            unsigned char CF0MLT:1;
            unsigned char CF1MLT:1;
            unsigned char CF2MLT:1;
            unsigned char CF3MLT:1;
            unsigned char CF4MLT:1;
            unsigned char CF5MLT:1;
            unsigned char :2;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0FMSTS;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char RF0IF:1;
            unsigned char RF1IF:1;
            unsigned char RF2IF:1;
            unsigned char RF3IF:1;
            unsigned char RF4IF:1;
            unsigned char RF5IF:1;
            unsigned char RF6IF:1;
            unsigned char RF7IF:1;
            unsigned char :8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0RFISTS;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char CF0RXIF:1;
            unsigned char CF1RXIF:1;
            unsigned char CF2RXIF:1;
            unsigned char CF3RXIF:1;
            unsigned char CF4RXIF:1;
            unsigned char CF5RXIF:1;
            unsigned char :2;
            unsigned char :8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0CFRISTS;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char CF0TXIF:1;
            unsigned char CF1TXIF:1;
            unsigned char CF2TXIF:1;
            unsigned char CF3TXIF:1;
            unsigned char CF4TXIF:1;
            unsigned char CF5TXIF:1;
            unsigned char :2;
            unsigned char :8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0CFTISTS;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTR:1;
            unsigned char TMTAR:1;
            unsigned char TMOM:1;
            unsigned char :5;
        } BIT;
    } RSCAN0TMC0;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTR:1;
            unsigned char TMTAR:1;
            unsigned char TMOM:1;
            unsigned char :5;
        } BIT;
    } RSCAN0TMC1;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTR:1;
            unsigned char TMTAR:1;
            unsigned char TMOM:1;
            unsigned char :5;
        } BIT;
    } RSCAN0TMC2;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTR:1;
            unsigned char TMTAR:1;
            unsigned char TMOM:1;
            unsigned char :5;
        } BIT;
    } RSCAN0TMC3;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTR:1;
            unsigned char TMTAR:1;
            unsigned char TMOM:1;
            unsigned char :5;
        } BIT;
    } RSCAN0TMC4;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTR:1;
            unsigned char TMTAR:1;
            unsigned char TMOM:1;
            unsigned char :5;
        } BIT;
    } RSCAN0TMC5;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTR:1;
            unsigned char TMTAR:1;
            unsigned char TMOM:1;
            unsigned char :5;
        } BIT;
    } RSCAN0TMC6;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTR:1;
            unsigned char TMTAR:1;
            unsigned char TMOM:1;
            unsigned char :5;
        } BIT;
    } RSCAN0TMC7;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTR:1;
            unsigned char TMTAR:1;
            unsigned char TMOM:1;
            unsigned char :5;
        } BIT;
    } RSCAN0TMC8;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTR:1;
            unsigned char TMTAR:1;
            unsigned char TMOM:1;
            unsigned char :5;
        } BIT;
    } RSCAN0TMC9;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTR:1;
            unsigned char TMTAR:1;
            unsigned char TMOM:1;
            unsigned char :5;
        } BIT;
    } RSCAN0TMC10;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTR:1;
            unsigned char TMTAR:1;
            unsigned char TMOM:1;
            unsigned char :5;
        } BIT;
    } RSCAN0TMC11;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTR:1;
            unsigned char TMTAR:1;
            unsigned char TMOM:1;
            unsigned char :5;
        } BIT;
    } RSCAN0TMC12;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTR:1;
            unsigned char TMTAR:1;
            unsigned char TMOM:1;
            unsigned char :5;
        } BIT;
    } RSCAN0TMC13;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTR:1;
            unsigned char TMTAR:1;
            unsigned char TMOM:1;
            unsigned char :5;
        } BIT;
    } RSCAN0TMC14;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTR:1;
            unsigned char TMTAR:1;
            unsigned char TMOM:1;
            unsigned char :5;
        } BIT;
    } RSCAN0TMC15;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTR:1;
            unsigned char TMTAR:1;
            unsigned char TMOM:1;
            unsigned char :5;
        } BIT;
    } RSCAN0TMC16;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTR:1;
            unsigned char TMTAR:1;
            unsigned char TMOM:1;
            unsigned char :5;
        } BIT;
    } RSCAN0TMC17;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTR:1;
            unsigned char TMTAR:1;
            unsigned char TMOM:1;
            unsigned char :5;
        } BIT;
    } RSCAN0TMC18;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTR:1;
            unsigned char TMTAR:1;
            unsigned char TMOM:1;
            unsigned char :5;
        } BIT;
    } RSCAN0TMC19;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTR:1;
            unsigned char TMTAR:1;
            unsigned char TMOM:1;
            unsigned char :5;
        } BIT;
    } RSCAN0TMC20;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTR:1;
            unsigned char TMTAR:1;
            unsigned char TMOM:1;
            unsigned char :5;
        } BIT;
    } RSCAN0TMC21;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTR:1;
            unsigned char TMTAR:1;
            unsigned char TMOM:1;
            unsigned char :5;
        } BIT;
    } RSCAN0TMC22;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTR:1;
            unsigned char TMTAR:1;
            unsigned char TMOM:1;
            unsigned char :5;
        } BIT;
    } RSCAN0TMC23;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTR:1;
            unsigned char TMTAR:1;
            unsigned char TMOM:1;
            unsigned char :5;
        } BIT;
    } RSCAN0TMC24;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTR:1;
            unsigned char TMTAR:1;
            unsigned char TMOM:1;
            unsigned char :5;
        } BIT;
    } RSCAN0TMC25;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTR:1;
            unsigned char TMTAR:1;
            unsigned char TMOM:1;
            unsigned char :5;
        } BIT;
    } RSCAN0TMC26;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTR:1;
            unsigned char TMTAR:1;
            unsigned char TMOM:1;
            unsigned char :5;
        } BIT;
    } RSCAN0TMC27;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTR:1;
            unsigned char TMTAR:1;
            unsigned char TMOM:1;
            unsigned char :5;
        } BIT;
    } RSCAN0TMC28;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTR:1;
            unsigned char TMTAR:1;
            unsigned char TMOM:1;
            unsigned char :5;
        } BIT;
    } RSCAN0TMC29;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTR:1;
            unsigned char TMTAR:1;
            unsigned char TMOM:1;
            unsigned char :5;
        } BIT;
    } RSCAN0TMC30;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTR:1;
            unsigned char TMTAR:1;
            unsigned char TMOM:1;
            unsigned char :5;
        } BIT;
    } RSCAN0TMC31;
    char           wk6[96];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTSTS:1;
            unsigned char TMTRF:2;
            unsigned char TMTRM:1;
            unsigned char TMTARM:1;
            unsigned char :3;
        } BIT;
    } RSCAN0TMSTS0;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTSTS:1;
            unsigned char TMTRF:2;
            unsigned char TMTRM:1;
            unsigned char TMTARM:1;
            unsigned char :3;
        } BIT;
    } RSCAN0TMSTS1;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTSTS:1;
            unsigned char TMTRF:2;
            unsigned char TMTRM:1;
            unsigned char TMTARM:1;
            unsigned char :3;
        } BIT;
    } RSCAN0TMSTS2;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTSTS:1;
            unsigned char TMTRF:2;
            unsigned char TMTRM:1;
            unsigned char TMTARM:1;
            unsigned char :3;
        } BIT;
    } RSCAN0TMSTS3;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTSTS:1;
            unsigned char TMTRF:2;
            unsigned char TMTRM:1;
            unsigned char TMTARM:1;
            unsigned char :3;
        } BIT;
    } RSCAN0TMSTS4;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTSTS:1;
            unsigned char TMTRF:2;
            unsigned char TMTRM:1;
            unsigned char TMTARM:1;
            unsigned char :3;
        } BIT;
    } RSCAN0TMSTS5;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTSTS:1;
            unsigned char TMTRF:2;
            unsigned char TMTRM:1;
            unsigned char TMTARM:1;
            unsigned char :3;
        } BIT;
    } RSCAN0TMSTS6;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTSTS:1;
            unsigned char TMTRF:2;
            unsigned char TMTRM:1;
            unsigned char TMTARM:1;
            unsigned char :3;
        } BIT;
    } RSCAN0TMSTS7;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTSTS:1;
            unsigned char TMTRF:2;
            unsigned char TMTRM:1;
            unsigned char TMTARM:1;
            unsigned char :3;
        } BIT;
    } RSCAN0TMSTS8;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTSTS:1;
            unsigned char TMTRF:2;
            unsigned char TMTRM:1;
            unsigned char TMTARM:1;
            unsigned char :3;
        } BIT;
    } RSCAN0TMSTS9;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTSTS:1;
            unsigned char TMTRF:2;
            unsigned char TMTRM:1;
            unsigned char TMTARM:1;
            unsigned char :3;
        } BIT;
    } RSCAN0TMSTS10;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTSTS:1;
            unsigned char TMTRF:2;
            unsigned char TMTRM:1;
            unsigned char TMTARM:1;
            unsigned char :3;
        } BIT;
    } RSCAN0TMSTS11;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTSTS:1;
            unsigned char TMTRF:2;
            unsigned char TMTRM:1;
            unsigned char TMTARM:1;
            unsigned char :3;
        } BIT;
    } RSCAN0TMSTS12;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTSTS:1;
            unsigned char TMTRF:2;
            unsigned char TMTRM:1;
            unsigned char TMTARM:1;
            unsigned char :3;
        } BIT;
    } RSCAN0TMSTS13;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTSTS:1;
            unsigned char TMTRF:2;
            unsigned char TMTRM:1;
            unsigned char TMTARM:1;
            unsigned char :3;
        } BIT;
    } RSCAN0TMSTS14;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTSTS:1;
            unsigned char TMTRF:2;
            unsigned char TMTRM:1;
            unsigned char TMTARM:1;
            unsigned char :3;
        } BIT;
    } RSCAN0TMSTS15;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTSTS:1;
            unsigned char TMTRF:2;
            unsigned char TMTRM:1;
            unsigned char TMTARM:1;
            unsigned char :3;
        } BIT;
    } RSCAN0TMSTS16;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTSTS:1;
            unsigned char TMTRF:2;
            unsigned char TMTRM:1;
            unsigned char TMTARM:1;
            unsigned char :3;
        } BIT;
    } RSCAN0TMSTS17;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTSTS:1;
            unsigned char TMTRF:2;
            unsigned char TMTRM:1;
            unsigned char TMTARM:1;
            unsigned char :3;
        } BIT;
    } RSCAN0TMSTS18;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTSTS:1;
            unsigned char TMTRF:2;
            unsigned char TMTRM:1;
            unsigned char TMTARM:1;
            unsigned char :3;
        } BIT;
    } RSCAN0TMSTS19;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTSTS:1;
            unsigned char TMTRF:2;
            unsigned char TMTRM:1;
            unsigned char TMTARM:1;
            unsigned char :3;
        } BIT;
    } RSCAN0TMSTS20;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTSTS:1;
            unsigned char TMTRF:2;
            unsigned char TMTRM:1;
            unsigned char TMTARM:1;
            unsigned char :3;
        } BIT;
    } RSCAN0TMSTS21;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTSTS:1;
            unsigned char TMTRF:2;
            unsigned char TMTRM:1;
            unsigned char TMTARM:1;
            unsigned char :3;
        } BIT;
    } RSCAN0TMSTS22;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTSTS:1;
            unsigned char TMTRF:2;
            unsigned char TMTRM:1;
            unsigned char TMTARM:1;
            unsigned char :3;
        } BIT;
    } RSCAN0TMSTS23;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTSTS:1;
            unsigned char TMTRF:2;
            unsigned char TMTRM:1;
            unsigned char TMTARM:1;
            unsigned char :3;
        } BIT;
    } RSCAN0TMSTS24;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTSTS:1;
            unsigned char TMTRF:2;
            unsigned char TMTRM:1;
            unsigned char TMTARM:1;
            unsigned char :3;
        } BIT;
    } RSCAN0TMSTS25;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTSTS:1;
            unsigned char TMTRF:2;
            unsigned char TMTRM:1;
            unsigned char TMTARM:1;
            unsigned char :3;
        } BIT;
    } RSCAN0TMSTS26;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTSTS:1;
            unsigned char TMTRF:2;
            unsigned char TMTRM:1;
            unsigned char TMTARM:1;
            unsigned char :3;
        } BIT;
    } RSCAN0TMSTS27;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTSTS:1;
            unsigned char TMTRF:2;
            unsigned char TMTRM:1;
            unsigned char TMTARM:1;
            unsigned char :3;
        } BIT;
    } RSCAN0TMSTS28;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTSTS:1;
            unsigned char TMTRF:2;
            unsigned char TMTRM:1;
            unsigned char TMTARM:1;
            unsigned char :3;
        } BIT;
    } RSCAN0TMSTS29;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTSTS:1;
            unsigned char TMTRF:2;
            unsigned char TMTRM:1;
            unsigned char TMTARM:1;
            unsigned char :3;
        } BIT;
    } RSCAN0TMSTS30;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TMTSTS:1;
            unsigned char TMTRF:2;
            unsigned char TMTRM:1;
            unsigned char TMTARM:1;
            unsigned char :3;
        } BIT;
    } RSCAN0TMSTS31;
    char           wk7[96];
    union {
        unsigned long LONG;
        struct {
            unsigned short TMTRSTSp_l;
            unsigned short TMTRSTSp_h;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
    } RSCAN0TMTRSTS0;
    char           wk8[12];
    union {
        unsigned long LONG;
        struct {
            unsigned short TMTARSTSp_l;
            unsigned short TMTARSTSp_h;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
    } RSCAN0TMTARSTS0;
    char           wk9[12];
    union {
        unsigned long LONG;
        struct {
            unsigned short TMTCSTSp_l;
            unsigned short TMTCSTSp_h;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
    } RSCAN0TMTCSTS0;
    char           wk10[12];
    union {
        unsigned long LONG;
        struct {
            unsigned short TMTASTSp_l;
            unsigned short TMTASTSp_h;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
    } RSCAN0TMTASTS0;
    char           wk11[12];
    union {
        unsigned long LONG;
        struct {
            unsigned short TMIEp_l;
            unsigned short TMIEp_h;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
    } RSCAN0TMIEC0;
    char           wk12[12];
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char TXQE:1;
            unsigned char :7;
            unsigned char TXQDC:4;
            unsigned char TXQIE:1;
            unsigned char TXQIM:1;
            unsigned char :2;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0TXQCC0;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char TXQE:1;
            unsigned char :7;
            unsigned char TXQDC:4;
            unsigned char TXQIE:1;
            unsigned char TXQIM:1;
            unsigned char :2;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0TXQCC1;
    char           wk13[24];
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char TXQEMP:1;
            unsigned char TXQFLL:1;
            unsigned char TXQIF:1;
            unsigned char :5;
            unsigned char :8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0TXQSTS0;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char TXQEMP:1;
            unsigned char TXQFLL:1;
            unsigned char TXQIF:1;
            unsigned char :5;
            unsigned char :8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0TXQSTS1;
    char           wk14[24];
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char TXQPC:8;
            unsigned char :8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0TXQPCTR0;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char TXQPC:8;
            unsigned char :8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0TXQPCTR1;
    char           wk15[24];
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char THLE:1;
            unsigned char :7;
            unsigned char THLIE:1;
            unsigned char THLIM:1;
            unsigned char THLDTE:1;
            unsigned char :5;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0THLCC0;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char THLE:1;
            unsigned char :7;
            unsigned char THLIE:1;
            unsigned char THLIM:1;
            unsigned char THLDTE:1;
            unsigned char :5;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0THLCC1;
    char           wk16[24];
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char THLEMP:1;
            unsigned char THLFLL:1;
            unsigned char THLELT:1;
            unsigned char THLIF:1;
            unsigned char :4;
            unsigned char THLMC:5;
            unsigned char :3;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0THLSTS0;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char THLEMP:1;
            unsigned char THLFLL:1;
            unsigned char THLELT:1;
            unsigned char THLIF:1;
            unsigned char :4;
            unsigned char THLMC:5;
            unsigned char :3;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0THLSTS1;
    char           wk17[24];
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char THLPC:8;
            unsigned char :8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0THLPCTR0;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char THLPC:8;
            unsigned char :8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0THLPCTR1;
    char           wk18[24];
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char TSIF0:1;
            unsigned char TAIF0:1;
            unsigned char TQIF0:1;
            unsigned char CFTIF0:1;
            unsigned char THIF0:1;
            unsigned char :3;
            unsigned char TSIF1:1;
            unsigned char TAIF1:1;
            unsigned char TQIF1:1;
            unsigned char CFTIF1:1;
            unsigned char THIF1:1;
            unsigned char :3;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0GTINTSTS0;
    char           wk19[4];
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char C0ICBCE:1;
            unsigned char C1ICBCE:1;
            unsigned char :6;
            unsigned char :8;
            unsigned char RTMPS:7;
            unsigned char :1;
            unsigned char :8;
        } BIT;
    } RSCAN0GTSTCFG;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char ICBCTME:1;
            unsigned char :1;
            unsigned char RTME:1;
            unsigned char :5;
            unsigned char :8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0GTSTCTR;
    char           wk20[12];
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
    } RSCAN0GLOCKK;
    char           wk21[128];
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLID:29;
            unsigned long GAFLLB:1;
            unsigned long GAFLRTR:1;
            unsigned long GAFLIDE:1;
        } BIT;
    } RSCAN0GAFLID0;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLIDM:29;
            unsigned long :1;
            unsigned long GAFLRTRM:1;
            unsigned long GAFLIDEM:1;
        } BIT;
    } RSCAN0GAFLM0;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long :8;
            unsigned long GAFLRMDP:7;
            unsigned long GAFLRMV:1;
            unsigned long GAFLPTR:12;
            unsigned long GAFLDLC:4;
        } BIT;
    } RSCAN0GAFLP00;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLFDPr:8;
            unsigned long GAFLFDP:18;
            unsigned long :6;
        } BIT;
    } RSCAN0GAFLP10;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLID:29;
            unsigned long GAFLLB:1;
            unsigned long GAFLRTR:1;
            unsigned long GAFLIDE:1;
        } BIT;
    } RSCAN0GAFLID1;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLIDM:29;
            unsigned long :1;
            unsigned long GAFLRTRM:1;
            unsigned long GAFLIDEM:1;
        } BIT;
    } RSCAN0GAFLM1;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long :8;
            unsigned long GAFLRMDP:7;
            unsigned long GAFLRMV:1;
            unsigned long GAFLPTR:12;
            unsigned long GAFLDLC:4;
        } BIT;
    } RSCAN0GAFLP01;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLFDPr:8;
            unsigned long GAFLFDP:18;
            unsigned long :6;
        } BIT;
    } RSCAN0GAFLP11;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLID:29;
            unsigned long GAFLLB:1;
            unsigned long GAFLRTR:1;
            unsigned long GAFLIDE:1;
        } BIT;
    } RSCAN0GAFLID2;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLIDM:29;
            unsigned long :1;
            unsigned long GAFLRTRM:1;
            unsigned long GAFLIDEM:1;
        } BIT;
    } RSCAN0GAFLM2;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long :8;
            unsigned long GAFLRMDP:7;
            unsigned long GAFLRMV:1;
            unsigned long GAFLPTR:12;
            unsigned long GAFLDLC:4;
        } BIT;
    } RSCAN0GAFLP02;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLFDPr:8;
            unsigned long GAFLFDP:18;
            unsigned long :6;
        } BIT;
    } RSCAN0GAFLP12;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLID:29;
            unsigned long GAFLLB:1;
            unsigned long GAFLRTR:1;
            unsigned long GAFLIDE:1;
        } BIT;
    } RSCAN0GAFLID3;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLIDM:29;
            unsigned long :1;
            unsigned long GAFLRTRM:1;
            unsigned long GAFLIDEM:1;
        } BIT;
    } RSCAN0GAFLM3;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long :8;
            unsigned long GAFLRMDP:7;
            unsigned long GAFLRMV:1;
            unsigned long GAFLPTR:12;
            unsigned long GAFLDLC:4;
        } BIT;
    } RSCAN0GAFLP03;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLFDPr:8;
            unsigned long GAFLFDP:18;
            unsigned long :6;
        } BIT;
    } RSCAN0GAFLP13;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLID:29;
            unsigned long GAFLLB:1;
            unsigned long GAFLRTR:1;
            unsigned long GAFLIDE:1;
        } BIT;
    } RSCAN0GAFLID4;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLIDM:29;
            unsigned long :1;
            unsigned long GAFLRTRM:1;
            unsigned long GAFLIDEM:1;
        } BIT;
    } RSCAN0GAFLM4;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long :8;
            unsigned long GAFLRMDP:7;
            unsigned long GAFLRMV:1;
            unsigned long GAFLPTR:12;
            unsigned long GAFLDLC:4;
        } BIT;
    } RSCAN0GAFLP04;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLFDPr:8;
            unsigned long GAFLFDP:18;
            unsigned long :6;
        } BIT;
    } RSCAN0GAFLP14;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLID:29;
            unsigned long GAFLLB:1;
            unsigned long GAFLRTR:1;
            unsigned long GAFLIDE:1;
        } BIT;
    } RSCAN0GAFLID5;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLIDM:29;
            unsigned long :1;
            unsigned long GAFLRTRM:1;
            unsigned long GAFLIDEM:1;
        } BIT;
    } RSCAN0GAFLM5;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long :8;
            unsigned long GAFLRMDP:7;
            unsigned long GAFLRMV:1;
            unsigned long GAFLPTR:12;
            unsigned long GAFLDLC:4;
        } BIT;
    } RSCAN0GAFLP05;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLFDPr:8;
            unsigned long GAFLFDP:18;
            unsigned long :6;
        } BIT;
    } RSCAN0GAFLP15;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLID:29;
            unsigned long GAFLLB:1;
            unsigned long GAFLRTR:1;
            unsigned long GAFLIDE:1;
        } BIT;
    } RSCAN0GAFLID6;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLIDM:29;
            unsigned long :1;
            unsigned long GAFLRTRM:1;
            unsigned long GAFLIDEM:1;
        } BIT;
    } RSCAN0GAFLM6;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long :8;
            unsigned long GAFLRMDP:7;
            unsigned long GAFLRMV:1;
            unsigned long GAFLPTR:12;
            unsigned long GAFLDLC:4;
        } BIT;
    } RSCAN0GAFLP06;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLFDPr:8;
            unsigned long GAFLFDP:18;
            unsigned long :6;
        } BIT;
    } RSCAN0GAFLP16;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLID:29;
            unsigned long GAFLLB:1;
            unsigned long GAFLRTR:1;
            unsigned long GAFLIDE:1;
        } BIT;
    } RSCAN0GAFLID7;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLIDM:29;
            unsigned long :1;
            unsigned long GAFLRTRM:1;
            unsigned long GAFLIDEM:1;
        } BIT;
    } RSCAN0GAFLM7;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long :8;
            unsigned long GAFLRMDP:7;
            unsigned long GAFLRMV:1;
            unsigned long GAFLPTR:12;
            unsigned long GAFLDLC:4;
        } BIT;
    } RSCAN0GAFLP07;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLFDPr:8;
            unsigned long GAFLFDP:18;
            unsigned long :6;
        } BIT;
    } RSCAN0GAFLP17;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLID:29;
            unsigned long GAFLLB:1;
            unsigned long GAFLRTR:1;
            unsigned long GAFLIDE:1;
        } BIT;
    } RSCAN0GAFLID8;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLIDM:29;
            unsigned long :1;
            unsigned long GAFLRTRM:1;
            unsigned long GAFLIDEM:1;
        } BIT;
    } RSCAN0GAFLM8;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long :8;
            unsigned long GAFLRMDP:7;
            unsigned long GAFLRMV:1;
            unsigned long GAFLPTR:12;
            unsigned long GAFLDLC:4;
        } BIT;
    } RSCAN0GAFLP08;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLFDPr:8;
            unsigned long GAFLFDP:18;
            unsigned long :6;
        } BIT;
    } RSCAN0GAFLP18;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLID:29;
            unsigned long GAFLLB:1;
            unsigned long GAFLRTR:1;
            unsigned long GAFLIDE:1;
        } BIT;
    } RSCAN0GAFLID9;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLIDM:29;
            unsigned long :1;
            unsigned long GAFLRTRM:1;
            unsigned long GAFLIDEM:1;
        } BIT;
    } RSCAN0GAFLM9;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long :8;
            unsigned long GAFLRMDP:7;
            unsigned long GAFLRMV:1;
            unsigned long GAFLPTR:12;
            unsigned long GAFLDLC:4;
        } BIT;
    } RSCAN0GAFLP09;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLFDPr:8;
            unsigned long GAFLFDP:18;
            unsigned long :6;
        } BIT;
    } RSCAN0GAFLP19;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLID:29;
            unsigned long GAFLLB:1;
            unsigned long GAFLRTR:1;
            unsigned long GAFLIDE:1;
        } BIT;
    } RSCAN0GAFLID10;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLIDM:29;
            unsigned long :1;
            unsigned long GAFLRTRM:1;
            unsigned long GAFLIDEM:1;
        } BIT;
    } RSCAN0GAFLM10;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long :8;
            unsigned long GAFLRMDP:7;
            unsigned long GAFLRMV:1;
            unsigned long GAFLPTR:12;
            unsigned long GAFLDLC:4;
        } BIT;
    } RSCAN0GAFLP010;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLFDPr:8;
            unsigned long GAFLFDP:18;
            unsigned long :6;
        } BIT;
    } RSCAN0GAFLP110;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLID:29;
            unsigned long GAFLLB:1;
            unsigned long GAFLRTR:1;
            unsigned long GAFLIDE:1;
        } BIT;
    } RSCAN0GAFLID11;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLIDM:29;
            unsigned long :1;
            unsigned long GAFLRTRM:1;
            unsigned long GAFLIDEM:1;
        } BIT;
    } RSCAN0GAFLM11;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long :8;
            unsigned long GAFLRMDP:7;
            unsigned long GAFLRMV:1;
            unsigned long GAFLPTR:12;
            unsigned long GAFLDLC:4;
        } BIT;
    } RSCAN0GAFLP011;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLFDPr:8;
            unsigned long GAFLFDP:18;
            unsigned long :6;
        } BIT;
    } RSCAN0GAFLP111;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLID:29;
            unsigned long GAFLLB:1;
            unsigned long GAFLRTR:1;
            unsigned long GAFLIDE:1;
        } BIT;
    } RSCAN0GAFLID12;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLIDM:29;
            unsigned long :1;
            unsigned long GAFLRTRM:1;
            unsigned long GAFLIDEM:1;
        } BIT;
    } RSCAN0GAFLM12;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long :8;
            unsigned long GAFLRMDP:7;
            unsigned long GAFLRMV:1;
            unsigned long GAFLPTR:12;
            unsigned long GAFLDLC:4;
        } BIT;
    } RSCAN0GAFLP012;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLFDPr:8;
            unsigned long GAFLFDP:18;
            unsigned long :6;
        } BIT;
    } RSCAN0GAFLP112;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLID:29;
            unsigned long GAFLLB:1;
            unsigned long GAFLRTR:1;
            unsigned long GAFLIDE:1;
        } BIT;
    } RSCAN0GAFLID13;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLIDM:29;
            unsigned long :1;
            unsigned long GAFLRTRM:1;
            unsigned long GAFLIDEM:1;
        } BIT;
    } RSCAN0GAFLM13;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long :8;
            unsigned long GAFLRMDP:7;
            unsigned long GAFLRMV:1;
            unsigned long GAFLPTR:12;
            unsigned long GAFLDLC:4;
        } BIT;
    } RSCAN0GAFLP013;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLFDPr:8;
            unsigned long GAFLFDP:18;
            unsigned long :6;
        } BIT;
    } RSCAN0GAFLP113;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLID:29;
            unsigned long GAFLLB:1;
            unsigned long GAFLRTR:1;
            unsigned long GAFLIDE:1;
        } BIT;
    } RSCAN0GAFLID14;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLIDM:29;
            unsigned long :1;
            unsigned long GAFLRTRM:1;
            unsigned long GAFLIDEM:1;
        } BIT;
    } RSCAN0GAFLM14;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long :8;
            unsigned long GAFLRMDP:7;
            unsigned long GAFLRMV:1;
            unsigned long GAFLPTR:12;
            unsigned long GAFLDLC:4;
        } BIT;
    } RSCAN0GAFLP014;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLFDPr:8;
            unsigned long GAFLFDP:18;
            unsigned long :6;
        } BIT;
    } RSCAN0GAFLP114;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLID:29;
            unsigned long GAFLLB:1;
            unsigned long GAFLRTR:1;
            unsigned long GAFLIDE:1;
        } BIT;
    } RSCAN0GAFLID15;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLIDM:29;
            unsigned long :1;
            unsigned long GAFLRTRM:1;
            unsigned long GAFLIDEM:1;
        } BIT;
    } RSCAN0GAFLM15;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long :8;
            unsigned long GAFLRMDP:7;
            unsigned long GAFLRMV:1;
            unsigned long GAFLPTR:12;
            unsigned long GAFLDLC:4;
        } BIT;
    } RSCAN0GAFLP015;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long GAFLFDPr:8;
            unsigned long GAFLFDP:18;
            unsigned long :6;
        } BIT;
    } RSCAN0GAFLP115;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMID:29;
            unsigned long :1;
            unsigned long RMRTR:1;
            unsigned long RMIDE:1;
        } BIT;
    } RSCAN0RMID0;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMTS:16;
            unsigned long RMPTR:12;
            unsigned long RMDLC:4;
        } BIT;
    } RSCAN0RMPTR0;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB0;
            unsigned char RMDB1;
            unsigned char RMDB2;
            unsigned char RMDB3;
        } BYTE;
    } RSCAN0RMDF00;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB4;
            unsigned char RMDB5;
            unsigned char RMDB6;
            unsigned char RMDB7;
        } BYTE;
    } RSCAN0RMDF10;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMID:29;
            unsigned long :1;
            unsigned long RMRTR:1;
            unsigned long RMIDE:1;
        } BIT;
    } RSCAN0RMID1;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMTS:16;
            unsigned long RMPTR:12;
            unsigned long RMDLC:4;
        } BIT;
    } RSCAN0RMPTR1;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB0;
            unsigned char RMDB1;
            unsigned char RMDB2;
            unsigned char RMDB3;
        } BYTE;
    } RSCAN0RMDF01;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB4;
            unsigned char RMDB5;
            unsigned char RMDB6;
            unsigned char RMDB7;
        } BYTE;
    } RSCAN0RMDF11;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMID:29;
            unsigned long :1;
            unsigned long RMRTR:1;
            unsigned long RMIDE:1;
        } BIT;
    } RSCAN0RMID2;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMTS:16;
            unsigned long RMPTR:12;
            unsigned long RMDLC:4;
        } BIT;
    } RSCAN0RMPTR2;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB0;
            unsigned char RMDB1;
            unsigned char RMDB2;
            unsigned char RMDB3;
        } BYTE;
    } RSCAN0RMDF02;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB4;
            unsigned char RMDB5;
            unsigned char RMDB6;
            unsigned char RMDB7;
        } BYTE;
    } RSCAN0RMDF12;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMID:29;
            unsigned long :1;
            unsigned long RMRTR:1;
            unsigned long RMIDE:1;
        } BIT;
    } RSCAN0RMID3;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMTS:16;
            unsigned long RMPTR:12;
            unsigned long RMDLC:4;
        } BIT;
    } RSCAN0RMPTR3;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB0;
            unsigned char RMDB1;
            unsigned char RMDB2;
            unsigned char RMDB3;
        } BYTE;
    } RSCAN0RMDF03;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB4;
            unsigned char RMDB5;
            unsigned char RMDB6;
            unsigned char RMDB7;
        } BYTE;
    } RSCAN0RMDF13;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMID:29;
            unsigned long :1;
            unsigned long RMRTR:1;
            unsigned long RMIDE:1;
        } BIT;
    } RSCAN0RMID4;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMTS:16;
            unsigned long RMPTR:12;
            unsigned long RMDLC:4;
        } BIT;
    } RSCAN0RMPTR4;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB0;
            unsigned char RMDB1;
            unsigned char RMDB2;
            unsigned char RMDB3;
        } BYTE;
    } RSCAN0RMDF04;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB4;
            unsigned char RMDB5;
            unsigned char RMDB6;
            unsigned char RMDB7;
        } BYTE;
    } RSCAN0RMDF14;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMID:29;
            unsigned long :1;
            unsigned long RMRTR:1;
            unsigned long RMIDE:1;
        } BIT;
    } RSCAN0RMID5;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMTS:16;
            unsigned long RMPTR:12;
            unsigned long RMDLC:4;
        } BIT;
    } RSCAN0RMPTR5;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB0;
            unsigned char RMDB1;
            unsigned char RMDB2;
            unsigned char RMDB3;
        } BYTE;
    } RSCAN0RMDF05;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB4;
            unsigned char RMDB5;
            unsigned char RMDB6;
            unsigned char RMDB7;
        } BYTE;
    } RSCAN0RMDF15;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMID:29;
            unsigned long :1;
            unsigned long RMRTR:1;
            unsigned long RMIDE:1;
        } BIT;
    } RSCAN0RMID6;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMTS:16;
            unsigned long RMPTR:12;
            unsigned long RMDLC:4;
        } BIT;
    } RSCAN0RMPTR6;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB0;
            unsigned char RMDB1;
            unsigned char RMDB2;
            unsigned char RMDB3;
        } BYTE;
    } RSCAN0RMDF06;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB4;
            unsigned char RMDB5;
            unsigned char RMDB6;
            unsigned char RMDB7;
        } BYTE;
    } RSCAN0RMDF16;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMID:29;
            unsigned long :1;
            unsigned long RMRTR:1;
            unsigned long RMIDE:1;
        } BIT;
    } RSCAN0RMID7;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMTS:16;
            unsigned long RMPTR:12;
            unsigned long RMDLC:4;
        } BIT;
    } RSCAN0RMPTR7;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB0;
            unsigned char RMDB1;
            unsigned char RMDB2;
            unsigned char RMDB3;
        } BYTE;
    } RSCAN0RMDF07;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB4;
            unsigned char RMDB5;
            unsigned char RMDB6;
            unsigned char RMDB7;
        } BYTE;
    } RSCAN0RMDF17;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMID:29;
            unsigned long :1;
            unsigned long RMRTR:1;
            unsigned long RMIDE:1;
        } BIT;
    } RSCAN0RMID8;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMTS:16;
            unsigned long RMPTR:12;
            unsigned long RMDLC:4;
        } BIT;
    } RSCAN0RMPTR8;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB0;
            unsigned char RMDB1;
            unsigned char RMDB2;
            unsigned char RMDB3;
        } BYTE;
    } RSCAN0RMDF08;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB4;
            unsigned char RMDB5;
            unsigned char RMDB6;
            unsigned char RMDB7;
        } BYTE;
    } RSCAN0RMDF18;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMID:29;
            unsigned long :1;
            unsigned long RMRTR:1;
            unsigned long RMIDE:1;
        } BIT;
    } RSCAN0RMID9;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMTS:16;
            unsigned long RMPTR:12;
            unsigned long RMDLC:4;
        } BIT;
    } RSCAN0RMPTR9;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB0;
            unsigned char RMDB1;
            unsigned char RMDB2;
            unsigned char RMDB3;
        } BYTE;
    } RSCAN0RMDF09;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB4;
            unsigned char RMDB5;
            unsigned char RMDB6;
            unsigned char RMDB7;
        } BYTE;
    } RSCAN0RMDF19;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMID:29;
            unsigned long :1;
            unsigned long RMRTR:1;
            unsigned long RMIDE:1;
        } BIT;
    } RSCAN0RMID10;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMTS:16;
            unsigned long RMPTR:12;
            unsigned long RMDLC:4;
        } BIT;
    } RSCAN0RMPTR10;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB0;
            unsigned char RMDB1;
            unsigned char RMDB2;
            unsigned char RMDB3;
        } BYTE;
    } RSCAN0RMDF010;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB4;
            unsigned char RMDB5;
            unsigned char RMDB6;
            unsigned char RMDB7;
        } BYTE;
    } RSCAN0RMDF110;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMID:29;
            unsigned long :1;
            unsigned long RMRTR:1;
            unsigned long RMIDE:1;
        } BIT;
    } RSCAN0RMID11;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMTS:16;
            unsigned long RMPTR:12;
            unsigned long RMDLC:4;
        } BIT;
    } RSCAN0RMPTR11;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB0;
            unsigned char RMDB1;
            unsigned char RMDB2;
            unsigned char RMDB3;
        } BYTE;
    } RSCAN0RMDF011;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB4;
            unsigned char RMDB5;
            unsigned char RMDB6;
            unsigned char RMDB7;
        } BYTE;
    } RSCAN0RMDF111;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMID:29;
            unsigned long :1;
            unsigned long RMRTR:1;
            unsigned long RMIDE:1;
        } BIT;
    } RSCAN0RMID12;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMTS:16;
            unsigned long RMPTR:12;
            unsigned long RMDLC:4;
        } BIT;
    } RSCAN0RMPTR12;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB0;
            unsigned char RMDB1;
            unsigned char RMDB2;
            unsigned char RMDB3;
        } BYTE;
    } RSCAN0RMDF012;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB4;
            unsigned char RMDB5;
            unsigned char RMDB6;
            unsigned char RMDB7;
        } BYTE;
    } RSCAN0RMDF112;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMID:29;
            unsigned long :1;
            unsigned long RMRTR:1;
            unsigned long RMIDE:1;
        } BIT;
    } RSCAN0RMID13;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMTS:16;
            unsigned long RMPTR:12;
            unsigned long RMDLC:4;
        } BIT;
    } RSCAN0RMPTR13;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB0;
            unsigned char RMDB1;
            unsigned char RMDB2;
            unsigned char RMDB3;
        } BYTE;
    } RSCAN0RMDF013;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB4;
            unsigned char RMDB5;
            unsigned char RMDB6;
            unsigned char RMDB7;
        } BYTE;
    } RSCAN0RMDF113;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMID:29;
            unsigned long :1;
            unsigned long RMRTR:1;
            unsigned long RMIDE:1;
        } BIT;
    } RSCAN0RMID14;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMTS:16;
            unsigned long RMPTR:12;
            unsigned long RMDLC:4;
        } BIT;
    } RSCAN0RMPTR14;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB0;
            unsigned char RMDB1;
            unsigned char RMDB2;
            unsigned char RMDB3;
        } BYTE;
    } RSCAN0RMDF014;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB4;
            unsigned char RMDB5;
            unsigned char RMDB6;
            unsigned char RMDB7;
        } BYTE;
    } RSCAN0RMDF114;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMID:29;
            unsigned long :1;
            unsigned long RMRTR:1;
            unsigned long RMIDE:1;
        } BIT;
    } RSCAN0RMID15;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMTS:16;
            unsigned long RMPTR:12;
            unsigned long RMDLC:4;
        } BIT;
    } RSCAN0RMPTR15;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB0;
            unsigned char RMDB1;
            unsigned char RMDB2;
            unsigned char RMDB3;
        } BYTE;
    } RSCAN0RMDF015;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB4;
            unsigned char RMDB5;
            unsigned char RMDB6;
            unsigned char RMDB7;
        } BYTE;
    } RSCAN0RMDF115;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMID:29;
            unsigned long :1;
            unsigned long RMRTR:1;
            unsigned long RMIDE:1;
        } BIT;
    } RSCAN0RMID16;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMTS:16;
            unsigned long RMPTR:12;
            unsigned long RMDLC:4;
        } BIT;
    } RSCAN0RMPTR16;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB0;
            unsigned char RMDB1;
            unsigned char RMDB2;
            unsigned char RMDB3;
        } BYTE;
    } RSCAN0RMDF016;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB4;
            unsigned char RMDB5;
            unsigned char RMDB6;
            unsigned char RMDB7;
        } BYTE;
    } RSCAN0RMDF116;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMID:29;
            unsigned long :1;
            unsigned long RMRTR:1;
            unsigned long RMIDE:1;
        } BIT;
    } RSCAN0RMID17;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMTS:16;
            unsigned long RMPTR:12;
            unsigned long RMDLC:4;
        } BIT;
    } RSCAN0RMPTR17;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB0;
            unsigned char RMDB1;
            unsigned char RMDB2;
            unsigned char RMDB3;
        } BYTE;
    } RSCAN0RMDF017;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB4;
            unsigned char RMDB5;
            unsigned char RMDB6;
            unsigned char RMDB7;
        } BYTE;
    } RSCAN0RMDF117;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMID:29;
            unsigned long :1;
            unsigned long RMRTR:1;
            unsigned long RMIDE:1;
        } BIT;
    } RSCAN0RMID18;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMTS:16;
            unsigned long RMPTR:12;
            unsigned long RMDLC:4;
        } BIT;
    } RSCAN0RMPTR18;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB0;
            unsigned char RMDB1;
            unsigned char RMDB2;
            unsigned char RMDB3;
        } BYTE;
    } RSCAN0RMDF018;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB4;
            unsigned char RMDB5;
            unsigned char RMDB6;
            unsigned char RMDB7;
        } BYTE;
    } RSCAN0RMDF118;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMID:29;
            unsigned long :1;
            unsigned long RMRTR:1;
            unsigned long RMIDE:1;
        } BIT;
    } RSCAN0RMID19;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMTS:16;
            unsigned long RMPTR:12;
            unsigned long RMDLC:4;
        } BIT;
    } RSCAN0RMPTR19;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB0;
            unsigned char RMDB1;
            unsigned char RMDB2;
            unsigned char RMDB3;
        } BYTE;
    } RSCAN0RMDF019;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB4;
            unsigned char RMDB5;
            unsigned char RMDB6;
            unsigned char RMDB7;
        } BYTE;
    } RSCAN0RMDF119;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMID:29;
            unsigned long :1;
            unsigned long RMRTR:1;
            unsigned long RMIDE:1;
        } BIT;
    } RSCAN0RMID20;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMTS:16;
            unsigned long RMPTR:12;
            unsigned long RMDLC:4;
        } BIT;
    } RSCAN0RMPTR20;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB0;
            unsigned char RMDB1;
            unsigned char RMDB2;
            unsigned char RMDB3;
        } BYTE;
    } RSCAN0RMDF020;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB4;
            unsigned char RMDB5;
            unsigned char RMDB6;
            unsigned char RMDB7;
        } BYTE;
    } RSCAN0RMDF120;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMID:29;
            unsigned long :1;
            unsigned long RMRTR:1;
            unsigned long RMIDE:1;
        } BIT;
    } RSCAN0RMID21;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMTS:16;
            unsigned long RMPTR:12;
            unsigned long RMDLC:4;
        } BIT;
    } RSCAN0RMPTR21;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB0;
            unsigned char RMDB1;
            unsigned char RMDB2;
            unsigned char RMDB3;
        } BYTE;
    } RSCAN0RMDF021;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB4;
            unsigned char RMDB5;
            unsigned char RMDB6;
            unsigned char RMDB7;
        } BYTE;
    } RSCAN0RMDF121;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMID:29;
            unsigned long :1;
            unsigned long RMRTR:1;
            unsigned long RMIDE:1;
        } BIT;
    } RSCAN0RMID22;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMTS:16;
            unsigned long RMPTR:12;
            unsigned long RMDLC:4;
        } BIT;
    } RSCAN0RMPTR22;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB0;
            unsigned char RMDB1;
            unsigned char RMDB2;
            unsigned char RMDB3;
        } BYTE;
    } RSCAN0RMDF022;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB4;
            unsigned char RMDB5;
            unsigned char RMDB6;
            unsigned char RMDB7;
        } BYTE;
    } RSCAN0RMDF122;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMID:29;
            unsigned long :1;
            unsigned long RMRTR:1;
            unsigned long RMIDE:1;
        } BIT;
    } RSCAN0RMID23;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMTS:16;
            unsigned long RMPTR:12;
            unsigned long RMDLC:4;
        } BIT;
    } RSCAN0RMPTR23;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB0;
            unsigned char RMDB1;
            unsigned char RMDB2;
            unsigned char RMDB3;
        } BYTE;
    } RSCAN0RMDF023;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB4;
            unsigned char RMDB5;
            unsigned char RMDB6;
            unsigned char RMDB7;
        } BYTE;
    } RSCAN0RMDF123;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMID:29;
            unsigned long :1;
            unsigned long RMRTR:1;
            unsigned long RMIDE:1;
        } BIT;
    } RSCAN0RMID24;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMTS:16;
            unsigned long RMPTR:12;
            unsigned long RMDLC:4;
        } BIT;
    } RSCAN0RMPTR24;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB0;
            unsigned char RMDB1;
            unsigned char RMDB2;
            unsigned char RMDB3;
        } BYTE;
    } RSCAN0RMDF024;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB4;
            unsigned char RMDB5;
            unsigned char RMDB6;
            unsigned char RMDB7;
        } BYTE;
    } RSCAN0RMDF124;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMID:29;
            unsigned long :1;
            unsigned long RMRTR:1;
            unsigned long RMIDE:1;
        } BIT;
    } RSCAN0RMID25;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMTS:16;
            unsigned long RMPTR:12;
            unsigned long RMDLC:4;
        } BIT;
    } RSCAN0RMPTR25;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB0;
            unsigned char RMDB1;
            unsigned char RMDB2;
            unsigned char RMDB3;
        } BYTE;
    } RSCAN0RMDF025;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB4;
            unsigned char RMDB5;
            unsigned char RMDB6;
            unsigned char RMDB7;
        } BYTE;
    } RSCAN0RMDF125;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMID:29;
            unsigned long :1;
            unsigned long RMRTR:1;
            unsigned long RMIDE:1;
        } BIT;
    } RSCAN0RMID26;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMTS:16;
            unsigned long RMPTR:12;
            unsigned long RMDLC:4;
        } BIT;
    } RSCAN0RMPTR26;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB0;
            unsigned char RMDB1;
            unsigned char RMDB2;
            unsigned char RMDB3;
        } BYTE;
    } RSCAN0RMDF026;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB4;
            unsigned char RMDB5;
            unsigned char RMDB6;
            unsigned char RMDB7;
        } BYTE;
    } RSCAN0RMDF126;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMID:29;
            unsigned long :1;
            unsigned long RMRTR:1;
            unsigned long RMIDE:1;
        } BIT;
    } RSCAN0RMID27;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMTS:16;
            unsigned long RMPTR:12;
            unsigned long RMDLC:4;
        } BIT;
    } RSCAN0RMPTR27;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB0;
            unsigned char RMDB1;
            unsigned char RMDB2;
            unsigned char RMDB3;
        } BYTE;
    } RSCAN0RMDF027;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB4;
            unsigned char RMDB5;
            unsigned char RMDB6;
            unsigned char RMDB7;
        } BYTE;
    } RSCAN0RMDF127;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMID:29;
            unsigned long :1;
            unsigned long RMRTR:1;
            unsigned long RMIDE:1;
        } BIT;
    } RSCAN0RMID28;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMTS:16;
            unsigned long RMPTR:12;
            unsigned long RMDLC:4;
        } BIT;
    } RSCAN0RMPTR28;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB0;
            unsigned char RMDB1;
            unsigned char RMDB2;
            unsigned char RMDB3;
        } BYTE;
    } RSCAN0RMDF028;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB4;
            unsigned char RMDB5;
            unsigned char RMDB6;
            unsigned char RMDB7;
        } BYTE;
    } RSCAN0RMDF128;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMID:29;
            unsigned long :1;
            unsigned long RMRTR:1;
            unsigned long RMIDE:1;
        } BIT;
    } RSCAN0RMID29;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMTS:16;
            unsigned long RMPTR:12;
            unsigned long RMDLC:4;
        } BIT;
    } RSCAN0RMPTR29;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB0;
            unsigned char RMDB1;
            unsigned char RMDB2;
            unsigned char RMDB3;
        } BYTE;
    } RSCAN0RMDF029;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB4;
            unsigned char RMDB5;
            unsigned char RMDB6;
            unsigned char RMDB7;
        } BYTE;
    } RSCAN0RMDF129;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMID:29;
            unsigned long :1;
            unsigned long RMRTR:1;
            unsigned long RMIDE:1;
        } BIT;
    } RSCAN0RMID30;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMTS:16;
            unsigned long RMPTR:12;
            unsigned long RMDLC:4;
        } BIT;
    } RSCAN0RMPTR30;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB0;
            unsigned char RMDB1;
            unsigned char RMDB2;
            unsigned char RMDB3;
        } BYTE;
    } RSCAN0RMDF030;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB4;
            unsigned char RMDB5;
            unsigned char RMDB6;
            unsigned char RMDB7;
        } BYTE;
    } RSCAN0RMDF130;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMID:29;
            unsigned long :1;
            unsigned long RMRTR:1;
            unsigned long RMIDE:1;
        } BIT;
    } RSCAN0RMID31;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RMTS:16;
            unsigned long RMPTR:12;
            unsigned long RMDLC:4;
        } BIT;
    } RSCAN0RMPTR31;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB0;
            unsigned char RMDB1;
            unsigned char RMDB2;
            unsigned char RMDB3;
        } BYTE;
    } RSCAN0RMDF031;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RMDB4;
            unsigned char RMDB5;
            unsigned char RMDB6;
            unsigned char RMDB7;
        } BYTE;
    } RSCAN0RMDF131;
    char           wk22[1536];
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RFID:29;
            unsigned long :1;
            unsigned long RFRTR:1;
            unsigned long RFIDE:1;
        } BIT;
    } RSCAN0RFID0;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RFTS:16;
            unsigned long RFPTR:12;
            unsigned long RFDLC:4;
        } BIT;
    } RSCAN0RFPTR0;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RFDB0;
            unsigned char RFDB1;
            unsigned char RFDB2;
            unsigned char RFDB3;
        } BYTE;
    } RSCAN0RFDF00;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RFDB4;
            unsigned char RFDB5;
            unsigned char RFDB6;
            unsigned char RFDB7;
        } BYTE;
    } RSCAN0RFDF10;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RFID:29;
            unsigned long :1;
            unsigned long RFRTR:1;
            unsigned long RFIDE:1;
        } BIT;
    } RSCAN0RFID1;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RFTS:16;
            unsigned long RFPTR:12;
            unsigned long RFDLC:4;
        } BIT;
    } RSCAN0RFPTR1;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RFDB0;
            unsigned char RFDB1;
            unsigned char RFDB2;
            unsigned char RFDB3;
        } BYTE;
    } RSCAN0RFDF01;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RFDB4;
            unsigned char RFDB5;
            unsigned char RFDB6;
            unsigned char RFDB7;
        } BYTE;
    } RSCAN0RFDF11;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RFID:29;
            unsigned long :1;
            unsigned long RFRTR:1;
            unsigned long RFIDE:1;
        } BIT;
    } RSCAN0RFID2;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RFTS:16;
            unsigned long RFPTR:12;
            unsigned long RFDLC:4;
        } BIT;
    } RSCAN0RFPTR2;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RFDB0;
            unsigned char RFDB1;
            unsigned char RFDB2;
            unsigned char RFDB3;
        } BYTE;
    } RSCAN0RFDF02;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RFDB4;
            unsigned char RFDB5;
            unsigned char RFDB6;
            unsigned char RFDB7;
        } BYTE;
    } RSCAN0RFDF12;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RFID:29;
            unsigned long :1;
            unsigned long RFRTR:1;
            unsigned long RFIDE:1;
        } BIT;
    } RSCAN0RFID3;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RFTS:16;
            unsigned long RFPTR:12;
            unsigned long RFDLC:4;
        } BIT;
    } RSCAN0RFPTR3;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RFDB0;
            unsigned char RFDB1;
            unsigned char RFDB2;
            unsigned char RFDB3;
        } BYTE;
    } RSCAN0RFDF03;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RFDB4;
            unsigned char RFDB5;
            unsigned char RFDB6;
            unsigned char RFDB7;
        } BYTE;
    } RSCAN0RFDF13;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RFID:29;
            unsigned long :1;
            unsigned long RFRTR:1;
            unsigned long RFIDE:1;
        } BIT;
    } RSCAN0RFID4;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RFTS:16;
            unsigned long RFPTR:12;
            unsigned long RFDLC:4;
        } BIT;
    } RSCAN0RFPTR4;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RFDB0;
            unsigned char RFDB1;
            unsigned char RFDB2;
            unsigned char RFDB3;
        } BYTE;
    } RSCAN0RFDF04;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RFDB4;
            unsigned char RFDB5;
            unsigned char RFDB6;
            unsigned char RFDB7;
        } BYTE;
    } RSCAN0RFDF14;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RFID:29;
            unsigned long :1;
            unsigned long RFRTR:1;
            unsigned long RFIDE:1;
        } BIT;
    } RSCAN0RFID5;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RFTS:16;
            unsigned long RFPTR:12;
            unsigned long RFDLC:4;
        } BIT;
    } RSCAN0RFPTR5;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RFDB0;
            unsigned char RFDB1;
            unsigned char RFDB2;
            unsigned char RFDB3;
        } BYTE;
    } RSCAN0RFDF05;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RFDB4;
            unsigned char RFDB5;
            unsigned char RFDB6;
            unsigned char RFDB7;
        } BYTE;
    } RSCAN0RFDF15;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RFID:29;
            unsigned long :1;
            unsigned long RFRTR:1;
            unsigned long RFIDE:1;
        } BIT;
    } RSCAN0RFID6;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RFTS:16;
            unsigned long RFPTR:12;
            unsigned long RFDLC:4;
        } BIT;
    } RSCAN0RFPTR6;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RFDB0;
            unsigned char RFDB1;
            unsigned char RFDB2;
            unsigned char RFDB3;
        } BYTE;
    } RSCAN0RFDF06;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RFDB4;
            unsigned char RFDB5;
            unsigned char RFDB6;
            unsigned char RFDB7;
        } BYTE;
    } RSCAN0RFDF16;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RFID:29;
            unsigned long :1;
            unsigned long RFRTR:1;
            unsigned long RFIDE:1;
        } BIT;
    } RSCAN0RFID7;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RFTS:16;
            unsigned long RFPTR:12;
            unsigned long RFDLC:4;
        } BIT;
    } RSCAN0RFPTR7;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RFDB0;
            unsigned char RFDB1;
            unsigned char RFDB2;
            unsigned char RFDB3;
        } BYTE;
    } RSCAN0RFDF07;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char RFDB4;
            unsigned char RFDB5;
            unsigned char RFDB6;
            unsigned char RFDB7;
        } BYTE;
    } RSCAN0RFDF17;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long CFID:29;
            unsigned long THLEN:1;
            unsigned long CFRTR:1;
            unsigned long CFIDE:1;
        } BIT;
    } RSCAN0CFID0;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long CFTS:16;
            unsigned long CFPTR:12;
            unsigned long CFDLC:4;
        } BIT;
    } RSCAN0CFPTR0;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char CFDB0;
            unsigned char CFDB1;
            unsigned char CFDB2;
            unsigned char CFDB3;
        } BYTE;
    } RSCAN0CFDF00;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char CFDB4;
            unsigned char CFDB5;
            unsigned char CFDB6;
            unsigned char CFDB7;
        } BYTE;
    } RSCAN0CFDF10;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long CFID:29;
            unsigned long THLEN:1;
            unsigned long CFRTR:1;
            unsigned long CFIDE:1;
        } BIT;
    } RSCAN0CFID1;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long CFTS:16;
            unsigned long CFPTR:12;
            unsigned long CFDLC:4;
        } BIT;
    } RSCAN0CFPTR1;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char CFDB0;
            unsigned char CFDB1;
            unsigned char CFDB2;
            unsigned char CFDB3;
        } BYTE;
    } RSCAN0CFDF01;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char CFDB4;
            unsigned char CFDB5;
            unsigned char CFDB6;
            unsigned char CFDB7;
        } BYTE;
    } RSCAN0CFDF11;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long CFID:29;
            unsigned long THLEN:1;
            unsigned long CFRTR:1;
            unsigned long CFIDE:1;
        } BIT;
    } RSCAN0CFID2;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long CFTS:16;
            unsigned long CFPTR:12;
            unsigned long CFDLC:4;
        } BIT;
    } RSCAN0CFPTR2;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char CFDB0;
            unsigned char CFDB1;
            unsigned char CFDB2;
            unsigned char CFDB3;
        } BYTE;
    } RSCAN0CFDF02;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char CFDB4;
            unsigned char CFDB5;
            unsigned char CFDB6;
            unsigned char CFDB7;
        } BYTE;
    } RSCAN0CFDF12;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long CFID:29;
            unsigned long THLEN:1;
            unsigned long CFRTR:1;
            unsigned long CFIDE:1;
        } BIT;
    } RSCAN0CFID3;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long CFTS:16;
            unsigned long CFPTR:12;
            unsigned long CFDLC:4;
        } BIT;
    } RSCAN0CFPTR3;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char CFDB0;
            unsigned char CFDB1;
            unsigned char CFDB2;
            unsigned char CFDB3;
        } BYTE;
    } RSCAN0CFDF03;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char CFDB4;
            unsigned char CFDB5;
            unsigned char CFDB6;
            unsigned char CFDB7;
        } BYTE;
    } RSCAN0CFDF13;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long CFID:29;
            unsigned long THLEN:1;
            unsigned long CFRTR:1;
            unsigned long CFIDE:1;
        } BIT;
    } RSCAN0CFID4;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long CFTS:16;
            unsigned long CFPTR:12;
            unsigned long CFDLC:4;
        } BIT;
    } RSCAN0CFPTR4;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char CFDB0;
            unsigned char CFDB1;
            unsigned char CFDB2;
            unsigned char CFDB3;
        } BYTE;
    } RSCAN0CFDF04;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char CFDB4;
            unsigned char CFDB5;
            unsigned char CFDB6;
            unsigned char CFDB7;
        } BYTE;
    } RSCAN0CFDF14;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long CFID:29;
            unsigned long THLEN:1;
            unsigned long CFRTR:1;
            unsigned long CFIDE:1;
        } BIT;
    } RSCAN0CFID5;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long CFTS:16;
            unsigned long CFPTR:12;
            unsigned long CFDLC:4;
        } BIT;
    } RSCAN0CFPTR5;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char CFDB0;
            unsigned char CFDB1;
            unsigned char CFDB2;
            unsigned char CFDB3;
        } BYTE;
    } RSCAN0CFDF05;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char CFDB4;
            unsigned char CFDB5;
            unsigned char CFDB6;
            unsigned char CFDB7;
        } BYTE;
    } RSCAN0CFDF15;
    char           wk23[288];
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long TMID:29;
            unsigned long THLEN:1;
            unsigned long TMRTR:1;
            unsigned long TMIDE:1;
        } BIT;
    } RSCAN0TMID0;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char :8;
            unsigned char :8;
            unsigned char TMPTR:8;
            unsigned char :4;
            unsigned char TMDLC:4;
        } BIT;
    } RSCAN0TMPTR0;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB0;
            unsigned char TMDB1;
            unsigned char TMDB2;
            unsigned char TMDB3;
        } BYTE;
    } RSCAN0TMDF00;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB4;
            unsigned char TMDB5;
            unsigned char TMDB6;
            unsigned char TMDB7;
        } BYTE;
    } RSCAN0TMDF10;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long TMID:29;
            unsigned long THLEN:1;
            unsigned long TMRTR:1;
            unsigned long TMIDE:1;
        } BIT;
    } RSCAN0TMID1;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char :8;
            unsigned char :8;
            unsigned char TMPTR:8;
            unsigned char :4;
            unsigned char TMDLC:4;
        } BIT;
    } RSCAN0TMPTR1;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB0;
            unsigned char TMDB1;
            unsigned char TMDB2;
            unsigned char TMDB3;
        } BYTE;
    } RSCAN0TMDF01;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB4;
            unsigned char TMDB5;
            unsigned char TMDB6;
            unsigned char TMDB7;
        } BYTE;
    } RSCAN0TMDF11;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long TMID:29;
            unsigned long THLEN:1;
            unsigned long TMRTR:1;
            unsigned long TMIDE:1;
        } BIT;
    } RSCAN0TMID2;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char :8;
            unsigned char :8;
            unsigned char TMPTR:8;
            unsigned char :4;
            unsigned char TMDLC:4;
        } BIT;
    } RSCAN0TMPTR2;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB0;
            unsigned char TMDB1;
            unsigned char TMDB2;
            unsigned char TMDB3;
        } BYTE;
    } RSCAN0TMDF02;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB4;
            unsigned char TMDB5;
            unsigned char TMDB6;
            unsigned char TMDB7;
        } BYTE;
    } RSCAN0TMDF12;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long TMID:29;
            unsigned long THLEN:1;
            unsigned long TMRTR:1;
            unsigned long TMIDE:1;
        } BIT;
    } RSCAN0TMID3;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char :8;
            unsigned char :8;
            unsigned char TMPTR:8;
            unsigned char :4;
            unsigned char TMDLC:4;
        } BIT;
    } RSCAN0TMPTR3;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB0;
            unsigned char TMDB1;
            unsigned char TMDB2;
            unsigned char TMDB3;
        } BYTE;
    } RSCAN0TMDF03;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB4;
            unsigned char TMDB5;
            unsigned char TMDB6;
            unsigned char TMDB7;
        } BYTE;
    } RSCAN0TMDF13;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long TMID:29;
            unsigned long THLEN:1;
            unsigned long TMRTR:1;
            unsigned long TMIDE:1;
        } BIT;
    } RSCAN0TMID4;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char :8;
            unsigned char :8;
            unsigned char TMPTR:8;
            unsigned char :4;
            unsigned char TMDLC:4;
        } BIT;
    } RSCAN0TMPTR4;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB0;
            unsigned char TMDB1;
            unsigned char TMDB2;
            unsigned char TMDB3;
        } BYTE;
    } RSCAN0TMDF04;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB4;
            unsigned char TMDB5;
            unsigned char TMDB6;
            unsigned char TMDB7;
        } BYTE;
    } RSCAN0TMDF14;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long TMID:29;
            unsigned long THLEN:1;
            unsigned long TMRTR:1;
            unsigned long TMIDE:1;
        } BIT;
    } RSCAN0TMID5;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char :8;
            unsigned char :8;
            unsigned char TMPTR:8;
            unsigned char :4;
            unsigned char TMDLC:4;
        } BIT;
    } RSCAN0TMPTR5;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB0;
            unsigned char TMDB1;
            unsigned char TMDB2;
            unsigned char TMDB3;
        } BYTE;
    } RSCAN0TMDF05;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB4;
            unsigned char TMDB5;
            unsigned char TMDB6;
            unsigned char TMDB7;
        } BYTE;
    } RSCAN0TMDF15;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long TMID:29;
            unsigned long THLEN:1;
            unsigned long TMRTR:1;
            unsigned long TMIDE:1;
        } BIT;
    } RSCAN0TMID6;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char :8;
            unsigned char :8;
            unsigned char TMPTR:8;
            unsigned char :4;
            unsigned char TMDLC:4;
        } BIT;
    } RSCAN0TMPTR6;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB0;
            unsigned char TMDB1;
            unsigned char TMDB2;
            unsigned char TMDB3;
        } BYTE;
    } RSCAN0TMDF06;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB4;
            unsigned char TMDB5;
            unsigned char TMDB6;
            unsigned char TMDB7;
        } BYTE;
    } RSCAN0TMDF16;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long TMID:29;
            unsigned long THLEN:1;
            unsigned long TMRTR:1;
            unsigned long TMIDE:1;
        } BIT;
    } RSCAN0TMID7;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char :8;
            unsigned char :8;
            unsigned char TMPTR:8;
            unsigned char :4;
            unsigned char TMDLC:4;
        } BIT;
    } RSCAN0TMPTR7;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB0;
            unsigned char TMDB1;
            unsigned char TMDB2;
            unsigned char TMDB3;
        } BYTE;
    } RSCAN0TMDF07;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB4;
            unsigned char TMDB5;
            unsigned char TMDB6;
            unsigned char TMDB7;
        } BYTE;
    } RSCAN0TMDF17;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long TMID:29;
            unsigned long THLEN:1;
            unsigned long TMRTR:1;
            unsigned long TMIDE:1;
        } BIT;
    } RSCAN0TMID8;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char :8;
            unsigned char :8;
            unsigned char TMPTR:8;
            unsigned char :4;
            unsigned char TMDLC:4;
        } BIT;
    } RSCAN0TMPTR8;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB0;
            unsigned char TMDB1;
            unsigned char TMDB2;
            unsigned char TMDB3;
        } BYTE;
    } RSCAN0TMDF08;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB4;
            unsigned char TMDB5;
            unsigned char TMDB6;
            unsigned char TMDB7;
        } BYTE;
    } RSCAN0TMDF18;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long TMID:29;
            unsigned long THLEN:1;
            unsigned long TMRTR:1;
            unsigned long TMIDE:1;
        } BIT;
    } RSCAN0TMID9;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char :8;
            unsigned char :8;
            unsigned char TMPTR:8;
            unsigned char :4;
            unsigned char TMDLC:4;
        } BIT;
    } RSCAN0TMPTR9;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB0;
            unsigned char TMDB1;
            unsigned char TMDB2;
            unsigned char TMDB3;
        } BYTE;
    } RSCAN0TMDF09;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB4;
            unsigned char TMDB5;
            unsigned char TMDB6;
            unsigned char TMDB7;
        } BYTE;
    } RSCAN0TMDF19;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long TMID:29;
            unsigned long THLEN:1;
            unsigned long TMRTR:1;
            unsigned long TMIDE:1;
        } BIT;
    } RSCAN0TMID10;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char :8;
            unsigned char :8;
            unsigned char TMPTR:8;
            unsigned char :4;
            unsigned char TMDLC:4;
        } BIT;
    } RSCAN0TMPTR10;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB0;
            unsigned char TMDB1;
            unsigned char TMDB2;
            unsigned char TMDB3;
        } BYTE;
    } RSCAN0TMDF010;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB4;
            unsigned char TMDB5;
            unsigned char TMDB6;
            unsigned char TMDB7;
        } BYTE;
    } RSCAN0TMDF110;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long TMID:29;
            unsigned long THLEN:1;
            unsigned long TMRTR:1;
            unsigned long TMIDE:1;
        } BIT;
    } RSCAN0TMID11;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char :8;
            unsigned char :8;
            unsigned char TMPTR:8;
            unsigned char :4;
            unsigned char TMDLC:4;
        } BIT;
    } RSCAN0TMPTR11;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB0;
            unsigned char TMDB1;
            unsigned char TMDB2;
            unsigned char TMDB3;
        } BYTE;
    } RSCAN0TMDF011;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB4;
            unsigned char TMDB5;
            unsigned char TMDB6;
            unsigned char TMDB7;
        } BYTE;
    } RSCAN0TMDF111;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long TMID:29;
            unsigned long THLEN:1;
            unsigned long TMRTR:1;
            unsigned long TMIDE:1;
        } BIT;
    } RSCAN0TMID12;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char :8;
            unsigned char :8;
            unsigned char TMPTR:8;
            unsigned char :4;
            unsigned char TMDLC:4;
        } BIT;
    } RSCAN0TMPTR12;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB0;
            unsigned char TMDB1;
            unsigned char TMDB2;
            unsigned char TMDB3;
        } BYTE;
    } RSCAN0TMDF012;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB4;
            unsigned char TMDB5;
            unsigned char TMDB6;
            unsigned char TMDB7;
        } BYTE;
    } RSCAN0TMDF112;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long TMID:29;
            unsigned long THLEN:1;
            unsigned long TMRTR:1;
            unsigned long TMIDE:1;
        } BIT;
    } RSCAN0TMID13;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char :8;
            unsigned char :8;
            unsigned char TMPTR:8;
            unsigned char :4;
            unsigned char TMDLC:4;
        } BIT;
    } RSCAN0TMPTR13;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB0;
            unsigned char TMDB1;
            unsigned char TMDB2;
            unsigned char TMDB3;
        } BYTE;
    } RSCAN0TMDF013;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB4;
            unsigned char TMDB5;
            unsigned char TMDB6;
            unsigned char TMDB7;
        } BYTE;
    } RSCAN0TMDF113;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long TMID:29;
            unsigned long THLEN:1;
            unsigned long TMRTR:1;
            unsigned long TMIDE:1;
        } BIT;
    } RSCAN0TMID14;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char :8;
            unsigned char :8;
            unsigned char TMPTR:8;
            unsigned char :4;
            unsigned char TMDLC:4;
        } BIT;
    } RSCAN0TMPTR14;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB0;
            unsigned char TMDB1;
            unsigned char TMDB2;
            unsigned char TMDB3;
        } BYTE;
    } RSCAN0TMDF014;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB4;
            unsigned char TMDB5;
            unsigned char TMDB6;
            unsigned char TMDB7;
        } BYTE;
    } RSCAN0TMDF114;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long TMID:29;
            unsigned long THLEN:1;
            unsigned long TMRTR:1;
            unsigned long TMIDE:1;
        } BIT;
    } RSCAN0TMID15;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char :8;
            unsigned char :8;
            unsigned char TMPTR:8;
            unsigned char :4;
            unsigned char TMDLC:4;
        } BIT;
    } RSCAN0TMPTR15;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB0;
            unsigned char TMDB1;
            unsigned char TMDB2;
            unsigned char TMDB3;
        } BYTE;
    } RSCAN0TMDF015;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB4;
            unsigned char TMDB5;
            unsigned char TMDB6;
            unsigned char TMDB7;
        } BYTE;
    } RSCAN0TMDF115;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long TMID:29;
            unsigned long THLEN:1;
            unsigned long TMRTR:1;
            unsigned long TMIDE:1;
        } BIT;
    } RSCAN0TMID16;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char :8;
            unsigned char :8;
            unsigned char TMPTR:8;
            unsigned char :4;
            unsigned char TMDLC:4;
        } BIT;
    } RSCAN0TMPTR16;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB0;
            unsigned char TMDB1;
            unsigned char TMDB2;
            unsigned char TMDB3;
        } BYTE;
    } RSCAN0TMDF016;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB4;
            unsigned char TMDB5;
            unsigned char TMDB6;
            unsigned char TMDB7;
        } BYTE;
    } RSCAN0TMDF116;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long TMID:29;
            unsigned long THLEN:1;
            unsigned long TMRTR:1;
            unsigned long TMIDE:1;
        } BIT;
    } RSCAN0TMID17;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char :8;
            unsigned char :8;
            unsigned char TMPTR:8;
            unsigned char :4;
            unsigned char TMDLC:4;
        } BIT;
    } RSCAN0TMPTR17;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB0;
            unsigned char TMDB1;
            unsigned char TMDB2;
            unsigned char TMDB3;
        } BYTE;
    } RSCAN0TMDF017;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB4;
            unsigned char TMDB5;
            unsigned char TMDB6;
            unsigned char TMDB7;
        } BYTE;
    } RSCAN0TMDF117;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long TMID:29;
            unsigned long THLEN:1;
            unsigned long TMRTR:1;
            unsigned long TMIDE:1;
        } BIT;
    } RSCAN0TMID18;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char :8;
            unsigned char :8;
            unsigned char TMPTR:8;
            unsigned char :4;
            unsigned char TMDLC:4;
        } BIT;
    } RSCAN0TMPTR18;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB0;
            unsigned char TMDB1;
            unsigned char TMDB2;
            unsigned char TMDB3;
        } BYTE;
    } RSCAN0TMDF018;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB4;
            unsigned char TMDB5;
            unsigned char TMDB6;
            unsigned char TMDB7;
        } BYTE;
    } RSCAN0TMDF118;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long TMID:29;
            unsigned long THLEN:1;
            unsigned long TMRTR:1;
            unsigned long TMIDE:1;
        } BIT;
    } RSCAN0TMID19;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char :8;
            unsigned char :8;
            unsigned char TMPTR:8;
            unsigned char :4;
            unsigned char TMDLC:4;
        } BIT;
    } RSCAN0TMPTR19;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB0;
            unsigned char TMDB1;
            unsigned char TMDB2;
            unsigned char TMDB3;
        } BYTE;
    } RSCAN0TMDF019;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB4;
            unsigned char TMDB5;
            unsigned char TMDB6;
            unsigned char TMDB7;
        } BYTE;
    } RSCAN0TMDF119;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long TMID:29;
            unsigned long THLEN:1;
            unsigned long TMRTR:1;
            unsigned long TMIDE:1;
        } BIT;
    } RSCAN0TMID20;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char :8;
            unsigned char :8;
            unsigned char TMPTR:8;
            unsigned char :4;
            unsigned char TMDLC:4;
        } BIT;
    } RSCAN0TMPTR20;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB0;
            unsigned char TMDB1;
            unsigned char TMDB2;
            unsigned char TMDB3;
        } BYTE;
    } RSCAN0TMDF020;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB4;
            unsigned char TMDB5;
            unsigned char TMDB6;
            unsigned char TMDB7;
        } BYTE;
    } RSCAN0TMDF120;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long TMID:29;
            unsigned long THLEN:1;
            unsigned long TMRTR:1;
            unsigned long TMIDE:1;
        } BIT;
    } RSCAN0TMID21;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char :8;
            unsigned char :8;
            unsigned char TMPTR:8;
            unsigned char :4;
            unsigned char TMDLC:4;
        } BIT;
    } RSCAN0TMPTR21;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB0;
            unsigned char TMDB1;
            unsigned char TMDB2;
            unsigned char TMDB3;
        } BYTE;
    } RSCAN0TMDF021;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB4;
            unsigned char TMDB5;
            unsigned char TMDB6;
            unsigned char TMDB7;
        } BYTE;
    } RSCAN0TMDF121;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long TMID:29;
            unsigned long THLEN:1;
            unsigned long TMRTR:1;
            unsigned long TMIDE:1;
        } BIT;
    } RSCAN0TMID22;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char :8;
            unsigned char :8;
            unsigned char TMPTR:8;
            unsigned char :4;
            unsigned char TMDLC:4;
        } BIT;
    } RSCAN0TMPTR22;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB0;
            unsigned char TMDB1;
            unsigned char TMDB2;
            unsigned char TMDB3;
        } BYTE;
    } RSCAN0TMDF022;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB4;
            unsigned char TMDB5;
            unsigned char TMDB6;
            unsigned char TMDB7;
        } BYTE;
    } RSCAN0TMDF122;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long TMID:29;
            unsigned long THLEN:1;
            unsigned long TMRTR:1;
            unsigned long TMIDE:1;
        } BIT;
    } RSCAN0TMID23;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char :8;
            unsigned char :8;
            unsigned char TMPTR:8;
            unsigned char :4;
            unsigned char TMDLC:4;
        } BIT;
    } RSCAN0TMPTR23;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB0;
            unsigned char TMDB1;
            unsigned char TMDB2;
            unsigned char TMDB3;
        } BYTE;
    } RSCAN0TMDF023;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB4;
            unsigned char TMDB5;
            unsigned char TMDB6;
            unsigned char TMDB7;
        } BYTE;
    } RSCAN0TMDF123;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long TMID:29;
            unsigned long THLEN:1;
            unsigned long TMRTR:1;
            unsigned long TMIDE:1;
        } BIT;
    } RSCAN0TMID24;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char :8;
            unsigned char :8;
            unsigned char TMPTR:8;
            unsigned char :4;
            unsigned char TMDLC:4;
        } BIT;
    } RSCAN0TMPTR24;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB0;
            unsigned char TMDB1;
            unsigned char TMDB2;
            unsigned char TMDB3;
        } BYTE;
    } RSCAN0TMDF024;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB4;
            unsigned char TMDB5;
            unsigned char TMDB6;
            unsigned char TMDB7;
        } BYTE;
    } RSCAN0TMDF124;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long TMID:29;
            unsigned long THLEN:1;
            unsigned long TMRTR:1;
            unsigned long TMIDE:1;
        } BIT;
    } RSCAN0TMID25;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char :8;
            unsigned char :8;
            unsigned char TMPTR:8;
            unsigned char :4;
            unsigned char TMDLC:4;
        } BIT;
    } RSCAN0TMPTR25;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB0;
            unsigned char TMDB1;
            unsigned char TMDB2;
            unsigned char TMDB3;
        } BYTE;
    } RSCAN0TMDF025;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB4;
            unsigned char TMDB5;
            unsigned char TMDB6;
            unsigned char TMDB7;
        } BYTE;
    } RSCAN0TMDF125;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long TMID:29;
            unsigned long THLEN:1;
            unsigned long TMRTR:1;
            unsigned long TMIDE:1;
        } BIT;
    } RSCAN0TMID26;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char :8;
            unsigned char :8;
            unsigned char TMPTR:8;
            unsigned char :4;
            unsigned char TMDLC:4;
        } BIT;
    } RSCAN0TMPTR26;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB0;
            unsigned char TMDB1;
            unsigned char TMDB2;
            unsigned char TMDB3;
        } BYTE;
    } RSCAN0TMDF026;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB4;
            unsigned char TMDB5;
            unsigned char TMDB6;
            unsigned char TMDB7;
        } BYTE;
    } RSCAN0TMDF126;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long TMID:29;
            unsigned long THLEN:1;
            unsigned long TMRTR:1;
            unsigned long TMIDE:1;
        } BIT;
    } RSCAN0TMID27;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char :8;
            unsigned char :8;
            unsigned char TMPTR:8;
            unsigned char :4;
            unsigned char TMDLC:4;
        } BIT;
    } RSCAN0TMPTR27;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB0;
            unsigned char TMDB1;
            unsigned char TMDB2;
            unsigned char TMDB3;
        } BYTE;
    } RSCAN0TMDF027;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB4;
            unsigned char TMDB5;
            unsigned char TMDB6;
            unsigned char TMDB7;
        } BYTE;
    } RSCAN0TMDF127;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long TMID:29;
            unsigned long THLEN:1;
            unsigned long TMRTR:1;
            unsigned long TMIDE:1;
        } BIT;
    } RSCAN0TMID28;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char :8;
            unsigned char :8;
            unsigned char TMPTR:8;
            unsigned char :4;
            unsigned char TMDLC:4;
        } BIT;
    } RSCAN0TMPTR28;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB0;
            unsigned char TMDB1;
            unsigned char TMDB2;
            unsigned char TMDB3;
        } BYTE;
    } RSCAN0TMDF028;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB4;
            unsigned char TMDB5;
            unsigned char TMDB6;
            unsigned char TMDB7;
        } BYTE;
    } RSCAN0TMDF128;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long TMID:29;
            unsigned long THLEN:1;
            unsigned long TMRTR:1;
            unsigned long TMIDE:1;
        } BIT;
    } RSCAN0TMID29;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char :8;
            unsigned char :8;
            unsigned char TMPTR:8;
            unsigned char :4;
            unsigned char TMDLC:4;
        } BIT;
    } RSCAN0TMPTR29;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB0;
            unsigned char TMDB1;
            unsigned char TMDB2;
            unsigned char TMDB3;
        } BYTE;
    } RSCAN0TMDF029;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB4;
            unsigned char TMDB5;
            unsigned char TMDB6;
            unsigned char TMDB7;
        } BYTE;
    } RSCAN0TMDF129;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long TMID:29;
            unsigned long THLEN:1;
            unsigned long TMRTR:1;
            unsigned long TMIDE:1;
        } BIT;
    } RSCAN0TMID30;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char :8;
            unsigned char :8;
            unsigned char TMPTR:8;
            unsigned char :4;
            unsigned char TMDLC:4;
        } BIT;
    } RSCAN0TMPTR30;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB0;
            unsigned char TMDB1;
            unsigned char TMDB2;
            unsigned char TMDB3;
        } BYTE;
    } RSCAN0TMDF030;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB4;
            unsigned char TMDB5;
            unsigned char TMDB6;
            unsigned char TMDB7;
        } BYTE;
    } RSCAN0TMDF130;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long TMID:29;
            unsigned long THLEN:1;
            unsigned long TMRTR:1;
            unsigned long TMIDE:1;
        } BIT;
    } RSCAN0TMID31;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char :8;
            unsigned char :8;
            unsigned char TMPTR:8;
            unsigned char :4;
            unsigned char TMDLC:4;
        } BIT;
    } RSCAN0TMPTR31;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB0;
            unsigned char TMDB1;
            unsigned char TMDB2;
            unsigned char TMDB3;
        } BYTE;
    } RSCAN0TMDF031;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char TMDB4;
            unsigned char TMDB5;
            unsigned char TMDB6;
            unsigned char TMDB7;
        } BYTE;
    } RSCAN0TMDF131;
    char           wk24[1536];
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char BT:3;
            unsigned char BN:4;
            unsigned char :1;
            unsigned char TID:8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0THLACC0;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char BT:3;
            unsigned char BN:4;
            unsigned char :1;
            unsigned char TID:8;
            unsigned char :8;
            unsigned char :8;
        } BIT;
    } RSCAN0THLACC1;
    char           wk25[248];
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC0;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC1;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC2;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC3;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC4;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC5;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC6;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC7;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC8;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC9;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC10;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC11;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC12;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC13;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC14;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC15;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC16;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC17;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC18;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC19;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC20;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC21;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC22;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC23;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC24;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC25;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC26;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC27;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC28;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC29;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC30;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC31;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC32;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC33;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC34;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC35;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC36;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC37;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC38;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC39;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC40;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC41;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC42;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC43;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC44;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC45;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC46;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC47;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC48;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC49;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC50;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC51;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC52;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC53;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC54;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC55;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC56;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC57;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC58;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC59;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC60;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC61;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC62;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDTA:32;
        } BIT;
    } RSCAN0RPGACC63;
    char           wk26[5632];
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned char ECEMF:1;
            unsigned char ECER1F:1;
            unsigned char ECER2F:1;
            unsigned char EC1EDIC:1;
            unsigned char EC2EDIC:1;
            unsigned char EC1ECP:1;
            unsigned char ECERVF:1;
            unsigned char ECTHM:1;
            unsigned char :1;
            unsigned char ECER1C:1;
            unsigned char ECER2C:1;
            unsigned char ECOVFF:1;
            unsigned char :2;
            unsigned char EMCA0:1;
            unsigned char EMCA1:1;
            unsigned char ECSEDF0:1;
            unsigned char ECDEDF0:1;
            unsigned char ECSEDF1:1;
            unsigned char ECDEDF1:1;
            unsigned char ECSEDF2:1;
            unsigned char ECDEDF2:1;
            unsigned char ECSEDF3:1;
            unsigned char ECDEDF3:1;
            unsigned char ECSEDF4:1;
            unsigned char ECDEDF4:1;
            unsigned char ECSEDF5:1;
            unsigned char ECDEDF5:1;
            unsigned char ECSEDF6:1;
            unsigned char ECDEDF6:1;
            unsigned char ECSEDF7:1;
            unsigned char ECDEDF7:1;
        } BIT;
    } ECCRCANCTL;
    char           wk27[12];
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long ECEADz:11;
            unsigned long :21;
        } BIT;
    } ECCRCANEAD0;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long ECEADz:11;
            unsigned long :21;
        } BIT;
    } ECCRCANEAD1;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long ECEADz:11;
            unsigned long :21;
        } BIT;
    } ECCRCANEAD2;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long ECEADz:11;
            unsigned long :21;
        } BIT;
    } ECCRCANEAD3;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long ECEADz:11;
            unsigned long :21;
        } BIT;
    } ECCRCANEAD4;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long ECEADz:11;
            unsigned long :21;
        } BIT;
    } ECCRCANEAD5;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long ECEADz:11;
            unsigned long :21;
        } BIT;
    } ECCRCANEAD6;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long ECEADz:11;
            unsigned long :21;
        } BIT;
    } ECCRCANEAD7;
};

struct st_rspi {
    union {
        unsigned char BYTE;
        struct {
            unsigned char SPMS:1;
            unsigned char TXMD:1;
            unsigned char MODFEN:1;
            unsigned char MSTR:1;
            unsigned char SPEIE:1;
            unsigned char SPTIE:1;
            unsigned char SPE:1;
            unsigned char SPRIE:1;
        } BIT;
    } SPCR;
    union {
        unsigned char BYTE;
        struct {
            unsigned char SSL0P:1;
            unsigned char SSL1P:1;
            unsigned char SSL2P:1;
            unsigned char SSL3P:1;
            unsigned char :4;
        } BIT;
    } SSLP;
    union {
        unsigned char BYTE;
        struct {
            unsigned char SPLP:1;
            unsigned char SPLP2:1;
            unsigned char SPOM:1;
            unsigned char :1;
            unsigned char MOIFV:1;
            unsigned char MOIFE:1;
            unsigned char :2;
        } BIT;
    } SPPCR;
    union {
        unsigned char BYTE;
        struct {
            unsigned char OVRF:1;
            unsigned char IDLNF:1;
            unsigned char MODF:1;
            unsigned char PERF:1;
            unsigned char :4;
        } BIT;
    } SPSR;
    union {
        unsigned long LONG;
        struct {
            unsigned short L;
            unsigned short H;
        } WORD;
    } SPDR;
    union {
        unsigned char BYTE;
        struct {
            unsigned char SPSLN:3;
            unsigned char :5;
        } BIT;
    } SPSCR;
    union {
        unsigned char BYTE;
        struct {
            unsigned char SPCP:3;
            unsigned char :1;
            unsigned char SPECM:3;
            unsigned char :1;
        } BIT;
    } SPSSR;
    unsigned char  SPBR;
    union {
        unsigned char BYTE;
        struct {
            unsigned char SPFC:2;
            unsigned char :2;
            unsigned char SPRDTD:1;
            unsigned char SPLW:1;
            unsigned char :2;
        } BIT;
    } SPDCR;
    union {
        unsigned char BYTE;
        struct {
            unsigned char SCKDL:3;
            unsigned char :5;
        } BIT;
    } SPCKD;
    union {
        unsigned char BYTE;
        struct {
            unsigned char SLNDL:3;
            unsigned char :5;
        } BIT;
    } SSLND;
    union {
        unsigned char BYTE;
        struct {
            unsigned char SPNDL:3;
            unsigned char :5;
        } BIT;
    } SPND;
    union {
        unsigned char BYTE;
        struct {
            unsigned char SPPE:1;
            unsigned char SPOE:1;
            unsigned char SPIIE:1;
            unsigned char PTE:1;
            unsigned char SCKASE:1;
            unsigned char :3;
        } BIT;
    } SPCR2;
    union {
        unsigned short WORD;
        struct {
            unsigned short CPHA:1;
            unsigned short CPOL:1;
            unsigned short BRDV:2;
            unsigned short SSLy:3;
            unsigned short SSLKP:1;
            unsigned short SPB:4;
            unsigned short LSBF:1;
            unsigned short SPNDEN:1;
            unsigned short SLNDEN:1;
            unsigned short SCKDEN:1;
        } BIT;
    } SPCMD0;
    union {
        unsigned short WORD;
        struct {
            unsigned short CPHA:1;
            unsigned short CPOL:1;
            unsigned short BRDV:2;
            unsigned short SSLy:3;
            unsigned short SSLKP:1;
            unsigned short SPB:4;
            unsigned short LSBF:1;
            unsigned short SPNDEN:1;
            unsigned short SLNDEN:1;
            unsigned short SCKDEN:1;
        } BIT;
    } SPCMD1;
    union {
        unsigned short WORD;
        struct {
            unsigned short CPHA:1;
            unsigned short CPOL:1;
            unsigned short BRDV:2;
            unsigned short SSLy:3;
            unsigned short SSLKP:1;
            unsigned short SPB:4;
            unsigned short LSBF:1;
            unsigned short SPNDEN:1;
            unsigned short SLNDEN:1;
            unsigned short SCKDEN:1;
        } BIT;
    } SPCMD2;
    union {
        unsigned short WORD;
        struct {
            unsigned short CPHA:1;
            unsigned short CPOL:1;
            unsigned short BRDV:2;
            unsigned short SSLy:3;
            unsigned short SSLKP:1;
            unsigned short SPB:4;
            unsigned short LSBF:1;
            unsigned short SPNDEN:1;
            unsigned short SLNDEN:1;
            unsigned short SCKDEN:1;
        } BIT;
    } SPCMD3;
    union {
        unsigned short WORD;
        struct {
            unsigned short CPHA:1;
            unsigned short CPOL:1;
            unsigned short BRDV:2;
            unsigned short SSLy:3;
            unsigned short SSLKP:1;
            unsigned short SPB:4;
            unsigned short LSBF:1;
            unsigned short SPNDEN:1;
            unsigned short SLNDEN:1;
            unsigned short SCKDEN:1;
        } BIT;
    } SPCMD4;
    union {
        unsigned short WORD;
        struct {
            unsigned short CPHA:1;
            unsigned short CPOL:1;
            unsigned short BRDV:2;
            unsigned short SSLy:3;
            unsigned short SSLKP:1;
            unsigned short SPB:4;
            unsigned short LSBF:1;
            unsigned short SPNDEN:1;
            unsigned short SLNDEN:1;
            unsigned short SCKDEN:1;
        } BIT;
    } SPCMD5;
    union {
        unsigned short WORD;
        struct {
            unsigned short CPHA:1;
            unsigned short CPOL:1;
            unsigned short BRDV:2;
            unsigned short SSLy:3;
            unsigned short SSLKP:1;
            unsigned short SPB:4;
            unsigned short LSBF:1;
            unsigned short SPNDEN:1;
            unsigned short SLNDEN:1;
            unsigned short SCKDEN:1;
        } BIT;
    } SPCMD6;
    union {
        unsigned short WORD;
        struct {
            unsigned short CPHA:1;
            unsigned short CPOL:1;
            unsigned short BRDV:2;
            unsigned short SSLy:3;
            unsigned short SSLKP:1;
            unsigned short SPB:4;
            unsigned short LSBF:1;
            unsigned short SPNDEN:1;
            unsigned short SLNDEN:1;
            unsigned short SCKDEN:1;
        } BIT;
    } SPCMD7;
};

struct st_s12adc0 {
    union {
        unsigned short WORD;
        struct {
            unsigned short DBLANS:5;
            unsigned short :1;
            unsigned short GBADIE:1;
            unsigned short DBLE:1;
            unsigned short EXTRG:1;
            unsigned short TRGE:1;
            unsigned short :2;
            unsigned short ADIE:1;
            unsigned short ADCS:2;
            unsigned short ADST:1;
        } BIT;
    } ADCSR;
    char           wk0[2];
    union {
        unsigned short WORD;
        struct {
            unsigned short ANSA:16;
        } BIT;
    } ADANSA;
    char           wk1[2];
    union {
        unsigned short WORD;
        struct {
            unsigned short ADS:16;
        } BIT;
    } ADADS;
    char           wk2[2];
    union {
        unsigned char BYTE;
        struct {
            unsigned char ADC:2;
            unsigned char :5;
            unsigned char AVEE:1;
        } BIT;
    } ADADC;
    char           wk3[1];
    union {
        unsigned short WORD;
        struct {
            unsigned short :1;
            unsigned short ADPRC:2;
            unsigned short :2;
            unsigned short ACE:1;
            unsigned short :2;
            unsigned short DIAGVAL:2;
            unsigned short DIAGLD:1;
            unsigned short DIAGM:1;
            unsigned short :3;
            unsigned short ADRFMT:1;
        } BIT;
    } ADCER;
    union {
        unsigned short WORD;
        struct {
            unsigned short TRSB:6;
            unsigned short :2;
            unsigned short TRSA:6;
            unsigned short :2;
        } BIT;
    } ADSTRGR;
    union {
        unsigned short WORD;
        struct {
            unsigned short TSSAD:1;
            unsigned short :7;
            unsigned short TSSA:1;
            unsigned short :1;
            unsigned short TSSB:1;
            unsigned short :5;
        } BIT;
    } ADEXICR;
    union {
        unsigned short WORD;
        struct {
            unsigned short ANSB:16;
        } BIT;
    } ADANSB;
    char           wk4[2];
    unsigned short ADDBLDR;
    unsigned short ADTSDR;
    char           wk5[2];
    unsigned short ADRD;
    unsigned short ADDR0;
    unsigned short ADDR1;
    unsigned short ADDR2;
    unsigned short ADDR3;
    unsigned short ADDR4;
    unsigned short ADDR5;
    unsigned short ADDR6;
    unsigned short ADDR7;
    char           wk6[48];
    union {
        unsigned char BYTE;
        struct {
            unsigned char SST:8;
        } BIT;
    } ADSSTR0;
    char           wk7[5];
    union {
        unsigned short WORD;
        struct {
            unsigned short SSTSH:8;
            unsigned short SHANS:4;
            unsigned short :4;
        } BIT;
    } ADSHCR;
    char           wk8[8];
    union {
        unsigned char BYTE;
        struct {
            unsigned char SST:8;
        } BIT;
    } ADSSTRT;
    char           wk9[2];
    union {
        unsigned char BYTE;
        struct {
            unsigned char SST:8;
        } BIT;
    } ADSSTR1;
    union {
        unsigned char BYTE;
        struct {
            unsigned char SST:8;
        } BIT;
    } ADSSTR2;
    union {
        unsigned char BYTE;
        struct {
            unsigned char SST:8;
        } BIT;
    } ADSSTR3;
    union {
        unsigned char BYTE;
        struct {
            unsigned char SST:8;
        } BIT;
    } ADSSTR4;
    union {
        unsigned char BYTE;
        struct {
            unsigned char SST:8;
        } BIT;
    } ADSSTR5;
    union {
        unsigned char BYTE;
        struct {
            unsigned char SST:8;
        } BIT;
    } ADSSTR6;
    union {
        unsigned char BYTE;
        struct {
            unsigned char SST:8;
        } BIT;
    } ADSSTR7;
    union {
        unsigned char BYTE;
        struct {
            unsigned char ADNDIS:5;
            unsigned char :3;
        } BIT;
    } ADDISCR;
    char           wk10[5];
    union {
        unsigned short WORD;
        struct {
            unsigned short PGS:1;
            unsigned short GBRSCN:1;
            unsigned short :13;
            unsigned short GBRP:1;
        } BIT;
    } ADGSPCR;
    char           wk11[2];
    unsigned short ADDBLDRA;
    unsigned short ADDBLDRB;
    char           wk12[8];
    union {
        unsigned char BYTE;
        struct {
            unsigned char :6;
            unsigned char WCMPE:1;
            unsigned char CMPIE:1;
        } BIT;
    } ADCMPCR;
    char           wk13[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char CMPSTS:1;
            unsigned char :7;
        } BIT;
    } ADCMPANSER;
    union {
        unsigned char BYTE;
        struct {
            unsigned char CMPLTS:1;
            unsigned char :7;
        } BIT;
    } ADCMPLER;
    union {
        unsigned short WORD;
        struct {
            unsigned short CMPS:16;
        } BIT;
    } ADCMPANSR;
    char           wk14[2];
    union {
        unsigned short WORD;
        struct {
            unsigned short CMPL:16;
        } BIT;
    } ADCMPLR;
    char           wk15[2];
    unsigned short ADCMPDR0;
    unsigned short ADCMPDR1;
    union {
        unsigned short WORD;
        struct {
            unsigned short CMPF:16;
        } BIT;
    } ADCMPSR;
    char           wk16[2];
    union {
        unsigned char BYTE;
        struct {
            unsigned char CMPFTS:1;
            unsigned char :7;
        } BIT;
    } ADCMPSER;
    char           wk17[35];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TDLV:2;
            unsigned char :5;
            unsigned char TDE:1;
        } BIT;
    } ADTDCR;
    char           wk18[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char :2;
            unsigned char OWEIE:1;
            unsigned char :5;
        } BIT;
    } ADERCR;
    union {
        unsigned char BYTE;
        struct {
            unsigned char :2;
            unsigned char OWEC:1;
            unsigned char :5;
        } BIT;
    } ADERCLR;
    char           wk19[6];
    union {
        unsigned short WORD;
        struct {
            unsigned short OWE:16;
        } BIT;
    } ADOWER;
    char           wk20[2];
    union {
        unsigned short WORD;
        struct {
            unsigned short DBOWE:1;
            unsigned short DAOWE:1;
            unsigned short DOWE:1;
            unsigned short DIAGOWE:1;
            unsigned short TSOWE:1;
            unsigned short :11;
        } BIT;
    } ADOWEER;
};

struct st_s12adc1 {
    union {
        unsigned short WORD;
        struct {
            unsigned short DBLANS:5;
            unsigned short :1;
            unsigned short GBADIE:1;
            unsigned short DBLE:1;
            unsigned short EXTRG:1;
            unsigned short TRGE:1;
            unsigned short :2;
            unsigned short ADIE:1;
            unsigned short ADCS:2;
            unsigned short ADST:1;
        } BIT;
    } ADCSR;
    char           wk0[2];
    union {
        unsigned short WORD;
        struct {
            unsigned short ANSA:16;
        } BIT;
    } ADANSA;
    char           wk1[2];
    union {
        unsigned short WORD;
        struct {
            unsigned short ADS:16;
        } BIT;
    } ADADS;
    char           wk2[2];
    union {
        unsigned char BYTE;
        struct {
            unsigned char ADC:2;
            unsigned char :5;
            unsigned char AVEE:1;
        } BIT;
    } ADADC;
    char           wk3[1];
    union {
        unsigned short WORD;
        struct {
            unsigned short :1;
            unsigned short ADPRC:2;
            unsigned short :2;
            unsigned short ACE:1;
            unsigned short :2;
            unsigned short DIAGVAL:2;
            unsigned short DIAGLD:1;
            unsigned short DIAGM:1;
            unsigned short :3;
            unsigned short ADRFMT:1;
        } BIT;
    } ADCER;
    union {
        unsigned short WORD;
        struct {
            unsigned short TRSB:6;
            unsigned short :2;
            unsigned short TRSA:6;
            unsigned short :2;
        } BIT;
    } ADSTRGR;
    union {
        unsigned short WORD;
        struct {
            unsigned short :13;
            unsigned short EXSEL:2;
            unsigned short EXOEN:1;
        } BIT;
    } ADEXICR;
    union {
        unsigned short WORD;
        struct {
            unsigned short ANSB:16;
        } BIT;
    } ADANSB;
    char           wk4[2];
    unsigned short ADDBLDR;
    char           wk5[4];
    unsigned short ADRD;
    unsigned short ADDR0;
    unsigned short ADDR1;
    unsigned short ADDR2;
    unsigned short ADDR3;
    unsigned short ADDR4;
    unsigned short ADDR5;
    unsigned short ADDR6;
    unsigned short ADDR7;
    unsigned short ADDR8;
    unsigned short ADDR9;
    unsigned short ADDR10;
    unsigned short ADDR11;
    unsigned short ADDR12;
    unsigned short ADDR13;
    unsigned short ADDR14;
    unsigned short ADDR15;
    char           wk6[32];
    union {
        unsigned char BYTE;
        struct {
            unsigned char SST:8;
        } BIT;
    } ADSSTR0;
    union {
        unsigned char BYTE;
        struct {
            unsigned char SST:8;
        } BIT;
    } ADSSTRL;
    char           wk7[17];
    union {
        unsigned char BYTE;
        struct {
            unsigned char SST:8;
        } BIT;
    } ADSSTR1;
    union {
        unsigned char BYTE;
        struct {
            unsigned char SST:8;
        } BIT;
    } ADSSTR2;
    union {
        unsigned char BYTE;
        struct {
            unsigned char SST:8;
        } BIT;
    } ADSSTR3;
    union {
        unsigned char BYTE;
        struct {
            unsigned char SST:8;
        } BIT;
    } ADSSTR4;
    union {
        unsigned char BYTE;
        struct {
            unsigned char SST:8;
        } BIT;
    } ADSSTR5;
    union {
        unsigned char BYTE;
        struct {
            unsigned char SST:8;
        } BIT;
    } ADSSTR6;
    union {
        unsigned char BYTE;
        struct {
            unsigned char SST:8;
        } BIT;
    } ADSSTR7;
    union {
        unsigned char BYTE;
        struct {
            unsigned char ADNDIS:5;
            unsigned char :3;
        } BIT;
    } ADDISCR;
    char           wk8[5];
    union {
        unsigned short WORD;
        struct {
            unsigned short PGS:1;
            unsigned short GBRSCN:1;
            unsigned short :13;
            unsigned short GBRP:1;
        } BIT;
    } ADGSPCR;
    char           wk9[2];
    unsigned short ADDBLDRA;
    unsigned short ADDBLDRB;
    char           wk10[8];
    union {
        unsigned char BYTE;
        struct {
            unsigned char :6;
            unsigned char WCMPE:1;
            unsigned char CMPIE:1;
        } BIT;
    } ADCMPCR;
    char           wk11[3];
    union {
        unsigned short WORD;
        struct {
            unsigned short CMPS:16;
        } BIT;
    } ADCMPANSR;
    char           wk12[2];
    union {
        unsigned short WORD;
        struct {
            unsigned short CMPL:16;
        } BIT;
    } ADCMPLR;
    char           wk13[2];
    unsigned short ADCMPDR0;
    unsigned short ADCMPDR1;
    union {
        unsigned short WORD;
        struct {
            unsigned short CMPF:16;
        } BIT;
    } ADCMPSR;
    char           wk14[38];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TDLV:2;
            unsigned char :5;
            unsigned char TDE:1;
        } BIT;
    } ADTDCR;
    char           wk15[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char :2;
            unsigned char OWEIE:1;
            unsigned char :5;
        } BIT;
    } ADERCR;
    union {
        unsigned char BYTE;
        struct {
            unsigned char :2;
            unsigned char OWEC:1;
            unsigned char :5;
        } BIT;
    } ADERCLR;
    char           wk16[6];
    union {
        unsigned short WORD;
        struct {
            unsigned short OWE:16;
        } BIT;
    } ADOWER;
    char           wk17[2];
    union {
        unsigned short WORD;
        struct {
            unsigned short DBOWE:1;
            unsigned short DAOWE:1;
            unsigned short DOWE:1;
            unsigned short DIAGOWE:1;
            unsigned short TSOWE:1;
            unsigned short :11;
        } BIT;
    } ADOWEER;
};

struct st_scifa {
    union {
        unsigned short WORD;
        struct {
            unsigned short CKS:2;
            unsigned short :1;
            unsigned short STOP:1;
            unsigned short PM:1;
            unsigned short PE:1;
            unsigned short CHR:1;
            unsigned short CM:1;
            unsigned short :8;
        } BIT;
    } SMR;
    union {
        unsigned char  BRR;
        unsigned char  MDDR;
    } BRR_MDDR;
    char           wk0[1];
    union {
        unsigned short WORD;
        struct {
            unsigned short CKE:2;
            unsigned short TEIE:1;
            unsigned short REIE:1;
            unsigned short RE:1;
            unsigned short TE:1;
            unsigned short RIE:1;
            unsigned short TIE:1;
            unsigned short :8;
        } BIT;
    } SCR;
    unsigned char  FTDR;
    char           wk1[1];
    union {
        unsigned short WORD;
        struct {
            unsigned short DR:1;
            unsigned short RDF:1;
            unsigned short PER:1;
            unsigned short FER:1;
            unsigned short BRK:1;
            unsigned short TDFE:1;
            unsigned short TEND:1;
            unsigned short ER:1;
            unsigned short :8;
        } BIT;
    } FSR;
    unsigned char  FRDR;
    char           wk2[1];
    union {
        unsigned short WORD;
        struct {
            unsigned short LOOP:1;
            unsigned short RFRST:1;
            unsigned short TFRST:1;
            unsigned short MCE:1;
            unsigned short TTRG:2;
            unsigned short RTRG:2;
            unsigned short RSTRG:3;
            unsigned short :5;
        } BIT;
    } FCR;
    union {
        unsigned short WORD;
        struct {
            unsigned short R:5;
            unsigned short :3;
            unsigned short T:5;
            unsigned short :3;
        } BIT;
    } FDR;
    union {
        unsigned short WORD;
        struct {
            unsigned short SPB2DT:1;
            unsigned short SPB2IO:1;
            unsigned short SCKDT:1;
            unsigned short SCKIO:1;
            unsigned short CTS2DT:1;
            unsigned short CTS2IO:1;
            unsigned short RTS2DT:1;
            unsigned short RTS2IO:1;
            unsigned short :8;
        } BIT;
    } SPTR;
    union {
        unsigned short WORD;
        struct {
            unsigned short ORER:1;
            unsigned short :1;
            unsigned short FER:4;
            unsigned short :2;
            unsigned short PER:4;
            unsigned short :4;
        } BIT;
    } LSR;
    union {
        unsigned char BYTE;
        struct {
            unsigned char ABCS0:1;
            unsigned char :1;
            unsigned char NFEN:1;
            unsigned char DIR:1;
            unsigned char MDDRS:1;
            unsigned char BRME:1;
            unsigned char :1;
            unsigned char BGDM:1;
        } BIT;
    } SEMR;
    char           wk3[1];
    union {
        unsigned short WORD;
        struct {
            unsigned short TFTC:5;
            unsigned short :2;
            unsigned short TTRGS:1;
            unsigned short RFTC:5;
            unsigned short :2;
            unsigned short RTRGS:1;
        } BIT;
    } FTCR;
};

struct st_spibsc {
    union {
        unsigned long LONG;
        struct {
            unsigned long BSZ:2;
            unsigned long :1;
            unsigned long CPOL:1;
            unsigned long SSLP:1;
            unsigned long CPHAR:1;
            unsigned long CPHAT:1;
            unsigned long :1;
            unsigned long IO0FV:2;
            unsigned long :2;
            unsigned long IO2FV:2;
            unsigned long IO3FV:2;
            unsigned long MOIIO0:2;
            unsigned long MOIIO1:2;
            unsigned long MOIIO2:2;
            unsigned long MOIIO3:2;
            unsigned long SFDE:1;
            unsigned long :6;
            unsigned long MD:1;
        } BIT;
    } CMNCR;
    union {
        unsigned long LONG;
        struct {
            unsigned long SCKDL:3;
            unsigned long :5;
            unsigned long SLNDL:3;
            unsigned long :5;
            unsigned long SPNDL:3;
            unsigned long :13;
        } BIT;
    } SSLDR;
    union {
        unsigned long LONG;
        struct {
            unsigned long BRDV:2;
            unsigned long :6;
            unsigned long SPBR:8;
            unsigned long :16;
        } BIT;
    } SPBCR;
    union {
        unsigned long LONG;
        struct {
            unsigned long SSLE:1;
            unsigned long :7;
            unsigned long RBE:1;
            unsigned long RCF:1;
            unsigned long :6;
            unsigned long RBURST:4;
            unsigned long :4;
            unsigned long SSLN:1;
            unsigned long :7;
        } BIT;
    } DRCR;
    union {
        unsigned long LONG;
        struct {
            unsigned long OCMD:8;
            unsigned long :8;
            unsigned long CMD:8;
            unsigned long :8;
        } BIT;
    } DRCMR;
    union {
        unsigned long LONG;
        struct {
            unsigned long EAC:3;
            unsigned long :13;
            unsigned long EAV:8;
            unsigned long :8;
        } BIT;
    } DREAR;
    union {
        unsigned long LONG;
        struct {
            unsigned long OPD0:8;
            unsigned long OPD1:8;
            unsigned long OPD2:8;
            unsigned long OPD3:8;
        } BIT;
    } DROPR;
    union {
        unsigned long LONG;
        struct {
            unsigned long :4;
            unsigned long OPDE:4;
            unsigned long ADE:4;
            unsigned long OCDE:1;
            unsigned long :1;
            unsigned long CDE:1;
            unsigned long DME:1;
            unsigned long DRDB:2;
            unsigned long :2;
            unsigned long OPDB:2;
            unsigned long :2;
            unsigned long ADB:2;
            unsigned long :2;
            unsigned long OCDB:2;
            unsigned long CDB:2;
        } BIT;
    } DRENR;
    union {
        unsigned long LONG;
        struct {
            unsigned long SPIE:1;
            unsigned long SPIWE:1;
            unsigned long SPIRE:1;
            unsigned long :5;
            unsigned long SSLKP:1;
            unsigned long :23;
        } BIT;
    } SMCR;
    union {
        unsigned long LONG;
        struct {
            unsigned long OCMD:8;
            unsigned long :8;
            unsigned long CMD:8;
            unsigned long :8;
        } BIT;
    } SMCMR;
    union {
        unsigned long LONG;
        struct {
            unsigned long ADR:24;
            unsigned long ADRE:8;
        } BIT;
    } SMADR;
    union {
        unsigned long LONG;
        struct {
            unsigned long OPD0:8;
            unsigned long OPD1:8;
            unsigned long OPD2:8;
            unsigned long OPD3:8;
        } BIT;
    } SMOPR;
    union {
        unsigned long LONG;
        struct {
            unsigned long SPIDE:4;
            unsigned long OPDE:4;
            unsigned long ADE:4;
            unsigned long OCDE:1;
            unsigned long :1;
            unsigned long CDE:1;
            unsigned long DME:1;
            unsigned long SPIDB:2;
            unsigned long :2;
            unsigned long OPDB:2;
            unsigned long :2;
            unsigned long ADB:2;
            unsigned long :2;
            unsigned long OCDB:2;
            unsigned long CDB:2;
        } BIT;
    } SMENR;
    char           wk0[4];
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long RDATA0:32;
        } BIT;
    } SMRDR0;
    char           wk1[4];
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long WDATA0:32;
        } BIT;
    } SMWDR0;
    char           wk2[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long TEND:1;
            unsigned long SSLF:1;
            unsigned long :30;
        } BIT;
    } CMNSR;
    char           wk3[12];
    union {
        unsigned long LONG;
        struct {
            unsigned long DMCYC:3;
            unsigned long :13;
            unsigned long DMDB:2;
            unsigned long :14;
        } BIT;
    } DRDMCR;
    char           wk4[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long DMCYC:3;
            unsigned long :13;
            unsigned long DMDB:2;
            unsigned long :14;
        } BIT;
    } SMDMCR;
};

struct st_ssi {
    union {
        unsigned long LONG;
        struct {
            unsigned long REN:1;
            unsigned long TEN:1;
            unsigned long :1;
            unsigned long MUEN:1;
            unsigned long CKDV:4;
            unsigned long DEL:1;
            unsigned long PDTA:1;
            unsigned long SDTA:1;
            unsigned long SPDP:1;
            unsigned long SWSP:1;
            unsigned long SCKP:1;
            unsigned long SWSD:1;
            unsigned long SCKD:1;
            unsigned long SWL:3;
            unsigned long DWL:3;
            unsigned long CHNL:2;
            unsigned long :1;
            unsigned long IIEN:1;
            unsigned long ROIEN:1;
            unsigned long RUIEN:1;
            unsigned long TOIEN:1;
            unsigned long TUIEN:1;
            unsigned long CKS:1;
            unsigned long :1;
        } BIT;
    } SSICR;
    union {
        unsigned long LONG;
        struct {
            unsigned long IDST:1;
            unsigned long RSWNO:1;
            unsigned long :2;
            unsigned long TSWNO:1;
            unsigned long :20;
            unsigned long IIRQ:1;
            unsigned long ROIRQ:1;
            unsigned long RUIRQ:1;
            unsigned long TOIRQ:1;
            unsigned long TUIRQ:1;
            unsigned long :2;
        } BIT;
    } SSISR;
    char           wk0[8];
    union {
        unsigned long LONG;
        struct {
            unsigned long RFRST:1;
            unsigned long TFRST:1;
            unsigned long RIE:1;
            unsigned long TIE:1;
            unsigned long RTRG:2;
            unsigned long TTRG:2;
            unsigned long :23;
            unsigned long AUCKE:1;
        } BIT;
    } SSIFCR;
    union {
        unsigned long LONG;
        struct {
            unsigned long RDF:1;
            unsigned long :7;
            unsigned long RDC:4;
            unsigned long :4;
            unsigned long TDE:1;
            unsigned long :7;
            unsigned long TDC:4;
            unsigned long :4;
        } BIT;
    } SSIFSR;
    unsigned long  SSIFTDR;
    unsigned long  SSIFRDR;
    union {
        unsigned long LONG;
        struct {
            unsigned long :8;
            unsigned long CONT:1;
            unsigned long :23;
        } BIT;
    } SSITDMR;
};

struct st_system {
    union {
        unsigned long LONG;
        struct {
            unsigned long PCKG:2;
            unsigned long PCKF:2;
            unsigned long PCKE:2;
            unsigned long :2;
            unsigned long CKIO:3;
            unsigned long :1;
            unsigned long ETCKE:1;
            unsigned long :1;
            unsigned long ETCKD:2;
            unsigned long SERICK:1;
            unsigned long :3;
            unsigned long TCLK:1;
            unsigned long :11;
        } BIT;
    } SCKCR;
    union {
        unsigned long LONG;
        struct {
            unsigned long CKSEL0:1;
            unsigned long :31;
        } BIT;
    } SCKCR2;
    union {
        unsigned long LONG;
        struct {
            unsigned long DSSEL0:1;
            unsigned long DSCK0:3;
            unsigned long DSINV0:1;
            unsigned long DSCHSEL:1;
            unsigned long :10;
            unsigned long DSSEL1:1;
            unsigned long DSCK1:3;
            unsigned long DSINV1:1;
            unsigned long :11;
        } BIT;
    } DSCR;
    char           wk0[8];
    union {
        unsigned long LONG;
        struct {
            unsigned long CPUCKSEL:2;
            unsigned long :30;
        } BIT;
    } PLL1CR;
    union {
        unsigned long LONG;
        struct {
            unsigned long PLL1EN:1;
            unsigned long :31;
        } BIT;
    } PLL1CR2;
    char           wk1[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long LCSTP:1;
            unsigned long :31;
        } BIT;
    } LOCOCR;
    char           wk2[8];
    union {
        unsigned long LONG;
        struct {
            unsigned long OSTDIE:1;
            unsigned long :6;
            unsigned long OSTDE:1;
            unsigned long :24;
        } BIT;
    } OSTDCR;
    char           wk3[432];
    union {
        unsigned long LONG;
        struct {
            unsigned long :1;
            unsigned long TRF:1;
            unsigned long ECMRF:1;
            unsigned long SWR1F:1;
            unsigned long :28;
        } BIT;
    } RSTSR0;
    char           wk4[12];
    union {
        unsigned long LONG;
    } SWRR1;
    char           wk5[12];
    union {
        unsigned long LONG;
    } SWRR2;
    char           wk6[36];
    union {
        unsigned long LONG;
        struct {
            unsigned long MRUSBF:1;
            unsigned long MRUSBH:1;
            unsigned long :30;
        } BIT;
    } MRCTLC;
    char           wk7[180];
    union {
        unsigned long LONG;
        struct {
            unsigned long MSTPCRA0:1;
            unsigned long MSTPCRA1:1;
            unsigned long MSTPCRA2:1;
            unsigned long MSTPCRA3:1;
            unsigned long MSTPCRA4:1;
            unsigned long MSTPCRA5:1;
            unsigned long MSTPCRA6:1;
            unsigned long MSTPCRA7:1;
            unsigned long MSTPCRA8:1;
            unsigned long MSTPCRA9:1;
            unsigned long :1;
            unsigned long MSTPCRA11:1;
            unsigned long :20;
        } BIT;
    } MSTPCRA;
    union {
        unsigned long LONG;
        struct {
            unsigned long :1;
            unsigned long MSTPCRB1:1;
            unsigned long MSTPCRB2:1;
            unsigned long MSTPCRB3:1;
            unsigned long :1;
            unsigned long MSTPCRB5:1;
            unsigned long MSTPCRB6:1;
            unsigned long MSTPCRB7:1;
            unsigned long MSTPCRB8:1;
            unsigned long MSTPCRB9:1;
            unsigned long MSTPCRB10:1;
            unsigned long MSTPCRB11:1;
            unsigned long MSTPCRB12:1;
            unsigned long MSTPCRB13:1;
            unsigned long MSTPCRB14:1;
            unsigned long MSTPCRB15:1;
            unsigned long MSTPCRB16:1;
            unsigned long MSTPCRB17:1;
            unsigned long MSTPCRB18:1;
            unsigned long MSTPCRB19:1;
            unsigned long :12;
        } BIT;
    } MSTPCRB;
    union {
        unsigned long LONG;
        struct {
            unsigned long :1;
            unsigned long MSTPCRC1:1;
            unsigned long MSTPCRC2:1;
            unsigned long MSTPCRC3:1;
            unsigned long MSTPCRC4:1;
            unsigned long MSTPCRC5:1;
            unsigned long MSTPCRC6:1;
            unsigned long MSTPCRC7:1;
            unsigned long MSTPCRC8:1;
            unsigned long MSTPCRC9:1;
            unsigned long MSTPCRC10:1;
            unsigned long MSTPCRC11:1;
            unsigned long MSTPCRC12:1;
            unsigned long MSTPCRC13:1;
            unsigned long MSTPCRC14:1;
            unsigned long :17;
        } BIT;
    } MSTPCRC;
    union {
        unsigned long LONG;
        struct {
            unsigned long :2;
            unsigned long MSTPCRD2:1;
            unsigned long :29;
        } BIT;
    } MSTPCRD;
    union {
        unsigned long LONG;
        struct {
            unsigned long :4;
            unsigned long MSTPCRE4:1;
            unsigned long MSTPCRE5:1;
            unsigned long :26;
        } BIT;
    } MSTPCRE;
    union {
        unsigned long LONG;
        struct {
            unsigned long MSTPCRF0:1;
            unsigned long :31;
        } BIT;
    } MSTPCRF;
    char           wk8[1256];
    union {
        unsigned long LONG;
        struct {
            unsigned long ATCMWAIT:2;
            unsigned long :30;
        } BIT;
    } SYTATCMWAIT;
    char           wk9[284];
    union {
        unsigned long LONG;
        struct {
            unsigned long SEMFEN:1;
            unsigned long :31;
        } BIT;
    } SYTSEMFEN;
    char           wk10[12];
    union {
        unsigned long LONG;
        struct {
            unsigned long SEMF0:1;
            unsigned long :31;
        } BIT;
    } SYTSEMF0;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEMF1:1;
            unsigned long :31;
        } BIT;
    } SYTSEMF1;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEMF2:1;
            unsigned long :31;
        } BIT;
    } SYTSEMF2;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEMF3:1;
            unsigned long :31;
        } BIT;
    } SYTSEMF3;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEMF4:1;
            unsigned long :31;
        } BIT;
    } SYTSEMF4;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEMF5:1;
            unsigned long :31;
        } BIT;
    } SYTSEMF5;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEMF6:1;
            unsigned long :31;
        } BIT;
    } SYTSEMF6;
    union {
        unsigned long LONG;
        struct {
            unsigned long SEMF7:1;
            unsigned long :31;
        } BIT;
    } SYTSEMF7;
    char           wk11[176];
    union {
        unsigned long LONG;
        struct {
            unsigned long SWVSEL:2;
            unsigned long :30;
        } BIT;
    } DBGIFCNT;
    char           wk12[92];
    union {
        unsigned long LONG;
        struct {
            unsigned long MD0:1;
            unsigned long MD1:1;
            unsigned long MD2:1;
            unsigned long :29;
        } BIT;
    } MDMONR;
    char           wk13[28];
    union {
        unsigned long LONG;
        struct {
            unsigned long MSKC:1;
            unsigned long MSKM:1;
            unsigned long :30;
        } BIT;
    } ECMMCNT;
    char           wk14[124];
    union {
        unsigned long LONG;
    } PRCR;
};

struct st_tpu0 {
    union {
        unsigned char BYTE;
        struct {
            unsigned char NFAEN:1;
            unsigned char NFBEN:1;
            unsigned char NFCEN:1;
            unsigned char NFDEN:1;
            unsigned char NFCS:2;
            unsigned char :2;
        } BIT;
    } NFCR;
    char           wk0[7];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TPSC:3;
            unsigned char CKEG:2;
            unsigned char CCLR:3;
        } BIT;
    } TCR;
    union {
        unsigned char BYTE;
        struct {
            unsigned char MD:4;
            unsigned char BFA:1;
            unsigned char BFB:1;
            unsigned char ICSELB:1;
            unsigned char ICSELD:1;
        } BIT;
    } TMDR;
    union {
        unsigned char BYTE;
        struct {
            unsigned char IOA:4;
            unsigned char IOB:4;
        } BIT;
    } TIORH;
    union {
        unsigned char BYTE;
        struct {
            unsigned char IOC:4;
            unsigned char IOD:4;
        } BIT;
    } TIORL;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TGIEA:1;
            unsigned char TGIEB:1;
            unsigned char TGIEC:1;
            unsigned char TGIED:1;
            unsigned char TCIEV:1;
            unsigned char TCIEU:1;
            unsigned char :1;
            unsigned char TTGE:1;
        } BIT;
    } TIER;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TGFA:1;
            unsigned char TGFB:1;
            unsigned char TGFC:1;
            unsigned char TGFD:1;
            unsigned char TCFV:1;
            unsigned char TCFU:1;
            unsigned char :1;
            unsigned char TCFD:1;
        } BIT;
    } TSR;
    unsigned short TCNT;
    unsigned short TGRA;
    unsigned short TGRB;
    unsigned short TGRC;
    unsigned short TGRD;
};

struct st_tpu1 {
    char           wk0[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char NFAEN:1;
            unsigned char NFBEN:1;
            unsigned char NFCEN:1;
            unsigned char NFDEN:1;
            unsigned char NFCS:2;
            unsigned char :2;
        } BIT;
    } NFCR;
    char           wk1[22];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TPSC:3;
            unsigned char CKEG:2;
            unsigned char CCLR:3;
        } BIT;
    } TCR;
    union {
        unsigned char BYTE;
        struct {
            unsigned char MD:4;
            unsigned char BFA:1;
            unsigned char BFB:1;
            unsigned char ICSELB:1;
            unsigned char ICSELD:1;
        } BIT;
    } TMDR;
    union {
        unsigned char BYTE;
        struct {
            unsigned char IOA:4;
            unsigned char IOB:4;
        } BIT;
    } TIOR;
    char           wk2[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TGIEA:1;
            unsigned char TGIEB:1;
            unsigned char TGIEC:1;
            unsigned char TGIED:1;
            unsigned char TCIEV:1;
            unsigned char TCIEU:1;
            unsigned char :1;
            unsigned char TTGE:1;
        } BIT;
    } TIER;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TGFA:1;
            unsigned char TGFB:1;
            unsigned char TGFC:1;
            unsigned char TGFD:1;
            unsigned char TCFV:1;
            unsigned char TCFU:1;
            unsigned char :1;
            unsigned char TCFD:1;
        } BIT;
    } TSR;
    unsigned short TCNT;
    unsigned short TGRA;
    unsigned short TGRB;
};

struct st_tpu2 {
    union {
        unsigned char BYTE;
        struct {
            unsigned char NFAEN:1;
            unsigned char NFBEN:1;
            unsigned char NFCEN:1;
            unsigned char NFDEN:1;
            unsigned char NFCS:2;
            unsigned char :2;
        } BIT;
    } NFCR;
    char           wk0[37];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TPSC:3;
            unsigned char CKEG:2;
            unsigned char CCLR:3;
        } BIT;
    } TCR;
    union {
        unsigned char BYTE;
        struct {
            unsigned char MD:4;
            unsigned char BFA:1;
            unsigned char BFB:1;
            unsigned char ICSELB:1;
            unsigned char ICSELD:1;
        } BIT;
    } TMDR;
    union {
        unsigned char BYTE;
        struct {
            unsigned char IOA:4;
            unsigned char IOB:4;
        } BIT;
    } TIOR;
    char           wk1[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TGIEA:1;
            unsigned char TGIEB:1;
            unsigned char TGIEC:1;
            unsigned char TGIED:1;
            unsigned char TCIEV:1;
            unsigned char TCIEU:1;
            unsigned char :1;
            unsigned char TTGE:1;
        } BIT;
    } TIER;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TGFA:1;
            unsigned char TGFB:1;
            unsigned char TGFC:1;
            unsigned char TGFD:1;
            unsigned char TCFV:1;
            unsigned char TCFU:1;
            unsigned char :1;
            unsigned char TCFD:1;
        } BIT;
    } TSR;
    unsigned short TCNT;
    unsigned short TGRA;
    unsigned short TGRB;
};

struct st_tpu3 {
    char           wk0[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char NFAEN:1;
            unsigned char NFBEN:1;
            unsigned char NFCEN:1;
            unsigned char NFDEN:1;
            unsigned char NFCS:2;
            unsigned char :2;
        } BIT;
    } NFCR;
    char           wk1[52];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TPSC:3;
            unsigned char CKEG:2;
            unsigned char CCLR:3;
        } BIT;
    } TCR;
    union {
        unsigned char BYTE;
        struct {
            unsigned char MD:4;
            unsigned char BFA:1;
            unsigned char BFB:1;
            unsigned char ICSELB:1;
            unsigned char ICSELD:1;
        } BIT;
    } TMDR;
    union {
        unsigned char BYTE;
        struct {
            unsigned char IOA:4;
            unsigned char IOB:4;
        } BIT;
    } TIORH;
    union {
        unsigned char BYTE;
        struct {
            unsigned char IOC:4;
            unsigned char IOD:4;
        } BIT;
    } TIORL;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TGIEA:1;
            unsigned char TGIEB:1;
            unsigned char TGIEC:1;
            unsigned char TGIED:1;
            unsigned char TCIEV:1;
            unsigned char TCIEU:1;
            unsigned char :1;
            unsigned char TTGE:1;
        } BIT;
    } TIER;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TGFA:1;
            unsigned char TGFB:1;
            unsigned char TGFC:1;
            unsigned char TGFD:1;
            unsigned char TCFV:1;
            unsigned char TCFU:1;
            unsigned char :1;
            unsigned char TCFD:1;
        } BIT;
    } TSR;
    unsigned short TCNT;
    unsigned short TGRA;
    unsigned short TGRB;
    unsigned short TGRC;
    unsigned short TGRD;
};

struct st_tpu4 {
    union {
        unsigned char BYTE;
        struct {
            unsigned char NFAEN:1;
            unsigned char NFBEN:1;
            unsigned char NFCEN:1;
            unsigned char NFDEN:1;
            unsigned char NFCS:2;
            unsigned char :2;
        } BIT;
    } NFCR;
    char           wk0[67];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TPSC:3;
            unsigned char CKEG:2;
            unsigned char CCLR:3;
        } BIT;
    } TCR;
    union {
        unsigned char BYTE;
        struct {
            unsigned char MD:4;
            unsigned char BFA:1;
            unsigned char BFB:1;
            unsigned char ICSELB:1;
            unsigned char ICSELD:1;
        } BIT;
    } TMDR;
    union {
        unsigned char BYTE;
        struct {
            unsigned char IOA:4;
            unsigned char IOB:4;
        } BIT;
    } TIOR;
    char           wk1[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TGIEA:1;
            unsigned char TGIEB:1;
            unsigned char TGIEC:1;
            unsigned char TGIED:1;
            unsigned char TCIEV:1;
            unsigned char TCIEU:1;
            unsigned char :1;
            unsigned char TTGE:1;
        } BIT;
    } TIER;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TGFA:1;
            unsigned char TGFB:1;
            unsigned char TGFC:1;
            unsigned char TGFD:1;
            unsigned char TCFV:1;
            unsigned char TCFU:1;
            unsigned char :1;
            unsigned char TCFD:1;
        } BIT;
    } TSR;
    unsigned short TCNT;
    unsigned short TGRA;
    unsigned short TGRB;
};

struct st_tpu5 {
    char           wk0[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char NFAEN:1;
            unsigned char NFBEN:1;
            unsigned char NFCEN:1;
            unsigned char NFDEN:1;
            unsigned char NFCS:2;
            unsigned char :2;
        } BIT;
    } NFCR;
    char           wk1[82];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TPSC:3;
            unsigned char CKEG:2;
            unsigned char CCLR:3;
        } BIT;
    } TCR;
    union {
        unsigned char BYTE;
        struct {
            unsigned char MD:4;
            unsigned char BFA:1;
            unsigned char BFB:1;
            unsigned char ICSELB:1;
            unsigned char ICSELD:1;
        } BIT;
    } TMDR;
    union {
        unsigned char BYTE;
        struct {
            unsigned char IOA:4;
            unsigned char IOB:4;
        } BIT;
    } TIOR;
    char           wk2[1];
    union {
        unsigned char BYTE;
        struct {
            unsigned char TGIEA:1;
            unsigned char TGIEB:1;
            unsigned char TGIEC:1;
            unsigned char TGIED:1;
            unsigned char TCIEV:1;
            unsigned char TCIEU:1;
            unsigned char :1;
            unsigned char TTGE:1;
        } BIT;
    } TIER;
    union {
        unsigned char BYTE;
        struct {
            unsigned char TGFA:1;
            unsigned char TGFB:1;
            unsigned char TGFC:1;
            unsigned char TGFD:1;
            unsigned char TCFV:1;
            unsigned char TCFU:1;
            unsigned char :1;
            unsigned char TCFD:1;
        } BIT;
    } TSR;
    unsigned short TCNT;
    unsigned short TGRA;
    unsigned short TGRB;
};

struct st_tpua {
    union {
        unsigned char BYTE;
        struct {
            unsigned char CST0:1;
            unsigned char CST1:1;
            unsigned char CST2:1;
            unsigned char CST3:1;
            unsigned char CST4:1;
            unsigned char CST5:1;
            unsigned char :2;
        } BIT;
    } TSTRA;
    union {
        unsigned char BYTE;
        struct {
            unsigned char SYNC0:1;
            unsigned char SYNC1:1;
            unsigned char SYNC2:1;
            unsigned char SYNC3:1;
            unsigned char SYNC4:1;
            unsigned char SYNC5:1;
            unsigned char :2;
        } BIT;
    } TSYRA;
    char           wk0[126];
    union {
        unsigned char BYTE;
        struct {
            unsigned char CST0:1;
            unsigned char CST1:1;
            unsigned char CST2:1;
            unsigned char CST3:1;
            unsigned char CST4:1;
            unsigned char CST5:1;
            unsigned char :2;
        } BIT;
    } TSTRB;
    union {
        unsigned char BYTE;
        struct {
            unsigned char SYNC0:1;
            unsigned char SYNC1:1;
            unsigned char SYNC2:1;
            unsigned char SYNC3:1;
            unsigned char SYNC4:1;
            unsigned char SYNC5:1;
            unsigned char :2;
        } BIT;
    } TSYRB;
};

struct st_tpusl {
    union {
        unsigned long LONG;
        struct {
            unsigned long TPU0EN:1;
            unsigned long :1;
            unsigned long FBSL0:3;
            unsigned long :3;
            unsigned long TPU1EN:1;
            unsigned long :1;
            unsigned long FBSL1:3;
            unsigned long :19;
        } BIT;
    } PWMFBSLR;
};

struct st_tsn {
    union {
        unsigned char BYTE;
        struct {
            unsigned char :4;
            unsigned char TSOE:1;
            unsigned char :2;
            unsigned char TSEN:1;
        } BIT;
    } TSCR;
};

struct st_usbf {
    union {
        unsigned short WORD;
        struct {
            unsigned short USBE:1;
            unsigned short :3;
            unsigned short DPRPU:1;
            unsigned short DRPD:1;
            unsigned short :1;
            unsigned short HSE:1;
            unsigned short :8;
        } BIT;
    } SYSCFG0;
    union {
        unsigned short WORD;
        struct {
            unsigned short BWAIT:6;
            unsigned short :10;
        } BIT;
    } SYSCFG1;
    union {
        unsigned short WORD;
        struct {
            unsigned short LNST:2;
            unsigned short :14;
        } BIT;
    } SYSSTS0;
    char           wk0[2];
    union {
        unsigned short WORD;
        struct {
            unsigned short RHST:3;
            unsigned short :5;
            unsigned short WKUP:1;
            unsigned short :7;
        } BIT;
    } DVSTCTR0;
    char           wk1[2];
    union {
        unsigned short WORD;
        struct {
            unsigned short UTST:4;
            unsigned short :12;
        } BIT;
    } TESTMODE;
    char           wk2[2];
    union {
        unsigned short WORD;
        struct {
            unsigned short :4;
            unsigned short TENDE:1;
            unsigned short :7;
            unsigned short DFACC:2;
            unsigned short :2;
        } BIT;
    } D0FBCFG;
    union {
        unsigned short WORD;
        struct {
            unsigned short :4;
            unsigned short TENDE:1;
            unsigned short :7;
            unsigned short DFACC:2;
            unsigned short :2;
        } BIT;
    } D1FBCFG;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long FIFOPORT:32;
        } BIT;
    } CFIFO;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long FIFOPORT:32;
        } BIT;
    } D0FIFO;
    union {
        unsigned long LONG;
        struct {
            unsigned short H;
            unsigned short L;
        } WORD;
        struct {
            unsigned char HH;
            unsigned char HL;
            unsigned char LH;
            unsigned char LL;
        } BYTE;
        struct {
            unsigned long FIFOPORT:32;
        } BIT;
    } D1FIFO;
    union {
        unsigned short WORD;
        struct {
            unsigned short CURPIPE:4;
            unsigned short :1;
            unsigned short ISEL:1;
            unsigned short :2;
            unsigned short BIGEND:1;
            unsigned short :1;
            unsigned short MBW:2;
            unsigned short :2;
            unsigned short REW:1;
            unsigned short RCNT:1;
        } BIT;
    } CFIFOSEL;
    union {
        unsigned short WORD;
        struct {
            unsigned short DTLN:12;
            unsigned short :1;
            unsigned short FRDY:1;
            unsigned short BCLR:1;
            unsigned short BVAL:1;
        } BIT;
    } CFIFOCTR;
    char           wk3[4];
    union {
        unsigned short WORD;
        struct {
            unsigned short CURPIPE:4;
            unsigned short :4;
            unsigned short BIGEND:1;
            unsigned short :1;
            unsigned short MBW:2;
            unsigned short DREQE:1;
            unsigned short DCLRM:1;
            unsigned short REW:1;
            unsigned short RCNT:1;
        } BIT;
    } D0FIFOSEL;
    union {
        unsigned short WORD;
        struct {
            unsigned short DTLN:12;
            unsigned short :1;
            unsigned short FRDY:1;
            unsigned short BCLR:1;
            unsigned short BVAL:1;
        } BIT;
    } D0FIFOCTR;
    union {
        unsigned short WORD;
        struct {
            unsigned short CURPIPE:4;
            unsigned short :4;
            unsigned short BIGEND:1;
            unsigned short :1;
            unsigned short MBW:2;
            unsigned short DREQE:1;
            unsigned short DCLRM:1;
            unsigned short REW:1;
            unsigned short RCNT:1;
        } BIT;
    } D1FIFOSEL;
    union {
        unsigned short WORD;
        struct {
            unsigned short DTLN:12;
            unsigned short :1;
            unsigned short FRDY:1;
            unsigned short BCLR:1;
            unsigned short BVAL:1;
        } BIT;
    } D1FIFOCTR;
    union {
        unsigned short WORD;
        struct {
            unsigned short :8;
            unsigned short BRDYE:1;
            unsigned short NRDYE:1;
            unsigned short BEMPE:1;
            unsigned short CTRE:1;
            unsigned short DVSE:1;
            unsigned short SOFE:1;
            unsigned short RSME:1;
            unsigned short VBSE:1;
        } BIT;
    } INTENB0;
    char           wk4[4];
    union {
        unsigned short WORD;
        struct {
            unsigned short PIPEBRDYE:10;
            unsigned short :6;
        } BIT;
    } BRDYENB;
    union {
        unsigned short WORD;
        struct {
            unsigned short PIPENRDYE:10;
            unsigned short :6;
        } BIT;
    } NRDYENB;
    union {
        unsigned short WORD;
        struct {
            unsigned short PIPEBEMPE:10;
            unsigned short :6;
        } BIT;
    } BEMPENB;
    union {
        unsigned short WORD;
        struct {
            unsigned short :4;
            unsigned short EDGESTS:1;
            unsigned short INTL:1;
            unsigned short BRDYM:1;
            unsigned short :9;
        } BIT;
    } SOFCFG;
    char           wk5[2];
    union {
        unsigned short WORD;
        struct {
            unsigned short CTSQ:3;
            unsigned short VALID:1;
            unsigned short DVSQ:3;
            unsigned short VBSTS:1;
            unsigned short BRDY:1;
            unsigned short NRDY:1;
            unsigned short BEMP:1;
            unsigned short CTRT:1;
            unsigned short DVST:1;
            unsigned short SOFR:1;
            unsigned short RESM:1;
            unsigned short VBINT:1;
        } BIT;
    } INTSTS0;
    char           wk6[4];
    union {
        unsigned short WORD;
        struct {
            unsigned short PIPEBRDY:10;
            unsigned short :6;
        } BIT;
    } BRDYSTS;
    union {
        unsigned short WORD;
        struct {
            unsigned short PIPENRDY:10;
            unsigned short :6;
        } BIT;
    } NRDYSTS;
    union {
        unsigned short WORD;
        struct {
            unsigned short PIPEBEMP:10;
            unsigned short :6;
        } BIT;
    } BEMPSTS;
    union {
        unsigned short WORD;
        struct {
            unsigned short FRNM:11;
            unsigned short :3;
            unsigned short CRCE:1;
            unsigned short OVRN:1;
        } BIT;
    } FRMNUM;
    union {
        unsigned short WORD;
        struct {
            unsigned short UFRNM:3;
            unsigned short :13;
        } BIT;
    } UFRMNUM;
    union {
        unsigned short WORD;
        struct {
            unsigned short USBADDR:7;
            unsigned short :9;
        } BIT;
    } USBADDR;
    char           wk7[2];
    union {
        unsigned short WORD;
        struct {
            unsigned short bmRequestType:8;
            unsigned short bRequest:8;
        } BIT;
    } USBREQ;
    union {
        unsigned short WORD;
        struct {
            unsigned short wValue:16;
        } BIT;
    } USBVAL;
    union {
        unsigned short WORD;
        struct {
            unsigned short wIndex:16;
        } BIT;
    } USBINDX;
    union {
        unsigned short WORD;
        struct {
            unsigned short wLength:16;
        } BIT;
    } USBLENG;
    unsigned short DCPCFG;
    union {
        unsigned short WORD;
        struct {
            unsigned short MXPS:7;
            unsigned short :9;
        } BIT;
    } DCPMAXP;
    union {
        unsigned short WORD;
        struct {
            unsigned short PID:2;
            unsigned short CCPL:1;
            unsigned short :2;
            unsigned short PBUSY:1;
            unsigned short SQMON:1;
            unsigned short SQSET:1;
            unsigned short SQCLR:1;
            unsigned short :6;
            unsigned short BSTS:1;
        } BIT;
    } DCPCTR;
    char           wk8[2];
    union {
        unsigned short WORD;
        struct {
            unsigned short PIPESEL:4;
            unsigned short :12;
        } BIT;
    } PIPESEL;
    char           wk9[2];
    union {
        unsigned short WORD;
        struct {
            unsigned short EPNUM:4;
            unsigned short DIR:1;
            unsigned short :2;
            unsigned short SHTNAK:1;
            unsigned short CNTMD:1;
            unsigned short DBLB:1;
            unsigned short BFRE:1;
            unsigned short :3;
            unsigned short TYPE:2;
        } BIT;
    } PIPECFG;
    union {
        unsigned short WORD;
        struct {
            unsigned short BUFNMB:8;
            unsigned short :2;
            unsigned short BUFSIZE:5;
            unsigned short :1;
        } BIT;
    } PIPEBUF;
    union {
        unsigned short WORD;
        struct {
            unsigned short MXPS:11;
            unsigned short :5;
        } BIT;
    } PIPEMAXP;
    union {
        unsigned short WORD;
        struct {
            unsigned short IITV:3;
            unsigned short :9;
            unsigned short IFIS:1;
            unsigned short :3;
        } BIT;
    } PIPEPERI;
    union {
        unsigned short WORD;
        struct {
            unsigned short PID:2;
            unsigned short :3;
            unsigned short PBUSY:1;
            unsigned short SQMON:1;
            unsigned short SQSET:1;
            unsigned short SQCLR:1;
            unsigned short ACLRM:1;
            unsigned short ATREPM:1;
            unsigned short :3;
            unsigned short INBUFM:1;
            unsigned short BSTS:1;
        } BIT;
    } PIPE1CTR;
    union {
        unsigned short WORD;
        struct {
            unsigned short PID:2;
            unsigned short :3;
            unsigned short PBUSY:1;
            unsigned short SQMON:1;
            unsigned short SQSET:1;
            unsigned short SQCLR:1;
            unsigned short ACLRM:1;
            unsigned short ATREPM:1;
            unsigned short :3;
            unsigned short INBUFM:1;
            unsigned short BSTS:1;
        } BIT;
    } PIPE2CTR;
    union {
        unsigned short WORD;
        struct {
            unsigned short PID:2;
            unsigned short :3;
            unsigned short PBUSY:1;
            unsigned short SQMON:1;
            unsigned short SQSET:1;
            unsigned short SQCLR:1;
            unsigned short ACLRM:1;
            unsigned short ATREPM:1;
            unsigned short :3;
            unsigned short INBUFM:1;
            unsigned short BSTS:1;
        } BIT;
    } PIPE3CTR;
    union {
        unsigned short WORD;
        struct {
            unsigned short PID:2;
            unsigned short :3;
            unsigned short PBUSY:1;
            unsigned short SQMON:1;
            unsigned short SQSET:1;
            unsigned short SQCLR:1;
            unsigned short ACLRM:1;
            unsigned short ATREPM:1;
            unsigned short :3;
            unsigned short INBUFM:1;
            unsigned short BSTS:1;
        } BIT;
    } PIPE4CTR;
    union {
        unsigned short WORD;
        struct {
            unsigned short PID:2;
            unsigned short :3;
            unsigned short PBUSY:1;
            unsigned short SQMON:1;
            unsigned short SQSET:1;
            unsigned short SQCLR:1;
            unsigned short ACLRM:1;
            unsigned short ATREPM:1;
            unsigned short :3;
            unsigned short INBUFM:1;
            unsigned short BSTS:1;
        } BIT;
    } PIPE5CTR;
    union {
        unsigned short WORD;
        struct {
            unsigned short PID:2;
            unsigned short :3;
            unsigned short PBUSY:1;
            unsigned short SQMON:1;
            unsigned short SQSET:1;
            unsigned short SQCLR:1;
            unsigned short ACLRM:1;
            unsigned short :5;
            unsigned short BSTS:1;
        } BIT;
    } PIPE6CTR;
    union {
        unsigned short WORD;
        struct {
            unsigned short PID:2;
            unsigned short :3;
            unsigned short PBUSY:1;
            unsigned short SQMON:1;
            unsigned short SQSET:1;
            unsigned short SQCLR:1;
            unsigned short ACLRM:1;
            unsigned short :5;
            unsigned short BSTS:1;
        } BIT;
    } PIPE7CTR;
    union {
        unsigned short WORD;
        struct {
            unsigned short PID:2;
            unsigned short :3;
            unsigned short PBUSY:1;
            unsigned short SQMON:1;
            unsigned short SQSET:1;
            unsigned short SQCLR:1;
            unsigned short ACLRM:1;
            unsigned short :5;
            unsigned short BSTS:1;
        } BIT;
    } PIPE8CTR;
    union {
        unsigned short WORD;
        struct {
            unsigned short PID:2;
            unsigned short :3;
            unsigned short PBUSY:1;
            unsigned short SQMON:1;
            unsigned short SQSET:1;
            unsigned short SQCLR:1;
            unsigned short ACLRM:1;
            unsigned short :5;
            unsigned short BSTS:1;
        } BIT;
    } PIPE9CTR;
    char           wk10[14];
    union {
        unsigned short WORD;
        struct {
            unsigned short :8;
            unsigned short TRCLR:1;
            unsigned short TRENB:1;
            unsigned short :6;
        } BIT;
    } PIPE1TRE;
    union {
        unsigned short WORD;
        struct {
            unsigned short TRNCNT:16;
        } BIT;
    } PIPE1TRN;
    union {
        unsigned short WORD;
        struct {
            unsigned short :8;
            unsigned short TRCLR:1;
            unsigned short TRENB:1;
            unsigned short :6;
        } BIT;
    } PIPE2TRE;
    union {
        unsigned short WORD;
        struct {
            unsigned short TRNCNT:16;
        } BIT;
    } PIPE2TRN;
    union {
        unsigned short WORD;
        struct {
            unsigned short :8;
            unsigned short TRCLR:1;
            unsigned short TRENB:1;
            unsigned short :6;
        } BIT;
    } PIPE3TRE;
    union {
        unsigned short WORD;
        struct {
            unsigned short TRNCNT:16;
        } BIT;
    } PIPE3TRN;
    union {
        unsigned short WORD;
        struct {
            unsigned short :8;
            unsigned short TRCLR:1;
            unsigned short TRENB:1;
            unsigned short :6;
        } BIT;
    } PIPE4TRE;
    union {
        unsigned short WORD;
        struct {
            unsigned short TRNCNT:16;
        } BIT;
    } PIPE4TRN;
    union {
        unsigned short WORD;
        struct {
            unsigned short :8;
            unsigned short TRCLR:1;
            unsigned short TRENB:1;
            unsigned short :6;
        } BIT;
    } PIPE5TRE;
    union {
        unsigned short WORD;
        struct {
            unsigned short TRNCNT:16;
        } BIT;
    } PIPE5TRN;
    char           wk11[92];
    unsigned short LPCTRL;
    union {
        unsigned short WORD;
        struct {
            unsigned short :14;
            unsigned short SUSPM:1;
            unsigned short :1;
        } BIT;
    } LPSTS;
    unsigned short PHYFUNCTR;
    char           wk12[90];
    union {
        unsigned long LONG;
        struct {
            unsigned long FIFOPORT:32;
        } BIT;
    } D0FIFOB0;
    union {
        unsigned long LONG;
        struct {
            unsigned long FIFOPORT:32;
        } BIT;
    } D0FIFOB1;
    union {
        unsigned long LONG;
        struct {
            unsigned long FIFOPORT:32;
        } BIT;
    } D0FIFOB2;
    union {
        unsigned long LONG;
        struct {
            unsigned long FIFOPORT:32;
        } BIT;
    } D0FIFOB3;
    union {
        unsigned long LONG;
        struct {
            unsigned long FIFOPORT:32;
        } BIT;
    } D0FIFOB4;
    union {
        unsigned long LONG;
        struct {
            unsigned long FIFOPORT:32;
        } BIT;
    } D0FIFOB5;
    union {
        unsigned long LONG;
        struct {
            unsigned long FIFOPORT:32;
        } BIT;
    } D0FIFOB6;
    union {
        unsigned long LONG;
        struct {
            unsigned long FIFOPORT:32;
        } BIT;
    } D0FIFOB7;
    union {
        unsigned long LONG;
        struct {
            unsigned long FIFOPORT:32;
        } BIT;
    } D1FIFOB0;
    union {
        unsigned long LONG;
        struct {
            unsigned long FIFOPORT:32;
        } BIT;
    } D1FIFOB1;
    union {
        unsigned long LONG;
        struct {
            unsigned long FIFOPORT:32;
        } BIT;
    } D1FIFOB2;
    union {
        unsigned long LONG;
        struct {
            unsigned long FIFOPORT:32;
        } BIT;
    } D1FIFOB3;
    union {
        unsigned long LONG;
        struct {
            unsigned long FIFOPORT:32;
        } BIT;
    } D1FIFOB4;
    union {
        unsigned long LONG;
        struct {
            unsigned long FIFOPORT:32;
        } BIT;
    } D1FIFOB5;
    union {
        unsigned long LONG;
        struct {
            unsigned long FIFOPORT:32;
        } BIT;
    } D1FIFOB6;
    union {
        unsigned long LONG;
        struct {
            unsigned long FIFOPORT:32;
        } BIT;
    } D1FIFOB7;
    union {
        unsigned short WORD;
        struct {
            unsigned short P1PORTSEL:2;
            unsigned short PHYPD:1;
            unsigned short PHYRESET:1;
            unsigned short PHYVBUSIN:1;
            unsigned short :11;
        } BIT;
    } PHYSET1;
};

struct st_usbh {
    union {
        unsigned long LONG;
        struct {
            unsigned long Revision:8;
            unsigned long :24;
        } BIT;
    } HcRevision;
    union {
        unsigned long LONG;
        struct {
            unsigned long CBSR:2;
            unsigned long PLE:1;
            unsigned long IE:1;
            unsigned long CLE:1;
            unsigned long BLE:1;
            unsigned long HCFS:2;
            unsigned long :1;
            unsigned long RWC:1;
            unsigned long RWE:1;
            unsigned long :21;
        } BIT;
    } HcControl;
    union {
        unsigned long LONG;
        struct {
            unsigned long HCR:1;
            unsigned long CLF:1;
            unsigned long BLF:1;
            unsigned long OCR:1;
            unsigned long :12;
            unsigned long SOC:2;
            unsigned long :14;
        } BIT;
    } HcCommandStatus;
    union {
        unsigned long LONG;
        struct {
            unsigned long SO:1;
            unsigned long WDH:1;
            unsigned long SF:1;
            unsigned long RD:1;
            unsigned long UE:1;
            unsigned long FNO:1;
            unsigned long RHSC:1;
            unsigned long :25;
        } BIT;
    } HcIntStatus;
    union {
        unsigned long LONG;
        struct {
            unsigned long SOE:1;
            unsigned long WDHE:1;
            unsigned long SFE:1;
            unsigned long RDE:1;
            unsigned long UEE:1;
            unsigned long FNOE:1;
            unsigned long RHSCE:1;
            unsigned long :24;
            unsigned long MIE:1;
        } BIT;
    } HcIntEnable;
    union {
        unsigned long LONG;
        struct {
            unsigned long SOD:1;
            unsigned long WDHD:1;
            unsigned long SFD:1;
            unsigned long RDD:1;
            unsigned long UED:1;
            unsigned long FNOD:1;
            unsigned long RHSCD:1;
            unsigned long :24;
            unsigned long MID:1;
        } BIT;
    } HcIntDisable;
    union {
        unsigned long LONG;
        struct {
            unsigned long :8;
            unsigned long HcHCCA:24;
        } BIT;
    } HcHCCA;
    union {
        unsigned long LONG;
        struct {
            unsigned long :4;
            unsigned long PeriodicCurrentED:28;
        } BIT;
    } HcPeriodCurED;
    union {
        unsigned long LONG;
        struct {
            unsigned long :4;
            unsigned long ControlHeadED:28;
        } BIT;
    } HcContHeadED;
    union {
        unsigned long LONG;
        struct {
            unsigned long :4;
            unsigned long ControlCurrentED:28;
        } BIT;
    } HcContCurrentED;
    union {
        unsigned long LONG;
        struct {
            unsigned long :4;
            unsigned long BulkHeadED:28;
        } BIT;
    } HcBulkHeadED;
    union {
        unsigned long LONG;
        struct {
            unsigned long :4;
            unsigned long BulkCurrentED:28;
        } BIT;
    } HcBulkCurrentED;
    union {
        unsigned long LONG;
        struct {
            unsigned long :4;
            unsigned long DoneHead:28;
        } BIT;
    } HcDoneHead;
    union {
        unsigned long LONG;
        struct {
            unsigned long FI:14;
            unsigned long :2;
            unsigned long FSMPS:15;
            unsigned long FIT:1;
        } BIT;
    } HcFmInterval;
    union {
        unsigned long LONG;
        struct {
            unsigned long FR:14;
            unsigned long :17;
            unsigned long FRT:1;
        } BIT;
    } HcFmRemaining;
    union {
        unsigned long LONG;
        struct {
            unsigned long FrameNumber:16;
            unsigned long :16;
        } BIT;
    } HcFmNumber;
    union {
        unsigned long LONG;
        struct {
            unsigned long PeriodicStart:14;
            unsigned long :18;
        } BIT;
    } HcPeriodicStart;
    union {
        unsigned long LONG;
        struct {
            unsigned long HcLSThreshold:12;
            unsigned long :20;
        } BIT;
    } HcLSThreshold;
    union {
        unsigned long LONG;
        struct {
            unsigned long NDP:8;
            unsigned long PSM:1;
            unsigned long NPS:1;
            unsigned long DT:1;
            unsigned long OCPM:1;
            unsigned long NOCP:1;
            unsigned long :11;
            unsigned long POTPGT:8;
        } BIT;
    } HcRhDescriptorA;
    union {
        unsigned long LONG;
        struct {
            unsigned long DR:16;
            unsigned long PPCM:16;
        } BIT;
    } HcRhDescriptorB;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long CGP:1;
                unsigned long OCI:1;
                unsigned long :13;
                unsigned long SRWE:1;
                unsigned long SGP:1;
                unsigned long OCIC:1;
                unsigned long :13;
                unsigned long CRWE:1;
            } BIT;
        } HcRhStatus_A;
        union {
            unsigned long LONG;
            struct {
                unsigned long LPS:1;
                unsigned long OCI:1;
                unsigned long :13;
                unsigned long DRWE:1;
                unsigned long LPSC:1;
                unsigned long OCIC:1;
                unsigned long :13;
                unsigned long CRWE:1;
            } BIT;
        } HcRhStatus_B;
    } HcRhStatus;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long CPE:1;
                unsigned long SPE:1;
                unsigned long SPS:1;
                unsigned long CSS:1;
                unsigned long SPR:1;
                unsigned long :3;
                unsigned long SPP:1;
                unsigned long CPP:1;
                unsigned long :6;
                unsigned long CSC:1;
                unsigned long PESC:1;
                unsigned long PSSC:1;
                unsigned long OCIC:1;
                unsigned long PRSC:1;
                unsigned long :11;
            } BIT;
        } HcRhPortStatus1_A;
        union {
            unsigned long LONG;
            struct {
                unsigned long CCS:1;
                unsigned long PES:1;
                unsigned long PSS:1;
                unsigned long POCI:1;
                unsigned long PRS:1;
                unsigned long :3;
                unsigned long PPS:1;
                unsigned long LSDA:1;
                unsigned long :6;
                unsigned long CSC:1;
                unsigned long PESC:1;
                unsigned long PSSC:1;
                unsigned long OCIC:1;
                unsigned long PRSC:1;
                unsigned long :11;
            } BIT;
        } HcRhPortStatus1_B;
    } HcRhPortStatus1;
    char           wk0[4008];
    union {
        unsigned long LONG;
        struct {
            unsigned long CapabilityRegistersLength:8;
            unsigned long :8;
            unsigned long InterfaceVersionNumber:16;
        } BIT;
    } CAPL_VERSION;
    union {
        unsigned long LONG;
        struct {
            unsigned long N_PORTS:4;
            unsigned long PPC:1;
            unsigned long :2;
            unsigned long PortRoutingRules:1;
            unsigned long N_PCC:4;
            unsigned long N_CC:4;
            unsigned long P_INDICATOR:1;
            unsigned long :3;
            unsigned long DebugPortNumber:4;
            unsigned long :8;
        } BIT;
    } HCSPARAMS;
    union {
        unsigned long LONG;
        struct {
            unsigned long AC64:1;
            unsigned long PFLF:1;
            unsigned long ASPC:1;
            unsigned long :1;
            unsigned long IST:4;
            unsigned long EECP:8;
            unsigned long :16;
        } BIT;
    } HCCPARAMS;
    union {
        unsigned long LONG;
        struct {
            unsigned long CompanionPortRoute:32;
        } BIT;
    } HCSP_PORTROUTE;
    char           wk1[16];
    union {
        unsigned long LONG;
        struct {
            unsigned long RS:1;
            unsigned long HCRESET:1;
            unsigned long FrameListSize:2;
            unsigned long PeriodicScheduleEnable:1;
            unsigned long ASPME:1;
            unsigned long InterruptonAsyncAdvanceDoorbell:1;
            unsigned long LightHostControllerReset:1;
            unsigned long ASPMC:2;
            unsigned long :1;
            unsigned long AsynchronousScheduleParkModeEnable:1;
            unsigned long :4;
            unsigned long InterruptThresholdControl:8;
            unsigned long :8;
        } BIT;
    } USBCMD;
    union {
        unsigned long LONG;
        struct {
            unsigned long USBINT:1;
            unsigned long USBERRINT:1;
            unsigned long PortChangeDetect:1;
            unsigned long FrameListRollover:1;
            unsigned long HostSystemError:1;
            unsigned long InterruptonAsyncAdvance:1;
            unsigned long :6;
            unsigned long HCHalted:1;
            unsigned long Reclamation:1;
            unsigned long PeriodicScheduleStatus:1;
            unsigned long AsynchronousScheduleStatus:1;
            unsigned long :16;
        } BIT;
    } USBSTS;
    union {
        unsigned long LONG;
        struct {
            unsigned long USBInterruptEnable:1;
            unsigned long USBErrorInterruptEnable:1;
            unsigned long PortChangeInterruptEnable:1;
            unsigned long FrameListRolloverEnable:1;
            unsigned long HostSystemErrorEnable:1;
            unsigned long InterruptonAsyncAdvanceEnable:1;
            unsigned long :26;
        } BIT;
    } USBINTR;
    union {
        unsigned long LONG;
        struct {
            unsigned long FrameIndex:14;
            unsigned long :18;
        } BIT;
    } FRINDEX;
    union {
        unsigned long LONG;
        struct {
            unsigned long CTRLDSSEGMENT:32;
        } BIT;
    } CTRLDSSEGMENT;
    union {
        unsigned long LONG;
        struct {
            unsigned long :12;
            unsigned long BaseAddressLow:20;
        } BIT;
    } PERIODICLIST;
    union {
        unsigned long LONG;
        struct {
            unsigned long :5;
            unsigned long LPL:27;
        } BIT;
    } ASYNCLISTADDR;
    char           wk2[36];
    union {
        unsigned long LONG;
        struct {
            unsigned long CF:1;
            unsigned long :31;
        } BIT;
    } CONFIGFLAG;
    union {
        unsigned long LONG;
        struct {
            unsigned long CurrentConnectStatus:1;
            unsigned long ConnectStatusChange:1;
            unsigned long PortEnabledDisabled:1;
            unsigned long PortEnableDisableChange:1;
            unsigned long OvercurrentActive:1;
            unsigned long OvercurrentChange:1;
            unsigned long ForcePortResume:1;
            unsigned long Suspend:1;
            unsigned long PortReset:1;
            unsigned long :1;
            unsigned long LineStatus:2;
            unsigned long PP:1;
            unsigned long PortOwner:1;
            unsigned long PortIndicatorControl:2;
            unsigned long PortTestControl:4;
            unsigned long WKCNNT_E:1;
            unsigned long WKDSCNNT_E:1;
            unsigned long WKOC_E:1;
            unsigned long :9;
        } BIT;
    } PORTSC1;
    char           wk3[61336];
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long VendorID:16;
                unsigned long DeviceID:16;
            } BIT;
        } VID_DID_O;
        union {
            unsigned long LONG;
            struct {
                unsigned long VENDOR_ID:16;
                unsigned long DEVICE_ID:16;
            } BIT;
        } VID_DID_A;
    } VID_DID;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long IOSpace:1;
                unsigned long MemorySpace:1;
                unsigned long BusMaster:1;
                unsigned long SpecialCycle:1;
                unsigned long MemoryWriteandInvalidateEnable:1;
                unsigned long VGAPaletteSnoop:1;
                unsigned long ParityErrorResponse:1;
                unsigned long WaitCycleControl:1;
                unsigned long SERREnable:1;
                unsigned long FastBacktoBackEnable:1;
                unsigned long :10;
                unsigned long CapabilitiesList:1;
                unsigned long :2;
                unsigned long FastBacktoBackCapable:1;
                unsigned long DataParityErrorDetected:1;
                unsigned long DevselTiming:2;
                unsigned long SignaledTargetAbort:1;
                unsigned long ReceivedTargetAbort:1;
                unsigned long ReceivedMasterAbort:1;
                unsigned long SignaledSystemError:1;
                unsigned long DetectedParityError:1;
            } BIT;
        } CMND_STS_O;
        union {
            unsigned long LONG;
            struct {
                unsigned long IOEN:1;
                unsigned long MEMEN:1;
                unsigned long MASTEREN:1;
                unsigned long SPECIALC:1;
                unsigned long MWINVEN:1;
                unsigned long VGAPSNP:1;
                unsigned long PERREN:1;
                unsigned long STEPCTR:1;
                unsigned long SERREN:1;
                unsigned long FBTBEN:1;
                unsigned long :10;
                unsigned long CAPLIST:1;
                unsigned long CAP66M:1;
                unsigned long :1;
                unsigned long FBTBCAP:1;
                unsigned long MDPERR:1;
                unsigned long DEVTIM:2;
                unsigned long SIGTABORT:1;
                unsigned long RETABORT:1;
                unsigned long REMABORT:1;
                unsigned long SIGSERR:1;
                unsigned long DETPERR:1;
            } BIT;
        } CMND_STS_A;
    } CMND_STS;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long RevisionID:8;
                unsigned long ProgrammingIF:8;
                unsigned long SubClass:8;
                unsigned long BaseClass:8;
            } BIT;
        } REVID_CC_O;
        union {
            unsigned long LONG;
            struct {
                unsigned long REVISION_ID:8;
                unsigned long CLASS_CODE:24;
            } BIT;
        } REVID_CC_A;
    } REVID_CC;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long CacheLineSize:8;
                unsigned long LatencyTimer:8;
                unsigned long HeaderType:8;
                unsigned long BIST:8;
            } BIT;
        } CLS_LT_HT_BIST_O;
        union {
            unsigned long LONG;
            struct {
                unsigned long CACHE_LINE_SIZE:8;
                unsigned long LATENCY_TIMER:8;
                unsigned long HEADER_TYPE:8;
                unsigned long BIST:8;
            } BIT;
        } CLS_LT_HT_BIST_A;
    } CLS_LT_HT_BIST;
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long MemorySpaceIndicator:1;
                unsigned long Type:2;
                unsigned long Prefetchable:1;
                unsigned long OHCIBaseAddress:28;
            } BIT;
        } BASEAD_O;
        union {
            unsigned long LONG;
            struct {
                unsigned long MEM:1;
                unsigned long TYPE:2;
                unsigned long PREFETCH:1;
                unsigned long :6;
                unsigned long PCICOM_BASEADR:22;
            } BIT;
        } BASEAD_A;
    } BASEAD;
    union {
        unsigned long LONG;
        struct {
            unsigned long MEM:1;
            unsigned long TYPE:2;
            unsigned long PREFETCH:1;
            unsigned long :24;
            unsigned long PCI_WIN1_BASEADR:4;
        } BIT;
    } WIN1_BASEAD;
    char           wk4[20];
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long SubsystemVendorID:16;
                unsigned long SubsystemID:16;
            } BIT;
        } SSVID_SSID_O;
        union {
            unsigned long LONG;
            struct {
                unsigned long SUBSYS_VENDOR_ID:16;
                unsigned long SUBSYS_ID:16;
            } BIT;
        } SSVID_SSID_A;
    } SSVID_SSID;
    union {
        unsigned long LONG;
        struct {
            unsigned long ROMDecodeEnable:1;
            unsigned long :9;
            unsigned long ExpansionROMBaseAddress:22;
        } BIT;
    } EROM_BASEAD;
    union {
        unsigned long LONG;
        struct {
            unsigned long CapabilityPointer:8;
            unsigned long :24;
        } BIT;
    } CAPPTR;
    char           wk5[4];
    union {
        union {
            unsigned long LONG;
            struct {
                unsigned long InterruptLine:8;
                unsigned long InterruptPin:8;
                unsigned long MINGnt:8;
                unsigned long MaxLatency:8;
            } BIT;
        } INTR_LINE_PIN_O;
        union {
            unsigned long LONG;
            struct {
                unsigned long INT_LINE:8;
                unsigned long INT_PIN:8;
                unsigned long MIN_GNT:8;
                unsigned long MAX_LAT:8;
            } BIT;
        } INTR_LINE_PIN_A;
    } INTR_LINE_PIN;
    union {
        unsigned long LONG;
        struct {
            unsigned long CapabilityIdentifier:8;
            unsigned long NextItemPointer:8;
            unsigned long Version:3;
            unsigned long PMECLK:1;
            unsigned long :1;
            unsigned long DSI:1;
            unsigned long AUXCurrent:3;
            unsigned long D1Support:1;
            unsigned long D2Support:1;
            unsigned long PMESupport:5;
        } BIT;
    } CAPID_NIP_PMCAP;
    union {
        unsigned long LONG;
        struct {
            unsigned long PowerState:2;
            unsigned long :6;
            unsigned long PMEEnable:1;
            unsigned long DataSelect:4;
            unsigned long DataScale:2;
            unsigned long PMEStatus:1;
            unsigned long :6;
            unsigned long B2_B3:1;
            unsigned long BPCCEnable:1;
            unsigned long Data:8;
        } BIT;
    } PMC_STS_PMCSR;
    char           wk6[152];
    union {
        unsigned long LONG;
        struct {
            unsigned long Port_no:2;
            unsigned long :5;
            unsigned long ID_Write_Enable:1;
            unsigned long :5;
            unsigned long HyperSpeedtransferControl1:1;
            unsigned long :5;
            unsigned long HyperSpeedtransferControl2:5;
            unsigned long potpgt:8;
        } BIT;
    } EXT1;
    union {
        unsigned long LONG;
        struct {
            unsigned long EHCI_mask:1;
            unsigned long HyperSpeedtransferControl3:1;
            unsigned long :14;
            unsigned long RUNRAMConnectCheck:1;
            unsigned long RAMConnectCheckENDFlag:1;
            unsigned long RAMConnectCheckResult:1;
            unsigned long :13;
        } BIT;
    } EXT2;
    char           wk7[24];
    union {
        unsigned long LONG;
        struct {
            unsigned long VendorID:16;
            unsigned long DeviceID:16;
        } BIT;
    } VID_DID_E;
    union {
        unsigned long LONG;
        struct {
            unsigned long IOSpace:1;
            unsigned long MemorySpace:1;
            unsigned long BusMaster:1;
            unsigned long SpecialCycle:1;
            unsigned long MemoryWriteandInvalidateEnable:1;
            unsigned long VGAPaletteSnoop:1;
            unsigned long ParityErrorResponse:1;
            unsigned long WaitCycleControl:1;
            unsigned long SERREnable:1;
            unsigned long FastBacktoBackEnable:1;
            unsigned long :10;
            unsigned long CapabilitiesList:1;
            unsigned long Capable66MHz:1;
            unsigned long :1;
            unsigned long FastBacktoBackCapable:1;
            unsigned long DataParityErrorDetected:1;
            unsigned long DevselTiming:2;
            unsigned long SignaledTargetAbort:1;
            unsigned long ReceivedTargetAbort:1;
            unsigned long ReceivedMasterAbort:1;
            unsigned long SignaledSystemError:1;
            unsigned long DetectedParityError:1;
        } BIT;
    } CMND_STS_E;
    union {
        unsigned long LONG;
        struct {
            unsigned long RevisionID:8;
            unsigned long ProgrammingIF:8;
            unsigned long SubClass:8;
            unsigned long BaseClass:8;
        } BIT;
    } REVID_CC_E;
    union {
        unsigned long LONG;
        struct {
            unsigned long CacheLineSize:8;
            unsigned long LatencyTimer:8;
            unsigned long HeaderType:8;
            unsigned long BIST:8;
        } BIT;
    } CLS_LT_HT_BIST_E;
    union {
        unsigned long LONG;
        struct {
            unsigned long MemorySpaceIndicator:1;
            unsigned long Type:2;
            unsigned long Prefetchable:1;
            unsigned long EHCIBaseAddress:28;
        } BIT;
    } BASEAD_E;
    char           wk8[24];
    union {
        unsigned long LONG;
        struct {
            unsigned long SubsystemVendorID:16;
            unsigned long SubsystemID:16;
        } BIT;
    } SSVID_SSID_E;
    union {
        unsigned long LONG;
        struct {
            unsigned long ROMDecodeEnable:1;
            unsigned long :9;
            unsigned long ExpansionROMBaseAddress:22;
        } BIT;
    } EROM_BASEAD_E;
    union {
        unsigned long LONG;
        struct {
            unsigned long CapabilityPointer:8;
            unsigned long :24;
        } BIT;
    } CAPPTR_E;
    char           wk9[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long InterruptLine:8;
            unsigned long InterruptPin:8;
            unsigned long MinGnt:8;
            unsigned long MaxLatency:8;
        } BIT;
    } INTR_LINE_PIN_E;
    union {
        unsigned long LONG;
        struct {
            unsigned long CapabilityIdentifier:8;
            unsigned long NextItemPointer:8;
            unsigned long Version:3;
            unsigned long PMECLK:1;
            unsigned long :1;
            unsigned long DSI:1;
            unsigned long AUXCurrent:3;
            unsigned long D1Support:1;
            unsigned long D2Support:1;
            unsigned long PMESupport:5;
        } BIT;
    } CAPID_NIP_PMCAP_E;
    union {
        unsigned long LONG;
        struct {
            unsigned long PowerState:2;
            unsigned long :6;
            unsigned long PMEEnable:1;
            unsigned long DataSelect:4;
            unsigned long DataScale:2;
            unsigned long PMEStatus:1;
            unsigned long :6;
            unsigned long B2_B3:1;
            unsigned long BPCCEnable:1;
            unsigned long Data:8;
        } BIT;
    } PMC_STS_PMCSR_E;
    char           wk10[24];
    union {
        unsigned long LONG;
        struct {
            unsigned long SBRN:8;
            unsigned long FLADJ:8;
            unsigned long PORTWAKECAP:16;
        } BIT;
    } SBRN_FLADJ_PW;
    char           wk11[124];
    unsigned long  EXT1_E;
    unsigned long  EXT2_E;
    char           wk12[1560];
    union {
        unsigned long LONG;
        struct {
            unsigned long PREFETCH:2;
            unsigned long :26;
            unsigned long AHB_BASEADR:4;
        } BIT;
    } PCIAHB_WIN1_CTR;
    char           wk13[12];
    union {
        unsigned long LONG;
        struct {
            unsigned long :1;
            unsigned long PCICMD:3;
            unsigned long :7;
            unsigned long PCIWIN1_BASEADR:21;
        } BIT;
    } AHBPCI_WIN1_CTR;
    union {
        unsigned long LONG;
        struct {
            unsigned long :1;
            unsigned long PCICMD:3;
            unsigned long :1;
            unsigned long BURST_EN:1;
            unsigned long :10;
            unsigned long PCIWIN2_BASEADR:16;
        } BIT;
    } AHBPCI_WIN2_CTR;
    char           wk14[8];
    union {
        unsigned long LONG;
        struct {
            unsigned long SIGTABORT_INTEN:1;
            unsigned long RETABORT_INTEN:1;
            unsigned long REMABORT_INTEN:1;
            unsigned long PERR_INTEN:1;
            unsigned long SIGSERR_INTEN:1;
            unsigned long RESERR_INTEN:1;
            unsigned long :6;
            unsigned long PCIAHB_WIN1_INTEN:1;
            unsigned long PCIAHB_WIN2_INTEN:1;
            unsigned long :2;
            unsigned long USBH_INTAEN:1;
            unsigned long USBH_INTBEN:1;
            unsigned long :1;
            unsigned long USBH_PMEEN:1;
            unsigned long :12;
        } BIT;
    } PCI_INT_ENABLE;
    union {
        unsigned long LONG;
        struct {
            unsigned long SIGTABORT_INT:1;
            unsigned long RETABORT_INT:1;
            unsigned long REMABORT_INT:1;
            unsigned long PERR_INT:1;
            unsigned long SIGSERR_INT:1;
            unsigned long RESERR_INT:1;
            unsigned long :6;
            unsigned long PCIAHB_WIN1_INT:1;
            unsigned long PCIAHB_WIN2_INT:1;
            unsigned long :2;
            unsigned long USBH_INTA:1;
            unsigned long USBH_INTB:1;
            unsigned long :1;
            unsigned long USBH_PME:1;
            unsigned long :12;
        } BIT;
    } PCI_INT_STATUS;
    char           wk15[8];
    union {
        unsigned long LONG;
        struct {
            unsigned long MMODE_HTRANS:1;
            unsigned long MMODE_BYTE_BURST:1;
            unsigned long MMODE_WR_INCR:1;
            unsigned long :4;
            unsigned long MMODE_HBUSREQ:1;
            unsigned long :9;
            unsigned long SMODE_READY_CTR:1;
            unsigned long :14;
        } BIT;
    } AHB_BUS_CTR;
    union {
        unsigned long LONG;
        struct {
            unsigned long USBH_RST:1;
            unsigned long PCICLK_MASK:1;
            unsigned long :7;
            unsigned long PCI_AHB_WIN2_EN:1;
            unsigned long PCI_AHB_WIN1_SIZE:2;
            unsigned long :20;
        } BIT;
    } USBCTR;
    char           wk16[8];
    union {
        unsigned long LONG;
        struct {
            unsigned long PCIREQ0:1;
            unsigned long PCIREQ1:1;
            unsigned long :10;
            unsigned long PCIBP_MODE:1;
            unsigned long :19;
        } BIT;
    } PCI_ARBITER_CTR;
    char           wk17[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long MinorRevisionID:16;
            unsigned long MajorRevisionID:16;
        } BIT;
    } PCI_UNIT_REV;
};

struct st_vic {
    union {
        unsigned long LONG;
        struct {
            unsigned long :1;
            unsigned long IRQ1:1;
            unsigned long IRQ2:1;
            unsigned long IRQ3:1;
            unsigned long IRQ4:1;
            unsigned long IRQ5:1;
            unsigned long IRQ6:1;
            unsigned long IRQ7:1;
            unsigned long IRQ8:1;
            unsigned long IRQ9:1;
            unsigned long IRQ10:1;
            unsigned long IRQ11:1;
            unsigned long IRQ12:1;
            unsigned long IRQ13:1;
            unsigned long IRQ14:1;
            unsigned long IRQ15:1;
            unsigned long IRQ16:1;
            unsigned long IRQ17:1;
            unsigned long IRQ18:1;
            unsigned long IRQ19:1;
            unsigned long IRQ20:1;
            unsigned long IRQ21:1;
            unsigned long IRQ22:1;
            unsigned long IRQ23:1;
            unsigned long IRQ24:1;
            unsigned long IRQ25:1;
            unsigned long IRQ26:1;
            unsigned long IRQ27:1;
            unsigned long IRQ28:1;
            unsigned long IRQ29:1;
            unsigned long IRQ30:1;
            unsigned long IRQ31:1;
        } BIT;
    } IRQS0;
    union {
        unsigned long LONG;
        struct {
            unsigned long IRQ32:1;
            unsigned long IRQ33:1;
            unsigned long IRQ34:1;
            unsigned long IRQ35:1;
            unsigned long IRQ36:1;
            unsigned long IRQ37:1;
            unsigned long IRQ38:1;
            unsigned long IRQ39:1;
            unsigned long IRQ40:1;
            unsigned long IRQ41:1;
            unsigned long IRQ42:1;
            unsigned long IRQ43:1;
            unsigned long IRQ44:1;
            unsigned long IRQ45:1;
            unsigned long IRQ46:1;
            unsigned long IRQ47:1;
            unsigned long IRQ48:1;
            unsigned long IRQ49:1;
            unsigned long IRQ50:1;
            unsigned long IRQ51:1;
            unsigned long IRQ52:1;
            unsigned long IRQ53:1;
            unsigned long IRQ54:1;
            unsigned long IRQ55:1;
            unsigned long IRQ56:1;
            unsigned long IRQ57:1;
            unsigned long IRQ58:1;
            unsigned long IRQ59:1;
            unsigned long IRQ60:1;
            unsigned long IRQ61:1;
            unsigned long IRQ62:1;
            unsigned long IRQ63:1;
        } BIT;
    } IRQS1;
    union {
        unsigned long LONG;
        struct {
            unsigned long IRQ64:1;
            unsigned long IRQ65:1;
            unsigned long IRQ66:1;
            unsigned long IRQ67:1;
            unsigned long IRQ68:1;
            unsigned long IRQ69:1;
            unsigned long IRQ70:1;
            unsigned long IRQ71:1;
            unsigned long IRQ72:1;
            unsigned long IRQ73:1;
            unsigned long IRQ74:1;
            unsigned long IRQ75:1;
            unsigned long IRQ76:1;
            unsigned long IRQ77:1;
            unsigned long IRQ78:1;
            unsigned long IRQ79:1;
            unsigned long IRQ80:1;
            unsigned long IRQ81:1;
            unsigned long IRQ82:1;
            unsigned long IRQ83:1;
            unsigned long IRQ84:1;
            unsigned long IRQ85:1;
            unsigned long IRQ86:1;
            unsigned long IRQ87:1;
            unsigned long IRQ88:1;
            unsigned long IRQ89:1;
            unsigned long IRQ90:1;
            unsigned long IRQ91:1;
            unsigned long IRQ92:1;
            unsigned long IRQ93:1;
            unsigned long IRQ94:1;
            unsigned long IRQ95:1;
        } BIT;
    } IRQS2;
    union {
        unsigned long LONG;
        struct {
            unsigned long IRQ96:1;
            unsigned long IRQ97:1;
            unsigned long IRQ98:1;
            unsigned long IRQ99:1;
            unsigned long IRQ100:1;
            unsigned long IRQ101:1;
            unsigned long IRQ102:1;
            unsigned long IRQ103:1;
            unsigned long IRQ104:1;
            unsigned long IRQ105:1;
            unsigned long IRQ106:1;
            unsigned long IRQ107:1;
            unsigned long IRQ108:1;
            unsigned long IRQ109:1;
            unsigned long IRQ110:1;
            unsigned long IRQ111:1;
            unsigned long IRQ112:1;
            unsigned long IRQ113:1;
            unsigned long IRQ114:1;
            unsigned long IRQ115:1;
            unsigned long IRQ116:1;
            unsigned long IRQ117:1;
            unsigned long IRQ118:1;
            unsigned long IRQ119:1;
            unsigned long IRQ120:1;
            unsigned long IRQ121:1;
            unsigned long IRQ122:1;
            unsigned long IRQ123:1;
            unsigned long IRQ124:1;
            unsigned long IRQ125:1;
            unsigned long IRQ126:1;
            unsigned long IRQ127:1;
        } BIT;
    } IRQS3;
    union {
        unsigned long LONG;
        struct {
            unsigned long IRQ128:1;
            unsigned long IRQ129:1;
            unsigned long IRQ130:1;
            unsigned long IRQ131:1;
            unsigned long IRQ132:1;
            unsigned long IRQ133:1;
            unsigned long IRQ134:1;
            unsigned long IRQ135:1;
            unsigned long IRQ136:1;
            unsigned long IRQ137:1;
            unsigned long IRQ138:1;
            unsigned long IRQ139:1;
            unsigned long IRQ140:1;
            unsigned long IRQ141:1;
            unsigned long IRQ142:1;
            unsigned long IRQ143:1;
            unsigned long IRQ144:1;
            unsigned long IRQ145:1;
            unsigned long IRQ146:1;
            unsigned long IRQ147:1;
            unsigned long IRQ148:1;
            unsigned long IRQ149:1;
            unsigned long IRQ150:1;
            unsigned long IRQ151:1;
            unsigned long IRQ152:1;
            unsigned long IRQ153:1;
            unsigned long IRQ154:1;
            unsigned long IRQ155:1;
            unsigned long IRQ156:1;
            unsigned long IRQ157:1;
            unsigned long IRQ158:1;
            unsigned long IRQ159:1;
        } BIT;
    } IRQS4;
    union {
        unsigned long LONG;
        struct {
            unsigned long IRQ160:1;
            unsigned long IRQ161:1;
            unsigned long IRQ162:1;
            unsigned long IRQ163:1;
            unsigned long IRQ164:1;
            unsigned long IRQ165:1;
            unsigned long IRQ166:1;
            unsigned long IRQ167:1;
            unsigned long IRQ168:1;
            unsigned long IRQ169:1;
            unsigned long IRQ170:1;
            unsigned long IRQ171:1;
            unsigned long IRQ172:1;
            unsigned long IRQ173:1;
            unsigned long IRQ174:1;
            unsigned long IRQ175:1;
            unsigned long IRQ176:1;
            unsigned long IRQ177:1;
            unsigned long IRQ178:1;
            unsigned long IRQ179:1;
            unsigned long IRQ180:1;
            unsigned long IRQ181:1;
            unsigned long IRQ182:1;
            unsigned long IRQ183:1;
            unsigned long IRQ184:1;
            unsigned long IRQ185:1;
            unsigned long IRQ186:1;
            unsigned long IRQ187:1;
            unsigned long IRQ188:1;
            unsigned long IRQ189:1;
            unsigned long IRQ190:1;
            unsigned long IRQ191:1;
        } BIT;
    } IRQS5;
    union {
        unsigned long LONG;
        struct {
            unsigned long IRQ192:1;
            unsigned long IRQ193:1;
            unsigned long IRQ194:1;
            unsigned long IRQ195:1;
            unsigned long IRQ196:1;
            unsigned long IRQ197:1;
            unsigned long IRQ198:1;
            unsigned long IRQ199:1;
            unsigned long IRQ200:1;
            unsigned long IRQ201:1;
            unsigned long IRQ202:1;
            unsigned long IRQ203:1;
            unsigned long IRQ204:1;
            unsigned long IRQ205:1;
            unsigned long IRQ206:1;
            unsigned long IRQ207:1;
            unsigned long IRQ208:1;
            unsigned long IRQ209:1;
            unsigned long IRQ210:1;
            unsigned long IRQ211:1;
            unsigned long IRQ212:1;
            unsigned long IRQ213:1;
            unsigned long IRQ214:1;
            unsigned long IRQ215:1;
            unsigned long IRQ216:1;
            unsigned long IRQ217:1;
            unsigned long IRQ218:1;
            unsigned long IRQ219:1;
            unsigned long IRQ220:1;
            unsigned long IRQ221:1;
            unsigned long IRQ222:1;
            unsigned long IRQ223:1;
        } BIT;
    } IRQS6;
    union {
        unsigned long LONG;
        struct {
            unsigned long IRQ224:1;
            unsigned long IRQ225:1;
            unsigned long IRQ226:1;
            unsigned long IRQ227:1;
            unsigned long IRQ228:1;
            unsigned long IRQ229:1;
            unsigned long IRQ230:1;
            unsigned long IRQ231:1;
            unsigned long IRQ232:1;
            unsigned long IRQ233:1;
            unsigned long IRQ234:1;
            unsigned long IRQ235:1;
            unsigned long IRQ236:1;
            unsigned long IRQ237:1;
            unsigned long IRQ238:1;
            unsigned long IRQ239:1;
            unsigned long IRQ240:1;
            unsigned long IRQ241:1;
            unsigned long IRQ242:1;
            unsigned long IRQ243:1;
            unsigned long IRQ244:1;
            unsigned long IRQ245:1;
            unsigned long IRQ246:1;
            unsigned long IRQ247:1;
            unsigned long IRQ248:1;
            unsigned long IRQ249:1;
            unsigned long IRQ250:1;
            unsigned long IRQ251:1;
            unsigned long IRQ252:1;
            unsigned long IRQ253:1;
            unsigned long IRQ254:1;
            unsigned long IRQ255:1;
        } BIT;
    } IRQS7;
    char           wk0[32];
    union {
        unsigned long LONG;
        struct {
            unsigned long :1;
            unsigned long RAI1:1;
            unsigned long RAI2:1;
            unsigned long RAI3:1;
            unsigned long RAI4:1;
            unsigned long RAI5:1;
            unsigned long RAI6:1;
            unsigned long RAI7:1;
            unsigned long RAI8:1;
            unsigned long RAI9:1;
            unsigned long RAI10:1;
            unsigned long RAI11:1;
            unsigned long RAI12:1;
            unsigned long RAI13:1;
            unsigned long RAI14:1;
            unsigned long RAI15:1;
            unsigned long RAI16:1;
            unsigned long RAI17:1;
            unsigned long RAI18:1;
            unsigned long RAI19:1;
            unsigned long RAI20:1;
            unsigned long RAI21:1;
            unsigned long RAI22:1;
            unsigned long RAI23:1;
            unsigned long RAI24:1;
            unsigned long RAI25:1;
            unsigned long RAI26:1;
            unsigned long RAI27:1;
            unsigned long RAI28:1;
            unsigned long RAI29:1;
            unsigned long RAI30:1;
            unsigned long RAI31:1;
        } BIT;
    } RAIS0;
    union {
        unsigned long LONG;
        struct {
            unsigned long RAI32:1;
            unsigned long RAI33:1;
            unsigned long RAI34:1;
            unsigned long RAI35:1;
            unsigned long RAI36:1;
            unsigned long RAI37:1;
            unsigned long RAI38:1;
            unsigned long RAI39:1;
            unsigned long RAI40:1;
            unsigned long RAI41:1;
            unsigned long RAI42:1;
            unsigned long RAI43:1;
            unsigned long RAI44:1;
            unsigned long RAI45:1;
            unsigned long RAI46:1;
            unsigned long RAI47:1;
            unsigned long RAI48:1;
            unsigned long RAI49:1;
            unsigned long RAI50:1;
            unsigned long RAI51:1;
            unsigned long RAI52:1;
            unsigned long RAI53:1;
            unsigned long RAI54:1;
            unsigned long RAI55:1;
            unsigned long RAI56:1;
            unsigned long RAI57:1;
            unsigned long RAI58:1;
            unsigned long RAI59:1;
            unsigned long RAI60:1;
            unsigned long RAI61:1;
            unsigned long RAI62:1;
            unsigned long RAI63:1;
        } BIT;
    } RAIS1;
    union {
        unsigned long LONG;
        struct {
            unsigned long RAI64:1;
            unsigned long RAI65:1;
            unsigned long RAI66:1;
            unsigned long RAI67:1;
            unsigned long RAI68:1;
            unsigned long RAI69:1;
            unsigned long RAI70:1;
            unsigned long RAI71:1;
            unsigned long RAI72:1;
            unsigned long RAI73:1;
            unsigned long RAI74:1;
            unsigned long RAI75:1;
            unsigned long RAI76:1;
            unsigned long RAI77:1;
            unsigned long RAI78:1;
            unsigned long RAI79:1;
            unsigned long RAI80:1;
            unsigned long RAI81:1;
            unsigned long RAI82:1;
            unsigned long RAI83:1;
            unsigned long RAI84:1;
            unsigned long RAI85:1;
            unsigned long RAI86:1;
            unsigned long RAI87:1;
            unsigned long RAI88:1;
            unsigned long RAI89:1;
            unsigned long RAI90:1;
            unsigned long RAI91:1;
            unsigned long RAI92:1;
            unsigned long RAI93:1;
            unsigned long RAI94:1;
            unsigned long RAI95:1;
        } BIT;
    } RAIS2;
    union {
        unsigned long LONG;
        struct {
            unsigned long RAI96:1;
            unsigned long RAI97:1;
            unsigned long RAI98:1;
            unsigned long RAI99:1;
            unsigned long RAI100:1;
            unsigned long RAI101:1;
            unsigned long RAI102:1;
            unsigned long RAI103:1;
            unsigned long RAI104:1;
            unsigned long RAI105:1;
            unsigned long RAI106:1;
            unsigned long RAI107:1;
            unsigned long RAI108:1;
            unsigned long RAI109:1;
            unsigned long RAI110:1;
            unsigned long RAI111:1;
            unsigned long RAI112:1;
            unsigned long RAI113:1;
            unsigned long RAI114:1;
            unsigned long RAI115:1;
            unsigned long RAI116:1;
            unsigned long RAI117:1;
            unsigned long RAI118:1;
            unsigned long RAI119:1;
            unsigned long RAI120:1;
            unsigned long RAI121:1;
            unsigned long RAI122:1;
            unsigned long RAI123:1;
            unsigned long RAI124:1;
            unsigned long RAI125:1;
            unsigned long RAI126:1;
            unsigned long RAI127:1;
        } BIT;
    } RAIS3;
    union {
        unsigned long LONG;
        struct {
            unsigned long RAI128:1;
            unsigned long RAI129:1;
            unsigned long RAI130:1;
            unsigned long RAI131:1;
            unsigned long RAI132:1;
            unsigned long RAI133:1;
            unsigned long RAI134:1;
            unsigned long RAI135:1;
            unsigned long RAI136:1;
            unsigned long RAI137:1;
            unsigned long RAI138:1;
            unsigned long RAI139:1;
            unsigned long RAI140:1;
            unsigned long RAI141:1;
            unsigned long RAI142:1;
            unsigned long RAI143:1;
            unsigned long RAI144:1;
            unsigned long RAI145:1;
            unsigned long RAI146:1;
            unsigned long RAI147:1;
            unsigned long RAI148:1;
            unsigned long RAI149:1;
            unsigned long RAI150:1;
            unsigned long RAI151:1;
            unsigned long RAI152:1;
            unsigned long RAI153:1;
            unsigned long RAI154:1;
            unsigned long RAI155:1;
            unsigned long RAI156:1;
            unsigned long RAI157:1;
            unsigned long RAI158:1;
            unsigned long RAI159:1;
        } BIT;
    } RAIS4;
    union {
        unsigned long LONG;
        struct {
            unsigned long RAI160:1;
            unsigned long RAI161:1;
            unsigned long RAI162:1;
            unsigned long RAI163:1;
            unsigned long RAI164:1;
            unsigned long RAI165:1;
            unsigned long RAI166:1;
            unsigned long RAI167:1;
            unsigned long RAI168:1;
            unsigned long RAI169:1;
            unsigned long RAI170:1;
            unsigned long RAI171:1;
            unsigned long RAI172:1;
            unsigned long RAI173:1;
            unsigned long RAI174:1;
            unsigned long RAI175:1;
            unsigned long RAI176:1;
            unsigned long RAI177:1;
            unsigned long RAI178:1;
            unsigned long RAI179:1;
            unsigned long RAI180:1;
            unsigned long RAI181:1;
            unsigned long RAI182:1;
            unsigned long RAI183:1;
            unsigned long RAI184:1;
            unsigned long RAI185:1;
            unsigned long RAI186:1;
            unsigned long RAI187:1;
            unsigned long RAI188:1;
            unsigned long RAI189:1;
            unsigned long RAI190:1;
            unsigned long RAI191:1;
        } BIT;
    } RAIS5;
    union {
        unsigned long LONG;
        struct {
            unsigned long RAI192:1;
            unsigned long RAI193:1;
            unsigned long RAI194:1;
            unsigned long RAI195:1;
            unsigned long RAI196:1;
            unsigned long RAI197:1;
            unsigned long RAI198:1;
            unsigned long RAI199:1;
            unsigned long RAI200:1;
            unsigned long RAI201:1;
            unsigned long RAI202:1;
            unsigned long RAI203:1;
            unsigned long RAI204:1;
            unsigned long RAI205:1;
            unsigned long RAI206:1;
            unsigned long RAI207:1;
            unsigned long RAI208:1;
            unsigned long RAI209:1;
            unsigned long RAI210:1;
            unsigned long RAI211:1;
            unsigned long RAI212:1;
            unsigned long RAI213:1;
            unsigned long RAI214:1;
            unsigned long RAI215:1;
            unsigned long RAI216:1;
            unsigned long RAI217:1;
            unsigned long RAI218:1;
            unsigned long RAI219:1;
            unsigned long RAI220:1;
            unsigned long RAI221:1;
            unsigned long RAI222:1;
            unsigned long RAI223:1;
        } BIT;
    } RAIS6;
    union {
        unsigned long LONG;
        struct {
            unsigned long RAI224:1;
            unsigned long RAI225:1;
            unsigned long RAI226:1;
            unsigned long RAI227:1;
            unsigned long RAI228:1;
            unsigned long RAI229:1;
            unsigned long RAI230:1;
            unsigned long RAI231:1;
            unsigned long RAI232:1;
            unsigned long RAI233:1;
            unsigned long RAI234:1;
            unsigned long RAI235:1;
            unsigned long RAI236:1;
            unsigned long RAI237:1;
            unsigned long RAI238:1;
            unsigned long RAI239:1;
            unsigned long RAI240:1;
            unsigned long RAI241:1;
            unsigned long RAI242:1;
            unsigned long RAI243:1;
            unsigned long RAI244:1;
            unsigned long RAI245:1;
            unsigned long RAI246:1;
            unsigned long RAI247:1;
            unsigned long RAI248:1;
            unsigned long RAI249:1;
            unsigned long RAI250:1;
            unsigned long RAI251:1;
            unsigned long RAI252:1;
            unsigned long RAI253:1;
            unsigned long RAI254:1;
            unsigned long RAI255:1;
        } BIT;
    } RAIS7;
    char           wk1[32];
    union {
        unsigned long LONG;
        struct {
            unsigned long :1;
            unsigned long IEN1:1;
            unsigned long IEN2:1;
            unsigned long IEN3:1;
            unsigned long IEN4:1;
            unsigned long IEN5:1;
            unsigned long IEN6:1;
            unsigned long IEN7:1;
            unsigned long IEN8:1;
            unsigned long IEN9:1;
            unsigned long IEN10:1;
            unsigned long IEN11:1;
            unsigned long IEN12:1;
            unsigned long IEN13:1;
            unsigned long IEN14:1;
            unsigned long IEN15:1;
            unsigned long IEN16:1;
            unsigned long IEN17:1;
            unsigned long IEN18:1;
            unsigned long IEN19:1;
            unsigned long IEN20:1;
            unsigned long IEN21:1;
            unsigned long IEN22:1;
            unsigned long IEN23:1;
            unsigned long IEN24:1;
            unsigned long IEN25:1;
            unsigned long IEN26:1;
            unsigned long IEN27:1;
            unsigned long IEN28:1;
            unsigned long IEN29:1;
            unsigned long IEN30:1;
            unsigned long IEN31:1;
        } BIT;
    } IEN0;
    union {
        unsigned long LONG;
        struct {
            unsigned long IEN32:1;
            unsigned long IEN33:1;
            unsigned long IEN34:1;
            unsigned long IEN35:1;
            unsigned long IEN36:1;
            unsigned long IEN37:1;
            unsigned long IEN38:1;
            unsigned long IEN39:1;
            unsigned long IEN40:1;
            unsigned long IEN41:1;
            unsigned long IEN42:1;
            unsigned long IEN43:1;
            unsigned long IEN44:1;
            unsigned long IEN45:1;
            unsigned long IEN46:1;
            unsigned long IEN47:1;
            unsigned long IEN48:1;
            unsigned long IEN49:1;
            unsigned long IEN50:1;
            unsigned long IEN51:1;
            unsigned long IEN52:1;
            unsigned long IEN53:1;
            unsigned long IEN54:1;
            unsigned long IEN55:1;
            unsigned long IEN56:1;
            unsigned long IEN57:1;
            unsigned long IEN58:1;
            unsigned long IEN59:1;
            unsigned long IEN60:1;
            unsigned long IEN61:1;
            unsigned long IEN62:1;
            unsigned long IEN63:1;
        } BIT;
    } IEN1;
    union {
        unsigned long LONG;
        struct {
            unsigned long IEN64:1;
            unsigned long IEN65:1;
            unsigned long IEN66:1;
            unsigned long IEN67:1;
            unsigned long IEN68:1;
            unsigned long IEN69:1;
            unsigned long IEN70:1;
            unsigned long IEN71:1;
            unsigned long IEN72:1;
            unsigned long IEN73:1;
            unsigned long IEN74:1;
            unsigned long IEN75:1;
            unsigned long IEN76:1;
            unsigned long IEN77:1;
            unsigned long IEN78:1;
            unsigned long IEN79:1;
            unsigned long IEN80:1;
            unsigned long IEN81:1;
            unsigned long IEN82:1;
            unsigned long IEN83:1;
            unsigned long IEN84:1;
            unsigned long IEN85:1;
            unsigned long IEN86:1;
            unsigned long IEN87:1;
            unsigned long IEN88:1;
            unsigned long IEN89:1;
            unsigned long IEN90:1;
            unsigned long IEN91:1;
            unsigned long IEN92:1;
            unsigned long IEN93:1;
            unsigned long IEN94:1;
            unsigned long IEN95:1;
        } BIT;
    } IEN2;
    union {
        unsigned long LONG;
        struct {
            unsigned long IEN96:1;
            unsigned long IEN97:1;
            unsigned long IEN98:1;
            unsigned long IEN99:1;
            unsigned long IEN100:1;
            unsigned long IEN101:1;
            unsigned long IEN102:1;
            unsigned long IEN103:1;
            unsigned long IEN104:1;
            unsigned long IEN105:1;
            unsigned long IEN106:1;
            unsigned long IEN107:1;
            unsigned long IEN108:1;
            unsigned long IEN109:1;
            unsigned long IEN110:1;
            unsigned long IEN111:1;
            unsigned long IEN112:1;
            unsigned long IEN113:1;
            unsigned long IEN114:1;
            unsigned long IEN115:1;
            unsigned long IEN116:1;
            unsigned long IEN117:1;
            unsigned long IEN118:1;
            unsigned long IEN119:1;
            unsigned long IEN120:1;
            unsigned long IEN121:1;
            unsigned long IEN122:1;
            unsigned long IEN123:1;
            unsigned long IEN124:1;
            unsigned long IEN125:1;
            unsigned long IEN126:1;
            unsigned long IEN127:1;
        } BIT;
    } IEN3;
    union {
        unsigned long LONG;
        struct {
            unsigned long IEN128:1;
            unsigned long IEN129:1;
            unsigned long IEN130:1;
            unsigned long IEN131:1;
            unsigned long IEN132:1;
            unsigned long IEN133:1;
            unsigned long IEN134:1;
            unsigned long IEN135:1;
            unsigned long IEN136:1;
            unsigned long IEN137:1;
            unsigned long IEN138:1;
            unsigned long IEN139:1;
            unsigned long IEN140:1;
            unsigned long IEN141:1;
            unsigned long IEN142:1;
            unsigned long IEN143:1;
            unsigned long IEN144:1;
            unsigned long IEN145:1;
            unsigned long IEN146:1;
            unsigned long IEN147:1;
            unsigned long IEN148:1;
            unsigned long IEN149:1;
            unsigned long IEN150:1;
            unsigned long IEN151:1;
            unsigned long IEN152:1;
            unsigned long IEN153:1;
            unsigned long IEN154:1;
            unsigned long IEN155:1;
            unsigned long IEN156:1;
            unsigned long IEN157:1;
            unsigned long IEN158:1;
            unsigned long IEN159:1;
        } BIT;
    } IEN4;
    union {
        unsigned long LONG;
        struct {
            unsigned long IEN160:1;
            unsigned long IEN161:1;
            unsigned long IEN162:1;
            unsigned long IEN163:1;
            unsigned long IEN164:1;
            unsigned long IEN165:1;
            unsigned long IEN166:1;
            unsigned long IEN167:1;
            unsigned long IEN168:1;
            unsigned long IEN169:1;
            unsigned long IEN170:1;
            unsigned long IEN171:1;
            unsigned long IEN172:1;
            unsigned long IEN173:1;
            unsigned long IEN174:1;
            unsigned long IEN175:1;
            unsigned long IEN176:1;
            unsigned long IEN177:1;
            unsigned long IEN178:1;
            unsigned long IEN179:1;
            unsigned long IEN180:1;
            unsigned long IEN181:1;
            unsigned long IEN182:1;
            unsigned long IEN183:1;
            unsigned long IEN184:1;
            unsigned long IEN185:1;
            unsigned long IEN186:1;
            unsigned long IEN187:1;
            unsigned long IEN188:1;
            unsigned long IEN189:1;
            unsigned long IEN190:1;
            unsigned long IEN191:1;
        } BIT;
    } IEN5;
    union {
        unsigned long LONG;
        struct {
            unsigned long IEN192:1;
            unsigned long IEN193:1;
            unsigned long IEN194:1;
            unsigned long IEN195:1;
            unsigned long IEN196:1;
            unsigned long IEN197:1;
            unsigned long IEN198:1;
            unsigned long IEN199:1;
            unsigned long IEN200:1;
            unsigned long IEN201:1;
            unsigned long IEN202:1;
            unsigned long IEN203:1;
            unsigned long IEN204:1;
            unsigned long IEN205:1;
            unsigned long IEN206:1;
            unsigned long IEN207:1;
            unsigned long IEN208:1;
            unsigned long IEN209:1;
            unsigned long IEN210:1;
            unsigned long IEN211:1;
            unsigned long IEN212:1;
            unsigned long IEN213:1;
            unsigned long IEN214:1;
            unsigned long IEN215:1;
            unsigned long IEN216:1;
            unsigned long IEN217:1;
            unsigned long IEN218:1;
            unsigned long IEN219:1;
            unsigned long IEN220:1;
            unsigned long IEN221:1;
            unsigned long IEN222:1;
            unsigned long IEN223:1;
        } BIT;
    } IEN6;
    union {
        unsigned long LONG;
        struct {
            unsigned long IEN224:1;
            unsigned long IEN225:1;
            unsigned long IEN226:1;
            unsigned long IEN227:1;
            unsigned long IEN228:1;
            unsigned long IEN229:1;
            unsigned long IEN230:1;
            unsigned long IEN231:1;
            unsigned long IEN232:1;
            unsigned long IEN233:1;
            unsigned long IEN234:1;
            unsigned long IEN235:1;
            unsigned long IEN236:1;
            unsigned long IEN237:1;
            unsigned long IEN238:1;
            unsigned long IEN239:1;
            unsigned long IEN240:1;
            unsigned long IEN241:1;
            unsigned long IEN242:1;
            unsigned long IEN243:1;
            unsigned long IEN244:1;
            unsigned long IEN245:1;
            unsigned long IEN246:1;
            unsigned long IEN247:1;
            unsigned long IEN248:1;
            unsigned long IEN249:1;
            unsigned long IEN250:1;
            unsigned long IEN251:1;
            unsigned long IEN252:1;
            unsigned long IEN253:1;
            unsigned long IEN254:1;
            unsigned long IEN255:1;
        } BIT;
    } IEN7;
    union {
        unsigned long LONG;
        struct {
            unsigned long :1;
            unsigned long IEC1:1;
            unsigned long IEC2:1;
            unsigned long IEC3:1;
            unsigned long IEC4:1;
            unsigned long IEC5:1;
            unsigned long IEC6:1;
            unsigned long IEC7:1;
            unsigned long IEC8:1;
            unsigned long IEC9:1;
            unsigned long IEC10:1;
            unsigned long IEC11:1;
            unsigned long IEC12:1;
            unsigned long IEC13:1;
            unsigned long IEC14:1;
            unsigned long IEC15:1;
            unsigned long IEC16:1;
            unsigned long IEC17:1;
            unsigned long IEC18:1;
            unsigned long IEC19:1;
            unsigned long IEC20:1;
            unsigned long IEC21:1;
            unsigned long IEC22:1;
            unsigned long IEC23:1;
            unsigned long IEC24:1;
            unsigned long IEC25:1;
            unsigned long IEC26:1;
            unsigned long IEC27:1;
            unsigned long IEC28:1;
            unsigned long IEC29:1;
            unsigned long IEC30:1;
            unsigned long IEC31:1;
        } BIT;
    } IEC0;
    union {
        unsigned long LONG;
        struct {
            unsigned long IEC32:1;
            unsigned long IEC33:1;
            unsigned long IEC34:1;
            unsigned long IEC35:1;
            unsigned long IEC36:1;
            unsigned long IEC37:1;
            unsigned long IEC38:1;
            unsigned long IEC39:1;
            unsigned long IEC40:1;
            unsigned long IEC41:1;
            unsigned long IEC42:1;
            unsigned long IEC43:1;
            unsigned long IEC44:1;
            unsigned long IEC45:1;
            unsigned long IEC46:1;
            unsigned long IEC47:1;
            unsigned long IEC48:1;
            unsigned long IEC49:1;
            unsigned long IEC50:1;
            unsigned long IEC51:1;
            unsigned long IEC52:1;
            unsigned long IEC53:1;
            unsigned long IEC54:1;
            unsigned long IEC55:1;
            unsigned long IEC56:1;
            unsigned long IEC57:1;
            unsigned long IEC58:1;
            unsigned long IEC59:1;
            unsigned long IEC60:1;
            unsigned long IEC61:1;
            unsigned long IEC62:1;
            unsigned long IEC63:1;
        } BIT;
    } IEC1;
    union {
        unsigned long LONG;
        struct {
            unsigned long IEC64:1;
            unsigned long IEC65:1;
            unsigned long IEC66:1;
            unsigned long IEC67:1;
            unsigned long IEC68:1;
            unsigned long IEC69:1;
            unsigned long IEC70:1;
            unsigned long IEC71:1;
            unsigned long IEC72:1;
            unsigned long IEC73:1;
            unsigned long IEC74:1;
            unsigned long IEC75:1;
            unsigned long IEC76:1;
            unsigned long IEC77:1;
            unsigned long IEC78:1;
            unsigned long IEC79:1;
            unsigned long IEC80:1;
            unsigned long IEC81:1;
            unsigned long IEC82:1;
            unsigned long IEC83:1;
            unsigned long IEC84:1;
            unsigned long IEC85:1;
            unsigned long IEC86:1;
            unsigned long IEC87:1;
            unsigned long IEC88:1;
            unsigned long IEC89:1;
            unsigned long IEC90:1;
            unsigned long IEC91:1;
            unsigned long IEC92:1;
            unsigned long IEC93:1;
            unsigned long IEC94:1;
            unsigned long IEC95:1;
        } BIT;
    } IEC2;
    union {
        unsigned long LONG;
        struct {
            unsigned long IEC96:1;
            unsigned long IEC97:1;
            unsigned long IEC98:1;
            unsigned long IEC99:1;
            unsigned long IEC100:1;
            unsigned long IEC101:1;
            unsigned long IEC102:1;
            unsigned long IEC103:1;
            unsigned long IEC104:1;
            unsigned long IEC105:1;
            unsigned long IEC106:1;
            unsigned long IEC107:1;
            unsigned long IEC108:1;
            unsigned long IEC109:1;
            unsigned long IEC110:1;
            unsigned long IEC111:1;
            unsigned long IEC112:1;
            unsigned long IEC113:1;
            unsigned long IEC114:1;
            unsigned long IEC115:1;
            unsigned long IEC116:1;
            unsigned long IEC117:1;
            unsigned long IEC118:1;
            unsigned long IEC119:1;
            unsigned long IEC120:1;
            unsigned long IEC121:1;
            unsigned long IEC122:1;
            unsigned long IEC123:1;
            unsigned long IEC124:1;
            unsigned long IEC125:1;
            unsigned long IEC126:1;
            unsigned long IEC127:1;
        } BIT;
    } IEC3;
    union {
        unsigned long LONG;
        struct {
            unsigned long IEC128:1;
            unsigned long IEC129:1;
            unsigned long IEC130:1;
            unsigned long IEC131:1;
            unsigned long IEC132:1;
            unsigned long IEC133:1;
            unsigned long IEC134:1;
            unsigned long IEC135:1;
            unsigned long IEC136:1;
            unsigned long IEC137:1;
            unsigned long IEC138:1;
            unsigned long IEC139:1;
            unsigned long IEC140:1;
            unsigned long IEC141:1;
            unsigned long IEC142:1;
            unsigned long IEC143:1;
            unsigned long IEC144:1;
            unsigned long IEC145:1;
            unsigned long IEC146:1;
            unsigned long IEC147:1;
            unsigned long IEC148:1;
            unsigned long IEC149:1;
            unsigned long IEC150:1;
            unsigned long IEC151:1;
            unsigned long IEC152:1;
            unsigned long IEC153:1;
            unsigned long IEC154:1;
            unsigned long IEC155:1;
            unsigned long IEC156:1;
            unsigned long IEC157:1;
            unsigned long IEC158:1;
            unsigned long IEC159:1;
        } BIT;
    } IEC4;
    union {
        unsigned long LONG;
        struct {
            unsigned long IEC160:1;
            unsigned long IEC161:1;
            unsigned long IEC162:1;
            unsigned long IEC163:1;
            unsigned long IEC164:1;
            unsigned long IEC165:1;
            unsigned long IEC166:1;
            unsigned long IEC167:1;
            unsigned long IEC168:1;
            unsigned long IEC169:1;
            unsigned long IEC170:1;
            unsigned long IEC171:1;
            unsigned long IEC172:1;
            unsigned long IEC173:1;
            unsigned long IEC174:1;
            unsigned long IEC175:1;
            unsigned long IEC176:1;
            unsigned long IEC177:1;
            unsigned long IEC178:1;
            unsigned long IEC179:1;
            unsigned long IEC180:1;
            unsigned long IEC181:1;
            unsigned long IEC182:1;
            unsigned long IEC183:1;
            unsigned long IEC184:1;
            unsigned long IEC185:1;
            unsigned long IEC186:1;
            unsigned long IEC187:1;
            unsigned long IEC188:1;
            unsigned long IEC189:1;
            unsigned long IEC190:1;
            unsigned long IEC191:1;
        } BIT;
    } IEC5;
    union {
        unsigned long LONG;
        struct {
            unsigned long IEC192:1;
            unsigned long IEC193:1;
            unsigned long IEC194:1;
            unsigned long IEC195:1;
            unsigned long IEC196:1;
            unsigned long IEC197:1;
            unsigned long IEC198:1;
            unsigned long IEC199:1;
            unsigned long IEC200:1;
            unsigned long IEC201:1;
            unsigned long IEC202:1;
            unsigned long IEC203:1;
            unsigned long IEC204:1;
            unsigned long IEC205:1;
            unsigned long IEC206:1;
            unsigned long IEC207:1;
            unsigned long IEC208:1;
            unsigned long IEC209:1;
            unsigned long IEC210:1;
            unsigned long IEC211:1;
            unsigned long IEC212:1;
            unsigned long IEC213:1;
            unsigned long IEC214:1;
            unsigned long IEC215:1;
            unsigned long IEC216:1;
            unsigned long IEC217:1;
            unsigned long IEC218:1;
            unsigned long IEC219:1;
            unsigned long IEC220:1;
            unsigned long IEC221:1;
            unsigned long IEC222:1;
            unsigned long IEC223:1;
        } BIT;
    } IEC6;
    union {
        unsigned long LONG;
        struct {
            unsigned long IEC224:1;
            unsigned long IEC225:1;
            unsigned long IEC226:1;
            unsigned long IEC227:1;
            unsigned long IEC228:1;
            unsigned long IEC229:1;
            unsigned long IEC230:1;
            unsigned long IEC231:1;
            unsigned long IEC232:1;
            unsigned long IEC233:1;
            unsigned long IEC234:1;
            unsigned long IEC235:1;
            unsigned long IEC236:1;
            unsigned long IEC237:1;
            unsigned long IEC238:1;
            unsigned long IEC239:1;
            unsigned long IEC240:1;
            unsigned long IEC241:1;
            unsigned long IEC242:1;
            unsigned long IEC243:1;
            unsigned long IEC244:1;
            unsigned long IEC245:1;
            unsigned long IEC246:1;
            unsigned long IEC247:1;
            unsigned long IEC248:1;
            unsigned long IEC249:1;
            unsigned long IEC250:1;
            unsigned long IEC251:1;
            unsigned long IEC252:1;
            unsigned long IEC253:1;
            unsigned long IEC254:1;
            unsigned long IEC255:1;
        } BIT;
    } IEC7;
    char           wk2[64];
    union {
        unsigned long LONG;
        struct {
            unsigned long :1;
            unsigned long PLS1:1;
            unsigned long PLS2:1;
            unsigned long PLS3:1;
            unsigned long PLS4:1;
            unsigned long PLS5:1;
            unsigned long PLS6:1;
            unsigned long PLS7:1;
            unsigned long PLS8:1;
            unsigned long PLS9:1;
            unsigned long PLS10:1;
            unsigned long PLS11:1;
            unsigned long PLS12:1;
            unsigned long PLS13:1;
            unsigned long PLS14:1;
            unsigned long PLS15:1;
            unsigned long PLS16:1;
            unsigned long PLS17:1;
            unsigned long PLS18:1;
            unsigned long PLS19:1;
            unsigned long PLS20:1;
            unsigned long PLS21:1;
            unsigned long PLS22:1;
            unsigned long PLS23:1;
            unsigned long PLS24:1;
            unsigned long PLS25:1;
            unsigned long PLS26:1;
            unsigned long PLS27:1;
            unsigned long PLS28:1;
            unsigned long PLS29:1;
            unsigned long PLS30:1;
            unsigned long PLS31:1;
        } BIT;
    } PLS0;
    union {
        unsigned long LONG;
        struct {
            unsigned long PLS32:1;
            unsigned long PLS33:1;
            unsigned long PLS34:1;
            unsigned long PLS35:1;
            unsigned long PLS36:1;
            unsigned long PLS37:1;
            unsigned long PLS38:1;
            unsigned long PLS39:1;
            unsigned long PLS40:1;
            unsigned long PLS41:1;
            unsigned long PLS42:1;
            unsigned long PLS43:1;
            unsigned long PLS44:1;
            unsigned long PLS45:1;
            unsigned long PLS46:1;
            unsigned long PLS47:1;
            unsigned long PLS48:1;
            unsigned long PLS49:1;
            unsigned long PLS50:1;
            unsigned long PLS51:1;
            unsigned long PLS52:1;
            unsigned long PLS53:1;
            unsigned long PLS54:1;
            unsigned long PLS55:1;
            unsigned long PLS56:1;
            unsigned long PLS57:1;
            unsigned long PLS58:1;
            unsigned long PLS59:1;
            unsigned long PLS60:1;
            unsigned long PLS61:1;
            unsigned long PLS62:1;
            unsigned long PLS63:1;
        } BIT;
    } PLS1;
    union {
        unsigned long LONG;
        struct {
            unsigned long PLS64:1;
            unsigned long PLS65:1;
            unsigned long PLS66:1;
            unsigned long PLS67:1;
            unsigned long PLS68:1;
            unsigned long PLS69:1;
            unsigned long PLS70:1;
            unsigned long PLS71:1;
            unsigned long PLS72:1;
            unsigned long PLS73:1;
            unsigned long PLS74:1;
            unsigned long PLS75:1;
            unsigned long PLS76:1;
            unsigned long PLS77:1;
            unsigned long PLS78:1;
            unsigned long PLS79:1;
            unsigned long PLS80:1;
            unsigned long PLS81:1;
            unsigned long PLS82:1;
            unsigned long PLS83:1;
            unsigned long PLS84:1;
            unsigned long PLS85:1;
            unsigned long PLS86:1;
            unsigned long PLS87:1;
            unsigned long PLS88:1;
            unsigned long PLS89:1;
            unsigned long PLS90:1;
            unsigned long PLS91:1;
            unsigned long PLS92:1;
            unsigned long PLS93:1;
            unsigned long PLS94:1;
            unsigned long PLS95:1;
        } BIT;
    } PLS2;
    union {
        unsigned long LONG;
        struct {
            unsigned long PLS96:1;
            unsigned long PLS97:1;
            unsigned long PLS98:1;
            unsigned long PLS99:1;
            unsigned long PLS100:1;
            unsigned long PLS101:1;
            unsigned long PLS102:1;
            unsigned long PLS103:1;
            unsigned long PLS104:1;
            unsigned long PLS105:1;
            unsigned long PLS106:1;
            unsigned long PLS107:1;
            unsigned long PLS108:1;
            unsigned long PLS109:1;
            unsigned long PLS110:1;
            unsigned long PLS111:1;
            unsigned long PLS112:1;
            unsigned long PLS113:1;
            unsigned long PLS114:1;
            unsigned long PLS115:1;
            unsigned long PLS116:1;
            unsigned long PLS117:1;
            unsigned long PLS118:1;
            unsigned long PLS119:1;
            unsigned long PLS120:1;
            unsigned long PLS121:1;
            unsigned long PLS122:1;
            unsigned long PLS123:1;
            unsigned long PLS124:1;
            unsigned long PLS125:1;
            unsigned long PLS126:1;
            unsigned long PLS127:1;
        } BIT;
    } PLS3;
    union {
        unsigned long LONG;
        struct {
            unsigned long PLS128:1;
            unsigned long PLS129:1;
            unsigned long PLS130:1;
            unsigned long PLS131:1;
            unsigned long PLS132:1;
            unsigned long PLS133:1;
            unsigned long PLS134:1;
            unsigned long PLS135:1;
            unsigned long PLS136:1;
            unsigned long PLS137:1;
            unsigned long PLS138:1;
            unsigned long PLS139:1;
            unsigned long PLS140:1;
            unsigned long PLS141:1;
            unsigned long PLS142:1;
            unsigned long PLS143:1;
            unsigned long PLS144:1;
            unsigned long PLS145:1;
            unsigned long PLS146:1;
            unsigned long PLS147:1;
            unsigned long PLS148:1;
            unsigned long PLS149:1;
            unsigned long PLS150:1;
            unsigned long PLS151:1;
            unsigned long PLS152:1;
            unsigned long PLS153:1;
            unsigned long PLS154:1;
            unsigned long PLS155:1;
            unsigned long PLS156:1;
            unsigned long PLS157:1;
            unsigned long PLS158:1;
            unsigned long PLS159:1;
        } BIT;
    } PLS4;
    union {
        unsigned long LONG;
        struct {
            unsigned long PLS160:1;
            unsigned long PLS161:1;
            unsigned long PLS162:1;
            unsigned long PLS163:1;
            unsigned long PLS164:1;
            unsigned long PLS165:1;
            unsigned long PLS166:1;
            unsigned long PLS167:1;
            unsigned long PLS168:1;
            unsigned long PLS169:1;
            unsigned long PLS170:1;
            unsigned long PLS171:1;
            unsigned long PLS172:1;
            unsigned long PLS173:1;
            unsigned long PLS174:1;
            unsigned long PLS175:1;
            unsigned long PLS176:1;
            unsigned long PLS177:1;
            unsigned long PLS178:1;
            unsigned long PLS179:1;
            unsigned long PLS180:1;
            unsigned long PLS181:1;
            unsigned long PLS182:1;
            unsigned long PLS183:1;
            unsigned long PLS184:1;
            unsigned long PLS185:1;
            unsigned long PLS186:1;
            unsigned long PLS187:1;
            unsigned long PLS188:1;
            unsigned long PLS189:1;
            unsigned long PLS190:1;
            unsigned long PLS191:1;
        } BIT;
    } PLS5;
    union {
        unsigned long LONG;
        struct {
            unsigned long PLS192:1;
            unsigned long PLS193:1;
            unsigned long PLS194:1;
            unsigned long PLS195:1;
            unsigned long PLS196:1;
            unsigned long PLS197:1;
            unsigned long PLS198:1;
            unsigned long PLS199:1;
            unsigned long PLS200:1;
            unsigned long PLS201:1;
            unsigned long PLS202:1;
            unsigned long PLS203:1;
            unsigned long PLS204:1;
            unsigned long PLS205:1;
            unsigned long PLS206:1;
            unsigned long PLS207:1;
            unsigned long PLS208:1;
            unsigned long PLS209:1;
            unsigned long PLS210:1;
            unsigned long PLS211:1;
            unsigned long PLS212:1;
            unsigned long PLS213:1;
            unsigned long PLS214:1;
            unsigned long PLS215:1;
            unsigned long PLS216:1;
            unsigned long PLS217:1;
            unsigned long PLS218:1;
            unsigned long PLS219:1;
            unsigned long PLS220:1;
            unsigned long PLS221:1;
            unsigned long PLS222:1;
            unsigned long PLS223:1;
        } BIT;
    } PLS6;
    union {
        unsigned long LONG;
        struct {
            unsigned long PLS224:1;
            unsigned long PLS225:1;
            unsigned long PLS226:1;
            unsigned long PLS227:1;
            unsigned long PLS228:1;
            unsigned long PLS229:1;
            unsigned long PLS230:1;
            unsigned long PLS231:1;
            unsigned long PLS232:1;
            unsigned long PLS233:1;
            unsigned long PLS234:1;
            unsigned long PLS235:1;
            unsigned long PLS236:1;
            unsigned long PLS237:1;
            unsigned long PLS238:1;
            unsigned long PLS239:1;
            unsigned long PLS240:1;
            unsigned long PLS241:1;
            unsigned long PLS242:1;
            unsigned long PLS243:1;
            unsigned long PLS244:1;
            unsigned long PLS245:1;
            unsigned long PLS246:1;
            unsigned long PLS247:1;
            unsigned long PLS248:1;
            unsigned long PLS249:1;
            unsigned long PLS250:1;
            unsigned long PLS251:1;
            unsigned long PLS252:1;
            unsigned long PLS253:1;
            unsigned long PLS254:1;
            unsigned long PLS255:1;
        } BIT;
    } PLS7;
    union {
        unsigned long LONG;
        struct {
            unsigned long :1;
            unsigned long PIC1:1;
            unsigned long PIC2:1;
            unsigned long PIC3:1;
            unsigned long PIC4:1;
            unsigned long PIC5:1;
            unsigned long PIC6:1;
            unsigned long PIC7:1;
            unsigned long PIC8:1;
            unsigned long PIC9:1;
            unsigned long PIC10:1;
            unsigned long PIC11:1;
            unsigned long PIC12:1;
            unsigned long PIC13:1;
            unsigned long PIC14:1;
            unsigned long PIC15:1;
            unsigned long PIC16:1;
            unsigned long PIC17:1;
            unsigned long PIC18:1;
            unsigned long PIC19:1;
            unsigned long PIC20:1;
            unsigned long PIC21:1;
            unsigned long PIC22:1;
            unsigned long PIC23:1;
            unsigned long PIC24:1;
            unsigned long PIC25:1;
            unsigned long PIC26:1;
            unsigned long PIC27:1;
            unsigned long PIC28:1;
            unsigned long PIC29:1;
            unsigned long PIC30:1;
            unsigned long PIC31:1;
        } BIT;
    } PIC0;
    union {
        unsigned long LONG;
        struct {
            unsigned long PIC32:1;
            unsigned long PIC33:1;
            unsigned long PIC34:1;
            unsigned long PIC35:1;
            unsigned long PIC36:1;
            unsigned long PIC37:1;
            unsigned long PIC38:1;
            unsigned long PIC39:1;
            unsigned long PIC40:1;
            unsigned long PIC41:1;
            unsigned long PIC42:1;
            unsigned long PIC43:1;
            unsigned long PIC44:1;
            unsigned long PIC45:1;
            unsigned long PIC46:1;
            unsigned long PIC47:1;
            unsigned long PIC48:1;
            unsigned long PIC49:1;
            unsigned long PIC50:1;
            unsigned long PIC51:1;
            unsigned long PIC52:1;
            unsigned long PIC53:1;
            unsigned long PIC54:1;
            unsigned long PIC55:1;
            unsigned long PIC56:1;
            unsigned long PIC57:1;
            unsigned long PIC58:1;
            unsigned long PIC59:1;
            unsigned long PIC60:1;
            unsigned long PIC61:1;
            unsigned long PIC62:1;
            unsigned long PIC63:1;
        } BIT;
    } PIC1;
    union {
        unsigned long LONG;
        struct {
            unsigned long PIC64:1;
            unsigned long PIC65:1;
            unsigned long PIC66:1;
            unsigned long PIC67:1;
            unsigned long PIC68:1;
            unsigned long PIC69:1;
            unsigned long PIC70:1;
            unsigned long PIC71:1;
            unsigned long PIC72:1;
            unsigned long PIC73:1;
            unsigned long PIC74:1;
            unsigned long PIC75:1;
            unsigned long PIC76:1;
            unsigned long PIC77:1;
            unsigned long PIC78:1;
            unsigned long PIC79:1;
            unsigned long PIC80:1;
            unsigned long PIC81:1;
            unsigned long PIC82:1;
            unsigned long PIC83:1;
            unsigned long PIC84:1;
            unsigned long PIC85:1;
            unsigned long PIC86:1;
            unsigned long PIC87:1;
            unsigned long PIC88:1;
            unsigned long PIC89:1;
            unsigned long PIC90:1;
            unsigned long PIC91:1;
            unsigned long PIC92:1;
            unsigned long PIC93:1;
            unsigned long PIC94:1;
            unsigned long PIC95:1;
        } BIT;
    } PIC2;
    union {
        unsigned long LONG;
        struct {
            unsigned long PIC96:1;
            unsigned long PIC97:1;
            unsigned long PIC98:1;
            unsigned long PIC99:1;
            unsigned long PIC100:1;
            unsigned long PIC101:1;
            unsigned long PIC102:1;
            unsigned long PIC103:1;
            unsigned long PIC104:1;
            unsigned long PIC105:1;
            unsigned long PIC106:1;
            unsigned long PIC107:1;
            unsigned long PIC108:1;
            unsigned long PIC109:1;
            unsigned long PIC110:1;
            unsigned long PIC111:1;
            unsigned long PIC112:1;
            unsigned long PIC113:1;
            unsigned long PIC114:1;
            unsigned long PIC115:1;
            unsigned long PIC116:1;
            unsigned long PIC117:1;
            unsigned long PIC118:1;
            unsigned long PIC119:1;
            unsigned long PIC120:1;
            unsigned long PIC121:1;
            unsigned long PIC122:1;
            unsigned long PIC123:1;
            unsigned long PIC124:1;
            unsigned long PIC125:1;
            unsigned long PIC126:1;
            unsigned long PIC127:1;
        } BIT;
    } PIC3;
    union {
        unsigned long LONG;
        struct {
            unsigned long PIC128:1;
            unsigned long PIC129:1;
            unsigned long PIC130:1;
            unsigned long PIC131:1;
            unsigned long PIC132:1;
            unsigned long PIC133:1;
            unsigned long PIC134:1;
            unsigned long PIC135:1;
            unsigned long PIC136:1;
            unsigned long PIC137:1;
            unsigned long PIC138:1;
            unsigned long PIC139:1;
            unsigned long PIC140:1;
            unsigned long PIC141:1;
            unsigned long PIC142:1;
            unsigned long PIC143:1;
            unsigned long PIC144:1;
            unsigned long PIC145:1;
            unsigned long PIC146:1;
            unsigned long PIC147:1;
            unsigned long PIC148:1;
            unsigned long PIC149:1;
            unsigned long PIC150:1;
            unsigned long PIC151:1;
            unsigned long PIC152:1;
            unsigned long PIC153:1;
            unsigned long PIC154:1;
            unsigned long PIC155:1;
            unsigned long PIC156:1;
            unsigned long PIC157:1;
            unsigned long PIC158:1;
            unsigned long PIC159:1;
        } BIT;
    } PIC4;
    union {
        unsigned long LONG;
        struct {
            unsigned long PIC160:1;
            unsigned long PIC161:1;
            unsigned long PIC162:1;
            unsigned long PIC163:1;
            unsigned long PIC164:1;
            unsigned long PIC165:1;
            unsigned long PIC166:1;
            unsigned long PIC167:1;
            unsigned long PIC168:1;
            unsigned long PIC169:1;
            unsigned long PIC170:1;
            unsigned long PIC171:1;
            unsigned long PIC172:1;
            unsigned long PIC173:1;
            unsigned long PIC174:1;
            unsigned long PIC175:1;
            unsigned long PIC176:1;
            unsigned long PIC177:1;
            unsigned long PIC178:1;
            unsigned long PIC179:1;
            unsigned long PIC180:1;
            unsigned long PIC181:1;
            unsigned long PIC182:1;
            unsigned long PIC183:1;
            unsigned long PIC184:1;
            unsigned long PIC185:1;
            unsigned long PIC186:1;
            unsigned long PIC187:1;
            unsigned long PIC188:1;
            unsigned long PIC189:1;
            unsigned long PIC190:1;
            unsigned long PIC191:1;
        } BIT;
    } PIC5;
    union {
        unsigned long LONG;
        struct {
            unsigned long PIC192:1;
            unsigned long PIC193:1;
            unsigned long PIC194:1;
            unsigned long PIC195:1;
            unsigned long PIC196:1;
            unsigned long PIC197:1;
            unsigned long PIC198:1;
            unsigned long PIC199:1;
            unsigned long PIC200:1;
            unsigned long PIC201:1;
            unsigned long PIC202:1;
            unsigned long PIC203:1;
            unsigned long PIC204:1;
            unsigned long PIC205:1;
            unsigned long PIC206:1;
            unsigned long PIC207:1;
            unsigned long PIC208:1;
            unsigned long PIC209:1;
            unsigned long PIC210:1;
            unsigned long PIC211:1;
            unsigned long PIC212:1;
            unsigned long PIC213:1;
            unsigned long PIC214:1;
            unsigned long PIC215:1;
            unsigned long PIC216:1;
            unsigned long PIC217:1;
            unsigned long PIC218:1;
            unsigned long PIC219:1;
            unsigned long PIC220:1;
            unsigned long PIC221:1;
            unsigned long PIC222:1;
            unsigned long PIC223:1;
        } BIT;
    } PIC6;
    union {
        unsigned long LONG;
        struct {
            unsigned long PIC224:1;
            unsigned long PIC225:1;
            unsigned long PIC226:1;
            unsigned long PIC227:1;
            unsigned long PIC228:1;
            unsigned long PIC229:1;
            unsigned long PIC230:1;
            unsigned long PIC231:1;
            unsigned long PIC232:1;
            unsigned long PIC233:1;
            unsigned long PIC234:1;
            unsigned long PIC235:1;
            unsigned long PIC236:1;
            unsigned long PIC237:1;
            unsigned long PIC238:1;
            unsigned long PIC239:1;
            unsigned long PIC240:1;
            unsigned long PIC241:1;
            unsigned long PIC242:1;
            unsigned long PIC243:1;
            unsigned long PIC244:1;
            unsigned long PIC245:1;
            unsigned long PIC246:1;
            unsigned long PIC247:1;
            unsigned long PIC248:1;
            unsigned long PIC249:1;
            unsigned long PIC250:1;
            unsigned long PIC251:1;
            unsigned long PIC252:1;
            unsigned long PIC253:1;
            unsigned long PIC254:1;
            unsigned long PIC255:1;
        } BIT;
    } PIC7;
    char           wk3[128];
    union {
        unsigned long LONG;
        struct {
            unsigned long PRLM0:1;
            unsigned long PRLM1:1;
            unsigned long PRLM2:1;
            unsigned long PRLM3:1;
            unsigned long PRLM4:1;
            unsigned long PRLM5:1;
            unsigned long PRLM6:1;
            unsigned long PRLM7:1;
            unsigned long PRLM8:1;
            unsigned long PRLM9:1;
            unsigned long PRLM10:1;
            unsigned long PRLM11:1;
            unsigned long PRLM12:1;
            unsigned long PRLM13:1;
            unsigned long PRLM14:1;
            unsigned long PRLM15:1;
            unsigned long :16;
        } BIT;
    } PRLM0;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRLC0:1;
            unsigned long PRLC1:1;
            unsigned long PRLC2:1;
            unsigned long PRLC3:1;
            unsigned long PRLC4:1;
            unsigned long PRLC5:1;
            unsigned long PRLC6:1;
            unsigned long PRLC7:1;
            unsigned long PRLC8:1;
            unsigned long PRLC9:1;
            unsigned long PRLC10:1;
            unsigned long PRLC11:1;
            unsigned long PRLC12:1;
            unsigned long PRLC13:1;
            unsigned long PRLC14:1;
            unsigned long PRLC15:1;
            unsigned long :16;
        } BIT;
    } PRLC0;
    union {
        unsigned long LONG;
        struct {
            unsigned long UE:1;
            unsigned long :31;
        } BIT;
    } UEN0;
    char           wk4[52];
    union {
        unsigned long LONG;
    } HVA0;
    char           wk5[12];
    union {
        unsigned long LONG;
        struct {
            unsigned long :1;
            unsigned long ISS1:1;
            unsigned long ISS2:1;
            unsigned long ISS3:1;
            unsigned long ISS4:1;
            unsigned long ISS5:1;
            unsigned long ISS6:1;
            unsigned long ISS7:1;
            unsigned long ISS8:1;
            unsigned long ISS9:1;
            unsigned long ISS10:1;
            unsigned long ISS11:1;
            unsigned long ISS12:1;
            unsigned long ISS13:1;
            unsigned long ISS14:1;
            unsigned long ISS15:1;
            unsigned long ISS16:1;
            unsigned long ISS17:1;
            unsigned long ISS18:1;
            unsigned long ISS19:1;
            unsigned long ISS20:1;
            unsigned long ISS21:1;
            unsigned long ISS22:1;
            unsigned long ISS23:1;
            unsigned long ISS24:1;
            unsigned long ISS25:1;
            unsigned long ISS26:1;
            unsigned long ISS27:1;
            unsigned long ISS28:1;
            unsigned long ISS29:1;
            unsigned long ISS30:1;
            unsigned long ISS31:1;
        } BIT;
    } ISS0;
    union {
        unsigned long LONG;
        struct {
            unsigned long ISS32:1;
            unsigned long ISS33:1;
            unsigned long ISS34:1;
            unsigned long ISS35:1;
            unsigned long ISS36:1;
            unsigned long ISS37:1;
            unsigned long ISS38:1;
            unsigned long ISS39:1;
            unsigned long ISS40:1;
            unsigned long ISS41:1;
            unsigned long ISS42:1;
            unsigned long ISS43:1;
            unsigned long ISS44:1;
            unsigned long ISS45:1;
            unsigned long ISS46:1;
            unsigned long ISS47:1;
            unsigned long ISS48:1;
            unsigned long ISS49:1;
            unsigned long ISS50:1;
            unsigned long ISS51:1;
            unsigned long ISS52:1;
            unsigned long ISS53:1;
            unsigned long ISS54:1;
            unsigned long ISS55:1;
            unsigned long ISS56:1;
            unsigned long ISS57:1;
            unsigned long ISS58:1;
            unsigned long ISS59:1;
            unsigned long ISS60:1;
            unsigned long ISS61:1;
            unsigned long ISS62:1;
            unsigned long ISS63:1;
        } BIT;
    } ISS1;
    union {
        unsigned long LONG;
        struct {
            unsigned long ISS64:1;
            unsigned long ISS65:1;
            unsigned long ISS66:1;
            unsigned long ISS67:1;
            unsigned long ISS68:1;
            unsigned long ISS69:1;
            unsigned long ISS70:1;
            unsigned long ISS71:1;
            unsigned long ISS72:1;
            unsigned long ISS73:1;
            unsigned long ISS74:1;
            unsigned long ISS75:1;
            unsigned long ISS76:1;
            unsigned long ISS77:1;
            unsigned long ISS78:1;
            unsigned long ISS79:1;
            unsigned long ISS80:1;
            unsigned long ISS81:1;
            unsigned long ISS82:1;
            unsigned long ISS83:1;
            unsigned long ISS84:1;
            unsigned long ISS85:1;
            unsigned long ISS86:1;
            unsigned long ISS87:1;
            unsigned long ISS88:1;
            unsigned long ISS89:1;
            unsigned long ISS90:1;
            unsigned long ISS91:1;
            unsigned long ISS92:1;
            unsigned long ISS93:1;
            unsigned long ISS94:1;
            unsigned long ISS95:1;
        } BIT;
    } ISS2;
    union {
        unsigned long LONG;
        struct {
            unsigned long ISS96:1;
            unsigned long ISS97:1;
            unsigned long ISS98:1;
            unsigned long ISS99:1;
            unsigned long ISS100:1;
            unsigned long ISS101:1;
            unsigned long ISS102:1;
            unsigned long ISS103:1;
            unsigned long ISS104:1;
            unsigned long ISS105:1;
            unsigned long ISS106:1;
            unsigned long ISS107:1;
            unsigned long ISS108:1;
            unsigned long ISS109:1;
            unsigned long ISS110:1;
            unsigned long ISS111:1;
            unsigned long ISS112:1;
            unsigned long ISS113:1;
            unsigned long ISS114:1;
            unsigned long ISS115:1;
            unsigned long ISS116:1;
            unsigned long ISS117:1;
            unsigned long ISS118:1;
            unsigned long ISS119:1;
            unsigned long ISS120:1;
            unsigned long ISS121:1;
            unsigned long ISS122:1;
            unsigned long ISS123:1;
            unsigned long ISS124:1;
            unsigned long ISS125:1;
            unsigned long ISS126:1;
            unsigned long ISS127:1;
        } BIT;
    } ISS3;
    union {
        unsigned long LONG;
        struct {
            unsigned long ISS128:1;
            unsigned long ISS129:1;
            unsigned long ISS130:1;
            unsigned long ISS131:1;
            unsigned long ISS132:1;
            unsigned long ISS133:1;
            unsigned long ISS134:1;
            unsigned long ISS135:1;
            unsigned long ISS136:1;
            unsigned long ISS137:1;
            unsigned long ISS138:1;
            unsigned long ISS139:1;
            unsigned long ISS140:1;
            unsigned long ISS141:1;
            unsigned long ISS142:1;
            unsigned long ISS143:1;
            unsigned long ISS144:1;
            unsigned long ISS145:1;
            unsigned long ISS146:1;
            unsigned long ISS147:1;
            unsigned long ISS148:1;
            unsigned long ISS149:1;
            unsigned long ISS150:1;
            unsigned long ISS151:1;
            unsigned long ISS152:1;
            unsigned long ISS153:1;
            unsigned long ISS154:1;
            unsigned long ISS155:1;
            unsigned long ISS156:1;
            unsigned long ISS157:1;
            unsigned long ISS158:1;
            unsigned long ISS159:1;
        } BIT;
    } ISS4;
    union {
        unsigned long LONG;
        struct {
            unsigned long ISS160:1;
            unsigned long ISS161:1;
            unsigned long ISS162:1;
            unsigned long ISS163:1;
            unsigned long ISS164:1;
            unsigned long ISS165:1;
            unsigned long ISS166:1;
            unsigned long ISS167:1;
            unsigned long ISS168:1;
            unsigned long ISS169:1;
            unsigned long ISS170:1;
            unsigned long ISS171:1;
            unsigned long ISS172:1;
            unsigned long ISS173:1;
            unsigned long ISS174:1;
            unsigned long ISS175:1;
            unsigned long ISS176:1;
            unsigned long ISS177:1;
            unsigned long ISS178:1;
            unsigned long ISS179:1;
            unsigned long ISS180:1;
            unsigned long ISS181:1;
            unsigned long ISS182:1;
            unsigned long ISS183:1;
            unsigned long ISS184:1;
            unsigned long ISS185:1;
            unsigned long ISS186:1;
            unsigned long ISS187:1;
            unsigned long ISS188:1;
            unsigned long ISS189:1;
            unsigned long ISS190:1;
            unsigned long ISS191:1;
        } BIT;
    } ISS5;
    union {
        unsigned long LONG;
        struct {
            unsigned long ISS192:1;
            unsigned long ISS193:1;
            unsigned long ISS194:1;
            unsigned long ISS195:1;
            unsigned long ISS196:1;
            unsigned long ISS197:1;
            unsigned long ISS198:1;
            unsigned long ISS199:1;
            unsigned long ISS200:1;
            unsigned long ISS201:1;
            unsigned long ISS202:1;
            unsigned long ISS203:1;
            unsigned long ISS204:1;
            unsigned long ISS205:1;
            unsigned long ISS206:1;
            unsigned long ISS207:1;
            unsigned long ISS208:1;
            unsigned long ISS209:1;
            unsigned long ISS210:1;
            unsigned long ISS211:1;
            unsigned long ISS212:1;
            unsigned long ISS213:1;
            unsigned long ISS214:1;
            unsigned long ISS215:1;
            unsigned long ISS216:1;
            unsigned long ISS217:1;
            unsigned long ISS218:1;
            unsigned long ISS219:1;
            unsigned long ISS220:1;
            unsigned long ISS221:1;
            unsigned long ISS222:1;
            unsigned long ISS223:1;
        } BIT;
    } ISS6;
    union {
        unsigned long LONG;
        struct {
            unsigned long ISS224:1;
            unsigned long ISS225:1;
            unsigned long ISS226:1;
            unsigned long ISS227:1;
            unsigned long ISS228:1;
            unsigned long ISS229:1;
            unsigned long ISS230:1;
            unsigned long ISS231:1;
            unsigned long ISS232:1;
            unsigned long ISS233:1;
            unsigned long ISS234:1;
            unsigned long ISS235:1;
            unsigned long ISS236:1;
            unsigned long ISS237:1;
            unsigned long ISS238:1;
            unsigned long ISS239:1;
            unsigned long ISS240:1;
            unsigned long ISS241:1;
            unsigned long ISS242:1;
            unsigned long ISS243:1;
            unsigned long ISS244:1;
            unsigned long ISS245:1;
            unsigned long ISS246:1;
            unsigned long ISS247:1;
            unsigned long ISS248:1;
            unsigned long ISS249:1;
            unsigned long ISS250:1;
            unsigned long ISS251:1;
            unsigned long ISS252:1;
            unsigned long ISS253:1;
            unsigned long ISS254:1;
            unsigned long ISS255:1;
        } BIT;
    } ISS7;
    union {
        unsigned long LONG;
        struct {
            unsigned long :1;
            unsigned long ISC1:1;
            unsigned long ISC2:1;
            unsigned long ISC3:1;
            unsigned long ISC4:1;
            unsigned long ISC5:1;
            unsigned long ISC6:1;
            unsigned long ISC7:1;
            unsigned long ISC8:1;
            unsigned long ISC9:1;
            unsigned long ISC10:1;
            unsigned long ISC11:1;
            unsigned long ISC12:1;
            unsigned long ISC13:1;
            unsigned long ISC14:1;
            unsigned long ISC15:1;
            unsigned long ISC16:1;
            unsigned long ISC17:1;
            unsigned long ISC18:1;
            unsigned long ISC19:1;
            unsigned long ISC20:1;
            unsigned long ISC21:1;
            unsigned long ISC22:1;
            unsigned long ISC23:1;
            unsigned long ISC24:1;
            unsigned long ISC25:1;
            unsigned long ISC26:1;
            unsigned long ISC27:1;
            unsigned long ISC28:1;
            unsigned long ISC29:1;
            unsigned long ISC30:1;
            unsigned long ISC31:1;
        } BIT;
    } ISC0;
    union {
        unsigned long LONG;
        struct {
            unsigned long ISC32:1;
            unsigned long ISC33:1;
            unsigned long ISC34:1;
            unsigned long ISC35:1;
            unsigned long ISC36:1;
            unsigned long ISC37:1;
            unsigned long ISC38:1;
            unsigned long ISC39:1;
            unsigned long ISC40:1;
            unsigned long ISC41:1;
            unsigned long ISC42:1;
            unsigned long ISC43:1;
            unsigned long ISC44:1;
            unsigned long ISC45:1;
            unsigned long ISC46:1;
            unsigned long ISC47:1;
            unsigned long ISC48:1;
            unsigned long ISC49:1;
            unsigned long ISC50:1;
            unsigned long ISC51:1;
            unsigned long ISC52:1;
            unsigned long ISC53:1;
            unsigned long ISC54:1;
            unsigned long ISC55:1;
            unsigned long ISC56:1;
            unsigned long ISC57:1;
            unsigned long ISC58:1;
            unsigned long ISC59:1;
            unsigned long ISC60:1;
            unsigned long ISC61:1;
            unsigned long ISC62:1;
            unsigned long ISC63:1;
        } BIT;
    } ISC1;
    union {
        unsigned long LONG;
        struct {
            unsigned long ISC64:1;
            unsigned long ISC65:1;
            unsigned long ISC66:1;
            unsigned long ISC67:1;
            unsigned long ISC68:1;
            unsigned long ISC69:1;
            unsigned long ISC70:1;
            unsigned long ISC71:1;
            unsigned long ISC72:1;
            unsigned long ISC73:1;
            unsigned long ISC74:1;
            unsigned long ISC75:1;
            unsigned long ISC76:1;
            unsigned long ISC77:1;
            unsigned long ISC78:1;
            unsigned long ISC79:1;
            unsigned long ISC80:1;
            unsigned long ISC81:1;
            unsigned long ISC82:1;
            unsigned long ISC83:1;
            unsigned long ISC84:1;
            unsigned long ISC85:1;
            unsigned long ISC86:1;
            unsigned long ISC87:1;
            unsigned long ISC88:1;
            unsigned long ISC89:1;
            unsigned long ISC90:1;
            unsigned long ISC91:1;
            unsigned long ISC92:1;
            unsigned long ISC93:1;
            unsigned long ISC94:1;
            unsigned long ISC95:1;
        } BIT;
    } ISC2;
    union {
        unsigned long LONG;
        struct {
            unsigned long ISC96:1;
            unsigned long ISC97:1;
            unsigned long ISC98:1;
            unsigned long ISC99:1;
            unsigned long ISC100:1;
            unsigned long ISC101:1;
            unsigned long ISC102:1;
            unsigned long ISC103:1;
            unsigned long ISC104:1;
            unsigned long ISC105:1;
            unsigned long ISC106:1;
            unsigned long ISC107:1;
            unsigned long ISC108:1;
            unsigned long ISC109:1;
            unsigned long ISC110:1;
            unsigned long ISC111:1;
            unsigned long ISC112:1;
            unsigned long ISC113:1;
            unsigned long ISC114:1;
            unsigned long ISC115:1;
            unsigned long ISC116:1;
            unsigned long ISC117:1;
            unsigned long ISC118:1;
            unsigned long ISC119:1;
            unsigned long ISC120:1;
            unsigned long ISC121:1;
            unsigned long ISC122:1;
            unsigned long ISC123:1;
            unsigned long ISC124:1;
            unsigned long ISC125:1;
            unsigned long ISC126:1;
            unsigned long ISC127:1;
        } BIT;
    } ISC3;
    union {
        unsigned long LONG;
        struct {
            unsigned long ISC128:1;
            unsigned long ISC129:1;
            unsigned long ISC130:1;
            unsigned long ISC131:1;
            unsigned long ISC132:1;
            unsigned long ISC133:1;
            unsigned long ISC134:1;
            unsigned long ISC135:1;
            unsigned long ISC136:1;
            unsigned long ISC137:1;
            unsigned long ISC138:1;
            unsigned long ISC139:1;
            unsigned long ISC140:1;
            unsigned long ISC141:1;
            unsigned long ISC142:1;
            unsigned long ISC143:1;
            unsigned long ISC144:1;
            unsigned long ISC145:1;
            unsigned long ISC146:1;
            unsigned long ISC147:1;
            unsigned long ISC148:1;
            unsigned long ISC149:1;
            unsigned long ISC150:1;
            unsigned long ISC151:1;
            unsigned long ISC152:1;
            unsigned long ISC153:1;
            unsigned long ISC154:1;
            unsigned long ISC155:1;
            unsigned long ISC156:1;
            unsigned long ISC157:1;
            unsigned long ISC158:1;
            unsigned long ISC159:1;
        } BIT;
    } ISC4;
    union {
        unsigned long LONG;
        struct {
            unsigned long ISC160:1;
            unsigned long ISC161:1;
            unsigned long ISC162:1;
            unsigned long ISC163:1;
            unsigned long ISC164:1;
            unsigned long ISC165:1;
            unsigned long ISC166:1;
            unsigned long ISC167:1;
            unsigned long ISC168:1;
            unsigned long ISC169:1;
            unsigned long ISC170:1;
            unsigned long ISC171:1;
            unsigned long ISC172:1;
            unsigned long ISC173:1;
            unsigned long ISC174:1;
            unsigned long ISC175:1;
            unsigned long ISC176:1;
            unsigned long ISC177:1;
            unsigned long ISC178:1;
            unsigned long ISC179:1;
            unsigned long ISC180:1;
            unsigned long ISC181:1;
            unsigned long ISC182:1;
            unsigned long ISC183:1;
            unsigned long ISC184:1;
            unsigned long ISC185:1;
            unsigned long ISC186:1;
            unsigned long ISC187:1;
            unsigned long ISC188:1;
            unsigned long ISC189:1;
            unsigned long ISC190:1;
            unsigned long ISC191:1;
        } BIT;
    } ISC5;
    union {
        unsigned long LONG;
        struct {
            unsigned long ISC192:1;
            unsigned long ISC193:1;
            unsigned long ISC194:1;
            unsigned long ISC195:1;
            unsigned long ISC196:1;
            unsigned long ISC197:1;
            unsigned long ISC198:1;
            unsigned long ISC199:1;
            unsigned long ISC200:1;
            unsigned long ISC201:1;
            unsigned long ISC202:1;
            unsigned long ISC203:1;
            unsigned long ISC204:1;
            unsigned long ISC205:1;
            unsigned long ISC206:1;
            unsigned long ISC207:1;
            unsigned long ISC208:1;
            unsigned long ISC209:1;
            unsigned long ISC210:1;
            unsigned long ISC211:1;
            unsigned long ISC212:1;
            unsigned long ISC213:1;
            unsigned long ISC214:1;
            unsigned long ISC215:1;
            unsigned long ISC216:1;
            unsigned long ISC217:1;
            unsigned long ISC218:1;
            unsigned long ISC219:1;
            unsigned long ISC220:1;
            unsigned long ISC221:1;
            unsigned long ISC222:1;
            unsigned long ISC223:1;
        } BIT;
    } ISC6;
    union {
        unsigned long LONG;
        struct {
            unsigned long ISC224:1;
            unsigned long ISC225:1;
            unsigned long ISC226:1;
            unsigned long ISC227:1;
            unsigned long ISC228:1;
            unsigned long ISC229:1;
            unsigned long ISC230:1;
            unsigned long ISC231:1;
            unsigned long ISC232:1;
            unsigned long ISC233:1;
            unsigned long ISC234:1;
            unsigned long ISC235:1;
            unsigned long ISC236:1;
            unsigned long ISC237:1;
            unsigned long ISC238:1;
            unsigned long ISC239:1;
            unsigned long ISC240:1;
            unsigned long ISC241:1;
            unsigned long ISC242:1;
            unsigned long ISC243:1;
            unsigned long ISC244:1;
            unsigned long ISC245:1;
            unsigned long ISC246:1;
            unsigned long ISC247:1;
            unsigned long ISC248:1;
            unsigned long ISC249:1;
            unsigned long ISC250:1;
            unsigned long ISC251:1;
            unsigned long ISC252:1;
            unsigned long ISC253:1;
            unsigned long ISC254:1;
            unsigned long ISC255:1;
        } BIT;
    } ISC7;
    char           wk6[436];
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD1:32;
        } BIT;
    } VAD1;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD2:32;
        } BIT;
    } VAD2;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD3:32;
        } BIT;
    } VAD3;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD4:32;
        } BIT;
    } VAD4;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD5:32;
        } BIT;
    } VAD5;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD6:32;
        } BIT;
    } VAD6;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD7:32;
        } BIT;
    } VAD7;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD8:32;
        } BIT;
    } VAD8;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD9:32;
        } BIT;
    } VAD9;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD10:32;
        } BIT;
    } VAD10;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD11:32;
        } BIT;
    } VAD11;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD12:32;
        } BIT;
    } VAD12;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD13:32;
        } BIT;
    } VAD13;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD14:32;
        } BIT;
    } VAD14;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD15:32;
        } BIT;
    } VAD15;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD16:32;
        } BIT;
    } VAD16;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD17:32;
        } BIT;
    } VAD17;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD18:32;
        } BIT;
    } VAD18;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD19:32;
        } BIT;
    } VAD19;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD20:32;
        } BIT;
    } VAD20;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD21:32;
        } BIT;
    } VAD21;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD22:32;
        } BIT;
    } VAD22;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD23:32;
        } BIT;
    } VAD23;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD24:32;
        } BIT;
    } VAD24;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD25:32;
        } BIT;
    } VAD25;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD26:32;
        } BIT;
    } VAD26;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD27:32;
        } BIT;
    } VAD27;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD28:32;
        } BIT;
    } VAD28;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD29:32;
        } BIT;
    } VAD29;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD30:32;
        } BIT;
    } VAD30;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD31:32;
        } BIT;
    } VAD31;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD32:32;
        } BIT;
    } VAD32;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD33:32;
        } BIT;
    } VAD33;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD34:32;
        } BIT;
    } VAD34;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD35:32;
        } BIT;
    } VAD35;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD36:32;
        } BIT;
    } VAD36;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD37:32;
        } BIT;
    } VAD37;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD38:32;
        } BIT;
    } VAD38;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD39:32;
        } BIT;
    } VAD39;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD40:32;
        } BIT;
    } VAD40;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD41:32;
        } BIT;
    } VAD41;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD42:32;
        } BIT;
    } VAD42;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD43:32;
        } BIT;
    } VAD43;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD44:32;
        } BIT;
    } VAD44;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD45:32;
        } BIT;
    } VAD45;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD46:32;
        } BIT;
    } VAD46;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD47:32;
        } BIT;
    } VAD47;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD48:32;
        } BIT;
    } VAD48;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD49:32;
        } BIT;
    } VAD49;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD50:32;
        } BIT;
    } VAD50;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD51:32;
        } BIT;
    } VAD51;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD52:32;
        } BIT;
    } VAD52;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD53:32;
        } BIT;
    } VAD53;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD54:32;
        } BIT;
    } VAD54;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD55:32;
        } BIT;
    } VAD55;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD56:32;
        } BIT;
    } VAD56;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD57:32;
        } BIT;
    } VAD57;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD58:32;
        } BIT;
    } VAD58;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD59:32;
        } BIT;
    } VAD59;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD60:32;
        } BIT;
    } VAD60;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD61:32;
        } BIT;
    } VAD61;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD62:32;
        } BIT;
    } VAD62;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD63:32;
        } BIT;
    } VAD63;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD64:32;
        } BIT;
    } VAD64;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD65:32;
        } BIT;
    } VAD65;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD66:32;
        } BIT;
    } VAD66;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD67:32;
        } BIT;
    } VAD67;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD68:32;
        } BIT;
    } VAD68;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD69:32;
        } BIT;
    } VAD69;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD70:32;
        } BIT;
    } VAD70;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD71:32;
        } BIT;
    } VAD71;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD72:32;
        } BIT;
    } VAD72;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD73:32;
        } BIT;
    } VAD73;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD74:32;
        } BIT;
    } VAD74;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD75:32;
        } BIT;
    } VAD75;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD76:32;
        } BIT;
    } VAD76;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD77:32;
        } BIT;
    } VAD77;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD78:32;
        } BIT;
    } VAD78;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD79:32;
        } BIT;
    } VAD79;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD80:32;
        } BIT;
    } VAD80;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD81:32;
        } BIT;
    } VAD81;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD82:32;
        } BIT;
    } VAD82;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD83:32;
        } BIT;
    } VAD83;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD84:32;
        } BIT;
    } VAD84;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD85:32;
        } BIT;
    } VAD85;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD86:32;
        } BIT;
    } VAD86;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD87:32;
        } BIT;
    } VAD87;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD88:32;
        } BIT;
    } VAD88;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD89:32;
        } BIT;
    } VAD89;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD90:32;
        } BIT;
    } VAD90;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD91:32;
        } BIT;
    } VAD91;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD92:32;
        } BIT;
    } VAD92;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD93:32;
        } BIT;
    } VAD93;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD94:32;
        } BIT;
    } VAD94;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD95:32;
        } BIT;
    } VAD95;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD96:32;
        } BIT;
    } VAD96;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD97:32;
        } BIT;
    } VAD97;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD98:32;
        } BIT;
    } VAD98;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD99:32;
        } BIT;
    } VAD99;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD100:32;
        } BIT;
    } VAD100;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD101:32;
        } BIT;
    } VAD101;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD102:32;
        } BIT;
    } VAD102;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD103:32;
        } BIT;
    } VAD103;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD104:32;
        } BIT;
    } VAD104;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD105:32;
        } BIT;
    } VAD105;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD106:32;
        } BIT;
    } VAD106;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD107:32;
        } BIT;
    } VAD107;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD108:32;
        } BIT;
    } VAD108;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD109:32;
        } BIT;
    } VAD109;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD110:32;
        } BIT;
    } VAD110;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD111:32;
        } BIT;
    } VAD111;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD112:32;
        } BIT;
    } VAD112;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD113:32;
        } BIT;
    } VAD113;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD114:32;
        } BIT;
    } VAD114;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD115:32;
        } BIT;
    } VAD115;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD116:32;
        } BIT;
    } VAD116;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD117:32;
        } BIT;
    } VAD117;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD118:32;
        } BIT;
    } VAD118;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD119:32;
        } BIT;
    } VAD119;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD120:32;
        } BIT;
    } VAD120;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD121:32;
        } BIT;
    } VAD121;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD122:32;
        } BIT;
    } VAD122;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD123:32;
        } BIT;
    } VAD123;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD124:32;
        } BIT;
    } VAD124;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD125:32;
        } BIT;
    } VAD125;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD126:32;
        } BIT;
    } VAD126;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD127:32;
        } BIT;
    } VAD127;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD128:32;
        } BIT;
    } VAD128;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD129:32;
        } BIT;
    } VAD129;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD130:32;
        } BIT;
    } VAD130;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD131:32;
        } BIT;
    } VAD131;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD132:32;
        } BIT;
    } VAD132;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD133:32;
        } BIT;
    } VAD133;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD134:32;
        } BIT;
    } VAD134;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD135:32;
        } BIT;
    } VAD135;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD136:32;
        } BIT;
    } VAD136;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD137:32;
        } BIT;
    } VAD137;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD138:32;
        } BIT;
    } VAD138;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD139:32;
        } BIT;
    } VAD139;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD140:32;
        } BIT;
    } VAD140;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD141:32;
        } BIT;
    } VAD141;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD142:32;
        } BIT;
    } VAD142;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD143:32;
        } BIT;
    } VAD143;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD144:32;
        } BIT;
    } VAD144;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD145:32;
        } BIT;
    } VAD145;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD146:32;
        } BIT;
    } VAD146;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD147:32;
        } BIT;
    } VAD147;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD148:32;
        } BIT;
    } VAD148;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD149:32;
        } BIT;
    } VAD149;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD150:32;
        } BIT;
    } VAD150;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD151:32;
        } BIT;
    } VAD151;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD152:32;
        } BIT;
    } VAD152;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD153:32;
        } BIT;
    } VAD153;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD154:32;
        } BIT;
    } VAD154;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD155:32;
        } BIT;
    } VAD155;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD156:32;
        } BIT;
    } VAD156;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD157:32;
        } BIT;
    } VAD157;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD158:32;
        } BIT;
    } VAD158;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD159:32;
        } BIT;
    } VAD159;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD160:32;
        } BIT;
    } VAD160;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD161:32;
        } BIT;
    } VAD161;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD162:32;
        } BIT;
    } VAD162;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD163:32;
        } BIT;
    } VAD163;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD164:32;
        } BIT;
    } VAD164;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD165:32;
        } BIT;
    } VAD165;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD166:32;
        } BIT;
    } VAD166;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD167:32;
        } BIT;
    } VAD167;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD168:32;
        } BIT;
    } VAD168;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD169:32;
        } BIT;
    } VAD169;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD170:32;
        } BIT;
    } VAD170;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD171:32;
        } BIT;
    } VAD171;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD172:32;
        } BIT;
    } VAD172;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD173:32;
        } BIT;
    } VAD173;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD174:32;
        } BIT;
    } VAD174;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD175:32;
        } BIT;
    } VAD175;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD176:32;
        } BIT;
    } VAD176;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD177:32;
        } BIT;
    } VAD177;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD178:32;
        } BIT;
    } VAD178;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD179:32;
        } BIT;
    } VAD179;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD180:32;
        } BIT;
    } VAD180;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD181:32;
        } BIT;
    } VAD181;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD182:32;
        } BIT;
    } VAD182;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD183:32;
        } BIT;
    } VAD183;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD184:32;
        } BIT;
    } VAD184;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD185:32;
        } BIT;
    } VAD185;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD186:32;
        } BIT;
    } VAD186;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD187:32;
        } BIT;
    } VAD187;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD188:32;
        } BIT;
    } VAD188;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD189:32;
        } BIT;
    } VAD189;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD190:32;
        } BIT;
    } VAD190;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD191:32;
        } BIT;
    } VAD191;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD192:32;
        } BIT;
    } VAD192;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD193:32;
        } BIT;
    } VAD193;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD194:32;
        } BIT;
    } VAD194;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD195:32;
        } BIT;
    } VAD195;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD196:32;
        } BIT;
    } VAD196;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD197:32;
        } BIT;
    } VAD197;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD198:32;
        } BIT;
    } VAD198;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD199:32;
        } BIT;
    } VAD199;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD200:32;
        } BIT;
    } VAD200;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD201:32;
        } BIT;
    } VAD201;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD202:32;
        } BIT;
    } VAD202;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD203:32;
        } BIT;
    } VAD203;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD204:32;
        } BIT;
    } VAD204;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD205:32;
        } BIT;
    } VAD205;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD206:32;
        } BIT;
    } VAD206;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD207:32;
        } BIT;
    } VAD207;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD208:32;
        } BIT;
    } VAD208;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD209:32;
        } BIT;
    } VAD209;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD210:32;
        } BIT;
    } VAD210;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD211:32;
        } BIT;
    } VAD211;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD212:32;
        } BIT;
    } VAD212;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD213:32;
        } BIT;
    } VAD213;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD214:32;
        } BIT;
    } VAD214;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD215:32;
        } BIT;
    } VAD215;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD216:32;
        } BIT;
    } VAD216;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD217:32;
        } BIT;
    } VAD217;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD218:32;
        } BIT;
    } VAD218;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD219:32;
        } BIT;
    } VAD219;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD220:32;
        } BIT;
    } VAD220;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD221:32;
        } BIT;
    } VAD221;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD222:32;
        } BIT;
    } VAD222;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD223:32;
        } BIT;
    } VAD223;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD224:32;
        } BIT;
    } VAD224;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD225:32;
        } BIT;
    } VAD225;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD226:32;
        } BIT;
    } VAD226;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD227:32;
        } BIT;
    } VAD227;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD228:32;
        } BIT;
    } VAD228;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD229:32;
        } BIT;
    } VAD229;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD230:32;
        } BIT;
    } VAD230;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD231:32;
        } BIT;
    } VAD231;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD232:32;
        } BIT;
    } VAD232;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD233:32;
        } BIT;
    } VAD233;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD234:32;
        } BIT;
    } VAD234;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD235:32;
        } BIT;
    } VAD235;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD236:32;
        } BIT;
    } VAD236;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD237:32;
        } BIT;
    } VAD237;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD238:32;
        } BIT;
    } VAD238;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD239:32;
        } BIT;
    } VAD239;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD240:32;
        } BIT;
    } VAD240;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD241:32;
        } BIT;
    } VAD241;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD242:32;
        } BIT;
    } VAD242;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD243:32;
        } BIT;
    } VAD243;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD244:32;
        } BIT;
    } VAD244;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD245:32;
        } BIT;
    } VAD245;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD246:32;
        } BIT;
    } VAD246;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD247:32;
        } BIT;
    } VAD247;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD248:32;
        } BIT;
    } VAD248;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD249:32;
        } BIT;
    } VAD249;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD250:32;
        } BIT;
    } VAD250;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD251:32;
        } BIT;
    } VAD251;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD252:32;
        } BIT;
    } VAD252;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD253:32;
        } BIT;
    } VAD253;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD254:32;
        } BIT;
    } VAD254;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD255:32;
        } BIT;
    } VAD255;
    char           wk7[4];
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL1;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL2;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL3;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL4;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL5;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL6;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL7;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL8;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL9;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL10;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL11;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL12;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL13;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL14;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL15;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL16;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL17;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL18;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL19;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL20;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL21;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL22;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL23;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL24;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL25;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL26;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL27;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL28;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL29;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL30;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL31;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL32;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL33;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL34;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL35;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL36;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL37;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL38;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL39;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL40;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL41;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL42;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL43;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL44;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL45;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL46;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL47;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL48;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL49;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL50;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL51;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL52;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL53;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL54;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL55;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL56;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL57;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL58;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL59;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL60;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL61;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL62;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL63;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL64;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL65;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL66;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL67;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL68;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL69;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL70;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL71;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL72;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL73;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL74;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL75;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL76;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL77;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL78;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL79;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL80;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL81;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL82;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL83;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL84;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL85;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL86;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL87;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL88;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL89;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL90;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL91;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL92;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL93;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL94;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL95;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL96;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL97;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL98;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL99;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL100;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL101;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL102;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL103;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL104;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL105;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL106;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL107;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL108;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL109;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL110;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL111;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL112;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL113;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL114;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL115;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL116;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL117;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL118;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL119;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL120;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL121;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL122;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL123;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL124;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL125;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL126;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL127;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL128;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL129;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL130;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL131;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL132;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL133;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL134;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL135;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL136;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL137;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL138;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL139;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL140;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL141;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL142;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL143;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL144;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL145;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL146;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL147;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL148;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL149;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL150;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL151;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL152;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL153;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL154;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL155;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL156;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL157;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL158;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL159;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL160;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL161;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL162;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL163;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL164;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL165;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL166;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL167;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL168;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL169;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL170;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL171;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL172;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL173;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL174;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL175;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL176;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL177;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL178;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL179;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL180;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL181;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL182;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL183;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL184;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL185;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL186;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL187;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL188;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL189;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL190;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL191;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL192;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL193;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL194;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL195;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL196;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL197;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL198;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL199;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL200;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL201;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL202;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL203;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL204;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL205;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL206;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL207;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL208;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL209;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL210;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL211;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL212;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL213;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL214;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL215;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL216;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL217;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL218;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL219;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL220;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL221;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL222;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL223;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL224;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL225;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL226;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL227;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL228;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL229;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL230;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL231;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL232;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL233;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL234;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL235;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL236;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL237;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL238;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL239;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL240;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL241;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL242;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL243;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL244;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL245;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL246;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL247;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL248;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL249;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL250;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL251;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL252;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL253;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL254;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL255;
    char           wk8[1024];
    union {
        unsigned long LONG;
        struct {
            unsigned long IRQ256:1;
            unsigned long IRQ257:1;
            unsigned long IRQ258:1;
            unsigned long IRQ259:1;
            unsigned long IRQ260:1;
            unsigned long IRQ261:1;
            unsigned long IRQ262:1;
            unsigned long IRQ263:1;
            unsigned long IRQ264:1;
            unsigned long IRQ265:1;
            unsigned long IRQ266:1;
            unsigned long IRQ267:1;
            unsigned long IRQ268:1;
            unsigned long IRQ269:1;
            unsigned long IRQ270:1;
            unsigned long IRQ271:1;
            unsigned long IRQ272:1;
            unsigned long IRQ273:1;
            unsigned long IRQ274:1;
            unsigned long IRQ275:1;
            unsigned long IRQ276:1;
            unsigned long IRQ277:1;
            unsigned long IRQ278:1;
            unsigned long IRQ279:1;
            unsigned long IRQ280:1;
            unsigned long IRQ281:1;
            unsigned long IRQ282:1;
            unsigned long IRQ283:1;
            unsigned long IRQ284:1;
            unsigned long IRQ285:1;
            unsigned long IRQ286:1;
            unsigned long IRQ287:1;
        } BIT;
    } IRQS8;
    union {
        unsigned long LONG;
        struct {
            unsigned long IRQ288:1;
            unsigned long IRQ289:1;
            unsigned long IRQ290:1;
            unsigned long IRQ291:1;
            unsigned long IRQ292:1;
            unsigned long IRQ293:1;
            unsigned long IRQ294:1;
            unsigned long IRQ295:1;
            unsigned long IRQ296:1;
            unsigned long IRQ297:1;
            unsigned long IRQ298:1;
            unsigned long IRQ299:1;
            unsigned long IRQ300:1;
            unsigned long :19;
        } BIT;
    } IRQS9;
    char           wk9[56];
    union {
        unsigned long LONG;
        struct {
            unsigned long RAI256:1;
            unsigned long RAI257:1;
            unsigned long RAI258:1;
            unsigned long RAI259:1;
            unsigned long RAI260:1;
            unsigned long RAI261:1;
            unsigned long RAI262:1;
            unsigned long RAI263:1;
            unsigned long RAI264:1;
            unsigned long RAI265:1;
            unsigned long RAI266:1;
            unsigned long RAI267:1;
            unsigned long RAI268:1;
            unsigned long RAI269:1;
            unsigned long RAI270:1;
            unsigned long RAI271:1;
            unsigned long RAI272:1;
            unsigned long RAI273:1;
            unsigned long RAI274:1;
            unsigned long RAI275:1;
            unsigned long RAI276:1;
            unsigned long RAI277:1;
            unsigned long RAI278:1;
            unsigned long RAI279:1;
            unsigned long RAI280:1;
            unsigned long RAI281:1;
            unsigned long RAI282:1;
            unsigned long RAI283:1;
            unsigned long RAI284:1;
            unsigned long RAI285:1;
            unsigned long RAI286:1;
            unsigned long RAI287:1;
        } BIT;
    } RAIS8;
    union {
        unsigned long LONG;
        struct {
            unsigned long RAI288:1;
            unsigned long RAI289:1;
            unsigned long RAI290:1;
            unsigned long RAI291:1;
            unsigned long RAI292:1;
            unsigned long RAI293:1;
            unsigned long RAI294:1;
            unsigned long RAI295:1;
            unsigned long RAI296:1;
            unsigned long RAI297:1;
            unsigned long RAI298:1;
            unsigned long RAI299:1;
            unsigned long RAI300:1;
            unsigned long :19;
        } BIT;
    } RAIS9;
    char           wk10[56];
    union {
        unsigned long LONG;
        struct {
            unsigned long IEN256:1;
            unsigned long IEN257:1;
            unsigned long IEN258:1;
            unsigned long IEN259:1;
            unsigned long IEN260:1;
            unsigned long IEN261:1;
            unsigned long IEN262:1;
            unsigned long IEN263:1;
            unsigned long IEN264:1;
            unsigned long IEN265:1;
            unsigned long IEN266:1;
            unsigned long IEN267:1;
            unsigned long IEN268:1;
            unsigned long IEN269:1;
            unsigned long IEN270:1;
            unsigned long IEN271:1;
            unsigned long IEN272:1;
            unsigned long IEN273:1;
            unsigned long IEN274:1;
            unsigned long IEN275:1;
            unsigned long IEN276:1;
            unsigned long IEN277:1;
            unsigned long IEN278:1;
            unsigned long IEN279:1;
            unsigned long IEN280:1;
            unsigned long IEN281:1;
            unsigned long IEN282:1;
            unsigned long IEN283:1;
            unsigned long IEN284:1;
            unsigned long IEN285:1;
            unsigned long IEN286:1;
            unsigned long IEN287:1;
        } BIT;
    } IEN8;
    union {
        unsigned long LONG;
        struct {
            unsigned long IEN288:1;
            unsigned long IEN289:1;
            unsigned long IEN290:1;
            unsigned long IEN291:1;
            unsigned long IEN292:1;
            unsigned long IEN293:1;
            unsigned long IEN294:1;
            unsigned long IEN295:1;
            unsigned long IEN296:1;
            unsigned long IEN297:1;
            unsigned long IEN298:1;
            unsigned long IEN299:1;
            unsigned long IEN300:1;
            unsigned long :19;
        } BIT;
    } IEN9;
    char           wk11[24];
    union {
        unsigned long LONG;
        struct {
            unsigned long IEC256:1;
            unsigned long IEC257:1;
            unsigned long IEC258:1;
            unsigned long IEC259:1;
            unsigned long IEC260:1;
            unsigned long IEC261:1;
            unsigned long IEC262:1;
            unsigned long IEC263:1;
            unsigned long IEC264:1;
            unsigned long IEC265:1;
            unsigned long IEC266:1;
            unsigned long IEC267:1;
            unsigned long IEC268:1;
            unsigned long IEC269:1;
            unsigned long IEC270:1;
            unsigned long IEC271:1;
            unsigned long IEC272:1;
            unsigned long IEC273:1;
            unsigned long IEC274:1;
            unsigned long IEC275:1;
            unsigned long IEC276:1;
            unsigned long IEC277:1;
            unsigned long IEC278:1;
            unsigned long IEC279:1;
            unsigned long IEC280:1;
            unsigned long IEC281:1;
            unsigned long IEC282:1;
            unsigned long IEC283:1;
            unsigned long IEC284:1;
            unsigned long IEC285:1;
            unsigned long IEC286:1;
            unsigned long IEC287:1;
        } BIT;
    } IEC8;
    union {
        unsigned long LONG;
        struct {
            unsigned long IEC288:1;
            unsigned long IEC289:1;
            unsigned long IEC290:1;
            unsigned long IEC291:1;
            unsigned long IEC292:1;
            unsigned long IEC293:1;
            unsigned long IEC294:1;
            unsigned long IEC295:1;
            unsigned long IEC296:1;
            unsigned long IEC297:1;
            unsigned long IEC298:1;
            unsigned long IEC299:1;
            unsigned long IEC300:1;
            unsigned long :19;
        } BIT;
    } IEC9;
    char           wk12[88];
    union {
        unsigned long LONG;
        struct {
            unsigned long PLS256:1;
            unsigned long PLS257:1;
            unsigned long PLS258:1;
            unsigned long PLS259:1;
            unsigned long PLS260:1;
            unsigned long PLS261:1;
            unsigned long PLS262:1;
            unsigned long PLS263:1;
            unsigned long PLS264:1;
            unsigned long PLS265:1;
            unsigned long PLS266:1;
            unsigned long PLS267:1;
            unsigned long PLS268:1;
            unsigned long PLS269:1;
            unsigned long PLS270:1;
            unsigned long PLS271:1;
            unsigned long PLS272:1;
            unsigned long PLS273:1;
            unsigned long PLS274:1;
            unsigned long PLS275:1;
            unsigned long PLS276:1;
            unsigned long PLS277:1;
            unsigned long PLS278:1;
            unsigned long PLS279:1;
            unsigned long PLS280:1;
            unsigned long PLS281:1;
            unsigned long PLS282:1;
            unsigned long PLS283:1;
            unsigned long PLS284:1;
            unsigned long PLS285:1;
            unsigned long PLS286:1;
            unsigned long PLS287:1;
        } BIT;
    } PLS8;
    union {
        unsigned long LONG;
        struct {
            unsigned long PLS288:1;
            unsigned long PLS289:1;
            unsigned long PLS290:1;
            unsigned long PLS291:1;
            unsigned long PLS292:1;
            unsigned long PLS293:1;
            unsigned long PLS294:1;
            unsigned long PLS295:1;
            unsigned long PLS296:1;
            unsigned long PLS297:1;
            unsigned long PLS298:1;
            unsigned long PLS299:1;
            unsigned long PLS300:1;
            unsigned long :19;
        } BIT;
    } PLS9;
    char           wk13[24];
    union {
        unsigned long LONG;
        struct {
            unsigned long PIC256:1;
            unsigned long PIC257:1;
            unsigned long PIC258:1;
            unsigned long PIC259:1;
            unsigned long PIC260:1;
            unsigned long PIC261:1;
            unsigned long PIC262:1;
            unsigned long PIC263:1;
            unsigned long PIC264:1;
            unsigned long PIC265:1;
            unsigned long PIC266:1;
            unsigned long PIC267:1;
            unsigned long PIC268:1;
            unsigned long PIC269:1;
            unsigned long PIC270:1;
            unsigned long PIC271:1;
            unsigned long PIC272:1;
            unsigned long PIC273:1;
            unsigned long PIC274:1;
            unsigned long PIC275:1;
            unsigned long PIC276:1;
            unsigned long PIC277:1;
            unsigned long PIC278:1;
            unsigned long PIC279:1;
            unsigned long PIC280:1;
            unsigned long PIC281:1;
            unsigned long PIC282:1;
            unsigned long PIC283:1;
            unsigned long PIC284:1;
            unsigned long PIC285:1;
            unsigned long PIC286:1;
            unsigned long PIC287:1;
        } BIT;
    } PIC8;
    union {
        unsigned long LONG;
        struct {
            unsigned long PIC288:1;
            unsigned long PIC289:1;
            unsigned long PIC290:1;
            unsigned long PIC291:1;
            unsigned long PIC292:1;
            unsigned long PIC293:1;
            unsigned long PIC294:1;
            unsigned long PIC295:1;
            unsigned long PIC296:1;
            unsigned long PIC297:1;
            unsigned long PIC298:1;
            unsigned long PIC299:1;
            unsigned long PIC300:1;
            unsigned long :19;
        } BIT;
    } PIC9;
    char           wk14[152];
    union {
        unsigned long LONG;
        struct {
            unsigned long PRLM0:1;
            unsigned long PRLM1:1;
            unsigned long PRLM2:1;
            unsigned long PRLM3:1;
            unsigned long PRLM4:1;
            unsigned long PRLM5:1;
            unsigned long PRLM6:1;
            unsigned long PRLM7:1;
            unsigned long PRLM8:1;
            unsigned long PRLM9:1;
            unsigned long PRLM10:1;
            unsigned long PRLM11:1;
            unsigned long PRLM12:1;
            unsigned long PRLM13:1;
            unsigned long PRLM14:1;
            unsigned long PRLM15:1;
            unsigned long :16;
        } BIT;
    } PRLM1;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRLC0:1;
            unsigned long PRLC1:1;
            unsigned long PRLC2:1;
            unsigned long PRLC3:1;
            unsigned long PRLC4:1;
            unsigned long PRLC5:1;
            unsigned long PRLC6:1;
            unsigned long PRLC7:1;
            unsigned long PRLC8:1;
            unsigned long PRLC9:1;
            unsigned long PRLC10:1;
            unsigned long PRLC11:1;
            unsigned long PRLC12:1;
            unsigned long PRLC13:1;
            unsigned long PRLC14:1;
            unsigned long PRLC15:1;
            unsigned long :16;
        } BIT;
    } PRLC1;
    union {
        unsigned long LONG;
        struct {
            unsigned long UE:1;
            unsigned long :31;
        } BIT;
    } UEN1;
    char           wk15[68];
    union {
        unsigned long LONG;
        struct {
            unsigned long ISS256:1;
            unsigned long ISS257:1;
            unsigned long ISS258:1;
            unsigned long ISS259:1;
            unsigned long ISS260:1;
            unsigned long ISS261:1;
            unsigned long ISS262:1;
            unsigned long ISS263:1;
            unsigned long ISS264:1;
            unsigned long ISS265:1;
            unsigned long ISS266:1;
            unsigned long ISS267:1;
            unsigned long ISS268:1;
            unsigned long ISS269:1;
            unsigned long ISS270:1;
            unsigned long ISS271:1;
            unsigned long ISS272:1;
            unsigned long ISS273:1;
            unsigned long ISS274:1;
            unsigned long ISS275:1;
            unsigned long ISS276:1;
            unsigned long ISS277:1;
            unsigned long ISS278:1;
            unsigned long SS279:1;
            unsigned long ISS280:1;
            unsigned long ISS281:1;
            unsigned long ISS282:1;
            unsigned long ISS283:1;
            unsigned long ISS284:1;
            unsigned long ISS285:1;
            unsigned long ISS286:1;
            unsigned long ISS287:1;
        } BIT;
    } ISS8;
    union {
        unsigned long LONG;
        struct {
            unsigned long ISS288:1;
            unsigned long ISS289:1;
            unsigned long ISS290:1;
            unsigned long ISS291:1;
            unsigned long ISS292:1;
            unsigned long ISS293:1;
            unsigned long ISS294:1;
            unsigned long ISS295:1;
            unsigned long ISS296:1;
            unsigned long ISS297:1;
            unsigned long ISS298:1;
            unsigned long ISS299:1;
            unsigned long ISS300:1;
            unsigned long :19;
        } BIT;
    } ISS9;
    char           wk16[24];
    union {
        unsigned long LONG;
        struct {
            unsigned long ISC256:1;
            unsigned long ISC257:1;
            unsigned long ISC258:1;
            unsigned long ISC259:1;
            unsigned long ISC260:1;
            unsigned long ISC261:1;
            unsigned long ISC262:1;
            unsigned long ISC263:1;
            unsigned long ISC264:1;
            unsigned long ISC265:1;
            unsigned long ISC266:1;
            unsigned long ISC267:1;
            unsigned long ISC268:1;
            unsigned long ISC269:1;
            unsigned long ISC270:1;
            unsigned long ISC271:1;
            unsigned long ISC272:1;
            unsigned long ISC273:1;
            unsigned long ISC274:1;
            unsigned long ISC275:1;
            unsigned long ISC276:1;
            unsigned long ISC277:1;
            unsigned long ISC278:1;
            unsigned long ISC279:1;
            unsigned long ISC280:1;
            unsigned long ISC281:1;
            unsigned long ISC282:1;
            unsigned long ISC283:1;
            unsigned long ISC284:1;
            unsigned long ISC285:1;
            unsigned long ISC286:1;
            unsigned long ISC287:1;
        } BIT;
    } ISC8;
    union {
        unsigned long LONG;
        struct {
            unsigned long ISC288:1;
            unsigned long ISC289:1;
            unsigned long ISC290:1;
            unsigned long ISC291:1;
            unsigned long ISC292:1;
            unsigned long ISC293:1;
            unsigned long ISC294:1;
            unsigned long ISC295:1;
            unsigned long ISC296:1;
            unsigned long ISC297:1;
            unsigned long ISC298:1;
            unsigned long ISC299:1;
            unsigned long ISC300:1;
            unsigned long :19;
        } BIT;
    } ISC9;
    char           wk17[456];
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD256:32;
        } BIT;
    } VAD256;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD257:32;
        } BIT;
    } VAD257;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD258:32;
        } BIT;
    } VAD258;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD259:32;
        } BIT;
    } VAD259;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD260:32;
        } BIT;
    } VAD260;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD261:32;
        } BIT;
    } VAD261;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD262:32;
        } BIT;
    } VAD262;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD263:32;
        } BIT;
    } VAD263;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD264:32;
        } BIT;
    } VAD264;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD265:32;
        } BIT;
    } VAD265;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD266:32;
        } BIT;
    } VAD266;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD267:32;
        } BIT;
    } VAD267;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD268:32;
        } BIT;
    } VAD268;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD269:32;
        } BIT;
    } VAD269;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD270:32;
        } BIT;
    } VAD270;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD271:32;
        } BIT;
    } VAD271;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD272:32;
        } BIT;
    } VAD272;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD273:32;
        } BIT;
    } VAD273;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD274:32;
        } BIT;
    } VAD274;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD275:32;
        } BIT;
    } VAD275;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD276:32;
        } BIT;
    } VAD276;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD277:32;
        } BIT;
    } VAD277;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD278:32;
        } BIT;
    } VAD278;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD279:32;
        } BIT;
    } VAD279;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD280:32;
        } BIT;
    } VAD280;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD281:32;
        } BIT;
    } VAD281;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD282:32;
        } BIT;
    } VAD282;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD283:32;
        } BIT;
    } VAD283;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD284:32;
        } BIT;
    } VAD284;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD285:32;
        } BIT;
    } VAD285;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD286:32;
        } BIT;
    } VAD286;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD287:32;
        } BIT;
    } VAD287;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD288:32;
        } BIT;
    } VAD288;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD289:32;
        } BIT;
    } VAD289;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD290:32;
        } BIT;
    } VAD290;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD291:32;
        } BIT;
    } VAD291;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD292:32;
        } BIT;
    } VAD292;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD293:32;
        } BIT;
    } VAD293;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD294:32;
        } BIT;
    } VAD294;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD295:32;
        } BIT;
    } VAD295;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD296:32;
        } BIT;
    } VAD296;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD297:32;
        } BIT;
    } VAD297;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD298:32;
        } BIT;
    } VAD298;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD299:32;
        } BIT;
    } VAD299;
    union {
        unsigned long LONG;
        struct {
            unsigned long VAD300:32;
        } BIT;
    } VAD300;
    char           wk18[844];
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL256;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL257;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL258;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL259;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL260;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL261;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL262;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL263;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL264;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL265;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL266;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL267;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL268;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL269;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL270;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL271;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL272;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL273;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL274;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL275;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL276;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL277;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL278;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL279;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL280;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL281;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL282;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL283;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL284;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL285;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL286;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL287;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL288;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL289;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL290;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL291;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL292;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL293;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL294;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL295;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL296;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL297;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL298;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL299;
    union {
        unsigned long LONG;
        struct {
            unsigned long PRL:4;
            unsigned long :28;
        } BIT;
    } PRL300;
};

struct st_wdt {
    union {
        unsigned char BYTE;
        struct {
            unsigned char REFRESH:8;
        } BIT;
    } WDTRR;
    char           wk0[1];
    union {
        unsigned short WORD;
        struct {
            unsigned short TOPS:2;
            unsigned short :2;
            unsigned short CKS:4;
            unsigned short RPES:2;
            unsigned short :2;
            unsigned short RPSS:2;
            unsigned short :2;
        } BIT;
    } WDTCR;
    union {
        unsigned short WORD;
        struct {
            unsigned short CNTVAL:14;
            unsigned short UNDFF:1;
            unsigned short REFEF:1;
        } BIT;
    } WDTSR;
    union {
        unsigned char BYTE;
        struct {
            unsigned char :7;
            unsigned char RSTIRQS:1;
        } BIT;
    } WDTRCR;
};

//-------------------------------------
// Peripheral I/O region
//-------------------------------------
#ifdef  _RZT1_REGISTER_CORTEX_M3_
#define PERI_BASE				(0x40000000UL)
#else
#define PERI_BASE				(0xA0000000UL)
#endif

#define BSC     (*(volatile struct st_bsc     *)(PERI_BASE + 0x00002004))
#define CLMA0   (*(volatile struct st_clma0   *)(PERI_BASE + 0x00090000))
#define CLMA1   (*(volatile struct st_clma1   *)(PERI_BASE + 0x00090020))
#define CLMA2   (*(volatile struct st_clma2   *)(PERI_BASE + 0x00090040))
#define CMT     (*(volatile struct st_cmt     *)(PERI_BASE + 0x00080000))
#define CMT0    (*(volatile struct st_cmt0    *)(PERI_BASE + 0x00080002))
#define CMT1    (*(volatile struct st_cmt0    *)(PERI_BASE + 0x00080008))
#define CMT2    (*(volatile struct st_cmt0    *)(PERI_BASE + 0x00080022))
#define CMT3    (*(volatile struct st_cmt0    *)(PERI_BASE + 0x00080028))
#define CMT4    (*(volatile struct st_cmt0    *)(PERI_BASE + 0x00080042))
#define CMT5    (*(volatile struct st_cmt0    *)(PERI_BASE + 0x00080048))
#define CMTW    (*(volatile struct st_cmtw    *)(PERI_BASE + 0x00080400))
#define CMTW0   (*(volatile struct st_cmtw0   *)(PERI_BASE + 0x00080300))
#define CMTW1   (*(volatile struct st_cmtw0   *)(PERI_BASE + 0x00080380))
#define CRC     (*(volatile struct st_crc     *)(PERI_BASE + 0x0007C000))
#define DMA0    (*(volatile struct st_dma0    *)(PERI_BASE + 0x00062000))
#define DMA1    (*(volatile struct st_dma1    *)(PERI_BASE + 0x00063000))
#define DMAC    (*(volatile struct st_dmac    *)(PERI_BASE + 0x00002000))
#define DOC     (*(volatile struct st_doc     *)(PERI_BASE + 0x00081200))
#define DSMIF   (*(volatile struct st_dsmif   *)(PERI_BASE + 0x00072000))
#define ECATC   (*(volatile struct st_ecatc   *)(PERI_BASE + 0x000BF100))
#define ECCRAM  (*(volatile struct st_eccram  *)(PERI_BASE + 0x000F3000))
#define ECM     (*(volatile struct st_ecm     *)(PERI_BASE + 0x0007D080))
#define ECMC    (*(volatile struct st_ecmc    *)(PERI_BASE + 0x0007D040))
#define ECMM    (*(volatile struct st_ecmm    *)(PERI_BASE + 0x0007D000))
#define ELC     (*(volatile struct st_elc     *)(PERI_BASE + 0x00080B00))
#define ETHERC  (*(volatile struct st_etherc  *)(PERI_BASE + 0x000BF000))
#define ETHERSW (*(volatile struct st_ethersw *)(PERI_BASE + 0x000BF014))
#define GPT     (*(volatile struct st_gpt     *)(PERI_BASE + 0x0006C000))
#define GPT0    (*(volatile struct st_gpt0    *)(PERI_BASE + 0x0006C100))
#define GPT1    (*(volatile struct st_gpt0    *)(PERI_BASE + 0x0006C180))
#define GPT2    (*(volatile struct st_gpt0    *)(PERI_BASE + 0x0006C200))
#define GPT3    (*(volatile struct st_gpt0    *)(PERI_BASE + 0x0006C280))
#define ICU     (*(volatile struct st_icu     *)(PERI_BASE + 0x00094200))
#define IWDT    (*(volatile struct st_iwdt    *)(PERI_BASE + 0x00080700))
#define MPC     (*(volatile struct st_mpc     *)(PERI_BASE + 0x00000200))
#define MTU     (*(volatile struct st_mtu     *)(PERI_BASE + 0x0006A00A))
#define MTU0    (*(volatile struct st_mtu0    *)(PERI_BASE + 0x0006A090))
#define MTU1    (*(volatile struct st_mtu1    *)(PERI_BASE + 0x0006A090))
#define MTU2    (*(volatile struct st_mtu2    *)(PERI_BASE + 0x0006A092))
#define MTU3    (*(volatile struct st_mtu3    *)(PERI_BASE + 0x0006A000))
#define MTU4    (*(volatile struct st_mtu4    *)(PERI_BASE + 0x0006A000))
#define MTU5    (*(volatile struct st_mtu5    *)(PERI_BASE + 0x0006A894))
#define MTU6    (*(volatile struct st_mtu6    *)(PERI_BASE + 0x0006A800))
#define MTU7    (*(volatile struct st_mtu7    *)(PERI_BASE + 0x0006A800))
#define MTU8    (*(volatile struct st_mtu8    *)(PERI_BASE + 0x0006A098))
#define POE3    (*(volatile struct st_poe     *)(PERI_BASE + 0x00080800))
#define PORT0   (*(volatile struct st_port0   *)(PERI_BASE + 0x00000000))
#define PORT1   (*(volatile struct st_port1   *)(PERI_BASE + 0x00000002))
#define PORT2   (*(volatile struct st_port2   *)(PERI_BASE + 0x00000004))
#define PORT3   (*(volatile struct st_port3   *)(PERI_BASE + 0x00000006))
#define PORT4   (*(volatile struct st_port4   *)(PERI_BASE + 0x00000008))
#define PORT5   (*(volatile struct st_port5   *)(PERI_BASE + 0x0000000A))
#define PORT6   (*(volatile struct st_port6   *)(PERI_BASE + 0x0000000C))
#define PORT7   (*(volatile struct st_port7   *)(PERI_BASE + 0x0000000E))
#define PORT8   (*(volatile struct st_port8   *)(PERI_BASE + 0x00000010))
#define PORT9   (*(volatile struct st_port9   *)(PERI_BASE + 0x00000012))
#define PORTA   (*(volatile struct st_porta   *)(PERI_BASE + 0x00000014))
#define PORTB   (*(volatile struct st_portb   *)(PERI_BASE + 0x00000016))
#define PORTC   (*(volatile struct st_portc   *)(PERI_BASE + 0x00000018))
#define PORTD   (*(volatile struct st_portd   *)(PERI_BASE + 0x0000001A))
#define PORTE   (*(volatile struct st_porte   *)(PERI_BASE + 0x0000001C))
#define PORTF   (*(volatile struct st_portf   *)(PERI_BASE + 0x0000001E))
#define PORTG   (*(volatile struct st_portg   *)(PERI_BASE + 0x00000020))
#define PORTH   (*(volatile struct st_porth   *)(PERI_BASE + 0x00000022))
#define PORTJ   (*(volatile struct st_portj   *)(PERI_BASE + 0x00000024))
#define PORTK   (*(volatile struct st_portk   *)(PERI_BASE + 0x00000026))
#define PORTL   (*(volatile struct st_portl   *)(PERI_BASE + 0x00000028))
#define PORTM   (*(volatile struct st_portm   *)(PERI_BASE + 0x0000002A))
#define PORTN   (*(volatile struct st_portn   *)(PERI_BASE + 0x0000002C))
#define PORTP   (*(volatile struct st_portp   *)(PERI_BASE + 0x0000002E))
#define PORTR   (*(volatile struct st_portr   *)(PERI_BASE + 0x00000030))
#define PORTS   (*(volatile struct st_ports   *)(PERI_BASE + 0x00000032))
#define PORTT   (*(volatile struct st_portt   *)(PERI_BASE + 0x00000034))
#define PORTU   (*(volatile struct st_portu   *)(PERI_BASE + 0x00000036))
#define PPG0    (*(volatile struct st_ppg0    *)(PERI_BASE + 0x00080506))
#define PPG1    (*(volatile struct st_ppg1    *)(PERI_BASE + 0x00080516))
#define RIIC0   (*(volatile struct st_riic    *)(PERI_BASE + 0x00080900))
#define RIIC1   (*(volatile struct st_riic    *)(PERI_BASE + 0x00080940))
#define RSCAN   (*(volatile struct st_rscan   *)(PERI_BASE + 0x00078000))
#define RSPI0   (*(volatile struct st_rspi    *)(PERI_BASE + 0x00068000))
#define RSPI1   (*(volatile struct st_rspi    *)(PERI_BASE + 0x00068400))
#define RSPI2   (*(volatile struct st_rspi    *)(PERI_BASE + 0x00068800))
#define RSPI3   (*(volatile struct st_rspi    *)(PERI_BASE + 0x00068C00))
#define S12ADC0 (*(volatile struct st_s12adc0 *)(PERI_BASE + 0x0008C000))
#define S12ADC1 (*(volatile struct st_s12adc1 *)(PERI_BASE + 0x0008C400))
#define SCIFA0  (*(volatile struct st_scifa   *)(PERI_BASE + 0x00065000))
#define SCIFA1  (*(volatile struct st_scifa   *)(PERI_BASE + 0x00065400))
#define SCIFA2  (*(volatile struct st_scifa   *)(PERI_BASE + 0x00065800))
#define SCIFA3  (*(volatile struct st_scifa   *)(PERI_BASE + 0x00065C00))
#define SCIFA4  (*(volatile struct st_scifa   *)(PERI_BASE + 0x00066000))
#define SPIBSC  (*(volatile struct st_spibsc  *)(PERI_BASE + 0x00005000))
#define SSI     (*(volatile struct st_ssi     *)(PERI_BASE + 0x00081000))
#define SYSTEM  (*(volatile struct st_system  *)(PERI_BASE + 0x000B0020))
#define TPU0    (*(volatile struct st_tpu0    *)(PERI_BASE + 0x00080108))
#define TPU1    (*(volatile struct st_tpu1    *)(PERI_BASE + 0x00080108))
#define TPU2    (*(volatile struct st_tpu2    *)(PERI_BASE + 0x0008010A))
#define TPU3    (*(volatile struct st_tpu3    *)(PERI_BASE + 0x0008010A))
#define TPU4    (*(volatile struct st_tpu4    *)(PERI_BASE + 0x0008010C))
#define TPU5    (*(volatile struct st_tpu5    *)(PERI_BASE + 0x0008010C))
#define TPU6    (*(volatile struct st_tpu0    *)(PERI_BASE + 0x00080188))
#define TPU7    (*(volatile struct st_tpu1    *)(PERI_BASE + 0x00080188))
#define TPU8    (*(volatile struct st_tpu2    *)(PERI_BASE + 0x0008018A))
#define TPU9    (*(volatile struct st_tpu3    *)(PERI_BASE + 0x0008018A))
#define TPU10   (*(volatile struct st_tpu4    *)(PERI_BASE + 0x0008018C))
#define TPU11   (*(volatile struct st_tpu5    *)(PERI_BASE + 0x0008018C))
#define TPUA    (*(volatile struct st_tpua    *)(PERI_BASE + 0x00080100))
#define TPUSL   (*(volatile struct st_tpusl   *)(PERI_BASE + 0x00080200))
#define TSN     (*(volatile struct st_tsn     *)(PERI_BASE + 0x00080A00))
#define USBf    (*(volatile struct st_usbf    *)(PERI_BASE + 0x00060000))
#define USBh    (*(volatile struct st_usbh    *)(PERI_BASE + 0x00040000))
#define VIC     (*(volatile struct st_vic     *)(PERI_BASE + 0x00010000))
#define WDT0    (*(volatile struct st_wdt     *)(PERI_BASE + 0x00080600))
#define WDT1    (*(volatile struct st_wdt     *)(PERI_BASE + 0x00080620))

#endif
