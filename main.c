/* 
 * File:   main.c
 * Author: 佐々木翔
 *
 * Created on 2021/09/08, 17:51
 * Discription　実装試験のコマンド系のプログラム
 * 編集履歴
 * 2021/10/18：タイマーの実装
 * 
 */

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include "spi.h"
#include "MCP2515.h"
#include "C.h"

// CONFIG1
#pragma config FOSC  = HS        // Oscillator Selection bits (HS oscillator: High-speed crystal/resonator on RA6/OSC2/CLKOUT and RA7/OSC1/CLKIN)
#pragma config WDTE  = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = ON       // RE3/MCLR pin function select bit (RE3/MCLR pin function is MCLR)
#pragma config CP    = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD   = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown Out Reset Selection bits (BOR enabled)
#pragma config IESO  = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP   = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR21V   // Brown-out Reset Selection bit (Brown-out Reset set to 2.1V)
#pragma config WRT   = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#define _XTAL_FREQ      20000000
#define _CAN_BAUDRATE   2

/*--Prtotype--*/
void CONFIG();                                                                  //初期設定
void timer_init();                                                             //タイマーの初期設定
void interrupt timer();                                                         //タイマーの割り込み処理
char BtoD(char data);                                                           //2進数を10進数に変換

/*--Grobal Variables--*/
volatile char *rx_data;                                                         //受信データの受け入れ口
volatile char *fram_data;                                                       //fram用の箱
volatile char rx_status;                                                        //受信ステータス用の箱
volatile char rx_int;                                                           //受信フラグ確認用の箱
volatile char data[8] = {2, 2, 2, 2, 2, 2, 2, 2};                               //HKデータ
volatile char mode = _ChargeMode;                                               //モードフラグ（初期モード：充電モード）
volatile char size;                                                             //データ長
volatile int cnt = 0;                                                           //コンペアマッチ用の変数

void main(void)
{   
    CONFIG();                                                                   //初期設定
    __delay_ms(100);
    
    RB5 = 0;
    RB6 = 0;
    RB7 = 1;
    
    T1CONbits.TMR1ON = 1;                                                       //タイマー開始
    
    while(1)
    {       
        if(cnt == 0)
        {
            mode = _ChargeMode;
            
            Write(_TXB0DLC, 0b00000001);                                            //メッセージサイズ1byte
            Load_TX_ID(_F_TXB0SIDH, SIDH_MODE, SIDL_W, EID8_MODE, EID0_MODE);       //モード書き込みIDの設定
            Load_TX_Data(_F_TXB0D0, 1, &mode);                                      //モード書き込み
            RTS0(_CAN_BAUDRATE);                                                    //送信要求
            
            RB5 = 0;
            RB6 = 0;
            RB7 = 1;
        }
        
        if(cnt == 50)
        {
            mode = _COMMMode;
            
            Write(_TXB0DLC, 0b00000001);                                            //メッセージサイズ1byte
            Load_TX_ID(_F_TXB0SIDH, SIDH_MODE, SIDL_W, EID8_MODE, EID0_MODE);       //モード書き込みIDの設定
            Load_TX_Data(_F_TXB0D0, 1, &mode);                                      //モード書き込み
            RTS0(_CAN_BAUDRATE);                                                    //送信要求
            
            RB5 = 0;
            RB6 = 1;
            RB7 = 0;
        }
        
        if(cnt == 100)
        {
            mode = _StanbyMode;
            
            Write(_TXB0DLC, 0b00000001);                                            //メッセージサイズ1byte
            Load_TX_ID(_F_TXB0SIDH, SIDH_MODE, SIDL_W, EID8_MODE, EID0_MODE);       //モード書き込みIDの設定
            Load_TX_Data(_F_TXB0D0, 1, &mode);                                      //モード書き込み
            RTS0(_CAN_BAUDRATE);                                                    //送信要求
            
            RB5 = 1;
            RB6 = 0;
            RB7 = 0;
        }
        
        if(cnt == 150)
        {
            mode = _MissionMode;
            
            Write(_TXB0DLC, 0b00000001);                                            //メッセージサイズ1byte
            Load_TX_ID(_F_TXB0SIDH, SIDH_MODE, SIDL_W, EID8_MODE, EID0_MODE);       //モード書き込みIDの設定
            Load_TX_Data(_F_TXB0D0, 1, &mode);                                      //モード書き込み
            RTS0(_CAN_BAUDRATE);                                                    //送信要求
            
            RB5 = 0;
            RB6 = 0;
            RB7 = 0;
        }
        
        if(cnt == 200)
        {
            cnt = 0;
        }
    }
}

void CONFIG()
{
    OSCCON = 0b01101000;
    ANSEL  = 0b00000000;
    ANSELH = 0b00000000;
    TRISB  = 0b00000000;
    TRISC  = 0b00000000;
    PORTB  = 0b00000000;
    PORTC  = 0b00000000;
    
    INTCON = 0b11000000;                                                        //割り込み設定
    
    spi_init();
    timer_init();
    __delay_ms(100);
    
    MCP2515_init(_CAN_BAUDRATE);                                                //とりあえず，動作している2にした．理解はまだ
    Write(_TXB0DLC , 0b00000001);                                               //メッセージサイズ8byte
    MCP2515_Open(0);                                                            //とりあえず，0にした．理解はまだ
}

void timer_init()
{
    T1CON = 0b00110000;
    TMR1H = 0b00000000;
    TMR1L = 0b00000000;
    PIE1bits.TMR1IE = 1;
    PIR1bits.TMR1IF = 0;
}

void interrupt timer()
{
    PIR1bits.TMR1IF = 0;
    TMR1H = 0b00000000;
    TMR1L = 0b00000000;
    
    cnt++;
}

char BtoD(char data)
{
    char  binary;
    char decimal = 0;
    char bas = 1;
            
    binary = data & 0b00001111;
    
    while(binary>0)
    {
        decimal = decimal + (binary % 10) * bas;
        binary = binary / 10;
        bas = bas * 2;
    }
    
    return decimal;
}