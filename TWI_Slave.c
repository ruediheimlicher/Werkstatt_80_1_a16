//
//  TWI_Slave.c
//  TWI_Slave
//
//  Created by Sysadmin on 14.10.07.
//  Copyright __Ruedi_Heimlicher__ 2009. All rights reserved.
//



#include <avr/io.h>
#include <avr/delay.h>
//#include <avr/interrupt.h>
//#include <avr/pgmspace.h>
//#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>
#include <stdlib.h>

#include "twislave.c"
#include "lcd.c"
#include "web_SPI.c"
#include "adc.c"
#include "ds18x20.c"
#include "defines.h"

//***********************************
//Werkstatt							*
#define SLAVE_ADRESSE 0x64 //		*
//									*
//***********************************

#define STARTDELAYBIT	0
#define HICOUNTBIT		1


void eep_write_wochentag(uint8_t *ablauf[24], uint8_t *tag);
void lcd_puts(const char *s);
volatile uint8_t rxbuffer[buffer_size];

/*Der Sendebuffer, der vom Master ausgelesen werden kann.*/
volatile uint8_t txbuffer[buffer_size];

static volatile uint8_t SlaveStatus=0x00; //status


void delay_ms(unsigned int ms);
//uint16_t                EEMEM Brennerlaufzeit;	// Akkumulierte Laufzeit


uint8_t                 EEMEM WDT_ErrCount0;	// Akkumulierte WDT Restart Events
uint8_t                 EEMEM WDT_ErrCount1;	// WDT Restart Events nach wdt-reset


volatile uint8_t        Lampestatus=0x00;
static volatile uint8_t Radiatorstatus=0x00;

volatile uint16_t          Servostellung=0;					//	Abstand der Impulspakete
//volatile uint16_t       Servotakt=20;					//	Abstand der Impulspakete
//volatile uint16_t       Servopause=0x00;				//	Zaehler fuer Pause
//volatile uint16_t       Servoimpuls=0x00;				//	Zaehler fuer Impuls
//volatile uint8_t        Servoimpulsdauer=20;			//	Dauer des Servoimpulses Definitiv
//volatile uint8_t        ServoimpulsdauerPuffer=22;		//	Puffer fuer Servoimpulsdauer
//volatile uint8_t        ServoimpulsdauerSpeicher=0;		//	Speicher  fuer Servoimpulsdauer
//volatile uint8_t        Potwert=45;
volatile uint8_t           TWI_Pause=1;
//volatile uint8_t        ServoimpulsOK=0;				//	Zaehler fuer richtige Impulsdauer
//uint8_t                 ServoimpulsNullpunkt=23;
uint8_t                 ServoimpulsSchrittweite=10;
uint16_t                 Servoposition[]={1000,1250,1500,1750,2000,1750,1500,1250};

volatile uint16_t       ADCImpuls=0;
volatile uint8_t       adccounter=0;

volatile uint16_t spannungA=0;
volatile uint16_t spannungB=0;

volatile uint8_t Tastenwert=0;
volatile uint8_t TastaturCount=0;


volatile uint8_t           twicount=0;
uint8_t                 EEMEM WDT_ErrCount;	// Akkumulierte WDT Restart Events


// SPI

//#define CS_HC_PASSIVE			SPI_CONTROL_PORT |= (1<< SPI_CONTROL_CS_HC)	// CS fuer HC ist HI
//#define CS_HC_ACTIVE				SPI_CONTROL_PORT &= ~(1<< SPI_CONTROL_CS_HC)	// CS fuer HC ist LO

#define out_PULSE_DELAY			200								// Pause bei shift_byte

volatile uint16_t EventCounter=0;

volatile uint16_t       timer0counter0=0;					//
volatile uint16_t       timer0counter1=0;

volatile uint16_t       alarmcounter=0;

//#define MAXSENSORS 2
static uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];
static int16_t gTempdata[MAXSENSORS]; // temperature times 10
static uint8_t gTemp_measurementstatus=0; // 0=ok,1=error
static int8_t gNsensors=0;



void SPI_shift_out(void)
{
   
   uint8_t byteindex=0;
   in_startdaten=0;
   in_enddaten=0;
   uint8_t delayfaktor=2;
   
   in_lbdaten=0;
   in_hbdaten=0;
   lcd_gotoxy(19,1);
   lcd_putc(' ');
   //OSZILO;
   
   SPI_CLK_HI; // Clk sicher HI
   delay_ms(1);
   CS_HC_ACTIVE; // CS LO fuer Slave: Beginn Uebertragung
   //delay_ms(1);
   _delay_us(delayfaktor*out_PULSE_DELAY);
   //OSZILO;
   in_startdaten=SPI_shift_out_byte(out_startdaten);
   //OSZIHI;
   _delay_us(delayfaktor*out_PULSE_DELAY);
   in_lbdaten=SPI_shift_out_byte(out_lbdaten);
   
   _delay_us(delayfaktor*out_PULSE_DELAY);
   in_hbdaten=SPI_shift_out_byte(out_hbdaten);
   
   _delay_us(delayfaktor*out_PULSE_DELAY);
   for (byteindex=0;byteindex<SPI_BUFSIZE;byteindex++)
   {
      _delay_us(delayfaktor*out_PULSE_DELAY);
      inbuffer[byteindex]=SPI_shift_out_byte(outbuffer[byteindex]);
      //
   }
   _delay_us(delayfaktor*out_PULSE_DELAY);
   
   // Enddaten schicken: Zweiercomplement von in-Startdaten
   uint8_t complement = ~in_startdaten;
   
   in_enddaten=SPI_shift_out_byte(complement);
   
   _delay_us(delayfaktor*out_PULSE_DELAY);
   CS_HC_PASSIVE; // CS HI fuer Slave: Uebertragung abgeschlossen
   _delay_us(out_PULSE_DELAY);
   
   SPI_CLK_HI; // Clk sicher HI
   
   /*
   lcd_gotoxy(5,1);
   lcd_putint(out_startdaten);
   lcd_putc('*');
   lcd_putint(complement);
   lcd_putc('*');
   lcd_putint(in_enddaten);
   */
   lcd_gotoxy(19,1);
   
   if (out_startdaten + in_enddaten==0xFF)
   {
      lcd_putc('+');
      
   }
   else
   {
      lcd_putc('-');
      errCounter++;
      SPI_ErrCounter++;
   }
   
   
   //	lcd_gotoxy(17,3);
   //	lcd_puthex(errCounter & 0x00FF);
   //OSZIHI;
   
}

// end SPI

uint8_t Tastenwahl(uint8_t Tastaturwert)
{
   if (Tastaturwert < TASTE1)
      return 1;
   if (Tastaturwert < TASTE2)
      return 2;
   if (Tastaturwert < TASTE3)
      return 3;
   if (Tastaturwert < TASTE4)
      return 4;
   if (Tastaturwert < TASTE5)
      return 5;
   if (Tastaturwert < TASTE6)
      return 6;
   if (Tastaturwert < TASTE7)
      return 7;
   if (Tastaturwert < TASTE8)
      return 8;
   if (Tastaturwert < TASTE9)
      return 9;
   if (Tastaturwert < TASTEL)
      return 10;
   if (Tastaturwert < TASTE0)
      return 0;
   if (Tastaturwert < TASTER)
      return 12;
   
   return -1;
}



void slaveinit(void)
{
   /*
    SLAVE_OUT_DDR |= (1<<LAMPEEIN);		//Pin 0 von PORT D als Ausgang fuer Schalter: ON
    SLAVE_OUT_DDR |= (1<<LAMPEAUS);		//Pin 1 von PORT D als Ausgang fuer Schalter: OFF
    SLAVE_OUT_DDR |= (1<<OFENEIN);		//Pin 2 von PORT D als Ausgang fuer OFENEIN
    SLAVE_OUT_DDR |= (1<<OFENAUS);		//Pin 3 von PORT D als Ausgang fuer OFENAUS
    SERVODDR |= (1<<SERVOPIN1);	//Pin 6 von PORT D als Ausgang fuer Servo-Enable
    SERVODDR |= (1<<SERVOPIN0);	//Pin 7 von PORT D als Ausgang fuer Servo-Impuls
    */
   
   //SLAVE_IN_DDR &= ~(1<<PB0);	//Bit 0 von PORT B als Eingang fŸr Taste 1
   //SLAVE_IN_PORT |= (1<<PB0);	//Pull-up
   
   //SLAVE_IN_DDR &= ~(1<<PB1);	//Bit 1 von PORT B als Eingang fŸr Taste 2
   //SLAVE_IN_PORT |= (1<<PB1);	//Pull-up
   
   //TIEFKUEHLALARM_PIN
   ALARM_IN_DDR &= ~(1<<TIEFKUEHLALARM_PIN);	//Pin 3 von PORT B als Eingang fŸr Tiefkuehlalarm
   ALARM_IN_PORT |= (1<<TIEFKUEHLALARM_PIN);	//Pull-up
   
   //WASSERALARM_PIN
   
   ALARM_IN_DDR &= ~(1<<WASSERALARM_PIN);	//Pin 4 von PORT B als Eingang fŸr Wasseralarm
   ALARM_IN_PORT |= (1<<WASSERALARM_PIN);	//Pull-up
   
   OSZIDDR |= (1<<PULSA);
   OSZIPORT |= (1<<PULSA);
   
   LOOPLEDDDR |= (1<<LOOPLED);

   //TIEFKUEHLALARMBIT
   SLAVE_IN_DDR &= ~(1<<TIEFKUEHLALARM_PIN);	//Pin 3 von PORT B als Eingang fŸr Tiefkuehlalarm
   SLAVE_IN_PORT |= (1<<TIEFKUEHLALARM_PIN);	//Pull-up
   
   //WASSERALARMBIT
   
   SLAVE_IN_DDR &= ~(1<<WASSERALARM_PIN);	//Pin 4 von PORT B als Eingang fŸr Wasseralarm
   SLAVE_IN_PORT |= (1<<WASSERALARM_PIN);	//Pull-up
   
   LOOPLEDDDR |= (1<<LOOPLED);
   
   //LCD
   LCD_DDR |= (1<<LCD_RSDS_PIN);		//Pin 5 von PORT B als Ausgang fuer LCD
   LCD_DDR |= (1<<LCD_ENABLE_A_PIN);	//Pin 6 von PORT B als Ausgang fuer LCD
   LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 7 von PORT B als Ausgang fuer LCD
   
   // TWI vorbereiten
   TWI_DDR &= ~(1<<SDAPIN);//Bit 4 von PORT C als Eingang fŸr SDA
   TWI_PORT |= (1<<SDAPIN); // HI
   
   TWI_DDR &= ~(1<<SCLPIN);//Bit 5 von PORT C als Eingang fŸr SCL
   TWI_PORT |= (1<<SCLPIN); // HI
   
   BUZZER_DDR |= (1<<BUZZER_PIN);
   
   
   SlaveStatus=0;
   SlaveStatus |= (1<<TWI_WAIT_BIT);
}



// provisorisch fuer timing
void timer0(void)
{
   
    TIMSK |= (1<<TOIE0);
    TIMSK |= (1<<OCIE0);
    
    TCCR0 |= (1<<CS02);
    TCCR0 |= (1<<CS00);
    TCNT0 = 0x00;					//RŸcksetzen des Timers
    OCR0 = 0x8F;
   
   /*
   TIMSK |= (1<<TOIE0);
   TIMSK |= (1<<OCIE0);
   
   TCCR0 |= (1<<CS02);
   TCCR0 |= (1<<CS00);
   TCNT0 = 0x00;					//RŸcksetzen des Timers
   OCR0 = 0x8F;
*/
   
}

ISR(TIMER0_COMP_vect)
{
    //OSZITOGG;
}

ISR(TIMER0_OVF_vect)
{
   OSZITOGG;
   //return;
   //OSZILO;
   timer0counter0++;
   if (timer0counter0 >= 0x00FF)
   {
      if (TEST)
      {
      lcd_gotoxy(16,0);
      lcd_putint(timer0counter1&0xFF);
      }
      
      
      timer0counter0=0;
      timer0counter1++;
      if (timer0counter1 >= 0xAFFF)
      {
         //OSZITOGG;
         //timer0counter1;
         //SlaveStatus |= (1<<TWI_OK_BIT);
      }
   }
   //OSZIHI;
}

void timer1(void)
{
   
   SERVODDR |= (1<<SERVOPIN0);
/*
   TCCR1A = (1<<WGM10)|(1<<COM1A1)   // Set up the two Control registers of Timer1.
   |(1<<COM1B1);             // Wave Form Generation is Fast PWM 8 Bit,
   TCCR1B = (1<<WGM12)|(1<<CS12)     // OC1A and OC1B are cleared on compare match
   |(1<<CS10);               // and set at BOTTOM. Clock Prescaler is 1024.
   
   OCR1A = 63;                       // Dutycycle of OC1A = 25%
   //OCR1B = 127;                      // Dutycycle of OC1B = 50%

   return;
   */
   // https://www.mikrocontroller.net/topic/83609
   OCR1A = 0x3E8;           // Pulsdauer 1ms
   OCR1A = 1000;
   OCR1A = Servoposition[2];
   //OCR1B = 0x0FFF;
   ICR1 = 0xC3FF;          // Pulsabstand 50 ms  0x9FFF: 40ms

  // TCCR1A |= (1<<COM1A0);
   TCCR1A |= (1<<COM1A1); // clear on compare match
   TCCR1A |= (1<<WGM11);
   
   TCCR1B |= (1<<WGM12);
   TCCR1B |= (1<<WGM13);
   
   TCCR1B |= (1<<CS11);
  // TCCR1B |= (1<<CS10);
   
   
   
 //  TIMSK |= (1<<OCIE1A) | (1<<TICIE1); // OC1A Int enablad
}



void timer2 (void)
{
   //----------------------------------------------------
   // Set up timer 0 to generate interrupts @ 1000Hz
   //----------------------------------------------------
   TCCR2 = _BV(WGM20);
   TCCR2 = _BV(CS20) | _BV(CS22);
   OCR2 = 0x08;
   TIMSK = _BV(OCIE2);
   
   
   // Timer fuer Exp
   //	TCCR0 |= (1<<CS00)|(1<<CS02);	//Takt /1024
   //	TCCR0 |= (1<<CS02);				//8-Bit Timer, Timer clock = system clock/256
   
   //Timer fuer Servo
   /*
    TCCR0 |= (1<<CS00)|(1<<CS01);	//Takt /64 Intervall 64 us
    
    TIFR |= (1<<TOV0); 				//Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
    TIMSK |= (1<<TOIE0);			//Overflow Interrupt aktivieren
    TCNT0 = 0x00;					//RŸcksetzen des Timers
    */
}

ISR(TIMER2_COMP_vect)
{
   if (SlaveStatus & (1<<ALARMBIT))
   {
      alarmcounter++;
      if ((alarmcounter > 0x0FFF) &&  (alarmcounter < 0x1FFF))// Ton ON
      {
         BUZZER_PORT ^= (1<<BUZZER_PIN);
      }
      if (alarmcounter > 0x4FFF)
      {
         alarmcounter=0;
      }
      
   TCNT2=0;
   }
}

uint8_t search_sensors(void)
{
   uint8_t i;
   uint8_t id[OW_ROMCODE_SIZE];
   uint8_t diff, nSensors;
   
   
   ow_reset();
   
   nSensors = 0;
   
   diff = OW_SEARCH_FIRST;
   while ( diff != OW_LAST_DEVICE && nSensors < MAXSENSORS )
   {
      DS18X20_find_sensor( &diff, &id[0] );
      
      if( diff == OW_PRESENCE_ERR )
      {
         lcd_gotoxy(0,1);
         lcd_puts("No Sensor found\0" );
         
         delay_ms(800);
         lcd_clr_line(1);
         break;
      }
      
      if( diff == OW_DATA_ERR )
      {
         lcd_gotoxy(0,1);
         lcd_puts("Bus Error\0" );
         break;
      }
      lcd_gotoxy(4,1);
      
      for ( i=0; i < OW_ROMCODE_SIZE; i++ )
      {
         //lcd_gotoxy(15,1);
         //lcd_puthex(id[i]);
         
         gSensorIDs[nSensors][i] = id[i];
         //delay_ms(100);
      }
      
      nSensors++;
   }
   
   return nSensors;
}

// start a measurement for all sensors on the bus:
void start_temp_meas(void)
{
   
   gTemp_measurementstatus=0;
   if ( DS18X20_start_meas(NULL) != DS18X20_OK)
   {
      gTemp_measurementstatus=1;
   }
}

// read the latest measurement off the scratchpad of the ds18x20 sensor
// and store it in gTempdata
void read_temp_meas(void){
   uint8_t i;
   uint8_t subzero, cel, cel_frac_bits;
   for ( i=0; i<gNsensors; i++ )
   {
      
      if ( DS18X20_read_meas( &gSensorIDs[i][0], &subzero,
                             &cel, &cel_frac_bits) == DS18X20_OK )
      {
         gTempdata[i]=cel*10;
         gTempdata[i]+=DS18X20_frac_bits_decimal(cel_frac_bits);
         if (subzero)
         {
            gTempdata[i]=-gTempdata[i];
         }
      }
      else
      {
         gTempdata[i]=0;
      }
   }
}


// Code 1_wire end


volatile uint8_t testwert=19;

void main (void)
{
   slaveinit();
   //PORT2 |=(1<<PC4);
   //PORTC |=(1<<PC5);
   //init_twi_slave (SLAVE_ADRESSE);
   //uint16_t ADC_Wert= readKanal(0);
   sei();
   
   /* initialize the LCD */
   lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
   lcd_puts("Guten Tag\0");
   delay_ms(1000);
   lcd_cls();
   lcd_puts(RAUM);
   
   /*
    SLAVE_OUT_PORT &= ~(1<<LAMPEEIN);//	LAMPEEIN sicher low
    SLAVE_OUT_PORT &= ~(1<<LAMPEAUS);//	LAMPEAus sicher low
    SLAVE_OUT_PORT |= (1<<LAMPEAUS);
    delay_ms(30);
    SLAVE_OUT_PORT &= ~(1<<LAMPEAUS);
    
    SLAVE_OUT_PORT &= ~(1<<OFENEIN);//	OFENEIN sicher low
    SLAVE_OUT_PORT &= ~(1<<OFENAUS);//	OFENAUS sicher low
    SLAVE_OUT_PORT |= (1<<OFENAUS);
    delay_ms(30);
    SLAVE_OUT_PORT &= ~(1<<OFENAUS);
   */
   //uint8_t Servowert=0;
   //uint8_t Servorichtung=1;
   
   uint16_t TastenStatus=0;
   uint16_t Tastencount=0;
   uint16_t Tastenprellen=0x01F;
   uint8_t Schalterposition=0;
   
   // Timer fuer SPI-
   //timer2();
   
   
   initADC(TASTATURPIN);
   
   //	wdt_enable(WDTO_2S);
   
   uint16_t loopcount0=0;
   uint16_t loopcount1=0;

#pragma mark DS1820 init
   // DS1820 init-stuff begin
   uint8_t i=0;
   uint8_t nSensors=0;
   ow_reset();
   gNsensors = search_sensors();
   
   delay_ms(100);
   lcd_gotoxy(0,0);
   lcd_puts("Sens: \0");
   lcd_puthex(gNsensors);
   if (gNsensors>0)
   {
      lcd_clr_line(1);
      start_temp_meas();
   }
   i=0;
   while(i<MAXSENSORS)
   {
      gTempdata[i]=0;
      i++;
   }

   
  Init_SPI_Master();
   
   /*
#pragma mark DS1820 init
   // DS1820 init-stuff begin
   uint8_t i=0;
   uint8_t nSensors=0;
   ow_reset();
   gNsensors = search_sensors();
   
   delay_ms(100);
   lcd_gotoxy(0,0);
   lcd_puts("Sens: \0");
   lcd_puthex(gNsensors);
   if (gNsensors>0)
   {
      lcd_clr_line(1);
      start_temp_meas();
   }
   i=0;
   while(i<MAXSENSORS)
   {
      gTempdata[i]=0;
      i++;
   }
   // DS1820 init-stuff end
*/
   
   
   uint8_t twierrcount=0;
   LOOPLEDPORT |=(1<<LOOPLED);
   
   delay_ms(800);
   uint8_t tempWDT_Count=eeprom_read_byte(&WDT_ErrCount);
   
   
   //	Zaehler fuer Wartezeit nach dem Start
   uint16_t startdelay0=0x00AF;
   //uint16_t startdelay1=0;
   
   //Zaehler fuer Zeit von (SDA || SCL = LO)
 //  uint16_t twi_LO_count0=0;
 //  uint16_t twi_LO_count1=0;
   uint8_t StartStatus=0x00; //status
   
   //Zaehler fuer Zeit von (SDA && SCL = HI)
//   uint16_t twi_HI_count0=0;
   
   uint8_t eepromWDT_Count0=eeprom_read_byte(&WDT_ErrCount0);
   uint8_t eepromWDT_Count1=eeprom_read_byte(&WDT_ErrCount1);
   /*
    eepromWDT_Count0: Zaehler der wdt-Resets mit restart.
    
    eepromWDT_Count1: Zaehler fuer neuen wdt-Reset. Wenn wdt anspricht, wird der Zaheler erhoeht.
    Beim Restart wird bei anhaltendem LO auf SDA oder SCL gewartet.
    Wenn SCL und SDA beide HI sind, wird der Zaehler auf den Wert von eepromWDT_Count0 gesetzt
    und der TWI-Slave gestartet.
    
    */
   
   // Ankuendigen, dass schon ein wdt erfolgte
   if (!(eepromWDT_Count0==eepromWDT_Count1))
   {
      //lcd_gotoxy(18,1);
      //lcd_puts("W\0");
   }
   init_twi_slave (SLAVE_ADRESSE);
   sei();

   
   timer2();
   lcd_cls();
//   timer0();
   
#pragma mark while

   while (1)
   {
      wdt_reset();
      //Blinkanzeige
      loopcount0++;
      if (loopcount0==0xAFFF)
      {
         //OSZITOGG;
       //  lcd_gotoxy(12,1);
       //  lcd_putint(loopcount1);

         loopcount0=0;
         LOOPLEDPORT ^=(1<<LOOPLED);
         
         loopcount1++;
         if (loopcount1 >= 0x0F)
         {
            loopcount1=0;
            Servostellung++;
            OCR1A = Servoposition[Servostellung %8];
            if (TEST || (!(TESTPIN & (1<<TEST_PIN))))
            {
              // rxdata=1;
               SlaveStatus |= (1<<TWI_OK_BIT); // TWI ist ON, Simulation Startroutine
               
               
            }
            spannungA = (readKanal(ADC_A_PIN));
            spannungB = (readKanal(ADC_B_PIN));
            
            adccounter++;


         }
       }
   //   continue;
            /**	Beginn Startroutinen	***********************/
      // wenn Startbedingung vom Master:  TWI_slave initiieren
      if (SlaveStatus & (1<<TWI_WAIT_BIT))
      {
         if ((TWI_PIN & (1<<SCLPIN))&&(!(TWI_PIN & (1<<SDAPIN))))// Startbedingung vom Master: SCL HI und SDA LO
         {
            lcd_gotoxy(16,0);
            lcd_puts(" TWI");
            twicount+=10;
 //           init_twi_slave (SLAVE_ADRESSE);
            sei();
            SlaveStatus &= ~(1<<TWI_WAIT_BIT);
            SlaveStatus |= (1<<TWI_OK_BIT); // TWI ist ON
            
            // StartDelayBit zuruecksetzen
         }
      }
     
    
      /**	Ende Startroutinen	***********************/
      
      /***** rx_buffer abfragen **************** */
      //rxdata=0;
      
      //***************
      //	Test
      if (TEST && (SlaveStatus &(1<<TWI_WAIT_BIT)))
      {
 
         SlaveStatus &= ~(1<<TWI_WAIT_BIT); // simulation TWI
         SlaveStatus |= (1<<TWI_OK_BIT);

         //rxdata=1;
      }
      

      //SlaveStatus |= (1<<TWI_OK_BIT);
      
      // end test
      //***************
      
      
      
      if ((SlaveStatus & (1<<TWI_OK_BIT)) &&(rxdata) && !(SlaveStatus & (1<<MANUELLBIT)))	//Daten von TWI liegen vor und Manuell ist OFF
      {
         twicount++;
        // lcd_gotoxy(0,1);
       //  lcd_putint(rxdata);
         
         lcd_gotoxy(0,1);
         lcd_putint(spannungA>>2);
         
         lcd_putc(' ');
         lcd_putint12(spannungB);

         if (TEST || (!(TESTPIN & (1<<TEST_PIN))))
         {
            SlaveStatus &= ~(1<<TWI_OK_BIT); // simulation TWI
         }
         // ++++++++++++++++++++++++++
         Lampestatus=rxbuffer[0];
         //lcd_gotoxy(12,0);
         //lcd_puts("L:\0");
         //lcd_puthex(Lampestatus);
         lcd_gotoxy(8,0);
         lcd_putc('L');
         
         if ( Lampestatus  & (1<<LAMPEBIT)) // Bit 0
         {
            //delay_ms(1000);
            //Lampe ein
            SLAVE_OUT_PORT &= ~(1<<LAMPEAUS);//	LAMPEAUS sicher low
            SLAVE_OUT_PORT &= ~(1<<LAMPEEIN);//	LAMPEEIN sicher low
            SLAVE_OUT_PORT |= (1<<LAMPEEIN);
            delay_ms(30);
            SLAVE_OUT_PORT &= ~(1<<LAMPEEIN);
            lcd_putc('1');
            
            //lcd_gotoxy(15,1);
            //lcd_puts("ON \0");
         }
         else
         {
            //delay_ms(1000);
            //Lampe aus
            //lcd_gotoxy(19,1);
            //lcd_putc('0');
            
            SLAVE_OUT_PORT &= ~(1<<LAMPEEIN);//	LAMPEEIN sicher low
            SLAVE_OUT_PORT &= ~(1<<LAMPEAUS);//	LAMPEAUS sicher low
            SLAVE_OUT_PORT |= (1<<LAMPEAUS);
            delay_ms(30);
            SLAVE_OUT_PORT &= ~(1<<LAMPEAUS);
            lcd_putc('0');
            
            //lcd_gotoxy(15,1);
            //lcd_puts("OFF\0");
            
         }
         
         // Ofen
         
         Radiatorstatus=rxbuffer[1];
         
         if (TEST)
         {
            /*
             SLAVE_OUT_PORT |= (1<<LAMPEAUS); // Impuls an OFF
             delay_ms(30);
             SLAVE_OUT_PORT &= ~(1<<LAMPEAUS);
             delay_ms(200);
             SLAVE_OUT_PORT |= (1<<LAMPEEIN); // Impuls an ON
             delay_ms(30);
             SLAVE_OUT_PORT &= ~(1<<LAMPEEIN);
             */
            
         }
         
#pragma mark Ofen
         //if ( Slavestatus  & (1<<OFENBIT)) // Bit 1
         if ( Radiatorstatus & (1<<OFENBIT)) // //Bit 1 Ofen ein
         {
            //delay_ms(1000);
            //Ofen ein
            
            //lcd_putc('I');
            SLAVE_OUT_PORT &= ~(1<<OFENAUS);//	OFENAUS sicher low
            SLAVE_OUT_PORT &= ~(1<<OFENEIN);//	OFENEIN sicher low
            SLAVE_OUT_PORT |= (1<<OFENEIN); // Impuls an ON
            delay_ms(30);
            SLAVE_OUT_PORT &= ~(1<<OFENEIN);
            //lcd_gotoxy(15,1);
            lcd_gotoxy(10,0);
            lcd_putc('R');

            lcd_putc('1');
         }
         else
         {
            //delay_ms(1000);
            //Ofen aus
            //lcd_putc('0');
            SLAVE_OUT_PORT &= ~(1<<OFENEIN);//	OFENEIN sicher low
            SLAVE_OUT_PORT &= ~(1<<OFENAUS);//	OFENAUS sicher low
            SLAVE_OUT_PORT |= (1<<OFENAUS); // Impuls an OFF
            delay_ms(30);
            SLAVE_OUT_PORT &= ~(1<<OFENAUS);
            //lcd_gotoxy(15,1);
            lcd_gotoxy(10,0);
            lcd_putc('R');

            lcd_putc('0');
         }
         
#pragma mark Sensors
         // Temperatur messen mit DS18S20
         if (gNsensors) // Sensor eingeseteckt
         {
            start_temp_meas();
            delay_ms(800);
            read_temp_meas();
            uint8_t line=0;
            //Sensor 1
            lcd_gotoxy(0,line);
            lcd_puts("T:     \0");
            if (gTempdata[0]/10>=100)
            {
               lcd_gotoxy(3,line);
               lcd_putint((gTempdata[0]/10));
            }
            else
            {
               lcd_gotoxy(2,line);
               lcd_putint2((gTempdata[0]/10));
            }
            
            lcd_putc('.');
            lcd_putint1(gTempdata[0]%10);
         }
         txbuffer[INNEN]=2*((gTempdata[0]/10)& 0x00FF);// T kommt mit Faktor 10 vom DS. Auf TWI ist T verdoppelt
         // Halbgrad addieren
         if (gTempdata[0]%10 >=5) // Dezimalstelle ist >=05: Wert  aufrunden, 1 addieren
         {
            txbuffer[INNEN] +=1;
         }
#pragma mark Kuehltruhe/Wasser
         //
         //	Kuehltruhe abfragen
         //
         if (ALARM_IN_PIN & (1<<TIEFKUEHLALARM_PIN)) // HI, Alles OK
         {
             SlaveStatus &= ~(1<<ALARMBIT);
            txbuffer[STATUS] &= ~(1<<TIEFKUEHLALARM_PIN); // TIEFKUEHLALARM_PIN zuruecksetzen Bit 3
            lcd_gotoxy(17,1);
            lcd_putc('-');
         }
         else
         {
             SlaveStatus |= (1<<ALARMBIT);
            txbuffer[STATUS] |= (1<<TIEFKUEHLALARM_PIN);	// TIEFKUEHLALARM_PIN setzen
            lcd_gotoxy(17,1);
            lcd_putc('t');
         }
         
         //
         //	Wasseralarm abfragen
         //
         if (ALARM_IN_PIN & (1<<WASSERALARM_PIN)) // HI, Alles OK
         {
            txbuffer[STATUS] &= ~(1<<WASSERALARM_PIN); // WASSERALARM_PIN zuruecksetzen
            lcd_gotoxy(18,1);
            lcd_putc('-');
            
         }
         else
         {
            txbuffer[STATUS] |= (1<<WASSERALARM_PIN);	// WASSERALARM_PIN setzen
            lcd_gotoxy(18,1);
            lcd_putc('w');
            
         }

         
         
         
         // ++++++++++++++++++++++++++

         
         rxdata=0;               // TWI erledigt
         
#pragma mark SPI
         /***** SPI: Daten von SPI_Slave_Strom abfragen **************** */
         //out_enddaten = 0xA4;
         
         inbuffer[0]=0;
         inbuffer[1]=0;
         inbuffer[2]=0;
         
         testwert++;
         
         outbuffer[0] = testwert;
         outbuffer[1] = 0;//testwert;
         outbuffer[2] = 0;//testwert;
         
         // if (TEST)
         {
            //lcd_gotoxy(0,1);
            //lcd_puts("SPI");
            out_startdaten = 0xA1;
            out_hbdaten = 0xA2;
            out_lbdaten = 0xA3;
            //lcd_gotoxy(0,1);
            //lcd_putint16(gTempdata[0]);
            
            lcd_gotoxy(0,2);
            lcd_puts("oS \0");
            lcd_putint(outbuffer[0]);
            lcd_putc('*');
            //lcd_putint(outbuffer[1]);
            //lcd_putc('*');
            //lcd_putint(outbuffer[2]);
            lcd_putc('s');
            lcd_putint(out_startdaten);
            
         }
         //OSZILO;
         
#pragma mark SPI_shift_out
         //****************************
         
         SPI_shift_out(); // delayfaktor 2: 80ms aktueller delayfaktor 16: 150ms
         //****************************
         //OSZIHI;
         //    if (TEST)
         {
            lcd_gotoxy(0,3);
            lcd_puts("iS \0");
            lcd_putint(inbuffer[0]);
            lcd_putc('*');
            lcd_putint(inbuffer[1]);
            lcd_putc('*');
            lcd_putint(inbuffer[2]);
            lcd_putc('*');
            lcd_putint(in_startdaten);
            
         }
         
         lcd_gotoxy(13,0);
         lcd_putint(SPI_ErrCounter);
         
         lcd_gotoxy(12,2);
         lcd_putint16(inbuffer[0]+0xFF*inbuffer[1]);
         
         txbuffer[STROML]= inbuffer[0]; // L: byte 4
         txbuffer[STROMH] = inbuffer[1]; // H: byte 5
         txbuffer[STROMHH] = inbuffer[2]; // HH: byte 6
         
         /***** End SPI**************** */

          //****************************
      }
      
      
   }//while
   
   
   // return 0;
}
