/*
   PJD7820HD Ballast Module Emulator
   Settings are suitable for Arduino Pro Mini, Arduino Nano (ATMEGA168 / ATMEGA328)
*/

#include <avr/io.h>
#include <avr/interrupt.h>

#define F_CPU  16000000UL          // Work frequency controller

struct Projector
{
   bool emulation = false;   

   bool on  = true;
   bool off = false;

   volatile int delay_counter = 0; 

   volatile const int* command = nullptr;
   volatile int command_counter = 0;
   volatile int command_length = 0;

   Projector();

   void timer_for_delay(bool ON);
   void delay();
   void transmitter(bool ON);

   void buffer_underrun_interrupt(bool ON);
   template <int N> void send_command(const int(&my_array)[N]);
   void next_command();
   void PJD7820HD(int command_byte);                              // List of projector commands

   void work();

};

Projector::Projector()
{
   /*
      Settings UART:  Asynchronous Normal mode, Even Parity, 8 bit
   */
   unsigned int BAUD = 9600;                       // Data exchange rate
   unsigned int UBRRL_value = F_CPU/BAUD/16-1;     // UBRR = 51 = 0x34    (16 000 000 / 9600 / 16) - 1   = 103 = 0x67   Table 20-6. 
   UBRR0   =    0x0067;                            // Asynchronous Normal mode 

   UCSR0B |=   (1<< RXEN0)|                        // Beat permission to receive  
               (1<< RXCIE0);                       // Allow interrupts upon completion of the reception, transmission  20.11.2 UCSRnA
   
   UCSR0C |=   (1<< UPM01)|                        // parity bit: Even Parity
               (1<< UCSZ01)|(1<< UCSZ00);          // 8 Data bit;     20.11.3 UCSRnB

   SREG   |=   (1<<7);                             // Allow interrupts
   
   /* 
      timer
      For exchange commands, the PJD7820HD projector requires a transmitting delay in approximately 5600 Miliseconds
   */
   TCCR1A  |=  (1<<COM1B0);                        //  Compare Output Mode, non-PWM    Toggle OC1A/OC1B on Compare Match    
   TCCR1B  |=  (1<<WGM12)|                         //  Waveform Generation Mode Bit Description  CTC
               (1<<CS12);                          //  clkI/O/256 (From prescaler)     16000000 / 256 / 6250 = 0,1 second (100 ms)   
   OCR1A    =   6250;                              //  prescaler
}
void Projector::PJD7820HD(int command_byte)                      
{
   volatile unsigned char previous_command_byte;                              // for dual bit command

   switch ( command_byte )
   {
      case 0x26: send_command({0x26});             break;

      case 0x75: work();                           break;                     // begin / finish emulation

      case 0x69: send_command({0x69,0x80});        break;
      case 0x70: send_command({0x70});             break;
      case 0xF0: send_command({0xF0,0x01});        break;
      case 0xF1: send_command({0xF1,0x2D,0x22});   break;
      case 0xF2: send_command({0xF2,0xBF});        break;
      case 0xF3: send_command({0xF3,0x00});        break;
      case 0xF4: send_command({0xF4,0x80});        break;
      case 0xF5: send_command({0xF5,0x01});        break;
      case 0xF6: send_command({0xF6,0x05});        break;
      case 0xFA: send_command({0xFA,0x62});        break;
      case 0xFB: send_command({0xFB,0x80});        break;   

      case 0x00: if ( previous_command_byte == 0x65 )  { send_command({0x65,0x00,0x01}); } break;
      case 0x01: if ( previous_command_byte == 0x71 )  { send_command({0x71,0x01}); }      break;

      default : previous_command_byte = command_byte;  break; 
   }
}
void Projector::timer_for_delay(bool ON)              
{
   if ( ON ){ TIMSK1  |=  (1<<OCIE1A); }
   else     { TIMSK1 &= ~ (1<<OCIE1A); }
}
void Projector::delay()
{
   if(delay_counter <= 56)
   {
      if (delay_counter == 56) 
      { 
         transmitter(on); 
      }
      delay_counter++;
   }
   else 
   { 
      delay_counter = 0;
      timer_for_delay(off); 
   }
}
void Projector::transmitter(bool ON)
{
   if ( ON ){ UCSR0B |=   (1 << TXEN0); }
   else     { UCSR0B &= ~ (1 << TXEN0); }
}
void Projector::buffer_underrun_interrupt(bool ON)
{
   if ( ON ){ UCSR0B |=  (1 << UDRIE0); }
   else     { UCSR0B &= ~(1 << UDRIE0); }
}
template <int N> void Projector::send_command(const int(&my_array)[N])
{
   buffer_underrun_interrupt(on);
   command_length=N;
   command = (const short int*)my_array;
}
void Projector::next_command()
{
   if (command_counter == command_length)          
   {  
      buffer_underrun_interrupt(off);
      command_counter=0;   
      command_length=0; 
   }  
   else  
   {  
      UDR0 = command[command_counter] ;            // take a command from the buffer.
      command_counter++;                           
   } 
}
void Projector::work()
{
   emulation = !emulation;
   if (emulation) 
   {
      timer_for_delay(on);
   }
   else
   {
      transmitter(off);
   }
}

Projector projector;

ISR (USART_RX_vect)              // Interruption at the end of the reception, module USART
{ 
   projector.PJD7820HD(UDR0);    // Processing commands
}

ISR (USART_UDRE_vect)            // Interrupt to devastation buffer USART
{
   projector.next_command();
}

ISR (TIMER1_COMPA_vect)          // Interruption by timer
{
   projector.delay();
}

int main()
{
   // Expect the startup command from the projector in interrupt ISR (USART_RX_vect) 

   while(1)
   {

   }
   return 0;
}