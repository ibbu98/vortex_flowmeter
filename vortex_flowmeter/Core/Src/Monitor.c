/*-----------------------------------------------------------------------------
            \file Monitor.c
--                                                                           --
--              ECEN 5003 Mastering Embedded System Architecture             --
--                  Project 1 Module 6                                       --
--                Microcontroller Firmware                                   --
--                      Monitor.c                                            --
--                                                                           --
-------------------------------------------------------------------------------
--
--  Designed for:  University of Colorado at Boulder
--               
--                
--  Designed by:  Tim Scherr
--  Revised by:  Ibrahim Muadh & Varun Shah
-- 
-- Version: 4.0
-- Date of current revision:  2025-09-11  
-- Target Microcontroller: ST STM32F401RE 
--              Keil MDK IDE or STMCubeIDE
--              ST Nucleo STM32F401RE Board
--              STMCubeMX Configurator 
-- 
--               
-- 
--   Functional Description: See below 
--
--      Copyright (c) 2015, 2022, 2025 Tim Scherr  All rights reserved.
--
*/              

#include <stdio.h>
#include "shared.h"
#include "freq_detect.h"

extern float freq_hz;
extern float flow_gpm_val;
extern float temp_c;


/*******************************************************************************
* Set Display Mode Function
* Function determines the correct display mode.  The 3 display modes operate as 
*   follows:
*
*  NORMAL MODE       Outputs only mode and state information changes   
*                     and calculated outputs
*
*  QUIET MODE        No Outputs
*
*  DEBUG MODE        Outputs mode and state information, error counts,
*                    register displays, sensor states, and calculated output
*                (currently not all features are operational, could be enhanced)
*
* There is deliberate delay in switching between modes to allow the RS-232 cable 
* to be plugged into the header without causing problems. 
*******************************************************************************/

void set_display_mode(void)
{
  UART_direct_msg_put((const unsigned char *)"\r\nSelect Mode");
  UART_direct_msg_put((const unsigned char *)"\r\n Hit NOR - Normal");
  UART_direct_msg_put((const unsigned char *)"\r\n Hit QUI - Quiet");
  UART_direct_msg_put((const unsigned char *)"\r\n Hit DEB - Debug" );
  UART_direct_msg_put((const unsigned char *)"\r\n Hit V - Version#\r\n");
  UART_direct_msg_put((const unsigned char *)"\r\nSelect:  ");
  
}


//*****************************************************************************/
/// \fn void chk_UART_msg(void) 
///
///  \brief - fills a message buffer until return is encountered, then calls
///           message processing
//*****************************************************************************/
/****************      ECEN 5803 add code as indicated   **********************/
  // Improve behavior of this function
void chk_UART_msg(void)
{
   uchar8_t j;
   while( UART_input() )      // becomes true only when a byte has been received
   {                                    // skip if no characters pending
      j = UART_get();                 // get next character

      if( j == '\r' )          // on a enter (return) key press
      {                // complete message (all messages end in carriage return)
         UART_msg_put((const unsigned char *)"->");
         UART_msg_process();
      }
      else 
      {
         if ((j != 0x02) )         // if not ^B
         {                             // if not command, then   
            UART_put(j);              // echo the character   
         }
         else
         {
           ;
         }
         if( j == '\b' ) 
         {                             // backspace editor
            if( msg_buf_idx != 0) 
            {                       // if not 1st character then destructive 
               UART_msg_put((const unsigned char *)" \b");// backspace
               msg_buf_idx--;
            }
         }
         else if( msg_buf_idx >= MSG_BUF_SIZE )  
         {                                // check message length too large
            UART_msg_put((const unsigned char *)"\r\nToo Long!");
            msg_buf_idx = 0;
         }
         else if ((display_mode == QUIET) && (msg_buf[0] != 0x02) && 
                  (msg_buf[0] != 'D') && (msg_buf[0] != 'N') && 
                  (msg_buf[0] != 'V') &&
                  (msg_buf_idx != 0))
         {                          // if first character is bad in Quiet mode
            msg_buf_idx = 0;        // then start over
         }
         else {                        // not complete message, store character
 
            msg_buf[msg_buf_idx] = j;
            msg_buf_idx++;
            if (msg_buf_idx > 2)
            {
               UART_msg_process();
            }
         }
      }
   }
}

//*****************************************************************************/
///  \fn void UART_msg_process(void) 
///UART Input Message Processing
//*****************************************************************************/
void UART_msg_process(void)
{
   uchar8_t chr,err=0;


   if( (chr = msg_buf[0]) <= 0x60 ) 
   {      // Upper Case
      switch( chr ) 
      {
         case 'D':
            if((msg_buf[1] == 'E') && (msg_buf[2] == 'B') && (msg_buf_idx == 3))
            {
               display_mode = DEBUG_MODE;
               UART_msg_put((const unsigned char *)"\r\nMode=DEBUG_MODE\n");
               display_timer = 0;
            }
            else
               err = 1;
            break;
      case(DEBUG_MODE):
            {
               if (display_flag == 1)
               {
                  extern float freq_hz;
                  extern float flow_gpm;
                  extern float temp_c;

                  UART_msg_put((const unsigned char *)"\r\nDEBUG ");

                  // --- Flow output ---
                  UART_msg_put((const unsigned char *)" Flow: ");
                  char flow_buf[16];
                  sprintf(flow_buf, "%.2f", flow_gpm);
                  UART_msg_put((const unsigned char *)flow_buf);

                  // --- Temperature output ---
                  UART_msg_put((const unsigned char *)" Temp: ");
                  char temp_buf[16];
                  sprintf(temp_buf, "%.2f", temp_c);
                  UART_msg_put((const unsigned char *)temp_buf);

                  // --- Frequency output ---
                  UART_msg_put((const unsigned char *)" Freq: ");
                  char freq_buf[16];
                  sprintf(freq_buf, "%.2f", freq_hz);
                  UART_msg_put((const unsigned char *)freq_buf);

                  UART_msg_put((const unsigned char *)"\r\n");

                  display_flag = 0;
               }
            }
            break;

         case 'N':
            if((msg_buf[1] == 'O') && (msg_buf[2] == 'R') && (msg_buf_idx == 3))
            {
               display_mode = NORMAL;
               UART_msg_put((const unsigned char *)"\r\nMode=NORMAL\n");
               //display_timer = 0;
            }
            else
               err = 1;
            break;
         case(NORMAL):
               {
                  if (display_flag == 1)
                  {
                     extern float freq_hz;
                     extern float flow_gpm;
                     extern float temp_c;

                     UART_msg_put((const unsigned char *)"\r\nNORMAL ");

                     // --- Flow output ---
                     UART_msg_put((const unsigned char *)" Flow: ");
                     char flow_buf[16];
                     sprintf(flow_buf, "%.2f", flow_gpm);
                     UART_msg_put((const unsigned char *)flow_buf);

                     // --- Temperature output ---
                     UART_msg_put((const unsigned char *)" Temp: ");
                     char temp_buf[16];
                     sprintf(temp_buf, "%.2f", temp_c);
                     UART_msg_put((const unsigned char *)temp_buf);

                     // --- Frequency output ---
                     UART_msg_put((const unsigned char *)" Freq: ");
                     char freq_buf[16];
                     sprintf(freq_buf, "%.2f", freq_hz);
                     UART_msg_put((const unsigned char *)freq_buf);

                     UART_msg_put((const unsigned char *)"\r\n");

                     display_flag = 0;
                  }
               }
               break;

         case 'Q':
            if((msg_buf[1] == 'U') && (msg_buf[2] == 'I') && (msg_buf_idx == 3)) 
            {
               display_mode = QUIET;
               UART_msg_put((const unsigned char *)"\r\nMode=QUIET\n");
               display_timer = 0;
            }
            else
               err = 1;
            break;

         case 'V':
            display_mode = VERSION_MODE;
            UART_msg_put((const unsigned char *)"\r\n");
            UART_msg_put((const unsigned char *) CODE_VERSION );
            UART_msg_put((const unsigned char *)"\r\nSelect  ");
            display_timer = 0;
            break;
/****************      ECEN 5803 add code as indicated   **********************/
//  Add other message types here
				 
				 
         default:
            err = 1;
      }
   }

   else 
   {                                 // Lower Case
      switch( chr ) 
      {
        default:
         err = 1;
      }
   }

   if( err == 1 )
   {
      UART_msg_put((const unsigned char *)"\n\rEntry Error!");
   }   
   else if( err == 2 )
   {
      UART_msg_put((const unsigned char *)"\n\rNot in DEBUG Mode!");
   }   
   else
   {
    msg_buf_idx = 0;          // put index to start of buffer for next message
      ;
   }
   msg_buf_idx = 0;          // put index to start of buffer for next message


}


//*****************************************************************************
///   \fn   is_hex
/// Function takes 
///  @param a single ASCII character and returns 
///  @return 1 if hex digit, 0 otherwise.
///    
//*****************************************************************************
uchar8_t is_hex(uchar8_t c)
{
   if( (((c |= 0x20) >= '0') && (c <= '9')) || ((c >= 'a') && (c <= 'f'))  )
      return 1;
   return 0;
}

/*******************************************************************************
*   \fn  DEBUG and DIAGNOSTIC Mode UART Operation
*******************************************************************************/
void monitor(void)
{

/**********************************/
/*     Spew outputs               */
/**********************************/

   switch(display_mode)
   {
      case(QUIET):
         {
             UART_msg_put((const unsigned char *)"\r\n ");
             display_flag = 0;
         }  
         break;
      case(VERSION_MODE):
         {
             display_flag = 0;
         }  
         break;         
      case(NORMAL):
         {
            if (display_flag == 1)
            {
               UART_msg_put((const unsigned char *)"\r\nNORMAL ");
               UART_msg_put((const unsigned char *)" Flow: ");
               // ECEN 5803 add code as indicated
               //  add flow data output here, use UART_hex_put or similar for 
               // numbers
               UART_msg_put((const unsigned char *)" Temp: ");
               //  add flow data output here, use UART_hex_put or similar for 
               // numbers
               UART_msg_put((const unsigned char *)" Freq: ");
               //  add flow data output here, use UART_hex_put or similar for 
               // numbers
               display_flag = 0;
            }
         }  
         break;
      case(DEBUG_MODE):
         {
            if (display_flag == 1)
            {
               UART_msg_put((const unsigned char *)"\r\nDEBUG ");
               UART_msg_put((const unsigned char *)" Flow: ");
               // ECEN 5803 add code as indicated               
               //  add flow data output here, use UART_hex_put or similar for 
               // numbers
               UART_msg_put((const unsigned char *)" Temp: ");
               //  add flow data output here, use UART_hex_put or similar for 
               // numbers
               UART_msg_put((const unsigned char *)" Freq: ");
               //  add flow data output here, use UART_hex_put or similar for 
               // numbers
               
/****************      ECEN 5803 add code as indicated   **********************/
               //  Create a display of  error counts, sensor states, and
               //  ARM Registers R0-R15
               
               //  Create a command to read a section of Memory and display it
               
               
               //  Create a command to read 16 words from the current stack 
               // and display it in reverse chronological order.
              
              
               // clear flag to ISR      
               display_flag = 0;
             }   
         }  
         break;

      default:
      {
         UART_msg_put((const unsigned char *)"Mode Error");
      }  
   }
}  
