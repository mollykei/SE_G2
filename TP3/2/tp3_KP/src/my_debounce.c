/*
 * my_debounce.c
 */

#include "sapi.h"
#include "my_debounce.h"

my_ButtonState_t buttonState;

my_ButtonState_t my_ButtonError( void )
{
	buttonState = STATE_BUTTON_UP;
}

my_ButtonState_t my_ButtonInit( void )
{
	buttonState = STATE_BUTTON_UP;
}

void my_ButtonUpdate( my_gpio_map_t tecla, bool_t * transicion )
{

   switch( buttonState )	{

      case STATE_BUTTON_UP:
         /* CHECK TRANSITION CONDITIONS */
         if( !my_gpio_read(tecla) ){
            buttonState = STATE_BUTTON_FALLING;
         }

         *transicion = FALSE;
         break;

      case STATE_BUTTON_DOWN:
         /* CHECK TRANSITION CONDITIONS */
         if( my_gpio_read(tecla) ){
            buttonState = STATE_BUTTON_RISING;
         }

         *transicion = FALSE;
         break;

      case STATE_BUTTON_FALLING:

    	  if( !my_gpio_read(tecla) ){
                buttonState = STATE_BUTTON_DOWN;
                *transicion = TRUE;
    	  } else{
                buttonState =  STATE_BUTTON_UP;
                *transicion = FALSE;
    	  }
    	  break;

      case STATE_BUTTON_RISING:

    	  if( my_gpio_read(tecla) ){
                buttonState = STATE_BUTTON_UP;
                *transicion = TRUE;

    	  } else{
                buttonState = STATE_BUTTON_DOWN;
                *transicion = FALSE;
    	  }
    	  break;

      default:
         my_ButtonError();
      break;
   }
}
