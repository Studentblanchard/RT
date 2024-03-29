/* Name: teatimer.c
 * Author: Trevor Blachard
 * Assignment: 1
 * Instructor: Jacky Baltes
 * Course: Real-time systems
 */

/* Includes */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <lcd_functions.h>
#include <lcd_driver.h>

#include <stdio.h>
#include <inttypes.h>

/* Includes */

/* Types */

typedef enum state_codes {MENU, SELECTION, COUNTDOWN, READY, READTIME} state_code_t;

typedef enum return_codes {OK, CANCEL, RESET, READ} return_code_t;

typedef enum button_type {UP = 6, DOWN = 7, CENTER = 4, NONE_ = 0} button_type_t;

typedef enum click_type {NONE, SINGLE, DOUBLE} click_type_t;

typedef enum tea_type {BLACK = 0, EARLGREY = 1, FRUIT = 2, CUSTOM = 3} tea_type_t;

typedef enum timer_type {TEATIMER} timer_type_t;

typedef struct {
  state_code_t src;
  return_code_t rc;
  state_code_t dest;
} transition_t;

typedef struct {
  tea_type_t tea_type;
  char *tea_name;
  uint32_t brew_time;// in seconds
} tea_t;

typedef struct {
  timer_type_t type;
  int duration;// in milliseconds
  uint32_t start;
} timer_t;

typedef struct {
  button_type_t type;
  int hysterisis;
  click_type_t state;
} button_t;

/* Types */

/* Prototypes */

return_code_t countdown_state(void);
return_code_t menu_state(void);
return_code_t selection_state(void);
return_code_t ready_state(void);
return_code_t readtime_state(void);
button_type_t read_button(void);
void to_mm_ss(uint32_t seconds);
state_code_t get_next_state(return_code_t rc, state_code_t cur_state);

/* Prototypes */

/* State Function Pointers */

int (*state[])(void) = {menu_state, selection_state, countdown_state, ready_state, readtime_state};

/* State Function Pointers */

/* Tables */

transition_t state_trans[] = {
  { MENU, OK, SELECTION },
  { MENU, READ, READTIME },
  { READTIME, OK, COUNTDOWN },
  { SELECTION, OK, COUNTDOWN },
  { COUNTDOWN, OK, READY },
  { COUNTDOWN, RESET, COUNTDOWN },
  { COUNTDOWN, CANCEL, MENU },
  { READY, OK, MENU }
};

tea_t tea_instructions[] = {
  { BLACK, "Black Tea", 180000 },
  { EARLGREY, "Earl Grey", 300000 },
  { FRUIT, "Fruit Tea", 480000 },
  { CUSTOM, "Custom", 0 }
};

timer_t timers[] = {
  {TEATIMER , 0, 0}
};

button_t buttons[] = {
  {CENTER , 0, 0},
  {UP, 0, 0},
  {DOWN, 0, 0}
};

/* Tables */

/* Constants */

static const int C_BUTTON = 0;
static const int D_BUTTON = 2;
static const int U_BUTTON = 1;

static const int NUM_STATE = 8;

static const int THRESHOLD = 5000;

static const int HYST_MIN = 0;
static const int HYST_LOW = 1;
static const int HYST_HIGH = 4;
static const int HYST_MAX = 5;

static const int LOW = 10;
static const int HIGH = 100;

static const uint8_t BUTTON_MASK = 0b11001000;

/* Constants */

/* Globals */

int selected_tea = BLACK;

int speed;

/* Globals */

int InitButtons( )
{
    /* Enable the joystick for reading */
    DDRB = DDRB & (~(1 << PB4) | (1 << PB6) | (1 << PB7));
    PORTB = PORTB | ((1 << PB4) | (1 << PB6) | (1 << PB7));
    /* Enable the joystick for reading */

    /* Prepare for speaker */
    DDRB = DDRB | (1 << PB5);
    /* Prepare for speaker */ 

    /* Disable pin change interrupt */
    PCMSK1 = 0x00;
    PCMSK0 = 0x00;
    /* Disable pin change interrupt */

    DIDR0 = 0x00;
    DIDR1 = 0x00;

    return 0;
}

int main(void){

  state_code_t cur_state = MENU;
  return_code_t rc;
  int (* state_function)(void);
  cli();
  LCD_Init();
  InitButtons();
  sei();

  for(;;){//enter the main loop
    state_function = state[cur_state];
    rc = state_function();
    cur_state = get_next_state(rc, cur_state);
    _delay_ms(100);
  }

  return 0;// never get here
}

/* The State Functions */

return_code_t menu_state(){
  reset_buttons();
  for(;;){
    LCD_puts(tea_instructions[selected_tea].tea_name, 0);
    count_hysterisis();

    switch (read_button()) {
      case CENTER:
        if(selected_tea == CUSTOM)
          return READ;
        return OK;
      case UP:
        selected_tea = (selected_tea + 1) % 4;
        break;
      case DOWN:
        selected_tea = (selected_tea + 3) % 4;
        break;
      default:
	break;
    }
    _delay_ms(5);
  }
}

return_code_t selection_state(){
  reset_buttons();
  for(;;){
    to_mm_ss(tea_instructions[selected_tea].brew_time);
    count_hysterisis();

    switch (read_button()) {
      case CENTER:
        timers[TEATIMER].duration = tea_instructions[selected_tea].brew_time;
        return OK;
    }
    _delay_ms(10);
  }
}

return_code_t countdown_state(){
  reset_buttons();
  click_type_t click = NONE;
  button_type_t button = NONE_;
  int double_click_timer = 50;//play around with this value

  uint32_t remaining_time = tea_instructions[selected_tea].brew_time;

  for(;;){
    to_mm_ss(remaining_time);
    count_hysterisis();

    button = read_button();

    if(button == CENTER && click == NONE)
    {
      click = SINGLE;
      double_click_timer = 50;
    }

    if(button == NONE_ && click == SINGLE)
      click = DOUBLE;

    if(button == CENTER && click == DOUBLE)
      return CANCEL;

    if(click != NONE && double_click_timer <= 0)
      return RESET;

    double_click_timer -= 10; 

    if(remaining_time <= 0)
      return OK;

    _delay_ms(10);
    remaining_time -= 100;

  }
}

return_code_t ready_state(){
  reset_buttons();
  notify();
  LCD_puts("Your tea is ready", 1);
  _delay_ms(5000);
  return OK;
}

return_code_t readtime_state(){
  reset_buttons();
  uint32_t delta_time = 0;
  int secondary_hysterisis = 0;

  speed = LOW;

  for(;;){
      to_mm_ss(delta_time);
      count_hysterisis();

      if(speed == LOW && secondary_hysterisis >= 500)
        speed = HIGH;

      switch (read_button()) {
        case CENTER:
          timers[TEATIMER].duration = delta_time - (delta_time % 1000);/* remove excess milliseconds */
          return OK;
        case UP:
          delta_time += 10 * speed;/* milliseconds */
          secondary_hysterisis++;
          break;
        case DOWN:
          delta_time -= 10 * speed;/* milliseconds */
          secondary_hysterisis++;
          break;
        default:
	  secondary_hysterisis = 0;
          speed = LOW;
      }
      _delay_ms(10);
  }
}

/* State Functions */

/* Button Functions */

void reset_buttons(){
    buttons[C_BUTTON].hysterisis = 0;
    buttons[D_BUTTON].hysterisis = 0;
    buttons[U_BUTTON].hysterisis = 0;
    buttons[C_BUTTON].state = NONE;
    buttons[D_BUTTON].state = NONE;
    buttons[U_BUTTON].state = NONE;
}

button_type_t read_button(){
  if(buttons[C_BUTTON].state == SINGLE)/* or double */
  {
    buttons[C_BUTTON].hysterisis = 0;
    buttons[C_BUTTON].state = NONE;
    return CENTER;
  }

  if(buttons[D_BUTTON].state == SINGLE)/* or double */
  {
    buttons[D_BUTTON].hysterisis = 0;
    buttons[D_BUTTON].state = NONE;
    return DOWN;
  }

  if(buttons[U_BUTTON].state == SINGLE)/* or double */
  {
    buttons[U_BUTTON].hysterisis = 0;
    buttons[U_BUTTON].state = NONE;
    return UP;
  }

  return NONE;
}

void check_button_state(button_t *btn){
  if(btn->hysterisis > HYST_HIGH)
  {
    if(btn->hysterisis > HYST_MAX)
      btn->hysterisis = HYST_MAX;
    btn->state = SINGLE;
  }


  if(btn->hysterisis < HYST_LOW)
  {
    if(btn->hysterisis < HYST_MIN)
      btn->hysterisis = HYST_MIN;
    btn->state = NONE_;
  }
}

void count_hysterisis(){
  uint8_t data = PINB;
  if (!(data & (1 << PB4)))
  {
      buttons[C_BUTTON].hysterisis++;
      buttons[D_BUTTON].hysterisis--;
      buttons[U_BUTTON].hysterisis--;
  }
  else if (!(data & (1 << PB7)))
  {
      buttons[D_BUTTON].hysterisis++;
      buttons[C_BUTTON].hysterisis--;
      buttons[U_BUTTON].hysterisis--;
  }
  else if (!(data & (1 << PB6)))
  {
      buttons[U_BUTTON].hysterisis++;
      buttons[C_BUTTON].hysterisis--;
      buttons[D_BUTTON].hysterisis--;
  }
  else
  {
      buttons[C_BUTTON].hysterisis--;
      buttons[D_BUTTON].hysterisis--;
      buttons[U_BUTTON].hysterisis--;
  }
  check_button_state(&buttons[C_BUTTON]);
  check_button_state(&buttons[U_BUTTON]);
  check_button_state(&buttons[D_BUTTON]);
}

/* Button Functions */

/* Helper Functions */

state_code_t get_next_state(return_code_t rc, state_code_t cur_state){
  for(int i = 0; i < NUM_STATE; i ++){
    transition_t t = state_trans[i];
    if( t.src == cur_state && t.rc == rc )
      return t.dest;
  }
  return 0;
}

void to_mm_ss(uint32_t mseconds){
  char buffer[8];
  sprintf(buffer, "%02u %02u", (uint32_t) (mseconds / (uint32_t) 60000), (uint32_t) (mseconds % (uint32_t) 60000)/1000);
  LCD_puts(buffer, 0);
}

void notify(){
  int buzzer_count = 500;
  for(;;){
    if(buzzer_count % 2)
      PORTB = PORTB | (1 << PB5);
    else 
      PORTB = PORTB & ~(1 << PB5);
    _delay_ms(5);
    buzzer_count --;
    if(buzzer_count < 0)
     break;
  }
  return;
}

/* Helper Functions */
