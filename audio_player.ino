#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

#define SAMPLE_RATE 8000
#define SPEAKER_PIN 11;

#include "sounddata.h"

volatile uint16_t smp_pos;
uint8_t last_sample;

void startPlayback() {
  DDRB |= (1<<DDB3); // Setup 13th (led) pin for output
  
  // Setup timer2 to do PWM on pin 11 (OCR2A)
  
  // Use internal clock (disable async on timer2) (datasheet p.133)
  ASSR &= ~((1<<EXCLK) | (1<<AS2));
  
  // Set fast PWM mode  (p.130)
  TCCR2A |= (1<<WGM21) | (1<<WGM20);
  TCCR2B &= ~(1<<WGM22);
  
  // Do non-inverting PWM on pin OC2A (p.128)
  TCCR2A = (TCCR2A | (1<<COM2A1)) & ~(1<<COM2A0);
  TCCR2A &= ~((1<<COM2B1) | (1<<COM2B0));
  
  // No prescaler (p.158)
  TCCR2B = (TCCR2B & ~((1<<CS22) | (1<<CS21))) | (1<<CS20);
  
  // Set initial pulse width to the first sample.
  OCR2A = pgm_read_byte(&sounddata_data[0]);
  
  
  // Set up Timer 1 to send a sample every interrupt.
  
  cli();
  
  // Set CTC mode (Clear Timer on Compare Match) (p.110)
  TCCR1B = (TCCR1B & ~(1<<WGM13)) | (1<<WGM12);
  TCCR1A &= ~((1<<WGM11) | (1<<WGM10));
  
  // No prescaler (p.110)
  TCCR1B = (TCCR1B & ~((1<<CS12) | (1<<CS11))) | (1<<CS10);
  
  // Set the 16-bit compare register (OCR1A).
  OCR1A = F_CPU / SAMPLE_RATE;    // 16000000 / 8000 = 2000
  
  // Enable interrupt when TCNT1 == OCR1A (p.112)
  TIMSK1 |= (1<<OCIE1A);

  last_sample = pgm_read_byte(&sounddata_data[sounddata_length-1]);
  
  sei();
}

void stopPlayback()
{
  // Disable playback per-sample interrupt.
  TIMSK1 &= ~(1<<OCIE1A);
  
  // Disable the per-sample timer completely.
  TCCR1B &= ~(1<<CS10);
  
  // Disable the PWM timer.
  TCCR2B &= ~(1<<CS20);
  
  PORTB |= (1<<PB3);

  Serial.println("End");
}

void setup() {
  startPlayback();
  Serial.begin(9600);

  Serial.println("Start");
}

// This is called at 8000 Hz to load the next sample.
ISR(TIMER1_COMPA_vect) {
  if (smp_pos < sounddata_length) {
    OCR2A = pgm_read_byte(&sounddata_data[smp_pos]);
  }
  else {
    if (smp_pos == sounddata_length + last_sample)
      stopPlayback();
    else
      OCR2A = sounddata_length + last_sample - smp_pos;
  }
  
  ++smp_pos;
}

void loop() {
}
