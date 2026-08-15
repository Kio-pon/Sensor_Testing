#include "Encoders.h"
#include "Config.h"

volatile long countFL = 0;
volatile long countFR = 0;
volatile long countRL = 0;
volatile long countRR = 0;

static uint8_t lastPortB = 0;

void initEncoders() {
  pinMode(FL_ENCODER_A, INPUT_PULLUP);
  pinMode(FL_ENCODER_B, INPUT_PULLUP);
  pinMode(FR_ENCODER_A, INPUT_PULLUP);
  pinMode(FR_ENCODER_B, INPUT_PULLUP);
  pinMode(RL_ENCODER_A, INPUT_PULLUP);
  pinMode(RL_ENCODER_B, INPUT_PULLUP);
  pinMode(RR_ENCODER_A, INPUT_PULLUP);
  pinMode(RR_ENCODER_B, INPUT_PULLUP);

  // Enable Pin Change Interrupts for PORTB (PCINT0 to PCINT7)
  PCICR |= (1 << PCIE0); 
  // Enable all 8 pins on PORTB to trigger the interrupt
  PCMSK0 |= 0xFF;

  lastPortB = PINB;
}

void resetEncoders() {
  noInterrupts();
  countFL = 0;
  countFR = 0;
  countRL = 0;
  countRR = 0;
  interrupts();
}

long getEncoderFL() { long c; noInterrupts(); c = countFL; interrupts(); return c; }
long getEncoderFR() { long c; noInterrupts(); c = countFR; interrupts(); return c; }
long getEncoderRL() { long c; noInterrupts(); c = countRL; interrupts(); return c; }
long getEncoderRR() { long c; noInterrupts(); c = countRR; interrupts(); return c; }

// Single Interrupt Service Routine for all 4 encoders (8 pins)
ISR(PCINT0_vect) {
  uint8_t currentPortB = PINB;
  uint8_t changes = currentPortB ^ lastPortB;

  // FL (A=50/PB3, B=51/PB2)
  if (changes & (1 << PB3)) {
    if ((currentPortB >> PB3) & 1) { // A rose
      if ((currentPortB >> PB2) & 1) countFL--; else countFL++;
    } else { // A fell
      if ((currentPortB >> PB2) & 1) countFL++; else countFL--;
    }
  }

  // FR (A=52/PB1, B=53/PB0)
  if (changes & (1 << PB1)) {
    if ((currentPortB >> PB1) & 1) {
      if ((currentPortB >> PB0) & 1) countFR++; else countFR--;
    } else {
      if ((currentPortB >> PB0) & 1) countFR--; else countFR++;
    }
  }

  // RL (A=10/PB4, B=11/PB5)
  if (changes & (1 << PB4)) {
    if ((currentPortB >> PB4) & 1) {
      if ((currentPortB >> PB5) & 1) countRL--; else countRL++;
    } else {
      if ((currentPortB >> PB5) & 1) countRL++; else countRL--;
    }
  }

  // RR (A=12/PB6, B=13/PB7)
  if (changes & (1 << PB6)) {
    if ((currentPortB >> PB6) & 1) {
      if ((currentPortB >> PB7) & 1) countRR++; else countRR--;
    } else {
      if ((currentPortB >> PB7) & 1) countRR--; else countRR++;
    }
  }

  lastPortB = currentPortB;
}
