// USART initialization def's
#define FOSC 20000000UL   //clock speed
#define BAUD 500000     //desired baud rate
//#define MYUBRR FOSC/16/BAUD-1
//#define MYUBRR (FOSC/4/BAUD-1)/2
//#define MYUBRR (FOSC/8/BAUD)-1
#define MYUBRR  0x004   // 500000
//#define MYUBRR  0x00A     // 115200



#define ADDR    0x02
#define IN_A 4
#define IN_B A3
#define   _RX   1
#define   _TX   0

/* ###################### Variables ###################### */

/*          Control loop            */
boolean control_loop = false;
boolean T1_MODE = _RX;
/*        Magnetic encoder          */
uint8_t u8byteCount;
uint8_t u8data;
uint32_t u32result = 0;
uint32_t u32send;


float current = 182.55;
uint8_t *c = (uint8_t *)&current;
float _PGAIN;
float _IGAIN;
float _DGAIN;
/*        USART variables         */
unsigned char bufferRx[32];
unsigned char bufferTx[32];
unsigned char counterRx = 0;
volatile boolean write_flag = false;
volatile boolean itsMe = false;
uint8_t num_send = 0;
boolean eRX = false;  // end of incoming buffer
volatile uint8_t error = 0;
float *rx_pnt32;

/*    --- Control variables ---   */
// Goal position
int16_t goal_position;
int16_t vel_reference;
int16_t *gp = &goal_position;
int16_t prev_vel;




float v_rad, vf_rad;

// Position (feedback) variables
int pos;
uint8_t *p = (uint8_t *)&pos;
int16_t prev_pos;
// Velocity (feedback) variables
int vel;
int velocity;
uint8_t *v = (uint8_t *)&velocity;
float vf;   // -v-elocity -f-iltered
float prev_vf;
float prev_vf1 = 0;
float prev_vf2 = 0;
float prev_vf3 = 0;
// PID variables


float ref, e, prev_e;
float uP, uI, uD;
uint8_t u;
float Kp, Ki, Kd;   // capital K
float pwm;
uint8_t duty8;
int16_t duty16;

// Filtet variables
float a = 0.1484;
float b = 0.8516;
// Lin/son variables
float af =  9.347;
float bf = 0.04626;
float cf = 0.00363;
float df = 0.3661;
//
boolean flag = false;;
boolean timeoutRX = false;

int loop_cnt = 0;
bool dir = 1;




/* Counter2 compare match interrupt - for control loop*/
ISR(TIMER2_COMPA_vect, ISR_NOBLOCK) {
  //  delayMicroseconds(70);  // read sensor time
  if (control_loop == 0) {
    position_update(pos);
    control_loop = 1;
  }
}

ISR(TIMER1_COMPA_vect, ISR_NOBLOCK) {
  //   Disable the counter (no matter the T1_MODE)
  TCCR1B = 0x00;
  TCNT1 = 0x00;
  //  digitalWrite(10, LOW);
  //   check the T1_MODE
  if (T1_MODE == _RX) {         // RX timeout occured
    //    digitalWrite(10, HIGH);
    //    digitalWrite(10, LOW);
    timeoutRX = true;
    error = 0xAA;
    // reset the rx counter
    counterRx = 0;
  } else {                       // RX was succesfull
    //    digitalWrite(10, LOW);
    //     Time to send, enable Tx
    //    enableTx();
    if (_crcRx(bufferRx)) { // CRC the incoming data
      error = 0x00;
      if (bufferRx[2] == ADDR) {
        itsMe = true;
      } else {
        itsMe = false;
      }
    } else {
      error = 0x09; // error in checksum
      if (bufferRx[2] == ADDR) {
        itsMe = true;
      } else {
        itsMe = false;
      }
    }
  }

  if (itsMe == true) {
    enableTx();
    uint8_t inst = bufferRx[4];
    num_send = fill_bufferTx(inst);
    //    digitalWrite(13, HIGH);
    if (write_flag == true) {
      write_flag = false;
      for (uint8_t i = 0; i < num_send; i++) {
        USART_Tx(bufferTx[i]);
      }
    }
    for (int i = 0; i < 300; i++) {
      asm("nop");
    }
    //    digitalWrite(13, LOW);
    // Data sent, enable Rx again
    enableRx();
  }
  itsMe = false;
}

ISR(USART_RX_vect) {

  //  digitalWrite(10, HIGH);
  //  digitalWrite(13, HIGH);
  if (counterRx != 11) {  // Check if for some reason odroid sent more than 11
    bufferRx[counterRx] = UDR0;
    counterRx++;
  } else {
    uint8_t dummy = UDR0;
  }

  // START of serial word
  if (counterRx == 1) {
    // Start counting for rx-timeout
    TCNT1 = 0x0000;
    T1_MODE = _RX;
    OCR1A = 154; // ~500 us
    //    digitalWrite(10, HIGH);
    TCCR1B |= B00001011;
  }

  // END of serial word
  if (counterRx == 11) { // data packet received before rx-timeout
    //    digitalWrite(10, LOW);
    // Stop timer1 (is at mode _RX but timeout didn't occur)
    TCCR1B = 0x00;
    TCNT1 = 0x0000;
    // And start timer1 at mode _TX
    T1_MODE = _TX;
    OCR1A = 15;
    TCCR1B |= B00001011;
    //    digitalWrite(10, HIGH);
    counterRx = 0;

  }
  //  if (counterRx>2) {
  //    if ((bufferRx[counterRx-1]==0x0F) && (bufferRx[counterRx-2]==0x0f)) {
  //      counterRx = 0;
  //      eRX = true;
  //      TCNT1 = 0x00;
  //      // Start timer1
  ////      digitalWrite(13, HIGH);
  //      TCCR1B |= B00001011;
  //      // Enable Tx and Disable Rx
  //      enableTx();
  //    }
  //  }

}


void setup() {
  pinMode(10, OUTPUT); // SPI pulse
  pinMode(4, OUTPUT);  // Direction pin
  pinMode(A3, OUTPUT);  // Direction pin
  pinMode(11, OUTPUT); // OC2A
  pinMode(5, OUTPUT);  // PWM
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  /* --------------------  Serial Iinit.  --------------------- */
  USART_Init(MYUBRR);
  //  Serial.begin(115200);

  /* ----------  Timer 2 (control loop) configuration  -------- */
  TCCR2A = 0;
  TCCR2B = 0;    //DON'T KNOW WHY - Init. Value of reg = B00000100 (!!)

  TCCR2A |= B00000010;
  TCCR2B |= B00000111;
  // DONT FORGET
  TIMSK2 |= B00000010;
  control_loop = 1;
  //  OCR0A = 155; // 14 = 1ms, 155 = 10ms
  OCR2A = 96; // (prescaler:111 -> 4.983ms period

  /* ---------- Timer 0 - Configuration (FAST_PWM) ------------ */
  TCCR0A = 0;
  TCCR0B = 0;    //DON'T WHY - Init. Value of reg = B00000100 (!!)
  TCCR0A |= B00100011;
  TCCR0B |= B00000010;  // 9.8Khz (bridge works up to 20 MHz BUT...
  /* -----------------------------------------------------------*/

  /* ---------- Timer 1 - Configuration (FAST_PWM) ------------ */
  TCCR1A = 0;
  TCCR1B = 0;    //DON'T WHY - Init. Value of reg = B00000100 (!!)
  //  TCCR1A |= B00000000;
  ////  TCCR2B |= B00000010;
  OCR1A = 15;  // for counting 20us (with 64 presc.) before ISR
  TIMSK1 |= B00000010;

  /* -----------------------------------------------------------*/

  /* ------------------  SPI configuration  ------------------- */
  DDRB = 0;  //Check if needed
  // configure SCK(PB5) and Slave Select(PB2) as output, MISO(PB4) as input
  DDRB = (1 << PB5) | (1 << PB2) | (0 << PB4);
  // configure SPI as master, SPR0=1 -> fosc/16 CHANGE: SPR0 = 0 -> fosc/4
  // 16 Mhz XTAL:
  //  SPCR = (1 << SPE) | (1 << MSTR) | (0 << CPOL) | (1 << SPR0) | (CPHA << 1);
  // 20 Mhz XTAL:
  SPCR = (1 << SPE) | (1 << MSTR) | (0 << CPOL) | (1 << SPR1) | (CPHA << 1);

  enableRx();
  bufferTx[0] = 0xFF;
  bufferTx[1] = 0xFF;
  bufferTx[2] = ADDR;

  //  Serial.begin(115200);

  digitalWrite(IN_B, LOW);
  digitalWrite(IN_A, HIGH);
  OCR0B = 0;

  ref = 0.0;
  //  Kp = 2.9554631;
  //  Kp = 0.6564;
  Kp = 0.7631;
  //  Ki = 6.4513*0.01;
  Ki = 0.008;
  Kd = 0.0;

  _PGAIN = Kp;
  _IGAIN = Ki;
  _DGAIN = Kd;

  sei();
}

void loop() {
  if (control_loop == true) {
    // New control loop

    // Position already updated inside ISR!

    // Velocity
    vel = velocity_update(pos, prev_pos); // noisy signal
    vf = a * prev_vel + b * prev_vf;
    vf = (vf + prev_vf + prev_vf1 + prev_vf2) / 4;
    velocity = (int)(vf);

    //    if ((loop_cnt <400)&&(dir == 1)) {
    //      u = 20;
    //      loop_cnt += 1;
    //    }if ((loop_cnt == 400)&&(dir == 1)) {
    //      dir = 0;
    //    }
    //    if ((loop_cnt >0)&&(dir == 0)) {
    //      u = 25;
    //      loop_cnt -=1;
    //    }else if ((loop_cnt == 0)&&(dir == 0)) {
    //      dir = 1;
    //    }

    /* ----------- CONTROLER CODE HERE ------------ */
    e = vel_reference - velocity;
    if ((e <= 0.1) && (e >= 0.1)) {
      e = 0.0;
    }
    // P-term
    uP = Kp * e;
    // I-term
    //    uI = uI + Ki*e;
    // I-term (trapezoidal)
    uI = uI + (Ki / 2) * prev_e + (Ki / 2) * e;
    // D-term
    //    uD = Kd*e - Kd*prev_e;
    // PI output (in Volts)
    u = uP + uI + uD;
    //    pwm = af*exp(bf*u) + cf*exp(df*u);
    //    duty16 = (int)(pwm);
    //    duty16 = abs(duty16);   // direction is changed by u

    // Direction+
    if (u >= 0) {
      digitalWrite(IN_A, LOW);
      digitalWrite(IN_B, HIGH);
    } else {
      digitalWrite(IN_A, HIGH);
      digitalWrite(IN_B, LOW);
    }

    // ---Sat in pwm
    //    if (duty16 > 255) {
    //      duty8 = 255;
    //    }else {
    //      duty8 = (uint8_t)duty16;
    //    }

    //    // Apply duty
    //    OCR0B = duty8;

    /* -------------------------------------------- */
    OCR0B = af * exp(bf * u) + cf * exp(df * u);
    //      if (vel_reference == 20) {
    //        OCR0B = 120;
    //      }
    //    Serial.print(u); Serial.print(", "); Serial.print(velocity);
    //    Serial.print('\n');

    // Filter variables
    prev_pos = pos;
    prev_vf3 = prev_vf2;
    prev_vf2 = prev_vf1;
    prev_vf1 = prev_vf;
    prev_vf = vf;
    prev_vel = vel;


    // Update -any- variables
    update_variables();
    // End of control loop, wait for the next timer2 interupt.
    control_loop = false;
  }

}



void USART_Init (unsigned int ubrr)
{
  UCSR0A = 0;
  UCSR0A |= (1 << U2X0);
  /* Set baud rate */
  UBRR0H = (unsigned char)(ubrr >> 8);
  UBRR0L = (unsigned char)(ubrr);
  UCSR0B = B00000000;
  // Enable receiver and transmitter
  //  UCSR0B |= (1 << RXEN0) | (1 << TXEN0);
  // Enable receiver only
  UCSR0B |= (1 << RXEN0);
  // Enable RX Complete Interrupt
  UCSR0B |= (1 << RXCIE0);
  UCSR0C = B00000000;
  // Set frame: 8data, 1 stp
  UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);

  //  enableRx();

}

void USART_Tx (char data)
{
  /* Wait for empty transmit buffer */
  while ( !(UCSR0A & (1 << UDRE0)) ) {
  }
  /* Put data into buffer, sends the data */
  UDR0 = data;
}



void enableRx() {
  // Check the Transmit complete flag
  while (!( UCSR0A & (1 << UDRE0)));
  for (uint8_t i = 0; i < 25; i++) {
    asm("nop");
  }
  UCSR0B = B00000000;
  // Enable receiver only
  UCSR0B |= (1 << RXEN0) | (1 << RXCIE0);
  // Enable RX Complete Interrupt
  //  UCSR0B |= (1 << RXCIE0);
  // Disable TX
  // UART no longer override the TxDn port.
  // Disabling will be immediate as we already checked if there are still bytes to transmit
  UCSR0B &= ~(1 << TXEN0);
  //  pinMode(1, OUTPUT);
  //  digitalWrite(1, LOW);


  counterRx = 0;
  //  digitalWrite(13, LOW);
}


void enableTx() {
  bitClear(UCSR0B, RXCIE0);
  bitClear(UCSR0B, RXEN0);
  //  bitClear(UCSR0B, TXEN0);
  //Enable Tx (overides port condition)
  bitSet(UCSR0B, TXEN0);
}



void position_update(int &pos) {
  uint8_t u8data;  uint32_t u32result;
  // Pulse to initiate new transfer
  digitalWrite(10, HIGH);
  digitalWrite(10, LOW);
  //Receive the 3 bytes (AS5145 sends 18bit word)
  for (uint8_t byteCount = 0; byteCount < 3; byteCount++) {
    u32result <<= 8;  // left shift the result so far - first time shifts 0's-no change
    SPDR = 0xFF;  // send 0xFF as dummy (triggers the transfer)
    while ( (SPSR & (1 << SPIF)) == 0);  // wait until transfer complete
    u8data = SPDR;  // read data from SPI register
    u32result |= u8data;  //store the byte
  }
  // TODO!  Check the flags before continue
  u32result >>= 12;
  int *ssi_pnt16 = (int *)&u32result;
  pos = *ssi_pnt16 - 4096;
}

int velocity_update(int pos, int prev_pos) {
  if ((pos - prev_pos) < -400) {
    pos = pos + 4096;
  } else if ((pos - prev_pos) > 3700) {
    prev_pos = prev_pos + 4096;
  }
  return (pos - prev_pos);
}

void update_variables() {
  Kp = _PGAIN;
  Ki = _IGAIN;
  Kd = _DGAIN;
}


