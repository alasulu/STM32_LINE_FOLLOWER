#include "stm32f10x.h"
#include "Clock_Config.h"
#include "Delay_ms.h"
#include "stdio.h"

/* ----------------- SR04 Pins on Port B (example) ----------------
   Adjust PB12, PB13 to suit your wiring on the NUCLEO board.
   TRIG = PB12  (output)
   ECHO = PB13  (input)
   --------------------------------------------------------------- */
#define TRIG_PIN   12
#define ECHO_PIN   13

/* Desired distance from the board in centimeters */
#define DESIRED_DISTANCE_CM  70.0f

/* Function Prototypes */
void pwm (void);
void pwm2 (void);
void PID_control(void);
void sensor_left (int Cl3,int Cl4);
void sensor_right (int Cr3,int Cr4);
//void sharp_turn(void);
void Ultrasonic_Init(void);
float getDistance(void);

/* Original data and sensor variables */
int data[8] = { 0, 0, 0, 0, 0, 0, 0, 0};
static int SensorData = 0x00000000;
int position;
float Kp = 0.015;  
float Ki = 0.003;  
float Kd = 10;   
int P, I, D;
int errors[10] = {0,0,0,0,0,0,0,0,0,0};
int error_sum = 0;

/* ---------------- Adjusted from const to normal globals ---------- */
uint8_t maxspeedr = 100;
uint8_t maxspeedl = 100;
uint8_t basespeedr = 50;
uint8_t basespeedl = 50;
/* ------------------------------------------------------------------ */

const int ARR_const = 13;
int lastError = 0;
int i;
int k;
int error;
int motorspeed;
int motorspeedl;
int motorspeedr;
int varan = 0;
int actives = 0;
int last_end = 0;
int last_idle = 0;

/* --------------------------- initGPIO --------------------------- */
void initGPIO(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // Enable port A clock
  GPIOA->CRL &= ~(0xFFFFFFFF);

  RCC->APB2ENR |= (1<<3); // GPIOB clock enable

  // PB0, PB1, PB10, PB"11 as alternate function push-pull (for PWM out)
  // Make sure to configure them as needed for your hardware setup
  GPIOB->CRL |=  ( (1<<0)|(1<<1)|(1<<3) ); // 50 MHz alt PP on PB0
  GPIOB->CRL &= ~( (1<<2) );
  
  GPIOB->CRL |=  ( (1<<4)|(1<<5)|(1<<7) );
  GPIOB->CRL &= ~( (1<<6) );
  
  GPIOB->CRH |=  ( (1<<8)|(1<<9)|(1<<11) );
  GPIOB->CRH &= ~( (1<<10) );
  
  GPIOB->CRH |=  ( (1<<12)|(1<<13)|(1<<15) );
  GPIOB->CRH &= ~( (1<<14) );
}

/* --------------------------- Buzzer_Init -------------------------
   Configure PB8 as a push-pull output to drive an active buzzer.
   If you have a passive buzzer, you’d need a PWM approach instead.
   -------------------------------------------------------------- */
void Buzzer_Init(void)
{
  // PB8 lies in CRH (pins 8..15). For PB8, the configuration bits are CRH [3:0].
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;  // Make sure Port B clock is enabled
  // Clear the 4 bits for PB8, then set as 50 MHz, push-pull = 0x3
  GPIOB->CRH &= ~(0xF << 0);  
  GPIOB->CRH |=  (0x3 << 0);

  // Initially buzzer off
  GPIOB->BRR = (1 << 8);
}

/* --------------------------- Buzzer_On -------------------------- */
void Buzzer_On(void)
{
  GPIOB->BSRR = (1 << 8); // Set PB8 High
}

/* --------------------------- Buzzer_Off ------------------------- */
void Buzzer_Off(void)
{
  GPIOB->BRR = (1 << 8);  // Set PB8 Low
}

/* -------------------------- motor_control ------------------------ */
void motor_control(int motorspeedl, int motorspeedr) 
{
  if(motorspeedl < 0)
  {
    sensor_right (ARR_const*0, -1*ARR_const*motorspeedl);
  }
  else
  {
    sensor_right (ARR_const*motorspeedl, ARR_const*0);
  }

  if(motorspeedr < 0)
  {
    sensor_left (ARR_const*0, -1*ARR_const*motorspeedr);
  }
  else
  {
    sensor_left (ARR_const*motorspeedr, ARR_const*0);
  }
}

/* ----------------------- QTR8_config, QTR8_Read ------------------ */
void QTR8_config(void)
{
  GPIOA->CRL |= (3<<28) | (3<<24) | (3<<20) | (3<<16) | (3<<12) | (3<<8) | (3<<4) | (3<<0);  
  GPIOA->CRL &= ~(GPIO_CRL_CNF0) & ~(GPIO_CRL_CNF1) & ~(GPIO_CRL_CNF2) & ~(GPIO_CRL_CNF3) & 
                 ~(GPIO_CRL_CNF4) & ~(GPIO_CRL_CNF5) & ~(GPIO_CRL_CNF6) & ~(GPIO_CRL_CNF7);
  GPIOA->ODR |= (1 << 7) | (1 << 6)| (1 << 5)| (1 << 4)| (1 << 3)| (1 << 2)| (1 << 1)| (1 << 0); 
  delay_us(12);

  GPIOA->CRL &= ~(3<<28) & ~(3<<24) & ~(3<<20) & ~(3<<16) & ~(3<<12) & ~(3<<8) & ~(3<<4) & ~(3<<0);
  GPIOA->CRL |= (2<<30) | (2<<26) | (2<<22) | (2<<18) | (2<<14) | (2<<10) | (2<<6) | (2<<2);
  delay_ms(6);
}

int QTR8_Read()
{
  QTR8_config();
  int pos = 0;
  int var = 0;
  int active = 0;
  
  if(((GPIOA->IDR & (1<<0)) == (1<<0)))
  {
    SensorData = 0x00000001;  
    pos += 1000;
    active++;
    last_end=1;
  }
  if(((GPIOA->IDR & (1<<1)) == (1<<1)))
  {
    SensorData = 0x00000010;  
    pos += 2000;
    active++;
  } 
  if(((GPIOA->IDR & (1<<2)) == (1<<2)))
  {
    SensorData = 0x00000100;  
    pos += 3000;
    active++;
  }
  if(((GPIOA->IDR & (1<<3)) == (1<<3)))
  {
    SensorData = 0x00001000;  
    pos += 4000;
    active++;
  }
  if(((GPIOA->IDR & (1<<4)) == (1<<4)))
  {
    SensorData = 0x00010000;  
    pos += 5000;
    active++;
  }
  if(((GPIOA->IDR & (1<<5)) == (1<<5)))
  {
    SensorData = 0x00100000;  
    pos += 6000;
    active++;
  }
  if(((GPIOA->IDR & (1<<6)) == (1<<6)))
  {
    SensorData = 0x01000000;  
    pos += 7000;
    active++;
  }
  if(((GPIOA->IDR & (1<<7)) == (1<<7)))
  {
    SensorData = 0x10000000;  
    pos += 8000;
    active++;
    last_end=0;   
  }
  
  data[0] = (GPIOA->IDR & (1<<0));
  data[1] = (GPIOA->IDR & (1<<1));
  data[2] = (GPIOA->IDR & (1<<2));
  data[3] = (GPIOA->IDR & (1<<3));
  data[4] = (GPIOA->IDR & (1<<4));
  data[5] = (GPIOA->IDR & (1<<5));
  data[6] = (GPIOA->IDR & (1<<6));
  data[7] = (GPIOA->IDR & (1<<7));

  position = pos/((active==0)?1:active);
  actives = active;
  varan = var;

  if (actives == 0) last_idle++;
  else last_idle = 0;

  return pos/((active==0)?1:active);
}

/* ----------------------------- pwm ------------------------------ */
void pwm(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;   // enable timer2
  AFIO->MAPR   |= AFIO_MAPR_TIM2_REMAP_FULLREMAP;
  
  TIM2->CCER  |= TIM_CCER_CC4E; // capture/compare 4 enabled
  TIM2->CCER  |= TIM_CCER_CC3E; // capture/compare 3 enabled
  TIM2->CR1   |= TIM_CR1_ARPE;  // Auto-reload preload enable
  TIM2->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE;
  TIM2->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE;

  // PWM freq = Fclk / PSC / ARR = 72MHz / 72 / 20000 = 50 Hz (for servo style)
  TIM2->PSC = 72-1;  
  TIM2->ARR = 40000;
}

/* ----------------------------- pwm2 ----------------------------- */
void pwm2(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // enable timer3

  TIM3->CCER  |= TIM_CCER_CC4E; 
  TIM3->CCER  |= TIM_CCER_CC3E; 
  TIM3->CR1   |= TIM_CR1_ARPE; 
  TIM3->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE; 
  TIM3->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE; 

  // PWM freq = 72MHz / 72 / 10000 = 100 Hz, for example
  TIM3->PSC = 72-1; 
  TIM3->ARR = 1500;
}

/* --------------------- sensor_left, sensor_right ---------------- */
void sensor_left (int Cl3,int Cl4)
{
  TIM2->CCR4 = Cl4; 
  TIM2->CCR3 = Cl3; 
  TIM2->EGR  |= TIM_EGR_UG; 
  TIM2->CR1  |= TIM_CR1_CEN; 
}

void sensor_right (int Cr3,int Cr4)
{
  TIM3->CCR4 = Cr4; 
  TIM3->CCR3 = Cr3; 
  TIM3->EGR  |= TIM_EGR_UG; 
  TIM3->CR1  |= TIM_CR1_CEN; 
}

/* ------------------------------ sharp_turn ----------------------- */
//void sharp_turn(void)
//{
//  if (last_idle < 25)
//  {
//    if (last_end == 1)
//    {
//      motorspeedl = 15;
//      motorspeedr = -25;
//      motor_control(motorspeedl, motorspeedr);
//    }
//    else if(last_end == 0)
//    {
//      motorspeedl = -25;
//      motorspeedr = 15;
//      motor_control(motorspeedl, motorspeedr);
//    }
//		else;
//  }
//  else 
//  {
//    if (last_end == 1)
//    {
//      motorspeedl = 70;
//      motorspeedr = -53;
//    }
//    else
//    {
//      motorspeedl = -53;
//      motorspeedr = 70;
//    }
//  }
//}

/* -------------------- past_errors, errors_sum -------------------- */
void past_errors(int error) 
{
  for (i = 9; i > 0; i--)
    errors[i] = errors[i-1];
  errors[0] = error;
}

int errors_sum(int index)
{
  int sum = 0;
  for (k = 0; k < index; k++)
  {
    sum += errors[k];
  }
  return sum;
}

/* --------------------------- PID_control -------------------------- */
void PID_control(void)
{
  position = QTR8_Read();
  int error = 3750 - position;
  past_errors(error);

  P = error;
  I = errors_sum(5);
  D = error - lastError;

  lastError = error;
  
  motorspeed = (int)(P*Kp + I*Ki + D*Kd);

  motorspeedl = basespeedl + motorspeed;
  motorspeedr = basespeedr - motorspeed;

  if (motorspeedl > maxspeedl)  motorspeedl = maxspeedl;
  if (motorspeedr > maxspeedr)  motorspeedr = maxspeedr;

  if (actives == 0)
  {
    // If no line is detected
//    sharp_turn();
  }
  else
  {
    motor_control(motorspeedl, motorspeedr);
  }
}

/* =====================================================================
   =============== ULTRASONIC (SR04) SENSOR CODE START =================
   ===================================================================== */

/* Configure PB12 as Output (Trigger), PB13 as Input (Echo) */
void Ultrasonic_Init(void)
{
  // Enable port B clock (already done above, but safe to do again)
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

  // PB12 (TRIG) as Push-Pull Output @ 50MHz
  if (TRIG_PIN < 8) 
  {
    // Configure CRL for PB12
    GPIOB->CRH &= ~(0xF << ((TRIG_PIN-8)*4));   // clear bits
    GPIOB->CRH |=  (0x3 << ((TRIG_PIN-8)*4));   // 0x3 = 50 MHz, push-pull
  }
  else
  {
    // if needed for CRL, else for CRH
    GPIOB->CRH &= ~(0xF << ((TRIG_PIN-8)*4));
    GPIOB->CRH |=  (0x3 << ((TRIG_PIN-8)*4));
  }

  // PB13 (ECHO) as input floating
  if (ECHO_PIN < 8)
  {
    // CRL
    GPIOB->CRL &= ~(0xF << (ECHO_PIN*4));
    GPIOB->CRL |=  (0x4 << (ECHO_PIN*4));   // 0x4 = input floating
  }
  else
  {
    // CRH
    GPIOB->CRH &= ~(0xF << ((ECHO_PIN-8)*4));
    GPIOB->CRH |=  (0x4 << ((ECHO_PIN-8)*4));
  }

  // Make sure Trigger pin is initially low
  GPIOB->BRR = (1 << TRIG_PIN);
}

/**
 * @brief  Returns the measured distance in centimeters using SR04
 * @note   Uses Timer2->CNT for microsecond timing (already set up).
 *         Speed of sound ~ 340 m/s -> ~29 us/cm or 58 us/2cm
 *         Distance (cm) = Pulse_width_in_microseconds / 58
 */
float getDistance(void)
{
  uint32_t startTime, stopTime;
  float distance;

  // Send 10us trigger pulse
  GPIOB->BSRR = (1 << TRIG_PIN);  // TRIG high
  delay_us(10);
  GPIOB->BRR  = (1 << TRIG_PIN);  // TRIG low

  // Wait for ECHO to go high
  TIM2->CNT = 0;
  while(!(GPIOB->IDR & (1 << ECHO_PIN)))
  {
    if (TIM2->CNT > 60000)  // ~60ms timeout
      return -1; // No echo => out of range or sensor not responding
  }
  
  // ECHO is high, get start
  TIM2->CNT = 0;
  while(GPIOB->IDR & (1 << ECHO_PIN))
  {
    if (TIM2->CNT > 60000)
      break;  // Too long => out of max range
  }
  stopTime = TIM2->CNT;

  // Convert to centimeters
  // Typically distance in cm = (time in us) / 58.0
  distance = (float)stopTime / 58.f;

  return distance;
}

/* =====================================================================
   =============== ULTRASONIC (SR04) SENSOR CODE END ===================
   ===================================================================== */

int main(void)
{
  initClockPLL();
  initGPIO();
  TIM2Config();  // For microsecond timing (Delay functions)
  pwm();
  pwm2();

  /* Initialize Ultrasonic sensor pins (Trigger PB12, Echo PB13) */
  Ultrasonic_Init();
	Buzzer_Init();
	
  while(1)
  {
		    /* ------------------------ Line Follow PID ------------------------ */
    PID_control();
    /* ------------------- DISTANCE-BASED SPEED CONTROL ------------------
       Here we read the distance and modify the base speeds so that the
       robot tries to keep ~60 cm from the board. You can refine as needed.
    */
		
		   float dist = getDistance(); 
    if (dist > 0) // valid reading
    {
      // Example logic: if far away, speed up, if too close, slow down
			if (dist < (DESIRED_DISTANCE_CM - 45))
      {
        basespeedl = 0;
        basespeedr = 0;
				Buzzer_On();
      }
			else if (dist < (DESIRED_DISTANCE_CM - 43))
      {
        basespeedl = 15;
        basespeedr = 15;
				Buzzer_Off();
      }
						else if (dist < (DESIRED_DISTANCE_CM - 41))
      {
        basespeedl = 16;
        basespeedr = 16;
				Buzzer_Off();
      }
						else if (dist < (DESIRED_DISTANCE_CM - 39))
      {
        basespeedl = 17;
        basespeedr = 17;
				Buzzer_Off();
      }
						else if (dist < (DESIRED_DISTANCE_CM - 37))
      {
        basespeedl = 18;
        basespeedr = 18;
				Buzzer_Off();
      }
						else if (dist < (DESIRED_DISTANCE_CM - 35))
      {
        basespeedl = 19;
        basespeedr = 19;
				Buzzer_Off();
      }
						else if (dist < (DESIRED_DISTANCE_CM - 33))
      {
        basespeedl = 20;
        basespeedr = 20;
				Buzzer_Off();
      }
			      else if (dist < (DESIRED_DISTANCE_CM - 31))
      {
        basespeedl = 22;
        basespeedr = 22;
				Buzzer_Off();
      }
			      else if (dist < (DESIRED_DISTANCE_CM - 29))
      {
        basespeedl = 24;
        basespeedr = 24;
				Buzzer_Off();
      }
			      else if (dist < (DESIRED_DISTANCE_CM - 25))
      {
        basespeedl = 25;
        basespeedr = 25;
				Buzzer_Off();
      }
			      else if (dist < (DESIRED_DISTANCE_CM - 23))
      {
        basespeedl = 26;
        basespeedr = 26;
				Buzzer_Off();
      }
			 else if (dist < (DESIRED_DISTANCE_CM - 21))
      {
        basespeedl = 28;
        basespeedr = 28;
				Buzzer_Off();
      }
						 else if (dist < (DESIRED_DISTANCE_CM - 19))
      {
        basespeedl = 30;
        basespeedr = 30;
				Buzzer_Off();
      }
						 else if (dist < (DESIRED_DISTANCE_CM - 17))
      {
        basespeedl = 31;
        basespeedr = 31;
				Buzzer_Off();
      }
						 else if (dist < (DESIRED_DISTANCE_CM - 15))
      {
        basespeedl = 32;
        basespeedr = 32;
				Buzzer_Off();
      }
						 else if (dist < (DESIRED_DISTANCE_CM - 13))
      {
        basespeedl = 33;
        basespeedr = 33;
				Buzzer_Off();
      }
			 else if (dist < (DESIRED_DISTANCE_CM - 3))
      {
        basespeedl = 34;
        basespeedr = 34;
				Buzzer_Off();
      }
      else
      {
        basespeedl = 35;
        basespeedr = 35;
				Buzzer_Off();
      }
    }
    else
    {
      // If sensor not reading or out of range, keep default or slow down
      basespeedl = 40;
      basespeedr = 40;
			Buzzer_Off();
    }
  }
  return 0;
}
