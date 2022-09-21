volatile int32_t period_hi = 0;
volatile int32_t period_lo = 0;
volatile uint8_t tmr_overf = 0;
volatile boolean end_capture = 0;
float freq_cntr;
float rpm; //INPUT
volatile boolean end_capture2 = 0; //FLAG2
const uint16_t T3_init =0; 
const uint16_t T3_comp = 20000;  //100hz
int count = 0;
int e[2]={0,0}; //ERROR
int u[2]={0,0}; //OUTPUT
float sp = 6000;  //SETPOINT
float kp=0.01404;
float Ti=0.001; //Ti = kp/ki  
float c1,c2;
float T = 0.01; //SAMPLE TIME
volatile int contador; 

void setup()
{
  Serial.begin(115200);
  pinMode(11 , OUTPUT); //PWM OUTPUT
  
  init_T1(); //SET TIMER 1
  delay(10000); //FOR ESC INITIALIZATION 
  init_T5(); //SET TIMER 5
  init_T3(); //SET TIMER 3
  
  c1 = kp; 
  c2 = -kp + (kp*T/Ti);  
}

void loop()
{
  //GET RPM
  if (end_capture){      
    freq_cntr = 16000000.00 / float(period_hi + period_lo);
    rpm = freq_cntr * 60;
    contador = 0;
    Serial.print("RPM : ");
    Serial.println(rpm, 2);      
    end_capture = false;
  }

  //SATURATION OUTPUT/ERROR 
  if (end_capture2){ 
    if(u[0]>=8000){
      u[0]=8000;
    }
    if(u[0]<=0){
      u[0]=0;
    }
    if(e[0]>=10){
      e[0]=10;
    }
   if(e[0]<=-10){
      e[0]=-10;
   }
   
  OCR1A = u[0] + 8000; //PWM OUTPUT 

  //UPDATE THE VARIABLES
  e[1]=e[0]; 
  u[1]=u[0];
 
  end_capture2 = false;
   
  } 
  
  //IF NO SIGNAL IN THE SENSOR FOR 1s
  if(contador>=100){
    rpm=0;
  }
}   

  
void init_T1(void){  
  noInterrupts();
  TCCR1A = 0;    
  TCCR1B = 0;
  TCNT1= 0; 
  // 1 PRESCALER
  TCCR1B &= ~(1 << CS12);  //0  
  TCCR1B &= ~(1 << CS11);  //0
  TCCR1B |=  (1 << CS10);  //1
  TCCR1B |=  (1 << WGM13); //FAST PWM 
  TCCR1B |=  (1 << WGM13); 
  TCCR1A |=  (1 << WGM11); 
  TCCR1A &= ~(1 << WGM10); 
  TCCR1A |=  (1 << COM1A1);
  ICR1 = 31950; //250HZ PWM
  OCR1A = 8000; //INITIALIZE ESC WITH 1ms PWM
  interrupts();
}

void init_T3(void){
  noInterrupts();
  TCCR3A = 0;             
  TCCR3B = 0;
  //8 PRESCALER
  TCCR3B &= ~(1 << CS32); //0
  TCCR3B |=  (1 << CS31); //1
  TCCR3B &= ~(1 << CS30); //0
  TCNT3= T3_init; 
  OCR3B = T3_comp; //SET 100HZ
  TIMSK3 = (1<<OCIE3B );
  interrupts();
}

void init_T5(void){  
  noInterrupts();
  TCCR5A = 0;
  TCCR5B = 0;
  TCCR5B |= (1<< CS50); // set prescaler to 16 MHz
  TCCR5B |= (1<<ICNC5); // input noise canceler on
  TCCR5B |= (1<<ICES5); // input capture edge select (lo-hi) 
  TIMSK5 |= (1<<TOIE5); // Overflow Interrupt Enable 
  TIMSK5 |= (1<<ICIE5); // InCaptureInterrupt Enable 
  interrupts();  
}

ISR(TIMER3_COMPB_vect){  
  TCNT3 = T3_init; 
  e[0]= sp-rpm;
  u[0] = u[1] + c1*e[0] + c2*e[1];  
  contador = contador + 1;   
  end_capture2 = true; 
}

ISR(TIMER5_OVF_vect){
  tmr_overf++;  
}

ISR(TIMER5_CAPT_vect) {
  static uint16_t last_v = 0;
  uint16_t curr_v = ICR5;
  uint32_t accuml = 0;

  accuml  = curr_v + tmr_overf *65536UL;  
  accuml -= last_v;    
  last_v  = curr_v;
  tmr_overf = 0;        

  if(TCCR5B &  (1<<ICES5)) {  
    TCCR5B &= ~(1<<ICES5);    
    period_lo = accuml;
    end_capture = true;       
    }
  else {  
    TCCR5B |= (1<<ICES5);    
    period_hi = accuml;
    }
}
