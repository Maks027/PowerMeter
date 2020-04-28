#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>
#include <stdio.h>

LiquidCrystal_PCF8574 lcd(0x27);

#define U_SENSE       0
#define I_SENSE       1
#define POWER_CALC    2
#define AVG           3

#define RELAY_DDR     DDRB
#define RELAY_PORT    PORTB
#define RELAY_PIN     PORTB4

#define RELAY_ON()    (RELAY_PORT |= (1 << RELAY_PIN))
#define RELAY_OFF()   (RELAY_PORT &= ~(1 << RELAY_PIN))


#define B1_PRESSED    (PINB & (1 << PB2))
#define B1_RELEASED   (!(PINB & (1 << PB2)))

#define B2_PRESSED    (PINB & (1 << PB3))
#define B2_RELEASED   !(PINB & (1 << PB3))

#define CAL_PRESSED    (!(PINB & (1 << PB0)))
#define CAL_RELEASED   (PINB & (1 << PB0))

#define MB_PRESSED    (!(PINB & (1 << PB1)))
#define MB_RELEASED   (PINB & (1 << PB1))

#define ZERO_VAL      512

#define MENU_NR       2

#define I_TRESHOLD    200
#define U_TRESHOLD    240
 
//#define kU            0.823741
#define kU            0.912
#define kI            0.049
#define kP            1.11

#define PRIM_SMP      64
#define SEC_SMP       50   
#define SMP_NR        (float)(PRIM_SMP * SEC_SMP)  
            
volatile uint16_t smp_cnt = 0;
bool data_ready = false;
uint8_t on_flag = 0;
uint8_t menu_cnt = 0;
long lastMillis = 0;
long deb_delay  = 100;
float cor_val = 0;

byte customChar[8] = {
  0b01010,
  0b01010,
  0b01010,
  0b01010,
  0b01010,
  0b01010,
  0b01010,
  0b01010
};

char f_buff[10];
char s_buff[16];

volatile long I_adc = 0;

long PrimSumIadc;
long PrimSumSqIadc;

long SecSumIadc;
long SecSumSqIadc;

long SecSumIadcArr[SEC_SMP];
long SecSumSqIadcArr[SEC_SMP];

long CopyPrimSumIadc;
long CopyPrimSumSqIadc;

int I_max;
int I_min;
float I_max_sc = 0;
float I_min_sc = 0;
int Ipp;
float Imed;
float Irms;

volatile long U_adc = 0;
long PrimSumUadc;
long PrimSumSqUadc;

long SecSumUadc;
long SecSumSqUadc;

long SecSumUadcArr[SEC_SMP];
long SecSumSqUadcArr[SEC_SMP];

int U_max;
int U_min;
float U_max_sc = 0;
float U_min_sc = 0;
int Upp;
float Umed;
float Urms;

long CopyPrimSumUadc;
long CopyPrimSumSqUadc;
uint16_t arr_cnt = 0;
uint16_t v_cnt = 0;

float P;
float S;
float Q;
float PF;

int P_max;
int P_min;
float P_max_sc = 0;
float P_min_sc = 0;

volatile long PrimSumP;
long CopyPrimSumP;
long SecSumP;
long SecSumP_Arr[SEC_SMP];

void ADC_init(void)                                                                   //Initializare ADC
{
  ADMUX |= (1 << REFS0);                                                              //Tensiunea de referinta 5V cu condensator extern pe AREF
  ADCSRA |= (1 << ADEN) | (1 << ADATE) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1);   //ADC enable, auto trigger, activare intrerupere, divizor 64

  ADCSRA |= (1 << ADSC);                                                              //Startul primei conversii
  ADCSRB |= (1 << ACME);                                                              //Multiplexarea canalelor, regim free running
}


ISR(ADC_vect)                                                                         //Intrerupere la finisarea conversiei ADC
{
  long adc_r = ADC;
  long adc_val = (long)adc_r - ZERO_VAL;
  
  static uint8_t mx_cont = 0;

//    mx_cont++;                           
//  mx_cont &= 0x03;
  ADMUX = mx_cont | 0x40;
  if(++mx_cont > 2)
    mx_cont  = 0;
    
    
    
  switch(mx_cont)
  {
    case 1: {
      I_adc = -adc_val;
      PrimSumIadc += I_adc;
      PrimSumSqIadc += (I_adc * I_adc);
      if(I_adc > I_max)                                                               //Determinarea valorii maxime si minime a curentului
        I_max = I_adc;
      if(I_adc < I_min)
        I_min = I_adc;
      break;
    }
    case 0: {
      
      U_adc = adc_val;

      PrimSumUadc += U_adc;
      PrimSumSqUadc += (U_adc * U_adc);
      if(U_adc > U_max)                                                              //Determinarea valorii maxime si minime a tensiunii
        U_max = U_adc;
      if(U_adc < U_min)
        U_min = U_adc;
        
      break;
    }
    case 2: {
      long TadcVal = long(U_adc) * long(I_adc);
      PrimSumP += TadcVal; 
      smp_cnt++;
      v_cnt++;
      if(smp_cnt >= PRIM_SMP)
      {
        CopyPrimSumSqUadc = PrimSumSqUadc;
        PrimSumSqUadc = 0;

        CopyPrimSumSqIadc = PrimSumSqIadc;
        PrimSumSqIadc = 0;

        CopyPrimSumP = PrimSumP;
        PrimSumP = 0;
        
        smp_cnt = 0;
        static uint8_t bFlag = 0, mode = 1;
        static uint32_t last_debounce_time = 0, debounce_delay = 200; 
  
        if (MB_PRESSED && (bFlag == 0))
        {
           bFlag = 1;
           last_debounce_time = millis();
        }
  
        if ((millis() - last_debounce_time) > debounce_delay)
        {
           if (MB_RELEASED && (bFlag == 1))
           {
              bFlag = 0;
              if(menu_cnt < 5)
                 menu_cnt++;
              else
                 menu_cnt = 0;
           }
        }

        if(CAL_PRESSED)                       
        {
          if((menu_cnt == 3) || (menu_cnt == 4))
          {
            I_max = 0;
            I_min = 0;
            U_max = 0;
            U_min = 0;
          }
          else
            cor_val = Irms;
        }
        
        data_ready = true;
      }
      break;
    }
  }
}

void setup() {
  lcd.begin(16,2);  //Initializare LCD 16x2
  ADC_init();       //Initializare ADC
  sei();            //Permite intreruperi globale    
  RELAY_DDR |= (1 << RELAY_PIN);
  DDRB &= ~(1 << PB0);
  DDRB &= ~(1 << PB1);
  PORTB |= (1 << PB0) | (1 << PB1);  
  lcd.createChar(0, customChar);   
  lcd.setBacklight(255);
  pinMode(5, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  
  if(data_ready == true)
  {
    ////////////////////////////////////////////
    SecSumSqUadc -= SecSumSqUadcArr[arr_cnt];
    SecSumSqUadc += CopyPrimSumSqUadc;
    SecSumSqUadcArr[arr_cnt] = CopyPrimSumSqUadc;
    ////////////////////////////////////////////
    SecSumSqIadc -= SecSumSqIadcArr[arr_cnt];
    SecSumSqIadc += CopyPrimSumSqIadc;
    SecSumSqIadcArr[arr_cnt] = CopyPrimSumSqIadc;
    /////////////////////////////////////////////
    SecSumP = SecSumP_Arr[arr_cnt];
    SecSumP += CopyPrimSumP;
    SecSumP_Arr[arr_cnt] = CopyPrimSumP;
    
    arr_cnt++;
    if(arr_cnt >= SEC_SMP)
      arr_cnt = 0;
    data_ready = false;
  }

  if(v_cnt > 1000)
  {
    Urms = kU *sqrt((float)SecSumSqUadc / SMP_NR);
    
    Irms =(kI * sqrt((float)SecSumSqIadc / SMP_NR)) - cor_val ;
    if(Irms <= 0)
      Irms = 0;
      
    S = Irms * Urms;
    if(S <= 0)
      S = 0;
    
    P =(float)SecSumP * kP / SMP_NR;
    if(P <= 0)
      P = 0;
    Q = sqrt((S * S) - (P * P));         
    if(Q <= 0)
      Q = 0;
      
    PF = P / S;
    if(PF <= 0)
      PF = 0;
    if(PF > 1)
      PF = 1;

    I_max_sc = I_max * kI;
    I_min_sc = I_min * kI;

    U_max_sc = U_max * kU;
    U_min_sc = U_min * kU;
    switch(menu_cnt)
    {
      case 0:{
        lcd.setCursor(0, 0);
        dtostrf(Urms, 4, 1, f_buff);
        sprintf(s_buff, "   U=%5sV    ",f_buff);
        lcd.print(s_buff);

        lcd.setCursor(0, 1);
        dtostrf(Irms, 4, 2, f_buff);
        sprintf(s_buff, "   I=%5sA    ",f_buff);
        lcd.print(s_buff);
        
        break;
      }
      case 1:{
        lcd.setCursor(0, 0);
        dtostrf(P, 4, 1, f_buff);
        sprintf(s_buff, "   P=%6sW   ",f_buff);
        lcd.print(s_buff);

        lcd.setCursor(0, 1);
        dtostrf(S, 4, 1, f_buff);
        sprintf(s_buff, "   S=%6sVA  ",f_buff);
        lcd.print(s_buff);
        break;
      }
      case 2:{
        lcd.setCursor(0, 0);
        dtostrf(Q, 4, 1, f_buff);
        sprintf(s_buff, "  Q=%6sVAR  ",f_buff);
        lcd.print(s_buff);

        lcd.setCursor(0, 1);
        dtostrf(PF, 3, 2, f_buff);
        sprintf(s_buff, "    PF=%3s  ",f_buff);
        lcd.print(s_buff);
        break;
      }
      case 3:{
        lcd.setCursor(0, 0);
        dtostrf(U_max_sc, 4, 1, f_buff);
        sprintf(s_buff, "  Umax=%5sV  ",f_buff);
        lcd.print(s_buff);

        lcd.setCursor(0, 1);
        dtostrf(U_min_sc, 4, 1, f_buff);
        sprintf(s_buff, "  Umin=%5sV  ",f_buff);
        lcd.print(s_buff);
        
        break;
      }
      case 4:{
        lcd.setCursor(0, 0);
        dtostrf(I_max_sc, 4, 2, f_buff);
        sprintf(s_buff, "  Imax=%5sA  ",f_buff);
        lcd.print(s_buff);

        lcd.setCursor(0, 1);
        dtostrf(I_min_sc, 4, 2, f_buff);
        sprintf(s_buff, "  Imin=%5sA  ",f_buff);
        lcd.print(s_buff);
        break;
      }
    }
    
    Serial.print("U=");
    Serial.println(Urms,1);

    Serial.print("I=");
    Serial.println(Irms,2);

    Serial.print("S=");
    Serial.println(S,1);

    Serial.print("P=");
    Serial.println(P,1);

    Serial.print("Q=");
    Serial.println(Q,1);

    Serial.print("PF=");
    Serial.println(PF,1);

    Serial.println();
    
    v_cnt = 0;
  }

  if(B1_PRESSED)                               //Prelucrarea butoanelor de pornire/oprire a alimentarii
    on_flag = 1;
  if(B2_PRESSED)
    on_flag = 0;
  if(Irms < I_TRESHOLD)
  {
    if(Urms < U_TRESHOLD)
    {
      if(on_flag)
        RELAY_ON();
      else
        RELAY_OFF();
    }
    else
      RELAY_OFF();
  }
  else 
    RELAY_OFF();
}
