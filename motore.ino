/**********************************************************************************************
 * Arduino Universal Motor Controller - Version 1.0.1
 * by teo Basili <basili.teo@gmail.com> https://github.com/teo666
 **********************************************************************************************/

#include "PID_ASYNC.h"
#include <EEPROM.h>
#include "pin_definition.h"

//#define TEST_MODE 

PID *motor_PID;

volatile uint16_t zcd_tick_log = 0;
volatile uint16_t tacho_tick_log = 0;
volatile uint16_t tick_after_zcd = 0;
volatile uint16_t tick_after_tacho = 0;
volatile uint8_t triac_state = 0;

//questa cosa del delay count mi serve per evitare di usare le funzioni di tempo di arduino
volatile uint8_t delay_count = 0;
volatile uint8_t delay_allow = 0;

uint16_t tick_per_phase = 0;

/*in alcune circostanze (dovute a problemi induttivi dal momento che ho potuto registrarli
solamente a motore connesso???) capita che il circuito zcd presenti dei disturbi
che fanno si' che alla sua uscita compaiano dei picchi di tensione che alterano il
corretto fasamento; questi picchi tuttavia hanno una durata molto breve rispetto al
tempo in cui l'onda quadra dello zcd si trova a livello logico alto, per cui utilizzo questa 
variabile per distinguere tali valori

- al cambiamento dello stato di uscita dello zdc azzero dei parametri, compreso zcd_error_correction
- controllo periodicamente (nell-interrupt del timer) il valore di ingresso dello zcd:
  se e' HIGH incremento zcd_error_correction
  altrimenti non faccio niente
- se dopo un po' di tempo tale valore raggiunge un valore limite (10) allora la mi sto trovando nel 
  punto un cui lo zcd e' effettivamente in fase e non si e' presentato il disturbo
*/
volatile uint8_t zcd_error_correction = 0;
volatile uint8_t found_correct_main_phase = 0;

volatile uint8_t tacho_error_correction = 0;
volatile uint8_t found_correct_tacho_phase = 0;


/*
variabili per la lettura dei valori analogici, contengono il valore delle letture
*/
volatile uint8_t HIGH_ANALOG_REG = 0;
volatile uint8_t LOW_ANALOG_REG = 0;

volatile uint8_t frequency_calc = 0;
volatile uint8_t frequency_calc_added = 0;

//il range dell' output va da 0 a 485, a 0 il motore gira a foo 485 val minimo
volatile uint16_t output = 65535; //max value
volatile uint16_t tacho_min_speed_value = 0;
volatile uint16_t tacho_max_speed_value = 0;

uint16_t output_min_speed_value = 0;
uint16_t output_max_speed_value = 0;

//variabili del pid

double Setpoint, Input, Output;
volatile uint8_t computeBarrier = 0;

#ifdef TEST_MODE

  volatile uint16_t delay_counter = 0;
  volatile uint16_t last_delay_counter = 0;

#endif

///////////// FUNZIONI DI CONFIGURAZIONE PID ///////////////////

#ifdef TEST_MODE

  Coefficient k_param;
  
  String serial_read;

  //funzione di ricerca dei parametri del pid
  
  CoefficientPtr search(){
    return &k_param;
  }

#else
  Coefficient k_param[9];

  void init_params(){
    k_param[0].Kp = 0.3;
    k_param[0].Ki = 0.0001;
    k_param[0].Kd = 0;
    
    k_param[1].Kp = 0.8;
    k_param[1].Ki = 0.0001;
    k_param[1].Kd = 0;
    
    k_param[2].Kp = 1;
    k_param[2].Ki = 0.0001;
    k_param[2].Kd = 0;
    
    k_param[3].Kp = 1.5;
    k_param[3].Ki = 0.0002;
    k_param[3].Kd = 0;
    
    k_param[4].Kp = 2;
    k_param[4].Ki = 0.0002;
    k_param[4].Kd = 0;
    
    k_param[5].Kp = 3;
    k_param[5].Ki = 0.0002;
    k_param[5].Kd = 0;
    
    k_param[6].Kp = 3;
    k_param[6].Ki = 0.001;
    k_param[6].Kd = 0;
    
    k_param[7].Kp = 3;
    k_param[7].Ki = 0.001;
    k_param[7].Kd = 0;
    
    k_param[8].Kp = 7;
    k_param[8].Ki = 0.002;
    k_param[8].Kd = 0;
  }

  CoefficientPtr search(){
   double _val = min(Setpoint,Input);
   uint8_t index = 0;
   
   if(_val >= 550){
    index = 0;
   } else if( _val >= 500 && _val < 550){
    index = 1;
   }else if( _val >= 450 && _val < 500){
    index = 2;
   }else if( _val >= 400 && _val < 450){
    index = 3;
   }else if( _val >= 350 && _val < 400){
    index = 4;
   }else if( _val >= 250 && _val < 350){
    index = 5;
   }else if( _val >= 200 && _val < 250){
    index = 6;
   }else if( _val >= 150 && _val < 200){
    index = 7;
   }else if( _val <= 150){
    index = 8;
   }
   //Serial.println(index);
   return &k_param[index];
  }

#endif

//////////////////// FUNZIONI DI UTILITA' ///////////////////////

uint8_t my_digital_read(uint8_t port_reg, uint8_t bit)
{
  if (port_reg & _BV(bit)) return 1;
  return 0;
}

void limit(volatile uint16_t* val, uint16_t min, uint16_t max) {
  if (*val > max) {
    *val = max;
  } else if (*val < min) {
    *val = min;
  }
}

uint16_t calculate_main_power_frequency(){
  frequency_calc = 1;
  uint32_t tick_sum = 0;

  while(frequency_calc != 0){
    if(frequency_calc_added){
      frequency_calc_added = 0;
      tick_sum += zcd_tick_log;
      frequency_calc++;
    }
  }

  return tick_sum / 255;
}

void check_programming_button(){
  //se il bottone e' pigiato chiededre di rilasciarlo
  if(!my_digital_read(PIND, PROG_PIN)){
    Serial.println("Release programming button, please");
    while(!my_digital_read(PIND, PROG_PIN)){
    }
  }
}

void button_hold_request(void (*loop)(), void (* on_exit)()){
  while(1){
    loop();
    if(delay_allow){
      on_exit();
      break;
    }
  }
}

void loop_read_fun(){
  output = map((HIGH_ANALOG_REG << 8) | LOW_ANALOG_REG, 1023, 0, 0, tick_per_phase);
}

//////////////////////////////////////////////////////////////////////


void save_low_speed_exit_fun(){
  tacho_min_speed_value = tacho_tick_log;
  output_min_speed_value = output;
  EEPROM.write(0,tacho_min_speed_value >> 8);
  EEPROM.write(1,tacho_min_speed_value);
  EEPROM.write(2,output_min_speed_value >> 8);
  EEPROM.write(3,output_min_speed_value);
  Serial.print(F("LOW speed tacho set to: "));
  Serial.print(tacho_min_speed_value);
  Serial.print(F(" LOW speed output set to: "));
  Serial.print(output_min_speed_value);
  Serial.println(F(", saving ..."));
}

void save_high_speed_exit_fun(){
  tacho_max_speed_value = tacho_tick_log;
  output_max_speed_value = output;
  if(tacho_max_speed_value > tacho_min_speed_value){
    Serial.println(F("The HIGH speed must be higher than LOW speed"));
    //se il bottone e' pigiato chiededre di rilasciarlo
    check_programming_button();
    Serial.println(F("try again"));
  } else {
    EEPROM.write(4,tacho_max_speed_value >> 8);
    EEPROM.write(5,tacho_max_speed_value);
    EEPROM.write(6,output_max_speed_value >> 8);
    EEPROM.write(7,output_max_speed_value);
    Serial.print(F("HIGH speed tacho set to: "));
    Serial.print(tacho_max_speed_value);
    Serial.print(F(" HIGH speed output set to: "));
    Serial.print(output_max_speed_value);
    Serial.println(F(", saving ..."));
  }
}

void setup() {

  INIT_TRIAC_LOG();
  INIT_ZCD_LOG();
  INIT_TACHO_LOG();

  INIT_TRIAC();
  INIT_PROG_BUTTON();
  INIT_ZCD();
  INIT_TACHO();

  //abilita le interruzioni sul pin 3 e 2 di arduino (bit meno significativo e' il 2) pin 3 = int1 pin 2 = int0
  EIMSK = 0b00000011;
  ////////////////32
  
  //interruzioni sul rising e falling edge 01, 11 solo rising
  EICRA = 0b00001111;
  
  /* configurazione timer 2
   * quindi il valore della variabile putput e' compreso fra 0 e 625
   */
  
  TCCR2A = 0b00000011;
  TCCR2B = 0b00000001;
  TIMSK2 = 0b00000001;

  #ifdef TEST_MODE
    //abilito le interruzioni del timer per la serial.available
    TIMSK0 = 0b00000001;
  #else
    //disabilito interruzioni timer 0
    TIMSK0 = 0b00000000;
  #endif

  //settaggio della lettura analogica
  //setto il voltaggio di riferimento del convertitore alla tensione di alimentazione
  //e il multiplexer per la lettura
  ADMUX = (1 << 6) | ( SPEED_READ & 0x07);
  
  //abilito l'ADC, le interruzioni hardware e il prescaler a 128 per letture accurate su 10bit
  ADCSRA = 0b10001111;
  //non setto le lettura automatiche utilizzando il registro ADCSRB perche le lancio a mano

  Serial.begin(9600);
  
  zcd_tick_log = 0;
  tick_after_zcd = 0;

  uint16_t prog_output = 0;

  Serial.println(F("Calculating frequency ..."));
  tick_per_phase = calculate_main_power_frequency();

  Serial.print(F("Number of tick per half period: "));
  Serial.println(tick_per_phase);
    
  if(!my_digital_read(PIND, PROG_PIN)){
    
    Serial.println(F("Programming mode"));
    
    //se il bottone e' pigiato chiededre di rilasciarlo
    check_programming_button();
    
    
    prog_output = (HIGH_ANALOG_REG << 8) | LOW_ANALOG_REG;
    if(prog_output){
      Serial.println(F("Turn speed potentiometer to lowest value ..."));
      while(prog_output){
        prog_output = (HIGH_ANALOG_REG << 8) | LOW_ANALOG_REG;
      }
    }
    
    //se il bottone e' pigiato chiededre di rilasciarlo
    check_programming_button();
    
    Serial.println(F("Turn potentiometer to increase speed and reach desire LOW speed, once done press and hold programming button ..."));

    button_hold_request(&loop_read_fun, &save_low_speed_exit_fun);

    //se il bottone e' pigiato chiededre di rilasciarlo
    check_programming_button();
    
    Serial.println(F("Turn potentiometer to increase speed and reach desire HIGH speed, once done press and hold programming button ..."));

    button_hold_request(&loop_read_fun, &save_high_speed_exit_fun);

    Serial.println(F("Configuration completed, enjoy!"));
    // impedisce la modalita' manuale quando si esce dal settaggio
    check_programming_button();
  }
  Serial.println(F("Operating mode"));
  Serial.println(F( "Reading paramters ..."));
  tacho_min_speed_value = ( EEPROM.read(0) << 8 ) | EEPROM.read(1);
  output_min_speed_value = ( EEPROM.read(2) << 8 ) | EEPROM.read(3);
  tacho_max_speed_value = ( EEPROM.read(4) << 8 ) | EEPROM.read(5);
  output_max_speed_value = ( EEPROM.read(6) << 8 ) | EEPROM.read(7);
  Serial.print(F("LOW speed tacho value: "));
  Serial.print(tacho_min_speed_value);
  Serial.print(F(", HIGH speed tacho value: "));
  Serial.println(tacho_max_speed_value);
  Serial.print(F("LOW speed output value: "));
  Serial.print(output_min_speed_value);
  Serial.print(F(", HIGH speed output value: "));
  Serial.println(output_max_speed_value);

  #ifndef TEST_MODE
    init_params();
  #endif
  
  output = output_min_speed_value;
  Output = output;

  motor_PID = new PID(&Input, &Output, &Setpoint, P_ON_M, &search, &tacho_tick_log, DIRECT);
  
  //TODO quando sopra un certo livello azzerare il valore di output per rendere il rallentamento migliore
  motor_PID->SetOutputLimits(0, output_min_speed_value + (abs(tick_per_phase - output_min_speed_value) >> 1) );
  
  motor_PID->SetMode(AUTOMATIC);

  #ifndef TEST_MODE
    //Serial.end();
  #endif
 
}

volatile uint8_t _tacho_trig = 0;

void loop() {

  //valore del potenziometro
  Setpoint = map( ((HIGH_ANALOG_REG << 8) | LOW_ANALOG_REG), 0, 1023, tacho_min_speed_value, tacho_max_speed_value);
  
  if(my_digital_read(PIND, PROG_PIN)){
    motor_PID->SetMode(AUTOMATIC);
    Input = tacho_tick_log;
    
    if(computeBarrier){
      motor_PID->Compute();
      computeBarrier = 0;
    }

    #ifdef TEST_MODE
    
      if(delay_counter - last_delay_counter > 60000){
        
        if(Serial.available()){
          serial_read = Serial.readString();
          String K = serial_read.substring(0,2);
          
          if (K.equals("kp")) {
            k_param.Kp = serial_read.substring(2).toFloat();
          } else if(K.equals("ki")){
            k_param.Ki = serial_read.substring(2).toFloat();
          } else if(K.equals("kd")){
            k_param.Kd = serial_read.substring(2).toFloat();
          }
          
          Serial.print("Kp: ");
          Serial.print(k_param.Kp);
          Serial.print(" Ki: ");
          Serial.print(k_param.Ki);
          Serial.print(" Kd:");
          Serial.println(k_param.Kd);
        }
        Serial.print(Setpoint);
        Serial.print(" ");
        Serial.print(Input);
        Serial.print(" ");
        Serial.println(Output);
        last_delay_counter = delay_counter;
      }
    #endif

    output = Output;
    
  } else {
    motor_PID->SetMode(MANUAL);
    output = map( ((HIGH_ANALOG_REG << 8) | LOW_ANALOG_REG), 0, 1023, tick_per_phase, 0);
  }
  
}

//handler dell' interrupt associato al pin 1 di arduino
//utilizzato per la sincronizzazione di fase, e' connesso 
//all'uscita del circuito di ZCD
ISR(INT0_vect) {
  zcd_error_correction = 0;
  found_correct_main_phase = 0;
  //lancia una lettura del valore analogico
  ANALOG_READ();
}

//handler dell' interrupt associato al pin 2 di arduino
//utilizzato per il rilevamento del tacogeneratore
ISR(INT1_vect) {
  tacho_error_correction = 0;
  found_correct_tacho_phase = 0;
}

//interrupt associato al timer, all'interno di una semionda della rete
//effettua dei controlli periodici, quali l'incremento del contatore
//per poter far si che il bottone do programmazione funzioni c'e' bisogno dello zcd
ISR(TIMER2_OVF_vect) {
  //
  // zcd
  //
  if (my_digital_read(PIND,ZCD_INPUT)) {
    zcd_error_correction++;
  }
  
  if (zcd_error_correction > 10 && !found_correct_main_phase) {
    zcd_tick_log = tick_after_zcd;
    frequency_calc_added = 1;
    if(!my_digital_read(PIND, PROG_PIN) && !delay_allow){
      delay_count++;
    } else {
      delay_count = 1;
      delay_allow = 0;
    }
    if(!delay_count){
      delay_allow = 1;
    }
    tick_after_zcd = 0;
    //spegni il triac
    TURN_OFF_TRIAC();
    triac_state = 0;
    found_correct_main_phase = 1;
  }
  //
  // tacho
  //
  if (my_digital_read(PIND,TACHO_INPUT)) {
    tacho_error_correction++;
  }
  
  if (tacho_error_correction > 15 && !found_correct_tacho_phase) {
    tacho_tick_log = tick_after_tacho;
    computeBarrier = 1;
    tick_after_tacho = 0;
    found_correct_tacho_phase = 1;
    _tacho_trig = !_tacho_trig;
    if(_tacho_trig){
      TURN_ON_TACHO_LOG();
    } else {
      TURN_OFF_TACHO_LOG();
    }
  }
  //////////////////////////////////////
  tick_after_zcd++;
  tick_after_tacho++;

  #ifdef TEST_MODE
    delay_counter++;
  #endif

  if (tick_after_zcd >= output && !triac_state) {
    //accendi il triac
    TURN_ON_TRIAC();
    triac_state = 1;
  }
}

//al completamento della lettura del valore del potenziometro salva i valori
ISR(ADC_vect){
  LOW_ANALOG_REG = ADCL;
  HIGH_ANALOG_REG = ADCH;
}

