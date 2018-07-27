#include <PID_v1.h>
#include <EEPROM.h>


//registro per l'IO B (PORTD)
//pin per controllare il triac (13 su Arduino UNO)
#define TRIAC_OUTPUT 7

//pin per la programmazione portD
#define PROG_PIN 6

//pin per l'input dello zcd (PORTD) non puo' cambiare a causa dell'interruzione abilitata
#define ZCD_INPUT 2
//pin per l'input del tacho (PORTD) non puo' cambiare a causa dell'interruzione abilitata
#define TACHO_INPUT 3

// pin per la lettura del valore del potenziometro per regolare la velocita' (PORTA)
#define SPEED_READ 0

//ZERO CROSSING DETECTOR LOG per il debug su oscilloscopio
#define ZCD_LOG 4

//TRIAC LOG per il debug del triac, stesso comportamento di TRIAC_OUTPUT
#define TRIAC_LOG 5

//TACHO LOG per il debug del tacho
#define TACHO_LOG 3

//identifica il minimo intervallo di tempo fra le creste di salita del
//segnale di tacho
//#define READ_UPPER_SPEED_LIMIT 70
//identifica il massimo intervallo di tempo fra le creste di salita del
//segnale di tacho,
//#define READ_LOWER_SPEED_LIMIT 260

//minima velocita' desiderata, valori alti, abbassano la velocita
//se 0 il motore riceve sempre il massimo della potenza
//#define MIN_SPEED_THRESHOLD 470

//definizione delle macro di accensione e espegnimento dei diversi pin
#define TURN_ON_TRIAC() sbi(PORTD,TRIAC_OUTPUT)
#define TURN_OFF_TRIAC() cbi(PORTD,TRIAC_OUTPUT)
#define TURN_ON_ZDC_LOG() sbi(PORTB,ZCD_LOG)
#define TURN_OFF_ZDC_LOG() cbi(PORTB,ZCD_LOG)
#define TURN_ON_TRIAC_LOG() sbi(PORTB,TRIAC_LOG)
#define TURN_OFF_TRIAC_LOG() cbi(PORTB,TRIAC_LOG)
#define TURN_ON_TACHO_LOG() sbi(PORTB,TACHO_LOG)
#define TURN_OFF_TACHO_LOG() cbi(PORTB,TACHO_LOG)
#define ANALOG_READ() sbi(ADCSRA,6)

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

volatile uint16_t zcd_tick_log = 0;
volatile uint16_t tacho_tick_log = 0;
volatile uint16_t tick_after_zcd = 0;
volatile uint16_t tick_after_tacho = 0;
volatile uint8_t triac_state = 0;
volatile uint8_t delay_count = 0;

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
double Kp=1.2, Ki=20, Kd=0.1;

String serial_read;

PID *motor_PID;
//FUNZIONI DI UTILITA'

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

void setup() {
  //setta i pin definiti sulla porta b come uscite
  sbi(DDRB,ZCD_LOG);
  sbi(DDRB,TRIAC_LOG);
  sbi(DDRB,TACHO_LOG);
  
  //setta i pin definiti sulla porta d come uscite
  sbi(DDRD,TRIAC_OUTPUT);

  //setta i pin di ingresso del tacho e dello zcd come ingressi
  //TODO : abilitare il pulldown???
  cbi(DDRD,TACHO_INPUT);
  cbi(DDRD,ZCD_INPUT);
  sbi(PORTD,TACHO_INPUT);
  //setta il pin di programmazione come ingresso e abilita la resistenza di pullup
  //permette di attaccare un push button senza ulteriori componenti
  //(ad essere pignoli servirebbe un condensatore in parallelo,
  //ma non lavoriamo in interrupt mode, quindi va bene ugualmente)
  cbi(DDRD,PROG_PIN);
  sbi(PORTD,PROG_PIN);

  //setta tutte le uscite LOW
  cbi(PORTD,TRIAC_OUTPUT);
  cbi(PORTB,ZCD_LOG);
  cbi(PORTB,TRIAC_LOG);
  cbi(PORTB,TACHO_LOG);

  //abilita le interruzioni sul pin 3 e 2 di arduino (bit meno significativo e' il 3) pin 3 = int1 pin 2 = int0
  EIMSK = 0b00000011; 
  
  //interruzioni sul rising e falling edge 01, 11 solo rising
  EICRA = 0b00001111;
  
  /* configurazione timer 2
   * per comee [ configurato il timer in una semionda a 50HZ esegue 625 interruzioni
   * quindi il valore della variabile putput e' compreso fra 0 e 625
   */
  
  TCCR2A = 0b00000011;
  TCCR2B = 0b00000001;
  TIMSK2 = 0b00000001;

  //configurazione timer 1 16 bit
  //TCCR1A = 0b00000001;
  //TCCR1B = 0b00001001;

  //TIMSK1 = 0b00100111;

  //configurazione timer 0
  //TCCR0A = 0b00000011;
  //TCCR0B = 0b00000001;

   
  //TIMSK0 = 0b00000000;

  //settaggio della lettura analogica
  //setto il voltaggio di riferimento del convertitore alla tensione di alimentazione
  //e il multiplexer per la lettura
  ADMUX = (1 << 6) | ( SPEED_READ & 0x07);
  //abilito l'ADC, le interruzioni hardware e il prescaler a 128 per letture accurate su 10bit
  ADCSRA = 0b10001111;
  //non setto le lettura automatiche utilizzando il registro ADCSRB perhce le lancio a mano

  Serial.begin(9600);
  
  zcd_tick_log = 0;
  tick_after_zcd = 0;

  uint16_t prog_output = 0;

  Serial.println(F("Calculating frequency ..."));
  tick_per_phase = calculate_main_power_frequency();

  if(!my_digital_read(PIND, PROG_PIN)){
    
    Serial.println(F("Programming mode"));
    
    //se il bottone e' pigiato chiededre di rilasciarlo
    check_programming_button();
    
    Serial.print(F("Number of tick per semi wave: "));
    Serial.println(tick_per_phase);
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

    delay_count = 1;
    while(1){
      
      prog_output = (HIGH_ANALOG_REG << 8) | LOW_ANALOG_REG;
      output = map(prog_output, 1023, 0, 0, tick_per_phase);
      //Serial.println(tacho_tick_log);
      if(!delay_count){
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
        break;
      }
    }
    
    //se il bottone e' pigiato chiededre di rilasciarlo
    check_programming_button();
    
    Serial.println(F("Turn potentiometer to increase speed and reach desire HIGH speed, once done press and hold programming button ..."));

    delay_count = 1;
    while(1){
      prog_output = (HIGH_ANALOG_REG << 8) | LOW_ANALOG_REG;
      output = map(prog_output, 1023, 0, 0, tick_per_phase);
      //Serial.println(tacho_tick_log);
      if(!delay_count){
        tacho_max_speed_value = tacho_tick_log;
        output_max_speed_value = output;
        if(tacho_max_speed_value > tacho_min_speed_value){
          Serial.println(F("The HIGH speed must be higher than LOW speed"));
          delay_count = 1;
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
          break;
        }
      }
    }

    Serial.println(F("Configuration completed, enjoy!"));

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
  output = output_min_speed_value;
  Output = output;
  motor_PID = new PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
  motor_PID->SetOutputLimits(0,output_min_speed_value);
  motor_PID->SetMode(AUTOMATIC);
  motor_PID->SetSampleTime(10);
  //Serial.end();
}

volatile uint8_t _tacho_trig = 0;

void loop() {
  //valore del potenziometro
  Setpoint = map( ((HIGH_ANALOG_REG << 8) | LOW_ANALOG_REG), 0, 1023, tacho_min_speed_value, tacho_max_speed_value);
  
  if(my_digital_read(PIND, PROG_PIN)){
    Input = tacho_tick_log;
    motor_PID->Compute();
    output = Output;
    if(Serial.available() > 0){
      serial_read = Serial.readString();
      String K = serial_read.substring(0,2);
      
      if (K.equals("kp")) {
        Kp = serial_read.substring(2).toFloat();
      } else if(K.equals("ki")){
        Ki = serial_read.substring(2).toFloat();
      } else if(K.equals("kd")){
        Kd = serial_read.substring(2).toFloat();
      }
      motor_PID->SetTunings(Kp,Ki,Kd);
      Serial.print("Kp: ");
      Serial.print(Kp);
      Serial.print(" Ki: ");
      Serial.print(Ki);
      Serial.print(" Kd:");
      Serial.println(Kd);
    }
    /*Serial.print(Setpoint);
    Serial.print(" ");
    Serial.print(Input);
    Serial.print(" ");
    Serial.println(Output);*/
    
  } else {
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
    if(!my_digital_read(PIND, PROG_PIN)){
      delay_count++;
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
    tick_after_tacho = 0;
    found_correct_tacho_phase = 1;
    _tacho_trig = !_tacho_trig;
  }
  //////////////////////////////////////
  tick_after_zcd++;
  tick_after_tacho++;

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

