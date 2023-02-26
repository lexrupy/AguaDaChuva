/* 
 *  CONTROLADORA DE SISTERNA E RESERVATÓRIO (CAIXA DÁGUA)
 *  
 *  A SISTERNA POSSUI 2 OU 3 SENSORES DE NÍVEL SENDO
 *  CISTER_UPPER_FLOAT
 *  CISTER_MIDDLE_FLOAT
 *  CISTER_LOWER_FLOAT
 *  
 *  QUANDO A AGUA ALCANCA ALGUM SENSOR, ELE FICA NA POSICAO LIGADO
 *  
 *  A CISTERNA ESTÁ VAZIA QUANDO O SENSOR CISTER_LOWER_FLOAT ESTIVER DESLIGADO
 *  A CISTERNA ESTÁ CHEIA QUANDO O SENSOR CISTER_UPPER_FLOAT ESTIVER LIGADO
 *  
 *   *  
 *  O RESERVATORIO POSSUI APENAS 1 BOIA 
 *  RESERV_FLOAT
 *  
 *  QUANDO A BOIA DO RESERVATORIO ESTIVER LIGADA O RESERVATORIO ESTÁ VAZIO, QUANDO ESTIVER DESLIGADO
 *  EXISTE QUALQUER NÍVEL DIFERENTE DE VAZIO NO RESERVATÓRIO
 *  
*/

//#define SERIAL_DEBUG true

#include <EEPROM.h>
#include <LiquidCrystal.h>
#define CISTER_FUL_FLOAT A1
#define CISTER_MID_FLOAT A2
#define CISTER_LOW_FLOAT A3

#define RESERV_FLOAT A0      /* SE LIGADO, RESERVATORIO VAZIO */

#define CONCES_FLOW_OUT 2   /* FLUXO DE ÁGUA DA CONSESSIONARIA*/
#define CISTER_FLOW_OUT 3   /* FLUXO DE ÁGUA DA CISTERNA */

// #define RESERV_STATUS_LED 9
// #define CISTER_STATUS_LED 10

#define ERROR_RESET_PIN 10

#define LCD_BL 13

#define CISTER_ERROR_ADDR 0
#define CONCES_ERROR_ADDR 1

#define HEART_BEAT A5


/* 
 *  DEFINIR TEMPO MAXIMO QUE O SISTEMA PERMANECE ABERTO ANTES DE DESLIGAMENTO DE SEGURANCA
 *  MEDIR TEMPO QUE LEVA PARA ENCHER O RESERVATORIO COM O MOTOR DA CISTERNA, E DEIXAR MARGEM 
 *  
 *  MEDIR TEMPO DE LEVA PARA ENCHER O RESERVATORIO COM A AGUA DA CONCESSIONARIA E DEIXAR MARGEM 
 *  CASO ATINGIR O TEMPO, PISCA O LED CORRESPONDENTE DE FLUXO 
 *  1 SEGUNDO = 1000
 *  1 MINUTO  = 60000 
 
*/

#define SECOND 1000
#define MINUTE 60000

#define MAX_TIME_CISTER_FLOW 100 * MINUTE // DEFAULT 5 MINUTOS 
#define MAX_TIME_CONCES_FLOW 100 * MINUTE // DEFAULT 5 MINUTOS
#define STARTUP_TIME 3 * SECOND

#define LCD_DT1 4
#define LCD_DT2 5
#define LCD_DT3 6
#define LCD_DT4 7

#define LCD_RS 8
#define LCD_EN 9

LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_DT1, LCD_DT2, LCD_DT3, LCD_DT4);

byte lvlEmpty[8] = {
	0b01110,
	0b10001,
	0b10001,
	0b10001,
	0b10001,
	0b10001,
	0b10001,
	0b11111
};

byte lvlLow[8] = {
	0b01110,
	0b10001,
	0b10001,
	0b10001,
	0b10001,
	0b10001,
	0b11111,
	0b11111
};

byte lvlMid[8] = {
	0b01110,
	0b10001,
	0b10001,
	0b10001,
	0b11111,
	0b11111,
	0b11111,
	0b11111
};


byte lvlFull[8] = {
	0b01110,
	0b11111,
	0b11111,
	0b11111,
	0b11111,
	0b11111,
	0b11111,
	0b11111
};

unsigned long HEART_BEAT_LAST = 0;
int HEART_BEAT_STATE = LOW;
int CISTER_STATUS_LED_STATE = LOW;
int RESERV_STATUS_LED_STATE = LOW;


int CISTER_MID = 0;
int CISTER_FUL = 0;
int CISTER_LOW = 0;


int CISTER_EMPTY_READ = 0;
int CISTER_EMPTY = 0;
unsigned long LAST_CISTER_EMPTY_DEBOUNCING_TIME = 0;


int RESERV_EMPTY_READ = 0;
int RESERV_EMPTY = 0;
unsigned long LAST_RESERV_EMPTY_DEBOUNCING_TIME = 0;


unsigned long CISTER_FLOW_STATUS_LED_LAST = 0;
int CISTER_FLOW_STATUS = 0;
int CISTER_FLOW_ERROR = 0;

unsigned long CONCES_FLOW_STATUS_LED_LAST = 0;
int CONCES_FLOW_STATUS = 0;
int CONCES_FLOW_ERROR = 0;

int ERROR_STATUS = 0;

unsigned long TIME_CISTER_FLOW = 0;
unsigned long TIME_CONCES_FLOW = 0;

unsigned long TOTAL_TIME_CISTER_FLOW = 0;
unsigned long TOTAL_TIME_CONCES_FLOW = 0;

unsigned long LOOP_TIME = 0;
unsigned long DEBOUNCING_DELAY = 1000;


void setup() {
  pinMode(CISTER_FUL_FLOAT, INPUT);
  pinMode(CISTER_MID_FLOAT, INPUT);
  pinMode(CISTER_LOW_FLOAT, INPUT);
  pinMode(ERROR_RESET_PIN, INPUT_PULLUP);
  pinMode(RESERV_FLOAT, INPUT);

  pinMode(LCD_BL, OUTPUT);
  digitalWrite(LCD_BL, HIGH);

  digitalWrite(CISTER_FUL_FLOAT, LOW);
  digitalWrite(CISTER_MID_FLOAT, LOW);
  digitalWrite(CISTER_LOW_FLOAT, LOW);
  digitalWrite(RESERV_FLOAT, LOW);
  
  // pinMode(RESERV_STATUS_LED, OUTPUT);
  // pinMode(CISTER_STATUS_LED, OUTPUT);
  
  pinMode(CISTER_FLOW_OUT, OUTPUT);
  pinMode(CONCES_FLOW_OUT, OUTPUT);

  digitalWrite(CISTER_FLOW_OUT, HIGH);
  digitalWrite(CONCES_FLOW_OUT, HIGH);


  pinMode(A5, OUTPUT);

  digitalWrite(A5, LOW);

  CONCES_FLOW_ERROR = EEPROM.read(CONCES_ERROR_ADDR);
  CISTER_FLOW_ERROR = EEPROM.read(CISTER_ERROR_ADDR);

  //------------lcd-----------

  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("INICIALIZANDO...");
  delay(2000);
  lcd.clear();



  lcd.createChar(0, lvlEmpty);
  lcd.createChar(1, lvlLow);
  lcd.createChar(2, lvlMid);
  lcd.createChar(3, lvlFull);
  lcd.setCursor(0, 0);
  lcd.print("Cx:    Cist: ");
  lcd.setCursor(0, 1);
  lcd.print("Sem Fluxo  --:--");  

  #ifdef SERIAL_DEBUG  
  Serial.begin(9600);
  #endif


}

void loop() {
  LOOP_TIME = millis();

  readCisternStatus();
  readReservatoryStatus();
  doSecurityCheck();
  updateLCD();

  if (RESERV_EMPTY == 1) { /* CASO O RESERVATORIO ACUSE VAZIO */
    if (CISTER_EMPTY == 0) { /* SE HOUVER AGUA NA SISTERNA ACIONA O MOTOR DA SISTERNA */
      if (CISTER_FLOW_STATUS == 0 & CISTER_FLOW_ERROR == 0) {
        /* INICIAR AQUI CONTEGEM DE TEMPO DE SERGURANCA... */
        TIME_CISTER_FLOW = LOOP_TIME;
        TOTAL_TIME_CISTER_FLOW = 0;
        solenoidOff();
        motorOn();
      }
    } else { /* SE NAO HOUVER AGUA NA SISTERNA ACIONA O MOTOR DA SISTERNA */
      if (CONCES_FLOW_STATUS == 0 & CONCES_FLOW_ERROR == 0) {
        TIME_CONCES_FLOW = LOOP_TIME;
        TOTAL_TIME_CONCES_FLOW = 0;
        motorOff();
        solenoidOn();
      }
    }
  } else {
    motorOff();
    solenoidOff();
  }
  //controlCisterLED();
  //controlReservLED();
  heartBeat();
}

void motorOn() {
  if (CISTER_FLOW_STATUS == 0) {
    digitalWrite(CISTER_FLOW_OUT, LOW);
    CISTER_FLOW_STATUS = 1;
    #ifdef SERIAL_DEBUG
    Serial.println("LIGA MOTOR CISTERNA");
    #endif
  }
}

void solenoidOff() {
  if (CONCES_FLOW_STATUS == 1) {
    digitalWrite(CONCES_FLOW_OUT, HIGH);
    CONCES_FLOW_STATUS = 0;
    #ifdef SERIAL_DEBUG
    Serial.println("DESLIGA SOLENOIDE CASAN");
    #endif
  }
}

void motorOff() {
  if (CISTER_FLOW_STATUS == 1) {
    digitalWrite(CISTER_FLOW_OUT, HIGH);
    CISTER_FLOW_STATUS = 0;
    #ifdef SERIAL_DEBUG
    Serial.println("DESLIGA MOTOR CISTERNA");
    #endif
  }
}

void solenoidOn() {
  if (CONCES_FLOW_STATUS == 0) {
    digitalWrite(CONCES_FLOW_OUT, LOW);
    CONCES_FLOW_STATUS = 1;
    #ifdef SERIAL_DEBUG
    Serial.println("LIGA SOLENOIDE CASAN");
    #endif
  }
}

void doSecurityCheck() {
  
  if (CISTER_FLOW_STATUS == 1) {
    TOTAL_TIME_CISTER_FLOW = LOOP_TIME - TIME_CISTER_FLOW;
    if (TOTAL_TIME_CISTER_FLOW > MAX_TIME_CISTER_FLOW) {
      motorOff();
      solenoidOff();
      CISTER_FLOW_ERROR = 1;
      EEPROM.update(CISTER_ERROR_ADDR, 1);
    }
  }

  if (CONCES_FLOW_STATUS == 1) {
    TOTAL_TIME_CONCES_FLOW = LOOP_TIME - TIME_CONCES_FLOW;
    if (TOTAL_TIME_CONCES_FLOW > MAX_TIME_CONCES_FLOW) {
      motorOff();
      solenoidOff();
      CONCES_FLOW_ERROR = 1;
      EEPROM.update(CONCES_ERROR_ADDR, 1);
    }
  }  


  if (digitalRead(ERROR_RESET_PIN) == LOW) {
    EEPROM.update(CONCES_ERROR_ADDR, 0);
    EEPROM.update(CISTER_ERROR_ADDR, 0);
    CONCES_FLOW_ERROR = 0;
    CISTER_FLOW_ERROR = 0;
  }
}

#ifdef SERIAL_DEBUG
void printSerialLog() {
  String ctrn = "NAO";
  
  if (CISTER_EMPTY) {
    ctrn = "SIM";
  }

  String rsrv = "NAO";
  if (RESERV_EMPTY) {
    rsrv = "SIM";
  }

  float TEMPO_CASAN = TOTAL_TIME_CONCES_FLOW / SECOND;
  float TEMPO_MOTOR = TOTAL_TIME_CISTER_FLOW / SECOND;

  Serial.print("CISTERNA VAZIA: "+ ctrn+ " RESERVATORIO VAZIO: "+rsrv);
  if (CONCES_FLOW_STATUS == 1) {
    Serial.print(" CASAN ABERTO TS: ");    
    Serial.print(TEMPO_CASAN);
  }
  if (CISTER_FLOW_STATUS == 1) {
    Serial.print(" MOTOR LIGADO TM: ");  
    Serial.print(TEMPO_MOTOR) ; 
  }
  
  Serial.println("");

}
#endif


void updateLCD() {

  // Atualizas Nivel Cisterna
  lcd.setCursor(13, 0);
  if (CISTER_FUL == HIGH) {
    lcd.write(byte(3));
  } else if (CISTER_MID == HIGH) {
    lcd.write(byte(2));
  } else if (CISTER_LOW == HIGH) {
    lcd.write(byte(1));
  } else {
    lcd.write(byte(0));
  }

  // Atualiza Nivel Caixa
  lcd.setCursor(3, 0);
  if (RESERV_EMPTY == HIGH) {
    lcd.write(byte(1));
  } else {
    lcd.write(byte(3));
  }

  float TEMPO_CASAN = TOTAL_TIME_CONCES_FLOW / SECOND;
  float TEMPO_MOTOR = TOTAL_TIME_CISTER_FLOW / SECOND;

  if (CONCES_FLOW_STATUS == 1) {
    lcd.setCursor(0, 1);
    //lcd.print("Sem Fluxo  --:--");  
    lcd.print("Solenoide  ");
    //lcd.print(TEMPO_CASAN);
    printTime(TOTAL_TIME_CONCES_FLOW);
  }
  if (CISTER_FLOW_STATUS == 1) {
    lcd.setCursor(0, 1);
    //lcd.print("Sem Fluxo  --:--");  
    lcd.print("Motor      ");    
    //lcd.print(TEMPO_MOTOR);
    printTime(TOTAL_TIME_CISTER_FLOW);
  }
}

void readCisternStatus() {
  // CISTER_FULL E CISTER_MIDL SAO APENAS INFORMATIVOS, PORTANTO NAO NECESSITAM DEBOUNCING
  CISTER_FUL  = digitalRead(CISTER_FUL_FLOAT);
  CISTER_MID  = digitalRead(CISTER_MID_FLOAT);
  CISTER_LOW  = digitalRead(CISTER_LOW_FLOAT);
  
  // UTILIZA O NOT PORQUE ESTARA VAZIO DEPOIS QUE O LOW (NIVEL BAIXO) ESTIVER DESLIGADO
  int EMPTY_READ = !CISTER_LOW;

  // QUANDO O ESTADO DE LEITURA MUDAR, REGISTRA O TEMPO
  if (EMPTY_READ != CISTER_EMPTY_READ) {
    LAST_CISTER_EMPTY_DEBOUNCING_TIME = LOOP_TIME;
  }

  if ((LOOP_TIME - LAST_CISTER_EMPTY_DEBOUNCING_TIME) >= DEBOUNCING_DELAY) {
    if (EMPTY_READ != CISTER_EMPTY) {
      CISTER_EMPTY = EMPTY_READ;
    }
  }
  // GUARDA A LEITURA PARA O PROXIMO CICLO
  CISTER_EMPTY_READ = EMPTY_READ;
}

void readReservatoryStatus() {

  int EMPTY_READ = digitalRead(RESERV_FLOAT);

  if (EMPTY_READ != RESERV_EMPTY_READ) {
    LAST_RESERV_EMPTY_DEBOUNCING_TIME = LOOP_TIME;
  }

  if ((LOOP_TIME - LAST_RESERV_EMPTY_DEBOUNCING_TIME) >= DEBOUNCING_DELAY) {
    if (EMPTY_READ != RESERV_EMPTY) {
      RESERV_EMPTY = EMPTY_READ;
    }
  }
  // GUARDA A LEITURA PARA O PROXIMO CICLO
  RESERV_EMPTY_READ = EMPTY_READ;
}

void heartBeat() {
  
  if (LOOP_TIME - HEART_BEAT_LAST >= 1) {
    digitalWrite(HEART_BEAT, LOW);
  }

  if (LOOP_TIME - HEART_BEAT_LAST >= 2000) {
      digitalWrite(HEART_BEAT, HIGH);
      HEART_BEAT_LAST = LOOP_TIME;
      #ifdef SERIAL_DEBUG
      printSerialLog();
      #endif
  }
}


// argument is time in milliseconds
void printTime(unsigned long t_milli)
{
    char buffer[20];
    int days, hours, mins, secs;
    int fractime;
    unsigned long inttime;

    inttime  = t_milli / 1000;
    fractime = t_milli % 1000;
    // inttime is the total number of number of seconds
    // fractimeis the number of thousandths of a second

    // number of days is total number of seconds divided by 24 divided by 3600
    days     = inttime / (24*3600);
    inttime  = inttime % (24*3600);

    // Now, inttime is the remainder after subtracting the number of seconds
    // in the number of days
    hours    = inttime / 3600;
    inttime  = inttime % 3600;

    // Now, inttime is the remainder after subtracting the number of seconds
    // in the number of days and hours
    mins     = inttime / 60;
    inttime  = inttime % 60;

    // Now inttime is the number of seconds left after subtracting the number
    // in the number of days, hours and minutes. In other words, it is the
    // number of seconds.
    secs = inttime;

    // Don't bother to print days
    sprintf(buffer, "%02d:%02d", mins, secs);
    lcd.print(buffer);
}
// void controlCisterLED() {
//   if (CISTER_FLOW_ERROR == 1) {
//     if ((LOOP_TIME - CISTER_FLOW_STATUS_LED_LAST >= 200) & CISTER_STATUS_LED_STATE == 1) {
//       digitalWrite(CISTER_STATUS_LED, LOW);
//       CISTER_STATUS_LED_STATE = 0;
//     }

//     if ((LOOP_TIME - CISTER_FLOW_STATUS_LED_LAST >= 400) & CISTER_STATUS_LED_STATE == 0) {
//       digitalWrite(CISTER_STATUS_LED, HIGH);
//       CISTER_STATUS_LED_STATE = 1;
//       CISTER_FLOW_STATUS_LED_LAST = LOOP_TIME;
//     }

//   } else {
//     if (CISTER_FLOW_STATUS == 1) {
//       if (CISTER_STATUS_LED_STATE == 0) {
//         digitalWrite(CISTER_STATUS_LED, HIGH);
//         CISTER_STATUS_LED_STATE = 1;
//       }
//     } else {
//       if (CISTER_STATUS_LED_STATE == 1) {
//         digitalWrite(CISTER_STATUS_LED, LOW);
//         CISTER_STATUS_LED_STATE = 0;
//       }
//     }
//   }
// }


// void controlReservLED() {
//   if (CONCES_FLOW_ERROR == 1) {
//     if ((LOOP_TIME - CONCES_FLOW_STATUS_LED_LAST >= 200) & RESERV_STATUS_LED_STATE == 1) {
//       digitalWrite(RESERV_STATUS_LED, LOW);
//       RESERV_STATUS_LED_STATE = 0;
//     }

//     if ((LOOP_TIME - CONCES_FLOW_STATUS_LED_LAST >= 400) & RESERV_STATUS_LED_STATE == 0) {
//       digitalWrite(RESERV_STATUS_LED, HIGH);
//       RESERV_STATUS_LED_STATE = 1;
//       CONCES_FLOW_STATUS_LED_LAST = LOOP_TIME;
//     }
//   } else {
//     if (CONCES_FLOW_STATUS == 1) { 
//       if (RESERV_STATUS_LED_STATE == 0) {
//         digitalWrite(RESERV_STATUS_LED, HIGH);
//         RESERV_STATUS_LED_STATE = 1;
//       }
//     } else {
//       if (RESERV_STATUS_LED_STATE == 1) {
//         digitalWrite(RESERV_STATUS_LED, LOW);
//         RESERV_STATUS_LED_STATE = 0;
//       }
//     }
//   }
// }

