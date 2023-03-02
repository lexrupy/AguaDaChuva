/*
 *  CONTROLADORA DE SISTERNA E RESERVATÓRIO (CAIXA DÁGUA)
 *
 *  A SISTERNA POSSUI 3 SENSORES DE NÍVEL SENDO
 *  CISTER_SENS
 *
 *  QUANDO A AGUA ALCANCA ALGUM SENSOR, ELE FICA NA POSICAO LIGADO
 *
 *  A CISTERNA ESTÁ VAZIA QUANDO O SENSOR APRESENTAR VALOR ~0
 *  A CISTERNA ESTÁ BAIXA QUANDO O SENSOR APRESENTAR VALOR ~517
 *  A CISTERNA ESTA MEIA  QUANDO O SENSOR APRESENTAR VALOR ~613
 *  A CISTERNA ESTA CHEIA QUANDO O SENSOR APRESENTAR VALOR ~1023
 *
 *   *
 *  O RESERVATORIO POSSUI 3 SENSORES DE NIVEL
 *  RESERV_SENS
 *
 *  O RESERVATÓRIO ESTÁ VAZIO QUANDO O SENSOR APRESENTAR VALOR ~0
 *  O RESERVATÓRIO ESTÁ BAIXO QUANDO O SENSOR APRESENTAR VALOR ~517
 *  O RESERVATÓRIO ESTA MEIO  QUANDO O SENSOR APRESENTAR VALOR ~613
 *  O RESERVATÓRIO ESTA CHEIO QUANDO O SENSOR APRESENTAR VALOR ~1023
 *
*/

#include <EEPROM.h>
#include <LiquidCrystal.h>


#define RESERV_SENS A1
#define CISTER_SENS A2

#define CONCES_FLOW_OUT 2   /* FLUXO DE ÁGUA DA CONSESSIONARIA*/
#define CISTER_FLOW_OUT 3   /* FLUXO DE ÁGUA DA CISTERNA */

#define LCD_DT4 4
#define LCD_DT5 5
#define LCD_DT6 6
#define LCD_DT7 7
#define LCD_RS 8
#define LCD_EN 9

#define ERROR_RESET_PIN 10 // Reset colocando ERROR_RESET_OUT TO GND
#define ERROR_RESET_OUT 11

#define LCD_BL 12

#define CISTER_ERROR_ADDR 0
#define CONCES_ERROR_ADDR 1

/*
 *  DEFINIR TEMPO MAXIMO QUE O SISTEMA PERMANECE ABERTO ANTES DE DESLIGAMENTO DE SEGURANCA
 *  MEDIR TEMPO QUE LEVA PARA ENCHER O RESERVATORIO COM O MOTOR DA CISTERNA, E DEIXAR MARGEM
 *
 *  MEDIR TEMPO DE LEVA PARA ENCHER O RESERVATORIO COM A AGUA DA CONCESSIONARIA E DEIXAR MARGEM
 *  CASO ATINGIR O TEMPO, PISCA O LED CORRESPONDENTE DE FLUXO
 *  1 SEGUNDO = 1000
 *  1 MINUTO  = 60000

*/

#define FLOW_CONCES 0
#define FLOW_CISTER 1

#define SECOND 1000
#define MINUTE 60000

#define MAX_TIME_CISTER_FLOW 3 * MINUTE // DEFAULT 5 MINUTOS
#define MAX_TIME_CONCES_FLOW 3 * MINUTE // DEFAULT 5 MINUTOS

#define STARTUP_TIME 3 * SECOND
#define SENSOR_DB_TIME 3 * SECOND

// Define o Nivel que a caixa deve estar para acionar motor ou solenoide
// 0 -> iniciar quando vazio
// 1 -> iniciar quando nivel baixo
// 2 -> iniciar quando nivel medio
#define START_LEVEL 1

LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_DT4, LCD_DT5, LCD_DT6, LCD_DT7);

byte lvlEmpty[8]    = { 0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1F };
byte lvlLow[8]      = { 0x0E, 0x11, 0x11, 0x11, 0x11, 0x1F, 0x1F, 0x1F };
byte lvlMid[8]      = { 0x0E, 0x11, 0x11, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F };
byte lvlFull[8]     = { 0x0E, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F };
byte heartOpen[8]   = { 0x00, 0x0A, 0x15, 0x11, 0x11, 0x0A, 0x04, 0x00 }; // {	0b00000,	0b01010,	0b10101,	0b10001,	0b01010,	0b00100,	0b00000,	0b00000};
byte heartClosed[8] = { 0x00, 0x0A, 0x1F, 0x1F, 0x1F, 0x0E, 0x04, 0x00 }; // {	0b00000,	0b01010,	0b11111,	0b11111,	0b01110,	0b00100,	0b00000,	0b00000};


unsigned long HEART_BEAT_LAST = 0;

int CISTER_LEVEL = 0;
// Iniciar Nivel do Reservatorio como 3 (Cheio), prevenindo ligar solenoide antes de terminar a leitura
int RESERV_LEVEL = 3;

int RESERV_LEVEL_READ = 0;
int CISTER_LEVEL_READ = 0;
unsigned long RESERV_LEVEL_READ_TIME = 0;
unsigned long CISTER_LEVEL_READ_TIME = 0;

bool RESERV_EMPTY_STATE = false;
bool CISTER_EMPTY_STATE = false;

int CISTER_FLOW_STATUS = 0;
int CISTER_FLOW_ERROR = 0;

int CONCES_FLOW_STATUS = 0;
int CONCES_FLOW_ERROR = 0;

bool FLOW_STATUS_CHANGED = false;
bool FLOW_ENABLED = false;

unsigned long TIME_CISTER_FLOW = 0;
unsigned long TIME_CONCES_FLOW = 0;

unsigned long TOTAL_TIME_CISTER_FLOW = 0;
unsigned long TOTAL_TIME_CONCES_FLOW = 0;

unsigned long LOOP_TIME = 0;

int LAST_FLOW_MODE = -1; // Flow Mode: CONCES/CISTER


int LCD_BL_STATE = 1; // 1 = Ligado, 0 = Desligado
unsigned long LCD_BL_TIMEOUT = 5 * SECOND;//1 * MINUTE; // Default 1 minuto
int LCD_BL_INTENSIDADE = 50; // 50%
unsigned long LAST_ITERATION_TIME = 0;

bool CISTER_STARTUP_DONE = false;
bool RESERV_STARTUP_DONE = false;



void setup() {
  pinMode(CISTER_SENS, INPUT);
  pinMode(RESERV_SENS, INPUT);

  pinMode(ERROR_RESET_OUT, OUTPUT);
  digitalWrite(ERROR_RESET_OUT, HIGH);

  pinMode(ERROR_RESET_PIN, INPUT_PULLUP);

  pinMode(LCD_BL, OUTPUT);
  digitalWrite(LCD_BL, HIGH);

  pinMode(CISTER_FLOW_OUT, OUTPUT);
  pinMode(CONCES_FLOW_OUT, OUTPUT);

  digitalWrite(CISTER_FLOW_OUT, HIGH);
  digitalWrite(CONCES_FLOW_OUT, HIGH);

  CONCES_FLOW_ERROR = EEPROM.read(CONCES_ERROR_ADDR);
  CISTER_FLOW_ERROR = EEPROM.read(CISTER_ERROR_ADDR);

  //------------lcd-----------

  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(" AguaDaChuva1.0 ");
  lcd.setCursor(0, 1);
  lcd.print("Inicializando...");
  delay(STARTUP_TIME);
  lcd.clear();
  lcd.createChar(0, lvlEmpty);
  lcd.createChar(1, lvlLow);
  lcd.createChar(2, lvlMid);
  lcd.createChar(3, lvlFull);
  lcd.createChar(4, heartOpen);
  lcd.createChar(5, heartClosed);
  printStatusSkel();

  Serial.begin(9600);
}

void loop() {
  LOOP_TIME = millis();
  readCisternStatus();
  readReservatoryStatus();
  doSecurityCheck();
  if (CISTER_STARTUP_DONE && RESERV_STARTUP_DONE) {
    if (RESERV_EMPTY_STATE) { /* CASO O RESERVATORIO NECESSITE ENCHER */
      if (!CISTER_EMPTY_STATE) { /* SE HOUVER AGUA NA CISTERNA ACIONA O MOTOR DA CISTERNA */
        if (CISTER_FLOW_STATUS == 0 && CISTER_FLOW_ERROR == 0) {
          /* INICIAR AQUI CONTEGEM DE TEMPO DE SERGURANCA... */
          TIME_CISTER_FLOW = LOOP_TIME;
          TOTAL_TIME_CISTER_FLOW = 0;
          solenoidOff();
          motorOn();
        }
      } else { /* SE NAO HOUVER AGUA NA SISTERNA ACIONA O MOTOR DA SISTERNA */
        if (CONCES_FLOW_STATUS == 0 && CONCES_FLOW_ERROR == 0) {
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
  }

  updateLCD();
  heartBeat();
  checkBacklight();
}

void checkBacklight() {
  // Backlight
  if (LOOP_TIME - LAST_ITERATION_TIME > LCD_BL_TIMEOUT) {
    if (LCD_BL_STATE == 1) {
      lcdBackLightOff();
    }
  } else {
    if (LCD_BL_STATE == 0) {
      lcdBackLightOn();
    }
  }
}

void lcdBackLightOff() {
  digitalWrite(LCD_BL, LOW);
  LCD_BL_STATE = 0;
}

void lcdBackLightOn() {
  digitalWrite(LCD_BL, HIGH);
  LCD_BL_STATE = 1;
}

void motorOn() {
  if (CISTER_FLOW_STATUS == 0) {
    digitalWrite(CISTER_FLOW_OUT, LOW);
    CISTER_FLOW_STATUS = 1;
    LAST_FLOW_MODE = FLOW_CISTER;
    FLOW_STATUS_CHANGED = true;
  }
}

void solenoidOff() {
  if (CONCES_FLOW_STATUS == 1) {
    digitalWrite(CONCES_FLOW_OUT, HIGH);
    CONCES_FLOW_STATUS = 0;
    FLOW_STATUS_CHANGED = true;
  }
}

void motorOff() {
  if (CISTER_FLOW_STATUS == 1) {
    digitalWrite(CISTER_FLOW_OUT, HIGH);
    CISTER_FLOW_STATUS = 0;
    FLOW_STATUS_CHANGED = true;
  }
}

void solenoidOn() {
  if (CONCES_FLOW_STATUS == 0) {
    digitalWrite(CONCES_FLOW_OUT, LOW);
    CONCES_FLOW_STATUS = 1;
    FLOW_STATUS_CHANGED = true;
    LAST_FLOW_MODE = FLOW_CONCES;
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

void printStatusSkel() {
  lcd.setCursor(0, 0);
  lcd.print("Cx:  Ct:  AUTO ");
  lcd.write(byte(4));
  lcd.setCursor(0, 1);
  lcd.print("Ocioso     --:--");
  if (CONCES_FLOW_ERROR == 1 | CISTER_FLOW_ERROR == 1) {
    lcd.setCursor(0, 1);
    if (CONCES_FLOW_ERROR == 1) {
      lcd.print("ERRO-SL");
    } else {
      lcd.print("ERRO-MT");
    }
  }
}

void updateLCD() {
  if (FLOW_STATUS_CHANGED) {
    printStatusSkel();
    if (LAST_FLOW_MODE == FLOW_CISTER) {
      lcd.setCursor(7,1);
      lcd.print("Mot>");
      printTime(TOTAL_TIME_CISTER_FLOW);
    } else if (LAST_FLOW_MODE == FLOW_CONCES) {
      lcd.setCursor(7,1);
      lcd.print("Sol>");
      printTime(TOTAL_TIME_CONCES_FLOW);
    } else {
      lcd.setCursor(7,1);
      lcd.print("   ");
    }

    FLOW_STATUS_CHANGED = false;
  }

  // Atualizas Nivel Cisterna
  lcd.setCursor(8, 0);
  switch (CISTER_LEVEL_READ) {
    case 0:
      lcd.write(byte(0));
      break;
    case 1:
      lcd.write(byte(1));
      break;
    case 2:
      lcd.write(byte(2));
      break;
    case 3:
      lcd.write(byte(3));
      break;
  }

  // Atualiza Nivel Caixa
  lcd.setCursor(3, 0);
  switch (RESERV_LEVEL_READ) {
    case 0:
      lcd.write(byte(0));
      break;
    case 1:
      lcd.write(byte(1));
      break;
    case 2:
      lcd.write(byte(2));
      break;
    case 3:
      lcd.write(byte(3));
      break;
  }

  if (CONCES_FLOW_STATUS == 1) {
    lcd.setCursor(0, 1);
               
    lcd.print("Solen. On  ");
    printTime(TOTAL_TIME_CONCES_FLOW);
  }
  if (CISTER_FLOW_STATUS == 1) {
    lcd.setCursor(0, 1);
    lcd.print("Motor On   ");
    printTime(TOTAL_TIME_CISTER_FLOW);
  }
}

void readCisternStatus() {
  int SENSOR_VALUE = analogRead(CISTER_SENS);
  int LEVEL_READ = 0;
  // CHEIA ~1023
  if (SENSOR_VALUE > 800) {
    LEVEL_READ = 3;
  } else if (SENSOR_VALUE < 800 && SENSOR_VALUE > 565) {
    // MEIA ~613
    LEVEL_READ = 2;
  } else if (SENSOR_VALUE < 565 && SENSOR_VALUE > 400) {
    // BAIXA ~517
    LEVEL_READ = 1;
  } else {
    // VAZIA ~0;
    LEVEL_READ = 0;
  }
  // RESERV_LEVEL = RESERV_LEVEL_READ;
  if (LEVEL_READ != CISTER_LEVEL_READ) {
    CISTER_LEVEL_READ_TIME = LOOP_TIME;
  }
  if ((LOOP_TIME - CISTER_LEVEL_READ_TIME) >= SENSOR_DB_TIME) {
    if (LEVEL_READ != CISTER_LEVEL) {
      CISTER_LEVEL = LEVEL_READ;
      CISTER_STARTUP_DONE = true;
    }
  }

  CISTER_LEVEL_READ = LEVEL_READ;

  if (CISTER_LEVEL < 1) {
    CISTER_EMPTY_STATE = true;
  } else {
    // Prevenir alternancia, entre motor e solenoide caso a cisterna começe a encher
    // e alcançe o primeiro nivel do sensor durante o enchimento
    if (CONCES_FLOW_STATUS == 0) {
      CISTER_EMPTY_STATE = false;
    }
  }
}

void readReservatoryStatus() {
  int SENSOR_VALUE = analogRead(RESERV_SENS);
  int LEVEL_READ = 0;
  // CHEIA ~1023
  if (SENSOR_VALUE > 800) {
    LEVEL_READ = 3;
  } else if (SENSOR_VALUE < 800 && SENSOR_VALUE > 565) {
    // MEIA ~613
    LEVEL_READ = 2;
  } else if (SENSOR_VALUE < 565 && SENSOR_VALUE > 400) {
    // BAIXA ~517
    LEVEL_READ = 1;
  } else {
    // VAZIA ~0;
    LEVEL_READ = 0;
  }
  // RESERV_LEVEL = RESERV_LEVEL_READ;
  if (LEVEL_READ != RESERV_LEVEL_READ) {
    RESERV_LEVEL_READ_TIME = LOOP_TIME;
  }
  if ((LOOP_TIME - RESERV_LEVEL_READ_TIME) >= SENSOR_DB_TIME) {
    if (LEVEL_READ != RESERV_LEVEL) {
      RESERV_LEVEL = LEVEL_READ;
      RESERV_STARTUP_DONE = true;
    }
  }

  RESERV_LEVEL_READ = LEVEL_READ;

  if (RESERV_LEVEL <= START_LEVEL) {
    RESERV_EMPTY_STATE = true;
  } else if (RESERV_LEVEL == 3 ) {
    RESERV_EMPTY_STATE = false;
  }
}

void heartBeat() {
  if (LOOP_TIME - HEART_BEAT_LAST >= 1000) {
    lcd.setCursor(15,0);
    lcd.write(byte(5));
  }

  if (LOOP_TIME - HEART_BEAT_LAST >= 1500) {
    lcd.setCursor(15,0);
    lcd.print(" ");
    HEART_BEAT_LAST = LOOP_TIME;
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
