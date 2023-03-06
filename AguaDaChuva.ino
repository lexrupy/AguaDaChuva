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
#include <RotaryEncoder.h>


#define RESERV_SENS A0
#define CISTER_SENS A1

#define ROTARY_ENCODER_DT  A2
#define ROTARY_ENCODER_CLK A3
#define ROTARY_ENCODER_SW  A4

#define CONCES_FLOW_OUT 2   /* FLUXO DE ÁGUA DA CONSESSIONARIA*/
#define CISTER_FLOW_OUT 3   /* FLUXO DE ÁGUA DA CISTERNA */

#define LCD_DT4 4
#define LCD_DT5 5
#define LCD_DT6 6
#define LCD_DT7 7
#define LCD_RS 8
#define LCD_EN 9

#define LCD_BL 10

#define ERROR_RESET_PIN 11 // Reset colocando ERROR_RESET_OUT TO GND
#define ERROR_RESET_OUT 12


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

#define MAX_TIME_CISTER_FLOW 5 * MINUTE // DEFAULT 5 MINUTOS
#define MAX_TIME_CONCES_FLOW 5 * MINUTE // DEFAULT 5 MINUTOS

#define STARTUP_TIME 3 * SECOND
#define SENSOR_DB_TIME 3 * SECOND

// Define o Nivel que a caixa deve estar para acionar motor ou solenoide
// 0 -> iniciar quando vazio
// 1 -> iniciar quando nivel baixo
// 2 -> iniciar quando nivel medio
#define START_LEVEL 1

#define LEVEL_EMPTY 0
#define LEVEL_LOW   1
#define LEVEL_MID   2
#define LEVEL_FULL  3
#define HEART       4
#define VOLTAR      5


LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_DT4, LCD_DT5, LCD_DT6, LCD_DT7);
RotaryEncoder encoder(ROTARY_ENCODER_DT, ROTARY_ENCODER_CLK, RotaryEncoder::LatchMode::TWO03);

byte lvlEmpty[8] = { 0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1F };
byte lvlLow[8]   = { 0x0E, 0x11, 0x11, 0x11, 0x11, 0x1F, 0x1F, 0x1F };
byte lvlMid[8]   = { 0x0E, 0x11, 0x11, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F };
byte lvlFull[8]  = { 0x0E, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F };
byte heartChr[8] = { 0x00, 0x0A, 0x1F, 0x1F, 0x1F, 0x0E, 0x04, 0x00 };
byte stateOn[8]  = { 0x00, 0x04, 0x0E, 0x1F, 0x1F, 0x0E, 0x04, 0x00 };//{ 0x0C, 0x12, 0x12, 0x0C, 0x12, 0x1A, 0x16, 0x12 };//{ 0x00, 0x0E, 0x15, 0x1F, 0x1F, 0x15, 0x0E, 0x00 };
byte stateOff[8] = { 0x00, 0x04, 0x0A, 0x11, 0x11, 0x0A, 0x04, 0x00 };//{ 0x00, 0x0E, 0x11, 0x11, 0x11, 0x11, 0x0E, 0x00 };
byte voltarChr[8]= { 0x04, 0x0C, 0x1F, 0x0D, 0x05, 0x01, 0x1F, 0x00 };



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
unsigned long LCD_BL_TIMEOUT = 1 * MINUTE; // Default 1 minuto
int LCD_BL_INTENSIDADE = 50; // 50%
unsigned long LAST_ITERATION_TIME = 0;

bool CISTER_STARTUP_DONE = false;
bool RESERV_STARTUP_DONE = false;

// ================================ menu ==================================
int ULTIMO_MENU = 0;
int ULTIMO_SUBMENU = 0;
int MENU_ATUAL = 0;
int SUBMENU_ATUAL = 0;
bool IN_MENU = false;
bool IN_SUBMENU = false;

bool DO_SUBMENU_ACTION = false;

int MENU_SIZE = 5;
int SUBMENU_SIZE = 2;

bool DO_MENU_DRAW = false;


int ENCODER_POS = 0;
unsigned long LAST_ENCODER_SW_TIME;
unsigned long ENCODER_SW_DEBOUNCE_TIME;
int LAST_ENCODER_SW_STATE = 0;



void setup() {
  pinMode(CISTER_SENS, INPUT);
  pinMode(RESERV_SENS, INPUT);

  pinMode(ERROR_RESET_OUT, OUTPUT);
  digitalWrite(ERROR_RESET_OUT, HIGH);

  pinMode(ERROR_RESET_PIN, INPUT_PULLUP);

  pinMode(ROTARY_ENCODER_SW, INPUT_PULLUP);

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
  lcd.createChar(LEVEL_EMPTY, lvlEmpty);
  lcd.createChar(LEVEL_LOW, lvlLow);
  lcd.createChar(LEVEL_MID, lvlMid);
  lcd.createChar(LEVEL_FULL, lvlFull);
  lcd.createChar(HEART, heartChr);
  lcd.createChar(VOLTAR, voltarChr);
  printStatusSkel();

  Serial.begin(9600);

  PCICR |= (1 << PCIE1);
  PCMSK1 |= (1 << PCINT10) | (1 << PCINT11);
}

ISR(PCINT1_vect) {
  encoder.tick(); // just call tick() to check the state.
}

void loop() {
  LOOP_TIME = millis();
  readMenuSwitch();
  readCisternStatus();
  readReservatoryStatus();
  // Checagem de segurança para desligar apos certo tempo sem concluir o enchimento da caixa
  doSecurityCheck();
  // Controle do Fluxo de Agua, motor, solenoide ETC
  controleDeFluxo();
  updateLCD();
  heartBeat();
  checkBacklight();
}

void controleDeFluxo(){
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
}

void readMenuSwitch(){
  int newPos = encoder.getPosition();
  int direction = 0;
  if (newPos != ENCODER_POS) {
    if (newPos > ENCODER_POS) direction = 1;
    LAST_ITERATION_TIME = LOOP_TIME;
    ENCODER_POS = newPos;
    if (LCD_BL_STATE == 1) {
      if (processEncoderRotation(direction) == 1) DO_MENU_DRAW = true;
    }
  }
  
  int encoderSWState = digitalRead(ROTARY_ENCODER_SW);

  if ( (LOOP_TIME - LAST_ENCODER_SW_TIME) > ENCODER_SW_DEBOUNCE_TIME) {
    if ((encoderSWState != LAST_ENCODER_SW_STATE) && (encoderSWState == LOW)) {
      LAST_ITERATION_TIME = LOOP_TIME;
      // Apenas executa Iteração se o Backlight Estiver Ligado, senão apenas atualiza o tempo de ultima
      // iteracao para o lcd acender
      if (LCD_BL_STATE == 1) {
        processEncoderButtonPress();
        DO_MENU_DRAW = true;
      }
    }
    LAST_ENCODER_SW_STATE = encoderSWState;
    ENCODER_SW_DEBOUNCE_TIME = LOOP_TIME;
  }
}

int processEncoderRotation(int direction) {
  int result = 0;
  if (IN_MENU) {
    // NO SUBMENU
    if (IN_SUBMENU) {
      ULTIMO_SUBMENU = SUBMENU_ATUAL;
      if (direction == 1) {
        if (SUBMENU_ATUAL < SUBMENU_SIZE) {
          SUBMENU_ATUAL++;
          result = 1;
        }
      } else {
        if (SUBMENU_ATUAL > 0) {
          SUBMENU_ATUAL--;
          result = 1;
        }
      }
    // NO MENU PRINCIPAL
    } else {
      ULTIMO_MENU = MENU_ATUAL;
      if (direction == 1) {
        if (MENU_ATUAL < MENU_SIZE) {
          MENU_ATUAL++;
          result = 1;
        }
      } else {
        if (MENU_ATUAL > 0) {
          MENU_ATUAL--;
          result = 1;
        }
      }
    }
  }
  return result;
}

void processEncoderButtonPress() {
  if (!IN_MENU) {
    MENU_ATUAL = 0;
    IN_MENU = true;
  } else {
    if (!IN_SUBMENU) {
      SUBMENU_ATUAL = 0;
      IN_SUBMENU = true;
    } else {
      DO_SUBMENU_ACTION = true;
    }
  }
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


  if (digitalRead(ROTARY_ENCODER_SW) == LOW) {
    EEPROM.update(CONCES_ERROR_ADDR, 0);
    EEPROM.update(CISTER_ERROR_ADDR, 0);
    CONCES_FLOW_ERROR = 0;
    CISTER_FLOW_ERROR = 0;
  }
}

void printStatusSkel() {
  lcd.setCursor(0, 0);
  lcd.print("Cx:  Ct:  AUTO ");
  lcd.write(byte(HEART));
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

void printMenu5(){}
void printMenu4(){}
void printMenu3(){}
void printMenu2(){}

void printMenu1() {
  lcd.setCursor(0,0);
  //         1234567890123456
  lcd.print("< Estado Motor >");

  if (IN_SUBMENU) {
    lcd.setCursor(0,1);
    //         1234567890123456
    lcd.print("  ON     OFF   ");
    lcd.write(byte(VOLTAR));
    if (CISTER_FLOW_STATUS == 1) {
      lcd.setCursor(1,1);
      lcd.print("*");
      lcd.setCursor(8,1);
      lcd.print(" ");
    } else {
      lcd.setCursor(1,1);
      lcd.print(" ");
      lcd.setCursor(8,1);
      lcd.print("*");
    }
    switch (SUBMENU_ATUAL) {
      case 0:
        lcd.setCursor(0,1);
        lcd.print(">");
        lcd.setCursor(7,1);
        lcd.print(" ");
        lcd.setCursor(14,1);
        lcd.print(" ");
        break;
      case 1:
        lcd.setCursor(0,1);
        lcd.print(" ");
        lcd.setCursor(7,1);
        lcd.print(">");
        lcd.setCursor(14,1);
        lcd.print(" ");
        break;
      case 2:
        lcd.setCursor(0,1);
        lcd.print(" ");
        lcd.setCursor(6,1);
        lcd.print(" ");
        lcd.setCursor(14,1);
        lcd.print(">");
        break;
    }
  } else {
    lcd.setCursor(0,1);
    lcd.print("                ");
  }
}

void printMenu0() {
  lcd.setCursor(0,0);
  //         1234567890123456
  lcd.print("   Modo Oper.  >");
  if (IN_SUBMENU) {
    lcd.setCursor(0,1);
    lcd.print(" * Automatico  >");
  } else {
    lcd.setCursor(0,1);
    lcd.print("                ");
  }
}

void updateLCD() {
  if (!IN_MENU) {
    statusScreen();    
  } else {
    if (DO_MENU_DRAW) {
      switch (MENU_ATUAL) {
        case 0:
          printMenu0();
          break;
        case 1:
          printMenu1();
          break;
        case 2:
          printMenu2();
          break;
        case 3:
          printMenu3();
          break;
        case 4:
          printMenu4();
          break;
        case 5:
          printMenu5();
          break;
      }
    }

  }
  DO_MENU_DRAW = false;
}

void statusScreen() {
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
      lcd.write(byte(LEVEL_EMPTY));
      break;
    case 1:
      lcd.write(byte(LEVEL_LOW));
      break;
    case 2:
      lcd.write(byte(LEVEL_MID));
      break;
    case 3:
      lcd.write(byte(LEVEL_FULL));
      break;
  }

  // Atualiza Nivel Caixa
  lcd.setCursor(3, 0);
  switch (RESERV_LEVEL_READ) {
    case 0:
      lcd.write(byte(LEVEL_EMPTY));
      break;
    case 1:
      lcd.write(byte(LEVEL_LOW));
      break;
    case 2:
      lcd.write(byte(LEVEL_MID));
      break;
    case 3:
      lcd.write(byte(LEVEL_FULL));
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
  } else if (SENSOR_VALUE < 700 && SENSOR_VALUE > 450) {
    // MEIA ~508
    LEVEL_READ = 2;
  } else if (SENSOR_VALUE < 450 && SENSOR_VALUE > 250) {
    // BAIXA ~337
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
  if (!IN_MENU) {
    if (LOOP_TIME - HEART_BEAT_LAST >= 1000) {
      lcd.setCursor(15,0);
      lcd.write(byte(HEART));
    }

    if (LOOP_TIME - HEART_BEAT_LAST >= 1500) {
      lcd.setCursor(15,0);
      lcd.print(" ");
      HEART_BEAT_LAST = LOOP_TIME;
    }
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
