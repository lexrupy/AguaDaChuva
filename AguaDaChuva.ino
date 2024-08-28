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

#define ROTARY_ENCODER_CLK A2
#define ROTARY_ENCODER_DT  A3
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
#define START_LEVEL_ADDR 2
#define CISTER_TIME_ADDR 3
#define CONCES_TIME_ADDR 4
#define BL_TIMEOUT_ADDR  5
#define CISTER_COUNT_ADDR 6 // INTEGER 2 BYTES
#define CONCES_COUNT_ADDR 8 // INTEGER 2 BYTES
#define CISTER_LAST_TIME_ADDR 10  // UNSIGNED LONG 4 BYTES
#define CONCES_LAST_TIME_ADDR 14  // UNSIGNED LONG 4 BYTES
#define LAST_FLOW_MODE_ADDR 18    // 1 BYTE


/*
 *  DEFINIR TEMPO MAXIMO QUE O SISTEMA PERMANECE ABERTO ANTES DE DESLIGAMENTO DE SEGURANCA
 *  MEDIR TEMPO QUE LEVA PARA ENCHER O RESERVATORIO COM O MOTOR DA CISTERNA, E DEIXAR MARGEM
 *
 *  MEDIR TEMPO DE LEVA PARA ENCHER O RESERVATORIO COM A AGUA DA CONCESSIONARIA E DEIXAR MARGEM
 *  CASO ATINGIR O TEMPO, PISCA O LED CORRESPONDENTE DE FLUXO
 *  1 SEGUNDO = 1000
 *  1 MINUTO  = 60000

*/


#define ST_OFF 0
#define ST_ON 1


#define ST_IDLE 0
#define ST_RUNNING 1


#define NO_ERROR 0
#define WITH_ERROR 1


#define FLOW_CONCES 0
#define FLOW_CISTER 1

#define SECOND 1000L
#define MINUTE 60000L

#define STARTUP_TIME 3 * SECOND
#define SENSOR_DB_TIME 3 * SECOND

// Define o Nivel que a caixa deve estar para acionar motor ou solenoide
// 0 -> iniciar quando vazio
// 1 -> iniciar quando nivel baixo
// 2 -> iniciar quando nivel medio
#define LEVEL_INVALID -1
#define LEVEL_EMPTY    0
#define LEVEL_LOW      1
#define LEVEL_MID      2
#define LEVEL_FULL     3
#define HEART          4
#define VOLTAR         5

LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_DT4, LCD_DT5, LCD_DT6, LCD_DT7);
RotaryEncoder encoder(ROTARY_ENCODER_DT, ROTARY_ENCODER_CLK, RotaryEncoder::LatchMode::TWO03);

byte lvlEmpty[8] = { 0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1F };
byte lvlLow[8]   = { 0x0E, 0x11, 0x11, 0x11, 0x11, 0x1F, 0x1F, 0x1F };
byte lvlMid[8]   = { 0x0E, 0x11, 0x11, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F };
byte lvlFull[8]  = { 0x0E, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F };
byte heartChr[8] = { 0x00, 0x0A, 0x1F, 0x1F, 0x1F, 0x0E, 0x04, 0x00 };
byte stateOn[8]  = { 0x00, 0x04, 0x0E, 0x1F, 0x1F, 0x0E, 0x04, 0x00 };
byte stateOff[8] = { 0x00, 0x04, 0x0A, 0x11, 0x11, 0x0A, 0x04, 0x00 };
byte voltarChr[8]= { 0x04, 0x0C, 0x1F, 0x0D, 0x05, 0x01, 0x1F, 0x00 };

unsigned long HEART_BEAT_LAST = 0;

unsigned long MAX_TIME_CISTER_FLOW = 7 * MINUTE; // DEFAULT 7 MINUTOS
unsigned long MAX_TIME_CONCES_FLOW = 7 * MINUTE; // DEFAULT 7 MINUTOS

int CISTER_LEVEL = LEVEL_INVALID;
int RESERV_LEVEL = LEVEL_INVALID;

int RESERV_LEVEL_READ = LEVEL_EMPTY;
int CISTER_LEVEL_READ = LEVEL_EMPTY;
unsigned long RESERV_LEVEL_READ_TIME = 0;
unsigned long CISTER_LEVEL_READ_TIME = 0;

bool RESERV_EMPTY_STATE = false;
bool CISTER_EMPTY_STATE = false;

int CISTER_FLOW_STATUS = ST_IDLE;
int CISTER_FLOW_ERROR = NO_ERROR;

int CONCES_FLOW_STATUS = ST_IDLE;
int CONCES_FLOW_ERROR = NO_ERROR;

bool FLOW_ENABLED = false;

unsigned long TIME_CISTER_FLOW = 0;
unsigned long TIME_CONCES_FLOW = 0;

unsigned long TOTAL_TIME_CISTER_FLOW = 0;
unsigned long TOTAL_TIME_CONCES_FLOW = 0;

unsigned int LOOP_TIME_CYCLES = 0;

unsigned long LAST_LOOP_TIME = 0;
unsigned long LOOP_TIME = 0;

byte LAST_FLOW_MODE = 255; // Flow Mode: CONCES/CISTER

unsigned int CONCES_COUNTER = 0;
unsigned int CISTER_COUNTER = 0;

bool REFRESH_STATUS_SCREEN = false;
int LCD_BL_STATE = ST_ON; // 1 = Ligado, 0 = Desligado
int LCD_BL_TIMEOUT = 3; // Tempo LCD Ligado
unsigned long LAST_ITERATION_TIME = 0;

bool CISTER_STARTUP_DONE = false;
bool RESERV_STARTUP_DONE = false;

int MODO_OPERACAO = 0; // 0 = AUTOMATICO, 1 = MANUAL

int START_LEVEL = LEVEL_LOW;

// ================================ menu ==================================
int MENU_ATUAL = 0;
int SUBMENU_ATUAL = 0;
bool IN_MENU = false;
bool IN_SUBMENU = false;
bool IN_SUBMENU_SELECTION = false;

bool DO_SUBMENU_ACTION = false;

int MENU_SIZE = 9;
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
  CISTER_FLOW_ERROR = EEPROM.read(CISTER_ERROR_ADDR);
  START_LEVEL = EEPROM.read(START_LEVEL_ADDR);
  LCD_BL_TIMEOUT = EEPROM.read(BL_TIMEOUT_ADDR);

  byte CISTER_TIME_VALUE = EEPROM.read(CISTER_TIME_ADDR);
  byte CONCES_TIME_VALUE = EEPROM.read(CONCES_TIME_ADDR);
  
  if (START_LEVEL > LEVEL_MID) {
    EEPROM.update(START_LEVEL_ADDR, LEVEL_EMPTY);
    START_LEVEL = LEVEL_EMPTY;
  }

  if (CISTER_TIME_VALUE > 40) {
    CISTER_TIME_VALUE = 14; // 7 Minutes
    EEPROM.update(CISTER_TIME_ADDR, CISTER_TIME_VALUE);
  }

  if (CONCES_TIME_VALUE > 40) {
    CONCES_TIME_VALUE = 14; // 7 Minutes
    EEPROM.update(CONCES_TIME_ADDR, CONCES_TIME_VALUE);
  }

  if (LCD_BL_TIMEOUT > 5) {
    LCD_BL_TIMEOUT = 5; // Max 5 Minutes
    EEPROM.update(BL_TIMEOUT_ADDR, LCD_BL_TIMEOUT);
  }

  // Multiplos de 30 Segundos
  MAX_TIME_CISTER_FLOW = 30000L * CISTER_TIME_VALUE;
  MAX_TIME_CONCES_FLOW = 30000L * CONCES_TIME_VALUE;
 

  LAST_FLOW_MODE = EEPROM.read(LAST_FLOW_MODE_ADDR);

  
  EEPROM.get(CISTER_COUNT_ADDR, CISTER_COUNTER);
  if (CISTER_COUNTER > 9999) {
    CISTER_COUNTER = 0;
    EEPROM.put(CISTER_COUNT_ADDR, CISTER_COUNTER);
  }

  
  EEPROM.get(CONCES_COUNT_ADDR, CONCES_COUNTER);
  if (CONCES_COUNTER > 9999) {
    CONCES_COUNTER = 0;
    EEPROM.put(CONCES_COUNT_ADDR, CONCES_COUNTER);
  }

  EEPROM.get(CISTER_LAST_TIME_ADDR, TOTAL_TIME_CISTER_FLOW);
  if (TOTAL_TIME_CISTER_FLOW >  3599000L) {
    TOTAL_TIME_CISTER_FLOW = 3599000L;
    EEPROM.put(CISTER_LAST_TIME_ADDR, TOTAL_TIME_CISTER_FLOW);
  }
  
  EEPROM.get(CONCES_LAST_TIME_ADDR, TOTAL_TIME_CONCES_FLOW);
  if (TOTAL_TIME_CONCES_FLOW >  3599000L) {
    TOTAL_TIME_CONCES_FLOW = 3599000L;
    EEPROM.put(CONCES_LAST_TIME_ADDR, TOTAL_TIME_CONCES_FLOW);
  }

  //------------lcd-----------

  lcd.begin(16, 2);
  lcd.clear();

  lcd.createChar(LEVEL_EMPTY, lvlEmpty);
  lcd.createChar(LEVEL_LOW, lvlLow);
  lcd.createChar(LEVEL_MID, lvlMid);
  lcd.createChar(LEVEL_FULL, lvlFull);
  lcd.createChar(HEART, heartChr);
  lcd.createChar(VOLTAR, voltarChr);


  lcd.setCursor(0, 0);
  lcd.print(" Agua da Chuva  ");
  lcd.setCursor(0, 1);
  lcd.print("Ver. 1.1        ");
  lcd.setCursor(9, 1);
  int _delay = STARTUP_TIME/7;
  for (int i = 0; i<7; i++){
    lcd.write(0xFF);
    delay(_delay);
  }
  
  lcd.clear();

  REFRESH_STATUS_SCREEN = true;

  printStatusSkel();

  // Serial.begin(9600);


  // Serial.print("LastFlow: ");
  // Serial.println(LAST_FLOW_MODE);

  PCICR |= (1 << PCIE1);
  PCMSK1 |= (1 << PCINT10) | (1 << PCINT11);
}

ISR(PCINT1_vect) {
  encoder.tick(); // just call tick() to check the state.
}

void loop() {
  LAST_LOOP_TIME = LOOP_TIME;
  LOOP_TIME = millis();
  // Completou um ciclo de 49 dias 17:02:47.295
  if (LOOP_TIME < LAST_LOOP_TIME) {
    LOOP_TIME_CYCLES++;
  }
  readMenuSwitch();
  readCisternStatus();
  readReservatoryStatus();
  // Checagem de segurança para desligar apos certo tempo sem concluir o enchimento da caixa
  doSecurityCheck();
  // Controle do Fluxo de Agua, motor, solenoide ETC
  if (MODO_OPERACAO == 0) controleDeFluxo();
  updateLCD();
  heartBeat();
  checkBacklight();
}

void controleDeFluxo(){
    if (CISTER_STARTUP_DONE && RESERV_STARTUP_DONE) {
      if (RESERV_EMPTY_STATE) { /* CASO O RESERVATORIO NECESSITE ENCHER */
        if (!CISTER_EMPTY_STATE) { /* SE HOUVER AGUA NA CISTERNA ACIONA O MOTOR DA CISTERNA */
          if (CISTER_FLOW_STATUS == ST_IDLE && CISTER_FLOW_ERROR == NO_ERROR) {
            /* INICIAR AQUI CONTEGEM DE TEMPO DE SERGURANCA... */
            TIME_CISTER_FLOW = LOOP_TIME;
            TOTAL_TIME_CISTER_FLOW = 0;
            // Increment and Write to EEPROM just before manage actuators 
            CISTER_COUNTER++;
            EEPROM.put(CISTER_COUNT_ADDR, CISTER_COUNTER);
            solenoidOff();
            motorOn();
          }
        } else { /* SE NAO HOUVER AGUA NA CISTERNA ACIONA A VALVULA ELETRICA */
          if (CONCES_FLOW_STATUS == ST_IDLE && CONCES_FLOW_ERROR == NO_ERROR) {
            TIME_CONCES_FLOW = LOOP_TIME;
            TOTAL_TIME_CONCES_FLOW = 0;
            // Increment and Write to EEPROM just before manage actuators 
            CONCES_COUNTER++;
            EEPROM.put(CONCES_COUNT_ADDR, CONCES_COUNTER);
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

bool validLevels() {
  if ((CISTER_LEVEL != LEVEL_INVALID) && (RESERV_LEVEL != LEVEL_INVALID)) {
    return true;
  } else {
    return false;
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
      // processEncoderRotation(direction);
      // DO_MENU_DRAW = true;
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
      // Selecao de Valor de Submenu
      if (IN_SUBMENU_SELECTION) {
        switch(MENU_ATUAL) {
          // Menu de Segurança
          case 6:
            switch(SUBMENU_ATUAL) {
              // Submenu Motor
              case 0:
                if (direction == 1) {
                  if (MAX_TIME_CISTER_FLOW < 1200000) MAX_TIME_CISTER_FLOW += 30000L;
                } else {
                  if (MAX_TIME_CISTER_FLOW > 30000) MAX_TIME_CISTER_FLOW -= 30000L;
                }
                // executar o print aqui mesmo para evitar redraw de toda a tela
                lcd.setCursor(5,1);
                printTime(MAX_TIME_CISTER_FLOW);
                //DO_MENU_DRAW = true;
                break;
              // Submenu Solenoide
              case 1:
                if (direction == 1) {
                  if (MAX_TIME_CONCES_FLOW < 1200000) MAX_TIME_CONCES_FLOW += 30000L;
                } else {
                  if (MAX_TIME_CONCES_FLOW > 30000) MAX_TIME_CONCES_FLOW -= 30000L;
                }
                // executar o print aqui mesmo para evitar redraw de toda a tela
                lcd.setCursor(5,1);
                printTime(MAX_TIME_CONCES_FLOW);
                //DO_MENU_DRAW = true;
                break;
            }
            break;
          case 7:
            switch(SUBMENU_ATUAL) {
              // Submenu Motor
              case 0:
                if (direction == 1) {
                  if (LCD_BL_TIMEOUT < 5) LCD_BL_TIMEOUT++;
                } else {
                  if (LCD_BL_TIMEOUT > 1) LCD_BL_TIMEOUT--;
                }
                // executar o print aqui mesmo para evitar redraw de toda a tela
                lcd.setCursor(4,1);
                lcd.print(LCD_BL_TIMEOUT);
                //DO_MENU_DRAW = true;
                break;
            }
            break;
        }
     
      // Rotação normal entre submenus
      } else {
        result = 1;
        if (direction == 1) {
          if (SUBMENU_ATUAL == SUBMENU_SIZE) SUBMENU_ATUAL = 0;
          else if (SUBMENU_ATUAL < SUBMENU_SIZE) SUBMENU_ATUAL++;
        } else {
          if (SUBMENU_ATUAL > 0) SUBMENU_ATUAL--;
          else SUBMENU_ATUAL = SUBMENU_SIZE;
        }

        // Se submen de debug, dispara draw
        if (MENU_ATUAL == 3 || MENU_ATUAL == 8) DO_MENU_DRAW = true;
      }

    // NO MENU PRINCIPAL
    } else {
      result = 1;
      if (direction == 1) {
        if (MENU_ATUAL == MENU_SIZE) MENU_ATUAL = 0;
        else if (MENU_ATUAL < MENU_SIZE) MENU_ATUAL++;
      } else {
        if (MENU_ATUAL > 0) MENU_ATUAL--;
        else MENU_ATUAL = MENU_SIZE;
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
      if (MENU_ATUAL == MENU_SIZE) {
        REFRESH_STATUS_SCREEN = true;
        IN_MENU = false;
      } else {
        SUBMENU_ATUAL = 0;
        IN_SUBMENU = true;
      }
    } else if (!IN_SUBMENU_SELECTION) {
      // Executa a acao de acordo com o menu e submenu selecionado
      switch (MENU_ATUAL) {
        // Menu Modo Operacao
        case 0:
          switch (SUBMENU_ATUAL) {
            case 0:
              if (MODO_OPERACAO == 1) {
                MODO_OPERACAO = 0;
                DO_MENU_DRAW = true;
              }
              break;
            case 1:
              if (MODO_OPERACAO == 0) {
                MODO_OPERACAO = 1;
                DO_MENU_DRAW = true;
              }
              break;
            // Voltar
            case 2:
              IN_SUBMENU = false;
              break;
          }
          break;
        // Menu Estado Motor
        case 1:
          switch (SUBMENU_ATUAL) {
            case 0:
              if (CISTER_FLOW_STATUS == ST_IDLE) {
                TIME_CISTER_FLOW = LOOP_TIME;
                TOTAL_TIME_CISTER_FLOW = 0;
                // Reseta o Empty State do reservatório, caso contrario o motor
                // desativara assim que ligar
                if (MODO_OPERACAO == 0) {
                  RESERV_EMPTY_STATE = true;
                }
                motorOn();
              }
              DO_MENU_DRAW = true;
              break;
            case 1:
              if (CISTER_FLOW_STATUS == ST_RUNNING) {
                motorOff();
                RESERV_EMPTY_STATE = false;
              }
              DO_MENU_DRAW = true;
              break;
            // Voltar
            case 2:
              IN_SUBMENU = false;
              break;
            
          }
          break;
          // Menu Estado Solenoide
        case 2:
          switch (SUBMENU_ATUAL) {
            case 0:
              if (CONCES_FLOW_STATUS == ST_IDLE) {
                TIME_CONCES_FLOW = LOOP_TIME;
                TOTAL_TIME_CONCES_FLOW = 0;
                solenoidOn();
              }
              DO_MENU_DRAW = true;
              break;
            case 1:
              if (CONCES_FLOW_STATUS == ST_RUNNING) solenoidOff();
              DO_MENU_DRAW = true;
              break;
            // Voltar
            case 2:
              IN_SUBMENU = false;
              break;
          }
          break;
        // Menu DEBUG
        case 3:
          switch (SUBMENU_ATUAL) {
            case 0:
              IN_SUBMENU = false;
              break;
          }
          break;
        // Menu Reset Erro
        case 4:
          switch (SUBMENU_ATUAL) {
            case 0:
              // Retira o Estado de erro
              EEPROM.update(CONCES_ERROR_ADDR, 0);
              EEPROM.update(CISTER_ERROR_ADDR, 0);
              CONCES_FLOW_ERROR = NO_ERROR;
              CISTER_FLOW_ERROR = NO_ERROR;
              IN_SUBMENU = false;
              break;
            case 1:
              IN_SUBMENU = false;
              break;
          }
          break;
        // Menu Nivel Partida
        case 5:
          switch (SUBMENU_ATUAL) {
            case 0:
              START_LEVEL = LEVEL_EMPTY;
              break;
            case 1:
              START_LEVEL = LEVEL_LOW;
              break;
            case 2:
              START_LEVEL = LEVEL_MID;
              break;
            case 3:
              EEPROM.update(START_LEVEL_ADDR, START_LEVEL);
              IN_SUBMENU = false;
              break;
          }
          break;
        // Menu Segurança
        case 6:
          switch ( SUBMENU_ATUAL) {
            case 0:
              IN_SUBMENU_SELECTION = true;
              break;
            case 1:
              IN_SUBMENU_SELECTION = true;
              break;
            case 2:
              IN_SUBMENU = false;
              break;
          }
          break;
        case 7:
          switch  (SUBMENU_ATUAL) {
            case 0:
              IN_SUBMENU_SELECTION = true;
              break;
          }
          break;
        case 8:
          switch  (SUBMENU_ATUAL) {
            case 0:
              IN_SUBMENU = false;
              break;
          }
          break;
      }
    // Selecao de Opcao de Submenu por rotacao do encoder
    } else {
      // Por algum motivo este calculo dentro do switch do submenu, não funciona
      // por tanto foi movido para fora do switch case
      int CISTER_TIME = MAX_TIME_CISTER_FLOW / 30000;
      int CONCES_TIME = MAX_TIME_CONCES_FLOW / 30000;
      switch ( MENU_ATUAL ) {
        case 6:
          switch ( SUBMENU_ATUAL ) {
            // SubMenu Tempo Motor
            case 0:
              EEPROM.update(CISTER_TIME_ADDR, CISTER_TIME);
              IN_SUBMENU_SELECTION = false;
              break;
            // SubMenu Tempo Solenoide
            case 1:
              EEPROM.update(CONCES_TIME_ADDR, CONCES_TIME);
              IN_SUBMENU_SELECTION = false;
              break;
            case 2:
              IN_SUBMENU_SELECTION = false;
              break;
          }
          break;
        case 7:
          switch ( SUBMENU_ATUAL ) {
            // SubMenu Tempo Motor
            case 0:
              EEPROM.update(BL_TIMEOUT_ADDR, LCD_BL_TIMEOUT);
              IN_SUBMENU_SELECTION = false;
              IN_SUBMENU = false;
              break;
          }
          break;
      }
    }
  }
}

void checkBacklight() {
  // Backlight
  if (LOOP_TIME - LAST_ITERATION_TIME > (LCD_BL_TIMEOUT*MINUTE)) {
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
  if ((CISTER_FLOW_STATUS == ST_IDLE) && validLevels()) {
    LAST_FLOW_MODE = FLOW_CISTER;
    EEPROM.update(LAST_FLOW_MODE_ADDR, FLOW_CISTER);
    CISTER_FLOW_STATUS = ST_RUNNING;
    REFRESH_STATUS_SCREEN = true;
    digitalWrite(CISTER_FLOW_OUT, LOW);
  }
}

void solenoidOn() {
  if ((CONCES_FLOW_STATUS == ST_IDLE) && validLevels()) {
    LAST_FLOW_MODE = FLOW_CONCES;
    EEPROM.update(LAST_FLOW_MODE_ADDR, FLOW_CONCES);
    CONCES_FLOW_STATUS = ST_RUNNING;
    REFRESH_STATUS_SCREEN = true;
    digitalWrite(CONCES_FLOW_OUT, LOW);
  }
}

void solenoidOff() {
  if (CONCES_FLOW_STATUS == ST_RUNNING) {
    CONCES_FLOW_STATUS = ST_IDLE;
    REFRESH_STATUS_SCREEN = true;
    EEPROM.put(CONCES_LAST_TIME_ADDR, TOTAL_TIME_CONCES_FLOW);
    digitalWrite(CONCES_FLOW_OUT, HIGH);
  }
}

void motorOff() {
  if (CISTER_FLOW_STATUS == ST_RUNNING) {
    CISTER_FLOW_STATUS = ST_IDLE;
    REFRESH_STATUS_SCREEN = true;
    EEPROM.put(CISTER_LAST_TIME_ADDR, TOTAL_TIME_CISTER_FLOW);
    digitalWrite(CISTER_FLOW_OUT, HIGH);
  }
}

void doSecurityCheck() {

  if (CISTER_FLOW_STATUS == ST_RUNNING) {
    TOTAL_TIME_CISTER_FLOW = LOOP_TIME - TIME_CISTER_FLOW;
    if (TOTAL_TIME_CISTER_FLOW > MAX_TIME_CISTER_FLOW) {
      CISTER_FLOW_ERROR = WITH_ERROR;
      EEPROM.update(CISTER_ERROR_ADDR, CISTER_FLOW_ERROR);
      motorOff();
      solenoidOff();
    }
  }

  if (CONCES_FLOW_STATUS == ST_RUNNING) {
    TOTAL_TIME_CONCES_FLOW = LOOP_TIME - TIME_CONCES_FLOW;
    if (TOTAL_TIME_CONCES_FLOW > MAX_TIME_CONCES_FLOW) {
      CONCES_FLOW_ERROR = WITH_ERROR;
      EEPROM.update(CONCES_ERROR_ADDR, CONCES_FLOW_ERROR);
      motorOff();
      solenoidOff();
    }
  }
}

void printStatusSkel() {
  lcd.setCursor(0, 0);
  lcd.print("Cx:  Ct:  ");
  if (MODO_OPERACAO == 0) {
    lcd.print("AUTO ");
  } else {
    lcd.print("MANU ");
  }
  lcd.write(byte(HEART));
  lcd.setCursor(0, 1);
  //         XXXXXXXXXXXXXXXX
  lcd.print("Off        --:--");
  if (CONCES_FLOW_ERROR == WITH_ERROR || CISTER_FLOW_ERROR == WITH_ERROR) {
    lcd.setCursor(0, 1);
    if (CONCES_FLOW_ERROR == WITH_ERROR) {
      lcd.print("ERRO-SL");
    } else {
      lcd.print("ERRO-MT");
    }
  }
}

void printMenu9(){
  SUBMENU_SIZE = 0;
  lcd.setCursor(0,0);
  //         1234567890123456
  lcd.print("<     Sair     >");
  lcd.setCursor(0,1);
  lcd.print("                ");
}


void printMenu8(){
  SUBMENU_SIZE = 0;
  lcd.setCursor(0,0);
  lcd.print("<    UpTime    >");
  lcd.setCursor(0,1);
  lcd.print(" ");
  if (IN_SUBMENU) {
    // Joga direto para a selecao rotativa, visto que nao há outros menus
    lcd.setCursor(0,0);
    lcd.print("#    UpTime    #");
    lcd.setCursor(0,1);
    lcd.print("    ");
    printTime(LOOP_TIME, true);
    lcd.setCursor(1,1);
    lcd.print(LOOP_TIME_CYCLES);
  } else {
    lcd.setCursor(0,1);
    lcd.print("    ");
    printTime(LOOP_TIME, true);
    lcd.setCursor(1,1);
    lcd.print(LOOP_TIME_CYCLES);
  }
  
}

void printMenu7(){
  SUBMENU_SIZE = 0;
  lcd.setCursor(0,0);
  lcd.print("< Tempo LED BL >");
  lcd.setCursor(0,1);
  lcd.print(" ");
  if (IN_SUBMENU) {
    // Joga direto para a selecao rotativa, visto que nao há outros menus
    IN_SUBMENU_SELECTION = true;
    lcd.setCursor(0,0);
    lcd.print("# Tempo LED BL #");
    lcd.setCursor(0,1);
    lcd.write(126);
  } else {
    lcd.setCursor(0,1);
    lcd.print("    ");
    lcd.print(LCD_BL_TIMEOUT);
    //         1234567890123456
    lcd.print(" Minutos   ");
  }
  
}

void printMenu6(){
  SUBMENU_SIZE = 2;
  lcd.setCursor(0,0);
  //         1234567890123456
  lcd.print("<  Seguranca   >");
  if (IN_SUBMENU) {
    lcd.setCursor(0,0);
    lcd.print("#  Seguranca   #");
    lcd.setCursor(0,1);
    lcd.print(" Motor  Sole.  ");
    lcd.write(byte(VOLTAR));

    switch (SUBMENU_ATUAL) {
      case 0:
        if (IN_SUBMENU_SELECTION) {
          lcd.setCursor(0,0);
          lcd.print("Tempo Max Motor:");
          lcd.setCursor(0,1);
          lcd.print("     ");
          printTime(MAX_TIME_CISTER_FLOW);
          lcd.print("      ");
        } else {
          lcd.setCursor(0,1);
          lcd.write(126);
          lcd.setCursor(7,1);
          lcd.print(" ");
          lcd.setCursor(14,1);
          lcd.print(" ");
        }
        break;
      case 1:
        if (IN_SUBMENU_SELECTION) {
          lcd.setCursor(0,0);
          lcd.print("Tempo Max Solen:");
          lcd.setCursor(0,1);
          lcd.print("     ");
          printTime(MAX_TIME_CONCES_FLOW);
          lcd.print("      ");
        } else {
          lcd.setCursor(0,1);
          lcd.print(" ");
          lcd.setCursor(7,1);
          lcd.write(126);
          lcd.setCursor(14,1);
          lcd.print(" ");
        }
        break;
      case 2:
        lcd.setCursor(0,1);
        lcd.print(" ");
        lcd.setCursor(7,1);
        lcd.print(" ");
        lcd.setCursor(14,1);
        lcd.write(126);
        break;
    }
  } else {
    lcd.setCursor(0,1);
    lcd.print("M:");
    printTime(MAX_TIME_CISTER_FLOW);
    lcd.print("  S:");
    printTime(MAX_TIME_CONCES_FLOW);
  }
}

void printMenu5(){
  SUBMENU_SIZE = 3;
  lcd.setCursor(0,0);
  lcd.print("< Nivel Partida>");
  if (IN_SUBMENU) {
    lcd.setCursor(0,0);
    lcd.print("# Nivel Partida#");
    lcd.setCursor(0,1);
    lcd.print("  ");
    lcd.write(byte(LEVEL_EMPTY));
    lcd.print("    ");
    lcd.write(byte(LEVEL_LOW));
    lcd.print("    ");
    lcd.write(byte(LEVEL_MID));
    lcd.print("  ");
    lcd.write(byte(VOLTAR));

    if (START_LEVEL == LEVEL_EMPTY) {
      lcd.setCursor(1,1);
      lcd.print("*");
      lcd.setCursor(6,1);
      lcd.print(" ");
      lcd.setCursor(11,1);
      lcd.print(" ");
    } else if (START_LEVEL == LEVEL_LOW){
      lcd.setCursor(1,1);
      lcd.print(" ");
      lcd.setCursor(6,1);
      lcd.print("*");
      lcd.setCursor(11,1);
      lcd.print(" ");
    } else if (START_LEVEL == LEVEL_MID){
      lcd.setCursor(1,1);
      lcd.print(" ");
      lcd.setCursor(6,1);
      lcd.print(" ");
      lcd.setCursor(11,1);
      lcd.print("*");
    }


    switch (SUBMENU_ATUAL) {
      case 0:
        lcd.setCursor(0,1);
        lcd.write(126);
        lcd.setCursor(5,1);
        lcd.print(" ");
        lcd.setCursor(10,1);
        lcd.print(" ");
        lcd.setCursor(14,1);
        lcd.print(" ");
        break;
      case 1:
        lcd.setCursor(0,1);
        lcd.print(" ");
        lcd.setCursor(5,1);
        lcd.write(126);
        lcd.setCursor(10,1);
        lcd.print(" ");
        lcd.setCursor(14,1);
        lcd.print(" ");
        break;
      case 2:
        lcd.setCursor(0,1);
        lcd.print(" ");
        lcd.setCursor(5,1);
        lcd.print(" ");
        lcd.setCursor(10,1);
        lcd.write(126);
        lcd.setCursor(14,1);
        lcd.print(" ");
        break;
      case 3:
        lcd.setCursor(0,1);
        lcd.print(" ");
        lcd.setCursor(5,1);
        lcd.print(" ");
        lcd.setCursor(10,1);
        lcd.print(" ");
        lcd.setCursor(14,1);
        lcd.write(126);
        break;
    }
  } else {
    lcd.setCursor(0,1);
    if (START_LEVEL == LEVEL_EMPTY) {
      lcd.print("    ");
      lcd.write(byte(LEVEL_EMPTY));
      lcd.write(" Vazio     ");
    } else if (START_LEVEL == LEVEL_LOW) {
      lcd.print("    ");
      lcd.write(byte(LEVEL_LOW));
      lcd.write(" Baixo     ");
    } else if (START_LEVEL == LEVEL_MID) {
      lcd.print("    ");
      lcd.write(byte(LEVEL_MID));
      lcd.write(" Medio     ");
    } else {
      lcd.print("        -       ");
    }
  }
}

void printMenu4(){
  SUBMENU_SIZE = 1;
  lcd.setCursor(0,0);
  lcd.print("<  Erro Oper.  >");
  if (IN_SUBMENU) {
    lcd.setCursor(0,0);
    lcd.print("#  Erro Oper.  #");
    lcd.setCursor(0,1);
    lcd.print("Reset  SIM  NAO");
    switch (SUBMENU_ATUAL) {
      case 0:
        lcd.setCursor(6,1);
        lcd.write(126);
        lcd.setCursor(11,1);
        lcd.print(" ");
        break;
      case 1:
        lcd.setCursor(6,1);
        lcd.print(" ");
        lcd.setCursor(11,1);
        lcd.write(126);
        break;
    }
  } else {
    lcd.setCursor(0,1);
    if (CISTER_FLOW_ERROR == NO_ERROR && CONCES_FLOW_ERROR == NO_ERROR) {
      lcd.print("    SEM ERRO    ");
    }

    if (CISTER_FLOW_ERROR == WITH_ERROR) {
      lcd.print("SIM: Tempo Motor");
    } else if (CONCES_FLOW_ERROR == WITH_ERROR) {
      lcd.print("SIM: Tempo Solen");
    }
    if (CISTER_FLOW_ERROR == WITH_ERROR && CONCES_FLOW_ERROR == WITH_ERROR) {
      lcd.print("SIM: Motor&Solen");
    }
  }
}
void printMenu3(){
  SUBMENU_SIZE = 0;
  lcd.setCursor(0,0);
  //         1234567890123456
  lcd.print("<     DEBUG    >");
  if (IN_SUBMENU) {
    lcd.setCursor(0,1);
    lcd.print("M:     S:       ");
    lcd.setCursor(0,0);
    lcd.print("CX:      CT:    ");

    char buffer[4];
    sprintf(buffer, "%04d", CISTER_COUNTER);
    lcd.setCursor(2,1);
    lcd.print(buffer);
    sprintf(buffer, "%04d", CONCES_COUNTER);
    lcd.setCursor(9,1);
    lcd.print(buffer);

    lcd.setCursor(3,0);
    lcd.print(analogRead(RESERV_SENS));
    lcd.setCursor(12,0);
    lcd.print(analogRead(CISTER_SENS));
    lcd.setCursor(14,1);
    lcd.write(126);
    lcd.write(byte(VOLTAR));
  } else {
    lcd.setCursor(0,1);
    //         1234567890123456
    lcd.print("CX:      CT:    ");
    lcd.setCursor(3,1);
    lcd.print(analogRead(RESERV_SENS));
    lcd.setCursor(12,1);
    lcd.print(analogRead(CISTER_SENS));
  }
}
void printMenu2(){
  SUBMENU_SIZE = 2;
  lcd.setCursor(0,0);
  lcd.print("<   Solenoide  >");
  if (IN_SUBMENU) {
    lcd.setCursor(0,0);
    lcd.print("#   Solenoide  #");
    lcd.setCursor(0,1);
    lcd.print("  ON     OFF   ");
    lcd.write(byte(VOLTAR));
    if (CONCES_FLOW_STATUS == ST_RUNNING) {
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
        //lcd.print(">");
        lcd.write(126);
        lcd.setCursor(7,1);
        lcd.print(" ");
        lcd.setCursor(14,1);
        lcd.print(" ");
        break;
      case 1:
        lcd.setCursor(0,1);
        lcd.print(" ");
        lcd.setCursor(7,1);
        lcd.write(126);
        lcd.setCursor(14,1);
        lcd.print(" ");
        break;
      case 2:
        lcd.setCursor(0,1);
        lcd.print(" ");
        lcd.setCursor(6,1);
        lcd.print(" ");
        lcd.setCursor(14,1);
        lcd.write(126);
        break;
    }
  } else {
    lcd.setCursor(0,1);
                                 //         1234567890123456
    if (CONCES_FLOW_STATUS == ST_RUNNING) lcd.print("     Ligado     ");
    //         1234567890123456
    lcd.print("    Desligado   ");
  }
}
void printMenu1() {
  SUBMENU_SIZE = 2;
  lcd.setCursor(0,0);
  lcd.print("<     Motor    >");
  if (IN_SUBMENU) {
    lcd.setCursor(0,0);
    lcd.print("#     Motor    #");

    lcd.setCursor(0,1);
    //         1234567890123456
    lcd.print("  ON     OFF   ");
    lcd.write(byte(VOLTAR));
    if (CISTER_FLOW_STATUS == ST_RUNNING) {
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
        //lcd.print(">");
        lcd.write(126);
        lcd.setCursor(7,1);
        lcd.print(" ");
        lcd.setCursor(14,1);
        lcd.print(" ");
        break;
      case 1:
        lcd.setCursor(0,1);
        lcd.print(" ");
        lcd.setCursor(7,1);
        lcd.write(126);
        lcd.setCursor(14,1);
        lcd.print(" ");
        break;
      case 2:
        lcd.setCursor(0,1);
        lcd.print(" ");
        lcd.setCursor(6,1);
        lcd.print(" ");
        lcd.setCursor(14,1);
        lcd.write(126);
        break;
    }
  } else {
    lcd.setCursor(0,1);
                                 //         1234567890123456
    if (CISTER_FLOW_STATUS == ST_RUNNING) lcd.print("     Ligado     ");
    //         1234567890123456
    lcd.print("    Desligado   ");
  }
}

void printMenu0() {
  SUBMENU_SIZE = 2;
  lcd.setCursor(0,0);
  //         1234567890123456
  lcd.print("<  Modo Oper.  >");
  if (IN_SUBMENU) {
    lcd.setCursor(0,0);
    lcd.print("#  Modo Oper.  #");
    lcd.setCursor(0,1);
    //         1234567890123456
    lcd.print("  AUTO   MANU  ");
    lcd.write(byte(VOLTAR));
    if (MODO_OPERACAO == 0) {
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
        //lcd.print(">");
        lcd.write(126);
        lcd.setCursor(7,1);
        lcd.print(" ");
        lcd.setCursor(14,1);
        lcd.print(" ");
        break;
      case 1:
        lcd.setCursor(0,1);
        lcd.print(" ");
        lcd.setCursor(7,1);
        lcd.write(126);
        lcd.setCursor(14,1);
        lcd.print(" ");
        break;
      case 2:
        lcd.setCursor(0,1);
        lcd.print(" ");
        lcd.setCursor(6,1);
        lcd.print(" ");
        lcd.setCursor(14,1);
        lcd.write(126);
        break;
    }
  } else {
    lcd.setCursor(0,1);
    if (MODO_OPERACAO == 0) lcd.print("   Automatico   "); 
    else lcd.print("     Manual     ");
  }
}

void updateLCD() {
  if (!IN_MENU) {
    statusScreen();
  } else {
    if (DO_MENU_DRAW) {
      switch (MENU_ATUAL) {
        // Modo Operacao
        case 0:
          printMenu0();
          break;
        // Estado Motor
        case 1:
          printMenu1();
          break;
        // Estado Solenoide
        case 2:
          printMenu2();
          break;
        // Debug
        case 3:
          printMenu3();
          break;
        // Erro Operacao
        case 4:
          printMenu4();
          break;
        // Nivel Partida
        case 5:
          printMenu5();
          break;
        // Seguranca
        case 6:
          printMenu6();
          break;
        // Timeout Backlight LCD
        case 7:
          printMenu7();
          break;
        // Uptime
        case 8:
          printMenu8();
          break;
        // Sair
        case 9:
          printMenu9();
          break;
      }
    }

  }
  DO_MENU_DRAW = false;
}

void statusScreen() {
  if (REFRESH_STATUS_SCREEN) {
    char buffer[4];
    lcd.clear();
    printStatusSkel();
    lcd.setCursor(4,1);
    if (LAST_FLOW_MODE == FLOW_CISTER) {
      sprintf(buffer, "%04d", CISTER_COUNTER);
      lcd.print(buffer);
      lcd.print(" M ");
      printTime(TOTAL_TIME_CISTER_FLOW);
    } else if (LAST_FLOW_MODE == FLOW_CONCES) {
      sprintf(buffer, "%04d", CONCES_COUNTER);
      lcd.print(buffer);
      lcd.print(" S ");
      printTime(TOTAL_TIME_CONCES_FLOW);
    } else {
      lcd.print("0000 - ");
    }
    REFRESH_STATUS_SCREEN = false;
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
    case LEVEL_EMPTY:
      lcd.write(byte(LEVEL_EMPTY));
      break;
    case LEVEL_LOW:
      lcd.write(byte(LEVEL_LOW));
      break;
    case LEVEL_MID:
      lcd.write(byte(LEVEL_MID));
      break;
    case LEVEL_FULL:
      lcd.write(byte(LEVEL_FULL));
      break;
  }

  if (CONCES_FLOW_STATUS == ST_RUNNING) {
    lcd.setCursor(0, 1);
              
    lcd.print("Solen. On  ");
    printTime(TOTAL_TIME_CONCES_FLOW);
  }
  if (CISTER_FLOW_STATUS == ST_RUNNING) {
    lcd.setCursor(0, 1);
    lcd.print("Motor On   ");
    printTime(TOTAL_TIME_CISTER_FLOW);
  }
}

void readCisternStatus() {
  int SENSOR_VALUE = analogRead(CISTER_SENS);
  int LEVEL_READ = LEVEL_EMPTY;
  // CHEIA ~1023
  if (SENSOR_VALUE > 800) {
    LEVEL_READ = LEVEL_FULL;
  } else if (SENSOR_VALUE < 700 && SENSOR_VALUE > 450) {
    // MEIA ~508
    LEVEL_READ = LEVEL_MID;
  } else if (SENSOR_VALUE < 450 && SENSOR_VALUE > 250) {
    // BAIXA ~337
    LEVEL_READ = LEVEL_LOW;
  } else {
    // VAZIA ~0;
    LEVEL_READ = LEVEL_EMPTY;
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
    if (CONCES_FLOW_STATUS == ST_IDLE) {
      CISTER_EMPTY_STATE = false;
    }
  }
}

void readReservatoryStatus() {
  int SENSOR_VALUE = analogRead(RESERV_SENS);
  int LEVEL_READ = LEVEL_EMPTY;
  // CHEIA ~1023
  if (SENSOR_VALUE > 800) {
    LEVEL_READ = LEVEL_FULL;
  } else if (SENSOR_VALUE < 800 && SENSOR_VALUE > 565) {
    // MEIA ~613
    LEVEL_READ = LEVEL_MID;
  } else if (SENSOR_VALUE < 565 && SENSOR_VALUE > 400) {
    // BAIXA ~517
    LEVEL_READ = LEVEL_LOW;
  } else {
    // VAZIA ~0;
    LEVEL_READ = LEVEL_EMPTY;
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
  } else if (RESERV_LEVEL == LEVEL_FULL ) {
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
  } else {
    if (IN_SUBMENU && (MENU_ATUAL == 3 || MENU_ATUAL == 8)) {
      // caso esteja no menu de debug, atualiza a tela a cada 1500ms
      if (LOOP_TIME - HEART_BEAT_LAST >= 1500) {
        DO_MENU_DRAW = true;
        HEART_BEAT_LAST = LOOP_TIME;
      }
    }
  }
}

void printTime(unsigned long t_milli)
{
  printTime(t_milli, false);
}

// argument is time in milliseconds
void printTime(unsigned long t_milli, bool print_days)
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
    if (print_days) {
      // Don't bother to print days
      sprintf(buffer, "%02d %02d:%02d:%02d", days, hours, mins, secs);
      lcd.print(buffer);
    } else {
      // Don't bother to print days
      sprintf(buffer, "%02d:%02d", mins, secs);
      lcd.print(buffer);
    }
}