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

#define START_LEVEL_ADDR 2

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

#define MAX_TIME_CISTER_FLOW 10 * MINUTE // DEFAULT 5 MINUTOS
#define MAX_TIME_CONCES_FLOW 10 * MINUTE // DEFAULT 5 MINUTOS

#define STARTUP_TIME 3 * SECOND
#define SENSOR_DB_TIME 3 * SECOND

// Define o Nivel que a caixa deve estar para acionar motor ou solenoide
// 0 -> iniciar quando vazio
// 1 -> iniciar quando nivel baixo
// 2 -> iniciar quando nivel medio

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

bool FLOW_ENABLED = false;

unsigned long TIME_CISTER_FLOW = 0;
unsigned long TIME_CONCES_FLOW = 0;

unsigned long TOTAL_TIME_CISTER_FLOW = 0;
unsigned long TOTAL_TIME_CONCES_FLOW = 0;

unsigned long LOOP_TIME = 0;

int LAST_FLOW_MODE = -1; // Flow Mode: CONCES/CISTER

bool REFRESH_STATUS_SCREEN = false;
int LCD_BL_STATE = 1; // 1 = Ligado, 0 = Desligado
unsigned long LCD_BL_TIMEOUT = 3 * MINUTE; // Tempo LCD Ligado
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

bool DO_SUBMENU_ACTION = false;

int MENU_SIZE = 6;
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

  if (START_LEVEL > LEVEL_MID) {
    EEPROM.update(START_LEVEL_ADDR, LEVEL_EMPTY);
    START_LEVEL = LEVEL_EMPTY;
  }
  

  //------------lcd-----------

  lcd.begin(16, 2);
  lcd.clear();
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
  if (MODO_OPERACAO == 0) controleDeFluxo();
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
      result = 1;
      if (direction == 1) {
        if (SUBMENU_ATUAL == SUBMENU_SIZE) SUBMENU_ATUAL = 0;
        else if (SUBMENU_ATUAL < SUBMENU_SIZE) SUBMENU_ATUAL++;
      } else {
        if (SUBMENU_ATUAL > 0) SUBMENU_ATUAL--;
        else SUBMENU_ATUAL = SUBMENU_SIZE;
      }

      // Se submen de debug, dispara draw
      if (MENU_ATUAL == 3) DO_MENU_DRAW = true;

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
      if (MENU_ATUAL == 6) {
        REFRESH_STATUS_SCREEN = true;
        IN_MENU = false;
      } else {
        SUBMENU_ATUAL = 0;
        IN_SUBMENU = true;
      }
    } else {
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
              if (CISTER_FLOW_STATUS == 0) {
                TIME_CISTER_FLOW = LOOP_TIME;
                TOTAL_TIME_CISTER_FLOW = 0;
                motorOn();
              }
              DO_MENU_DRAW = true;
              break;
            case 1:
              if (CISTER_FLOW_STATUS == 1) motorOff();
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
              if (CONCES_FLOW_STATUS == 0) {
                TIME_CONCES_FLOW = LOOP_TIME;
                TOTAL_TIME_CONCES_FLOW = 0;
                solenoidOn();
              }
              DO_MENU_DRAW = true;
              break;
            case 1:
              if (CONCES_FLOW_STATUS == 1) solenoidOff();
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
              CONCES_FLOW_ERROR = 0;
              CISTER_FLOW_ERROR = 0;
              IN_SUBMENU = false;
              break;
            case 1:
              IN_SUBMENU = false;
              break;
          }
          break;
        case 5:
          switch (SUBMENU_ATUAL) {
            case 0:
              START_LEVEL = 0;
              break;
            case 1:
              START_LEVEL = 1;
              break;
            case 2:
              START_LEVEL = 2;
              break;
            case 3:
              EEPROM.update(START_LEVEL_ADDR, START_LEVEL);
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
    REFRESH_STATUS_SCREEN = true;
  }
}

void solenoidOff() {
  if (CONCES_FLOW_STATUS == 1) {
    digitalWrite(CONCES_FLOW_OUT, HIGH);
    CONCES_FLOW_STATUS = 0;
    REFRESH_STATUS_SCREEN = true;
  }
}

void motorOff() {
  if (CISTER_FLOW_STATUS == 1) {
    digitalWrite(CISTER_FLOW_OUT, HIGH);
    CISTER_FLOW_STATUS = 0;
    REFRESH_STATUS_SCREEN = true;
  }
}

void solenoidOn() {
  if (CONCES_FLOW_STATUS == 0) {
    digitalWrite(CONCES_FLOW_OUT, LOW);
    CONCES_FLOW_STATUS = 1;
    REFRESH_STATUS_SCREEN = true;
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

void printMenu6(){
  SUBMENU_SIZE = 0;
  lcd.setCursor(0,0);
  //         1234567890123456
  lcd.print("<     Sair     >");
  lcd.setCursor(0,1);
  lcd.print("                ");
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

    if (START_LEVEL == 0) {
      lcd.setCursor(1,1);
      lcd.print("*");
      lcd.setCursor(6,1);
      lcd.print(" ");
      lcd.setCursor(11,1);
      lcd.print(" ");
    } else if (START_LEVEL == 1){
      lcd.setCursor(1,1);
      lcd.print(" ");
      lcd.setCursor(6,1);
      lcd.print("*");
      lcd.setCursor(11,1);
      lcd.print(" ");
    } else if (START_LEVEL == 2){
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
    if (CISTER_FLOW_ERROR == 0 && CONCES_FLOW_ERROR == 0) {
      lcd.print("    SEM ERRO    ");
    }

    if (CISTER_FLOW_ERROR == 1) {
      lcd.print("SIM: Tempo Motor");
    } else if (CONCES_FLOW_ERROR == 1) {
      lcd.print("SIM: Tempo Solen");
    }
    if (CISTER_FLOW_ERROR == 1 && CONCES_FLOW_ERROR == 1) {
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
    lcd.print("                ");
    lcd.setCursor(0,0);
    lcd.print("CX:   CT:  DEBUG");
  // Atualizas Nivel Cisterna
    lcd.setCursor(9, 0);
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
    lcd.setCursor(0,1);
    //         1234567890123456
    lcd.setCursor(0,1);
    lcd.print(analogRead(RESERV_SENS));
    lcd.setCursor(6,1);
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
    if (CONCES_FLOW_STATUS == 1) {
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
    if (CISTER_FLOW_STATUS == 1) lcd.print("     Ligado     ");
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
    if (CISTER_FLOW_STATUS == 1) lcd.print("     Ligado     ");
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
        // Sair
        case 6:
          printMenu6();
          break;
      }
    }

  }
  DO_MENU_DRAW = false;
}

void statusScreen() {
  if (REFRESH_STATUS_SCREEN) {
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
  } else {
    if (IN_SUBMENU && MENU_ATUAL == 3) {
      // caso esteja no menu de debug, atualiza a tela a cada 1500ms
      if (LOOP_TIME - HEART_BEAT_LAST >= 1500) {
        DO_MENU_DRAW = true;
        HEART_BEAT_LAST = LOOP_TIME;
      }
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
