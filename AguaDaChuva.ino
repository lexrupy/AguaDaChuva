

#define CISTER_FLOAT 4
#define RESERV_FLOAT 5

#define CISTER_LOW_LED 12
#define RESERV_LOW_LED 11
#define CISTER_FLOW_LED 10
#define CONCES_FLOW_LED 9

int CISTER_STATUS = 1;
int RESERV_STATUS = 1;
int CISTER_FLOW_STATUS = 0;
int CONCES_FLOW_STATUS = 0;

void setup() {
  pinMode(CISTER_FLOAT, INPUT);
  pinMode(RESERV_FLOAT, INPUT);
  pinMode(CISTER_LOW_LED, OUTPUT);
  pinMode(RESERV_LOW_LED, OUTPUT);
  pinMode(CISTER_FLOW_LED, OUTPUT);
  pinMode(CONCES_FLOW_LED, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  CISTER_STATUS = digitalRead(CISTER_FLOAT);
  RESERV_STATUS = digitalRead(RESERV_FLOAT);

//  digitalWrite(CISTER_LOW_LED, !CISTER_STATUS);
//  digitalWrite(RESERV_LOW_LED, !RESERV_STATUS);

  if (RESERV_STATUS == LOW) {
    if (CISTER_STATUS == HIGH) {
      CONCES_FLOW_STATUS = 0;
      CISTER_FLOW_STATUS = 1;
    } else {
      CISTER_FLOW_STATUS = 0;
      CONCES_FLOW_STATUS = 1;
    }
  } else {
    CISTER_FLOW_STATUS = 0;
    CONCES_FLOW_STATUS = 0;
  }

  
  digitalWrite(CISTER_LOW_LED, !CISTER_STATUS);
  digitalWrite(RESERV_LOW_LED, !RESERV_STATUS);

  digitalWrite(CISTER_FLOW_LED, CISTER_FLOW_STATUS);
  digitalWrite(CONCES_FLOW_LED, CONCES_FLOW_STATUS);

  
  Serial.print("CISTERNA:");
  Serial.print(CISTER_STATUS);
  Serial.print(" CAIXA:");
  Serial.print(RESERV_STATUS);
  Serial.print(" BOMBA:");
  Serial.print(CISTER_FLOW_STATUS);
  Serial.print(" CASAN:");
  Serial.println(CONCES_FLOW_STATUS);
}
