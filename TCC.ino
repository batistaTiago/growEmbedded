#include "virtuabotixRTC.h"
#include "DHT.h"
#include <PID_v1.h>
#include <EEPROM.h>

#define ledVermelho 13
#define ledVerde 12
#define ledAzul 11
#define luzPin 10
#define bombaPin 9
#define RTC_CLK 8
#define RTC_DAT 7
#define RTC_RST 6

#define HL69Pin A0
#define dht11 A1
#define LDRPin A2
#define coolerPin 3

#define limiarSuperiorSimuladorUmidade 0
#define limiarInferiorSimuladorUmidade 1000

#define limiarSuperiorSimuladorTemperatura 15
#define limiarInferiorSimuladorTemperatura 40

#define ROMAddressCoolerONDoW 0
#define ROMAddressCoolerOFFDoW 1
#define ROMAddressCoolerONHours 2
#define ROMAddressCoolerOFFHours 3
#define ROMAddressCoolerONMinutes 4
#define ROMAddressCoolerOFFMinutes 5
#define ROMAddressCoolerONSeconds 6
#define ROMAddressCoolerOFFSeconds 7


#define ROMAddressPumpONDoW 8
#define ROMAddressPumpOFFDoW 9
#define ROMAddressPumpONHours 10
#define ROMAddressPumpOFFHours 11
#define ROMAddressPumpONMinutes 12
#define ROMAddressPumpOFFMinutes 13
#define ROMAddressPumpONSeconds 14
#define ROMAddressPumpOFFSeconds 15


#define ROMAddressLightONDoW 16
#define ROMAddressLightOFFDoW 17
#define ROMAddressLightONHours 18
#define ROMAddressLightOFFHours 19
#define ROMAddressLightONMinutes 20
#define ROMAddressLightOFFMinutes 21
#define ROMAddressLightONSeconds 22
#define ROMAddressLightOFFSeconds 23


#define ROMAddressPIDSetpoint 24
#define ROMAddressPIDKp 25
#define ROMAddressPIDKi 26
#define ROMAddressPIDKd 27

#define ROMAddressPIDUmidadeMin 28
#define ROMAddressPIDUmidadeMax 29

#define ROMAddressOpMode 30

struct ActiveTime {
  int dayOfWeek = -1;
  int seconds = -1;
  int minutes = -1;
  int hours = -1;
};


int coolerProgPower = 128;
int MODE, UMIDADE, UMIDADE_MIN, UMIDADE_MAX, UMIDADE_AR, LUMINOSIDADE, leitura, i, inChar;
float umidadeNormalizada, umidadeMinNormalizada, umidadeMaxNormalizada;
ActiveTime pumpON, pumpOFF, lightON, lightOFF, coolerON, coolerOFF;
char c;
String comando;
float TEMP;
bool bombaOn, luzOn;
double tempControllerSetpoint, tempControllerInput, tempControllerOutput;
double tempControllerKp = 1;
double tempControllerKi = 0.05;
double tempControllerKd = 0.5;
int coolerPower = 0;

PID tempController(&tempControllerInput, &tempControllerOutput, &tempControllerSetpoint, 2, 5, 1, DIRECT); //PID::PID(double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd, int POn, int ControllerDirection)

DHT dht(dht11, DHT11);
virtuabotixRTC currentTime(RTC_CLK, RTC_DAT, RTC_RST);

bool hasMommentPassed(virtuabotixRTC cTime, ActiveTime refTime) {
  if (cTime.hours > refTime.hours) {
    return true;
  }
  else if ((cTime.hours == refTime.hours) && (cTime.minutes > refTime.minutes)) {
    return true;
  }
  else if ((cTime.hours == refTime.hours) && (cTime.minutes == refTime.minutes) && (cTime.seconds >= refTime.seconds)) {
    return true;
  }
  return false;
}

void hora_atual() {
  currentTime.updateTime();

  if (currentTime.hours < 10)
  {
    Serial.print("0");
  }
  Serial.print(currentTime.hours);
  Serial.print("/");
  if (currentTime.minutes < 10)
  {
    Serial.print("0");
  }
  Serial.print(currentTime.minutes);
  Serial.print("/");
  if (currentTime.seconds < 10)
  {
    Serial.print("0");
  }
  Serial.print(currentTime.seconds);
  Serial.print("/");
  Serial.print(currentTime.dayofweek);
}

bool checkValidTime(ActiveTime t) {
  if (t.seconds != -1) {
    if (t.minutes != -1) {
      if (t.hours != -1) {
        if (t.dayOfWeek != -1) {
          return true;
        } else {
          //Serial.println("Day Of Week invalido");
        }
      } else {
        //Serial.println("Horas invalido");
      }
    } else {
      //Serial.println("Minuto invalido");
    }
  } else {
    //Serial.println("Segundo invalido");
  }
  return false;
}

String split(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void erro1() {
  Serial.println("<<<<<<<<< ERRO: MODO MANUAL DESTAVIVADO >>>>>>>>>");
}

void erro2() {
  Serial.println("<<<<<<<<< ERRO: MODO NAO RECONHECIDO. >>>>>>>>>");
  Serial.println("<<<<<<<<< USE 24: AUTOMATICO, 25: MANUAL, 26: PROGRAMADO, 27: SIMULAÇÃO >>>>>>>>>");
}

void erro3() {
  Serial.println("<<<<<<<<< ERRO: OS VALORES DE UMIDADE N�O PODEM EXCEDER 1000 >>>>>>>>>");
}

void erro4() {
  Serial.println("<<<<<<<<< ERRO: O VALOR DE UMIDADE MINIMA DEVE SER MENOR QUE O DE UMIDADE MAXIMA >>>>>>>>>");
}

void erro5() {
  Serial.println("<<<<<<<<< ERRO: COMANDO DESCONHECIDO. UTILIZE O COMANDO H PARA AJUDA >>>>>>>>>");
}

void erro6() {
  Serial.println("<<<<<<<<< FORMATO INVALIDO. USE R:SEG:MIN:HORA:DOW:DATA:MES:ANO >>>>>>>>>");
}

//void ligaledVermelho() {
//  digitalWrite(ledVermelho, HIGH);
//  digitalWrite(ledVerde, LOW);
//  digitalWrite(ledAzul, LOW);
//}

//void ligaledVerde() {
//  digitalWrite(ledVerde, HIGH);
//  digitalWrite(ledVermelho, LOW);
//  digitalWrite(ledAzul, LOW);
//}
//
//void ligaledAzul() {
//  digitalWrite(ledAzul, HIGH);
//  digitalWrite(ledVermelho, LOW);
//  digitalWrite(ledVerde, LOW);
//}
//
//void ligaledRoxo() {
//  digitalWrite(ledVermelho, HIGH);
//  digitalWrite(ledVerde, LOW);
//  digitalWrite(ledAzul, HIGH);
//}
//
//void liga3led() {
//  digitalWrite(ledVermelho, HIGH);
//  digitalWrite(ledVerde, HIGH);
//  digitalWrite(ledAzul, HIGH);
//}
//
//void desliga3led() {
//  digitalWrite(ledVermelho, LOW);
//  digitalWrite(ledVerde, LOW);
//  digitalWrite(ledAzul, LOW);
//}
//
//void pisca3led(int t) {
//  liga3led();
//  delay(t);
//  desliga3led();
//  delay(t);
//}

void ligaBomba() {
//  Serial.println("tentando ligar bomba");
  if (digitalRead(bombaPin) == 0) {
//    Serial.println("ligando bomba");
//    currentTime.updateTime();
    digitalWrite(bombaPin, HIGH);
//    rtcNoiseSoftwareFilter(&currentTime);
    bombaOn = true;
  }
}

void desligaBomba() {
//  Serial.println("tentando desligar bomba");
  if (digitalRead(bombaPin) == 1) {
//    Serial.println("desligando bomba");
    hora_atual();
//    currentTime.updateTime();
    digitalWrite(bombaPin, LOW);
//    rtcNoiseSoftwareFilter(&currentTime);
    bombaOn = false;
  }
}

void ligaCooler(int potencia) {
  if ((potencia != 0) && (coolerPower == 0)) {
//    Serial.println("Dando partida!");
    hora_atual();
    analogWrite(coolerPin, 255);
    delay(1500);
  }
  
  if (potencia == 0) {
          desligaCooler();
      return;
  }
  int effectivePower = potencia;
  
  if (effectivePower > 255) {
    effectivePower = 255;
  } else if (effectivePower < 65) {
    effectivePower = 65;
  }
  
  analogWrite(coolerPin, effectivePower);
  coolerPower = effectivePower; 
//  Serial.print("ligando com potencia: ");
//  Serial.println(effectivePower);
}

//void ligaCooler(int potencia) {
//  if (potencia == 0) {
//          desligaCooler();
//      return;
//  }
//  int potenciaReal = potencia;
//  if (potenciaReal > 255) {
//    potenciaReal = 255;
//  }
//  Serial.print("ligando coolers com potencia de: ");
//  Serial.println(potenciaReal);
//  analogWrite(coolerPin, potenciaReal);
//  coolerPower = potenciaReal;
//}

void desligaCooler() {
  if (coolerPower > 0) {
//    Serial.println("desligando coolers");
    analogWrite(coolerPin, 0);
    coolerPower = 0;
  }
}

void ligaLampada() {
//  Serial.println("tentando ligar luz");
  if (digitalRead(luzPin) == 0) {
//    Serial.print("ligando luz em: ");

//    currentTime.updateTime();
    
    digitalWrite(luzPin, HIGH);
    
    luzOn = true;
  }
}

void desligaLampada() {
//  Serial.println("tentando desligar luz");
  if (digitalRead(luzPin) == 1) {
//    Serial.println("desligando luz");
    digitalWrite(luzPin, LOW);
    luzOn = false;
  }
}

void serialEvent() {
  while (Serial.available()) {
    c = Serial.read();
    if (c == '\n') {
      comandar(comando);
      //Serial.println(" Comando Recebido: " + comando); //Serial.print(comando); Serial.println();
      comando = "";
    }
    else {
      comando += char(c);
    }
  }
}

void comandar(String com) {

  String p0 = split(com, ':', 0);
  String p1 = split(com, ':', 1);
  String p2 = split(com, ':', 2);
  String p3 = split(com, ':', 3);
  String p4 = split(com, ':', 4);
  String p5 = split(com, ':', 5);
  String p6 = split(com, ':', 6);
  String p7 = split(com, ':', 7);
  String p8 = split(com, ':', 8);
  String p9 = split(com, ':', 9);
  String p10 = split(com, ':', 10);

  char OPCODE = p0[0];

  if (OPCODE == '?') {
    if (MODE != 27) {
      readSensors();
    }
    currentTime.updateTime();
    Serial.print("___");
    if (umidadeNormalizada != 0) {
      Serial.print(umidadeNormalizada);
    }
    else {
      Serial.print(0.1);
    }
    Serial.print("/");
    Serial.print(UMIDADE_AR);
    Serial.print("/");
    Serial.print(TEMP);
    Serial.print("/");
    Serial.print(LUMINOSIDADE);
    Serial.print("/");
    Serial.print(bombaOn ==  true);
    Serial.print("/");
    Serial.print(luzOn ==  true);
    Serial.print("/");
    Serial.print(coolerPower);
    Serial.print("/");
    Serial.print(MODE);
    Serial.print("/");
    Serial.print(umidadeMinNormalizada);
    Serial.print("/");
    Serial.print(umidadeMaxNormalizada);
    Serial.print("/");
    hora_atual();
    Serial.print("/");
    Serial.print(tempControllerSetpoint);
    Serial.print("/");
    Serial.print(tempControllerKp);
    Serial.print("/");
    Serial.print(tempControllerKi);
    Serial.print("/");
    Serial.print(tempControllerKd);
    Serial.println("&&&");
  }

  else if (OPCODE == '$') {
    Serial.print(lightON.dayOfWeek);
    Serial.print("/");
    Serial.print(lightOFF.dayOfWeek);
    Serial.print("/");

    Serial.print(lightON.hours);
    Serial.print("/");
    Serial.print(lightON.minutes);
    Serial.print("/");
    Serial.print(lightON.seconds);
    Serial.print("/");
    Serial.print(lightOFF.hours);
    Serial.print("/");
    Serial.print(lightOFF.minutes);
    Serial.print("/");
    Serial.println(lightOFF.seconds);
  }

  else if (OPCODE == '#') {
    Serial.print(pumpON.dayOfWeek);
    Serial.print("/");
    Serial.print(pumpOFF.dayOfWeek);
    Serial.print("/");

    Serial.print(pumpON.hours);
    Serial.print("/");
    Serial.print(pumpON.minutes);
    Serial.print("/");
    Serial.print(pumpON.seconds);
    Serial.print("/");
    Serial.print(pumpOFF.hours);
    Serial.print("/");
    Serial.print(pumpOFF.minutes);
    Serial.print("/");
    Serial.println(pumpOFF.seconds);
  }

  else if (OPCODE == '@') {
    Serial.print(coolerON.dayOfWeek);
    Serial.print("/");
    Serial.print(coolerOFF.dayOfWeek);
    Serial.print("/");
    
    Serial.print(coolerON.hours);
    Serial.print("/");
    Serial.print(coolerON.minutes);
    Serial.print("/");
    Serial.print(coolerON.seconds);
    Serial.print("/");
    Serial.print(coolerOFF.hours);
    Serial.print("/");
    Serial.print(coolerOFF.minutes);
    Serial.print("/");
    Serial.println(coolerOFF.seconds);
  }

  else if (OPCODE == 'M') {

    //reconhecimento do modo de operação
    if ((p1.toInt() == 24) || (p1.toInt() == 25) || (p1.toInt() == 26) || (p1.toInt() == 27)) {

      //Serial.print("p1 = "); Serial.println(p1.toInt());
      MODE = p1.toInt();
      EEPROM.write(ROMAddressOpMode, MODE);
//      Serial.println("ok");
      desligaBomba();
      desligaLampada();
      desligaCooler();
      //Serial.print("Modo setado para: "); Serial.println(MODE);
    }
    else {
      erro2();
    }

    if ((p1.toInt() == 24) && (p2.toInt() > 0) && (p3.toInt() > 0)) {
      if ((UMIDADE_MAX >= 1023) || (UMIDADE_MIN >= 1023)) {
        erro3();
      }
      else if (p3.toInt() <= p2.toInt()) {
        erro4();
      }
      else {
        UMIDADE_MIN = p2.toInt();
        UMIDADE_MAX = p3.toInt();
        EEPROM.write(ROMAddressPIDUmidadeMin, UMIDADE_MIN);
        EEPROM.write(ROMAddressPIDUmidadeMax, UMIDADE_MAX);

//PID::PID(double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd, int POn, int ControllerDirection)
        tempControllerSetpoint = p4.toDouble();
        tempControllerKp = p5.toDouble();
        tempControllerKi = p6.toDouble();
        tempControllerKd = p7.toDouble();
        EEPROM.write(ROMAddressPIDSetpoint, tempControllerSetpoint);
        EEPROM.write(ROMAddressPIDKp, tempControllerKp);
        EEPROM.write(ROMAddressPIDKi, tempControllerKi);
        EEPROM.write(ROMAddressPIDKd, tempControllerKd);
        //Serial.println(p2.toInt());
        //Serial.println(p3.toInt());
      }
    }

    if ((p1.toInt() == 26) && (p2[0] == '#')) {
//      Serial.println("detectado");
      pumpON.dayOfWeek = p3.toInt();
      pumpOFF.dayOfWeek = p4.toInt();

      pumpON.hours = p5.toInt();
      pumpON.minutes = p6.toInt();
      pumpON.seconds = p7.toInt();

      pumpOFF.hours = p8.toInt();
      pumpOFF.minutes = p9.toInt();
      pumpOFF.seconds = p10.toInt();

      EEPROM.write(ROMAddressPumpONDoW, pumpON.dayOfWeek);
      EEPROM.write(ROMAddressPumpOFFDoW, pumpOFF.dayOfWeek);

      EEPROM.write(ROMAddressPumpONHours, pumpON.hours);
      EEPROM.write(ROMAddressPumpONMinutes, pumpON.minutes);
      EEPROM.write(ROMAddressPumpONSeconds, pumpON.seconds);

      EEPROM.write(ROMAddressPumpOFFHours, pumpOFF.hours);
      EEPROM.write(ROMAddressPumpOFFMinutes, pumpOFF.minutes);
      EEPROM.write(ROMAddressPumpOFFSeconds, pumpOFF.seconds);
    }

    if ((p1.toInt() == 26) && (p2[0] == '$')) {
//      Serial.println("detectado");
      lightON.dayOfWeek = p3.toInt();
      lightOFF.dayOfWeek = p4.toInt();

      lightON.hours = p5.toInt();
      lightON.minutes = p6.toInt();
      lightON.seconds = p7.toInt();

      lightOFF.hours = p8.toInt();
      lightOFF.minutes = p9.toInt();
      lightOFF.seconds = p10.toInt();

      EEPROM.write(ROMAddressLightONDoW, lightON.dayOfWeek);
      EEPROM.write(ROMAddressLightOFFDoW, lightOFF.dayOfWeek);

      EEPROM.write(ROMAddressLightONHours, lightON.hours);
      EEPROM.write(ROMAddressLightONMinutes, lightON.minutes);
      EEPROM.write(ROMAddressLightONSeconds, lightON.seconds);

      EEPROM.write(ROMAddressLightOFFHours, lightOFF.hours);
      EEPROM.write(ROMAddressLightOFFMinutes, lightOFF.minutes);
      EEPROM.write(ROMAddressLightOFFSeconds, lightOFF.seconds);

      comandar("$");
    }

        if ((p1.toInt() == 26) && (p2[0] == '@')) {
//      Serial.println("detectado");
      coolerON.dayOfWeek = p3.toInt();
      coolerOFF.dayOfWeek = p4.toInt();

      coolerON.hours = p5.toInt();
      coolerON.minutes = p6.toInt();
      coolerON.seconds = p7.toInt();

      coolerOFF.hours = p8.toInt();
      coolerOFF.minutes = p9.toInt();
      coolerOFF.seconds = p10.toInt();

      EEPROM.write(ROMAddressCoolerONDoW, coolerON.dayOfWeek);
      EEPROM.write(ROMAddressCoolerOFFDoW, coolerOFF.dayOfWeek);

      EEPROM.write(ROMAddressCoolerONHours, coolerON.hours);
      EEPROM.write(ROMAddressCoolerONMinutes, coolerON.minutes);
      EEPROM.write(ROMAddressCoolerONSeconds, coolerON.seconds);

      EEPROM.write(ROMAddressCoolerOFFHours, coolerOFF.hours);
      EEPROM.write(ROMAddressCoolerOFFMinutes, coolerOFF.minutes);
      EEPROM.write(ROMAddressCoolerOFFSeconds, coolerOFF.seconds);
    }
  }

  if ((OPCODE == 'R')) {
    //R:30:16:19:3:6:6:2017
    if ((p1.toInt() >= 0) && (p1.toInt() < 60) && (p2.toInt() >= 0) && (p2.toInt() < 60)
        && (p3.toInt() >= 0) && (p3.toInt() < 24) && (p4.toInt() >= 1) && (p4.toInt() <= 7)
        && (p5.toInt() > 0) && (p5.toInt() <= 31) && (p6.toInt() > 0) && (p6.toInt() <= 12)
        && (p7.toInt() >= 2000) && (p7.toInt() <= 3000)) {

      currentTime.setDS1302Time(p1.toInt(), p2.toInt(), p3.toInt(), p4.toInt(),
                                p5.toInt(), p6.toInt(), p7.toInt());
      currentTime.updateTime();
//      Serial.println("Hora atualizada!");
      hora_atual();
      Serial.println("\r\n");
    }

    else if ((p1 == "") && (p2 == "") && (p3 == "") && (p4 == "") && (p5 == "")
             && (p6 == "") && (p7 == "")) {
      currentTime.updateTime();
      hora_atual();
      Serial.println("\r\n");
    }
    else {
      erro6();
    }
  }

  else if (OPCODE == 'b') {
    if (MODE == 25) {
      desligaBomba();
    }
    else {
      erro1();
    }
  }
  else if (OPCODE == 'B') {
    if (MODE == 25) {
      ligaBomba();
    }
    else {
      erro1();
    }
  }
  else if (OPCODE == 'l') {
    if (MODE == 25) {
      desligaLampada();
    }
    else {
      erro1();
    }
  }
  else if (OPCODE == 'L') {
    if (MODE == 25) {
      ligaLampada();
    }
    else {
      erro1();
    }
  }
    else if (OPCODE == 'C') {
    if (MODE == 25) {
      int potencia = p1.toInt();
      ligaCooler(potencia);
    }
    else {
      erro1();
    }
  }
      else if (OPCODE == 'c') {
    if (MODE == 25) {
      desligaCooler();
    }
    else {
      erro1();
    }
  }
  else {
    //erro5();
  }
}

int getValueIndex(int inputArray[], bool highest) {
  int counter = 0;
  int targetValueIndex = -1;
  int valueSoFar = -1;
  for (counter = 0; counter < sizeof(inputArray); counter++) {
    if (highest) {
      if (inputArray[counter] > valueSoFar) {
        targetValueIndex = counter;
      }
    } else {
      if (inputArray[counter] < valueSoFar) {
        targetValueIndex = counter;
      }
    }
  }
  return targetValueIndex;
}

void readSensors() {
  //    Serial.println("Reading Sensors");
  int counter = 0;
  UMIDADE = 0;
  for (counter = 0; counter < 8; counter++) {
    UMIDADE += analogRead(HL69Pin);
  }
  
  UMIDADE = UMIDADE/8;
  
  float umidadeReal = 1023 - UMIDADE;
  umidadeNormalizada = (float)umidadeReal / 7.0;
  umidadeMinNormalizada = 1000 * ((float)UMIDADE_MIN / 1023.0);
  umidadeMaxNormalizada = 1000 * ((float)UMIDADE_MAX / 1023.0);
  if (umidadeMinNormalizada < 0) {
    umidadeMinNormalizada = 0;
  }
  if (umidadeMaxNormalizada > 100) {
    umidadeMaxNormalizada = 100;
  }
  
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (!(isnan(h) || isnan(t))) {
    TEMP = t;
    UMIDADE_AR = h;
  }

  LUMINOSIDADE = analogRead(LDRPin);
  currentTime.updateTime();
}

void setup() {
  Serial.begin(9600);
  dht.begin();

  TEMP = 30;
  UMIDADE = 60;
  UMIDADE_AR = 45;
  LUMINOSIDADE = 80;

  pinMode(bombaPin, OUTPUT);
  pinMode(ledVermelho, OUTPUT);
  pinMode(ledVerde, OUTPUT);
  pinMode(ledAzul, OUTPUT);
  pinMode(luzPin, OUTPUT);
  pinMode(coolerPin, OUTPUT);

  comando.reserve(50);

  MODE = EEPROM.read(ROMAddressOpMode);
  
  UMIDADE_MIN = EEPROM.read(ROMAddressPIDUmidadeMin);
  UMIDADE_MAX = EEPROM.read(ROMAddressPIDUmidadeMax);

  pumpON.dayOfWeek = EEPROM.read(ROMAddressPumpONDoW);
  pumpOFF.dayOfWeek = EEPROM.read(ROMAddressPumpOFFDoW);

  pumpON.hours = EEPROM.read(ROMAddressPumpONHours);
  pumpON.minutes = EEPROM.read(ROMAddressPumpONMinutes);
  pumpON.seconds = EEPROM.read(ROMAddressPumpONSeconds);

  pumpOFF.hours = EEPROM.read(ROMAddressPumpOFFHours);
  pumpOFF.minutes = EEPROM.read(ROMAddressPumpOFFMinutes);
  pumpOFF.seconds = EEPROM.read(ROMAddressPumpOFFSeconds);

  lightON.dayOfWeek = EEPROM.read(ROMAddressLightONDoW);
  lightOFF.dayOfWeek = EEPROM.read(ROMAddressLightOFFDoW);

  lightON.hours = EEPROM.read(ROMAddressLightONHours);
  lightON.minutes = EEPROM.read(ROMAddressLightONMinutes);
  lightON.seconds = EEPROM.read(ROMAddressLightONSeconds);

  lightOFF.hours = EEPROM.read(ROMAddressLightOFFHours);
  lightOFF.minutes = EEPROM.read(ROMAddressLightOFFMinutes);
  lightOFF.seconds = EEPROM.read(ROMAddressLightOFFSeconds);
  
  tempControllerSetpoint = EEPROM.read(ROMAddressPIDSetpoint);
  tempControllerKp = EEPROM.read(ROMAddressPIDKp);
  tempControllerKi = EEPROM.read(ROMAddressPIDKi);
  tempControllerKd = EEPROM.read(ROMAddressPIDKd);

  coolerON.dayOfWeek = EEPROM.read(ROMAddressCoolerONDoW);
  coolerOFF.dayOfWeek = EEPROM.read(ROMAddressCoolerOFFDoW);

  coolerON.hours = EEPROM.read(ROMAddressCoolerONHours);
  coolerON.minutes = EEPROM.read(ROMAddressCoolerONMinutes);
  coolerON.seconds = EEPROM.read(ROMAddressCoolerONSeconds);

  coolerOFF.hours = EEPROM.read(ROMAddressCoolerOFFHours);
  coolerOFF.minutes = EEPROM.read(ROMAddressCoolerOFFMinutes);
  coolerOFF.seconds = EEPROM.read(ROMAddressCoolerOFFSeconds);
  
  Serial.println("Bem vindo!");

  tempController.SetMode(AUTOMATIC);
}

void loop() {

  bombaOn = digitalRead(bombaPin);
  luzOn = digitalRead(luzPin);


  if (MODE != 27) {
    readSensors();
//    Serial.println(umidadeNormalizada);
  }

  else {
    TEMP = 15;
    umidadeNormalizada = 50;
    LUMINOSIDADE = 420;
    UMIDADE_AR = 42;
    int r = random(-2, 3);

    TEMP += r;

    r = random(-6, 7);
    umidadeNormalizada += r;
    UMIDADE_AR += r;

    r = random(-29, 30);
    LUMINOSIDADE += r;

  }

  if (MODE == 24) {
    //controle de umidade (por histerese)
    if ((umidadeNormalizada > umidadeMinNormalizada) && (umidadeNormalizada < umidadeMaxNormalizada)) {
    } else if (umidadeNormalizada < umidadeMinNormalizada) {
      ligaBomba();
    } else {
      desligaBomba();
    }

    tempControllerInput = TEMP;

    tempController.Compute();
    ligaCooler(tempControllerOutput);
//    Serial.print("temp atual: ");
//    Serial.println(tempControllerInput);
//    Serial.print("temp desejada: ");
//    Serial.println(tempControllerSetpoint);
//    Serial.print("output: ");
//    Serial.println(tempControllerOutput);

//    delay(2000);
//    ligaledAzul();
  }

  else if (MODE == 25) {
//    ligaledVerde();
  }

  else if (MODE == 26) {
    //Serial.println("Modo programado detectado, analizando horas");
    //Serial.println("Hora atual: ");
    //hora_atual();


    //escrever codigo para controle programado.
    if (checkValidTime(pumpON) && checkValidTime(pumpOFF)) {
      int dayOfWeekON = pumpON.dayOfWeek;
      int dayOfWeekOFF = pumpOFF.dayOfWeek;
      int dayOfWeek = currentTime.dayofweek;
      if (dayOfWeekOFF < dayOfWeekON) {
        dayOfWeekOFF += 7;
        if (dayOfWeek == 1) {
          dayOfWeek = 8;
        }
      }
      if ((dayOfWeek >= dayOfWeekON) && (dayOfWeek <= dayOfWeekOFF)) {
        if ((hasMommentPassed(currentTime, pumpON) && (!hasMommentPassed(currentTime, pumpOFF)))) {
          ligaBomba();
        }
        else {
          desligaBomba();
        }
      }
      else {
//        Serial.print("Não ligo pois dayofweek nao esta entre on (");
//        Serial.print(pumpON.dayOfWeek);
//        Serial.print(") e off (");
//        Serial.print(pumpOFF.dayOfWeek);
//        Serial.println(") - pump");
        desligaBomba();
      }
    }
    else {
//      Serial.println("check valid time deu false");
      desligaBomba();
    }

    if (checkValidTime(lightON) && checkValidTime(lightOFF)) {
      int dayOfWeekON = lightON.dayOfWeek;
      int dayOfWeekOFF = lightOFF.dayOfWeek;
      int dayOfWeek = currentTime.dayofweek;
      if (dayOfWeekOFF < dayOfWeekON) {
        dayOfWeekOFF += 7;
        if (dayOfWeek == 1) {
          dayOfWeek = 8;
        }
      }
      if ((dayOfWeek >= dayOfWeekON) && (dayOfWeek <= dayOfWeekOFF)) {
        if ((hasMommentPassed(currentTime, lightON)) && (!hasMommentPassed(currentTime, lightOFF))) {
          ligaLampada();
        }
        else {
          desligaLampada();
        }
      }
      else {
        Serial.print("Não ligo pois dayofweek nao esta entre on (");
        Serial.print(lightON.dayOfWeek);
        Serial.print(") e off (");
        Serial.print(lightOFF.dayOfWeek);
        Serial.println(") - light");
        desligaLampada();
      }
    }
    else {
      Serial.println("check valid time deu false");
      desligaLampada();
    }

    //M:26:@:1:7:16:25:00:16:25:40
    if (checkValidTime(coolerON) && checkValidTime(coolerOFF)) {
      int dayOfWeekON = coolerON.dayOfWeek;
      int dayOfWeekOFF = coolerOFF.dayOfWeek;
      int dayOfWeek = currentTime.dayofweek;
      if (dayOfWeekOFF < dayOfWeekON) {
        dayOfWeekOFF += 7;
        if (dayOfWeek == 1) {
          dayOfWeek = 8;
        }
      }
      if ((dayOfWeek >= dayOfWeekON) && (dayOfWeek <= dayOfWeekOFF)) {
        if ((hasMommentPassed(currentTime, coolerON) && (!hasMommentPassed(currentTime, coolerOFF)))) {
          ligaCooler(coolerProgPower);
        }
        else {
          desligaCooler();
        }
      }
      else {
//        Serial.print("Não ligo pois dayofweek nao esta entre on (");
//        Serial.print(coolerON.dayOfWeek);
//        Serial.print(") e off (");
//        Serial.print(coolerOFF.dayOfWeek);
//        Serial.println(") - cooler");
        desligaCooler();
      }
    }
    else {
      Serial.println("check valid time deu false");
      desligaCooler();
    }
  }

  else if (MODE == 27) {
  }
}

//void rtcNoiseSoftwareFilter(virtuabotixRTC *rtc) {
//  virtuabotixRTC nonPointerRTC = *rtc;
//  currentTime.setDS1302Time(nonPointerRTC.seconds, nonPointerRTC.minutes, nonPointerRTC.hours, nonPointerRTC.dayofweek, nonPointerRTC.dayofmonth, nonPointerRTC.month, nonPointerRTC.year);
//}

