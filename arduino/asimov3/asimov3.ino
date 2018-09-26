#include <Arduino.h>
#include <Wire.h>
#include <AccelStepper.h>

AccelStepper MotorDireito(AccelStepper::DRIVER, 8, 9);
AccelStepper MotorEsquerdo(AccelStepper::DRIVER, 10, 11);

AccelStepper Base(AccelStepper::DRIVER, 47, 49);
AccelStepper Braco(AccelStepper::DRIVER, 51, 53);
AccelStepper Antebraco(AccelStepper::DRIVER, 50, 52);
AccelStepper Garra(AccelStepper::DRIVER, 46, 48);

bool Jog = false;

int reducaoA = 10000;
int reducaoB = 10000;
int reducaoC = 10000;
int reducaoGarra = 10000;

int PosicaoBase = 0;
int PosicaoBraco = 0;
int PosicaoAntebraco = 0;
int PosicaoGarra = 0;

float VelocidadeBase = 0;
float VelocidadeBraco = 0;
float VelocidadeAntebraco = 0;
float VelocidadeGarra = 0;

float VelocidadeAtualBase = 0;
float VelocidadeAtualBraco = 0;
float VelocidadeAtualAntebraco = 0;
float VelocidadeAtualGarra = 0;

float VelocidadeEsquerdo = 0;
float VelocidadeDireito = 0;
int VelocidadeAtualDireito = 0;
int VelocidadeAtualEsquerdo = 0;
long int tempoAceleracao = micros();

const int timeout = 10000;
unsigned long last_message = 0;


void calcularMovimentacao(int Velocidade, int Direcao, int RotacaoA, int RotacaoB, int RotacaoC, int GarraP, int jog) {
  // Cálculo de rotação dos motores, baseado em ângulo e velocidade.

  if (Direcao <= 90 && Direcao >= 0) {
    VelocidadeDireito = -cos(2 * Direcao * PI / 180) * Velocidade;
    VelocidadeEsquerdo = Velocidade;
    //Serial.print("90-0");
    //Serial.println();
  }

  else if (Direcao <= 180 && Direcao > 90) {
    VelocidadeEsquerdo = -cos(2 * Direcao * PI / 180) * Velocidade;
    VelocidadeDireito = Velocidade;
    //Serial.print("180-91");
    //Serial.println();
  }

  else if (Direcao <= 270 && Direcao > 180) {
    VelocidadeDireito = cos(2 * Direcao * PI / 180) * Velocidade;
    VelocidadeEsquerdo = -Velocidade;
    //Serial.print("270-181");
    //Serial.println();
  }

  else if (Direcao <= 360 && Direcao > 270) {
    VelocidadeEsquerdo = cos(2 * Direcao * PI / 180) * Velocidade;
    VelocidadeDireito = -Velocidade;
    //Serial.print("359-271");
    //Serial.println();
  }

  if(jog == 1){
    Jog = true;
    Base.setAcceleration(0);
    Braco.setAcceleration(0);
    Antebraco.setAcceleration(0);
    Garra.setAcceleration(0);

    VelocidadeBase = map(RotacaoA, -255, 255, -5000, 5000);
    VelocidadeBraco = map(RotacaoB, -255, 255, -5000, 5000);
    VelocidadeAntebraco = map(RotacaoC, -255, 255, -5000, 5000);
    VelocidadeGarra = map(GarraP, -255, 255, -5000, 5000);
  }
  
  else if(jog == 0){
    Jog = false;
    Base.setAcceleration(2000);
    Braco.setAcceleration(2000);
    Antebraco.setAcceleration(2000);
    Garra.setAcceleration(2000);

    VelocidadeAtualBase = 0;
    VelocidadeAtualBraco = 0;
    VelocidadeAtualAntebraco = 0;
    VelocidadeAtualGarra = 0;
    
    PosicaoBase = map(RotacaoA, 0, 360, 0, 200*reducaoA);
    PosicaoBraco = map(RotacaoB, 0, 360, 0, 200*reducaoB);
    PosicaoAntebraco = map(RotacaoC, 0, 360, 0, 200*reducaoC);
    PosicaoGarra = map(GarraP, 0, 360, 0, 200*reducaoGarra);
  
    Base.moveTo(PosicaoBase);
    Braco.moveTo(PosicaoBraco);
    Antebraco.moveTo(PosicaoAntebraco);
    Garra.moveTo(PosicaoGarra);
  }
  
}

void CalculoDir() {
  if (micros() - tempoAceleracao > 100) {
    if (VelocidadeDireito != VelocidadeAtualDireito) {
      if (VelocidadeDireito > VelocidadeAtualDireito) {
        VelocidadeAtualDireito++;
      }
      else {
        VelocidadeAtualDireito--;
      }
    }
    MotorDireito.setSpeed(VelocidadeAtualDireito);

    if (VelocidadeEsquerdo != VelocidadeAtualEsquerdo) {
      if (VelocidadeEsquerdo > VelocidadeAtualEsquerdo) {
        VelocidadeAtualEsquerdo++;
      }
      else {
        VelocidadeAtualEsquerdo--;
      }
    }
    MotorEsquerdo.setSpeed(VelocidadeAtualEsquerdo);

    if(Jog){
      if (VelocidadeBase != VelocidadeAtualBase) {
      if (VelocidadeBase > VelocidadeAtualBase) {
        VelocidadeAtualBase++;
        }
      else {
        VelocidadeAtualBase--;
        }
      }
      Base.setSpeed(VelocidadeAtualBase);

      if (VelocidadeBraco != VelocidadeAtualBraco) {
      if (VelocidadeBraco > VelocidadeAtualBraco) {
        VelocidadeAtualBraco++;
        }
      else {
        VelocidadeAtualBraco--;
        }
      }
      Braco.setSpeed(VelocidadeAtualBraco);

      if (VelocidadeAntebraco != VelocidadeAtualAntebraco) {
      if (VelocidadeAntebraco > VelocidadeAtualAntebraco) {
        VelocidadeAtualAntebraco++;
        }
      else {
        VelocidadeAtualAntebraco--;
        }
      }
      Antebraco.setSpeed(VelocidadeAtualAntebraco);

      if (VelocidadeGarra != VelocidadeAtualGarra) {
      if (VelocidadeGarra > VelocidadeAtualGarra) {
        VelocidadeAtualGarra++;
        }
      else {
        VelocidadeAtualGarra--;
        }
      }
      Garra.setSpeed(VelocidadeAtualGarra);
      
    }
    
    tempoAceleracao = micros();
  }
}

void setup()
{
  Serial.begin(115200);
  //Serial.setTimeout(200);
  delay(100);
  
  MotorDireito.setPinsInverted(true,true);
  MotorDireito.setMaxSpeed(10000);
  MotorDireito.setSpeed(0);
  
  MotorEsquerdo.setPinsInverted(true,true);
  MotorEsquerdo.setMaxSpeed(10000);
  MotorEsquerdo.setSpeed(0);

  Base.setPinsInverted(true,true);
  Base.setMaxSpeed(10000);
  Base.setSpeed(0);

  Braco.setPinsInverted(true,true);
  Braco.setMaxSpeed(10000);
  Braco.setSpeed(0);

  Antebraco.setPinsInverted(true,true);
  Antebraco.setMaxSpeed(10000);
  Antebraco.setSpeed(0);
  
  Garra.setPinsInverted(true,true);
  Garra.setMaxSpeed(10000);
  Garra.setSpeed(0);
  
  pinMode(13, OUTPUT);
}

void loop() {
  /*
      Formato dos dados recebidos: "v<velocidade>d<direção>x<servo X>y<servo Y>f<faróis>c<cooler>"

      Velocidade: Varia de -255 a 255 (ré a velocidade máxima)
      Direção: Varia de -255 a 255 (esquerda a direita)
      Servo X: Varia de -90 a 90
      Servo Y: Varia de -90 a 90
      Faróis: Varia de 0 a 255 (apagado a brilho máximo)
      Cooler: Varia de 0 a 255 (desligado a velocidade máxima)

  */
digitalWrite(13, HIGH);
  if (Serial.available() >= 8) {
    last_message = millis();

    Serial.readStringUntil('v');
    int v = Serial.parseInt();          // Valor da velocidade

    Serial.readStringUntil('d');
    int d = Serial.parseInt();          // Valor da direção

    Serial.readStringUntil('a');
    int a = Serial.parseInt();          // Valor em X do braco ou rotacao da base

    Serial.readStringUntil('b');
    int b = Serial.parseInt();          // Valor em Y ou rotacao do braco

    Serial.readStringUntil('c');
    int c = Serial.parseInt();          // Valor em Z ou rotacao do cotovelo

    Serial.readStringUntil('g');
    int g = Serial.parseInt();          // garra

    Serial.readStringUntil('j');
    int j = Serial.parseInt();          // jog

    j = constrain(j, 0, 1);

    v = constrain(v, 0, 255);
    d = constrain(d, 0, 360);

    if(j == 1){
      a = constrain(a, -255, 255);
      b = constrain(b, -255, 255);
      c = constrain(c, -255, 255);
  
      g = constrain(g, -255, 255);
    }
    else if(j==0){
      a = constrain(a, 0, 360);
      b = constrain(b, 0, 360);
      c = constrain(c, 0, 360);
  
      g = constrain(g, 0, 360);
    }
    
    calcularMovimentacao(map(v, 0, 255, 0, 2500), d, a, b, c, g, j);
  }
  
  digitalWrite(13, LOW);
  CalculoDir();
  MotorDireito.runSpeed();
  MotorEsquerdo.runSpeed();
  
  if(Jog){
    Base.runSpeed();
    Braco.runSpeed();
    Antebraco.runSpeed();
    Garra.runSpeed();
  }
  else{
    Base.run();
    Braco.run();
    Antebraco.run();
    Garra.run();
  }
  
  
  //delay(10);
  //Serial.print("TargetD-" + String(VelocidadeDireito) + "-AtualD-" + String(VelocidadeAtualDireito));
  //Serial.println("-TargetE-" + String(VelocidadeEsquerdo) + "-AtualE-" + String(VelocidadeAtualEsquerdo));

  if (millis() - last_message > timeout) {
    //VelocidadeDireito = 0;
    //VelocidadeEsquerdo = 0;
  }
}
