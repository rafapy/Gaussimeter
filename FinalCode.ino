#include <stdio.h> // Biblioteca de funções úteis para entrada e saída de dados

#include <LiquidCrystal.h> //Biblioteca do Painel LCD

#include <math.h> //Biblioteca de aporeções matemáticas

//Bibliotecas para o Magnetômetro:
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
//_________________________________

#include "BluetoothSerial.h" //Biblioteca para implementação Bluetooth

//Verificações para o Bluetooth:
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
//_________________________________

#define PI 3.141592654 //Define o número de PI como uma variável global

#define BUTTON_PIN 23 //Porta em que está conectado o Push Button

BluetoothSerial SerialBT; //Chama a função bluetooth como "SerialBT"

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345); //Declara o código do sensor GY-273

LiquidCrystal lcd(4,2,5,18,13,19); //Passa para a função as portas nas quais o Painel LCD está conectado



void setup(){
    Serial.begin(115200); //Inicia o leitura serial

    pinMode(BUTTON_PIN, INPUT_PULLUP); //Define a porta do Push Button como input

    //Inicia e limpa o painel LCD:
    lcd.begin(16, 2); //Define a resolução do painel LCD
    lcd.clear();
    lcd.setCursor(0,1); 
    //_________________________________

    SerialBT.begin("Fisica_Experimental"); //Inicia o Bluetooth com o nome "Fisica_Experimental"
    Serial.println("O dispositivo já pode ser pareado!");//Printa a mensagem no monitor serial

    //Checa se a biblioteca não encontrou as portas do Magnetômetro:
    if(!mag.begin())
    {
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
    }
    //_________________________________
}

void loop(void)
{
    int buttonState, headingDegrees, cont;
    float xMax, yMax, xMin, yMin, tvalue, rawvalue, heading, declinationAngle;
    char data[20];

    buttonState = digitalRead(BUTTON_PIN); //Leitura da saída do Push Button
    cont = 0;

    if(buttonState == LOW){ //Seção para calibrar o sensor, se o botão for pressionado

      lcd.clear(); //Limpa o painel LCD
      lcd.print("Calibrando");//Printa no painel LCD
      SerialBT.print("Calibrando");//Printa no Serial Bluetooth

      while(cont<15){
        sensors_event_t event; //Chama a biblioteca do magnetômetro
        mag.getEvent(&event);  //Inicia a magnetômetro

        float x,y;
        
        if (cont == 0) {
          xMax = event.magnetic.x;
          yMax = event.magnetic.y;
          xMin = event.magnetic.x;
          yMin = event.magnetic.y;
        }
        
        x = event.magnetic.x;
        y = event.magnetic.y;

        if (x>xMax){
          xMax = x;
        }

        if (y>yMax){
          yMax = y;
        }

        if (x<xMin){
          xMin = x;
        }

        if (y<yMin){
          yMin = y;
        }

        if(cont == 5 || cont == 10 || cont == 14){
          lcd.print(".");
          SerialBT.print(".");
        }

        cont++;
        delay(500);
      }
      
    }
    else{ //Leitura padrão do Magnetômetro
    sensors_event_t event; //Chama a biblioteca do sensor
    mag.getEvent(&event);//Inicia a leitura do sensor

    heading = atan2((event.magnetic.y - ((yMax + yMin)/2.0)), (event.magnetic.x - ((xMax + xMin)/2.0))); //Calcula a bissetriz das coordenadas fornecidas pelo Magnetômetro
    declinationAngle = -0.384; // Ângulo de declinação, que varia dependendo do lugar na Terra, devido ao campo irregular, consultado em "https://www.magnetic-declination.com/"
    heading += declinationAngle;

    if(heading < 0)
    heading += 2*PI;

    if(heading > 2*PI)
    heading -= 2*PI;
  
    headingDegrees = heading * 180/M_PI; //Transforma de radianos para graus

    Serial.print("Heading (degrees): "); //Printa no Monitor Serial 
    Serial.println(headingDegrees); //Printa no Monitor Serial 
    SerialBT.println(headingDegrees); //Printa no Serial Bluetooth
    }


    rawvalue = analogRead(15); //Lê continuamente o valor que chega na entrada 15, do sensor de efeitor Hall (UGN3503)

    tvalue = ((rawvalue*0.2896) - 861.5385); //Transforma a saída analógica em Gauss, por meio de uma aproximação linear, feita pelo programa SciDavis 
    tvalue = abs(tvalue); //Associa o módulo do valor de Gauss
    //Serial.println(rawvalue);
    //Serial.println(tvalue);

    lcd.clear(); //Limpa o painel LCD
    lcd.print("Gauss: "); 
    sprintf(data,"%4.2f", tvalue); //Passa o valor em float para uma string
    lcd.print(data);

    lcd.setCursor(0,1);  //Passa para a linha inferior do painel LCD
    lcd.print ("Angulo: "); 
    lcd.print (headingDegrees); 

    delay(500);
}