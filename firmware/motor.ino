#include "Cdrv8833.h"

// Direccion
#define PIN_A 12
// Direccion
#define PIN_B 13
// Sleep
#define PIN_C 14

#define IN1_PIN 12 // in1 pin from one of the two DRV8833 H-bridge
#define IN2_PIN 13 // in2 pin from one of the two DRV8833 H-bridge
#define CHANNEL 0  // there are 16 unique PWM channels (0..15)
#define SWAP false // swap motor rotation direction

Cdrv8833 myMotor; // default constructor

void setup1()
{
  Serial.begin(9600);

  pinMode(PIN_A, OUTPUT);
  pinMode(PIN_B, OUTPUT);
  pinMode(PIN_C, OUTPUT);
  digitalWrite(PIN_C, HIGH);

  myMotor.init(IN1_PIN, IN2_PIN, CHANNEL, SWAP);
}

void loop1()
{
  String command;
  if (Serial.available())
  {                                // check Serial for new command
    command = Serial.readString(); // read the new command from Serial
    command.toLowerCase();         // convert it to lowercase

    if (command.equals("swap"))
    {
      myMotor.swapDirection(true); // swap rotation direction
      Serial.println("--> swapped rotation direction.");
    }
    else if (command.equals("noswap"))
    {
      myMotor.swapDirection(false); // default rotation direction
      Serial.println("--> default rotation direction.");
    }
    else if (command.equals("slow"))
    {
      myMotor.setDecayMode(drv8833DecaySlow); // decay mode SLOW
      Serial.println("--> Decay mode SLOW - good torque.");
    }
    else if (command.equals("fast"))
    {
      myMotor.setDecayMode(drv8833DecayFast); // decay mode FAST
      Serial.println("--> Decay mode FAST - poor torque.");
    }
    else if (command.equals("stop"))
    {
      myMotor.stop(); // stop moto rotation
      Serial.println("--> Motor stopped.");
    }
    else if (command.startsWith("move"))
    {
      command.replace("move", "");   // remove the word "move"
      command.replace(" ", "");      // remove spaces (if present)
      myMotor.move(command.toInt()); // start rotation at desired speed
      Serial.printf("--> Motor rotation speed: %ld.\n", command.toInt());
    }
  }
}

void printParams()
{
  int pina = digitalRead(PIN_A);
  int pinb = digitalRead(PIN_B);
  int pinc = digitalRead(PIN_C);
  Serial.println("pines ---");
  Serial.print("pina: ");
  Serial.print(pina);
  Serial.print(" pinb: ");
  Serial.print(pinb);
  Serial.print(" pinc: ");
  Serial.print(pinc);
  Serial.println("---");
}
