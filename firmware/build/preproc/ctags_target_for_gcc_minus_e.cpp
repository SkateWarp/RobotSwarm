# 1 "C:\\Users\\felix\\Documents\\Alterna\\esp32\\esp32.ino"
# 2 "C:\\Users\\felix\\Documents\\Alterna\\esp32\\esp32.ino" 2
# 3 "C:\\Users\\felix\\Documents\\Alterna\\esp32\\esp32.ino" 2
# 4 "C:\\Users\\felix\\Documents\\Alterna\\esp32\\esp32.ino" 2
# 5 "C:\\Users\\felix\\Documents\\Alterna\\esp32\\esp32.ino" 2

Adafruit_MPU6050 mpu;

void setup()
{
  Serial.begin(115200);

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }

  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange())
  {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange())
  {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth())
  {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}

void loop()
{
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);


  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(500);
}
# 1 "C:\\Users\\felix\\Documents\\Alterna\\esp32\\motor.ino"
# 2 "C:\\Users\\felix\\Documents\\Alterna\\esp32\\motor.ino" 2

// Direccion

// Direccion

// Sleep







Cdrv8833 myMotor; // default constructor

void setup1()
{
  Serial.begin(9600);

  pinMode(12, 0x03);
  pinMode(13, 0x03);
  pinMode(14, 0x03);
  digitalWrite(14, 0x1);

  myMotor.init(12 /* in1 pin from one of the two DRV8833 H-bridge*/, 13 /* in2 pin from one of the two DRV8833 H-bridge*/, 0 /* there are 16 unique PWM channels (0..15)*/, false /* swap motor rotation direction*/);
}

void loop1()
{
  String command;
  if (Serial.available())
  { // check Serial for new command
    command = Serial.readString(); // read the new command from Serial
    command.toLowerCase(); // convert it to lowercase

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
      command.replace("move", ""); // remove the word "move"
      command.replace(" ", ""); // remove spaces (if present)
      myMotor.move(command.toInt()); // start rotation at desired speed
      Serial.printf("--> Motor rotation speed: %ld.\n", command.toInt());
    }
  }
}

void printParams()
{
  int pina = digitalRead(12);
  int pinb = digitalRead(13);
  int pinc = digitalRead(14);
  Serial.println("pines ---");
  Serial.print("pina: ");
  Serial.print(pina);
  Serial.print(" pinb: ");
  Serial.print(pinb);
  Serial.print(" pinc: ");
  Serial.print(pinc);
  Serial.println("---");
}
