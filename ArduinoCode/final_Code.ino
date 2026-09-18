// ============================================================
// 3-AXIS ROBOT ARM
// Arduino + CNC Shield V3 + TMC2208
// Python Serial Control
//
// Commands from Python:
//
// HOME
//
// A,j1,j2,j3
// Example:
// A,-30.00,-45.00,-10.00
// ============================================================


// ============================================================
// MOTOR PINS
// ============================================================

const int STEP1 = 2;
const int DIR1  = 5;

const int STEP2 = 3;
const int DIR2  = 6;

const int STEP3 = 4;
const int DIR3  = 7;

const int EN = 8;


// ============================================================
// LIMIT SWITCHES
// ============================================================

const int LIMIT_X = 9;
const int LIMIT_Y = 10;
const int LIMIT_Z = 11;


// ============================================================
// MOTOR PARAMETERS
// ============================================================

const float steps = 200.0;

const float gearboxreduction = 25.0;

const float microstepFactor = 9.6;


// ============================================================
// HOMING
// ============================================================

const int HOME_DIR = HIGH;

const unsigned long HOMING_DELAY = 1000;

const long MAX_HOMING_STEPS = 100000;


// ============================================================
// CURRENT ANGLES
// ============================================================

float currentAngle1 = 0.0;
float currentAngle2 = 0.0;
float currentAngle3 = 0.0;


// ============================================================
// SOFTWARE LIMITS
// ============================================================

const float MIN_ANGLE1 = -120.0;
const float MIN_ANGLE2 = -180.0;
const float MIN_ANGLE3 = -120.0;

const float MAX_ANGLE1 = 0.0;
const float MAX_ANGLE2 = 0.0;
const float MAX_ANGLE3 = 0.0;


// ============================================================
// SERIAL BUFFER
// ============================================================

String inputString = "";


// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(9600);

  // ==========================================================
  // MOTOR PINS
  // ==========================================================

  pinMode(STEP1, OUTPUT);
  pinMode(DIR1, OUTPUT);

  pinMode(STEP2, OUTPUT);
  pinMode(DIR2, OUTPUT);

  pinMode(STEP3, OUTPUT);
  pinMode(DIR3, OUTPUT);


  // ==========================================================
  // ENABLE
  // ==========================================================

  pinMode(EN, OUTPUT);

  // CNC Shield V3:
  // LOW = Enable motors

  digitalWrite(EN, LOW);


  // ==========================================================
  // LIMIT SWITCHES
  // ==========================================================

  pinMode(LIMIT_X, INPUT_PULLUP);
  pinMode(LIMIT_Y, INPUT_PULLUP);
  pinMode(LIMIT_Z, INPUT_PULLUP);


  // ==========================================================
  // INITIAL STEP STATE
  // ==========================================================

  digitalWrite(STEP1, LOW);
  digitalWrite(STEP2, LOW);
  digitalWrite(STEP3, LOW);


  delay(500);


  Serial.println("STARTING");


  // ==========================================================
  // AUTOMATIC HOMING
  //
  // ALL 3 MOTORS MOVE TOGETHER
  // ==========================================================

  Home_All_Axes();


  // ==========================================================
  // RESET CURRENT ANGLES
  // ==========================================================

  currentAngle1 = 0.0;
  currentAngle2 = 0.0;
  currentAngle3 = 0.0;


  Serial.println("HOMING_DONE");
  Serial.println("READY");
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
  // ----------------------------------------------------------
  // Receive commands from Python
  // ----------------------------------------------------------

  if (Serial.available() > 0)
  {
    inputString = Serial.readStringUntil('\n');

    inputString.trim();

    if (inputString.length() > 0)
    {
      processCommand(inputString);
    }
  }
}


// ============================================================
// PROCESS SERIAL COMMAND
// ============================================================

void processCommand(String command)
{
  command.trim();


  // ==========================================================
  // HOME COMMAND
  // ==========================================================

  if (command.equalsIgnoreCase("HOME"))
  {
    Serial.println("HOME_COMMAND_RECEIVED");


    // --------------------------------------------------------
    // Home all three motors simultaneously
    // --------------------------------------------------------

    Home_All_Axes();


    // --------------------------------------------------------
    // Reset angles
    // --------------------------------------------------------

    currentAngle1 = 0.0;
    currentAngle2 = 0.0;
    currentAngle3 = 0.0;


    Serial.println("HOMING_DONE");


    return;
  }


  // ==========================================================
  // ANGLE COMMAND
  //
  // A,j1,j2,j3
  // ==========================================================

  if (command.startsWith("A,"))
  {
    String data = command.substring(2);


    int comma1 = data.indexOf(',');

    if (comma1 < 0)
    {
      Serial.println("ERROR,BAD_FORMAT");
      return;
    }


    int comma2 = data.indexOf(
      ',',
      comma1 + 1
    );

    if (comma2 < 0)
    {
      Serial.println("ERROR,BAD_FORMAT");
      return;
    }


    String s1 = data.substring(
      0,
      comma1
    );

    String s2 = data.substring(
      comma1 + 1,
      comma2
    );

    String s3 = data.substring(
      comma2 + 1
    );


    float target1 = s1.toFloat();
    float target2 = s2.toFloat();
    float target3 = s3.toFloat();


    // --------------------------------------------------------
    // Print targets
    // --------------------------------------------------------

    Serial.print("TARGETS: ");

    Serial.print(target1);

    Serial.print(", ");

    Serial.print(target2);

    Serial.print(", ");

    Serial.println(target3);


    // ========================================================
    // SOFTWARE LIMIT J1
    // ========================================================

    if (
      target1 < MIN_ANGLE1 ||
      target1 > MAX_ANGLE1
    )
    {
      Serial.println("ERROR,J1_LIMIT");
      return;
    }


    // ========================================================
    // SOFTWARE LIMIT J2
    // ========================================================

    if (
      target2 < MIN_ANGLE2 ||
      target2 > MAX_ANGLE2
    )
    {
      Serial.println("ERROR,J2_LIMIT");
      return;
    }


    // ========================================================
    // SOFTWARE LIMIT J3
    // ========================================================

    if (
      target3 < MIN_ANGLE3 ||
      target3 > MAX_ANGLE3
    )
    {
      Serial.println("ERROR,J3_LIMIT");
      return;
    }


    // ========================================================
    // MOVE
    // ========================================================

    MoveToAngles(
      target1,
      target2,
      target3
    );


    Serial.println("DONE");


    return;
  }


  // ==========================================================
  // UNKNOWN COMMAND
  // ==========================================================

  Serial.println("ERROR,UNKNOWN_COMMAND");
}


// ============================================================
// ANGLE TO STEPS
// ============================================================

long Angle_Converter(float angle)
{
  float result =
    (
      abs(angle)
      *
      steps
      *
      gearboxreduction
      *
      microstepFactor
    )
    / 360.0;


  return (long)result;
}


// ============================================================
// MOVE TO TARGET ANGLES
//
// J1 + J2 + J3 move synchronously
// ============================================================

void MoveToAngles(
  float target1,
  float target2,
  float target3
)
{
  // ----------------------------------------------------------
  // Calculate angle differences
  // ----------------------------------------------------------

  float delta1 =
    target1 - currentAngle1;

  float delta2 =
    target2 - currentAngle2;

  float delta3 =
    target3 - currentAngle3;


  // ----------------------------------------------------------
  // Directions
  // ----------------------------------------------------------

  bool dir1 =
    (delta1 >= 0)
    ? HIGH
    : LOW;

  bool dir2 =
    (delta2 >= 0)
    ? HIGH
    : LOW;

  bool dir3 =
    (delta3 >= 0)
    ? HIGH
    : LOW;


  // ----------------------------------------------------------
  // Set directions BEFORE stepping
  // ----------------------------------------------------------

  digitalWrite(
    DIR1,
    dir1
  );

  digitalWrite(
    DIR2,
    dir2
  );

  digitalWrite(
    DIR3,
    dir3
  );


  // ----------------------------------------------------------
  // Calculate number of steps
  // ----------------------------------------------------------

  long steps1 =
    Angle_Converter(delta1);

  long steps2 =
    Angle_Converter(delta2);

  long steps3 =
    Angle_Converter(delta3);


  // ----------------------------------------------------------
  // Find maximum steps
  // ----------------------------------------------------------

  long maxSteps =
    max(
      steps1,
      max(
        steps2,
        steps3
      )
    );


  if (maxSteps <= 0)
  {
    return;
  }


  // ----------------------------------------------------------
  // Calculate movement ratios
  // ----------------------------------------------------------

  float ratio1 =
    (float)steps1 / maxSteps;

  float ratio2 =
    (float)steps2 / maxSteps;

  float ratio3 =
    (float)steps3 / maxSteps;


  float accum1 = 0.0;
  float accum2 = 0.0;
  float accum3 = 0.0;


  // ----------------------------------------------------------
  // Stop flags
  // ----------------------------------------------------------

  bool stop1 = false;
  bool stop2 = false;
  bool stop3 = false;


  // ==========================================================
  // SYNCHRONIZED MOVEMENT
  // ==========================================================

  for (
    long i = 0;
    i < maxSteps;
    i++
  )
  {

    // ========================================================
    // LIMIT CHECK J1
    // ========================================================

    if (
      digitalRead(LIMIT_X) == LOW &&
      dir1 == HIGH &&
      !stop1
    )
    {
      stop1 = true;

      currentAngle1 = 0.0;

      Serial.println("LIMIT,X");
    }


    // ========================================================
    // LIMIT CHECK J2
    // ========================================================

    if (
      digitalRead(LIMIT_Y) == LOW &&
      dir2 == HIGH &&
      !stop2
    )
    {
      stop2 = true;

      currentAngle2 = 0.0;

      Serial.println("LIMIT,Y");
    }


    // ========================================================
    // LIMIT CHECK J3
    // ========================================================

    if (
      digitalRead(LIMIT_Z) == LOW &&
      dir3 == HIGH &&
      !stop3
    )
    {
      stop3 = true;

      currentAngle3 = 0.0;

      Serial.println("LIMIT,Z");
    }


    // ========================================================
    // MOTOR 1
    // ========================================================

    accum1 += ratio1;

    if (
      accum1 >= 1.0 &&
      !stop1 &&
      steps1 > 0
    )
    {
      digitalWrite(
        STEP1,
        HIGH
      );

      delayMicroseconds(
        HOMING_DELAY
      );

      digitalWrite(
        STEP1,
        LOW
      );

      delayMicroseconds(
        HOMING_DELAY
      );

      accum1 -= 1.0;
    }


    // ========================================================
    // MOTOR 2
    // ========================================================

    accum2 += ratio2;

    if (
      accum2 >= 1.0 &&
      !stop2 &&
      steps2 > 0
    )
    {
      digitalWrite(
        STEP2,
        HIGH
      );

      delayMicroseconds(
        HOMING_DELAY
      );

      digitalWrite(
        STEP2,
        LOW
      );

      delayMicroseconds(
        HOMING_DELAY
      );

      accum2 -= 1.0;
    }


    // ========================================================
    // MOTOR 3
    // ========================================================

    accum3 += ratio3;

    if (
      accum3 >= 1.0 &&
      !stop3 &&
      steps3 > 0
    )
    {
      digitalWrite(
        STEP3,
        HIGH
      );

      delayMicroseconds(
        HOMING_DELAY
      );

      digitalWrite(
        STEP3,
        LOW
      );

      delayMicroseconds(
        HOMING_DELAY
      );

      accum3 -= 1.0;
    }
  }


  // ----------------------------------------------------------
  // Update current angles
  // ----------------------------------------------------------

  if (!stop1)
  {
    currentAngle1 = target1;
  }

  if (!stop2)
  {
    currentAngle2 = target2;
  }

  if (!stop3)
  {
    currentAngle3 = target3;
  }
}


// ============================================================
// HOME ALL AXES
//
// IMPORTANT:
//
// J1 + J2 + J3 START TOGETHER.
//
// Each motor stops when its own limit switch is reached.
//
// The function finishes only when ALL 3 motors
// have reached their limit switches.
//
// ============================================================

void Home_All_Axes()
{
  Serial.println("HOMING_START");


  // ==========================================================
  // SET HOME DIRECTION FOR ALL MOTORS FIRST
  // ==========================================================

  digitalWrite(
    DIR1,
    HOME_DIR
  );

  digitalWrite(
    DIR2,
    HOME_DIR
  );

  digitalWrite(
    DIR3,
    HOME_DIR
  );


  // ==========================================================
  // HOMING COUNTERS
  // ==========================================================

  long counter1 = 0;
  long counter2 = 0;
  long counter3 = 0;


  // ==========================================================
  // HOMING STOP FLAGS
  //
  // false = motor still moving
  // true  = motor reached limit
  // ==========================================================

  bool home1_done = false;
  bool home2_done = false;
  bool home3_done = false;


  // ==========================================================
  // CHECK IF A LIMIT SWITCH IS ALREADY PRESSED
  // ==========================================================

  if (digitalRead(LIMIT_X) == LOW)
  {
    home1_done = true;

    Serial.println("HOME_LIMIT,X_ALREADY");
  }

  if (digitalRead(LIMIT_Y) == LOW)
  {
    home2_done = true;

    Serial.println("HOME_LIMIT,Y_ALREADY");
  }

  if (digitalRead(LIMIT_Z) == LOW)
  {
    home3_done = true;

    Serial.println("HOME_LIMIT,Z_ALREADY");
  }


  // ==========================================================
  // ALL THREE MOTORS MOVE TOGETHER
  // ==========================================================

  while (
    !home1_done ||
    !home2_done ||
    !home3_done
  )
  {

    // ========================================================
    // CHECK LIMIT X / J1
    // ========================================================

    if (!home1_done)
    {
      if (
        digitalRead(LIMIT_X) == LOW
      )
      {
        home1_done = true;

        currentAngle1 = 0.0;

        digitalWrite(
          STEP1,
          LOW
        );

        Serial.println(
          "HOME_LIMIT,X"
        );
      }
    }


    // ========================================================
    // CHECK LIMIT Y / J2
    // ========================================================

    if (!home2_done)
    {
      if (
        digitalRead(LIMIT_Y) == LOW
      )
      {
        home2_done = true;

        currentAngle2 = 0.0;

        digitalWrite(
          STEP2,
          LOW
        );

        Serial.println(
          "HOME_LIMIT,Y"
        );
      }
    }


    // ========================================================
    // CHECK LIMIT Z / J3
    // ========================================================

    if (!home3_done)
    {
      if (
        digitalRead(LIMIT_Z) == LOW
      )
      {
        home3_done = true;

        currentAngle3 = 0.0;

        digitalWrite(
          STEP3,
          LOW
        );

        Serial.println(
          "HOME_LIMIT,Z"
        );
      }
    }


    // ========================================================
    // J1 STEP
    //
    // J1 continues only if it has NOT reached its limit.
    // ========================================================

    if (
      !home1_done &&
      counter1 < MAX_HOMING_STEPS
    )
    {
      digitalWrite(
        STEP1,
        HIGH
      );
    }


    // ========================================================
    // J2 STEP
    //
    // J2 continues only if it has NOT reached its limit.
    // ========================================================

    if (
      !home2_done &&
      counter2 < MAX_HOMING_STEPS
    )
    {
      digitalWrite(
        STEP2,
        HIGH
      );
    }


    // ========================================================
    // J3 STEP
    //
    // J3 continues only if it has NOT reached its limit.
    // ========================================================

    if (
      !home3_done &&
      counter3 < MAX_HOMING_STEPS
    )
    {
      digitalWrite(
        STEP3,
        HIGH
      );
    }


    // ========================================================
    // HIGH PULSE
    //
    // All active motors receive the HIGH pulse together.
    // ========================================================

    delayMicroseconds(
      HOMING_DELAY
    );


    // ========================================================
    // STEP LOW
    // ========================================================

    if (!home1_done)
    {
      digitalWrite(
        STEP1,
        LOW
      );
    }

    if (!home2_done)
    {
      digitalWrite(
        STEP2,
        LOW
      );
    }

    if (!home3_done)
    {
      digitalWrite(
        STEP3,
        LOW
      );
    }


    // ========================================================
    // LOW PULSE DELAY
    // ========================================================

    delayMicroseconds(
      HOMING_DELAY
    );


    // ========================================================
    // INCREMENT COUNTERS
    // ========================================================

    if (!home1_done)
    {
      counter1++;
    }

    if (!home2_done)
    {
      counter2++;
    }

    if (!home3_done)
    {
      counter3++;
    }


    // ========================================================
    // MAXIMUM STEP SAFETY
    //
    // If a motor cannot find its switch after
    // MAX_HOMING_STEPS, stop it.
    // ========================================================

    if (
      counter1 >= MAX_HOMING_STEPS &&
      !home1_done
    )
    {
      home1_done = true;

      digitalWrite(
        STEP1,
        LOW
      );

      Serial.println(
        "ERROR,HOMING_TIMEOUT_X"
      );
    }


    if (
      counter2 >= MAX_HOMING_STEPS &&
      !home2_done
    )
    {
      home2_done = true;

      digitalWrite(
        STEP2,
        LOW
      );

      Serial.println(
        "ERROR,HOMING_TIMEOUT_Y"
      );
    }


    if (
      counter3 >= MAX_HOMING_STEPS &&
      !home3_done
    )
    {
      home3_done = true;

      digitalWrite(
        STEP3,
        LOW
      );

      Serial.println(
        "ERROR,HOMING_TIMEOUT_Z"
      );
    }
  }


  // ==========================================================
  // MAKE SURE ALL STEP PINS ARE LOW
  // ==========================================================

  digitalWrite(
    STEP1,
    LOW
  );

  digitalWrite(
    STEP2,
    LOW
  );

  digitalWrite(
    STEP3,
    LOW
  );


  // ==========================================================
  // RESET CURRENT ANGLES
  // ==========================================================

  currentAngle1 = 0.0;
  currentAngle2 = 0.0;
  currentAngle3 = 0.0;


  // ==========================================================
  // HOMING FINISHED
  //
  // This is printed ONLY after ALL THREE axes stopped.
  // ==========================================================

  Serial.println(
    "HOMING_FINISHED"
  );
}