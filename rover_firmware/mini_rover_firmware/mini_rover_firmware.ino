#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

//GLOBAL DATA
static char dataBuffer[64] = {0};
static char dataMotors[64] = {0};
static float motors[6] = {0.0f};
//MOTOR1
#define EN1 15
#define IN11 2
#define IN12 0
//MOTOR2
#define EN2 17
#define IN21 16
#define IN22 4
//MOTOR3
#define EN3 5
#define IN31 18
#define IN32 19
//MOTOR4
#define EN4 23
#define IN41 21
#define IN42 22
//MOTOR5
#define EN5 12
#define IN51 14
#define IN52 27
//MOTOR6
#define EN6 33
#define IN61 25
#define IN62 26
#define LOWRANGE -250
#define UPRANGE 250

//*********************************************************
//FUNCTIONS

//ERROR CHECKING: Size of Data
bool checkSize(char *dataBuffer, int actualSize){
  
  //if size of array is too small return error
  if(actualSize < 15) {
    Serial.println("Error: Invalid entry.");
    return false;
  }
  return true;
}

//ERROR CHECKING: Format of Data
bool checkSequence (char *dataBuffer, int actualSize){
  
  //check for starting sequence
  if(dataBuffer[0] != '$' || dataBuffer[1] != 'V' || dataBuffer[2] != '(') {
    Serial.println("Error: Invalid start sequence");
    return false;
  }
  //check for ending sequence
  if(dataBuffer[actualSize-1] != ')' || dataBuffer[actualSize] != '\0') {
    Serial.println("Error: Invalid end sequence");
    return false;
  }
  return true;
}

//ERROR CHECKING: Validity of Data
bool checkValid(char *dataBuffer, int actualSize){
  
  //variables for checking 6 non-empty valid floats
  int commaCounter = 0;
  int dataBufCounter = 3;
  bool commaSeen = false;
  
  //temp array to store and check each float
  char floatCheck[64] = {0};
  int floatCheckCounter = 0;

  //while loop iterates through each character in raw data
  while (dataBufCounter < actualSize){
    char currentChar = dataBuffer[dataBufCounter];

    //appends valid characters to temporary array
    if (isDigit(currentChar) || currentChar == '.' || currentChar == '-'){
      floatCheck[floatCheckCounter] = currentChar;
      floatCheckCounter++; 
      commaSeen = false;
 

    //checks for ',' and ')', parses and stores sections into temporary array
    } else if (currentChar == ',' || currentChar == ')'){
      
      //null-terminates temporary array
      floatCheck[floatCheckCounter] = '\0';

      //TEMP ARRAY CHECK 1: DECIMALS AND NEGATIVES
      //checks multiple decimals or misplaced negatives
      int decimalCount = 0;
      for (int i = 0; i < strlen(floatCheck); i++){
        if (floatCheck[i] == '.'){
          decimalCount++;
          if (decimalCount > 1){
            Serial.println("Error: Invalid decimal");
            return false;
          }
        }
        if (floatCheck[i+1] == '-'){
          Serial.println("Error: Invalid negative");
          return false;
        }
      }

      
      //TEMP ARRAY CHECK 2: EMPTY ENTRIES
      //checks if two commas are seen in a row, or if the penultimate character is a comma
      if (commaSeen || dataBuffer[actualSize-2] == ','){
        Serial.println("Error: Empty entry");
        return false;
      }
      //TEMP ARRAY CHECK 3: WITHIN BOUNDS
      float secToFloat = atof(floatCheck);
      if (secToFloat < LOWRANGE || secToFloat > UPRANGE){
        Serial.println("Error: Value(s) out of range");
        return false;
      }
      
      //signifies end of section
      if (currentChar == ','){
        commaCounter++;
      }
      commaSeen = true;
      floatCheckCounter = 0;
      floatCheck[0] = '\0';
      
    } else {
      Serial.println("Error: Invalid characters");
      return false;
    }

    //counts next character
    dataBufCounter++;
  }

  //counts commas/entries at the end
  if (commaCounter != 5){
    Serial.println("Error: Invalid number of entries");
    return false;
  }
  return true;
}



//FUNCTION: Parse the six motor values
void parseData(char *dataStored, float *motorValues){
  
  //create a copy of dataBuffer to avoid overwriting
  char dataCopy[64];
  strncpy(dataCopy, dataStored + 3, 61);
  int index = 0;

  //create a token separated by comma
  char *token = strtok(dataCopy, ",");

  //looping through six tokens  
  while (token != NULL && index < 6){
    motorValues[index] = atof(token);
    token = strtok(NULL, ",");
    index++;
  }

  if (index == 6){
    Serial.println("Data extraction successful!");
  } else {
    Serial.println("Error.");
  }
}

//FUNCTION: Output information for debugging
void outputInfo(char *dataBuffer, char *dataStored, float *motorValues){
  Serial.print("RAW DATA: ");
  Serial.println(dataBuffer);
  Serial.print("STORED DATA: ");
  Serial.println(dataStored);

  Serial.println("MOTOR STORED: ");
  for (int count = 0; count < 6; count++){
    Serial.print(motorValues[count], 2);
    Serial.print("   ");
  }
  Serial.println();
}

void powerWheels(float *motorValues){

  Serial.println("WHEEL SPEEDS: ");
  /*
   * ##### STRUCTURE #####
   * WHEEL1
   * IN11 (Wheel Input #1)
   * IN12 (Wheel Input #2)
   * EN1  (Enable Pin #1)
   * 
   * 
   * if (negative) { //reverse direction
   *  digitalWrite(PIN1, HIGH);
   *  digitalWrite(PIN2, LOW);
   *  analogWrite(EN1, map(motors[0], 0, LOWRANGE, 0, 255)); //map value from lower range - 0 to 0 - 255
   *  Serial.print(map(motors[0], 0, LOWRANGE, 0, 255));
   *  } else if (positive){ //forward direction
   *  digitalWrite(PIN1, LOW);
   *  digitalWrite(PIN2, HIGH);
   *  analogWrite(EN1, map(motors[0], 0, UPRANGE, 0, 255)); //map value from 0 - upper range to 0 - 255
   *  Serial.print(map(motors[0], 0, UPRANGE, 0, 255));
   *  }
   */
        
  //WHEEL 1
  if (motors[0] < 0){ 
      digitalWrite(IN11, HIGH); digitalWrite(IN12, LOW);
      analogWrite(EN1, map(motors[0], 0, LOWRANGE, 0, 255));
      Serial.print(map(motors[0], 0, LOWRANGE, 0, 255));
      Serial.print(" ");     
    } else if (motors[0] >=0){
      digitalWrite(IN11, LOW); digitalWrite(IN12, HIGH);
      analogWrite(EN1, map(motors[0], 0, UPRANGE, 0, 255));
      Serial.print(map(motors[0], 0, UPRANGE, 0, 255));   
      Serial.print(" "); 
    }

   
  //WHEEL 2  
  if (motors[1] < 0){ 
      digitalWrite(IN21, HIGH); digitalWrite(IN22, LOW);
      analogWrite(EN2, map(motors[1], 0, LOWRANGE, 0, 255));
      Serial.print(map(motors[1], 0, LOWRANGE, 0, 255));  
      Serial.print(" ");  
    } else if (motors[1] >=0){
      digitalWrite(IN21, LOW); digitalWrite(IN22, HIGH);
      analogWrite(EN2, map(motors[1], 0, UPRANGE, 0, 255));
      Serial.print(map(motors[1], 0, UPRANGE, 0, 255));   
      Serial.print(" "); 
    }
 
  //WHEEL 3
  if (motors[2] < 0){ 
      digitalWrite(IN31, HIGH); digitalWrite(IN32, LOW);
      analogWrite(EN3, map(motors[0], 0, LOWRANGE, 0, 255));
      Serial.print(map(motors[2], 0, LOWRANGE, 0, 255));  
      Serial.print(" ");   
    } else if (motors[2] >=0){
      digitalWrite(IN31, LOW); digitalWrite(IN32, HIGH);
      analogWrite(EN3, map(motors[0], 0, UPRANGE, 0, 255));
      Serial.print(map(motors[2], 0, UPRANGE, 0, 255));   
      Serial.print(" ");    
    }
    
  //WHEEL 4
  if (motors[3] < 0){ 
      digitalWrite(IN41, HIGH); digitalWrite(IN42, LOW);
      analogWrite(EN4, map(motors[0], 0, LOWRANGE, 0, 255));
      Serial.print(map(motors[3], 0, LOWRANGE, 0, 255));  
      Serial.print(" ");   
    } else if (motors[0] >=0){
      digitalWrite(IN41, LOW); digitalWrite(IN42, HIGH);
      analogWrite(EN4, map(motors[0], 0, UPRANGE, 0, 255));
      Serial.print(map(motors[3], 0, UPRANGE, 0, 255));   
      Serial.print(" ");    
    }

  //WHEEL 5
  if (motors[4] < 0){ 
      digitalWrite(IN51, HIGH); digitalWrite(IN52, LOW);
      analogWrite(EN5, map(motors[0], 0, LOWRANGE, 0, 255));
      Serial.print(map(motors[4], 0, LOWRANGE, 0, 255));  
      Serial.print(" ");      
    } else if (motors[0] >=0){
      digitalWrite(IN51, LOW); digitalWrite(IN52, HIGH);
      analogWrite(EN5, map(motors[0], 0, UPRANGE, 0, 255));
      Serial.print(map(motors[4], 0, UPRANGE, 0, 255));   
      Serial.print(" ");       
    }

  //WHEEL 6
  if (motors[5] < 0){ 
      digitalWrite(IN61, HIGH); digitalWrite(IN62, LOW);
      analogWrite(EN6, map(motors[0], 0, LOWRANGE, 0, 255));
      Serial.print(map(motors[5], 0, LOWRANGE, 0, 255));  
      Serial.print(" ");     
    } else if (motors[0] >=0){
      digitalWrite(IN61, LOW); digitalWrite(IN62, HIGH);
      analogWrite(EN6, map(motors[0], 0, UPRANGE, 0, 255));
      Serial.print(map(motors[5], 0, UPRANGE, 0, 255));   
      Serial.print(" ");    
    }

    Serial.println(); 
}


//*********************************************************
//TASKS

TaskHandle_t task1Handle, task2Handle, task3Handle;

//MOTOR TASK
void motorTask(void *parameters){
  while(1){
    if (Serial.available() > 0){
      xTaskResumeFromISR(task2Handle);
      vTaskSuspend(task1Handle);
    }
    
    outputInfo(dataBuffer, dataMotors, motors);
          
    //debugging first motor
    powerWheels(motors);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
  }
}

//SERIAL TASK
void storeSerialTask (void *parameters){
  while(1){
    
    //charCount = indices, readingData returns if the data is valid and being stored
    int charCount = 0;
    bool readingData = false;
    unsigned long previousTime = millis();
    long timeInterval = 1000;
    //digitalWrite(LED_BUILTIN, LOW);
  
    //runs if serial is available and sending data
    while (Serial.available() > 0){
      //reads serial and stores character to temporary variable
      char tempChar = Serial.read();
      Serial.println(tempChar);
      //digitalWrite(LED_BUILTIN, HIGH);
      unsigned long currentTime = millis();
  
      //if start character is located, start storing data
      if (tempChar == '$'){
        readingData = true;
        charCount = 0;
        Serial.println("Starting Character Located");
      }
  
      //if data is being read, append character to string and index up
      if (readingData){
        dataBuffer[charCount] = tempChar;
        //counts for each character after '$' including '$'
        charCount++;
        Serial.println("Reading Data...");
  
        //if the temp character is ')', stop reading data and null-terminate
        if (tempChar == ')'){
          readingData = false;
          //digitalWrite(LED_BUILTIN, LOW);
          dataBuffer[charCount] = '\0';
          Serial.println("Not Reading Data...");
          xTaskResumeFromISR(task3Handle);
          vTaskSuspend(task2Handle);
        }
      }
  
      //Serial receiving taking too long
      if (currentTime - previousTime > timeInterval){
        readingData = false;
        charCount = 0;
        Serial.println("Error: Serial taking too long");
        xTaskResumeFromISR(task3Handle);
        vTaskSuspend(task2Handle);
        break;
      }
  
      //Buffer overflow protection
      if (charCount >= sizeof(dataBuffer)- 1){
        readingData = false;
        charCount = 0;
        Serial.println("Error: Buffer exceeded limit");
        xTaskResumeFromISR(task3Handle);
        vTaskSuspend(task2Handle);
      }
    } 
  }
}

//UPDATE MOTOR TASK
void updateMotorsTask (void *parameters){
  while (1){
    int actualSize = strlen(dataBuffer);
    //PERFORM ERROR-CHECKING FOR VALIDITY
    if (checkSize(dataBuffer, actualSize) && checkSequence(dataBuffer, actualSize) && checkValid(dataBuffer, actualSize)) {
    
      //if valid, stores buffer to data and parses to respective motors  
      memcpy(dataMotors, dataBuffer, 64); 
      Serial.println("Valid data. Stored in memory.");
      parseData(dataMotors, motors);
    } 
    
    xTaskResumeFromISR(task1Handle); 
    vTaskSuspend(task3Handle);
  }
}


void setup(){
  //MOTORS
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(EN3, OUTPUT);
  pinMode(EN4, OUTPUT);
  pinMode(EN5, OUTPUT);
  pinMode(EN6, OUTPUT);
  pinMode(IN11, OUTPUT);
  pinMode(IN12, OUTPUT);
  pinMode(IN21, OUTPUT);
  pinMode(IN22, OUTPUT);
  pinMode(IN31, OUTPUT);
  pinMode(IN32, OUTPUT);
  pinMode(IN41, OUTPUT);
  pinMode(IN42, OUTPUT);
  pinMode(IN51, OUTPUT);
  pinMode(IN52, OUTPUT);
  pinMode(IN61, OUTPUT);
  pinMode(IN62, OUTPUT);
  // Set initial rotation direction
  digitalWrite(IN11, LOW);
  digitalWrite(IN12, HIGH);
  digitalWrite(IN21, LOW);
  digitalWrite(IN22, HIGH);
  digitalWrite(IN31, LOW);
  digitalWrite(IN32, HIGH);
  digitalWrite(IN41, LOW);
  digitalWrite(IN42, HIGH);
  digitalWrite(IN51, LOW);
  digitalWrite(IN52, HIGH);
  digitalWrite(IN61, LOW);
  digitalWrite(IN62, HIGH);
  //pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  Serial.println("------MINI ROVER FIRMWARE TEST------");

  //Create Motor Control Task
  xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
            motorTask,      // Function to be called
            "Control Motors",   // Name of task
            2048,           // Stack size (bytes in ESP32, words in FreeRTOS)
            NULL,           // Parameter to pass
            1,              // Task priority
            &task1Handle,   // Task handle
            app_cpu);       // Run on one core for demo purposes (ESP32 only)
            
  //Create Serial Read Task
  xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
            storeSerialTask,     // Function to be called
            "Read Serial",  // Name of task
            2048,           // Stack size (bytes in ESP32, words in FreeRTOS)
            NULL,           // Parameter to pass
            1,              // Task priority
            &task2Handle,   // Task handle
            app_cpu);       // Run on one core for demo purposes (ESP32 only)
  //Suspends task on startup
  vTaskSuspend(task2Handle); 
  //Create Update Motors Task
  xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
            updateMotorsTask,     // Function to be called
            "Update Motors",  // Name of task
            2048,           // Stack size (bytes in ESP32, words in FreeRTOS)
            NULL,           // Parameter to pass
            1,              // Task priority
            &task3Handle,   // Task handle
            app_cpu);       // Run on one core for demo purposes (ESP32 only)
  //Suspends task on startup
  vTaskSuspend(task3Handle); 
  
  //Delete "setup and loop" task
  vTaskDelete(NULL);
}

void loop(){
  //execution never gets here
}