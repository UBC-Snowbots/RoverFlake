//GLOBAL DATA
char dataBuf[64] = {0};
char data[64] = {0};
float motors[6] = {0.0f};
#define ENA 9
#define IN1 6
#define IN2 7
#define LOWRANGE -250
#define UPRANGE 250

//edge cases

//FUNCTION: Take serial input and store a specified section to a character array
void storeSerialData(char *dataBuffer, int bufferSize){
  
  //charCount = indices, readingData returns if the data is valid and being stored
  int charCount = 0;
  bool readingData = false;
  unsigned long previousTime = millis();
  long timeInterval = 1000;

  //runs if serial is available and sending data
  while (Serial.available() > 0){
    //reads serial and stores character to temporary variable
    char tempChar = Serial.read();
    Serial.println(tempChar);
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
        dataBuffer[charCount] = '\0';
        Serial.println("Not Reading Data...");
      }
    }

    //Serial receiving taking too long
    if (currentTime - previousTime > timeInterval){
      readingData = false;
      charCount = 0;
      Serial.println("Error: Serial taking too long");
      break;
    }

    //Buffer overflow protection
    if (charCount >= bufferSize - 1){
      readingData = false;
      charCount = 0;
      Serial.println("Error: Buffer exceeded limit");
    }
  } 

}

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

  Serial.println("Motors 1 through 6: ");
  for (int count = 0; count < 6; count++){
    Serial.print(motorValues[count], 2);
    Serial.print("   ");
  }
  Serial.println();
}



void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  // Set initial rotation direction
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  
  Serial.begin(9600);
  delay(100);
}

void loop() {

  //STORE SERIAL INPUT TO BUFFER
  storeSerialData(dataBuf, sizeof(dataBuf));
  //variable to determine buffer size
  int actualSize = strlen(dataBuf);
  
  //PERFORM ERROR-CHECKING FOR VALIDITY
  if (checkSize(dataBuf, actualSize) && checkSequence(dataBuf, actualSize) && checkValid(dataBuf, actualSize)) {
    
    //if valid, stores buffer to data and parses to respective motors  
    memcpy(data, dataBuf, 64); 
    Serial.println("Valid data. Stored in memory.");
    parseData(data, motors);
  } 

  //OUTPUT SYSTEM INFORMATION FOR DEBUGGING
  outputInfo(dataBuf, data, motors);

  //TESTING MOTOR 1
  if (motors[0] < 0){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, map(abs(motors[0]), 0, 250, 1, 255));
    Serial.println(map(motors[0], 0, 250, 1, 255));
  } else if (motors[0] >=0){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, map(motors[0], 0, 250, 1, 255));
    Serial.println(map(motors[0], 0, 250, 1, 255));
  }
 
  
}