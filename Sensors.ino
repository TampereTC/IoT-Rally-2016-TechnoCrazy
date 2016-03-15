



void JsonReportSensorDHT() {
 
  // DHT functions
  // Reading temperature or humidity takes about 250 milliseconds!
  float h = dht.readHumidity();  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
    float f = dht.readTemperature(true);
    // Compute heat index in Fahrenheit (the default)
    float hif = dht.computeHeatIndex(f, h);
    // Compute heat index in Celsius (isFahreheit = false)
    float hic = dht.computeHeatIndex(t, h, false);
    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t) || isnan(f)) {
        // Debugging
        //Serial.println("Failed to read from DHT sensor!");
        return;
    }
    // Create Json for sensor print out
    StaticJsonBuffer<512> jsonOutBuffer;   // 514 B
    String rootJson = ""; String arrayJson = "";
    JsonObject& root = jsonOutBuffer.createObject();
    root["sensor"] = "temp_hum"; root["time"] =  TimeStr; JsonArray& array = jsonOutBuffer.createArray();
    array.add(t); array.add(h);
    // Print to Serial
    root.printTo(rootJson); array.printTo(arrayJson); String JointJson = rootJson + ":" + arrayJson + "}";
    //Serial1.println(JointJson);
    //mqtt.publish("iot-rally", JointJson);
    memset(bufff, 0, 512);
  JointJson.toCharArray(bufff, 512);
  mqtt.publish("iot-uplink", bufff);
    return;
}

void JsonReportSensorDistance(){

  StaticJsonBuffer<512> jsonOutBuffer;   // 514 B
  String rootJson = ""; String arrayJson = "";
  JsonObject& root = jsonOutBuffer.createObject();
  root["sensor"] = "distance"; root["time"] =  TimeStr; JsonArray& array = jsonOutBuffer.createArray();
  array.add(ultrasonic.Ranging(CM)); // CM or INC
    // Debugging
    //Serial.print(ultrasonic.Ranging(CM)); // CM or INC
    //Serial.println(" cm" ); 
  root.printTo(rootJson); array.printTo(arrayJson); String JointJson = rootJson + ":" + arrayJson + "}";
    // Debugging
    //Serial.println("json string for edge:" + JointJson);
  //Serial1.println(JointJson);
    memset(bufff, 0, 512);
  JointJson.toCharArray(bufff, 512);
  mqtt.publish("iot-uplink", bufff);
  return;
}  
/*
 *   Read accelerometer and gyroscope raw values and send them
 *   to ESP
 * 
 */
void JsonReportSensorAccAndGyro(){
  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Construct json
  StaticJsonBuffer<512> jsonOutBuffer;   // 514 B
  String rootJson = ""; 
  String arrayJson = "";
  JsonObject& root = jsonOutBuffer.createObject();
  
  root["sensor"] = "acc_gyro"; 
  root["time"] =  TimeStr;
  JsonArray& array = jsonOutBuffer.createArray();
  array.add( ax );
  array.add( ay );
  array.add( az );
  array.add( gx );
  array.add( gy );
  array.add( gz );
  
  root.printTo(rootJson); 
  array.printTo(arrayJson); 
  String JointJson = rootJson + ":" + arrayJson + "}";
  
  // Debuggung
  //Serial.println("json string for edge:" + JointJson);
  
  // Send to ESP
  //Serial1.println(JointJson);
    memset(bufff, 0, 512);
  JointJson.toCharArray(bufff, 512);
  mqtt.publish("iot-uplink", bufff);
}

void JsonReportSensorMagneto(){
  // read raw accel/gyro measurements from device
  magneto.getHeading(&mx, &my, &mz);

  // Construct json
  StaticJsonBuffer<512> jsonOutBuffer;   // 514 B
  String rootJson = ""; 
  String arrayJson = "";
  JsonObject& root = jsonOutBuffer.createObject();
  
  root["sensor"] = "magneto"; 
  root["time"] =  TimeStr;
  JsonArray& array = jsonOutBuffer.createArray();
  array.add( mx );
  array.add( my );
  array.add( mz );
  
  root.printTo(rootJson); 
  array.printTo(arrayJson); 
  String JointJson = rootJson + ":" + arrayJson + "}";
  
  // Debuggung
  //Serial.println("json string for edge:" + JointJson);
  
  // Send to ESP
  //Serial1.println(JointJson);
    memset(bufff, 0, 512);
  JointJson.toCharArray(bufff, 512);
  mqtt.publish("iot-uplink", bufff);
}

void JsonReportSensorEdge() {
   StaticJsonBuffer<512> jsonOutBuffer;   // 514 B
  String rootJson = ""; String arrayJson = "";
  JsonObject& root = jsonOutBuffer.createObject();
  root["sensor"] = "edge"; 
  root["time"] =  TimeStr;
  JsonArray& array = jsonOutBuffer.createArray();
  array.add( digitalRead(left_edge) );
  array.add( digitalRead(right_edge) );
  
  root.printTo(rootJson); array.printTo(arrayJson); String JointJson = rootJson + ":" + arrayJson + "}";
  // Debuggung
  //Serial.println("json string for edge:" + JointJson);
  //Serial1.println(JointJson);
    memset(bufff, 0, 512);
  JointJson.toCharArray(bufff, 512);
  mqtt.publish("iot-uplink", bufff);
  return;
}  

void JsonReportSensorRFID() {
  if ( ! rfid.PICC_ReadCardSerial()) 
    return;
  Serial.print(F("PICC type: "));
  MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);
  Serial.println(rfid.PICC_GetTypeName(piccType));
  // Check is the PICC of Classic MIFARE type
  if (piccType != MFRC522::PICC_TYPE_MIFARE_MINI &&  
    piccType != MFRC522::PICC_TYPE_MIFARE_1K &&
    piccType != MFRC522::PICC_TYPE_MIFARE_4K) {
    Serial.println(F("Your tag is not of type MIFARE Classic."));
    return;
  }
  if (rfid.uid.uidByte[0] != nuidPICC[0] || 
    rfid.uid.uidByte[1] != nuidPICC[1] || 
    rfid.uid.uidByte[2] != nuidPICC[2] || 
    rfid.uid.uidByte[3] != nuidPICC[3] ) {
    Serial.println(F("A new card has been detected."));
    // Define Json for sensor print out
    StaticJsonBuffer<512> jsonOutBuffer;   // 514 B
    String rootJson = "";
    String arrayJson = "";
    JsonObject& root = jsonOutBuffer.createObject();
    root["sensor"] = "rfid";
    JsonArray& array = jsonOutBuffer.createArray();
    // Store NUID into nuidPICC array
    for (byte i = 0; i < 4; i++) {
      nuidPICC[i] = rfid.uid.uidByte[i];
      }   
    //Serial.println(F("The NUID tag is:"));
    //Serial.print(F("In dec: "));
    byte *buffer = rfid.uid.uidByte; 
    byte bufferSize = rfid.uid.size;
    String NUID = "";
    for (byte i = 0; i < bufferSize; i++) {
      //Serial.print(buffer[i] < 0x10 ? " 0" : " ");
      //Serial.print(buffer[i], DEC);
      NUID = String(NUID + String(buffer[i], DEC)); 
      }
    array.add(NUID);
    // Print json string to Serial1
    root.printTo(rootJson);
    array.printTo(arrayJson);
    String JointJson = rootJson + ":" + arrayJson + "}";
    //Serial.println("json string for rfid:" + JointJson);
    memset(bufff, 0, 512);
    JointJson.toCharArray(bufff, 512);
    mqtt.publish("iot-uplink", bufff);
    }
  else Serial.println(F("Card read previously."));
  rfid.PICC_HaltA(); // Halt PICC
  rfid.PCD_StopCrypto1(); // Stop encryption on PCD
  return;  
}

