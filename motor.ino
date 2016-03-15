void driveMotors()
{
  Serial.println(__func__);
  
    // this function will run the motors across the range of possible speeds
    // note that maximum speed is determined by the motor itself and the operating voltage
    // the PWM values sent by analogWrite() are fractions of the maximum speed possible by your hardware
      // Drive Forward  
      if (course == 1) {
        digitalWrite(in1, HIGH); digitalWrite(in2, LOW); digitalWrite(in3, HIGH); digitalWrite(in4, LOW); 
        analogWrite(enA, mspeed); analogWrite(enB, mspeed);
      }
      // Drive Backward
      else if (course == 2 ) {
        digitalWrite(in1, LOW); digitalWrite(in2, HIGH); digitalWrite(in3, LOW); digitalWrite(in4, HIGH); 
        analogWrite(enA, mspeed); analogWrite(enB, mspeed);
      }
      // Turn Right, direction forward
      else if (course == 3 ) {
        digitalWrite(in1, LOW); digitalWrite(in2, HIGH); digitalWrite(in3, HIGH); digitalWrite(in4, LOW); 
        analogWrite(enA, mspeed); analogWrite(enB, mspeed);
      }
      // Turn Left, direction forward
      else if (course == 4 ) {
        digitalWrite(in1, HIGH); digitalWrite(in2, LOW); digitalWrite(in3, LOW); digitalWrite(in4, HIGH); 
        analogWrite(enA, mspeed); analogWrite(enB, mspeed);
      }
      // Stop
      else if (course == 0) {
        // now turn off motors
        digitalWrite(in1, HIGH); digitalWrite(in2, HIGH); digitalWrite(in3, HIGH); digitalWrite(in4, HIGH);
        delay(100);
        analogWrite(enA, 0); analogWrite(enB, 0);
      }  
}

void stopMotors() {
      // now turn off motors
    // clean up & return
    digitalWrite(in1, HIGH); digitalWrite(in2, HIGH); digitalWrite(in3, HIGH); digitalWrite(in4, HIGH);
    delay(100);
    analogWrite(enA, 0); analogWrite(enB, 0);
    course = NULL;
    mspeed = NULL;
    mdelay = NULL;
    motor_active = false;
    // return;
}

int fix_angle(int delta) {
  Serial.println(delta);

  if (delta < 5)
    return 0;
    
  if (delta > 0) {
    turn_left();
  }
  
  if (delta < 0) {
    turn_right();
  }
  
  return 1;
}

int get_angle(void) {
  return 15;
}

int get_angle_delta(int orig_angle) {
  static int debug_error = 20;
    
  return orig_angle - get_angle() + (debug_error--);
}

void turn_left(void) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW); digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
    analogWrite(enA, 255); analogWrite(enB, 255);
    delay(100);
    digitalWrite(in1, HIGH); digitalWrite(in2, HIGH); digitalWrite(in3, HIGH); digitalWrite(in4, HIGH);
    delay(100);
    analogWrite(enA, 0); analogWrite(enB, 0);
}

void turn_right(void) {
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH); digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
    analogWrite(enA, 255); analogWrite(enB, 255);
    delay(100);
    digitalWrite(in1, HIGH); digitalWrite(in2, HIGH); digitalWrite(in3, HIGH); digitalWrite(in4, HIGH);
    delay(100);
    analogWrite(enA, 0); analogWrite(enB, 0);
}

