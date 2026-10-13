#include <Wire.h>
#include <stdio.h>
#include <NHD_Character_LCD.h>

#define TEMP A1
#define RELAY 13
#define BUTTON 16 // and 17
#define AREF 5.18 //usb: 5.19 Power source @ 5.8: 5.03
#define ADC_RES 10



NHD_Character_LCD lcd;

//vars
float temp;
int target_temp = 0;
float prev_error;
unsigned long prev_time, reflow_clock;
int buttonState = 0;
int test = 0;
unsigned long cooldown_start = 0;

unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_INTERVAL_MS = 300;
int lastButtonState = -1;

//PID constants (TBD)
float kp = 25; float ki = .01; float kd = 500.0;
float PID_p = 0; float PID_i = 0; float PID_d = 0;

const unsigned long WINDOW_MS = 1000;
unsigned long windowStartTime;

//print value to LCD
void update_display_heating(float temp, int set_temp){
  char str[8];
  char row0[17];
  char row1[17];

  dtostrf(temp,5,1,str);
  
  snprintf(row0, sizeof(row0), "HEATING  %5s%cC", str, (char)223);
  snprintf(row1, sizeof(row1), "Target:  %3d%cC", set_temp, (char)223);

  lcd.write(0,0,(unsigned char*)row0);
  lcd.write(0,1,(unsigned char*)row1);
}

// read hotplate temp and convert to digital value
float get_temp() {
  static float filtered_temp=15.5; //may need adjusted

  float alpha=.1;


  int val = analogRead(TEMP);
  float voltage = val*(AREF/(pow(2, ADC_RES)-1));
  float raw_temp = (voltage-1.25)/0.005;
  //exponential moving average filter & toss out outliers

  if (abs(raw_temp - filtered_temp) < 10.0) {
    filtered_temp = (alpha * raw_temp) + ((1.0 - alpha) * filtered_temp);
  }
  return filtered_temp;
}

// PID control function
unsigned long PID(float temp, int set_temp) {
  unsigned long now = millis();
  float elapsed_time = (now-prev_time)/1000.0;
  prev_time = now;

  float PID_error = set_temp - temp;

  PID_p = kp*PID_error;

  if(PID_error > -10 && PID_error < 10) {
    PID_i += (ki*PID_error*elapsed_time);
  } else{
    PID_i =0;
  }

  if (elapsed_time > 0) {
    PID_d = kd*((PID_error - prev_error)/elapsed_time);
  } 
  
  long PID_value = (long)(PID_p + PID_i + PID_d);
  PID_value = constrain(PID_value, 0, WINDOW_MS);

  prev_error = PID_error;

  return (unsigned long)PID_value;
}

// change RGB LED based on current temp
void temp_led(float temp) {
  if (temp >= 150) {
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDR, LOW);
    digitalWrite(LEDB, HIGH);
  } else if (temp > 100) {
    digitalWrite(LEDB, HIGH);
    digitalWrite(LEDG, LOW);
    digitalWrite(LEDR, HIGH);
  } else if (temp > 50) {
    digitalWrite(LEDB, LOW);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDR, HIGH);
  }
}

// click to start
void button_logic(void) {
  buttonState = digitalRead(BUTTON);
  if (buttonState != lastButtonState) {
    lcd.clearScreen();
    lastButtonState = buttonState;

    if (buttonState == HIGH) {
      test = 1;
      reflow_clock = millis();
      
    }
    if (buttonState == LOW) {
      lcd.write(0, 0, (unsigned char*)"     Hello      ");
      lcd.write(0, 1, (unsigned char*)"I am Hamilton :)");
    }
  }
}

// serial output for debugging
void temp_log(float temp) {
  Serial.print(millis());
  Serial.print(",");
  Serial.print(temp, 3);
  Serial.print("\n");
}


void handle_temp(void) {
  temp = get_temp();
  temp_log(temp);
  temp_led(temp);

  //update temp on display
  if (millis() - lastDisplayUpdate >= DISPLAY_INTERVAL_MS) {
  update_display_heating(temp, target_temp); //add general update display with heatin/cooling passed as var
  lastDisplayUpdate = millis();
  }
}

// temp control for reflow
int set_temp(void){
  
  static int last_phase =0;
  const float H = 2.0;
  int phase=last_phase;

  if (last_phase==0) {
    if (temp <= 150+H) {
      phase =0;
      target_temp = 150;
      kp = 20; ki = 0.008; kd = 0;
    } else {
      phase =1;
      target_temp=175;
      kp = 20; ki = 0.010; kd = 0;
    }
  } else if(last_phase==1) {
    if (temp<=175+H) {
      phase=1;
      target_temp=175;
      kp = 16; ki = 0.006; kd = 0;
    } else {
      phase=2;
      target_temp=223;
      kp = 25; ki = 0.010; kd = 0;
    }
  } else if(last_phase==2) {
    if(temp<=223) {
      phase=2;
      target_temp=223;
      kp = 25; ki = 0.010; kd = 0;
    } else {
      test=2;
      cooldown_start = millis();
      digitalWrite(RELAY, LOW);
      return target_temp;
    }
  }
  if (phase!=last_phase) {
    PID_i=0;
    prev_error=0;
    last_phase=phase;
  }
  return target_temp;
}

// Turn hotplate on
void ssr_on(void) {
  digitalWrite(RELAY, HIGH);
  //Serial.print("high"); //debug
  digitalWrite(LED_BUILTIN, LOW);
}

// Turn hotplate off
void ssr_off(void) {
  digitalWrite(RELAY,LOW);
  //Serial.print("low"); //debug
  digitalWrite(LED_BUILTIN, HIGH);
}

void ssr(unsigned long windowStartTime, unsigned long onTime) {
  if ((millis() - windowStartTime) < onTime) {
      ssr_on();

    } else{
      ssr_off();
    }
}

void setup() {
  //init LEDS
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TEMP, INPUT);
  pinMode(RELAY, OUTPUT);
  pinMode(BUTTON, INPUT);

  windowStartTime = millis();
  prev_time = millis();

  Serial.begin(115200);
  

  //start off
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);
  digitalWrite(LED_BUILTIN, HIGH);

  digitalWrite(RELAY,LOW);
  lcd.initLCD(16, 2, 8, 6, 7, 3, 2, 0, 1);

}


void loop() {
  // put your main code here, to run repeatedly:
  button_logic();

  if (test == 1) {
    handle_temp();
    target_temp = set_temp();

    unsigned long onTime = 0;
    
    if (temp<=120) {
      PID_i = 0;
      prev_error=0;
      ssr_on();
    } else {
      onTime = PID(temp, target_temp);

      if ((millis() - windowStartTime) >= WINDOW_MS) {
      windowStartTime = millis();
      }

      ssr(windowStartTime, onTime);
    }

  } else if (test == 2) {
    digitalWrite(RELAY, LOW);
    temp = get_temp();
    temp_log(temp);
    temp_led(temp);

    if (millis() - lastDisplayUpdate >= DISPLAY_INTERVAL_MS) {
      char str[5];
      char row0[17];
      dtostrf(temp, 5, 1, str);
      snprintf(row0, sizeof(row0), "COOLING  %5s%cC", str, (char)223);
      lcd.write(0, 0, (unsigned char*)row0);
      lcd.write(0, 1, (unsigned char*)"Have A Good Day!");
      lastDisplayUpdate = millis();
    }

    if ((millis() - cooldown_start > 300000UL) || temp < 50) {
      test = 0;
      lcd.clearScreen();
      lcd.write(0, 0, (unsigned char*)"     Hello      ");
      lcd.write(0, 1, (unsigned char*)"I am Hamilton :)");
      
    }
  } else {
    digitalWrite(RELAY, LOW);
  }
}

