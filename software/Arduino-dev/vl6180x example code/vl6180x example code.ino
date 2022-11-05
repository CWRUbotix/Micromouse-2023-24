#include <Wire.h>
#include "Adafruit_VL6180X.h"

Adafruit_VL6180X vl = Adafruit_VL6180X();

void setup() {
  Serial.begin(115200);

  // wait for serial port to open on native usb devices
  while (!Serial) {
    delay(1);
  }
  
  Serial.println("Adafruit VL6180x test!");
  if (! vl.begin()) {
    Serial.println("Failed to find sensor");
    while (1);
  }
  Serial.println("Sensor found!");
}

void loop() {
  Serial.print(getDistance());
  Serial.print(" " + defineError());
  Serial.println();
 /* uint8_t status;
  uint8_t i;
  if ((status = getDistance(&i)) == VL6180X_ERROR_NONE) {
    Serial.print("Range: "); Serial.println(i);
  }
  else {
  // Some error occurred, print it out!
  
    if  ((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5)) {
      Serial.println("System error");
    }
    else if (status == VL6180X_ERROR_ECEFAIL) {
      Serial.println("ECE failure");
    }
    else if (status == VL6180X_ERROR_NOCONVERGE) {
      Serial.println("No convergence");
    }
    else if (status == VL6180X_ERROR_RANGEIGNORE) {
      Serial.println("Ignoring range");
    }
    else if (status == VL6180X_ERROR_SNR) {
      Serial.println("Signal/Noise error");
    }
    else if (status == VL6180X_ERROR_RAWUFLOW) {
      Serial.println("Raw reading underflow");
    }
    else if (status == VL6180X_ERROR_RAWOFLOW) {
      Serial.println("Raw reading overflow");
    }
    else if (status == VL6180X_ERROR_RANGEUFLOW) {
      Serial.println("Range reading underflow");
    }
    else if (status == VL6180X_ERROR_RANGEOFLOW) {
      Serial.println("Range reading overflow");
    }
  }*/
      

    /*float lux = vl.readLux(VL6180X_ALS_GAIN_5);

    Serial.print("Lux: "); Serial.println(lux);
    
    uint8_t range = vl.readRange();
    uint8_t status = vl.readRangeStatus();

    if (status == VL6180X_ERROR_NONE) {
      Serial.print("Range: "); Serial.println(range);
    }

    // Some error occurred, print it out!
  
  if  ((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5)) {
    Serial.println("System error");
  }
  else if (status == VL6180X_ERROR_ECEFAIL) {
    Serial.println("ECE failure");
  }
  else if (status == VL6180X_ERROR_NOCONVERGE) {
    Serial.println("No convergence");
  }
  else if (status == VL6180X_ERROR_RANGEIGNORE) {
    Serial.println("Ignoring range");
  }
  else if (status == VL6180X_ERROR_SNR) {
    Serial.println("Signal/Noise error");
  }
  else if (status == VL6180X_ERROR_RAWUFLOW) {
    Serial.println("Raw reading underflow");
  }
  else if (status == VL6180X_ERROR_RAWOFLOW) {
    Serial.println("Raw reading overflow");
  }
  else if (status == VL6180X_ERROR_RANGEUFLOW) {
    Serial.println("Range reading underflow");
  }
  else if (status == VL6180X_ERROR_RANGEOFLOW) {
    Serial.println("Range reading overflow");
  } */
  delay(50);
} 


String defineError() {
  uint8_t status = vl.readRangeStatus();
   if (status == VL6180X_ERROR_NONE) {
     return "No error";
  }
 
  // Some error occurred, print it out!
  
  if  ((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5)) {
    return "System error";
  }
  else if (status == VL6180X_ERROR_ECEFAIL) {
    return "ECE failure";
  }
  else if (status == VL6180X_ERROR_NOCONVERGE) {
    return "No convergence";
  }
  else if (status == VL6180X_ERROR_RANGEIGNORE) {
    return "Ignoring range";
  }
  else if (status == VL6180X_ERROR_SNR) {
    return "Signal/Noise error";
  }
  else if (status == VL6180X_ERROR_RAWUFLOW) {
    return "Raw reading underflow";
  }
  else if (status == VL6180X_ERROR_RAWOFLOW) {
    return "Raw reading overflow";
  }
  else if (status == VL6180X_ERROR_RANGEUFLOW) {
    return "Range reading underflow";
  }
  else if (status == VL6180X_ERROR_RANGEOFLOW) {
    return "Range reading overflow";
  } 
}


uint8_t getDistance() {
  uint8_t status = vl.readRangeStatus();
    if (status == VL6180X_ERROR_NONE) {
     return vl.readRange();
    }
   }

 /* uint8_t getDistance(uint8_t *dist) {
    uint8_t status = vl.readRangeStatus();
    if (status == VL6180X_ERROR_NONE) {
     *dist = vl.readRange();
    }
    Serial.println(*dist);
    Serial.println(status);
    return status;
   }*/
