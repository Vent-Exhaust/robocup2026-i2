#include <Arduino.h>

#define Mux 19
#define S0 14
#define S1 15
#define S2 16
#define S3 17


int Binary(int n){
  int binaryNum[32]; // Max bits for int
  int i = 0;
  if (n == 0) { // Special case
      return 0;
  }
  while (n > 0) {
      binaryNum[i] = n % 2; // Store remainder
      n = n / 2; // Update number
      i++;
  }
  for (int j = i - 1; j >= 0; j--) {
      Serial.print(binaryNum[j]);
  }

  Serial.println();

  return 0;
}
void setup() {
  Serial.begin(9600);
  pinMode(Mux, INPUT);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
}

void loop() {

  // for(int i = 0; i < 16; i++){
  //   digitalWrite(S0, 1);
  //   digitalWrite(S1, 0);
  //   digitalWrite(S2, 0);
  //   digitalWrite(S3, 0);
  //   Serial.println(analogRead(Mux));
  // }
  Binary(15);
}
