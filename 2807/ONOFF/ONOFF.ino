// Definiciones de pines de motores
#define m1 6  // Right Motor PIN_MOTOR_R_1
#define m2 7  // Right Motor PIN_MOTOR_R_2
#define m3 2  // Left Motor PIN_MOTOR_L_1
#define m4 4  // Left Motor PIN_MOTOR_L_2
#define e1 5  // Right Motor Enable PIN_MOTOR_R_PWM
#define e2 3  // Left Motor Enable PIN_MOTOR_L_PWM

// Definiciones de pines de sensores
#define ir0 17
#define ir1 A0
#define ir2 A1
#define ir3 A2
#define ir4 A3
#define ir5 A4
#define ir6 A5
#define ir7 18

// Variable de velocidad
int velocidad = 150; // Puedes ajustar la velocidad entre 0 y 255

void setup() {
  Serial.begin(9600);
  Serial.println("STARTING SERIAL");
  Serial.println("==========================================");

  // Configuración de pines de motores
  pinMode(m1, OUTPUT);
  pinMode(m2, OUTPUT);
  pinMode(m3, OUTPUT);
  pinMode(m4, OUTPUT);
  pinMode(e1, OUTPUT);
  pinMode(e2, OUTPUT);

  // Configuración de pines de sensores
  pinMode(ir0, INPUT);
  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  pinMode(ir3, INPUT);
  pinMode(ir4, INPUT);
  pinMode(ir5, INPUT);
  pinMode(ir6, INPUT);
  pinMode(ir7, INPUT);
}

void loop() {
  // Lectura de valores de sensores
  int s0 = digitalRead(ir0);  // Left Most Sensor
  int s1 = digitalRead(ir1);  // Left Sensor
  int s2 = digitalRead(ir2);  // Left Center Sensor
  int s3 = digitalRead(ir3);  // Center Left Sensor
  int s4 = digitalRead(ir4);  // Center Right Sensor
  int s5 = digitalRead(ir5);  // Right Center Sensor
  int s6 = digitalRead(ir6);  // Right Sensor
  int s7 = digitalRead(ir7);  // Right Most Sensor



  Serial.print("Sensors: ");
  Serial.print(s0);
  Serial.print(" ");
  Serial.print(s1);
  Serial.print(" ");
  Serial.print(s2);
  Serial.print(" ");
  Serial.print(s3);
  Serial.print(" ");
  Serial.print(s4);
  Serial.print(" ");
  Serial.print(s5);
  Serial.print(" ");
  Serial.print(s6);
  Serial.print(" ");
  Serial.println(s7);

  // Lógica de control del seguidor de línea

   if (s3 == 0 || s4 == 0) {
    // Avanzar cuando el sensor central izquierdo detecta la línea
    analogWrite(e1, velocidad);
    analogWrite(e2, velocidad);
    digitalWrite(m1, LOW);   
    digitalWrite(m2, HIGH);  
    digitalWrite(m3, LOW);   
    digitalWrite(m4, HIGH);  
  } else if ((s2 == 0) || (s1 == 0)) {
    // Girar ligeramente a la derecha
    analogWrite(e1, velocidad);
    analogWrite(e2, velocidad / 2);
    digitalWrite(m1, LOW);   
    digitalWrite(m2, HIGH);  
    digitalWrite(m3, LOW);  
    digitalWrite(m4, HIGH);   
  } else if (s0 == 0) {
    // Girar fuertemente a la derecha
    analogWrite(e1, velocidad);
    analogWrite(e2, 0);
    digitalWrite(m1, LOW);   
    digitalWrite(m2, HIGH);  
    digitalWrite(m3, LOW);  
    digitalWrite(m4, HIGH);   
  } else if ((s5 == 0) || (s6 == 0)) {
    // Girar ligeramente a la izquierda
    analogWrite(e1, velocidad / 2);
    analogWrite(e2, velocidad);
    digitalWrite(m1, LOW);   
    digitalWrite(m2, HIGH);  
    digitalWrite(m3, LOW);   
    digitalWrite(m4, HIGH);  
  } else if (s7 == 0) {
    // Girar fuertemente a la izquierda
    analogWrite(e1, 0);
    analogWrite(e2, velocidad);
    digitalWrite(m1, LOW);   
    digitalWrite(m2, HIGH);  
    digitalWrite(m3, LOW);   
    digitalWrite(m4, HIGH);  
  } else {
    // Detenerse si no se detecta la línea
    digitalWrite(m1, LOW);
    digitalWrite(m2, LOW);
    digitalWrite(m3, LOW);
    digitalWrite(m4, LOW);
  }

  delay(200);
}