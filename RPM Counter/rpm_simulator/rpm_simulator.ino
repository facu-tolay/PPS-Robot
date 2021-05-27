#define SIGNAL_OUT 6
#define DESIRED_RPM 254
#define CANT_RANURAS 24
#define PULSE_LENGTH 1000 // in microseconds

long int cant_pulsos;
float periodo;

int microseg;
int effective_delay_us;
int effective_delay_ms;

void setup() {
  Serial.begin(9600);
  pinMode(SIGNAL_OUT, OUTPUT);
  
  cant_pulsos = (DESIRED_RPM * CANT_RANURAS) / 60; // me da cuantos tengo que hacer en 1 seg
  periodo = 1.0/cant_pulsos;
  Serial.print("Pulsos por segundo: ");
  Serial.println(cant_pulsos);
  Serial.print("Periodo: ");
  Serial.println(periodo, 10);
  
  microseg = periodo * 1000000;
  Serial.print("Tiempo entre pulsos [us]: ");
  Serial.println(microseg, 10); 
  Serial.println("---------");

  effective_delay_us = microseg - PULSE_LENGTH;

  if(effective_delay_us > 15000)
  {
    // use miliseconds
    effective_delay_ms = effective_delay_us / 1000.0;
  }
  else
  {
    effective_delay_ms = 0;
  }
}

void loop()
{
  if(effective_delay_ms!=0)
  {
      digitalWrite(SIGNAL_OUT, HIGH);
      delayMicroseconds(PULSE_LENGTH);
      digitalWrite(SIGNAL_OUT, LOW);
      delay(effective_delay_ms);
  }
  else
  {
    digitalWrite(SIGNAL_OUT, HIGH);
    delayMicroseconds(PULSE_LENGTH);
    digitalWrite(SIGNAL_OUT, LOW);
    delayMicroseconds(effective_delay_us);  
  }
}
