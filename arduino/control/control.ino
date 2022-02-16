#define SAMPLE_TIME 10

int voltage = 0;
int sleep = 1000;
unsigned long time_now = 0;

void setup(){
  Serial.begin(115200);
}

void loop() {
  
  if(millis() >= sleep){
    voltage = 1;
    Serial.println("Stepping voltage");
  }
  
  if(millis() >= time_now + SAMPLE_TIME){
    time_now += SAMPLE_TIME;
    sample();
    if(millis() > time_now+SAMPLE_TIME) Serial.println("Uh oh?");
  }
}

void sample(){
  // do stuff
  Serial.println("Ello govna");
}
