#define CoilA 2
#define CoilB 3
void setup() 
{
}

void  loop() 
{
  digitalWrite(CoilB, HIGH);        // Start Sequenz
  delay(5000);
  digitalWrite(CoilB, LOW);        // Start Sequenz
  while(1) {}
}

