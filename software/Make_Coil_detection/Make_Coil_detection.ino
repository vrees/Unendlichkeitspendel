#define CoilA 2
#define CoilB 3
void setup() 
{
    //pinMode(CoilA, OUTPUT);
    pinMode(CoilB, OUTPUT);
}

void  loop() 
{
//  digitalWrite(CoilA, HIGH);        // Start Sequenz
  digitalWrite(CoilB, HIGH);        // Start Sequenz
  delay(10000);
//  digitalWrite(CoilA, LOW);        // Start Sequenz
  digitalWrite(CoilB, LOW);        // Start Sequenz
  while(1) {}
}
