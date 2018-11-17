#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <LIDARLite.h>
#include <LIDARLite_v3HP.h>
#include <IRremote.h>


//NEW VERSION
//LIDAR Hookup
//Blue - SDA
//Green - SCL
//Red - 5V
//Black - Ground

LIDARLite lidarLite;
SoftwareSerial gpsSerial(6,7);
TinyGPSPlus gps;
int cal_cnt = 0;

String AEnvoyer = "";
IRsend irsend;
int RECV_PIN = 11;
IRrecv irrecv(RECV_PIN);
decode_results results;

int counter = 0; 
float distances[5];

void setup()
{
  Serial.begin(9600); // Initialize serial connection to display distance readings
  gpsSerial.begin(9600);
  lidarLite.begin(0, true); // Set configuration to default and I2C to 400 kHz
  lidarLite.configure(0); // Change this number to try out alternate configurations
  irrecv.enableIRIn(); // Start the receiver2w

  array_init(); 
}

//initialize the distances array with -1 for every value
void array_init(){
  for (int i = 0; i < 5; i++){
    distances[i] = -1; 
  }
}

//debug helper function for seeing the values of distances 
void print_distances(){
  Serial.print("[");
  Serial.print(distances[0]); 
  
  for (int i = 1; i < 5; i++){
    Serial.print(", "); 
    Serial.print(distances[i]); 
  }

  Serial.print("]");
}

//Use the last 5 distance measurements to get the change 
//in speed (Kmph) of the car in front of you 
float gather_distances(){
  int index = counter % 5; 
  int i = 0; 
  float elapsed = 0; 

  float first = distances[index]; 
  float second = 0; 

  //no value to compare to 
  if(counter == 0){
    return 0; 
  }

  //a full cycle hasn't elapsed 
  if(counter < 5){
    second = distances[0]; 
    elapsed = (counter % 5); 
  }
  //the entire array has values
  else{
     if (++index > 4){
      index = 0; 
     }
     elapsed = 5; 
     second = distances[index]; 
  }

  //result is the difference in distance 
  float result = first - second;

  //0.01305s is the time it takes between polling for values 
  float timePassed = 0.01305 * elapsed;
  return mps_to_kph(result / timePassed); 
} 

float kph_to_mps(float speed)
{
  return speed * 3.6;
}

float mps_to_kph(float speed)
{
  return speed * 0.277778;
}

//Speed in Kilometers, friction coefficient is normally 0.8
//Left is reaction time, right is stopping distance
//extraDist is the distance from the IR sensor if the car in front is following too close
//reactionTime is the expected time it takes a user to react
float calculation(float speeds, float friction, float safetyDist, float extraDist, float reactionTime, float theirCalc){
  float stopping = 0.0, reactionDist = 0.0, stoppingPrime;
  stopping = stopping_dist(speeds, friction);
  reactionDist = reaction_dist(speeds, reactionTime);
  stoppingPrime = theirCalc; //Replace with other persons speed calculation////////////////////////
  return (reactionDist + stopping + safetyDist + extraDist) - stoppingPrime ; 
}

//Equation used to calculate distance required to stop if breaking starts immediately 
float stopping_dist(float speeds, float friction){
  return ((speeds * speeds) / (250 * friction));
}

//Equation used to calculate distance covered over the time it takes user to react
float reaction_dist(float speeds, float reactionTime){
  return (speeds * reactionTime) / 3.6;
}

//Determine the speed of the car in front of you
float det_speed(float curSpeed, float curDist)
{
  return 1.0;
}

//Determine State current state. 
//1 is good, 2 is back off, 3 is crash mode
int determine_state(float lidarDist, float calcDist){
  if(lidarDist >= calcDist){
    return 1;
  }
  else{
    if(calcDist - lidarDist < 5){
      return 2;
    }
    else{
      return 3;
    }
  }
}


//IR Sensor State Receiver
int checker()
{
  int returnVal = 0;
 
  if(results.value == 0xE0E08877)
  {
    Serial.println("Received State 0");
    returnVal = 0; 
  }
  else if(results.value == 0xE0E020DF)
  {
    Serial.println("Received State 1");
    returnVal = 1; 
  }
  else if(results.value == 0xE0E0A05F)
  {
    Serial.println("Received State 2");
    returnVal = 2; 
  }
  else if(results.value == 0xE0E0609F)
  {
    Serial.println("Received State 1");
    returnVal = 3; 
  }
  else if(results.value == 0xE0E010EF)
  {
    Serial.println("Received State 4");
    returnVal = 4; 
  }
  else if(results.value == 0xE0E0906F)
  {
    Serial.println("Received State 5");
    returnVal = 5; 
  }
  else if(results.value == 0xE0E050AF)
  {
    Serial.println("Received State 6");
    returnVal = 6; 
  }  
  results.value = 0;
  return returnVal; 
}

//Figure out how much distance to translate
float transmit_distance(float lidarDist, float yourDist){
  if(lidarDist >= yourDist){
    return 0.0;
  }
  else{
    return yourDist - lidarDist;
  }
}

//IR Codes
//0. 0xE0E08877 UNUSED
//1. 0xE0E020DF used for state 1 
//2. 0xE0E0A05F used for state 2
//3. 0xE0E0609F used for state 3
//4. 0xE0E010EF UNUSED
//5. 0xE0E0906F UNUSED
//6. 0xE0E050AF UNUSED
//7. 0xE0E030CF UNUSED
//8. 0xE0E0B04F UNUSED
//9. 0xE0E0708F UNUSED

//Given a number between 0-9, transmits it in IR codes 
float transmit(int dist){
  Serial.print("TX: ");
  if (dist == 0){
    Serial.println("State 0");
    irsend.sendNEC(0xE0E08877, 32); 
  }
  else if (dist == 1){
    Serial.println("State 1");
    irsend.sendNEC(0xE0E020DF, 32); 
  }
  else if (dist == 2){
    Serial.println("State 2");
    irsend.sendNEC(0xE0E0A05F, 32); 
  }
  else if (dist == 3){
    Serial.println("State 3");
    irsend.sendNEC(0xE0E0609F, 32); 
  }
  else if (dist == 4){
    Serial.println("State 4");
    irsend.sendNEC(0xE0E010EF, 32); 
  }
  else if (dist >= 5 && dist < 10){
    Serial.println("State 5");
    irsend.sendNEC(0xE0E0906F, 32); 
  }
  else{
    Serial.println("State 6");
    irsend.sendNEC(0xE0E050AF, 32); 
  } 
}

void loop()
{
  int dist;
  float speeder = -1.0, extraDist = 0.0;

    //Code for receiving IR data
  if (irrecv.decode(&results)) {
    //Serial.print(char(results.value));
    irrecv.resume(); // Receive the next value    
  }
  int checked = checker();//Where the checker is called, in order to determine the data from the IR

  // At the beginning of every 100 readings,
  // take a measurement with receiver bias correction
  if ( cal_cnt == 0 ) {
    dist = lidarLite.distance();      // With bias correction
  } else {
    dist = lidarLite.distance(false); // Without bias correction
  }

  //get the current speed
  while(gpsSerial.available())
  {
    gps.encode(gpsSerial.read());
  }
  if (gps.location.isUpdated())
  {
    speeder = gps.speed.kmph();
  }

  // Increment reading counter
  cal_cnt++;
  cal_cnt = cal_cnt % 100;

  //used for coding purposes to set a speed 
  //sets the speed to 0 just in case the GPS is failing so it doesn't read negative values 
  if(speeder < 0){
    speeder = 0.0; //Base value for speed
  }
  
  int state;
  boolean carInFront = true; 
  float findDist, outputRes, theirSpeed = -1.0, theirOutputRes = -1.0, friction = 0.80;
  bool stopping = false;

  // Distance you have, in meters
  findDist = dist / 100.0; 

  //CASE: nothing closer than 40m 
  if (findDist < 0.05){
    findDist = 40.0; 
    carInFront = false; 
  }

  //keep track of the last 5 values of distance 
  distances[counter % 5] = findDist; 
  float speedDelta = gather_distances();
  counter++; 

  //Set the extra distance values accorrding to the state receieved 
  if(checked == 2){
    extraDist = 5.0; 
  }
  else if(checked == 3){
    extraDist = 10.0; 
  }
  
  //"if" is for if there is a car in front of you include them in the calculations
  //"then" is to make sure it doesn't run the calculations for a car in front if there is no car in front 
  if(carInFront){
    //calculate the speed of the car in front in kph
    theirSpeed = speeder + speedDelta;   

    // Distance the car in front would need to stop 
    theirOutputRes = calculation(theirSpeed, friction, 0.0, extraDist, 2.0, 0.0);

    // Distance you would need to stop 
    outputRes = calculation(speeder, friction, speeder * 0.10, extraDist, 2.0, theirOutputRes); 
  }
  else{
    outputRes = calculation(speeder, friction, speeder * 0.10, extraDist, 2.0, 0.0); // Distance needed to stop
  }

  //Determine state based off of measurements and calculations 
  //relay that information to user 
  //transmit that info to car behind 
  state = determine_state(findDist, outputRes);
  if(state == 3){
    stopping = false;
    Serial.println("Not enough room to stop. BACK OFF NOW.");
    transmit(3); 
  }
  else if(state == 2){
    stopping = true;
    Serial.println("Getting close, back off");
    transmit(2); 
  }
  else{
      stopping = true;
      Serial.println("Enough room to stop");
      transmit(1); 
  }

  //Printers to check status values 
  Serial.print("Received State: ");
  Serial.println(checked);

  Serial.print("Speeder: "); 
  Serial.println(speeder); 

  Serial.print("TheirOutputRes: "); 
  Serial.println(theirOutputRes); 

  Serial.print("YourOutputRes: "); 
  Serial.println(outputRes); 

  Serial.print("Actual Dist: "); 
  Serial.println(findDist); 

  Serial.println();
  delay(10);
}
