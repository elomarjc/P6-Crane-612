#define pin_magnet_led 50

void setup() {
    
    Serial3.begin(9600);  // Communication with head
    Serial.begin(115200); // Communication with PC
    Serial.println("INITIALIZED");

    pinMode(pin_magnet_led,OUTPUT);
}

uint32_t timer = 0;
float angleFloat;
fastReader reader = fastReader(&Serial3);

void loop(){
    reader.getFloatln(&angleFloat);
    float xtime = micros();
    if (xtime >= timer + 1000) {
        timer = xtime + 1000;
        angleFloat = angleFloat;// - lp.update(angleFloat);
        digitalWrite(pin_magnet_led, (abs(angleFloat) > 0.5) );
    }
}

class fastreader {
  public:
    HardwareSerial intSerial;
    fastReader(HardwareSerial *serial){
      intSerial = serial;
    }
    bool getFloatln(float *output){
    while (intSerial->available() > 0){
      buffer += (char)intSerial->read();
      if (buffer.indexOf("\n") > -1){
        *output = buffer.substring(0,buffer.indexOf("\n")-1).toFloat();
        buffer = "";
        return true;
      } else {return false;}
    }
  }
};