/*
 * ESP32 Code for Ultrasonic Sensor (USX)
 *
 * This code waits for a "POLL_DATA" command from the MCXC444 via UART.
 * Upon receiving the command, it reads the ultrasonic distance and sends
 * back the specific threshold message (T1, T2, or NO_OBJECT).
 */

 #define TRIG_PIN 4
 #define ECHO_PIN 10
 #define BAUD_RATE 9600

 const int NEW_TX_PIN = 1;
 const int NEW_RX_PIN = 2;

 // Thresholds in millimeters (mm)
 #define T1_THRESHOLD_MM 1000
 #define T2_THRESHOLD_MM 500

 // // Messages expected by the MCXC444
 #define POLL_COMMAND "POLL_DATA"
 // #define T1_MESSAGE "T1"
 // #define T2_MESSAGE "T2"
 // #define NO_OBJECT_MESSAGE "SAFE"

 long duration_us;
 int distance_mm; // We work in millimeters for precision

 void setup() {
   Serial.begin(BAUD_RATE);
   Serial1.begin(BAUD_RATE, SERIAL_8N1, NEW_RX_PIN, NEW_TX_PIN); // UART to MCXC444
   pinMode(TRIG_PIN, OUTPUT);
   pinMode(ECHO_PIN, INPUT);
   Serial.println("ESP32 ready. Waiting for POLL_DATA...");
 }

 /**
  * @brief Reads ultrasonic distance and returns in millimeters (mm).
  * @return Distance in mm, or -1 if timeout/no reading.
  */
 int readUltrasonic_mm() {
   digitalWrite(TRIG_PIN, LOW);
   delayMicroseconds(2);
   digitalWrite(TRIG_PIN, HIGH);
   delayMicroseconds(10);
   digitalWrite(TRIG_PIN, LOW);

   // Measure pulse duration (timeout set for max ~5.1m range)
   duration_us = pulseIn(ECHO_PIN, HIGH);
   Serial.printf("Duration: %d\n", duration_us);

   if (duration_us == 0) {
     // If pulseIn timed out, assume no object or beyond reliable range
     return T1_THRESHOLD_MM + 1;
   }

   // Speed of sound = 0.343 mm/us
   // distance = (duration * 0.343) / 2
   // We round to the nearest integer mm.
   return (duration_us * 343) / 2000;
 }

 // /**
 //  * @brief Checks distance against thresholds and sends the appropriate message.
 //  * @param distance The measured distance in mm.
 //  */
 // void checkAndSendThreshold(int distance) {
 //   const char *message_to_send = NO_OBJECT_MESSAGE;

 //   // T2 Check (Critical Alert)
 //   // If object is 100mm or closer (and detected)
 //   if (distance > 0 && distance <= T2_THRESHOLD_MM) {
 //     message_to_send = T2_MESSAGE;
 //   }
 //   // T1 Check (Warning Alert)
 //   // If object is between 101mm and 500mm
 //   else if (distance > T2_THRESHOLD_MM && distance <= T1_THRESHOLD_MM) {
 //     message_to_send = T1_MESSAGE;
 //   }
 //   // No Object/Safe Zone
 //   // If object is further than 500mm or undetected (returns > 500 on timeout)
 //   else {
 //     message_to_send = NO_OBJECT_MESSAGE;
 //   }
 //   Serial.printf("Sending message to MCXC: %s\n", message_to_send);
 //   Serial1.println(message_to_send);
 // }

 void loop() {
   while (Serial1.available() <= 0);

   String cmd = Serial1.readString();
   cmd.trim();
   Serial.printf("Command: %s\n", cmd);
   // Check if the command is the expected polling signal
   if (cmd.equals(POLL_COMMAND)) {
     Serial.println("Polled");

     // 1. Read the distance
     distance_mm = readUltrasonic_mm();
     Serial.println(distance_mm);

     // 2. Determine threshold and send the response back
     // checkAndSendThreshold(distance_mm);
     Serial1.println(String(distance_mm));
   }
 }