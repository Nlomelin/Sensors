int const wind_sensor = A0;
int wind_speed_raw = 0;
int wind_speed_calibrated = 0;
int baseline = 78; // Adjust this based on your lowest reading with no wind

void setup() {
  pinMode(wind_sensor, INPUT);
  Serial.begin(115200);
}

void loop() {
  wind_speed_raw = analogRead(wind_sensor);
  wind_speed_calibrated = wind_speed_raw - baseline;
  if (wind_speed_calibrated < 0) wind_speed_calibrated = 0; // Avoid negative values
  Serial.print("Raw: ");
  Serial.print(wind_speed_raw);
  Serial.print(" | Wind Speed: ");
  Serial.println(wind_speed_calibrated);
  delay(500);
}
