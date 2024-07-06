#define NUM_SAMPLES 20  // Number of samples to read
#define TRIM_AMOUNT 5  // Number of elements to trim from both ends

int adcPin = 26; // Define the analog pin to read from
int samples[NUM_SAMPLES]; // Array to store samples
float current;

void setup() {
  analogReadResolution(12);

  Serial.begin(9600); // Start serial communication
}

void loop() {
  // Read NUM_SAMPLES samples from the ADC
  for (int i = 0; i < NUM_SAMPLES; i++) {
    samples[i] = analogRead(adcPin);
    delay(1); // Small delay between readings
  }

  // Calculate the trimmed mean
  float trimmedMean = calculateTrimmedMean(samples, NUM_SAMPLES, TRIM_AMOUNT);
  
  current = trimmedMean;
  current = current-14;
  current = current*1.1;

  Serial.println(current);
  // Delay for a short period before the next reading
}

float calculateTrimmedMean(int *data, int size, int trimAmount) {
  // Sort the array
  for (int i = 0; i < size - 1; i++) {
    for (int j = i + 1; j < size; j++) {
      if (data[i] > data[j]) {
        int temp = data[i];
        data[i] = data[j];
        data[j] = temp;
      }
    }
  }

  // Calculate the sum of the trimmed data
  int sum = 0;
  for (int i = trimAmount; i < size - trimAmount; i++) {
    sum += data[i];
  }

  // Calculate the mean of the trimmed data
  float trimmedMean = (float)sum / (size - 2 * trimAmount);

  return trimmedMean;
}