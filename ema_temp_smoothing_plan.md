# Plan for Implementing Exponential Moving Average (EMA) for Temperature Smoothing

The current temperature smoothing in `src/main.cpp` uses a Simple Moving Average (SMA). To address small fluctuations (e.g., 96.1, 96.2, 96.1, 96.3) that affect settling logic and readability, we will replace SMA with EMA.

## Plan Details:

1.  **Remove Existing SMA Logic:**
    *   The variables `tempReadings` (line 124), `currentReadingIndex` (line 125), `totalReadingsSum` (line 126), and `numReadingsStored` (line 127) in `src/main.cpp` will be removed as they are specific to SMA for temperature.
    *   The SMA calculation block (lines 717-730 in `src/main.cpp`) will be replaced.

2.  **Introduce EMA Variables:**
    *   Add a constant for the EMA smoothing factor: `const float TEMP_EMA_ALPHA = 0.2;` (A smaller alpha means more smoothing. 0.2 is a good starting point).
    *   Add a variable to store the current EMA value: `double smoothedTempC_EMA = NAN;` (Initialize to Not-a-Number to handle the first reading).

3.  **Implement EMA Calculation:**
    *   Locate the temperature reading block in `src/main.cpp` (after `double calibratedTempC = getCalibratedTemperature(rawTempC);` around line 714).
    *   Replace the existing SMA logic with the following EMA calculation:
        ```cpp
        // (Existing variable 'smoothedTempC' will be used to store the EMA result)
        if (isnan(smoothedTempC_EMA)) { // If it's the first valid reading after startup or error
          smoothedTempC_EMA = calibratedTempC;
        } else {
          smoothedTempC_EMA = (TEMP_EMA_ALPHA * calibratedTempC) + ((1.0 - TEMP_EMA_ALPHA) * smoothedTempC_EMA);
        }
        // Assign the EMA result to the variable used by downstream logic and display
        smoothedTempC = smoothedTempC_EMA; 
        ```

4.  **Confirm `web_smoothedTempC` Update:**
    *   The existing line `web_smoothedTempC = smoothedTempC;` (around line 731 in `src/main.cpp`) will automatically use the new EMA-smoothed value stored in `smoothedTempC`.

## Mermaid Diagram of the new Temperature Smoothing Logic:

```mermaid
graph TD
    A[Read Raw Temperature] --> B{Is Reading Valid?};
    B -- Yes --> C[Get Calibrated Temperature (`calibratedTempC`)];
    C --> D{Is `smoothedTempC_EMA` NAN? (First valid reading)};
    D -- Yes --> E[`smoothedTempC_EMA = calibratedTempC`];
    D -- No --> F[`smoothedTempC_EMA = (ALPHA * calibratedTempC) + ((1-ALPHA) * smoothedTempC_EMA)`];
    E --> G[Assign to `smoothedTempC = smoothedTempC_EMA`];
    F --> G;
    G --> H[Use `smoothedTempC` for Display, Control Logic, Web];
    B -- No --> I[Handle Thermocouple Read Error];
```

This approach aims to provide a more stable temperature reading, which should be less affected by small jitters, thereby aiding the settling logic and improving display readability.