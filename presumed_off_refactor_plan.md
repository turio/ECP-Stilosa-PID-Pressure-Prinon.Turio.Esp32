# Refactoring Plan: Presumed Off Logic

**Overall Goal:**
Implement a clearer, more robust, and configurable logic for detecting when the machine is presumed to be powered off. This involves monitoring if the temperature stays below a defined threshold and continues to decrease or remain stable for a specified duration (5 minutes).

**Plan Details:**

1.  **Introduce New Configuration Parameters:**
    *   Define `const float PRESUMED_OFF_TEMP_THRESHOLD_C = 84.0;` (or your desired default).
    *   Define `const unsigned long PRESUMED_OFF_DURATION_MS = 5 * 60 * 1000;` (5 minutes).
    *   The existing `PRESUMED_OFF_CHECK_INTERVAL_MS` (currently 10 seconds, line 172 in `src/main.cpp`) can be reused or renamed for clarity.

2.  **Consolidate and Rename State Variables:**
    *   Replace `bool machinePresumedOff` (line 167 in `src/main.cpp`) with `bool machineIsPresumedOff`.
    *   Replace `bool monitoringForPresumedOff` (line 169 in `src/main.cpp`) with `bool isMonitoringForMachineOff`.
    *   Replace `unsigned long below84StartTime` (line 168 in `src/main.cpp`) with `unsigned long machineOffMonitorStartTime`.
    *   Replace `float lastTempForPresumedOffCheck` (line 170 in `src/main.cpp`) with `float lastTempDuringMachineOffMonitoring`.
    *   Replace `unsigned long lastPresumedOffCheckTime` (line 173 in `src/main.cpp`) with `unsigned long lastMachineOffCheckTimestamp`.
    *   The old `PRESUMED_OFF_MONITOR_DURATION_MS` (line 171 in `src/main.cpp`) will be superseded by the new `PRESUMED_OFF_DURATION_MS`.

3.  **Refactor Logic in `loop()` (within `IDLE` state):**
    *   **If `machineIsPresumedOff` is `true`:**
        *   Maintain the existing logic (lines 829-844 in `src/main.cpp`) that checks for a significant temperature rise (rate of change) to detect if the machine has been powered back on. If detected, set `machineIsPresumedOff = false` and `isMonitoringForMachineOff = false`.
        *   The relay behavior in this state (currently `digitalWrite(RELAY_PIN, LOW);` on line 825 in `src/main.cpp`) will be maintained unless specified otherwise.
    *   **If `machineIsPresumedOff` is `false`:**
        *   **Start/Continue Monitoring Condition:** Check if `smoothedTempC < PRESUMED_OFF_TEMP_THRESHOLD_C` AND `desiredTemperatureC >= PRESUMED_OFF_TEMP_THRESHOLD_C` (this ensures monitoring only happens if the machine is expected to be hot).
            *   If this condition is met:
                *   If `!isMonitoringForMachineOff` (start of monitoring):
                    *   Set `isMonitoringForMachineOff = true;`
                    *   Record `machineOffMonitorStartTime = currentMillis;`
                    *   Record `lastTempDuringMachineOffMonitoring = smoothedTempC;`
                    *   Record `lastMachineOffCheckTimestamp = currentMillis;`
                    *   Update OLED: e.g., "Monitoring Power..."
                *   Else (`isMonitoringForMachineOff` is true, continue monitoring):
                    *   If `currentMillis - lastMachineOffCheckTimestamp >= PRESUMED_OFF_CHECK_INTERVAL_MS`:
                        *   If `smoothedTempC <= lastTempDuringMachineOffMonitoring` (temp is decreasing or stable):
                            *   Update `lastTempDuringMachineOffMonitoring = smoothedTempC;`
                            *   If `currentMillis - machineOffMonitorStartTime >= PRESUMED_OFF_DURATION_MS`:
                                *   Set `machineIsPresumedOff = true;`
                                *   Set `isMonitoringForMachineOff = false;`
                                *   Initialize variables for power-on detection (e.g., `previousTempForRateCheck = smoothedTempC; lastRateCheckTime = currentMillis;`).
                                *   Update OLED: e.g., "Machine Off Mode"
                        *   Else (temp increased while monitoring below threshold, e.g., `smoothedTempC > lastTempDuringMachineOffMonitoring`):
                            *   Set `isMonitoringForMachineOff = false;` (machine activity detected)
                            *   Update OLED: e.g., "Monitoring Halted"
                        *   Update `lastMachineOffCheckTimestamp = currentMillis;`
            *   Else (Start/Continue Monitoring Condition is NOT met, e.g., temp rose above threshold):
                *   If `isMonitoringForMachineOff`:
                    *   Set `isMonitoringForMachineOff = false;`
                    *   Update OLED: e.g., "Monitoring Stopped"

4.  **Refactor `handleSetTemp()` Function (Lines 408-440 in `src/main.cpp`):**
    *   Update the logic that handles user changes to `desiredTemperatureC`:
        *   If `machineIsPresumedOff` is true and the user sets `desiredTemperatureC >= PRESUMED_OFF_TEMP_THRESHOLD_C`:
            *   Set `machineIsPresumedOff = false;`
            *   Set `isMonitoringForMachineOff = false;`
            *   Update OLED: e.g., "Exiting Off Mode"
        *   If `isMonitoringForMachineOff` is true and the user sets `desiredTemperatureC < PRESUMED_OFF_TEMP_THRESHOLD_C`:
            *   Set `isMonitoringForMachineOff = false;`
            *   Update OLED: e.g., "Monitoring Off (Low Set)"

5.  **Remove Old Logic and Variables:**
    *   Delete the now-obsolete global variables.
    *   Remove the old block of presumed-off logic within the `IDLE` state (approximately lines 850-890 in `src/main.cpp`, and the parts of lines 823-828 in `src/main.cpp` that refer to the old variables).

**Visual Representation (Simplified Flow):**
```mermaid
graph TD
    subgraph Presumed Off Detection Logic
        A[IDLE State Active] --> B{machineIsPresumedOff?};
        B -- Yes --> C[Standby: Monitor for Temp Rise to Exit];
        C -- Power On Detected --> D[Normal Operation];
        B -- No --> E{Temp < Threshold AND DesiredTemp >= Threshold?};
        E -- Yes --> F{Monitoring Active?};
        F -- No (Start Monitoring) --> G[Init Monitor Vars: StartTime, LastTemp];
        G --> A;
        F -- Yes (Continue) --> H{Check Interval?};
        H -- Yes --> I{Temp Still Decreasing/Stable (<= LastTemp)?};
        I -- Yes --> J{5 Mins Passed?};
        J -- Yes --> K[Set machineIsPresumedOff = true];
        K --> A;
        J -- No --> L[Update LastTemp, Continue Monitoring];
        L --> A;
        I -- No (Temp Increased) --> M[Stop Monitoring];
        M --> A;
        H -- No --> A;
        E -- No (Temp >= Threshold or Desired Low) --> M;
    end

    subgraph User Input via Web UI
        U1[User sets Desired Temp] --> U2{machineIsPresumedOff AND NewDesiredTemp >= Threshold?};
        U2 -- Yes --> D;
        U1 --> U3{Monitoring Active AND NewDesiredTemp < Threshold?};
        U3 -- Yes --> M;
    end
```

**Benefits of this Plan:**
*   **Clarity:** The logic will be more straightforward and easier to understand.
*   **Robustness:** The condition "kept decreasing or stable" is more precise.
*   **Configurability:** The temperature threshold and duration become parameters.
*   **Consolidation:** Reduces the number of distinct states/flags for this specific feature by having a more streamlined monitoring process.