#include <Arduino.h>
#include <MAX6675.h>
#include <stdlib.h> // For qsort
#include <algorithm> // For std::sort (or use qsort)
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WebServer.h> // For Web Server

// --- LED_BUILTIN Definition ---
#ifndef LED_BUILTIN
#define LED_BUILTIN 2 // Define LED_BUILTIN if not already defined (e.g., for some ESP32 boards)
#endif

// --- Feature Toggle ---
#define ENABLE_DATETIME_WEATHER_FEATURE // Comment this line out to disable Date/Time/Weather feature

#ifdef ENABLE_DATETIME_WEATHER_FEATURE
#include <NTPClient.h> // Added for NTP
#include <HTTPClient.h> // Added for Weather API
#include <ArduinoJson.h> // Added for parsing Weather JSON
#endif

// WiFi credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// --- WiFi Connection Management ---
enum WiFiState { WIFI_DISCONNECTED, WIFI_CONNECTING, WIFI_CONNECTED };
WiFiState currentWiFiState = WIFI_DISCONNECTED;
unsigned long wifiConnectStartTime = 0;
const unsigned long WIFI_CONNECT_TIMEOUT_MS = 15000; // 15 seconds to connect
const unsigned long WIFI_RETRY_DELAY_MS = 5000;    // 5 seconds before retrying connection
unsigned long wifiLastRetryTime = 0;
int wifiRetryCount = 0;
const int MAX_WIFI_RETRIES_BEFORE_REBOOT = 5;
// Define the pins for the MAX6675
// Connect MAX6675 SO (Serial Out) to ESP32 GPIO 19 (MISO)
const int thermoSO = 19;
// Connect MAX6675 CS (Chip Select) to ESP32 GPIO 5
const int thermoCS = 5;
// Connect MAX6675 SCK (Serial Clock) to ESP32 GPIO 18 (SCK)
const int thermoSCK = 18;

// Initialize the MAX6675 library with the defined pins
MAX6675 thermocouple(thermoSCK, thermoCS, thermoSO);

// --- Pressure Sensor Setup ---
// GPIO26 is an ADC2 pin. ADC2 pins cannot be used when Wi-Fi is active.
// Using GPIO34 (ADC1_CH6) as it's an ADC1 pin, suitable for use with Wi-Fi,
// and commonly available on ESP32 devkits.
const int pressureSensorPin = 35; // Using GPIO35 (ADC1_CH7), an ADC1 pin suitable for use with Wi-Fi.
                                // GPIO4 is an ADC2 pin and may not work reliably when Wi-Fi is active.

// --- Pressure Sensor ADC Processing Setup ---
const int PRESSURE_RAW_SAMPLES_COUNT = 7; // Number of raw ADC samples to take ( reverted to older commit value)
const int PRESSURE_SAMPLES_TO_DISCARD_EACH_END = 1; // Discard 1 lowest and 1 highest (reverted to older commit value)
int pressureAdcSampleReadings[PRESSURE_RAW_SAMPLES_COUNT];

const int PRESSURE_SMOOTHING_SAMPLES = 5; // Number of stable ADC values to average for final smoothing (reverted to older commit value)
double pressureAdcSmoothingBuffer[PRESSURE_SMOOTHING_SAMPLES];
int currentPressureAdcSmoothingIndex = 0;
double totalPressureAdcSmoothingSum = 0;
int numPressureAdcValuesStored = 0;

// Pressure Calibration Constants
const float VOLTS_AT_0_BAR = 0.34f; // New calibration: 0.34V at 0 bar
const float VOLTS_AT_16_BAR = 4.34f; // New calibration: 4.34V at 16 bar
const float PRESSURE_MAX_BAR = 16.0f; // Max pressure (reverted to older commit value)
const float ESP32_ADC_MAX_VOLTAGE = 3.3f; // ESP32 ADC reference voltage
const float ESP32_ADC_MAX_VALUE = 4095.0f; // ESP32 12-bit ADC max value


// --- Temperature Calibration Setup ---
// Add more points for better accuracy. Ensure raw_temps_c is sorted if using more than 2 points
// and updating the getCalibratedTemperature function accordingly.
const int CALIBRATION_POINTS_COUNT = 2;
// Raw temperatures from the thermocouple (e.g., {reading1, reading2})
double raw_temps_c[CALIBRATION_POINTS_COUNT] = {99.0, 115.0}; // {99.0C raw, 115.0C raw}
// Corresponding actual temperatures (e.g., {actual_for_reading1, actual_for_reading2})
double actual_temps_c[CALIBRATION_POINTS_COUNT] = {85.0, 97.8}; // {85.0C actual, 97.8C actual}

/**
 * Calculates calibrated temperature using linear interpolation/extrapolation
 * based on the defined calibration points.
 *
 * @param rawTempC The raw temperature reading from the thermocouple.
 * @return The calibrated temperature.
 */
double getCalibratedTemperature(double rawTempC) {
  if (CALIBRATION_POINTS_COUNT < 2) {
    // Not enough points for linear interpolation, return raw temperature
    Serial.println(F("Calibration requires at least 2 points. Returning raw temperature."));
    return rawTempC;
  }

  // Using the two defined points for linear interpolation/extrapolation
  // (x1, y1) = (raw_temps_c[0], actual_temps_c[0])
  // (x2, y2) = (raw_temps_c[1], actual_temps_c[1])
  // Formula: y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
  // where x is rawTempC and y is the calibrated temperature.

  double x1 = raw_temps_c[0];
  double y1 = actual_temps_c[0];
  double x2 = raw_temps_c[1];
  double y2 = actual_temps_c[1];

  // Check to prevent division by zero if calibration points are identical for raw readings
  if (x2 - x1 == 0) {
    Serial.println(F("Calibration error: Raw temperature points are identical. Returning raw temperature."));
    return rawTempC; // Or return y1, as actual temps might also be the same or it's an error state
  }

  double calibratedTemp = y1 + (rawTempC - x1) * (y2 - y1) / (x2 - x1);
  return calibratedTemp;
}

// --- Temperature Smoothing Setup ---
// EMA (Exponential Moving Average) for temperature
const float TEMP_EMA_ALPHA = 0.07f; // Smoothing factor for EMA (smaller = more smoothing, e.g., 0.1-0.3)
                                   // Alpha = 2 / (N+1) where N is SMA equivalent. N=19 -> Alpha approx 0.1
double smoothedTempC_EMA = NAN;    // Stores the current EMA smoothed temperature, persists across readings

// Variables for non-blocking temperature reading
unsigned long lastTempReadTime = 0;
const long tempReadInterval = 500; // Read temperature every 100 milliseconds (1 second)

// --- Relay Control Setup ---
const int RELAY_PIN = 14; // Corrected RELAY_PIN back to 14
bool isRelayOn = false;
unsigned long heaterStopTimeMs = 0; // Timestamp for when the heater should turn off (during HEATING state)

// --- Temperature Control Parameters & States ---
enum HeaterState { IDLE, HEATING, SETTLING };
HeaterState currentHeaterState = IDLE;

volatile double desiredTemperatureC = 90.0; // Target temperature, no longer const
const float HEATER_SECONDS_PER_DEGREE_C = 2.0f ; // Approx 2 seconds of heating raises temp by 1 degree C
const unsigned long MAX_HEATER_ON_DURATION_MS = 70 * 1000; // Max 30 seconds heater on time for safety

// Dynamic Settling Parameters
const double SETTLED_TEMP_RISE_MAX_C = 0.3; // Max allowed temperature rise during observation to be considered settled
const unsigned long SETTLED_OBSERVATION_PERIOD_MS = 10 * 1000; // 10 seconds observation window
const double EARLY_CUTOFF_TEMP_C = 76.0; // Temperature threshold for early cutoff in HEATING state
unsigned long settlingCheckStartTimeMs = 0; // Timestamp when the current settling observation period started
double tempAtSettlingCheckStartC = 0.0; // Temperature at the start of the current settling observation

// The lower threshold for heating is now dynamically calculated as desiredTemperatureC - 1.0.

// Variable to store the last calculated heating duration when entering HEATING state
unsigned long lastCalculatedHeatDurationMs = 0;

// --- Cooldown after Early Cutoff ---
bool enteredIdleAfterEarlyCutoff = false; // Flag set in HEATING on early cutoff
bool inEarlyCutoffCooldown = false;       // True if IDLE is currently in the mandatory off period
unsigned long earlyCutoffCooldownEndTime = 0; // Timestamp for when the cooldown ends
const unsigned long EARLY_CUTOFF_COOLDOWN_DURATION_MS = 60 * 1000; // 1 minute mandatory off time

// --- LED Control Setup ---
// --- Machine Presumed Off Detection & Standby Logic ---
// New Configuration Parameters
const float PRESUMED_OFF_TEMP_THRESHOLD_C = 86.0f; // Temperature below which monitoring for presumed off starts
volatile bool web_early_cutoff_signal = false; // Signal for client plot reset
const unsigned long PRESUMED_OFF_DURATION_MS = 3 * 60 * 1000; // 3 minutes: duration for temp to be below threshold AND stable/decreasing to be presumed off
// PRESUMED_OFF_CHECK_INTERVAL_MS is retained from original, just moved its declaration up for clarity
const unsigned long PRESUMED_OFF_CHECK_INTERVAL_MS = 10000; // Check every 10 seconds during monitoring

// Consolidated and Renamed State Variables
bool machineIsPresumedOff = false;       // True if the system believes the main machine power is off
unsigned long machineOffMonitorStartTime = 0; // Timestamp when temp first dropped < PRESUMED_OFF_TEMP_THRESHOLD_C while monitoring
bool isMonitoringForMachineOff = false;  // True if actively checking the PRESUMED_OFF_DURATION_MS condition
float lastTempDuringMachineOffMonitoring = 100.0f; // Stores temp from previous check during monitoring, init high
unsigned long lastMachineOffCheckTimestamp = 0; // Timestamp for the PRESUMED_OFF_CHECK_INTERVAL_MS check

int consecutiveFailedHeatingAttempts = 0;
const int MAX_CONSECUTIVE_HEATING_FAILURES = 5;
const float TEMP_DIFF_THRESHOLD_FOR_HEATING_FAILURE = 5.0; // Degrees C below desired to count as failure
// Variables for detecting machine power-on (temperature rise rate)
float previousTempForRateCheck = 0.0;
unsigned long lastRateCheckTime = 0;
const unsigned long RATE_CHECK_INTERVAL_MS = 5000; // Check rate every 5 seconds
// const int BUILTIN_LED_PIN = 2; // Using LED_BUILTIN, but if not defined for your board, uncomment and set this.
unsigned long lastBlinkTimeLed = 0;
const long blinkIntervalRapid = 150; // milliseconds for rapid blink
const long blinkIntervalSlow = 1000;        // milliseconds for slow blink (settling, not at temp)
const long blinkIntervalVeryRapid = 50;     // milliseconds for very rapid blink (overheat)

// --- Status LED on GPIO27 ---
const int STATUS_LED_PIN = 27;
unsigned long lastStatusLedToggleTime = 0;
const long statusLedToggleInterval = 5000; // 5 seconds

#ifdef ENABLE_DATETIME_WEATHER_FEATURE
// --- NTP Client Setup ---
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -14400; // Replace with your GMT offset in seconds (e.g., for GMT+1, use 3600)
const int daylightOffset_sec = 3600; // Replace with your daylight offset in seconds if applicable

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, ntpServer, gmtOffset_sec, daylightOffset_sec);

// --- Weather Setup ---
String openWeatherMapApiKey = "YOUR_OPEN_WEATHER_API_KEY";
String city = "Montreal,Canada";
String units = "metric"; // or "imperial"
String weatherApiUrlBase = "http://api.openweathermap.org/data/2.5/weather?q=";

unsigned long lastWeatherReadTime = 0;
const long weatherReadInterval = 15 * 60 * 1000; // 15 minutes in milliseconds
float currentWeatherDataTemp = NAN; // Store current weather temperature
String currentTimeStr = "--:--";
String currentDateStr = "--/--";
String currentDayOfWeekStr = "---";
#endif

// --- Web Server Setup ---
WebServer server(80);
volatile double web_smoothedTempC = NAN;
volatile float web_pressureBar = NAN; // Reverted to web_pressureBar
volatile bool web_isRelayOn = false;
volatile double web_desiredTempC = 90.0; // To send current desired temp to UI
String oledStatusMessage = "System Booting..."; // For displaying status on OLED

// --- Shot Timer & History ---
struct DataPoint {
  unsigned long time_ms;
  float value;
};
const int HISTORY_SIZE = 90; // 90 seconds of data, 1 sample/sec
DataPoint tempHistory[HISTORY_SIZE];
DataPoint pressureHistory[HISTORY_SIZE];
int tempHistoryIndex = 0;
int pressureHistoryIndex = 0;
int tempHistoryCount = 0;
int pressureHistoryCount = 0;
unsigned long lastHistorySampleTime = 0;
const long historySampleInterval = 1000; // 1 second

// --- Shot Timer Variables ---
volatile bool isShotRunning = false;
volatile unsigned long shotStartTime_ms = 0;
volatile unsigned long web_shotDuration_ms = 0; // Duration to be sent to the web UI

// --- Server-side Plot Pause State ---
volatile bool web_isTempPlotPaused = false;
volatile bool web_isPressurePlotPaused = false;
float lastPressureForPauseCheck_server = 0.0f;

// --- Max Pressure Tracking ---
volatile float maxObservedPressure = 0.0f; // Stores the maximum stable pressure observed
const int PRESSURE_STABILITY_SAMPLES_FOR_MAX = 5; // Number of samples to check for stability for max pressure
float pressureMaxStabilityBuffer[PRESSURE_STABILITY_SAMPLES_FOR_MAX];
int pressureMaxStabilityIndex = 0;
int pressureMaxStabilityCount = 0;
const float PRESSURE_STABILITY_THRESHOLD_FOR_MAX = 0.2f; // Max deviation in bar for considering pressure stable for max update
const float PRESSURE_RESUME_THRESHOLD_BAR = 2.0f; // Hysteresis: pressure must rise to this value to resume plotting
// --- End Max Pressure Tracking ---

const char HTML_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en" class="dark">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>De'Longhi Monitor</title>
    <script src="https://cdn.tailwindcss.com"></script>
    <script src="https://cdn.jsdelivr.net/npm/apexcharts"></script>
    <style>
        .apexcharts-tooltip {
            background: #2d3748;
            border: 1px solid #4a5568;
            color: #e2e8f0;
        }
        .paused-overlay {
            position: absolute;
            top: 0;
            left: 0;
            right: 0;
            bottom: 0;
            background-color: rgba(0, 0, 0, 0.5);
            color: white;
            display: flex;
            justify-content: center;
            align-items: center;
            font-size: 2rem;
            font-weight: bold;
            z-index: 10;
            border-radius: 0.5rem;
        }
    </style>
</head>
<body class="bg-gray-900 text-gray-200 font-sans">
    <div class="container mx-auto p-4 max-w-5xl">
        <header class="text-center mb-8">
            <h1 class="text-4xl font-bold text-cyan-400">De'Longhi Pro</h1>
            <p class="text-gray-400">Live Temperature & Pressure Monitoring</p>
        </header>

        <main class="grid grid-cols-1 md:grid-cols-3 gap-6">
            <!-- Left Column: Controls -->
            <div class="md:col-span-1 bg-gray-800 p-6 rounded-lg shadow-lg flex flex-col">
                <h2 class="text-2xl font-semibold mb-4 border-b border-gray-700 pb-2">Controls</h2>
                


                <div class="mb-6">
                    <label for="tempSlider" class="block mb-2 text-lg text-gray-400">Set Desired Temp: <span id="desiredTempDisplay" class="font-bold text-cyan-400">--</span>&deg;C</label>
                    <input type="range" id="tempSlider" min="70" max="100" value="90" step="0.5" class="w-full h-3 bg-gray-700 rounded-lg appearance-none cursor-pointer accent-cyan-500">
                </div>
                
                <div class="flex justify-between items-center mb-6 bg-gray-700 p-3 rounded-lg">
                    <p class="text-md text-gray-300">Max Pressure:</p>
                    <p class="text-xl font-bold"><span id="maxPress">--</span> bar</p>
                </div>

                <div class="flex justify-between items-center mb-6 bg-gray-700 p-3 rounded-lg">
                    <p class="text-md text-gray-300">Heater Status:</p>
                    <p class="text-xl font-bold"><span id="relay">--</span></p>
                </div>

                
                <div class="grid grid-cols-1 gap-4 mb-6">
                    <div>
                        <p class="text-lg text-gray-400">Boiler Temp</p>
                        <p class="text-5xl font-mono font-bold"><span id="temp">--</span><span class="text-3xl">&deg;C</span></p>
                    </div>
                    <div>
                        <p class="text-lg text-gray-400">Pressure</p>
                        <p class="text-5xl font-mono font-bold"><span id="press">--</span><span class="text-3xl">bar</span></p>
                    </div>
                    <div>
                        <p class="text-lg text-gray-400">Shot Time</p>
                        <p class="text-5xl font-mono font-bold"><span id="shotTime">--.-</span><span class="text-3xl">s</span></p>
                    </div>
                </div>


                <div class="mt-auto">
                    <button id="resetMaxPressureBtn" class="w-full bg-red-600 hover:bg-red-700 text-white font-bold py-3 px-4 rounded-lg transition duration-300">Reset Max & Plots</button>
                </div>
            </div>

            <!-- Right Column: Charts -->
            <div class="md:col-span-2 bg-gray-800 p-6 rounded-lg shadow-lg">
                <div class="relative mb-6">
                    <h3 class="text-xl font-semibold mb-2">Temperature (&deg;C)</h3>
                    <div id="tempChart"></div>
                    <div id="tempChartPaused" class="paused-overlay hidden">PAUSED</div>
                </div>
                <div class="relative">
                    <h3 class="text-xl font-semibold mb-2">Pressure (bar)</h3>
                    <div id="pressureChart"></div>
                    <div id="pressureChartPaused" class="paused-overlay hidden">PAUSED</div>
                </div>
            </div>
        </main>
    </div>

    <script>
        let desiredTempSlider = document.getElementById('tempSlider');
        let desiredTempDisplay = document.getElementById('desiredTempDisplay');
        let sliderBeingDragged = false;
        let debounceTimer;

        let tempChart, pressureChart;
        let tempData = [];
        let pressureData = [];
        let isTempPlotPaused = false;
        let isPressurePlotPaused = false;
        const PLOT_MAX_DURATION_MS = 60000; // 60 seconds

        function createChart(elementId, title, color) {
            const options = {
                series: [{ name: title, data: [] }],
                chart: {
                    height: 250,
                    type: 'line',
                    animations: {
                        enabled: true,
                        easing: 'linear',
                        dynamicAnimation: { speed: 1000 }
                    },
                    toolbar: {
                        show: true,
                        tools: {
                            download: true,
                            selection: false,
                            zoom: false,
                            zoomin: false,
                            zoomout: false,
                            pan: false,
                            reset: false
                        }
                    },
                    zoom: { enabled: false },
                    background: 'transparent'
                },
                stroke: { curve: 'smooth', width: 2, colors: [color] },
                grid: {
                    borderColor: '#4a5568',
                    row: { colors: ['transparent', 'transparent'], opacity: 0.5 },
                },
                xaxis: {
                    type: 'datetime',
                    range: PLOT_MAX_DURATION_MS,
                    labels: {
                        style: { colors: '#9ca3af' },
                        format: 'mm:ss'
                    }
                },
                yaxis: {
                    labels: {
                        formatter: (val) => val.toFixed(1),
                        style: { colors: '#9ca3af' }
                    }
                },
                tooltip: { theme: 'dark' },
                markers: { size: 0 }
            };
            const chart = new ApexCharts(document.querySelector("#" + elementId), options);
            chart.render();
            return chart;
        }

        desiredTempSlider.addEventListener('input', function() {
            desiredTempDisplay.innerText = parseFloat(this.value).toFixed(1);
            sliderBeingDragged = true;
            clearTimeout(debounceTimer);
            debounceTimer = setTimeout(() => {
                sendDesiredTemp(this.value);
                sliderBeingDragged = false;
            }, 500);
        });

        desiredTempSlider.addEventListener('change', function() {
            clearTimeout(debounceTimer);
            sendDesiredTemp(this.value);
            sliderBeingDragged = false;
        });

        function sendDesiredTemp(temp) {
            console.log("Sending desired temp:", temp);
            fetch('/settemp', {
                method: 'POST',
                headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                body: 'temp=' + temp
            }).then(response => {
                if (!response.ok) console.error('Error setting temperature:', response.statusText);
                else console.log("Desired temp successfully set to", temp);
                updateSensorData();
            }).catch(error => {
                console.error('Error sending desired temperature:', error);
                updateSensorData();
            });
        }

        function updatePlots(currentTemp, currentPressure) {
            const currentTime = Date.now();

            if (!isTempPlotPaused) {
                tempData.push({ x: currentTime, y: currentTemp });
                tempData = tempData.filter(p => currentTime - p.x <= PLOT_MAX_DURATION_MS + 2000);
                if (tempChart) tempChart.updateSeries([{ data: tempData }]);
            }

            if (!isPressurePlotPaused) {
                pressureData.push({ x: currentTime, y: currentPressure });
                pressureData = pressureData.filter(p => currentTime - p.x <= PLOT_MAX_DURATION_MS + 2000);
                if (pressureChart) pressureChart.updateSeries([{ data: pressureData }]);
            }
        }

        function resetPlots() {
            console.log("Plots reset.");
            tempData = [];
            pressureData = [];
            isTempPlotPaused = false;
            isPressurePlotPaused = false;
            document.getElementById('tempChartPaused').classList.add('hidden');
            document.getElementById('pressureChartPaused').classList.add('hidden');
            if (tempChart) tempChart.updateSeries([{ data: [] }]);
            if (pressureChart) pressureChart.updateSeries([{ data: [] }]);
            fetchHistory();
        }

        function updateSensorData() {
            fetch('/data')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('temp').innerText = data.temperature.toFixed(1);
                    document.getElementById('press').innerText = data.pressure.toFixed(1);
                    document.getElementById('maxPress').innerText = data.max_observed_pressure.toFixed(1);

                    let shotTimeDisplay = document.getElementById('shotTime');
                    const newShotTimeText = data.shot_duration > 0 ? (data.shot_duration / 1000.0).toFixed(1) : '--.-';
                    if (shotTimeDisplay.innerText !== newShotTimeText) {
                        shotTimeDisplay.innerText = newShotTimeText;
                    }

                    let relaySpan = document.getElementById('relay');
                    relaySpan.innerText = data.relay_status;
                    relaySpan.className = (data.relay_status === 'ON') ? 'text-green-400' : 'text-red-500';

                    if (!sliderBeingDragged) {
                        desiredTempDisplay.innerText = data.desired_temp.toFixed(1);
                        desiredTempSlider.value = data.desired_temp.toFixed(1);
                    }

                    const wasTempPaused = isTempPlotPaused;
                    isTempPlotPaused = data.is_temp_plot_paused;
                    document.getElementById('tempChartPaused').classList.toggle('hidden', !isTempPlotPaused);
                    if (wasTempPaused && !isTempPlotPaused) {
                        console.log("Temp plot resumed on server. Resetting client plot.");
                        resetPlots();
                    }

                    const wasPressurePaused = isPressurePlotPaused;
                    isPressurePlotPaused = data.is_pressure_plot_paused;
                    document.getElementById('pressureChartPaused').classList.toggle('hidden', !isPressurePlotPaused);
                    if (wasPressurePaused && !isPressurePlotPaused) {
                        console.log("Pressure plot resumed on server. Resetting client plot.");
                        resetPlots();
                    }

                    if (data.early_cutoff_event) {
                        console.log("Early cutoff event received from server. Resetting plots.");
                        resetPlots();
                    }
                    
                    updatePlots(data.temperature, data.pressure);
                })
                .catch(error => console.error('Error fetching data:', error));
        }

        document.getElementById('resetMaxPressureBtn').addEventListener('click', function() {
            fetch('/resetmaxpressure', { method: 'POST' })
                .then(response => {
                    if (!response.ok) console.error('Error resetting max pressure:', response.statusText);
                    else {
                        console.log("Max pressure reset signal sent. Resetting plots.");
                        resetPlots();
                        updateSensorData();
                    }
                })
                .catch(error => console.error('Error sending reset max pressure request:', error));
        });

        function fetchHistory() {
            fetch('/history')
                .then(response => response.json())
                .then(data => {
                    const now = Date.now();
                    tempData = data.temp_history.map(p => ({ x: now + p.time, y: p.value }));
                    pressureData = data.pressure_history.map(p => ({ x: now + p.time, y: p.value }));
                    console.log("Fetched and processed historical data.");
                    if (tempChart) tempChart.updateSeries([{ data: tempData }]);
                    if (pressureChart) pressureChart.updateSeries([{ data: pressureData }]);
                })
                .catch(error => console.error('Error fetching history:', error));
        }

        window.onload = function() {
            tempChart = createChart('tempChart', 'Temperature', '#f97316');
            pressureChart = createChart('pressureChart', 'Pressure', '#3b82f6');
            updateSensorData();
            fetchHistory();
        };

        setInterval(updateSensorData, 2000);
    </script>
</body>
</html>
)rawliteral";

// Function to update the status message on the OLED
void updateOledStatus(const String& newMessage) {
  oledStatusMessage = newMessage; // Allow full message, will be handled during display
}

void handleRoot() {
  server.send(200, "text/html", HTML_PAGE);
}

void handleData() {
  String json = "{";
  json += "\"temperature\":" + String(web_smoothedTempC, 1) + ",";
  json += "\"pressure\":" + String(web_pressureBar, 1) + ",";
  json += "\"max_observed_pressure\":" + String(maxObservedPressure, 1) + ",";
  json += "\"relay_status\":\"" + String(web_isRelayOn ? "ON" : "OFF") + "\",";
  json += "\"desired_temp\":" + String(web_desiredTempC, 1) + ",";
  json += "\"shot_duration\":" + String(web_shotDuration_ms) + ",";
  json += "\"presumed_off_threshold\":" + String(PRESUMED_OFF_TEMP_THRESHOLD_C, 1) + ",";
  json += "\"is_temp_plot_paused\":" + String(web_isTempPlotPaused ? "true" : "false") + ",";
  json += "\"is_pressure_plot_paused\":" + String(web_isPressurePlotPaused ? "true" : "false") + ",";
  json += "\"early_cutoff_event\":" + String(web_early_cutoff_signal ? "true" : "false");
  json += "}";
  if (web_early_cutoff_signal) {
    web_early_cutoff_signal = false; // Reset signal after sending
  }
  server.send(200, "application/json", json);
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

// --- End Web Server Setup ---

// --- Handler to Reset Max Pressure ---
void handleResetMaxPressure() {
  maxObservedPressure = 0.0f;
  pressureMaxStabilityCount = 0;
  pressureMaxStabilityIndex = 0;
  
  // Clear history data
  tempHistoryCount = 0;
  tempHistoryIndex = 0;
  pressureHistoryCount = 0;
  pressureHistoryIndex = 0;

  Serial.println("Max observed pressure and history reset via WebUI.");
  server.send(200, "text/plain", "Max pressure and history reset.");
  updateOledStatus("Max/Hist Reset"); // Update OLED status
}

void handleHistory() {
  String json = "{\"temp_history\":[";
  unsigned long now_ms = millis();

  // Add temperature history, from oldest to newest
  for (int i = 0; i < tempHistoryCount; i++) {
    int index = (tempHistoryIndex - tempHistoryCount + i + HISTORY_SIZE) % HISTORY_SIZE;
    // Send time as a negative offset from the current time
    long time_offset = (long)tempHistory[index].time_ms - (long)now_ms;
    json += "{\"time\":" + String(time_offset) + ",\"value\":" + String(tempHistory[index].value, 1) + "}";
    if (i < tempHistoryCount - 1) {
      json += ",";
    }
  }
  json += "],\"pressure_history\":[";

  // Add pressure history, from oldest to newest
  for (int i = 0; i < pressureHistoryCount; i++) {
    int index = (pressureHistoryIndex - pressureHistoryCount + i + HISTORY_SIZE) % HISTORY_SIZE;
    long time_offset = (long)pressureHistory[index].time_ms - (long)now_ms;
    json += "{\"time\":" + String(time_offset) + ",\"value\":" + String(pressureHistory[index].value, 1) + "}";
    if (i < pressureHistoryCount - 1) {
      json += ",";
    }
  }
  json += "]}";

  server.send(200, "application/json", json);
}


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// SDA -> GPIO21
// SCL -> GPIO22
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Comparison function for qsort
int compareIntegers(const void *a, const void *b) {
  return (*(int*)a - *(int*)b);
}

// Function to get a stable ADC reading by sampling, sorting, and averaging
int getStableAdcValue() {
  if (PRESSURE_RAW_SAMPLES_COUNT < (2 * PRESSURE_SAMPLES_TO_DISCARD_EACH_END + 1)) {
    // Not enough samples to discard and average, return a single reading
    return analogRead(pressureSensorPin);
  }

  for (int i = 0; i < PRESSURE_RAW_SAMPLES_COUNT; i++) {
    pressureAdcSampleReadings[i] = analogRead(pressureSensorPin);
  }

  qsort(pressureAdcSampleReadings, PRESSURE_RAW_SAMPLES_COUNT, sizeof(int), compareIntegers);
  // std::sort(pressureAdcSampleReadings, pressureAdcSampleReadings + PRESSURE_RAW_SAMPLES_COUNT); // Alternative using algorithm header

  long sum = 0;
  int samplesToAverage = 0;
  for (int i = PRESSURE_SAMPLES_TO_DISCARD_EACH_END; i < PRESSURE_RAW_SAMPLES_COUNT - PRESSURE_SAMPLES_TO_DISCARD_EACH_END; i++) {
    sum += pressureAdcSampleReadings[i];
    samplesToAverage++;
  }

  if (samplesToAverage == 0) return analogRead(pressureSensorPin); // Fallback
  return sum / samplesToAverage;
}
void handleSetTemp() {
  if (server.hasArg("temp")) {
    String tempStr = server.arg("temp");
    double newTemp = tempStr.toDouble();
    // Add validation for temperature range, e.g., 70 to 100 C
    if (newTemp >= 70.0 && newTemp <= 100.0) {
      desiredTemperatureC = newTemp;
      web_desiredTempC = newTemp; // Update for /data endpoint
      if (consecutiveFailedHeatingAttempts > 0) {
        Serial.println("User set new temp, consecutive heating failures reset.");
      }
      consecutiveFailedHeatingAttempts = 0; // Reset on user temp change
      Serial.print("Desired temperature set to: ");
      Serial.println(desiredTemperatureC, 1);

      // If user sets a new active temperature while in standby, exit standby.
      if (machineIsPresumedOff && newTemp >= PRESUMED_OFF_TEMP_THRESHOLD_C) {
        machineIsPresumedOff = false;
        isMonitoringForMachineOff = false; // Ensure this is also reset
        Serial.println("User set new active temperature. Exiting machine presumed off standby.");
        updateOledStatus("User: Active Temp");
      }
      // If user sets a low temperature while monitoring, stop monitoring.
      if (isMonitoringForMachineOff && newTemp < PRESUMED_OFF_TEMP_THRESHOLD_C) {
        isMonitoringForMachineOff = false;
        Serial.println("User set low temperature. Stopped monitoring for presumed machine off.");
        updateOledStatus("User: Low Temp Set");
      }
      server.send(200, "text/plain", "OK");
    } else {
      Serial.println("Invalid temperature value received.");
      server.send(400, "text/plain", "Invalid temperature value. Must be between 70.0 and 100.0.");
    }
  } else {
    server.send(400, "text/plain", "Missing temp parameter.");
  }
}

void handleWiFiConnection() {
  unsigned long currentMillis = millis();

  switch (currentWiFiState) {
    case WIFI_DISCONNECTED:
      // Try to connect only if WIFI_RETRY_DELAY_MS has passed since last attempt
      if (currentMillis - wifiLastRetryTime >= WIFI_RETRY_DELAY_MS) {
        Serial.println(F("Attempting WiFi connection..."));
        updateOledStatus("WiFi Connecting...");
        WiFi.mode(WIFI_STA); // Ensure STA mode
        WiFi.begin(ssid, password);
        wifiConnectStartTime = currentMillis;
        currentWiFiState = WIFI_CONNECTING;
        wifiLastRetryTime = currentMillis; // Record time of this attempt
        wifiRetryCount++;
        Serial.print(F("WiFi Retry Attempt: ")); Serial.println(wifiRetryCount);
      }
      break;

    case WIFI_CONNECTING:
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println(F(""));
        Serial.println(F("WiFi connected!"));
        Serial.print(F("IP address: "));
        Serial.println(WiFi.localIP());
        updateOledStatus("WiFi Connected");
        currentWiFiState = WIFI_CONNECTED;
        wifiRetryCount = 0; // Reset retry count on successful connection

        // OTA is initialized in setup() and handled in loop() unconditionally.
        // ArduinoOTA.setHostname("esp32-delonghi"); // Moved to setup
        // ... OTA event handlers ... // Moved to setup
        // ArduinoOTA.begin(); // Moved to setup
        // Serial.println(F("OTA Ready")); // Moved to setup

        #ifdef ENABLE_DATETIME_WEATHER_FEATURE
        timeClient.begin();
        Serial.println(F("NTP Client started."));
        #endif

        // Initialize Web Server now that WiFi is up
        server.on("/", HTTP_GET, handleRoot);
        server.on("/data", HTTP_GET, handleData);
        server.on("/settemp", HTTP_POST, handleSetTemp);
        server.on("/resetmaxpressure", HTTP_POST, handleResetMaxPressure); // New route
        server.on("/history", HTTP_GET, handleHistory); // New route for historical data
        server.onNotFound(handleNotFound);
        server.begin();
        Serial.println(F("HTTP server started"));

      } else if (currentMillis - wifiConnectStartTime >= WIFI_CONNECT_TIMEOUT_MS) {
        Serial.println(F("WiFi Connection Timeout."));
        updateOledStatus("WiFi Timeout");
        WiFi.disconnect(true); // Disconnect
        currentWiFiState = WIFI_DISCONNECTED; // Go back to disconnected to retry
        // wifiLastRetryTime is already set from the start of this connection attempt,
        // so it will wait WIFI_RETRY_DELAY_MS before trying again.
        if (wifiRetryCount >= MAX_WIFI_RETRIES_BEFORE_REBOOT) {
          Serial.println(F("Max WiFi retries reached. Rebooting..."));
          updateOledStatus("WiFi Fail Reboot");
          delay(100); // Short delay for serial print
          ESP.restart();
        }
      }
      break;

    case WIFI_CONNECTED:
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("WiFi Disconnected. Attempting to reconnect..."));
        updateOledStatus("WiFi Lost");
        currentWiFiState = WIFI_DISCONNECTED; // Go to disconnected to trigger re-connection logic
        wifiConnectStartTime = 0; // Reset start time for next attempt
        wifiLastRetryTime = currentMillis; // Start retry delay timer from now
        
        // Stop services that depend on WiFi
        server.stop();
        Serial.println(F("HTTP server stopped."));
        #ifdef ENABLE_DATETIME_WEATHER_FEATURE
        timeClient.end(); // Properly stop NTP client if it has an end method
        Serial.println(F("NTP Client stopped."));
        #endif
      }
      break;
  }
}


void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }
  Serial.println("Booting");
  updateOledStatus("Booting..."); // Initial OLED status

  WiFi.mode(WIFI_STA); // Set WiFi mode early for OTA
  
  // Initialize LED_BUILTIN pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Turn LED on initially (HIGH = ON as per user feedback)

  // Initialize Status LED pin
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW); // Turn Status LED off initially

  // Initialize Relay Pin
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH); // Heater OFF (Relay is likely Active LOW, so HIGH is OFF)
  isRelayOn = false; // Ensure state matches
  web_isRelayOn = false;

  // Initialize OLED display
  Wire.begin(); // SDA 21, SCL 22 for ESP32 (default if not specified)
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.display();
  delay(100); // Pause for 2 seconds
  display.clearDisplay();
  display.setRotation(3); // Rotate 270 degrees clockwise (for 90 deg physical clockwise rotation)
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println(F("System Initializing..."));
  display.display();
  delay(500);

  // Initialize WiFi connection handling
  handleWiFiConnection(); // Initial attempt to connect

  // OTA Setup
  ArduinoOTA.setHostname("esp32-delonghi");
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";
      Serial.println("Start updating " + type);
      updateOledStatus("OTA Update...");
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0,0);
      display.println("OTA Update...");
      display.display();
    })
    .onEnd([]() {
      Serial.println("\nEnd");
      updateOledStatus("OTA Done! Reboot...");
      display.clearDisplay();
      display.setCursor(0,0);
      display.println("OTA Done! Reboot...");
      display.display();
      delay(1000);
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0,0);
      display.printf("OTA Progress: %u%%", (progress / (total / 100)));
      display.display();
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      updateOledStatus("OTA Error!");
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0,0);
      display.println("OTA Error!");
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
      display.printf("Error: %u", error);
      display.display();
      delay(2000);
    });

  ArduinoOTA.begin(); // Start OTA

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  updateOledStatus("System Ready");
  digitalWrite(LED_BUILTIN, LOW); // Turn LED off after setup (LOW = OFF as per user feedback)
}


#ifdef ENABLE_DATETIME_WEATHER_FEATURE
void getWeatherData() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String serverPath = weatherApiUrlBase + city + "&appid=" + openWeatherMapApiKey + "&units=" + units;
    
    http.begin(serverPath.c_str()); //Specify request destination
    int httpResponseCode = http.GET();
    
    if (httpResponseCode > 0) {
      String payload = http.getString();
      // Serial.println(payload); // For debugging weather response

      JsonDocument doc; // Use JsonDocument instead of DynamicJsonDocument
      DeserializationError error = deserializeJson(doc, payload);

      if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        currentWeatherDataTemp = NAN; // Indicate error
        return;
      }
      
      JsonObject main = doc["main"];
      currentWeatherDataTemp = main["temp"]; // مثال: 29.09
      // Serial.print("Weather Temp: "); Serial.println(currentWeatherDataTemp);

    } else {
      Serial.print("Error on HTTP request: ");
      Serial.println(httpResponseCode);
      currentWeatherDataTemp = NAN; // Indicate error
    }
    http.end(); //Free resources
  } else {
    // Serial.println("WiFi not connected, skipping weather update.");
    currentWeatherDataTemp = NAN; // Indicate no data due to WiFi
  }
}
#endif


void loop() {
  unsigned long currentMillis = millis();

  // Handle WiFi connection state
  handleWiFiConnection();

  // Handle OTA updates if WiFi is connected
  if (currentWiFiState == WIFI_CONNECTED) {
    ArduinoOTA.handle();
    server.handleClient(); // Handle web server requests

    #ifdef ENABLE_DATETIME_WEATHER_FEATURE
    // Update time from NTP
    timeClient.update();
    currentTimeStr = timeClient.getFormattedTime(); // HH:MM:SS (Restored seconds)
    
    // Date and Day of Week calculation removed as they are no longer displayed
    // time_t epochTime = timeClient.getEpochTime();
    // struct tm *ptm = localtime(&epochTime);
    // char dateBuffer[10];
    // sprintf(dateBuffer, "%02d/%02d", ptm->tm_mday, ptm->tm_mon + 1);
    // currentDateStr = String(dateBuffer);
    //
    // char dayBuffer[4];
    // strftime(dayBuffer, sizeof(dayBuffer), "%a", ptm); // Short day name e.g. "Mon"
    // currentDayOfWeekStr = String(dayBuffer);

    // Get weather data periodically
    if (currentMillis - lastWeatherReadTime >= weatherReadInterval || isnan(currentWeatherDataTemp) && lastWeatherReadTime == 0) {
      getWeatherData();
      lastWeatherReadTime = currentMillis;
    }
    #endif
  }


  // --- Temperature Reading and Smoothing ---
  double smoothedTempC = NAN; // Default to NAN if no valid reading
  if (currentMillis - lastTempReadTime >= tempReadInterval) {
    lastTempReadTime = currentMillis;
    double rawTempC = thermocouple.readCelsius();

    if (isnan(rawTempC)) { // Check raw temperature for validity
      Serial.println("Failed to read from thermocouple sensor!");
      updateOledStatus("Thermo Err");
      // smoothedTempC_EMA remains as is, or could be set to NAN to indicate error propagation
    } else {
      // Get calibrated temperature
      double calibratedTempC = getCalibratedTemperature(rawTempC);

      // --- EMA Smoothing Logic ---
      if (isnan(smoothedTempC_EMA)) { // If it's the first valid reading or after an error/reset
        smoothedTempC_EMA = calibratedTempC;
      } else {
        // Apply EMA formula: EMA_new = alpha * new_value + (1 - alpha) * EMA_old
        smoothedTempC_EMA = (TEMP_EMA_ALPHA * calibratedTempC) + ((1.0f - TEMP_EMA_ALPHA) * smoothedTempC_EMA);
      }
      smoothedTempC = smoothedTempC_EMA; // This is the smoothed value to be used by the rest of the system
      
      web_smoothedTempC = smoothedTempC; // Update web data
      web_desiredTempC = desiredTemperatureC; // Ensure web_desiredTempC is kept up-to-date
      // --- End EMA Smoothing Logic ---
    }
  } else {
    // If not time to read, use the last known smoothed value
    smoothedTempC = smoothedTempC_EMA;
  }
  // Ensure web_smoothedTempC is always updated, even if no new reading this cycle
  // This might be redundant if smoothedTempC is always assigned from smoothedTempC_EMA
  if (!isnan(smoothedTempC)) {
      web_smoothedTempC = smoothedTempC;
  }

  // --- Server-side Plot Pause Logic (Temperature) ---
  if (!isnan(smoothedTempC)) {
    if (smoothedTempC < PRESUMED_OFF_TEMP_THRESHOLD_C) {
      if (!web_isTempPlotPaused) {
        Serial.println("Server: Temperature plot paused.");
        web_isTempPlotPaused = true;
      }
    } else {
      if (web_isTempPlotPaused) {
        Serial.println("Server: Temperature plot resumed. Clearing history.");
        web_isTempPlotPaused = false;
        // Clear history so plot restarts cleanly on client
        tempHistoryCount = 0;
        tempHistoryIndex = 0;
        pressureHistoryCount = 0; // Also clear pressure history for consistency
        pressureHistoryIndex = 0;
        maxObservedPressure = 0.0f; // And max pressure
      }
    }
  }

  // --- Built-in LED Blinking Logic ---
  // LED_BUILTIN: HIGH = ON, LOW = OFF (as per user feedback)

  if (machineIsPresumedOff) {
    // Machine is presumed off: LED OFF
    digitalWrite(LED_BUILTIN, LOW); // LED OFF (LOW = OFF)
  } else if (!isnan(smoothedTempC) && smoothedTempC > 103.0) {
    // 5. Temperature above 103°C: Very rapid blinking
    if (currentMillis - lastBlinkTimeLed >= blinkIntervalVeryRapid) {
      lastBlinkTimeLed = currentMillis;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Toggle LED
    }
  } else if (currentHeaterState == HEATING && smoothedTempC > 80.0) {
    // 2. Heating: Fast blinking
    if (currentMillis - lastBlinkTimeLed >= blinkIntervalRapid) {
      lastBlinkTimeLed = currentMillis;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Toggle LED
    }
  } else if (!isnan(smoothedTempC) && (smoothedTempC >= desiredTemperatureC - 1.0 && smoothedTempC <= desiredTemperatureC + 1.0)) {
    // 3. Temperature in +/-1 of set temperature: Steady on
    digitalWrite(LED_BUILTIN, HIGH); // LED ON (HIGH = ON)
  } else if (currentHeaterState == SETTLING) {
    // 4. Temperature not in +/-1 of set temperature and settle check is going on: Slow blinking
    // This condition is met if the previous "steady on" condition was false.
    if (currentMillis - lastBlinkTimeLed >= blinkIntervalSlow) {
      lastBlinkTimeLed = currentMillis;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Toggle LED
    }
  } else {
    // 1. Default led state (when not presumed off and no other condition met): light off
    // This includes IDLE state when not meeting other conditions.
    digitalWrite(LED_BUILTIN, LOW); // LED OFF (LOW = OFF)
  }

  // --- Status LED on GPIO27 ---
  if (currentMillis - lastStatusLedToggleTime >= statusLedToggleInterval) {
    lastStatusLedToggleTime = currentMillis;
    digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN)); // Toggle Status LED
  }


  // --- Pressure Reading and Smoothing ---
  // Removed timed interval to align with older commit's behavior (read every loop)
  // lastPressureReadTime = currentMillis; // Removed

  int stableAdcValue = getStableAdcValue(); // This function now handles its own timing/sampling

  // SMA for ADC readings
    if (numPressureAdcValuesStored == PRESSURE_SMOOTHING_SAMPLES) {
      totalPressureAdcSmoothingSum -= pressureAdcSmoothingBuffer[currentPressureAdcSmoothingIndex];
    }
    pressureAdcSmoothingBuffer[currentPressureAdcSmoothingIndex] = stableAdcValue;
    totalPressureAdcSmoothingSum += stableAdcValue;
    currentPressureAdcSmoothingIndex = (currentPressureAdcSmoothingIndex + 1) % PRESSURE_SMOOTHING_SAMPLES;
    if (numPressureAdcValuesStored < PRESSURE_SMOOTHING_SAMPLES) {
      numPressureAdcValuesStored++;
    }
    double smoothedAdcValue = totalPressureAdcSmoothingSum / numPressureAdcValuesStored;
    
    // Convert ADC value to voltage
    float voltage = (smoothedAdcValue / ESP32_ADC_MAX_VALUE) * ESP32_ADC_MAX_VOLTAGE;
    // Convert voltage to pressure (bar)
    float currentPressureBar = ((voltage - VOLTS_AT_0_BAR) / (VOLTS_AT_16_BAR - VOLTS_AT_0_BAR)) * PRESSURE_MAX_BAR;
    // Clamp pressure to 0 if calculated as negative
    if (currentPressureBar < 0) {
      currentPressureBar = 0.0f;
    }
    web_pressureBar = currentPressureBar; // Update web data with the current (smoothed) pressure

    // --- Server-side Plot Pause Logic (Pressure) & Shot Timer ---
    if (!isnan(currentPressureBar)) {
        // Shot Timer Start
        if (currentPressureBar >= 2.0f && !isShotRunning && !web_isPressurePlotPaused) {
            isShotRunning = true;
            shotStartTime_ms = millis();
            web_shotDuration_ms = 0;
            Serial.println("Shot timer started.");
        }

        // Update shot duration if running
        if (isShotRunning) {
            web_shotDuration_ms = millis() - shotStartTime_ms;
        }

        // Plot Pause & Shot Timer Stop
        if (currentPressureBar < 1.7 && lastPressureForPauseCheck_server >= 1.7) {
            if (!web_isPressurePlotPaused) {
                Serial.println("Server: Pressure plot paused.");
                web_isPressurePlotPaused = true;
                if (isShotRunning) {
                    isShotRunning = false; // Stop the timer, final value is already set
                    Serial.print("Shot timer stopped. Duration: ");
                    Serial.print(web_shotDuration_ms / 1000.0, 1);
                    Serial.println("s");
                }
            }
        } else if (currentPressureBar >= PRESSURE_RESUME_THRESHOLD_BAR) {
            if (web_isPressurePlotPaused) {
                Serial.println("Server: Pressure plot resumed. Clearing history.");
                web_isPressurePlotPaused = false;
                // Clear history so plot restarts cleanly on client
                tempHistoryCount = 0;
                tempHistoryIndex = 0;
                pressureHistoryCount = 0;
                pressureHistoryIndex = 0;
                maxObservedPressure = 0.0f;
                web_shotDuration_ms = 0; // Reset shot timer display
            }
        }
        lastPressureForPauseCheck_server = currentPressureBar;
    }

    // --- Max Pressure Stability Check ---
    pressureMaxStabilityBuffer[pressureMaxStabilityIndex] = currentPressureBar;
    pressureMaxStabilityIndex = (pressureMaxStabilityIndex + 1) % PRESSURE_STABILITY_SAMPLES_FOR_MAX;
    if (pressureMaxStabilityCount < PRESSURE_STABILITY_SAMPLES_FOR_MAX) {
      pressureMaxStabilityCount++;
    }

    if (pressureMaxStabilityCount == PRESSURE_STABILITY_SAMPLES_FOR_MAX) {
      float minVal = pressureMaxStabilityBuffer[0];
      float maxVal = pressureMaxStabilityBuffer[0];
      for (int i = 1; i < PRESSURE_STABILITY_SAMPLES_FOR_MAX; i++) {
        if (pressureMaxStabilityBuffer[i] < minVal) minVal = pressureMaxStabilityBuffer[i];
        if (pressureMaxStabilityBuffer[i] > maxVal) maxVal = pressureMaxStabilityBuffer[i];
      }

      if ((maxVal - minVal) <= PRESSURE_STABILITY_THRESHOLD_FOR_MAX) {
        // Pressure is considered stable for max pressure update
        float sumStable = 0;
        for(int i=0; i<PRESSURE_STABILITY_SAMPLES_FOR_MAX; ++i) sumStable += pressureMaxStabilityBuffer[i];
        float stablePressureForMax = sumStable / PRESSURE_STABILITY_SAMPLES_FOR_MAX;
        
        if (stablePressureForMax > maxObservedPressure) {
          maxObservedPressure = stablePressureForMax;
        }
      }
    }
    // --- End Max Pressure Stability Check ---
  // } // Removed closing bracket of the timed interval

  // --- History Sampling ---
  if (currentMillis - lastHistorySampleTime >= historySampleInterval) {
    lastHistorySampleTime = currentMillis;

    if (!isnan(smoothedTempC) && !web_isTempPlotPaused) {
      tempHistory[tempHistoryIndex] = {currentMillis, (float)smoothedTempC};
      tempHistoryIndex = (tempHistoryIndex + 1) % HISTORY_SIZE;
      if (tempHistoryCount < HISTORY_SIZE) {
        tempHistoryCount++;
      }
    }

    if (!isnan(web_pressureBar) && !web_isPressurePlotPaused) {
      pressureHistory[pressureHistoryIndex] = {currentMillis, web_pressureBar};
      pressureHistoryIndex = (pressureHistoryIndex + 1) % HISTORY_SIZE;
      if (pressureHistoryCount < HISTORY_SIZE) {
        pressureHistoryCount++;
      }
    }
  }

  // --- Main Heater Control State Machine ---
  // Only run if smoothedTempC is a valid number
  if (!isnan(smoothedTempC)) {
    switch (currentHeaterState) {
        case IDLE:
          // --- Early Cutoff Cooldown Check ---
          if (inEarlyCutoffCooldown) {
            if (currentMillis >= earlyCutoffCooldownEndTime) {
              inEarlyCutoffCooldown = false; // Cooldown finished
              Serial.println("IDLE: Early cutoff cooldown finished.");
              updateOledStatus("Cooldown Over");
            } else {
              // Serial.println("IDLE: In early cutoff cooldown. Skipping heating checks.");
              break; // Exit IDLE state logic for this loop iteration
            }
          }
          // --- End Early Cutoff Cooldown Check ---

          // --- Machine Presumed Off Logic ---
          if (machineIsPresumedOff) {
            digitalWrite(RELAY_PIN, LOW); // Keep relay ON (LOW) in standby
            isRelayOn = true;
            web_isRelayOn = true;

            if (currentMillis - lastRateCheckTime >= RATE_CHECK_INTERVAL_MS) {
              float deltaTimeSeconds = (float)(currentMillis - lastRateCheckTime) / 1000.0f;
              if (deltaTimeSeconds > 0) {
                float rateOfChange = (smoothedTempC - previousTempForRateCheck) / deltaTimeSeconds;
                previousTempForRateCheck = smoothedTempC; // Update for next check
                lastRateCheckTime = currentMillis;

                if (rateOfChange > 0.1) { // If temp rises by more than 0.1 C/sec (e.g. machine turned on)
                  machineIsPresumedOff = false;
                  isMonitoringForMachineOff = false; // Reset monitoring flag
                  if (consecutiveFailedHeatingAttempts > 0) {
                    Serial.println("Machine power detected, consecutive heating failures reset.");
                  }
                  consecutiveFailedHeatingAttempts = 0; // Reset on machine power detection
                  Serial.println("Machine power detected (temp rise rate). Exiting standby, resuming normal control.");
                  updateOledStatus("Machine On");
                  // Fall through to normal IDLE logic below in the same cycle or next
                }
              }
            }
            break; // In presumed off mode, skip normal IDLE heating logic
          }
          // --- End Machine Presumed Off Logic ---


          // Normal IDLE state operation (not in cooldown, not in presumed off)
          // --- New Machine Presumed Off Monitoring Logic ---
          if (!machineIsPresumedOff) { // Only monitor if not already presumed off
            // Start/Continue Monitoring Condition:
            // Monitor if current temp is below threshold AND system is trying to maintain a temp at or above threshold
            if (smoothedTempC < PRESUMED_OFF_TEMP_THRESHOLD_C && desiredTemperatureC >= PRESUMED_OFF_TEMP_THRESHOLD_C) {
              if (!isMonitoringForMachineOff) { // Start of monitoring
                isMonitoringForMachineOff = true;
                machineOffMonitorStartTime = currentMillis;
                lastTempDuringMachineOffMonitoring = smoothedTempC;
                lastMachineOffCheckTimestamp = currentMillis;
                Serial.println("Temp < THRESHOLD & desired is high. Starting to monitor for presumed machine off.");
                updateOledStatus("Monitoring Power..."); // Plan's message
              } else { // Continue monitoring
                if (currentMillis - lastMachineOffCheckTimestamp >= PRESUMED_OFF_CHECK_INTERVAL_MS) {
                  if (smoothedTempC <= lastTempDuringMachineOffMonitoring) { // Temp is decreasing or stable
                    lastTempDuringMachineOffMonitoring = smoothedTempC; // Update with current temp
                    bool tempLowForDuration = (currentMillis - machineOffMonitorStartTime >= PRESUMED_OFF_DURATION_MS);
                    bool maxFailuresReached = (consecutiveFailedHeatingAttempts >= MAX_CONSECUTIVE_HEATING_FAILURES);

                    if (tempLowForDuration || maxFailuresReached) {
                      machineIsPresumedOff = true;
                      web_early_cutoff_signal = true; // Signal client for plot reset
                      isMonitoringForMachineOff = false; // Stop monitoring once presumed off
                      previousTempForRateCheck = smoothedTempC; // Init for power-on detection
                      lastRateCheckTime = currentMillis;      // Init for power-on detection

                      if (maxFailuresReached) {
                        Serial.println("Max heating failures reached. Machine presumed off due to heating issues.");
                        // web_early_cutoff_signal is already set above if this block is reached
                        updateOledStatus("Err: Heat Fail");
                      } else { // tempLowForDuration must be true
                        Serial.println("Temp consistently low for PRESUMED_OFF_DURATION_MS. Machine presumed off.");
                        updateOledStatus("Machine Off. Relay On");
                      }

                      // Ensure heater is ON when machine is presumed off
                      digitalWrite(RELAY_PIN, LOW); // Heater ON (Relay is Active LOW)
                      isRelayOn = true;
                      web_isRelayOn = true;
                      Serial.println(F("IDLE: Heater activated for presumed machine off state."));
                      // Built-in LED will be turned OFF by the logic at the beginning of loop() for machineIsPresumedOff.
                    }
                    // else, still waiting for PRESUMED_OFF_DURATION_MS to elapse
                  } else { // Temp has increased while monitoring below threshold (smoothedTempC > lastTempDuringMachineOffMonitoring)
                    isMonitoringForMachineOff = false; // Machine activity detected, stop monitoring
                    Serial.println("Temp increased while monitoring for presumed off. Resetting monitoring.");
                    updateOledStatus("Monitoring Halted"); // Plan's message
                  }
                  lastMachineOffCheckTimestamp = currentMillis; // Reset check interval timestamp
                }
              }
            } else { // Start/Continue Monitoring Condition NOT met (e.g., temp rose above threshold, or desired temp was lowered below threshold)
              if (isMonitoringForMachineOff) { // If we were monitoring
                isMonitoringForMachineOff = false; // Stop monitoring
                Serial.println("Monitoring condition (temp < THRESHOLD or desired >= THRESHOLD) no longer met. Stopped monitoring.");
                updateOledStatus("Monitoring Stopped"); // Plan's message
              }
            }
          }
          // --- End New Machine Presumed Off Monitoring Logic ---

          // If not presumed off AND not currently monitoring for it, then proceed with normal heating logic
          if (!machineIsPresumedOff) { // Allow heating even if monitoring, as long as not yet presumed off
            // Check if temperature has dropped enough below desired to start heating
            // The -1.0 acts as a hysteresis or deadband.
            double tempDifferenceToDesired = desiredTemperatureC - smoothedTempC;
            if (tempDifferenceToDesired >= .5) {
              Serial.print("IDLE: Triggering heat. Current T: "); Serial.print(smoothedTempC, 1);
              Serial.print("C, Desired T: "); Serial.print(desiredTemperatureC, 1); Serial.println("C.");
              updateOledStatus("Heating...");

              unsigned long calculatedHeatDurationMs = (unsigned long)(tempDifferenceToDesired * HEATER_SECONDS_PER_DEGREE_C * 1000.0f);
              
              if (calculatedHeatDurationMs > MAX_HEATER_ON_DURATION_MS) {
                calculatedHeatDurationMs = MAX_HEATER_ON_DURATION_MS;
                Serial.println("Heater duration capped by MAX_HEATER_ON_DURATION_MS");
                
              }
              if (calculatedHeatDurationMs < 2000 && tempDifferenceToDesired > 0.1) { // Min 2 sec heating if meaningfully below
                  calculatedHeatDurationMs = 2000;
              }

              if (calculatedHeatDurationMs >= 2000) { // Only heat if duration is meaningful
                digitalWrite(RELAY_PIN, LOW); // Heater ON
                isRelayOn = true;
                web_isRelayOn = true;
                currentHeaterState = HEATING;
                heaterStopTimeMs = currentMillis + calculatedHeatDurationMs;
                lastCalculatedHeatDurationMs = calculatedHeatDurationMs; // Store for early cutoff logic
                Serial.print("Calculated heat duration: ");
                Serial.print(calculatedHeatDurationMs / 1000.0f, 1);
                Serial.println("s. State: HEATING");
                // updateOledStatus("Heating..."); // Already set
              }
            }
          }
          break;

        case HEATING:
          // Early cutoff for long heating cycles if temp reaches EARLY_CUTOFF_TEMP_C
          if (lastCalculatedHeatDurationMs > 30000 && smoothedTempC >= EARLY_CUTOFF_TEMP_C) {
            digitalWrite(RELAY_PIN, HIGH); // Heater OFF
            isRelayOn = false;
            web_isRelayOn = false;
            inEarlyCutoffCooldown = true; // Activate cooldown
            earlyCutoffCooldownEndTime = currentMillis + EARLY_CUTOFF_COOLDOWN_DURATION_MS;
            currentHeaterState = SETTLING;
            settlingCheckStartTimeMs = currentMillis;
            tempAtSettlingCheckStartC = smoothedTempC;
            Serial.print("HEATING: Early cutoff for long heat cycle. Trigger: ");
            Serial.print(EARLY_CUTOFF_TEMP_C, 1); Serial.print("C. Current Temp: ");
            Serial.print(smoothedTempC, 1);
            Serial.println("C. State: SETTLING.");
            updateOledStatus("EarlyCutoffSetlng");
          }
          // Original timer-based logic
          else if (currentMillis >= heaterStopTimeMs) { // Time to check if we should stop or continue
            bool shouldContinueHeating = false;
            // If timer is up, check if we are still below the early cutoff threshold AND below desired temp
            if (smoothedTempC < EARLY_CUTOFF_TEMP_C) { 
              double tempDifferenceToDesired = desiredTemperatureC - smoothedTempC;
              // Only continue heating if we are meaningfully below desired temp
              if (tempDifferenceToDesired > 0.1) { 
                unsigned long remainingHeatDurationMs = (unsigned long)(tempDifferenceToDesired * HEATER_SECONDS_PER_DEGREE_C * 1000.0f);
                
                if (remainingHeatDurationMs > MAX_HEATER_ON_DURATION_MS) remainingHeatDurationMs = MAX_HEATER_ON_DURATION_MS;
                
                if (remainingHeatDurationMs < 1000 && tempDifferenceToDesired > 0.1) { // Min 1 sec more if needed
                    remainingHeatDurationMs = 1000;
                }

                if (remainingHeatDurationMs >= 1000) {
                    heaterStopTimeMs = currentMillis + remainingHeatDurationMs; // Extend heating time
                    lastCalculatedHeatDurationMs = remainingHeatDurationMs; // Update for next potential early cutoff check
                    shouldContinueHeating = true;
                    Serial.print("HEATING: Timer up, but <EARLY_CUTOFF_TEMP_C & <desired. Continuing for ");
                    Serial.print(remainingHeatDurationMs / 1000.0f, 1);
                    Serial.println("s more.");
                    updateOledStatus("Heating Cont.");
                }
              }
            }

            if (!shouldContinueHeating) {
              digitalWrite(RELAY_PIN, HIGH); // Heater OFF
              isRelayOn = false;
              web_isRelayOn = false;
              // After heating, always go to SETTLING to observe temperature behavior
              currentHeaterState = SETTLING;
              settlingCheckStartTimeMs = currentMillis;
              tempAtSettlingCheckStartC = smoothedTempC;
              Serial.println("State: SETTLING. Starting observation.");
              updateOledStatus("Settling...");
            }
          }
          break;

        case SETTLING:
          if (currentMillis - settlingCheckStartTimeMs >= SETTLED_OBSERVATION_PERIOD_MS) {
            // Observation period ended, check temperature rise
            double tempRiseDuringObservation = smoothedTempC - tempAtSettlingCheckStartC;
            if (tempRiseDuringObservation <= SETTLED_TEMP_RISE_MAX_C) {
              // Temperature is considered settled
              currentHeaterState = IDLE;
              isMonitoringForMachineOff = false; // Reset monitoring flag
              Serial.print("Temp rise: "); Serial.print(tempRiseDuringObservation, 2);
              Serial.println("C. Settled. State: IDLE");

              // Check if heating was successful or if it's a failed attempt
              if (smoothedTempC < (desiredTemperatureC - TEMP_DIFF_THRESHOLD_FOR_HEATING_FAILURE)) {
                consecutiveFailedHeatingAttempts++;
                Serial.print("Heating attempt considered failed. Consecutive failures: ");
                Serial.println(consecutiveFailedHeatingAttempts);
                if (consecutiveFailedHeatingAttempts >= MAX_CONSECUTIVE_HEATING_FAILURES) {
                  Serial.println("Max heating failures reached. Machine will enter presumed off state.");
                  updateOledStatus("Max Heat Fails");
                } else {
                  updateOledStatus("Heat Fail #" + String(consecutiveFailedHeatingAttempts));
                }
              } else {
                if (consecutiveFailedHeatingAttempts > 0) { // If there were failures, log the reset
                    Serial.println("Heating successful or temp acceptable, consecutive failures reset.");
                }
                consecutiveFailedHeatingAttempts = 0; // Reset on successful/acceptable heating outcome
                updateOledStatus("Idle (Settled)");
              }
            } else {
              // Temperature still rising too much, restart observation window
              settlingCheckStartTimeMs = currentMillis;
              tempAtSettlingCheckStartC = smoothedTempC;
              Serial.print("Temp rise: "); Serial.print(tempRiseDuringObservation, 2);
              Serial.println("C. Not settled. Restarting observation.");
              updateOledStatus("Temp rise.Resettle CHK");
            }
          }
          // If still in settling period, do nothing, just wait. Heater is already off.
          break;
    }
  }


  // --- OLED Display Update ---
  // Only update display if it's time (e.g., every 500ms or 1s) to avoid flicker and save CPU
  // This timing is implicitly handled by the tempReadInterval for now, which is 500ms.
  // If tempReadInterval becomes very short, add a separate display update timer.
  if (currentMillis - lastTempReadTime < tempReadInterval) { // Use the same timing as temp reading for now
    display.clearDisplay();

    int current_y = 0;
    const int gap = 6; // Consistent gap between major items
    const int small_gap = 4; // Smaller gap for related items like time/weather or status lines

    // Line 1: Current Temperature
    display.setTextSize(2);
    display.setCursor(0, current_y);
    if (isnan(smoothedTempC)) {
      display.print(F("--.-"));
    } else {
      display.print(smoothedTempC, 1);
    }
    display.setTextSize(1); // Smaller for unit
    display.print(F("C"));
    current_y += 16; // Height of TextSize 2

    // Line 2: Desired Temperature
    current_y += gap;
    display.setTextSize(1);
    display.setCursor(0, current_y);
    display.print(F("S:"));
    display.print(desiredTemperatureC, 1);
    display.print(F("C"));
    current_y += 8; // Height of TextSize 1

    // Line 3: Current Pressure
    current_y += gap;
    display.setTextSize(2);
    display.setCursor(0, current_y);
    if (isnan(web_pressureBar)) {
      display.print(F("--.-"));
    } else {
      display.print(web_pressureBar, 1);
    }
    display.setTextSize(1); // Smaller for unit
    display.print(F("bar"));
    current_y += 16; // Height of TextSize 2

    // Line 4: Max Pressure
    current_y += gap;
    display.setTextSize(1);
    display.setCursor(0, current_y);
    display.print(F("MxP:"));
    if (isnan(maxObservedPressure) || maxObservedPressure < 0.01) {
      display.print(F("--.-"));
    } else {
      display.print(maxObservedPressure, 1);
    }
    current_y += 8; // Height of TextSize 1

    // Line 5: Shot Timer
    current_y += gap;
    display.setTextSize(2);
    display.setCursor(0, current_y);
    if (isShotRunning || web_shotDuration_ms > 0) {
        display.print(web_shotDuration_ms / 1000.0, 1);
    } else {
        display.print(F("--.-"));
    }
    display.setTextSize(1);
    display.print(F("s"));
    current_y += 16; // Height of TextSize 2

    // Line 6: Status Message (single line)
    current_y += gap;
    display.setTextSize(1);
    display.setCursor(0, current_y);
    String statusMsg = oledStatusMessage;
    int maxCharsPerLine = display.width() / 6;
    if (maxCharsPerLine == 0) maxCharsPerLine = 10;
    if (statusMsg.length() > maxCharsPerLine) {
        statusMsg = statusMsg.substring(0, maxCharsPerLine);
    }
    display.print(statusMsg);
    current_y += 8;

#ifdef ENABLE_DATETIME_WEATHER_FEATURE
    // Line 7: Time and External Weather Temperature
    current_y += small_gap;
    display.setCursor(0, current_y);
    display.print(currentTimeStr.substring(0, 8)); // HH:MM
    
    String weatherStr = " E:--C";
    if (!isnan(currentWeatherDataTemp)) {
        weatherStr = "    E:" + String(currentWeatherDataTemp, 0) + "C";
    }
    display.print(weatherStr);
    current_y += 8;
#endif

    display.display();
  }
}