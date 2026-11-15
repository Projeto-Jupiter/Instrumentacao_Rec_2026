/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h>
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    const char *name;        // no commas allowed (they separate protocol fields)
    float counts_per_gram;   // 0.0f = not calibrated yet
} LoadCell_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define FW_VERSION           "2.0"

// ---- Load cells -------------------------------------------------------------
// Factors in counts per gram. The PC app remembers its own calibrations and
// sends them back with SCALE after every reset, so editing this table is only
// needed if you also want the firmware to start with the right value.
static const LoadCell_t load_cells[] = {
    { "Load cell 1", 9.4076f},
    { "Load cell 2", 0.0f   },   // not calibrated yet
    { "Load cell 3", 0.0f   },   // not calibrated yet
};
#define NUM_CELLS            ((int)(sizeof(load_cells) / sizeof(load_cells[0])))
#define DEFAULT_CELL         0       // index in the table above (0 = first)

// ---- Physics ----------------------------------------------------------------
//   grams   = (filtered_counts - offset) / scale   (scale = counts per gram)
//   force_N = grams / 1000 * g
#define GRAVITY_MS2          9.80665f    // m/s^2
#define GRAM_TO_NEWTON       (GRAVITY_MS2 / 1000.0f)

// ---- Filtering --------------------------------------------------------------
#define MEDIAN_N             5       // sliding median window (odd): kills spikes
#define EMA_ALPHA            0.15f   // 0..1, smaller = smoother but slower

// ---- Zeroing ----------------------------------------------------------------
#define TARE_SAMPLES         20      // samples averaged for a tare (~2 s at 10 SPS)
#define ZERO_DEADBAND_N      0.05f   // |force| below this is reported as exactly 0
#define AUTO_ZERO_ENABLE     1       // default state of the auto-zero (AUTOZERO cmd)
#define AUTO_ZERO_WINDOW_N   0.20f   // only track if reading is within +-this of zero
#define AUTO_ZERO_RATE       0.02f   // how fast the offset follows (per sample)

// ---- Calibration ------------------------------------------------------------
#define CAL_SETTLE_SAMPLES   15      // samples discarded after the weight is placed
#define CAL_AVG_SAMPLES      100     // samples averaged (~10 s at 10 SPS)
#define CAL_MIN_DELTA_COUNTS 50      // smaller change than this = weight not detected

// ---- HX711 limits -----------------------------------------------------------
// A reading stuck at either limit means the input is out of range (overload,
// bad wiring, floating input, unbalanced bridge): the value is meaningless.
#define HX711_SAT_HI         8388607
#define HX711_SAT_LO         (-8388608)

// ---- Serial protocol --------------------------------------------------------
#define RX_BUF_SIZE          256
#define LINE_MAX_LEN         64
#define DEFAULT_INTERVAL_MS  2000    // time between F lines until INTERVAL is sent
#define INTERVAL_MIN_MS      100
#define INTERVAL_MAX_MS      60000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;

// USB receive ring buffer, filled from CDC_Receive_FS() (USB interrupt)
static volatile uint8_t  rx_buf[RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

// Command line being assembled from the received bytes
static char line_buf[LINE_MAX_LEN];
static int  line_len      = 0;
static int  line_overflow = 0;

// USB transmit buffer. CDC_Transmit_FS() does not copy the data, so the bytes
// must stay untouched until the transfer finishes.
static uint8_t tx_buf[256];
static int     tx_stalled = 0;       // host is not reading: drop output, don't wait

static int      streaming     = 0;   // 1 = send F lines
static uint32_t interval_ms   = DEFAULT_INTERVAL_MS;
static uint32_t last_print_ms = 0;
static int      autozero_on   = AUTO_ZERO_ENABLE;
static int      cal_hold      = 0;   // 1 between "CAL ZERO" and "CAL <grams>"
static int      got_command   = 0;   // stops the "ready" reminder
static int      active_cell   = DEFAULT_CELL;
static float    cell_scale[NUM_CELLS];   // runtime factors, counts per gram
static float    scale_cpg     = 1.0f;    // factor of the active cell

static int32_t  last_raw      = 0;
static int32_t  median_buf[MEDIAN_N];
static int      median_count  = 0;
static int      median_idx    = 0;

static float    offset_counts = 0.0f;
static float    ema_value     = 0.0f;
static int      ema_ready     = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ---------------- USB output -------------------------------------------------
static int Cdc_Busy(void) {
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
    return (hcdc == NULL) || (hcdc->TxState != 0);
}

// Called by printf() and by Tx_Line()
int _write(int file, char *ptr, int len) {
    int sent = 0;
    while (sent < len) {
        uint32_t t0 = HAL_GetTick();
        while (Cdc_Busy()) {
            if (tx_stalled) return len;                 // nobody is listening
            if (HAL_GetTick() - t0 > 50) { tx_stalled = 1; return len; }
        }
        tx_stalled = 0;

        int chunk = len - sent;
        if (chunk > (int)sizeof(tx_buf)) chunk = (int)sizeof(tx_buf);
        memcpy(tx_buf, ptr + sent, (size_t)chunk);
        if (CDC_Transmit_FS(tx_buf, (uint16_t)chunk) != USBD_OK) return len;
        sent += chunk;
    }
    return len;
}

// Sends one complete line ("...\r\n") in a single USB transfer
static void Tx_Line(const char *fmt, ...) {
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf) - 2, fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n > (int)sizeof(buf) - 3) n = (int)sizeof(buf) - 3;
    buf[n++] = '\r';
    buf[n++] = '\n';
    _write(1, buf, n);
}

// Formats a float with fixed decimals (newlib-nano has no float printf)
static void Fmt_Fixed(char *out, size_t n, float v, int decimals) {
    long p = 1;
    for (int i = 0; i < decimals; i++) p *= 10;

    int neg = (v < 0);
    if (neg) v = -v;

    long whole = (long)v;
    long frac  = (long)((v - (float)whole) * (float)p + 0.5f);
    if (frac >= p) { whole++; frac -= p; }
    if (whole == 0 && frac == 0) neg = 0;      // never print "-0.000"

    snprintf(out, n, "%s%ld.%0*ld", neg ? "-" : "", whole, decimals, frac);
}

static inline float absf_(float x) { return x < 0 ? -x : x; }

// ---------------- USB receive (called from usbd_cdc_if.c) --------------------
void App_OnUsbRx(const uint8_t *buf, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        uint16_t next = (rx_head + 1) % RX_BUF_SIZE;
        if (next != rx_tail) {            // drop the byte if the buffer is full
            rx_buf[rx_head] = buf[i];
            rx_head = next;
        }
    }
}

// Returns 1 and a byte if one is available, else 0
static int Rx_Get(uint8_t *c) {
    if (rx_tail == rx_head) return 0;
    *c = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1) % RX_BUF_SIZE;
    return 1;
}

// ---------------- HX711 ------------------------------------------------------
// Returns 1 on success, 0 on timeout (HX711 not responding)
int HX711_Read(int32_t *out) {
    uint32_t t0 = HAL_GetTick();

    // Wait for the HX711 to be ready (DT goes low)
    while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET) {
        if (HAL_GetTick() - t0 > 1000) return 0;
    }

    uint32_t count = 0;

    // SCK held high > 60 us puts the HX711 into power-down, so an interrupt
    // (e.g. USB) in the middle of the read would corrupt it. Block IRQs briefly.
    __disable_irq();

    for (int i = 0; i < 24; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);   // SCK High
        for (volatile int d = 0; d < 10; d++) {}

        count <<= 1;

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // SCK Low
        for (volatile int d = 0; d < 10; d++) {}

        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET) {
            count++;
        }
    }

    // 25th pulse: next conversion = Channel A, gain 128
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    for (volatile int d = 0; d < 10; d++) {}
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

    __enable_irq();

    // 24-bit two's complement -> 32-bit signed
    if (count & 0x00800000UL) {
        count |= 0xFF000000UL;
    }

    *out = (int32_t)count;
    return 1;
}

// Averages n raw readings. Returns how many were actually read.
// avg / mn / mx may be NULL if not needed (mn/mx = min and max raw value).
static int Average_Samples(int n, float *avg, int32_t *mn, int32_t *mx) {
    int64_t sum = 0;
    int got = 0;
    int32_t lo = 0, hi = 0;

    for (int i = 0; i < n; i++) {
        int32_t v;
        if (!HX711_Read(&v)) continue;
        if (got == 0 || v < lo) lo = v;
        if (got == 0 || v > hi) hi = v;
        sum += v;
        got++;
    }

    if (avg) *avg = (got > 0) ? (float)((double)sum / (double)got) : 0.0f;
    if (mn)  *mn = lo;
    if (mx)  *mx = hi;
    return got;
}

// ---------------- Filtering --------------------------------------------------
// Median of the last MEDIAN_N samples (removes single-sample spikes)
static int32_t Median_Push(int32_t sample) {
    median_buf[median_idx] = sample;
    median_idx = (median_idx + 1) % MEDIAN_N;
    if (median_count < MEDIAN_N) median_count++;

    int32_t tmp[MEDIAN_N];
    memcpy(tmp, median_buf, sizeof(int32_t) * (size_t)median_count);

    // insertion sort (tiny array)
    for (int i = 1; i < median_count; i++) {
        int32_t key = tmp[i];
        int j = i - 1;
        while (j >= 0 && tmp[j] > key) {
            tmp[j + 1] = tmp[j];
            j--;
        }
        tmp[j + 1] = key;
    }
    return tmp[median_count / 2];
}

// Median -> exponential moving average (low-pass)
static float Filter_Update(int32_t raw) {
    float med = (float)Median_Push(raw);
    if (!ema_ready) {
        ema_value = med;
        ema_ready = 1;
    } else {
        ema_value += EMA_ALPHA * (med - ema_value);
    }
    return ema_value;
}

static void Filter_Reset(void) {
    median_count = 0;
    median_idx   = 0;
    ema_ready    = 0;
}

// ---------------- Zero / force -----------------------------------------------
enum { TARE_OK = 0, TARE_SATURATED = 1, TARE_NO_HX711 = 2 };

// Average TARE_SAMPLES readings and use that as the new zero.
// Keep the load cell unloaded and still while this runs.
static int Tare(void) {
    float avg;
    int32_t mn, mx;
    int n = Average_Samples(TARE_SAMPLES, &avg, &mn, &mx);
    if (n <= 0) return TARE_NO_HX711;
    offset_counts = avg;
    Filter_Reset();
    if (mx >= HX711_SAT_HI || mn <= HX711_SAT_LO) return TARE_SATURATED;
    return TARE_OK;
}

static const char *Tare_Reason(int rc) {
    return (rc == TARE_SATURATED) ? "SATURATED" : "NO_HX711";
}

static float Force_Compute(float filtered_counts) {
    float grams = (filtered_counts - offset_counts) / scale_cpg;
    return grams * GRAM_TO_NEWTON;
}

// ---------------- Load cell selection ----------------------------------------
static void Apply_Scale(int idx) {
    scale_cpg = (cell_scale[idx] != 0.0f) ? cell_scale[idx] : 1.0f;
}

// ---------------- Calibration ------------------------------------------------
// The load cell must already be zeroed with "CAL ZERO" and the known mass placed.
static void Calibrate(float grams) {
    char sbuf[24];
    streaming = 0;      // the stream stays off afterwards; the app restarts it
    cal_hold  = 0;

    int total = CAL_SETTLE_SAMPLES + CAL_AVG_SAMPLES;
    int last_pct = -5, fail_streak = 0;
    int64_t sum = 0;
    int got = 0;
    int32_t lo = 0, hi = 0;

    for (int i = 0; i < total; i++) {
        int32_t v;
        if (!HX711_Read(&v)) {
            if (++fail_streak >= 3) { Tx_Line("ERR,CAL,NO_HX711"); return; }
        } else {
            fail_streak = 0;
            if (i >= CAL_SETTLE_SAMPLES) {          // discard the settling samples
                if (got == 0 || v < lo) lo = v;
                if (got == 0 || v > hi) hi = v;
                sum += v;
                got++;
            }
        }
        int pct = (i + 1) * 100 / total;
        if (pct >= last_pct + 5 && pct < 100) {
            Tx_Line("CAL,PROGRESS,%d", pct);
            last_pct = pct;
        }
    }

    if (got < CAL_AVG_SAMPLES / 2) { Tx_Line("ERR,CAL,NO_HX711"); return; }
    if (hi >= HX711_SAT_HI || lo <= HX711_SAT_LO) { Tx_Line("ERR,CAL,SATURATED"); return; }

    float avg   = (float)((double)sum / (double)got);
    float delta = avg - offset_counts;
    if (absf_(delta) < CAL_MIN_DELTA_COUNTS) { Tx_Line("ERR,CAL,NO_WEIGHT"); return; }

    float new_scale = delta / grams;
    cell_scale[active_cell] = new_scale;
    scale_cpg = new_scale;
    Filter_Reset();

    Fmt_Fixed(sbuf, sizeof(sbuf), new_scale, 4);
    Tx_Line("CAL,PROGRESS,100");
    Tx_Line("OK,CAL,RESULT,%d,%s,%ld", active_cell + 1, sbuf, (long)(hi - lo));
}

// ---------------- Command parser ---------------------------------------------
static int Parse_Float(const char *s, float *out) {
    char *end;
    float v = strtof(s, &end);
    if (end == s || *end != '\0' || v != v || v > 1e9f || v < -1e9f) return 0;
    *out = v;
    return 1;
}

static void Print_Help(void) {
    Tx_Line("# Load cell reader v%s. Commands (type, then Enter):", FW_VERSION);
    Tx_Line("#  PING                  check the connection");
    Tx_Line("#  STREAM 1|0            start / stop the F,<ms>,<N> readings");
    Tx_Line("#  INTERVAL <ms>         time between readings (100..60000)");
    Tx_Line("#  TARE                  zero (cell must be unloaded)");
    Tx_Line("#  AUTOZERO 1|0          slow zero tracking on / off");
    Tx_Line("#  CELLS                 list the load cells");
    Tx_Line("#  CELL <n>              select load cell n (and zero it)");
    Tx_Line("#  SCALE <n> <c/g>       set the factor of cell n until reset");
    Tx_Line("#  CAL ZERO              calibration step 1: zero, cell empty");
    Tx_Line("#  CAL <grams>           calibration step 2: known mass placed");
    Tx_Line("#  CAL CANCEL            abort a calibration");
    Tx_Line("#  STATUS                one-line status");
}

static void Process_Line(char *line) {
    char sbuf[24];
    char *argv[4];
    int argc = 0;

    for (char *p = line; *p; p++) *p = (char)toupper((unsigned char)*p);
    for (char *tok = strtok(line, " ,\t"); tok && argc < 4; tok = strtok(NULL, " ,\t")) {
        argv[argc++] = tok;
    }
    if (argc == 0) return;

    got_command = 1;
    const char *cmd = argv[0];

    if (!strcmp(cmd, "PING")) {
        Tx_Line("OK,PING,%s", FW_VERSION);

    } else if (!strcmp(cmd, "HELP") || !strcmp(cmd, "?")) {
        Print_Help();
        Tx_Line("OK,HELP");

    } else if (!strcmp(cmd, "STREAM")) {
        if (argc < 2 || (argv[1][0] != '0' && argv[1][0] != '1') || argv[1][1] != '\0') {
            Tx_Line("ERR,STREAM,ARG");
            return;
        }
        streaming = (argv[1][0] == '1');
        if (streaming) last_print_ms = HAL_GetTick() - interval_ms;   // first line now
        Tx_Line("OK,STREAM,%d", streaming);

    } else if (!strcmp(cmd, "INTERVAL")) {
        float v;
        if (argc < 2 || !Parse_Float(argv[1], &v)) { Tx_Line("ERR,INTERVAL,ARG"); return; }
        if (v < INTERVAL_MIN_MS || v > INTERVAL_MAX_MS) { Tx_Line("ERR,INTERVAL,RANGE"); return; }
        interval_ms = (uint32_t)v;
        Tx_Line("OK,INTERVAL,%lu", (unsigned long)interval_ms);

    } else if (!strcmp(cmd, "AUTOZERO")) {
        if (argc < 2 || (argv[1][0] != '0' && argv[1][0] != '1') || argv[1][1] != '\0') {
            Tx_Line("ERR,AUTOZERO,ARG");
            return;
        }
        autozero_on = (argv[1][0] == '1');
        Tx_Line("OK,AUTOZERO,%d", autozero_on);

    } else if (!strcmp(cmd, "TARE")) {
        int rc = Tare();
        if (rc == TARE_OK) Tx_Line("OK,TARE");
        else               Tx_Line("ERR,TARE,%s", Tare_Reason(rc));

    } else if (!strcmp(cmd, "CELLS")) {
        for (int i = 0; i < NUM_CELLS; i++) {
            Fmt_Fixed(sbuf, sizeof(sbuf), cell_scale[i], 4);
            Tx_Line("CELL,%d,%s,%s,%d", i + 1, load_cells[i].name, sbuf, i == active_cell);
        }
        Tx_Line("OK,CELLS,%d", NUM_CELLS);

    } else if (!strcmp(cmd, "CELL")) {
        float v;
        if (argc < 2 || !Parse_Float(argv[1], &v)) { Tx_Line("ERR,CELL,ARG"); return; }
        int n = (int)v;
        if (n < 1 || n > NUM_CELLS) { Tx_Line("ERR,CELL,RANGE"); return; }
        active_cell = n - 1;
        cal_hold = 0;
        Apply_Scale(active_cell);
        if (cell_scale[active_cell] == 0.0f) Tx_Line("W,UNCALIBRATED,%d", n);
        int rc = Tare();
        if (rc != TARE_OK) { Tx_Line("ERR,CELL,%s", Tare_Reason(rc)); return; }
        Fmt_Fixed(sbuf, sizeof(sbuf), cell_scale[active_cell], 4);
        Tx_Line("OK,CELL,%d,%s,%s", n, load_cells[active_cell].name, sbuf);

    } else if (!strcmp(cmd, "SCALE")) {
        float n_f, v;
        if (argc < 3 || !Parse_Float(argv[1], &n_f) || !Parse_Float(argv[2], &v)) {
            Tx_Line("ERR,SCALE,ARG");
            return;
        }
        int n = (int)n_f;
        if (n < 1 || n > NUM_CELLS) { Tx_Line("ERR,SCALE,RANGE"); return; }
        cell_scale[n - 1] = v;
        if (n - 1 == active_cell) { Apply_Scale(active_cell); Filter_Reset(); }
        Fmt_Fixed(sbuf, sizeof(sbuf), v, 4);
        Tx_Line("OK,SCALE,%d,%s", n, sbuf);

    } else if (!strcmp(cmd, "CAL")) {
        if (argc < 2) { Tx_Line("ERR,CAL,ARG"); return; }
        if (!strcmp(argv[1], "ZERO")) {
            int rc = Tare();
            if (rc != TARE_OK) { cal_hold = 0; Tx_Line("ERR,CAL,%s", Tare_Reason(rc)); return; }
            cal_hold = 1;
            Tx_Line("OK,CAL,ZERO");
        } else if (!strcmp(argv[1], "CANCEL")) {
            cal_hold = 0;
            Tx_Line("OK,CAL,CANCEL");
        } else {
            float grams;
            if (!Parse_Float(argv[1], &grams) || grams <= 0.0f) { Tx_Line("ERR,CAL,BAD_ARG"); return; }
            if (!cal_hold) { Tx_Line("ERR,CAL,ZERO_FIRST"); return; }
            Calibrate(grams);
        }

    } else if (!strcmp(cmd, "STATUS")) {
        Fmt_Fixed(sbuf, sizeof(sbuf), scale_cpg, 4);
        Tx_Line("STATUS,%d,%lu,%d,%s,%d,%ld,%ld,%d", streaming, (unsigned long)interval_ms,
                active_cell + 1, sbuf, autozero_on, (long)last_raw, (long)offset_counts, cal_hold);

    } else {
        Tx_Line("ERR,%s,UNKNOWN", cmd);
    }
}

// Collects received bytes into lines and runs each complete line
static void Poll_Serial(void) {
    uint8_t c;
    while (Rx_Get(&c)) {
        if (c == '\r' || c == '\n') {
            if (line_overflow) {
                line_overflow = 0;
                line_len = 0;
                Tx_Line("ERR,LINE,TOO_LONG");
            } else if (line_len > 0) {
                line_buf[line_len] = '\0';
                line_len = 0;
                Process_Line(line_buf);
            }
        } else if (line_len < LINE_MAX_LEN - 1) {
            line_buf[line_len++] = (char)c;
        } else {
            line_overflow = 1;
        }
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
  // Make sure SCK starts low
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  // Give the USB port 1 second to connect to the PC before sending anything
  HAL_Delay(1000);

  // Start with the default load cell and zero it (load cell must be empty at power-up)
  for (int i = 0; i < NUM_CELLS; i++) cell_scale[i] = load_cells[i].counts_per_gram;
  active_cell = DEFAULT_CELL;
  Apply_Scale(active_cell);
  Tare();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Until the first command arrives, remind whoever is listening (a terminal
    // opened after the board booted would otherwise miss a one-time message).
    static uint32_t last_hint = 0;
    if (!got_command && (HAL_GetTick() - last_hint) >= 2000) {
        last_hint = HAL_GetTick();
        Tx_Line("# Load cell reader v%s ready. Type HELP and press Enter.", FW_VERSION);
    }

    // Run any commands received from the PC
    Poll_Serial();

    // One HX711 sample (blocks until ready: ~10 Hz with the default RATE pin)
    int32_t raw;
    if (!HX711_Read(&raw)) {
        static uint32_t last_warn = 0;
        if ((HAL_GetTick() - last_warn) >= 2000) {
            last_warn = HAL_GetTick();
            Tx_Line("W,HX711_TIMEOUT");
        }
        continue;
    }
    last_raw = raw;

    float filtered = Filter_Update(raw);
    float force_N  = Force_Compute(filtered);

    // Zero tracking: if the reading is very close to zero, assume it's drift
    // and let the offset slowly follow it. Frozen during a calibration.
    if (autozero_on && !cal_hold && absf_(force_N) < AUTO_ZERO_WINDOW_N) {
        offset_counts += AUTO_ZERO_RATE * (filtered - offset_counts);
        force_N = Force_Compute(filtered);
    }

    // Deadband: report a clean zero when unloaded
    if (absf_(force_N) < ZERO_DEADBAND_N) {
        force_N = 0.0f;
    }

    // The +20 ms tolerance keeps a 100 ms interval from skipping every other sample
    if (streaming && (HAL_GetTick() - last_print_ms + 20u) >= interval_ms) {
        last_print_ms = HAL_GetTick();
        if (raw >= HX711_SAT_HI || raw <= HX711_SAT_LO) {
            Tx_Line("W,SATURATED");
        } else {
            char fbuf[24];
            Fmt_Fixed(fbuf, sizeof(fbuf), force_N, 3);
            Tx_Line("F,%lu,%s", (unsigned long)last_print_ms, fbuf);
        }
    }

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
