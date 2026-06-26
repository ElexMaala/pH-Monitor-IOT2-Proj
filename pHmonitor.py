#!/usr/bin/env python3
"""
pH Monitor -- Raspberry Pi 5
ST7735S TFT | ADS1115 | pH probe | 2x peristaltic pumps
Rotary encoder | Capacitive touch | Buzzer | RGB LED | Flask dashboard
"""

import ST7735
import time
import math
import threading
import board
import busio
import socket
import os
import logging
import json
from logging.handlers import RotatingFileHandler
import RPi.GPIO as GPIO
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from PIL import Image, ImageDraw, ImageFont
from collections import deque
from flask import Flask, jsonify, request, Response


# ══════════════════════════════════════════════════════════════════════════════
#  LOGGING
# ══════════════════════════════════════════════════════════════════════════════
_log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'phmonitor_v6.log')
_handler  = RotatingFileHandler(_log_path, maxBytes=1_000_000, backupCount=2)
_handler.setFormatter(logging.Formatter('%(asctime)s %(message)s', '%Y-%m-%d %H:%M:%S'))
log = logging.getLogger('phmonitor')
log.setLevel(logging.INFO)
log.addHandler(_handler)
log.addHandler(logging.StreamHandler())


# ══════════════════════════════════════════════════════════════════════════════
#  PIN MAP
# ══════════════════════════════════════════════════════════════════════════════
ENC_CLK    = 17
ENC_DT     = 18
ENC_SW     = 27

TOUCH_PIN  = 22
BUZZER_PIN = 23

LED_R_PIN  = 5
LED_G_PIN  = 6
LED_B_PIN  = 13

PUMP_ACID  = 19    # A_Motor
PUMP_BASE  = 26    # B_Motor


# ══════════════════════════════════════════════════════════════════════════════
#  pH PROBE CALIBRATION  (3-point piecewise linear)
# ══════════════════════════════════════════════════════════════════════════════
CAL_PH4_V  = 1.42436
CAL_PH7_V  = 1.77000
CAL_PH10_V = 2.08568

SLOPE_LOW      = (7.0 - 4.0)  / (CAL_PH7_V  - CAL_PH4_V)
INTERCEPT_LOW  = 4.0 - SLOPE_LOW  * CAL_PH4_V
SLOPE_HIGH     = (10.0 - 7.0) / (CAL_PH10_V - CAL_PH7_V)
INTERCEPT_HIGH = 7.0 - SLOPE_HIGH * CAL_PH7_V

def voltage_to_pH(v):
    pH = SLOPE_LOW * v + INTERCEPT_LOW if v <= CAL_PH7_V else SLOPE_HIGH * v + INTERCEPT_HIGH
    return max(0.0, min(14.0, round(pH, 2)))


# ══════════════════════════════════════════════════════════════════════════════
#  STATE-MACHINE CONFIG
# ══════════════════════════════════════════════════════════════════════════════
STABILIZE_SEC     = 60.0      # CALIBRATING duration
MC_INIT_SEC       = 0.5       # MC_INIT duration
MC_PULSE_SEC      = 3.0       # MC_A_ENTRY / MC_B_ENTRY pulse length
MC_MEASURE_SEC    = 60.0      # MC_A_STAY / MC_B_STAY settle window

TOUCH_LONG_SEC    = 3.0       # ≥ 3.0 s → motor-lock toggle
TOUCH_SHORT_SEC   = 0.1       # 0.1 … 3.0 s → rotary-lock toggle
LOCK_BEEP_SEC     = 0.08      # chirp on lock toggle
LOOP_PERIOD_SEC   = 0.001     # main loop tick

PH_DEADBAND       = 0.05      # pump stops within this band
PH_HYSTERESIS     = 0.20      # pump only fires when error exceeds this
K_STEP_COARSE     = 0.50
K_STEP_FINE       = 0.05
K_MIN, K_MAX      = 0.0, 14.0

# Dosing limits
DOSE_MIN_PULSE    = 0.20      # never run a pump for less than this (s)
DOSE_MAX_PULSE    = 10.0      # nor more than this in one shot (s)
DOSE_SETTLE_SEC   = 15.0      # wait after a pulse before re-evaluating
SENS_MIN          = 0.005     # sensitivity range (pH/s)
SENS_MAX          = 1.000
SENS_DEFAULT      = 0.05      # fallback if calibration fails

# Anti-overshoot: project pH forward and stop the pump early
LIVE_RATE_LOOKBACK_SEC = 2.0  # pump this long before trusting live rate
LIVE_RATE_WINDOW_SEC   = 2.0  # measure slope over the last 2 s
LAG_COMPENSATION_SEC   = 3.0  # how far ahead to project pH (tune if overshooting)
MIN_USEFUL_RATE        = 0.005   # ignore rates below this (probably noise)


# ══════════════════════════════════════════════════════════════════════════════
#  DISPLAY CONFIG
# ══════════════════════════════════════════════════════════════════════════════
WIDTH        = 160
HEIGHT       = 128
GRAPH_X      = 14
GRAPH_Y      = 22
GRAPH_WIDTH  = 136
GRAPH_HEIGHT = 68
GRAPH_ZOOM   = 1.5            # ±pH around target K for auto-zoom
WAVEFORM_LEN = 600            # 5 min of history at 0.5s per sample
HYSTERESIS   = PH_HYSTERESIS  # band drawn on graph — no dosing inside this range

BLACK     = (0,   0,   0)
WHITE     = (255, 255, 255)
GREEN     = (100, 255, 100)
RED       = (255, 100, 100)
BLUE      = (100, 100, 255)
YELLOW    = (255, 255,   0)
ORANGE    = (255, 165,   0)
GRAY      = (120, 120, 120)
CYAN      = (0,   220, 220)
DARK      = (20,  20,  40)
DIM_YELLOW = (40, 40,   0)


# ══════════════════════════════════════════════════════════════════════════════
#  STATES & OUTPUT TABLE  (RGB, B_Motor, A_Motor, TFT, Buzzer)
# ══════════════════════════════════════════════════════════════════════════════
S_CALIBRATING  = "CALIBRATING"
S_MC_INIT      = "MC_INIT"
S_MC_A_ENTRY   = "MC_A_ENTRY"
S_MC_A_STAY    = "MC_A_STAY"
S_MC_B_ENTRY   = "MC_B_ENTRY"
S_MC_B_STAY    = "MC_B_STAY"
S_SENSING      = "SENSING"
S_A_DRIVE      = "A_DRIVE"       # c > K + dead
S_B_DRIVE      = "B_DRIVE"       # c < K − dead

OUTPUTS = {
    S_CALIBRATING : (1, 0, 0, 1, 1),
    S_MC_INIT     : (1, 0, 0, 1, 1),
    S_MC_A_ENTRY  : (1, 0, 1, 1, 1),
    S_MC_A_STAY   : (1, 0, 0, 1, 0),    # A_motor OFF — settle & measure only
    S_MC_B_ENTRY  : (1, 1, 0, 1, 1),
    S_MC_B_STAY   : (1, 0, 0, 1, 0),    # B_motor OFF — settle & measure only
    S_SENSING     : (1, 0, 0, 1, 0),
    S_A_DRIVE     : (1, 0, 1, 1, 0),
    S_B_DRIVE     : (1, 1, 0, 1, 0),
}

SENSING_FAMILY = {S_SENSING, S_A_DRIVE, S_B_DRIVE}


# ══════════════════════════════════════════════════════════════════════════════
#  GPIO + HARDWARE
# ══════════════════════════════════════════════════════════════════════════════
_buzzer_pwm = None

def setup_gpio():
    global _buzzer_pwm
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(ENC_CLK,    GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(ENC_DT,     GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(ENC_SW,     GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(TOUCH_PIN,  GPIO.IN)
    GPIO.setup(BUZZER_PIN, GPIO.OUT, initial=GPIO.LOW)
    _buzzer_pwm = GPIO.PWM(BUZZER_PIN, 2000)
    for p in (LED_R_PIN, LED_G_PIN, LED_B_PIN, PUMP_ACID, PUMP_BASE):
        GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)


def set_rgb(on):
    """Turn all RGB channels on or off together."""
    v = GPIO.HIGH if on else GPIO.LOW
    GPIO.output(LED_R_PIN, v)
    GPIO.output(LED_G_PIN, v)
    GPIO.output(LED_B_PIN, v)


def beep(duration=0.08, frequency=2000):
    """Blocking beep — use chirp_async() if you don't want to block."""
    _buzzer_pwm.ChangeFrequency(frequency)
    _buzzer_pwm.start(50)
    time.sleep(duration)
    _buzzer_pwm.stop()
    GPIO.output(BUZZER_PIN, GPIO.LOW)


def chirp_async(duration=LOCK_BEEP_SEC, frequency=2000):
    threading.Thread(target=beep, args=(duration, frequency), daemon=True).start()


def setup_ads():
    i2c = busio.I2C(board.SCL, board.SDA)
    ads = ADS.ADS1115(i2c)
    ads.gain = 2 / 3
    return AnalogIn(ads, 0)


# ══════════════════════════════════════════════════════════════════════════════
#  SHARED SYSTEM STATE
# ══════════════════════════════════════════════════════════════════════════════
class System:
    """All shared state in one place. Access under self.lock from all threads."""
    def __init__(self, chan):
        self.lock           = threading.Lock()
        self.chan           = chan

        self.state          = S_CALIBRATING
        self.state_entered  = time.monotonic()

        self.K              = 7.00          # target pH
        self.current_pH     = 7.00
        self.fine_mode      = False         # encoder step: True → 0.05, False → 1.0
        self.motors_locked  = False         # long-touch lock
        self.rotary_locked  = False         # short-touch lock

        self.ph_history     = deque([7.0] * WAVEFORM_LEN, maxlen=WAVEFORM_LEN)
        self.dose_events    = deque(maxlen=20)
        self.pump_acid_on   = False
        self.pump_base_on   = False

        # motor calibration readings
        self.pre_acid_pH     = None
        self.acid_baseline   = None
        self.pre_base_pH     = None
        self.base_baseline   = None
        self.acid_sensitivity = SENS_DEFAULT   # pH/s from acid pump
        self.base_sensitivity = SENS_DEFAULT   # pH/s from base pump

        # dosing state
        self.current_pulse_sec = 0.0
        self.last_dose_end_time = 0.0

        # pH samples collected during a dose pulse
        self.pulse_samples = deque(maxlen=60)   # (time, pH) sampled at ~10 Hz
        self._last_pulse_sample_t = 0.0

    def elapsed(self):
        return time.monotonic() - self.state_entered

    # ---------- output application ----------
    def apply_outputs(self):
        """Apply GPIO outputs for the current state, respecting motor lock."""
        with self.lock:
            rgb, bm, am, _tft, _buz = OUTPUTS[self.state]
            if self.motors_locked:
                am, bm = 0, 0
            self.pump_acid_on = bool(am)
            self.pump_base_on = bool(bm)
        set_rgb(rgb)
        GPIO.output(PUMP_ACID, GPIO.HIGH if am else GPIO.LOW)
        GPIO.output(PUMP_BASE, GPIO.HIGH if bm else GPIO.LOW)

    def goto(self, new_state):
        prev = self.state
        with self.lock:
            self.state = new_state
            self.state_entered = time.monotonic()
            buz_on_entry = OUTPUTS[new_state][4] == 1
            # record dose for graph markers
            if new_state == S_A_DRIVE:
                self.dose_events.append((time.time(), "ACID"))
            elif new_state == S_B_DRIVE:
                self.dose_events.append((time.time(), "BASE"))
            # clear sample buffer at start of new pulse
            if new_state in (S_A_DRIVE, S_B_DRIVE):
                self.pulse_samples.clear()
                self._last_pulse_sample_t = 0.0
        self.apply_outputs()
        if buz_on_entry:
            chirp_async()
        log.info(f"[STATE] {prev} -> {new_state}")


# ══════════════════════════════════════════════════════════════════════════════
#  pH SAMPLING
# ══════════════════════════════════════════════════════════════════════════════
_voltage_lock = threading.Lock()
_voltage_samples = deque(maxlen=5)

def read_pH(chan):
    for attempt in range(3):
        try:
            v = chan.voltage
            with _voltage_lock:
                _voltage_samples.append(v)
                avg = sum(_voltage_samples) / len(_voltage_samples)
            return voltage_to_pH(avg)
        except OSError:
            if attempt < 2:
                time.sleep(0.01)
    with _voltage_lock:
        if _voltage_samples:
            return voltage_to_pH(sum(_voltage_samples) / len(_voltage_samples))
    return 7.0


def ph_thread(sysobj):
    while True:
        pH = read_pH(sysobj.chan)
        with sysobj.lock:
            sysobj.current_pH = pH
            sysobj.ph_history.append(pH)
        time.sleep(0.5)


# ══════════════════════════════════════════════════════════════════════════════
#  ROTARY ENCODER
# ══════════════════════════════════════════════════════════════════════════════
_ENCODER_TABLE = [
    [  0, -1, +1,  0],
    [ +1,  0,  0, -1],
    [ -1,  0,  0, +1],
    [  0, +1, -1,  0],
]

def encoder_thread(sysobj):
    prev_clk = GPIO.input(ENC_CLK)
    prev_dt  = GPIO.input(ENC_DT)
    accum    = 0
    while True:
        time.sleep(0.001)
        clk = GPIO.input(ENC_CLK)
        dt  = GPIO.input(ENC_DT)
        if clk == prev_clk and dt == prev_dt:
            continue
        direction = _ENCODER_TABLE[prev_clk * 2 + prev_dt][clk * 2 + dt]
        prev_clk, prev_dt = clk, dt
        if direction == 0:
            continue
        accum += direction
        if abs(accum) >= 4:        # 4 transitions = 1 detent
            with sysobj.lock:
                # only adjust target if in sensing and not locked
                if sysobj.state in SENSING_FAMILY and not sysobj.rotary_locked:
                    step = K_STEP_FINE if sysobj.fine_mode else K_STEP_COARSE
                    delta = step if accum > 0 else -step
                    sysobj.K = max(K_MIN, min(K_MAX, round(sysobj.K + delta, 2)))
            accum = 0


def encoder_sw_callback_factory(sysobj):
    def cb(_channel):
        time.sleep(0.05)
        if GPIO.input(ENC_SW) != GPIO.LOW:
            return
        with sysobj.lock:
            if sysobj.state in SENSING_FAMILY:
                sysobj.fine_mode = not sysobj.fine_mode
                mode_txt = "FINE" if sysobj.fine_mode else "COARSE"
            else:
                mode_txt = None
        if mode_txt:
            chirp_async()
            log.info(f"[ENC] Mode → {mode_txt}")
    return cb


# ══════════════════════════════════════════════════════════════════════════════
#  TOUCH SENSOR  (short press = rotary lock, long press = motor lock)
# ══════════════════════════════════════════════════════════════════════════════
_touch_press_time = 0.0
_touch_active     = False

def touch_callback_factory(sysobj):
    def cb(_channel):
        global _touch_press_time, _touch_active
        time.sleep(0.01)
        state = GPIO.input(TOUCH_PIN)
        if state == GPIO.HIGH and not _touch_active:
            _touch_press_time = time.monotonic()
            _touch_active = True
        elif state == GPIO.LOW and _touch_active:
            held = time.monotonic() - _touch_press_time
            _touch_active = False
            # ignore touch during calibration
            with sysobj.lock:
                if sysobj.state not in SENSING_FAMILY:
                    return
            if held >= TOUCH_LONG_SEC:
                with sysobj.lock:
                    sysobj.motors_locked = not sysobj.motors_locked
                    new_val = sysobj.motors_locked
                sysobj.apply_outputs()
                chirp_async(0.12, 1000)
                log.info(f"[TOUCH] motors_locked = {new_val}")
            elif held >= TOUCH_SHORT_SEC:
                with sysobj.lock:
                    sysobj.rotary_locked = not sysobj.rotary_locked
                    new_val = sysobj.rotary_locked
                chirp_async()
                log.info(f"[TOUCH] rotary_locked = {new_val}")
    return cb


# ══════════════════════════════════════════════════════════════════════════════
#  DISPLAY
# ══════════════════════════════════════════════════════════════════════════════
def ph_to_y(ph_value, g_min, g_max):
    """Convert a pH value to a Y pixel position on the graph."""
    normalized = max(0.0, min(1.0, (ph_value - g_min) / (g_max - g_min)))
    return max(GRAPH_Y, min(GRAPH_Y + GRAPH_HEIGHT,
               GRAPH_Y + GRAPH_HEIGHT - int(normalized * GRAPH_HEIGHT)))


def get_ph_color(ph_value, tgt):
    if abs(ph_value - tgt) <= HYSTERESIS:
        return GREEN
    return BLUE if ph_value < tgt else RED


def _draw_boot_screen(draw, font_large, font_small, font_med, sysobj, ip):
    """Screen shown during startup and motor calibration."""
    with sysobj.lock:
        st        = sysobj.state
        elapsed   = sysobj.elapsed()
        live_pH   = sysobj.current_pH
        acid_sens = sysobj.acid_sensitivity
        base_sens = sysobj.base_sensitivity

    draw.rectangle((0, 0, WIDTH, 22), fill=DARK)
    draw.text((5, 5), "pH Monitor", font=font_large, fill=WHITE)

    if st == S_CALIBRATING:
        title = "Stabilising sensor"
        remaining = max(0, STABILIZE_SEC - elapsed)
        sub = f"{remaining:5.1f} s remaining"
    elif st == S_MC_INIT:
        title = "Motor calibration"
        sub   = "Initialising..."
    elif st == S_MC_A_ENTRY:
        title = "Acid pump pulse"
        sub   = f"{max(0, MC_PULSE_SEC - elapsed):.1f} s"
    elif st == S_MC_A_STAY:
        title = "Settling — measuring acid"
        sub   = f"{max(0, MC_MEASURE_SEC - elapsed):.1f} s"
    elif st == S_MC_B_ENTRY:
        title = "Base pump pulse"
        sub   = f"{max(0, MC_PULSE_SEC - elapsed):.1f} s"
    elif st == S_MC_B_STAY:
        title = "Settling — measuring base"
        sub   = f"{max(0, MC_MEASURE_SEC - elapsed):.1f} s"
    else:
        title, sub = st, ""

    draw.text((5, 30), title, font=font_med,   fill=YELLOW)
    draw.text((5, 48), sub,   font=font_small, fill=CYAN)

    # current pH
    draw.text((5, 66), "Current pH:",            font=font_small, fill=WHITE)
    draw.text((75, 64), f"{live_pH:.2f}",        font=font_med,   fill=CYAN)

    # show sensitivity values once measured
    if st in (S_MC_B_ENTRY, S_MC_B_STAY) or st == S_MC_INIT:
        draw.text((5, 84), f"A sens: {acid_sens:.3f} pH/s", font=font_small, fill=WHITE)
    if st == S_MC_B_STAY and base_sens != SENS_DEFAULT:
        draw.text((5, 96), f"B sens: {base_sens:.3f} pH/s", font=font_small, fill=WHITE)

    draw.text((5, 112), f"IP: {ip}", font=font_small, fill=WHITE)


def _draw_sensing_screen(draw, font_small, font_medium, font_large, sysobj):
    """Main monitoring screen with waveform and controls."""
    with sysobj.lock:
        cur     = sysobj.current_pH
        tgt     = sysobj.K
        md      = "FINE" if sysobj.fine_mode else "COARSE"
        lk      = sysobj.rotary_locked
        mt      = sysobj.motors_locked
        hist    = list(sysobj.ph_history)
        a_on    = sysobj.pump_acid_on
        b_on    = sysobj.pump_base_on
        events  = list(sysobj.dose_events)

    # graph window follows the target
    g_min = max(0.0,  tgt - GRAPH_ZOOM)
    g_max = min(14.0, tgt + GRAPH_ZOOM)
    g_min = min(g_min, cur - 0.1)
    g_max = max(g_max, cur + 0.1)

    # Header
    draw.rectangle((0, 0, WIDTH, 20), fill=DARK)
    draw.text((5, 3), "pH Monitor", font=font_medium, fill=WHITE)

    # stability dot
    stab_win = hist[-20:] if len(hist) >= 20 else hist
    if len(stab_win) >= 5:
        avg_s = sum(stab_win) / len(stab_win)
        var_s = sum((x - avg_s) ** 2 for x in stab_win) / len(stab_win)
        dot_color = GREEN if var_s < 0.005 else (YELLOW if var_s < 0.02 else RED)
    else:
        dot_color = GRAY
    draw.ellipse((85, 5, 92, 14), fill=dot_color)

    # Mode badge
    if mt:        badge_color, badge_label = ORANGE, "NO PUMP"
    elif lk:      badge_color, badge_label = RED,    "LOCKED"
    elif md == "FINE": badge_color, badge_label = BLUE, "FINE"
    else:         badge_color, badge_label = GREEN,  "COARSE"
    draw.rectangle((95, 2, 158, 18), fill=badge_color)
    draw.text((98, 4), badge_label, font=font_small, fill=BLACK)

    # Graph border
    draw.rectangle(
        (GRAPH_X, GRAPH_Y, GRAPH_X + GRAPH_WIDTH, GRAPH_Y + GRAPH_HEIGHT),
        outline=GRAY, width=1,
    )

    # hysteresis band
    band_top    = ph_to_y(tgt + HYSTERESIS, g_min, g_max)
    band_bottom = ph_to_y(tgt - HYSTERESIS, g_min, g_max)
    draw.rectangle(
        (GRAPH_X + 1, band_top, GRAPH_X + GRAPH_WIDTH - 1, band_bottom),
        fill=DIM_YELLOW,
    )

    # grid lines
    ph_line = math.ceil(g_min)
    while ph_line <= math.floor(g_max):
        y     = ph_to_y(ph_line, g_min, g_max)
        color = (55, 55, 55) if ph_line == round(tgt) else (28, 28, 28)
        draw.line((GRAPH_X, y, GRAPH_X + GRAPH_WIDTH, y), fill=color, width=1)
        if ph_line != round(tgt):
            draw.text((GRAPH_X - 13, y - 5), str(ph_line), font=font_small, fill=WHITE)
        ph_line += 1

    # target line
    target_y = ph_to_y(tgt, g_min, g_max)
    draw.text((GRAPH_X - 13, target_y - 5), f"{tgt:.0f}", font=font_small, fill=YELLOW)

    # dose event markers
    now = time.time()
    for ev_time, ev_pump in events:
        age = now - ev_time
        if age > WAVEFORM_LEN * 0.5:
            continue
        readings_ago = int(age / 0.5)
        ev_x = GRAPH_X + GRAPH_WIDTH - readings_ago
        if GRAPH_X < ev_x < GRAPH_X + GRAPH_WIDTH:
            ev_color = (180, 60, 60) if ev_pump == "ACID" else (60, 60, 180)
            draw.line((ev_x, GRAPH_Y, ev_x, GRAPH_Y + GRAPH_HEIGHT), fill=ev_color, width=1)

    # ACID / BASE active labels
    draw.text((GRAPH_X + GRAPH_WIDTH - 68, GRAPH_Y + 2), "ACID",
              font=font_small, fill=RED  if a_on else WHITE)
    draw.text((GRAPH_X + GRAPH_WIDTH - 36, GRAPH_Y + 2), "BASE",
              font=font_small, fill=BLUE if b_on else WHITE)

    # Waveform
    if len(hist) >= 2:
        for i in range(len(hist) - 1):
            x1 = GRAPH_X + GRAPH_WIDTH - len(hist) + i
            x2 = x1 + 1
            if GRAPH_X <= x1 <= GRAPH_X + GRAPH_WIDTH:
                draw.line(
                    (x1, ph_to_y(hist[i],     g_min, g_max),
                     x2, ph_to_y(hist[i + 1], g_min, g_max)),
                    fill=CYAN, width=2,
                )

    # bottom info strip
    info_y = GRAPH_Y + GRAPH_HEIGHT + 2
    draw.text((GRAPH_X, info_y),
              "STEP: " + ("0.05" if md == "FINE" else "0.50"),
              font=font_small, fill=WHITE)

    ph_color = get_ph_color(cur, tgt)
    box_y    = info_y + 12
    draw.text((4, box_y), f"{cur:.2f}", font=font_large, fill=ph_color)

    # Trend arrow
    if len(hist) >= 10:
        recent = hist[-10:]
        slope  = (recent[-1] - recent[0]) / (len(recent) * 0.5)
        arrow = "↑" if slope > 0.005 else ("↓" if slope < -0.005 else "→")
    else:
        arrow = "?"

    delta   = cur - tgt
    d_sign  = "+" if delta >= 0 else ""
    d_color = GREEN if abs(delta) <= HYSTERESIS else (RED if delta > 0 else BLUE)
    draw.text((85, box_y),      f"→{tgt:.2f}",                font=font_medium, fill=YELLOW)
    draw.text((85, box_y + 13), f"{d_sign}{delta:.2f} {arrow}", font=font_small,  fill=d_color)


def display_thread(sysobj, disp, ip):
    try:
        font_small  = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 10)
        font_medium = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 11)
        font_large  = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 18)
    except Exception:
        font_small = font_medium = font_large = ImageFont.load_default()

    img  = Image.new('RGB', (WIDTH, HEIGHT), BLACK)
    draw = ImageDraw.Draw(img)

    while True:
        try:
            with sysobj.lock:
                st = sysobj.state
            draw.rectangle((0, 0, WIDTH, HEIGHT), fill=BLACK)
            if st in SENSING_FAMILY:
                _draw_sensing_screen(draw, font_small, font_medium, font_large, sysobj)
            else:
                _draw_boot_screen(draw, font_large, font_small, font_medium, sysobj, ip)
            disp.display(img)
        except Exception as e:
            log.warning(f"[DISP] Render error: {e}")
        time.sleep(0.1)


# ══════════════════════════════════════════════════════════════════════════════
#  STATE MACHINE
# ══════════════════════════════════════════════════════════════════════════════
def _plan_pulse(error_pH, sensitivity):
    """Calculate how long to run the pump based on pH error and sensitivity."""
    sens = sensitivity if SENS_MIN <= sensitivity <= SENS_MAX else SENS_DEFAULT
    raw  = abs(error_pH) / sens
    return max(DOSE_MIN_PULSE, min(DOSE_MAX_PULSE, raw))


def _live_rate(samples, window_sec):
    """Return pH change rate (pH/s) over the last window_sec. Returns None if not enough data."""
    if not samples:
        return None
    now    = samples[-1][0]
    cutoff = now - window_sec
    window = [(t, ph) for (t, ph) in samples if t >= cutoff]
    if len(window) < 3:
        return None
    t0, ph0 = window[0]
    t1, ph1 = window[-1]
    span = t1 - t0
    if span < 0.5:
        return None
    return (ph1 - ph0) / span


def step(sysobj):
    st = sysobj.state
    el = sysobj.elapsed()

    if st == S_CALIBRATING:
        if el >= STABILIZE_SEC:
            sysobj.goto(S_MC_INIT)

    elif st == S_MC_INIT:
        if el >= MC_INIT_SEC:
            # record pH before acid pulse
            with sysobj.lock:
                sysobj.pre_acid_pH = sysobj.current_pH
            log.info(f"[MC] pre_acid_pH = {sysobj.pre_acid_pH:.2f}")
            sysobj.goto(S_MC_A_ENTRY)

    elif st == S_MC_A_ENTRY:
        if el >= MC_PULSE_SEC:
            sysobj.goto(S_MC_A_STAY)

    elif st == S_MC_A_STAY:
        if el >= MC_MEASURE_SEC:
            with sysobj.lock:
                sysobj.acid_baseline = sysobj.current_pH
                # acid drops pH so pre > post
                raw_sens = (sysobj.pre_acid_pH - sysobj.acid_baseline) / MC_PULSE_SEC
                if SENS_MIN <= raw_sens <= SENS_MAX:
                    sysobj.acid_sensitivity = raw_sens
                else:
                    log.warning(f"[MC] Acid sensitivity {raw_sens:.4f} out of range; "
                                f"keeping default {SENS_DEFAULT}")
                # base starts from current pH
                sysobj.pre_base_pH = sysobj.current_pH
            log.info(f"[MC] acid_baseline = {sysobj.acid_baseline:.2f}  "
                     f"acid_sensitivity = {sysobj.acid_sensitivity:.4f} pH/s")
            sysobj.goto(S_MC_B_ENTRY)

    elif st == S_MC_B_ENTRY:
        if el >= MC_PULSE_SEC:
            sysobj.goto(S_MC_B_STAY)

    elif st == S_MC_B_STAY:
        if el >= MC_MEASURE_SEC:
            with sysobj.lock:
                sysobj.base_baseline = sysobj.current_pH
                # base raises pH so post > pre
                raw_sens = (sysobj.base_baseline - sysobj.pre_base_pH) / MC_PULSE_SEC
                if SENS_MIN <= raw_sens <= SENS_MAX:
                    sysobj.base_sensitivity = raw_sens
                else:
                    log.warning(f"[MC] Base sensitivity {raw_sens:.4f} out of range; "
                                f"keeping default {SENS_DEFAULT}")
            log.info(f"[MC] base_baseline = {sysobj.base_baseline:.2f}  "
                     f"base_sensitivity = {sysobj.base_sensitivity:.4f} pH/s")
            sysobj.goto(S_SENSING)

    elif st == S_SENSING:
        with sysobj.lock:
            c        = sysobj.current_pH
            K        = sysobj.K
            locked   = sysobj.motors_locked
            since    = time.monotonic() - sysobj.last_dose_end_time
            acid_s   = sysobj.acid_sensitivity
            base_s   = sysobj.base_sensitivity
        if locked or since < DOSE_SETTLE_SEC:
            return
        # only dose if error is outside the hysteresis band
        if c > K + PH_HYSTERESIS:
            pulse = _plan_pulse(c - K, acid_s)
            with sysobj.lock:
                sysobj.current_pulse_sec = pulse
            log.info(f"[DOSE] A_DRIVE  c={c:.2f}  K={K:.2f}  err={c-K:+.2f}  "
                     f"pulse={pulse:.2f}s  sens={acid_s:.4f}")
            sysobj.goto(S_A_DRIVE)
        elif c < K - PH_HYSTERESIS:
            pulse = _plan_pulse(K - c, base_s)
            with sysobj.lock:
                sysobj.current_pulse_sec = pulse
            log.info(f"[DOSE] B_DRIVE  c={c:.2f}  K={K:.2f}  err={c-K:+.2f}  "
                     f"pulse={pulse:.2f}s  sens={base_s:.4f}")
            sysobj.goto(S_B_DRIVE)

    elif st == S_A_DRIVE:
        now = time.monotonic()
        with sysobj.lock:
            c, K, locked, plan = (sysobj.current_pH, sysobj.K,
                                  sysobj.motors_locked, sysobj.current_pulse_sec)
            last_t = sysobj._last_pulse_sample_t
        # sample pH at ~10 Hz during dose
        if now - last_t > 0.1:
            sysobj.pulse_samples.append((now, c))
            sysobj._last_pulse_sample_t = now

        stop = False
        reason = ""
        projected_pH = None
        rate = None

        if locked:
            stop, reason = True, "locked"
        elif el >= plan:
            stop, reason = True, f"plan ({plan:.2f}s) exhausted"
        elif c <= K + PH_DEADBAND:
            stop, reason = True, f"reached deadband (c={c:.2f})"
        elif el >= LIVE_RATE_LOOKBACK_SEC:
            rate = _live_rate(sysobj.pulse_samples, LIVE_RATE_WINDOW_SEC)
            # acid drops pH, expect negative rate
            if rate is not None and rate < -MIN_USEFUL_RATE:
                projected_pH = c + rate * LAG_COMPENSATION_SEC
                if projected_pH <= K:
                    stop, reason = True, (f"projection: rate={rate:+.3f} pH/s, "
                                          f"projected={projected_pH:.2f} ≤ K={K:.2f}")
        if stop:
            with sysobj.lock:
                sysobj.last_dose_end_time = time.monotonic()
            log.info(f"[DOSE] A_DRIVE stop @ {el:.2f}s  c={c:.2f}  ({reason})")
            sysobj.goto(S_SENSING)

    elif st == S_B_DRIVE:
        now = time.monotonic()
        with sysobj.lock:
            c, K, locked, plan = (sysobj.current_pH, sysobj.K,
                                  sysobj.motors_locked, sysobj.current_pulse_sec)
            last_t = sysobj._last_pulse_sample_t
        if now - last_t > 0.1:
            sysobj.pulse_samples.append((now, c))
            sysobj._last_pulse_sample_t = now

        stop = False
        reason = ""
        projected_pH = None
        rate = None

        if locked:
            stop, reason = True, "locked"
        elif el >= plan:
            stop, reason = True, f"plan ({plan:.2f}s) exhausted"
        elif c >= K - PH_DEADBAND:
            stop, reason = True, f"reached deadband (c={c:.2f})"
        elif el >= LIVE_RATE_LOOKBACK_SEC:
            rate = _live_rate(sysobj.pulse_samples, LIVE_RATE_WINDOW_SEC)
            # base raises pH, expect positive rate
            if rate is not None and rate > MIN_USEFUL_RATE:
                projected_pH = c + rate * LAG_COMPENSATION_SEC
                if projected_pH >= K:
                    stop, reason = True, (f"projection: rate={rate:+.3f} pH/s, "
                                          f"projected={projected_pH:.2f} ≥ K={K:.2f}")
        if stop:
            with sysobj.lock:
                sysobj.last_dose_end_time = time.monotonic()
            log.info(f"[DOSE] B_DRIVE stop @ {el:.2f}s  c={c:.2f}  ({reason})")
            sysobj.goto(S_SENSING)


# ══════════════════════════════════════════════════════════════════════════════
#  WEB DASHBOARD  (Flask, port 5000)
# ══════════════════════════════════════════════════════════════════════════════
_flask_app    = Flask(__name__)
_dash_sysobj  = None   # set in main()

DASHBOARD_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>pH Monitor</title>
<script crossorigin src="https://unpkg.com/react@18/umd/react.production.min.js"></script>
<script crossorigin src="https://unpkg.com/react-dom@18/umd/react-dom.production.min.js"></script>
<script src="https://unpkg.com/@babel/standalone/babel.min.js"></script>
  <style>
    *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }
    body { background: #eeeef6; color: #18182e; font-family: 'Courier New', monospace; }
    input[type=range] {
      -webkit-appearance: none; appearance: none;
      height: 4px; border-radius: 2px; outline: none;
      background: #d0d0e4;
    }
    input[type=range]::-webkit-slider-thumb {
      -webkit-appearance: none; appearance: none;
      width: 16px; height: 16px; border-radius: 50%; cursor: pointer;
      background: #a87800; border: 2px solid #fff;
      box-shadow: 0 1px 4px rgba(0,0,0,.25);
    }
    input[type=range]:disabled { opacity: 0.35; }
    input[type=range]:disabled::-webkit-slider-thumb { cursor: not-allowed; }
    .ph-input {
      background: #f8f8ff; border: 1.5px solid #c8c8e0; border-radius: 5px;
      color: #18182e; font-family: 'Courier New', monospace;
      font-size: 15px; font-weight: bold; text-align: center;
      padding: 4px 6px; width: 72px; outline: none; transition: border-color .15s;
    }
    .ph-input:focus { border-color: #a87800; box-shadow: 0 0 0 3px rgba(168,120,0,.15); }
    .ph-input:disabled { opacity: 0.35; cursor: not-allowed; }
    button { transition: all .15s; }
    button:hover:not(:disabled) { filter: brightness(0.92); }
  </style>
</head>
<body>
  <div id="root"></div>
  <script type="text/babel">
    const { useState, useEffect, useRef } = React;

    const CAL_PH4_V   = 1.42436;
    const CAL_PH7_V   = 1.77000;
    const CAL_PH10_V  = 2.08568;
    const SLOPE_LOW      = (7.0 - 4.0)  / (CAL_PH7_V  - CAL_PH4_V);
    const INTERCEPT_LOW  = 4.0 - SLOPE_LOW  * CAL_PH4_V;
    const SLOPE_HIGH     = (10.0 - 7.0) / (CAL_PH10_V - CAL_PH7_V);
    const INTERCEPT_HIGH = 7.0 - SLOPE_HIGH * CAL_PH7_V;

    const PH_HYSTERESIS = 0.20;
    const PH_DEADBAND   = 0.05;
    const GRAPH_ZOOM    = 1.5;
    const WAVEFORM_LEN  = 600;

    const STAGE_TOTAL = {
      CALIBRATING: 60.0, MC_INIT: 0.5,
      MC_A_ENTRY: 3.0,   MC_A_STAY: 60.0,
      MC_B_ENTRY: 3.0,   MC_B_STAY: 60.0,
    };
    const STAGE_LABEL = {
      CALIBRATING: "Stabilising sensor",
      MC_INIT:     "Motor calibration",
      MC_A_ENTRY:  "Acid pump pulse",
      MC_A_STAY:   "Settling — measuring acid",
      MC_B_ENTRY:  "Base pump pulse",
      MC_B_STAY:   "Settling — measuring base",
    };

    const SENSING_FAMILY = new Set(["SENSING", "A_DRIVE", "B_DRIVE"]);

    const C = {
      bg:"#080812", dk:"#0e0e1e", br:"#1a1a2c",
      wh:"#dcdce8", gr:"#4a4a62",
      gn:"#3ecc3e", rd:"#e03c3c", bl:"#3c3ce0",
      yl:"#dede00", or:"#de9400", cy:"#00bcbc",
    };
    const U = {
      pageBg: "#eeeef6", panel: "#ffffff", border: "#d4d4e6",
      pri: "#18182e", sec: "#58587a", dim: "#9898b8",
      gn: "#1a9a1a", rd: "#be2020", bl: "#2020be",
      yl: "#a87800", or: "#b46400", cy: "#007080",
    };

    const FIELD_JS = {
      motors_locked: "motorsLocked",
      rotary_locked: "rotaryLocked",
      fine_mode:     "fineMode",
    };

    function renderCanvas(cvs, s) {
      if (!cvs || !s) return;
      const ctx = cvs.getContext("2d");
      const W = cvs.width, H = cvs.height;
      ctx.fillStyle = C.bg; ctx.fillRect(0, 0, W, H);
      SENSING_FAMILY.has(s.state) ? drawSensing(ctx, W, H, s) : drawBoot(ctx, W, H, s);
    }

    function drawBoot(ctx, W, H, s) {
      ctx.fillStyle = C.dk; ctx.fillRect(0, 0, W, 36);
      ctx.fillStyle = C.wh; ctx.font = "bold 18px 'Courier New',monospace";
      ctx.fillText("pH Monitor", 12, 24);
      const title   = STAGE_LABEL[s.state] || s.state;
      const total   = STAGE_TOTAL[s.state] || 1;
      const elapsed = s.elapsed || 0;
      const pct     = Math.min(1, elapsed / total);
      ctx.fillStyle = C.yl; ctx.font = "14px 'Courier New',monospace";
      ctx.fillText(title, 12, 64);
      ctx.fillStyle = "#181828"; ctx.fillRect(12, 74, W - 24, 10);
      ctx.fillStyle = C.cy;     ctx.fillRect(12, 74, (W - 24) * pct, 10);
      ctx.fillStyle = C.cy; ctx.font = "12px 'Courier New',monospace";
      ctx.fillText(Math.max(0, total - elapsed).toFixed(1) + " s remaining", 12, 100);
      ctx.fillStyle = C.wh; ctx.font = "12px 'Courier New',monospace";
      ctx.fillText("Current pH:", 12, 132);
      ctx.fillStyle = C.cy; ctx.font = "bold 22px 'Courier New',monospace";
      ctx.fillText((s.pH || 0).toFixed(2), 128, 132);
      if (["MC_INIT","MC_B_ENTRY","MC_B_STAY"].includes(s.state)) {
        ctx.fillStyle = C.wh; ctx.font = "12px 'Courier New',monospace";
        ctx.fillText("A sens: " + (s.acidSens||0).toFixed(4) + " pH/s", 12, 164);
      }
      if (s.state === "MC_B_STAY") {
        ctx.fillStyle = C.wh;
        ctx.fillText("B sens: " + (s.baseSens||0).toFixed(4) + " pH/s", 12, 186);
      }
    }

    function drawSensing(ctx, W, H, s) {
      const now  = Date.now() / 1000;
      const GX   = 44, GY = 36, GW = W - GX - 8, GH = H - GY - 68;
      const hist = s.phHistory || [];
      let gMin = Math.max(0.0,  s.K - GRAPH_ZOOM);
      let gMax = Math.min(14.0, s.K + GRAPH_ZOOM);
      gMin = Math.min(gMin, s.pH - 0.1);
      gMax = Math.max(gMax, s.pH + 0.1);
      const pY = ph =>
        GY + GH - Math.round(Math.max(0, Math.min(1, (ph - gMin) / (gMax - gMin))) * GH);
      ctx.fillStyle = C.dk; ctx.fillRect(0, 0, W, 30);
      ctx.fillStyle = C.wh; ctx.font = "bold 13px 'Courier New',monospace";
      ctx.fillText("pH Monitor", 8, 20);
      const win = hist.slice(-20);
      if (win.length >= 5) {
        const avg = win.reduce((a,b)=>a+b,0)/win.length;
        const vr  = win.reduce((a,v)=>a+(v-avg)**2,0)/win.length;
        ctx.fillStyle = vr<0.005?C.gn:vr<0.02?C.yl:C.rd;
        ctx.beginPath(); ctx.arc(W*.56,15,5,0,Math.PI*2); ctx.fill();
      }
      const [bCol,bLbl] = s.motorsLocked?[C.or,"NO PUMP"]
                        : s.rotaryLocked?[C.rd,"LOCKED"]
                        : s.fineMode    ?[C.bl,"FINE"]
                                        :[C.gn,"COARSE"];
      ctx.fillStyle = bCol; ctx.fillRect(W-90,3,84,23);
      ctx.fillStyle = C.bg; ctx.font = "bold 10px 'Courier New',monospace";
      ctx.textAlign = "center"; ctx.fillText(bLbl,W-48,18); ctx.textAlign = "left";
      ctx.strokeStyle = C.wh; ctx.lineWidth = 1; ctx.strokeRect(GX,GY,GW,GH);
      ctx.fillStyle = "rgba(220,220,0,0.055)";
      ctx.fillRect(GX+1, pY(s.K+PH_HYSTERESIS), GW-2,
                   pY(s.K-PH_HYSTERESIS)-pY(s.K+PH_HYSTERESIS));
      for (let ph=Math.ceil(gMin); ph<=Math.floor(gMax); ph++) {
        const y=pY(ph); if(y<GY||y>GY+GH) continue;
        ctx.strokeStyle = ph===Math.round(s.K)?"#30304a":"#181826";
        ctx.lineWidth=1; ctx.beginPath(); ctx.moveTo(GX,y); ctx.lineTo(GX+GW,y); ctx.stroke();
        ctx.fillStyle=C.wh; ctx.font="10px 'Courier New',monospace"; ctx.textAlign="right";
        if(ph!==Math.round(s.K)) ctx.fillText(ph,GX-4,y+4);
        ctx.textAlign="left";
      }
      ctx.strokeStyle=C.yl; ctx.lineWidth=1; ctx.setLineDash([5,4]);
      ctx.beginPath(); ctx.moveTo(GX,pY(s.K)); ctx.lineTo(GX+GW,pY(s.K)); ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle=C.yl; ctx.font="10px 'Courier New',monospace"; ctx.textAlign="right";
      ctx.fillText(s.K.toFixed(1),GX-4,pY(s.K)+4); ctx.textAlign="left";
      for (const ev of (s.doseEvents||[])) {
        const age=now-ev.t; if(age>WAVEFORM_LEN*0.5) continue;
        const ex=GX+GW-(age/0.5/(WAVEFORM_LEN-1))*GW;
        if(ex<=GX||ex>=GX+GW) continue;
        ctx.strokeStyle=ev.type==="ACID"?"rgba(210,55,55,0.75)":"rgba(55,55,210,0.75)";
        ctx.lineWidth=1.5; ctx.beginPath(); ctx.moveTo(ex,GY); ctx.lineTo(ex,GY+GH); ctx.stroke();
      }
      ctx.font="11px 'Courier New',monospace";
      ctx.fillStyle=s.acidOn?C.rd:"rgba(220,220,232,0.35)"; ctx.fillText("ACID",GX+5,GY+16);
      ctx.fillStyle=s.baseOn?C.bl:"rgba(220,220,232,0.35)"; ctx.fillText("BASE",GX+54,GY+16);
      const n=hist.length;
      if(n>=2){
        const xStep=GW/(WAVEFORM_LEN-1);
        ctx.strokeStyle=C.cy; ctx.lineWidth=2; ctx.beginPath();
        for(let i=0;i<n;i++){
          const x=GX+GW-(n-1-i)*xStep, y=pY(hist[i]);
          i===0?ctx.moveTo(x,y):ctx.lineTo(x,y);
        }
        ctx.stroke();
      }
      const iy=GY+GH+5, delta=s.pH-s.K;
      const phCol=Math.abs(delta)<=PH_HYSTERESIS?C.gn:delta>0?C.rd:C.bl;
      ctx.fillStyle=C.wh; ctx.font="11px 'Courier New',monospace";
      ctx.fillText("STEP: " + (s.fineMode?"0.05":"0.50"),GX,iy+14);
      ctx.fillStyle=phCol; ctx.font="bold 38px 'Courier New',monospace";
      ctx.fillText(s.pH.toFixed(2),GX-4,iy+58);
      let arrow="→";
      if(hist.length>=10){
        const rec=hist.slice(-10), slope=(rec[rec.length-1]-rec[0])/(rec.length*0.5);
        arrow=slope>0.005?"↑":slope<-0.005?"↓":"→";
      }
      ctx.fillStyle=C.yl; ctx.font="13px 'Courier New',monospace"; ctx.textAlign="right";
      ctx.fillText("→"+s.K.toFixed(2),W-8,iy+22);
      ctx.fillStyle=phCol; ctx.font="12px 'Courier New',monospace";
      ctx.fillText((delta>=0?"+":"")+delta.toFixed(2)+" "+arrow,W-8,iy+42);
      ctx.textAlign="left";
    }

    function openPDF(s) {
      const w=window.open("","_blank"), ts=new Date(), delta=s.pH-s.K;
      w.document.write(`<!DOCTYPE html><html><head>
<title>pH Monitor Report</title>
<style>
body{font-family:monospace;max-width:780px;margin:auto;padding:28px}
h1{border-bottom:3px solid #000;padding-bottom:8px}
h2{border-bottom:1px solid #ccc;margin-top:24px;padding-bottom:4px}
table{border-collapse:collapse;width:100%;margin:10px 0}
td,th{border:1px solid #ddd;padding:5px 12px}
th{background:#f2f2f2;text-align:left}
.acid{color:#c00}.base{color:#00c}.ok{color:#080}
@media print{button{display:none!important}}
</style></head><body>
<h1>pH Monitor — Session Report</h1>
<p><b>Timestamp:</b> ${ts.toLocaleString()}</p>
<p><b>System state:</b> ${s.state}</p>
<h2>Current Readings</h2>
<table>
<tr><th>Metric</th><th>Value</th></tr>
<tr><td>Current pH</td><td class="${s.pH>s.K+PH_HYSTERESIS?"acid":s.pH<s.K-PH_HYSTERESIS?"base":"ok"}">${s.pH.toFixed(2)}</td></tr>
<tr><td>Target pH (K)</td><td>${s.K.toFixed(2)}</td></tr>
<tr><td>Δ (current − target)</td><td>${delta>=0?"+":""}${delta.toFixed(2)}</td></tr>
<tr><td>Hysteresis band</td><td>±${PH_HYSTERESIS} pH</td></tr>
<tr><td>Deadband</td><td>±${PH_DEADBAND} pH</td></tr>
<tr><td>Motors locked</td><td>${s.motorsLocked}</td></tr>
<tr><td>Rotary locked</td><td>${s.rotaryLocked}</td></tr>
</table>
<h2>Motor Calibration</h2>
<table>
<tr><th>Motor</th><th>Sensitivity</th><th>Effect</th></tr>
<tr><td>A_Motor (acid)</td><td>${(s.acidSens||0).toFixed(4)} pH/s</td><td class="acid">Lowers pH</td></tr>
<tr><td>B_Motor (base)</td><td>${(s.baseSens||0).toFixed(4)} pH/s</td><td class="base">Raises pH</td></tr>
</table>
<h2>Dosing Events (${(s.doseEvents||[]).length} total)</h2>
<table>
<tr><th>#</th><th>Time</th><th>Motor</th><th>Age</th></tr>
${(s.doseEvents||[]).slice().reverse().slice(0,20).map((ev,i)=>
  `<tr><td>${i+1}</td><td>${new Date(ev.t*1000).toLocaleTimeString()}</td>
   <td class="${ev.type==="ACID"?"acid":"base"}">${ev.type==="ACID"?"A_DRIVE (acid)":"B_DRIVE (base)"}</td>
   <td>${(Date.now()/1000-ev.t).toFixed(0)} s ago</td></tr>`).join("")}
</table>
<h2>Recent pH History (last 30 readings)</h2>
<table>
<tr><th>#</th><th>pH</th></tr>
${(s.phHistory||[]).slice(-30).slice().reverse().map((ph,i)=>
  `<tr><td>${i+1}</td><td>${ph.toFixed(2)}</td></tr>`).join("")}
</table>
<button onclick="window.print()" style="margin-top:16px;padding:8px 20px;font-family:monospace;font-size:14px;cursor:pointer;">
  Print / Save as PDF
</button>
</body></html>`);
      w.document.close(); setTimeout(()=>w.print(),400);
    }

    function PHDashboard() {
      const [sys, setSys]        = useState(null);
      const [connected, setConn] = useState(false);
      const [kInput, setKInput]  = useState("7.00");
      const [editing, setEditing]= useState(false);
      const cvsRef               = useRef(null);
      const debRef               = useRef(null);

      useEffect(() => {
        let mounted = true;
        const poll = async () => {
          try {
            const r = await fetch('/api/state');
            if (!r.ok) throw new Error();
            const d = await r.json();
            if (mounted) { setSys(d); setConn(true); }
          } catch {
            if (mounted) setConn(false);
          }
        };
        poll();
        const id = setInterval(poll, 500);
        return () => { mounted = false; clearInterval(id); };
      }, []);

      useEffect(() => {
        if (!editing && sys) setKInput(sys.K.toFixed(2));
      }, [sys && sys.K, editing]);

      useEffect(() => { renderCanvas(cvsRef.current, sys); }, [sys]);

      const inS = sys && SENSING_FAMILY.has(sys.state);

      const sendTarget = (K) => {
        fetch('/api/target', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ K }),
        }).catch(() => {});
      };

      const handleSlider = (v) => {
        if (!inS || (sys && sys.rotaryLocked)) return;
        setSys(p => p ? { ...p, K: +v } : p);
        clearTimeout(debRef.current);
        debRef.current = setTimeout(() => sendTarget(+v), 200);
      };

      const handleKType = (e) => setKInput(e.target.value);

      const commitKInput = () => {
        setEditing(false);
        const val = parseFloat(kInput);
        if (isNaN(val)) { setKInput(sys && sys.K ? sys.K.toFixed(2) : "7.00"); return; }
        const clamped = Math.max(0, Math.min(14, Math.round(val * 100) / 100));
        setKInput(clamped.toFixed(2));
        if (inS && sys && !sys.rotaryLocked) {
          setSys(p => p ? { ...p, K: clamped } : p);
          sendTarget(clamped);
        }
      };

      const handleKKeyDown = (e) => {
        if (e.key === "Enter") { e.target.blur(); }
        if (e.key === "Escape") {
          setEditing(false);
          setKInput(sys && sys.K ? sys.K.toFixed(2) : "7.00");
        }
      };

      const toggle = async (pyField) => {
        if (!inS) return;
        const jsField = FIELD_JS[pyField];
        if (jsField) setSys(p => p ? { ...p, [jsField]: !p[jsField] } : p);
        try {
          await fetch('/api/toggle', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ field: pyField }),
          });
        } catch {}
      };

      const delta = sys ? sys.pH - sys.K : 0;
      const phCol = sys
        ? Math.abs(delta) <= PH_HYSTERESIS ? U.gn : delta > 0 ? U.rd : U.bl
        : U.sec;

      const btnStyle = (active, col, disabled) => ({
        background:   active ? col + "18" : "#f4f4fc",
        border:       `1.5px solid ${active ? col : U.border}`,
        color:        active ? col : U.sec,
        borderRadius: 5, padding: "9px 4px", fontSize: 10,
        fontFamily:   "'Courier New',monospace", width: "100%",
        cursor:       disabled ? "not-allowed" : "pointer",
        opacity:      disabled ? 0.4 : 1,
      });

      return (
        React.createElement('div', {style:{background:U.pageBg,minHeight:"100vh",display:"flex",
          flexDirection:"column",padding:"14px 16px",color:U.pri,boxSizing:"border-box"}},

          React.createElement('div', {style:{display:"flex",justifyContent:"space-between",
            alignItems:"center",marginBottom:12,fontSize:10}},
            React.createElement('span', {style:{color:connected?U.gn:U.rd,fontWeight:"bold"}},
              connected ? "● CONNECTED" : "● DISCONNECTED — waiting for Pi…"),
            sys && React.createElement('span', {style:{color:U.sec}},
              sys.state + " · pH " + (sys.pH&&sys.pH.toFixed(2)) + " · K " + (sys.K&&sys.K.toFixed(2)))
          ),

          React.createElement('div', {style:{display:"flex",gap:14,flex:1}},

            React.createElement('div', {style:{flex:1,minWidth:0}},
              React.createElement('canvas', {ref:cvsRef,width:720,height:420,
                style:{width:"100%",height:"auto",display:"block",borderRadius:6,
                       boxShadow:"0 2px 12px rgba(0,0,0,0.18)"}})
            ),

            React.createElement('div', {style:{width:228,flexShrink:0,background:U.panel,
              border:`1.5px solid ${U.border}`,borderRadius:8,padding:"16px 14px",
              display:"flex",flexDirection:"column",gap:16,
              boxShadow:"0 1px 6px rgba(0,0,0,0.08)"}},

              React.createElement('div', {style:{textAlign:"center",fontSize:10,color:U.dim,
                letterSpacing:".2em",fontWeight:"bold",
                borderBottom:`1px solid ${U.border}`,paddingBottom:12}}, "CONTROLS"),

              React.createElement('div', null,
                React.createElement('div', {style:{display:"flex",justifyContent:"space-between",
                  alignItems:"center",marginBottom:8}},
                  React.createElement('span', {style:{color:U.sec,fontSize:11}}, "Target pH:"),
                  React.createElement('input', {
                    className:"ph-input", type:"number", min:"0", max:"14", step:"0.01",
                    value:kInput, disabled:!inS||(sys&&sys.rotaryLocked),
                    onChange:handleKType, onFocus:()=>setEditing(true),
                    onBlur:commitKInput, onKeyDown:handleKKeyDown,
                  })
                ),
                React.createElement('input', {
                  type:"range", min:"0", max:"14",
                  step: sys && sys.fineMode ? "0.05" : "0.50",
                  value: sys ? sys.K : 7,
                  onChange: e => handleSlider(e.target.value),
                  disabled: !inS||(sys&&sys.rotaryLocked),
                  style:{width:"100%",cursor:(!inS||(sys&&sys.rotaryLocked))?"not-allowed":"pointer"},
                }),
                React.createElement('div', {style:{display:"flex",justifyContent:"space-between",
                  fontSize:9,color:U.dim,marginTop:4}},
                  React.createElement('span',null,"0"),
                  React.createElement('span',null,"7"),
                  React.createElement('span',null,"14")
                )
              ),

              React.createElement('div', {style:{display:"grid",gridTemplateColumns:"1fr 1fr",gap:8}},
                ...[
                  {on:sys&&sys.acidOn, col:U.rd, lbl:"Acid Motor"},
                  {on:sys&&sys.baseOn, col:U.bl, lbl:"Base Motor"},
                ].map(({on,col,lbl}) =>
                  React.createElement('div', {key:lbl, style:{
                    background:on?col+"12":"#f6f6fc",
                    border:`1.5px solid ${on?col:U.border}`,
                    borderRadius:6,padding:"10px 4px",textAlign:"center",
                  }},
                    React.createElement('div', {style:{width:10,height:10,borderRadius:"50%",
                      background:on?col:"#c8c8e0",margin:"0 auto 7px",
                      boxShadow:on?`0 0 8px ${col}88`:"none"}}),
                    React.createElement('div', {style:{fontSize:10,color:on?col:U.sec,fontWeight:"bold"}}, lbl),
                    React.createElement('div', {style:{fontSize:9,color:on?col:U.dim,marginTop:2}},
                      on?"DOSING":"IDLE")
                  )
                )
              ),

              React.createElement('div', {style:{display:"grid",gridTemplateColumns:"1fr 1fr",gap:8}},
                React.createElement('button',
                  {onClick:()=>toggle("rotary_locked"),disabled:!inS,style:btnStyle(sys&&sys.rotaryLocked,U.rd,!inS)},
                  sys&&sys.rotaryLocked?"🔒 LOCKED":"🔓 Lock/unlock"),
                React.createElement('button',
                  {onClick:()=>toggle("motors_locked"),disabled:!inS,style:btnStyle(sys&&sys.motorsLocked,U.or,!inS)},
                  sys&&sys.motorsLocked?"⏹ NO PUMP":"▶ Pump On/Off")
              ),

              React.createElement('button',
                {onClick:()=>toggle("fine_mode"),disabled:!inS,style:btnStyle(sys&&sys.fineMode,U.bl,!inS)},
                "MODE: "+(sys&&sys.fineMode?"FINE ±0.05":"COARSE ±0.50")),

              React.createElement('div', {style:{borderTop:`1px solid ${U.border}`,paddingTop:12}},
                React.createElement('button', {
                  onClick:()=>sys&&openPDF(sys), disabled:!sys,
                  style:{width:"100%",background:sys?"#eeeeff":"#f4f4fc",
                    border:`1.5px solid ${sys?"#8080cc":U.border}`,
                    color:sys?"#4040aa":U.dim,borderRadius:5,padding:11,
                    fontWeight:"bold",cursor:sys?"pointer":"not-allowed",
                    fontSize:12,fontFamily:"'Courier New',monospace",letterSpacing:".05em"}},
                  "PDF Report")
              ),

              React.createElement('div', {style:{borderTop:`1px solid ${U.border}`,paddingTop:12,
                fontSize:11,display:"flex",flexDirection:"column",gap:6}},
                ...[
                  {k:"pH",    v:sys&&sys.pH!=null?sys.pH.toFixed(2):"—",       col:phCol, bold:true},
                  {k:"Δ",     v:sys?(delta>=0?"+":"")+delta.toFixed(2):"—",    col:phCol},
                  {k:"state", v:sys?sys.state:"—",                              col:U.cy,  sm:true},
                  {k:"doses", v:sys&&sys.doseEvents?sys.doseEvents.length:"-",  col:U.pri},
                ].map(({k,v,col,bold,sm}) =>
                  React.createElement('div', {key:k,style:{display:"flex",justifyContent:"space-between",alignItems:"center"}},
                    React.createElement('span',{style:{color:U.sec}},k),
                    React.createElement('span',{style:{color:col,fontSize:sm?9:11,fontWeight:bold?"bold":"normal"}},v)
                  )
                )
              )
            )
          )
        )
      );
    }

    ReactDOM.createRoot(document.getElementById('root')).render(React.createElement(PHDashboard));
  </script>
</body>
</html>
"""


@_flask_app.route('/')
def dashboard_index():
    return Response(DASHBOARD_HTML, mimetype='text/html')


@_flask_app.route('/api/state')
def api_state():
    s = _dash_sysobj
    with s.lock:
        elapsed = s.elapsed()
        data = {
            'state':        s.state,
            'pH':           round(s.current_pH, 3),
            'K':            round(s.K, 2),
            'acidOn':       s.pump_acid_on,
            'baseOn':       s.pump_base_on,
            'motorsLocked': s.motors_locked,
            'rotaryLocked': s.rotary_locked,
            'fineMode':     s.fine_mode,
            'acidSens':     round(s.acid_sensitivity, 4),
            'baseSens':     round(s.base_sensitivity, 4),
            'phHistory':    [round(v, 3) for v in s.ph_history],
            'doseEvents':   [{'t': t, 'type': tp} for t, tp in s.dose_events],
            'elapsed':      round(elapsed, 2),
        }
    return jsonify(data)


@_flask_app.route('/api/target', methods=['POST'])
def api_set_target():
    body = request.get_json(force=True, silent=True) or {}
    K = body.get('K')
    if K is None:
        return jsonify({'error': 'missing K'}), 400
    try:
        K = float(K)
    except (TypeError, ValueError):
        return jsonify({'error': 'invalid K'}), 400
    K = max(0.0, min(14.0, round(K, 2)))
    with _dash_sysobj.lock:
        _dash_sysobj.K = K
    log.info(f"[WEB] Target pH set to {K}")
    return jsonify({'ok': True, 'K': K})


@_flask_app.route('/api/toggle', methods=['POST'])
def api_toggle():
    body  = request.get_json(force=True, silent=True) or {}
    field = body.get('field')
    if field not in ('motors_locked', 'rotary_locked', 'fine_mode'):
        return jsonify({'error': 'unknown field'}), 400
    with _dash_sysobj.lock:
        old = getattr(_dash_sysobj, field)
        setattr(_dash_sysobj, field, not old)
        new_val = not old
    if field == 'motors_locked':
        _dash_sysobj.apply_outputs()
    log.info(f"[WEB] {field} toggled → {new_val}")
    return jsonify({'ok': True, field: new_val})


def dashboard_thread():
    import logging as _lg
    _lg.getLogger('werkzeug').setLevel(_lg.ERROR)
    _flask_app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False, threaded=True)


# ══════════════════════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════════════════════
def get_local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "?.?.?.?"


def main():
    global _dash_sysobj
    log.info("=" * 50)
    log.info(" pH Monitor v6 — state-machine rewrite")
    log.info("=" * 50)

    setup_gpio()
    chan = setup_ads()
    sysobj = System(chan)
    _dash_sysobj = sysobj

    disp = ST7735.ST7735(
        port=0, cs=0, dc=24, rst=25,
        width=128, height=160,
        rotation=270,
        invert=False, bgr=False,
        offset_left=0, offset_top=0,
        spi_speed_hz=8_000_000,
    )
    disp.begin()

    ip = get_local_ip()
    log.info(f"[BOOT] IP: {ip}")

    GPIO.add_event_detect(ENC_SW,    GPIO.FALLING,
                          callback=encoder_sw_callback_factory(sysobj), bouncetime=200)
    GPIO.add_event_detect(TOUCH_PIN, GPIO.BOTH,
                          callback=touch_callback_factory(sysobj),       bouncetime=50)

    threading.Thread(target=encoder_thread, args=(sysobj,),       daemon=True).start()
    threading.Thread(target=ph_thread,      args=(sysobj,),       daemon=True).start()
    threading.Thread(target=display_thread, args=(sysobj, disp, ip), daemon=True).start()
    threading.Thread(target=dashboard_thread,                      daemon=True).start()
    log.info(f"[BOOT] Dashboard → http://{ip}:5000")

    # start calibration
    sysobj.goto(S_CALIBRATING)

    try:
        while True:
            step(sysobj)
            time.sleep(LOOP_PERIOD_SEC)
    except KeyboardInterrupt:
        log.info("Shutting down...")
    finally:
        GPIO.output(PUMP_ACID, GPIO.LOW)
        GPIO.output(PUMP_BASE, GPIO.LOW)
        set_rgb(False)
        try:
            _buzzer_pwm.stop()
        except Exception:
            pass
        GPIO.cleanup()
        log.info("Cleanup done.")


if __name__ == "__main__":
    main()
