#include <AccelStepper.h>
#include <Arduino.h>
#include <comm.h>
#include <lwip/sockets.h>

#define STEPPER_CORRECT_DIR(x) ((x)) // Normal (right)
// #define STEPPER_CORRECT_DIR(x) (-(x)) // Reversed (left)

#define STEPPER_PULSE_PER_REV 800 // pulse/rev
#define RAIL_MM_PER_REV 75 // mm/rev
#define RAIL_MAX_LENGTH_MM 1200
#define MM_TO_PULSE(mm) ((float)(mm) / RAIL_MM_PER_REV * STEPPER_PULSE_PER_REV)
#define STEPPER_MAX_PULSE MM_TO_PULSE(RAIL_MAX_LENGTH_MM)

// slower for better stability
// Upper limits. COMM_TYPE_PARAM can lower the speed and acceleration at run
// time (until the next reset), never raise them above these.
#define MAX_SPEED_MM 300 // mm/s
#define ACCELERATION_MM 100 // mm/s/s
#define HOME_SPEED_MM 100 // mm/s, capped by the current max speed

// Home sensor: slot photoelectric switch, NPN output on GPIO14 with the
// internal pull-up, LOW while the flag is in the slot. The stepper driver
// induces short LOW spikes on the line, so only a LOW that lasts
// HOME_SENSOR_DEBOUNCE_US counts as blocked (0.2 mm at the homing speed).
#define HOME_SENSOR_PIN 14
#define HOME_SENSOR_DEBOUNCE_US 2000
#define HOME_BACKOFF_PULSE 10
// Homing that starts with the flag already in the slot first climbs up to
// this far until the sensor clears, so the home is always the edge reached
// from above.
#define HOME_CLEAR_MAX_MM 30
// Above this the flag cannot be in the slot: a blocked sensor there means
// lost steps or a fault, never a reason to rewrite the position.
#define HOME_SENSOR_MAX_PULSE 50

// Homing runs only on COMM_TYPE_HOME, never at boot: the board does not move
// until the host asks. Until homed, COMM_TYPE_CTRL is ignored.
//
// FEEDBACK payload (4 x big-endian uint32):
//   [0] position, pulses from home (as before)
//   [1] status: bits 0-1 state (lift_state_t), bit 2 sensor LOW now (raw),
//       bit 3 sensor blocked (debounced), bits 4-7 fault (lift_fault_t),
//       bits 16-31 int16 position at the last home trigger, counted before
//       it was reset to 0 (re-homing from a known position: the drift)
//   [2] LOW spikes rejected by the debounce since boot
//   [3] longest rejected LOW spike, us
// Hosts that read only [0] (astra_controller lift_controller.py) still work.
//
// PARAM payload (big-endian uint32): [0] max speed mm/s, [1] acceleration
// mm/s/s; 0 keeps the current value, larger than the limit is clamped. The
// reply is a PARAM packet with the values in effect.
typedef enum { LIFT_UNHOMED = 0, LIFT_HOMING = 1, LIFT_HOMED = 2, LIFT_FAULT = 3 } lift_state_t;
typedef enum { FAULT_NONE = 0, FAULT_HOME_NOT_FOUND = 1, FAULT_SENSOR_AWAY_FROM_HOME = 2 } lift_fault_t;

// Define some steppers and the pins the will use
AccelStepper stepper(AccelStepper::DRIVER, 20, 19);

// The stepper is driven only from the timer callback. loop() hands commands
// over through these, under mux.
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static bool home_requested = false;
static bool target_pending = false;
static long target_pulse = 0;
static bool param_pending = false;
static uint32_t speed_mm = MAX_SPEED_MM; // in effect, under mux
static uint32_t accel_mm = ACCELERATION_MM;

static volatile lift_state_t state = LIFT_UNHOMED;
static volatile lift_fault_t fault = FAULT_NONE;
static volatile bool sensor_low = false;
static volatile bool sensor_blocked = false;
static volatile int16_t home_trigger_pulse = 0;
static volatile uint32_t spikes = 0;
static volatile uint32_t longest_spike_us = 0;

static int64_t low_since_us = -1; // -1: sensor not LOW
static bool homing_clearing = false; // climbing out of the slot first

uint32_t last_action_time;

#define TIMER_TIMEOUT_US 40

esp_timer_handle_t timer;

static void update_sensor(void) {
  int64_t now = esp_timer_get_time();
  if (digitalRead(HOME_SENSOR_PIN)) {
    if (low_since_us >= 0 && !sensor_blocked) {
      uint32_t len = (uint32_t)(now - low_since_us);
      spikes = spikes + 1;
      if (len > longest_spike_us) longest_spike_us = len;
    }
    low_since_us = -1;
    sensor_low = false;
    sensor_blocked = false;
    return;
  }
  if (low_since_us < 0) low_since_us = now;
  sensor_low = true;
  sensor_blocked = now - low_since_us >= HOME_SENSOR_DEBOUNCE_US;
}

static float max_speed_pulse = MM_TO_PULSE(MAX_SPEED_MM); // timer callback only

static float home_speed_pulse(void) {
  return min(max_speed_pulse, (float)MM_TO_PULSE(HOME_SPEED_MM));
}

static void stop_with_fault(lift_fault_t f) {
  stepper.setSpeed(0);
  stepper.moveTo(stepper.currentPosition());
  stepper.setMaxSpeed(max_speed_pulse);
  fault = f;
  state = LIFT_FAULT;
}

void timer_callback(void *arg) {
  update_sensor();

  bool home = false, move = false, param = false;
  long target = 0;
  uint32_t speed = 0, accel = 0;
  portENTER_CRITICAL_ISR(&mux);
  home = home_requested;
  home_requested = false;
  move = target_pending;
  target_pending = false;
  target = target_pulse;
  param = param_pending;
  param_pending = false;
  speed = speed_mm;
  accel = accel_mm;
  portEXIT_CRITICAL_ISR(&mux);

  if (param) {
    max_speed_pulse = MM_TO_PULSE(speed);
    stepper.setAcceleration(MM_TO_PULSE(accel));
    stepper.setMaxSpeed(state == LIFT_HOMING ? home_speed_pulse() : max_speed_pulse);
  }

  if (home) {
    fault = FAULT_NONE;
    state = LIFT_HOMING;
    stepper.setMaxSpeed(home_speed_pulse());
    homing_clearing = sensor_low;
    if (homing_clearing) {
      stepper.moveTo(stepper.currentPosition() + STEPPER_CORRECT_DIR((long)MM_TO_PULSE(HOME_CLEAR_MAX_MM)));
    } else {
      stepper.moveTo(stepper.currentPosition() + STEPPER_CORRECT_DIR(-(long)STEPPER_MAX_PULSE));
    }
  }

  switch (state) {
    case LIFT_HOMING:
      if (homing_clearing) {
        if (low_since_us < 0) { // cleared: now approach the edge from above
          homing_clearing = false;
          stepper.moveTo(stepper.currentPosition() + STEPPER_CORRECT_DIR(-(long)STEPPER_MAX_PULSE));
        } else if (stepper.distanceToGo() == 0) {
          stop_with_fault(FAULT_HOME_NOT_FOUND);
        }
      } else if (sensor_blocked) {
        home_trigger_pulse = (int16_t)constrain(STEPPER_CORRECT_DIR(stepper.currentPosition()), -32768L, 32767L);
        stepper.setCurrentPosition(0); // also stops: the speed is set to 0
        stepper.setMaxSpeed(max_speed_pulse);
        stepper.moveTo(STEPPER_CORRECT_DIR(HOME_BACKOFF_PULSE));
        state = LIFT_HOMED;
      } else if (stepper.distanceToGo() == 0) {
        stop_with_fault(FAULT_HOME_NOT_FOUND);
      }
      break;
    case LIFT_HOMED:
      if (sensor_blocked && STEPPER_CORRECT_DIR(stepper.currentPosition()) > HOME_SENSOR_MAX_PULSE) {
        stop_with_fault(FAULT_SENSOR_AWAY_FROM_HOME);
      } else if (move) {
        stepper.moveTo(STEPPER_CORRECT_DIR(target));
      }
      break;
    default: // unhomed or fault: hold, ignore moves
      break;
  }

  for (int i = 0; i < 10; ++i) stepper.run();

  ESP_ERROR_CHECK(esp_timer_start_once(timer, TIMER_TIMEOUT_US));
}

static void send_feedback(void) {
  uint32_t status = (uint32_t)state | (sensor_low ? 1u << 2 : 0) | (sensor_blocked ? 1u << 3 : 0) |
                    ((uint32_t)fault << 4) | ((uint32_t)(uint16_t)home_trigger_pulse << 16);
  uint32_t pos[4] = {
    (uint32_t)STEPPER_CORRECT_DIR(stepper.currentPosition()), status, spikes, longest_spike_us
  };
  for (int i = 0; i < 4; ++i) pos[i] = htonl(pos[i]);
  comm_send_blocking(COMM_TYPE_FEEDBACK, (uint8_t *)pos);
}

void setup() {
  comm_init();

  pinMode(HOME_SENSOR_PIN, INPUT_PULLUP);

  stepper.setMinPulseWidth(2); // 防止丢步 驱动器要求最小 2us 持续时间
  stepper.setMaxSpeed(MM_TO_PULSE(MAX_SPEED_MM)); // 空载最大 750rpm
  stepper.setAcceleration(MM_TO_PULSE(ACCELERATION_MM));

  // https://github.com/espressif/esp-idf/blob/v5.2.2/examples/system/esp_timer/main/esp_timer_example_main.c
  const esp_timer_create_args_t timer_args = {
    .callback = &timer_callback
  };
  ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
  ESP_ERROR_CHECK(esp_timer_start_once(timer, TIMER_TIMEOUT_US));

  last_action_time = millis();
}

void loop() {
  comm_type_t type;
  uint8_t buf[20];
  bool ret = comm_recv_poll_last(&type, buf);

  if (!ret) {
    if (type == COMM_TYPE_PING) {
      comm_send_blocking(COMM_TYPE_PONG, buf);
    } else if (type == COMM_TYPE_PONG) {
      ;
    } else if (type == COMM_TYPE_CTRL) {
      last_action_time = millis();

      uint32_t pos;
      memcpy(&pos, buf, sizeof pos);
      pos = ntohl(pos);

      if (pos > STEPPER_MAX_PULSE) {
        Serial.println("out of limit");
      } else if (state == LIFT_HOMED) {
        portENTER_CRITICAL(&mux);
        target_pulse = (long)pos;
        target_pending = true;
        portEXIT_CRITICAL(&mux);
      }
      send_feedback();
    } else if (type == COMM_TYPE_HOME) {
      last_action_time = millis();
      portENTER_CRITICAL(&mux);
      home_requested = true;
      target_pending = false;
      portEXIT_CRITICAL(&mux);
      send_feedback();
    } else if (type == COMM_TYPE_PARAM) {
      last_action_time = millis();
      uint32_t v[4];
      memcpy(v, buf, sizeof v);
      uint32_t speed = ntohl(v[0]), accel = ntohl(v[1]);
      portENTER_CRITICAL(&mux);
      if (speed) speed_mm = min(speed, (uint32_t)MAX_SPEED_MM);
      if (accel) accel_mm = min(accel, (uint32_t)ACCELERATION_MM);
      param_pending = true;
      v[0] = htonl(speed_mm);
      v[1] = htonl(accel_mm);
      portEXIT_CRITICAL(&mux);
      v[2] = v[3] = 0;
      comm_send_blocking(COMM_TYPE_PARAM, (uint8_t *)v);
    } else {
      Serial.println("Wrong Cmd");
    }
  }
  uint32_t this_action_time = millis();
  if (this_action_time - last_action_time > 10) { // 每隔 10ms执行一次
    last_action_time = this_action_time;
    send_feedback();
  }

  delay(1);
}
