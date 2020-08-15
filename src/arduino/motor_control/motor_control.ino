/* Motor Control

   Written for a Seeeduino XIAO.

   Encoders: 13 PPR motor / 663 PPR output shaft
   Wheels  : 13.5 cm dia. / 42.4 cm cir.
*/


#include <PID_saltyhash.h>


#define RIGHT_MOTOR_PWM_PIN   9
#define RIGHT_MOTOR_DIR_PIN   10
#define RIGHT_MOTOR_ENC_A_PIN 6
#define RIGHT_MOTOR_ENC_B_PIN 5

#define LEFT_MOTOR_PWM_PIN    7
#define LEFT_MOTOR_DIR_PIN    8
#define LEFT_MOTOR_ENC_A_PIN  4
#define LEFT_MOTOR_ENC_B_PIN  3

#define LEFT_BUMPER_PIN   0
#define MIDDLE_BUMPER_PIN 1
#define RIGHT_BUMPER_PIN  2

#define LEFT 2
#define RIGHT 1
#define MIN_MOTOR_PWM 1
//#define MAX_MOTOR_PWM 128
#define MAX_MOTOR_PWM 192


typedef signed long ticks_t;

union {
  float value;
  byte bytes[4];
} float_byte_array;

union {
  long value;
  byte bytes[4];
} long_byte_array;

union {
  short value;
  byte bytes[2];
} short_byte_array;


volatile ticks_t _right_motor_ticks = 0;
volatile ticks_t _left_motor_ticks = 0;

// These are automatically updated by ISRs any time the bumper state changes
volatile boolean   left_bumper = true;  // Assume true until updated
volatile boolean middle_bumper = true;
volatile boolean  right_bumper = true;


class Motor {
  private:
    int pwm_pin;
    int dir_pin;
    volatile ticks_t* ticks_ptr;

    long prev_position;
    unsigned long prev_time;

    // For PID controller
    PID *pid;
    double actual_velocity = 0.0;
    double target_velocity = 0.0;
    double pid_output = 0.0;

    static bool CloseToEqual(double value0, double value1) {
      return abs(value0 - value1) < 0.000001;
    }

    /* Args:
     *   speed: [-MAX_MOTOR_PWM, MAX_MOTOR_PWM]
     */
    void SetPwm(int speed) {
      // Limit the motor speed to [-MAX_MOTOR_PWM, MAX_MOTOR_PWM]
      speed = constrain(speed, -MAX_MOTOR_PWM, MAX_MOTOR_PWM);

      // Clamp the speed to [-255, 255]
      speed = constrain(speed, -255, 255);

      // Determine direction pin state
      analogWrite(pwm_pin, abs(speed));
      digitalWrite(dir_pin, speed < 0);
    }

    /* Sets actual_velocity [ticks / s]. */
    double UpdateActualVelocity() {
      // Get current state
      const long position = GetActualPosition();
      const unsigned long time = millis();
      const double dt = (time - prev_time) / 1000.0;

      // Calculate velocity [ticks / s]
      actual_velocity = (position - prev_position) / dt;

      // Save state
      prev_position = position;
      prev_time = time;

      return actual_velocity;
    }

  public:
    // [ms]
    int sample_time = 1000 / 50;

    // [ticks / s^2]
    unsigned short acceleration = 8000;

    bool debug = false;
    bool (*forward_check_func)(void) = nullptr;

    Motor(const int pwm_pin, const int dir_pin, volatile ticks_t* ticks_ptr) {
      this->pwm_pin = pwm_pin;
      this->dir_pin = dir_pin;
      this->ticks_ptr = ticks_ptr;

      pinMode(pwm_pin, OUTPUT);
      pinMode(dir_pin, OUTPUT);

      this->pid = new PID(&actual_velocity, &pid_output, &target_velocity, 0.05, 0.5, 0, DIRECT);
      pid->SetSampleTime(sample_time);
      pid->SetOutputLimits(0, 0);
      pid->SetMode(MANUAL);

      Stop();
    }

    /* [ticks] */
    long GetActualPosition() {
      return *ticks_ptr;
    }

    /* [ticks / s] */
    long GetActualVelocity() {
      return actual_velocity;
    }

    void SetPidTunings(const double p, const double i, const double d) {
      pid->SetTunings(p, i, d);
    }

    /* Args:
     *   velocity: [ticks / s]
     */
    void SetTargetVelocity(double velocity) {
      // Return if not trying to stop and it's not yet time to re-compute the output
      if (!pid->ReadyToCompute(millis())) {
        return;
      }

      // If we want to go forward but we can't, set velocity to 0
      if (velocity > 0 && forward_check_func != nullptr && !(*forward_check_func)()) {
        velocity = 0;
      }

      // Update the target and actual velocities
      if (acceleration == 0) {
        target_velocity = velocity;
      } else {
        const long dv = (double)acceleration * (double)sample_time / 1000.0;
        if (abs(velocity - target_velocity) > dv) {
          target_velocity += velocity > target_velocity ? dv : -dv;
        } else {
          target_velocity = velocity;
        }
      }
      UpdateActualVelocity();

      if (CloseToEqual(target_velocity, 0)) {
        if (pid->GetMode() != MANUAL) {
          pid->SetMode(MANUAL);
        }
        pid_output = 0.0;
      } else {
        if (pid->GetMode() != AUTOMATIC) {
          pid->SetMode(AUTOMATIC);
        }

        // We don't want the PID controller to tell the motor to go
        // the opposite direction of where it should be going
        if (target_velocity >= 0) {
          pid->SetOutputLimits(0, MAX_MOTOR_PWM);
        } else {
          pid->SetOutputLimits(-MAX_MOTOR_PWM, 0);
        }
      }

      pid->Compute();
      SetPwm(pid_output);

      if (debug) {
        Serial.println("target_velocity\tactual_velocity\toutput");
        Serial.print(target_velocity); Serial.print('\t');
        Serial.print(actual_velocity); Serial.print('\t');
        Serial.print(pid_output); Serial.print('\t');
        Serial.println();
      }
    }

    void Stop() {
      SetPwm(0);
    }
};


Motor  left_motor( LEFT_MOTOR_PWM_PIN,  LEFT_MOTOR_DIR_PIN,  &_left_motor_ticks);
Motor right_motor(RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_DIR_PIN, &_right_motor_ticks);


void setup() {
  Serial.begin(115200);

  setup_bumpers();
  setup_motors();
}


void setup_bumpers() {
  pinMode(  LEFT_BUMPER_PIN, INPUT_PULLUP);
  pinMode(MIDDLE_BUMPER_PIN, INPUT_PULLUP);
  pinMode( RIGHT_BUMPER_PIN, INPUT_PULLUP);

  left_bumper   = get_bumper(  LEFT_BUMPER_PIN);
  middle_bumper = get_bumper(MIDDLE_BUMPER_PIN);
  right_bumper  = get_bumper( RIGHT_BUMPER_PIN);

  attachInterrupt(
    digitalPinToInterrupt(LEFT_BUMPER_PIN),
    handle_left_bumper_change,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(MIDDLE_BUMPER_PIN),
    handle_middle_bumper_change,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(RIGHT_BUMPER_PIN),
    handle_right_bumper_change,
    CHANGE
  );
}


void setup_motors() {
  // Setup left motor
  pinMode(LEFT_MOTOR_ENC_A_PIN, INPUT);
  pinMode(LEFT_MOTOR_ENC_B_PIN, INPUT);
  left_motor.forward_check_func = can_move_left_forward;

  // Setup right motor
  pinMode(RIGHT_MOTOR_ENC_A_PIN, INPUT);
  pinMode(RIGHT_MOTOR_ENC_B_PIN, INPUT);
  right_motor.forward_check_func = can_move_right_forward;
  right_motor.debug = false;

  // Setup interrupts
  attachInterrupt(
    digitalPinToInterrupt(RIGHT_MOTOR_ENC_A_PIN),
    handle_right_motor_enc_a_change,
    CHANGE
  );
  attachInterrupt(
    digitalPinToInterrupt(RIGHT_MOTOR_ENC_B_PIN),
    handle_right_motor_enc_b_change,
    CHANGE
  );
  attachInterrupt(
    digitalPinToInterrupt(LEFT_MOTOR_ENC_A_PIN),
    handle_left_motor_enc_a_change,
    CHANGE
  );
  attachInterrupt(
    digitalPinToInterrupt(LEFT_MOTOR_ENC_B_PIN),
    handle_left_motor_enc_b_change,
    CHANGE
  );
}


void loop() {
  process_command();
}


void process_command() {
  const static byte ACK = 0xAA;
  const static byte NCK = 0xFF;
  const static byte SET_VELOCITY_COMMAND = 0xC0;
  const static byte SET_PID_TUNINGS_COMMAND = 0xC1;

  static short  left_motor_velocity = 0;
  static short right_motor_velocity = 0;
  static unsigned long last_command_time = 0;

  if ((millis() - last_command_time) > 2000) {
    left_motor_velocity = right_motor_velocity = 0;
  }

  left_motor.SetTargetVelocity(left_motor_velocity);
  right_motor.SetTargetVelocity(right_motor_velocity);

  if (!Serial.available()) {
    return;
  }

  const byte command = Serial.peek();

  /* "Set Velocity" command structure:
   * - Recv: 5 bytes total
   *   - command: byte = 0xC0
   *   - left_motor_velocity : signed short (MSB first, ticks/s)
   *   - right_motor_velocity: signed short (MSB first, ticks/s)
   * - Send: 14 bytes total
   *   - ACK: byte = 0xAA
   *   - actual_left_motor_position : signed long  (MSB first, ticks)
   *   - actual_left_motor_velocity : signed short (MSB first, ticks/s)
   *   - actual_right_motor_position: signed long  (MSB first, ticks)
   *   - actual_right_motor_velocity: signed short (MSB first, ticks/s)
   *   - bumpers: byte (bit 2: left bumper; bit 1: middle bumper; bit 0: right bumper)
   */
  if (command == SET_VELOCITY_COMMAND) {
    // Skip if there are not enough bytes available in the read/write buffers
    if (Serial.available() < 5 || Serial.availableForWrite() < 14) {
      return;
    }

    Serial.read();  // Discard the command

    // Read the target motor velocities
    left_motor_velocity  = Serial_ReadShortBytes();
    right_motor_velocity = Serial_ReadShortBytes();

    // Get actual motor and bumper states
    const long   actual_left_motor_position =  left_motor.GetActualPosition();
    const short  actual_left_motor_velocity =  left_motor.GetActualVelocity();
    const long  actual_right_motor_position = right_motor.GetActualPosition();
    const short actual_right_motor_velocity = right_motor.GetActualVelocity();
    const byte bumpers = left_bumper << 2 | middle_bumper << 1 | right_bumper;

    // Send the ACK and states
    Serial.write(ACK);
    Serial_WriteLongBytes( actual_left_motor_position);
    Serial_WriteShortBytes(actual_left_motor_velocity);
    Serial_WriteLongBytes( actual_right_motor_position);
    Serial_WriteShortBytes(actual_right_motor_velocity);
    Serial.write(bumpers);

    last_command_time = millis();
  }

  /* "Set PID Tunings" command structure:
   * - Recv: 13 bytes total
   *   - command: byte = 0xC1
   *   - p: float (MSB first)
   *   - i: float (MSB first)
   *   - d: float (MSB first)
   * - Send: 14 bytes total
   *   - ACK: byte = 0xAA
   */
  else if (command == SET_PID_TUNINGS_COMMAND) {
    if (Serial.available() < 13 || Serial.availableForWrite() < 1) {
      return;
    }

    // Discard the command
    Serial.read();

    // Get the new PID tunings
    const float new_p = Serial_ReadFloatBytes();
    const float new_i = Serial_ReadFloatBytes();
    const float new_d = Serial_ReadFloatBytes();

    // Set the new PID tunings
    left_motor.SetPidTunings(new_p, new_i, new_d);
    right_motor.SetPidTunings(new_p, new_i, new_d);

    // All is well
    Serial.write(ACK);
  }

  else {
    // Unknown command!
    Serial.read();
    Serial.write(NCK);
  }
}


float Serial_ReadFloatBytes() {
  float_byte_array.bytes[0] = Serial.read();
  float_byte_array.bytes[1] = Serial.read();
  float_byte_array.bytes[2] = Serial.read();
  float_byte_array.bytes[3] = Serial.read();

  return float_byte_array.value;
}


short Serial_ReadShortBytes() {
  short_byte_array.bytes[0] = Serial.read();
  short_byte_array.bytes[1] = Serial.read();

  return short_byte_array.value;
}


void Serial_WriteFloatBytes(const float value) {
  float_byte_array.value = value;

  Serial.write(float_byte_array.bytes[0]);
  Serial.write(float_byte_array.bytes[1]);
  Serial.write(float_byte_array.bytes[2]);
  Serial.write(float_byte_array.bytes[3]);
}


void Serial_WriteLongBytes(const long value) {
  long_byte_array.value = value;

  Serial.write(long_byte_array.bytes[0]);
  Serial.write(long_byte_array.bytes[1]);
  Serial.write(long_byte_array.bytes[2]);
  Serial.write(long_byte_array.bytes[3]);
}


void Serial_WriteShortBytes(const short value) {
  short_byte_array.value = value;

  Serial.write(short_byte_array.bytes[0]);
  Serial.write(short_byte_array.bytes[1]);
}


/* Returns true if the bumper is pressed. */
boolean get_bumper(const int bumper_pin) {
  return !digitalRead(bumper_pin);
}


/* - - - - - Interrupt Handlers: Begin - - - - - */


void handle_right_motor_enc_a_change() {
  _right_motor_ticks += digitalRead(RIGHT_MOTOR_ENC_A_PIN) == digitalRead(RIGHT_MOTOR_ENC_B_PIN) ? 1 : -1;
}


void handle_right_motor_enc_b_change() {
  _right_motor_ticks += digitalRead(RIGHT_MOTOR_ENC_A_PIN) == digitalRead(RIGHT_MOTOR_ENC_B_PIN) ? -1 : 1;
}


void handle_left_motor_enc_a_change() {
  _left_motor_ticks += digitalRead(LEFT_MOTOR_ENC_A_PIN) == digitalRead(LEFT_MOTOR_ENC_B_PIN) ? -1 : 1;
}


void handle_left_motor_enc_b_change() {
  _left_motor_ticks += digitalRead(LEFT_MOTOR_ENC_A_PIN) == digitalRead(LEFT_MOTOR_ENC_B_PIN) ? 1 : -1;
}


void handle_left_bumper_change() {
  left_bumper = get_bumper(LEFT_BUMPER_PIN);

  if (left_bumper) {
    left_motor.Stop();
  }
}


void handle_middle_bumper_change() {
  middle_bumper = get_bumper(MIDDLE_BUMPER_PIN);

  if (middle_bumper) {
    stop();
  }
}


void handle_right_bumper_change() {
  right_bumper = get_bumper(RIGHT_BUMPER_PIN);

  if (right_bumper) {
    right_motor.Stop();
  }
}


/* - - - - - Interrupt Handlers: End - - - - - */


void stop() {
  left_motor.Stop();
  right_motor.Stop();
}


bool can_move_left_forward() {
  return !left_bumper && !middle_bumper;
}


bool can_move_right_forward() {
  return !right_bumper && !middle_bumper;
}
