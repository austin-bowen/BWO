/* Motor Control

   Written for a Seeeduino XIAO.

   Encoders: 13 PPR motor / 663 PPR output shaft
   Wheels  : 13.5 cm dia. / 42.4 cm cir.
*/


#include <PID_saltyhash.h>
#include <SerialPackets.h>
#include "wiring_analog_extras.h"


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
#define MAX_MOTOR_PWM 255


typedef signed long ticks_t;

union {
  float float_value;
  long long_value;
  short short_value;
  unsigned short unsigned_short_value;
  byte bytes[4];
} values_to_bytes;


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
     *   velocity: [-MAX_MOTOR_PWM, MAX_MOTOR_PWM]
     */
    void SetPwm(int velocity) {
      // Limit the motor velocity to [-MAX_MOTOR_PWM, MAX_MOTOR_PWM]
      velocity = constrain(velocity, -MAX_MOTOR_PWM, MAX_MOTOR_PWM);

      // Clamp the velocity to [-255, 255]
      velocity = constrain(velocity, -255, 255);

      // With a 20kHz PWM signal, the motors don't respond until the duty cycle is ~50%,
      // so scale the speed to be in the range of [128, 255], if the velocity is not 0.
      const unsigned int speed = velocity == 0 ? 0 : abs(velocity) / 2 + 128;

      // Determine direction pin state
      analogWrite20kHz(pwm_pin, speed);
      digitalWrite(dir_pin, velocity < 0);
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

    // If this is true, then the robot will resist movement when stopped.
    // if it is false, then the robot can be pushed around when stopped.
    bool brake = false;

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
      pid->SetOutputLimits(-MAX_MOTOR_PWM, MAX_MOTOR_PWM);
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

      if (CloseToEqual(target_velocity, 0) && !brake) {
        if (pid->GetMode() != MANUAL) {
          pid->SetMode(MANUAL);
        }
        pid_output = 0.0;
      } else {
        if (pid->GetMode() != AUTOMATIC) {
          pid->SetMode(AUTOMATIC);
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

SerialPackets serial_packets;
const uint8_t packet_buffer_len = 63 - SerialPackets::kPacketHeaderLen;
uint8_t packet_buffer[packet_buffer_len];


void setup() {
  Serial.begin(2000000);

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
  //debug();
  process_command();
}


void debug() {
  static short motor_velocity = 0;

  if (Serial.available()) {
    motor_velocity = Serial.parseInt();

    while (Serial.available()) {
      Serial.read();
    }
  }

  left_motor.debug = right_motor.debug = true;

  left_motor.SetTargetVelocity(motor_velocity);
  //right_motor.SetTargetVelocity(motor_velocity);
}


void process_command() {
  const static byte ACK = 0xAA;
  const static byte NCK = 0xFF;
  const static byte SET_VELOCITY_COMMAND = 0xC0;
  const static byte SET_PID_TUNINGS_COMMAND = 0xC1;
  const static byte SET_ACCELERATION_COMMAND = 0xC2;
  const static byte SET_BRAKE_COMMAND = 0xC3;

  static short  left_motor_velocity = 0;
  static short right_motor_velocity = 0;
  static unsigned long last_command_time = 0;

  if ((millis() - last_command_time) > 2000) {
    left_motor_velocity = right_motor_velocity = 0;
  }

  left_motor.SetTargetVelocity(left_motor_velocity);
  right_motor.SetTargetVelocity(right_motor_velocity);

  int packet_len = serial_packets.ReadNonblocking(packet_buffer, packet_buffer_len);
  if (packet_len < 1) {
    return;
  }

  const byte command = packet_buffer[0];

  /* "Set Velocity" command structure:
   * - Recv: 5 bytes total
   *   - command: byte = 0xC0
   *   - left_motor_velocity : signed short (ticks / s)
   *   - right_motor_velocity: signed short (ticks / s)
   * - Send: 14 bytes total
   *   - command: byte = 0xC0
   *   - actual_left_motor_position : signed long  (ticks)
   *   - actual_left_motor_velocity : signed short (ticks / s)
   *   - actual_right_motor_position: signed long  (ticks)
   *   - actual_right_motor_velocity: signed short (ticks / s)
   *   - bumpers: byte (bit 2: left bumper; bit 1: middle bumper; bit 0: right bumper)
   */
  if (command == SET_VELOCITY_COMMAND) {
    // Skip if the packet is not the right size
    if (packet_len != 5) {
      return;
    }

    // Read the target motor velocities
    left_motor_velocity = GetShortFromBuffer(packet_buffer, 1);
    right_motor_velocity = GetShortFromBuffer(packet_buffer, 3);

    // Get actual motor and bumper states
    const long   actual_left_motor_position =  left_motor.GetActualPosition();
    const short  actual_left_motor_velocity =  left_motor.GetActualVelocity();
    const long  actual_right_motor_position = right_motor.GetActualPosition();
    const short actual_right_motor_velocity = right_motor.GetActualVelocity();
    const byte bumpers = left_bumper << 2 | middle_bumper << 1 | right_bumper;

    // Send the states
    packet_len = 1;
    packet_len += CopyLongToBuffer(actual_left_motor_position, packet_buffer, packet_len);
    packet_len += CopyShortToBuffer(actual_left_motor_velocity, packet_buffer, packet_len);
    packet_len += CopyLongToBuffer(actual_right_motor_position, packet_buffer, packet_len);
    packet_len += CopyShortToBuffer(actual_right_motor_velocity, packet_buffer, packet_len);
    packet_buffer[packet_len++] = bumpers;
    serial_packets.Write(packet_buffer, packet_len);

    last_command_time = millis();
  }

  /* "Set PID Tunings" command structure:
   * - Recv: 13 bytes total
   *   - command: byte = 0xC1
   *   - p: float
   *   - i: float
   *   - d: float
   * - Send: 1 byte total
   *   - command: byte = 0xC1
   */
  else if (command == SET_PID_TUNINGS_COMMAND) {
    // Skip if the packet is not the right size
    if (packet_len != 13) {
      return;
    }

    // Get the new PID tunings
    const float new_p = GetFloatFromBuffer(packet_buffer, 1 + 0);
    const float new_i = GetFloatFromBuffer(packet_buffer, 1 + 4);
    const float new_d = GetFloatFromBuffer(packet_buffer, 1 + 8);

    // Set the new PID tunings
    left_motor.SetPidTunings(new_p, new_i, new_d);
    right_motor.SetPidTunings(new_p, new_i, new_d);

    // All is well
    packet_len = 1;
    serial_packets.Write(packet_buffer, packet_len);
  }

  /* "Set Acceleration" command structure:
   * - Recv: 3 bytes total
   *   - command: byte = 0xC2
   *   - acceleration: unsigned short (ticks / s^2)
   * - Send: 1 byte total
   *   - command: byte = 0xC2
   */
  else if (command == SET_ACCELERATION_COMMAND) {
    // Skip if the packet is not the right size
    if (packet_len != 3) {
      return;
    }

    // Read and set the new acceleration
    left_motor.acceleration = right_motor.acceleration = GetUnsignedShortFromBuffer(packet_buffer, 1);

    // All is well
    packet_len = 1;
    serial_packets.Write(packet_buffer, packet_len);
  }

  /* "Set Brake" command structure:
   * - Recv: 2 bytes total
   *   - command: byte = 0xC3
   *   - brake  : boolean
   * - Send: 1 byte total
   *   - command: byte = 0xC3
   */
  else if (command == SET_BRAKE_COMMAND) {
    // Skip if the packet is not the right size
    if (packet_len != 2) {
      return;
    }

    left_motor.brake = right_motor.brake = packet_buffer[1];

    // All is well
    packet_len = 1;
    serial_packets.Write(packet_buffer, packet_len);
  }

  else {
    // Unknown command! Send the inverse of the command.
    packet_buffer[0] = ~command;
    serial_packets.Write(packet_buffer, 1);
  }
}


size_t CopyValueBytesToBuffer(uint8_t buffer[], const size_t offset, const size_t length) {
  memcpy(buffer + offset, values_to_bytes.bytes, length);
  return length;
}


void CopyValueBytesFromBuffer(uint8_t buffer[], const size_t offset, const size_t length) {
  memcpy(values_to_bytes.bytes, buffer + offset, length);
}


size_t CopyLongToBuffer(const long value, uint8_t buffer[], size_t offset) {
  values_to_bytes.long_value = value;
  return CopyValueBytesToBuffer(buffer, offset, 4);
}


size_t CopyShortToBuffer(const short value, uint8_t buffer[], size_t offset) {
  values_to_bytes.short_value = value;
  return CopyValueBytesToBuffer(buffer, offset, 4);
}


float GetFloatFromBuffer(uint8_t buffer[], size_t offset) {
  CopyValueBytesFromBuffer(buffer, offset, 4);
  return values_to_bytes.float_value;
}


short GetShortFromBuffer(uint8_t buffer[], size_t offset) {
  CopyValueBytesFromBuffer(buffer, offset, 2);
  return values_to_bytes.short_value;
}


unsigned short GetUnsignedShortFromBuffer(uint8_t buffer[], size_t offset) {
  CopyValueBytesFromBuffer(buffer, offset, 2);
  return values_to_bytes.unsigned_short_value;
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
