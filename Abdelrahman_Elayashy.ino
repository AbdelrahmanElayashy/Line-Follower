enum last_dir { LEFT, RIGHT, unknown };
last_dir last_dark = unknown;

const int NUM = 5;
boolean allow_change [NUM];

long long current_time_motor_problem;
long long current_time_obstacle;
long long current_time;

const int trigPin = 4;
const int echoPin = 8;

int count_motor_problem = 1;
int speed_motor = 120;
int p_constant = 11;
int speed_lr = 120;
int count = 1;

// obstacle variables
boolean obstacle_passed;
long duration;
int distance;

void setup()
{
  //Setup: Initalization
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(7, OUTPUT);

  //Then set the start value of the signals to zero:
  analogWrite(9, 0);
  analogWrite(10, 0);
  analogWrite(5, 0);
  analogWrite(6, 0);

  Serial.begin(9600);

  for (int i = 0; i < NUM; i++)
    allow_change[i] = true;

  obstacle_passed = true;
  current_time_obstacle = millis();
  digitalWrite(7, HIGH);

  //Startup delay:
  delay(2000);

}

void loop()
{

  linesensor();


}

void stop_motor() {
  digitalWrite(10, LOW); digitalWrite(9, LOW);
  digitalWrite(6, LOW); digitalWrite(5, LOW);
}

void backward_motor() {
  digitalWrite(9, LOW); analogWrite(10, speed_motor);
  digitalWrite(5, LOW); analogWrite(6, speed_motor);
}

void opposite_motor_r() {
  digitalWrite(10, LOW); analogWrite(9, 120);
  digitalWrite(5, LOW); analogWrite(6, 100);
  last_dark = RIGHT;
}

void opposite_motor_l() {
  digitalWrite(6, LOW); analogWrite(5, 100);
  digitalWrite(9, LOW); analogWrite(10, 120);
  last_dark = LEFT;
}

void forward_motor(int _speed) {
  digitalWrite(10, LOW); analogWrite(9, _speed);
  digitalWrite(6, LOW); analogWrite(5, _speed);
}

void l() {
  digitalWrite(10, LOW); analogWrite(9, speed_lr);
  digitalWrite(6, LOW); digitalWrite(5, LOW); //make a left turn
  last_dark = LEFT;
  delay(10);
}
void r() {
  digitalWrite(6, LOW); analogWrite(5, speed_lr);
  digitalWrite(9, LOW); digitalWrite(10, LOW); //make a right turn
  last_dark = RIGHT;
  delay(10);
}

void straight_line() {

  int error = 0;
  int left_speed = 0;
  int right_speed = 0;
  error = digitalRead(3) - digitalRead(2);
  left_speed = speed_motor + error * p_constant;
  right_speed = speed_motor - error * p_constant;
  analogWrite(9, left_speed);
  analogWrite(5, right_speed);
  delay(10);
}


boolean found_line() {

  int left = 0;
  int right = 0;
  left = analogRead(A6);
  right = analogRead(A7);

  if (right < 150 || left < 150) {
    obstacle_passed = true;
    return true;
  }
  return false;
}

void check_motor_problem() {

  if (digitalRead(3) == 0 && digitalRead(2) == 0) {
    solve_motor_problem();
    return;
  }
  count_motor_problem = 1;
}

void solve_motor_problem() {
  if (count_motor_problem % 2 == 1) {
    current_time_motor_problem = millis();
    count_motor_problem = 2;
  }
  else {

    if (current_time_motor_problem + 2000 >= millis())
      return;

    straight_line();
    delay(500);
    stop_motor();
    backward_motor();
    delay(500);
    stop_motor();
    count_motor_problem = 1;
  }
}

void parallel_search_line() {

  if (count % 2 == 1) {
	  // the robot losts his way.
    current_time = millis();
    update_search_line();
    count = 2;
  }
  else {
    if (found_line()) {
      update_search_line();
      return;
    }

    if (current_time + 500 >= millis())
      return;

    // left or right status dependes on what was the last turn.
    if (allow_change[0]) {
      stop_motor();
      delay(500);
      if (last_dark == LEFT) {
        opposite_motor_r();
      }
      else {
        opposite_motor_l();
      }
      allow_change[0] = false;
    }

    if (current_time + 2000 >= millis())
      return;

    // left or right status dependes on what was the last turn.
    if (allow_change[1]) {
      stop_motor();
      delay(500);
      if (last_dark == LEFT) {
        opposite_motor_r();
      }
      else {
        opposite_motor_l();
      }
      allow_change[1] = false;
    }

    if (current_time + 5000 >= millis())
      return;

    // center status.
    if (allow_change[2]) {
      stop_motor();
      delay(500);
      if (last_dark == LEFT) {
        opposite_motor_r();
      }
      else {
        opposite_motor_l();
      }
      allow_change[2] = false;
    }

    if (current_time + 7000 >= millis())
      return;

    // overcomeGap status.
    if (allow_change[3]) {
      task_overcome_gap();
    }

    if (current_time + 9000 >= millis())
      return;

      if (allow_change[4]) {
      stop_motor();
      delay(500);
      backward_motor();
      allow_change[4] = false;
    }

    if (current_time + 10500 <= millis())
      update_search_line();

  }
}

void task_overcome_gap() {
  stop_motor();
  delay(500);
  straight_line();
  allow_change[3] = false;
}

void update_search_line() {

  if (count % 2 == 0)
    count = 1;

  for (int i = 0; i < NUM; i++)
    allow_change[i] = true;
}

void linesensor() {
  int left = 0;
  int right = 0;

  left = analogRead(A6);
  right = analogRead(A7);

  if (left < 150) {
    r();
    update_search_line();
  }


  if (right < 150) {
    l();
    update_search_line();

  }

  if (right < 150 && left < 150) {
    straight_line();
    update_search_line();

  }

  if (right > 150 && left > 150) {
    parallel_search_line();
  }

  check_obstacle();
  check_motor_problem();
}



void check_obstacle() {

  if (current_time_obstacle + 1500 >= millis())
    return;

  current_time_obstacle = millis();
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);

  distance = duration * 0.034 / 2;

  Serial.print("Distance: ");
  Serial.println(distance);

  if (distance <= 15) {
    obstacle_passed = false;
    avoid_obstacle();
  }
}


void avoid_obstacle() {

  stop_motor();
  delay(2000);
  l();
  delay(500);
  stop_motor();
  delay(2000);
  straight_line();
  delay(1500);
  stop_motor();
  delay(2000);
  r();
  delay(1800);
  stop_motor();
  delay(2000);
  straight_line();
  delay(2000);
  stop_motor();
  delay(2000);
  opposite_motor_l();
  delay(1000);
  stop_motor();
  delay(2000);
  while (!obstacle_passed) {
    straight_line();
    found_line();
  }

}
