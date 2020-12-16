#include "mazesolver.h"

void RunMaze();

// Button
const int buttonPin = 1;

// Echolocation 2-9 pins
const int echoTriggerPin = 2;
const int echoPin = 3;
const int echoNumberOfSensors = 4;

// Driver
const int driverRightPwm = A0;
const int driverRightIn1 = 10;
const int driverRightIn2 = 11;
const int driverLeftPwm = A1;
const int driverLeftIn1 = 12;
const int driverLeftIn2 = 13;

// Time of movement / rotation
const int movementDelay = 5000;
const int rotationDelay = 2000;

void setup()
{
  Serial.begin(9600);
  
  pinMode(buttonPin, INPUT);
  
  for (unsigned int i = 0; i < echoNumberOfSensors; i++)
  {
    pinMode(echoTriggerPin + i * 2, OUTPUT);
    pinMode(echoPin + i * 2, INPUT);
  }
  
  pinMode(driverRightPwm, OUTPUT);
  pinMode(driverRightIn1, OUTPUT);
  pinMode(driverRightIn2, OUTPUT);
  pinMode(driverLeftPwm, OUTPUT);
  pinMode(driverLeftIn1, OUTPUT);
  pinMode(driverLeftIn2, OUTPUT);
}

bool pressed = false;

void loop()
{
  int buttonState = digitalRead(buttonPin);
  if (buttonState == LOW)
  {
    if (pressed)
    {
      // Button released, start the algorithm
      RunMaze();
    }
    pressed = false;
  }
  else
  {
    // Button is being pressed
    pressed = true;
  }
}

enum class Rotation : unsigned char
{
  Right,
  Left
};

void Turn(Rotation rotation);
MazeSolver::Direction GetProperDirection(MazeSolver::Direction direction);

bool Scan(MazeSolver::Direction direction);
void Move(MazeSolver::Direction direction);

MazeSolver::Direction orientation;

void RunMaze()
{
  orientation = MazeSolver::Direction::Up;
  if (Scan(MazeSolver::Direction::Right))
    Turn(Rotation::Right);
  else
    Turn(Rotation::Left);
  MoveProper(MazeSolver::Direction::Up);
  
  MazeSolver::Solver maze(Scan, Move);
  if (maze.Solve(MazeSolver::Solution::PeriodicCorrection))
  {
    if (orientation == MazeSolver::Direction::Up)
      Turn(Rotation::Right);
    
    MoveProper(MazeSolver::Direction::Up);
  }
}

bool Scan(MazeSolver::Direction direction)
{
  MazeSolver::Direction properDirection = GetProperDirection(direction);
  unsigned int sensorOffset;
  switch (properDirection)
  {
    case MazeSolver::Direction::Up:
      sensorOffset = 0;
      break;
    case MazeSolver::Direction::Right:
      sensorOffset = 2;
      break;
    case MazeSolver::Direction::Down:
      sensorOffset = 4;
      break;
    case MazeSolver::Direction::Left:
      sensorOffset = 6;
      break;
  }
  
  digitalWrite(echoTriggerPin + sensorOffset, LOW);
  delayMicroseconds(2);
  digitalWrite(echoTriggerPin + sensorOffset, HIGH);
  delayMicroseconds(10);
  digitalWrite(echoTriggerPin + sensorOffset, LOW);
  long offset = pulseIn(echoPin + sensorOffset, HIGH);

  const float ratio = 27.6233; // Centimeters
  float distance = offset / 2.0 / ratio;

  if (distance < 0.5)
  {
    // Perhaps a sensor error?
    // Assume there's nothing in front of us
    return true;
  }

  const float distanceThreshold = 21; // 400mm (width of cell) / 2 + 10mm (room for error)
  return distance >= distanceThreshold;
}

void Move(MazeSolver::Direction direction)
{
  MazeSolver::Direction properDirection = GetProperDirection(direction);
  MoveProper(properDirection);
}

void MoveProper(MazeSolver::Direction direction)
{
  switch (direction)
  {
    case MazeSolver::Direction::Up:
    {
      digitalWrite(driverRightIn1, HIGH);
      digitalWrite(driverRightIn2, LOW);
      analogWrite(driverRightPwm, 255);

      digitalWrite(driverLeftIn1, HIGH);
      digitalWrite(driverLeftIn2, LOW);
      analogWrite(driverLeftPwm, 255);
      
      delay(movementDelay);
      
      digitalWrite(driverRightIn1, LOW);
      digitalWrite(driverRightIn2, LOW);

      digitalWrite(driverLeftIn1, LOW);
      digitalWrite(driverLeftIn2, LOW);
    }
    case MazeSolver::Direction::Right:
    {
      Turn(Rotation::Right);
      MoveProper(MazeSolver::Direction::Up);
    }
    case MazeSolver::Direction::Left:
    {
      Turn(Rotation::Left);
      MoveProper(MazeSolver::Direction::Up);
    }
    case MazeSolver::Direction::Down:
    {
      digitalWrite(driverRightIn1, LOW);
      digitalWrite(driverRightIn2, HIGH);
      analogWrite(driverRightPwm, 255);

      digitalWrite(driverLeftIn1, LOW);
      digitalWrite(driverLeftIn2, HIGH);
      analogWrite(driverLeftPwm, 255);
      
      delay(movementDelay);
      
      digitalWrite(driverRightIn1, LOW);
      digitalWrite(driverRightIn2, LOW);

      digitalWrite(driverLeftIn1, LOW);
      digitalWrite(driverLeftIn2, LOW);
    }
  }
}

void Turn(Rotation rotation)
{
  if (rotation == Rotation::Right)
  {
    digitalWrite(driverRightIn1, LOW);
    digitalWrite(driverRightIn2, HIGH);
    analogWrite(driverRightPwm, 255);

    digitalWrite(driverLeftIn1, HIGH);
    digitalWrite(driverLeftIn2, LOW);
    analogWrite(driverLeftPwm, 255);
    
    delay(rotationDelay);
    
    digitalWrite(driverRightIn1, LOW);
    digitalWrite(driverRightIn2, LOW);

    digitalWrite(driverLeftIn1, LOW);
    digitalWrite(driverLeftIn2, LOW);
    
    switch (orientation)
    {
      case MazeSolver::Direction::Up:
      {
        orientation = MazeSolver::Direction::Right;
        break;
      }
      case MazeSolver::Direction::Right:
      {
        orientation = MazeSolver::Direction::Down;
        break;
      }
      case MazeSolver::Direction::Left:
      {
        orientation = MazeSolver::Direction::Up;
        break;
      }
      case MazeSolver::Direction::Down:
      {
        orientation = MazeSolver::Direction::Left;
        break;
      }
    }
  }
  else
  {
    digitalWrite(driverRightIn1, HIGH);
    digitalWrite(driverRightIn2, LOW);
    analogWrite(driverRightPwm, 255);

    digitalWrite(driverLeftIn1, LOW);
    digitalWrite(driverLeftIn2, HIGH);
    analogWrite(driverLeftPwm, 255);
    
    delay(rotationDelay);
    
    digitalWrite(driverRightIn1, LOW);
    digitalWrite(driverRightIn2, LOW);

    digitalWrite(driverLeftIn1, LOW);
    digitalWrite(driverLeftIn2, LOW);
    
    switch (orientation)
    {
      case MazeSolver::Direction::Up:
      {
        orientation = MazeSolver::Direction::Left;
        break;
      }
      case MazeSolver::Direction::Right:
      {
        orientation = MazeSolver::Direction::Up;
        break;
      }
      case MazeSolver::Direction::Left:
      {
        orientation = MazeSolver::Direction::Down;
        break;
      }
      case MazeSolver::Direction::Down:
      {
        orientation = MazeSolver::Direction::Right;
        break;
      }
    }
  }
}

MazeSolver::Direction GetProperDirection(MazeSolver::Direction direction)
{
  switch (orientation)
  {
    case MazeSolver::Direction::Up:
    {
      return direction;
    }
    case MazeSolver::Direction::Right:
    {
      switch (direction)
      {
        case MazeSolver::Direction::Up:
          return MazeSolver::Direction::Left;
        case MazeSolver::Direction::Right:
          return MazeSolver::Direction::Up;
        case MazeSolver::Direction::Left:
          return MazeSolver::Direction::Down;
        case MazeSolver::Direction::Down:
          return MazeSolver::Direction::Right;
      }
    }
    case MazeSolver::Direction::Left:
    {
      switch (direction)
      {
        case MazeSolver::Direction::Up:
          return MazeSolver::Direction::Right;
        case MazeSolver::Direction::Right:
          return MazeSolver::Direction::Down;
        case MazeSolver::Direction::Left:
          return MazeSolver::Direction::Up;
        case MazeSolver::Direction::Down:
          return MazeSolver::Direction::Left;
      }
    }
    case MazeSolver::Direction::Down:
    {
      switch (direction)
      {
        case MazeSolver::Direction::Up:
          return MazeSolver::Direction::Down;
        case MazeSolver::Direction::Right:
          return MazeSolver::Direction::Left;
        case MazeSolver::Direction::Left:
          return MazeSolver::Direction::Right;
        case MazeSolver::Direction::Down:
          return MazeSolver::Direction::Up;
      }
    }
  }
  
  return MazeSolver::Direction::Invalid;
}
