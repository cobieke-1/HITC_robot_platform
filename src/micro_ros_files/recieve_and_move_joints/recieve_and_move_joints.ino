#include <Wire.h>
#include <AS5600.h>


#include <micro_ros_arduino.h>

#include <stdio.h>
#include <stdint.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>


rcl_subscription_t subscriber_renamed;
rcl_publisher_t publisher_renamed;
std_msgs__msg__Float32MultiArray test_msg;
std_msgs__msg__Float32MultiArray msg;
rclc_executor_t executor_pub;
rclc_executor_t executor_sub;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
AS5600 as5600(&Wire);
#define SDA 21
#define SCL 22

#define LED_PIN 2

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


#include <AccelStepper.h>

#define stepPin 33
#define dirPin 25


AccelStepper stepper1(1,stepPin, dirPin);



//PID parameters - tuned by the user
float proportional = 1.35; //k_pp
float integral = 0.00005; //k_i 
float derivative = 0.01; //k_d 
float controlSignal = 0; //u 
//-----------------------------------
//-----------------------------------
//PID-related
float pT[2] = {0, 0}; // previousTime, for calculating delta t
float pE[2] = {0, 0}; //previousError for calculating the derivative (edot)
float eI[2] = {0,0}; // errorIntegral
float cT[2] = {0,0}; //currentTime, time in the moment of calculation
float dT[2] = {0,0}; //deltaTime, time difference
float eV[2] = {0,0}; //errorValue, error
float ed[2] = {0,0}; // edot, derivative (de/dt)

float globalAngles[2] = {0.0, 0.0};

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void control_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    float encoderAngles[2];
    move_motors(globalAngles, encoderAngles);
//    RCSOFTCHECK(rcl_publish(&publisher_renamed, &test_msg, NULL));
  }
}

// Subscriber Callback
void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  digitalWrite(LED_PIN, (msg->data.size > 0) ? LOW : HIGH); 
//   
  float angles[2] = {msg->data.data[0],msg->data.data[1]} ;
  memcpy(test_msg.data.data,angles,sizeof(angles));
  test_msg.data.size = 2;

  // update target for motors.
  globalAngles[0] = angles[0];
  globalAngles[1] = angles[1];
}

void move_motors(float angles[2], float (&currPosition)[2])
{
  float j1Target = angles[0];  // set up current target.
  
  float j1CurrPosition = as5600.readAngle()*0.08789; // current encoder reading
  
  float control1 = calculatePID(j1CurrPosition,j1Target,1); // PID algorithm j for joint
  
  driveMotor(stepper1, control1); //move motor using Accel stepper library
  
  //---------------------Code for second joint (via canbus)-------------------------//

  float j2Target = angles[1];  // set up current target.
  
  float j2CurrPosition = 0.0;
  
//  float control2 = calculatePID(j1CurrPosition,j1Target,1); // PID algorithm j for joint
//  
//  driveMotor(stepper2, control2); //move motor using Accel stepper library
  
  //-------------------------------------------------------------------------------//
  currPosition[0] = j1CurrPosition;
  currPosition[1] = j2CurrPosition;
}

float calculatePID(float currPosition, float Target, int jointNum)
{
  cT[jointNum-1] = micros();//current time
  dT[jointNum-1] = (cT[jointNum-1] - pT[jointNum-1]) / 1000000.0; //time difference in seconds
  pT[jointNum-1] = cT[jointNum-1]; //save the current time for the next iteration to get the time difference
  eV[jointNum-1] = currPosition - Target; //Current position - target position (or setpoint)
  ed[jointNum-1] = (eV[jointNum-1] - pE[jointNum-1]) / dT[jointNum-1]; //edot = de/dt - derivative term
  eI[jointNum-1] = eI[jointNum-1] + (eV[jointNum-1] * dT[jointNum-1]); //integral term - Newton-Leibniz, notice, this is a running sum!6
  controlSignal = (proportional * eV[jointNum-1]) + (derivative * ed[jointNum-1]) + (integral * eI[jointNum-1]); //final sum, proportional term also calculated here
  pE[jointNum-1] = eV[jointNum-1]; //save the error for the next iteration to get the difference (for edot)
 
  return controlSignal;
}

void driveMotor(AccelStepper stepper, float controlSignal)
{
  if(fabs(controlSignal) > 5) // currently set to within +/- 5 degrees per step
  {
    stepper.setSpeed(controlSignal*20);
  }
  else
  {
    stepper.setSpeed(0);
    stepper.stop();
  }

   stepper.runSpeed();
}


void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); 

  Wire.begin();           //start i2C
  Wire.setClock(800000L); // faster clock for the encoder (so that we can read from it faster).
  Wire.setPins(SDA, SCL);
   
  as5600.begin(); 
  as5600.setDirection(AS5600_CLOCK_WISE);
  Serial.print("Connect device 1: ");
  Serial.println(as5600.isConnected() ? "true" : "false");


  
  stepper1.setMaxSpeed(200); // stepper1.setMaxSpeed(200) // if the stepper driver is in full step mode this means the max is 200 steps per second.
  stepper1.setAcceleration(100); //100 steps per second square.
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber_renamed,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "student_trajectory"));  
  
  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher_renamed,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "encoder_readings"));

  // create timer, (control loop)
  const unsigned int timer_timeout = 10; // every 10 milliseconds
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    control_timer_callback));


  // Initialize float Array
  test_msg.data.capacity = 1;
  test_msg.data.size = 0;
  test_msg.data.data = (float*) malloc(test_msg.data.capacity * sizeof(float));

  test_msg.layout.dim.capacity = 100;
  test_msg.layout.dim.size = 0; // currently nothing in the array
  test_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension *) malloc(test_msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));

  for(int i = 0; i < test_msg.layout.dim.capacity; i++){
    test_msg.layout.dim.data[i].label.capacity = 20;
    test_msg.layout.dim.data[i].label.size = 0;
    test_msg.layout.dim.data[i].label.data = (char*) malloc(test_msg.layout.dim.data[i].label.capacity * sizeof(char));
  }



  // For the subscriber message we still need to initialize it before we hand it over to the executor. We don't need to assigna a value but it must be initialized.

  msg.data.capacity = 3; 
  msg.data.size = 0;
  msg.data.data = (float*) malloc(msg.data.capacity * sizeof(float));
  
  msg.layout.dim.capacity = 3;
  msg.layout.dim.size = 0;
  msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));
  
  for(size_t i = 0; i < msg.layout.dim.capacity; i++){
    msg.layout.dim.data[i].label.capacity = 3;
    msg.layout.dim.data[i].label.size = 0;
    msg.layout.dim.data[i].label.data = (char*) malloc(msg.layout.dim.data[i].label.capacity * sizeof(char));
  }

  // create executor
  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber_renamed, &msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));

}

void loop() {
  delay(10);
  RCSOFTCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(10)));
  RCSOFTCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(10)));
}
