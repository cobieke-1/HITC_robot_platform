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

#define LED_PIN 2

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


#include <AccelStepper.h>

#define stepPin 33
#define dirPin 25


AccelStepper stepper1(1,stepPin, dirPin);

float globalAngles[2] = {0.0, 0.0};

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
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

  globalAngles[0] = angles[0];
  globalAngles[1] = angles[1];
// assign value t float array msg 

// move motor to received positions.

  move_motors(globalAngles);
  RCSOFTCHECK(rcl_publish(&publisher_renamed, &test_msg, NULL));
}

void move_motors(angles)
{
  // set up current target.
  j1Target = angles[0];
  j2Target = angles[1];

  // get encoder reading for both joints
  j1CurrPosition = as5600.readAngle()*0.08789;
  j2CurrPosition = as5600.readAngle()*0.08789;

  // PID algorithm j1
  float control1 = calculatePID(j1CurrPosition,j2Target)
  // PID algorithm j2
  float control2 = calculatePID(j2CurrPosition, j2Target)
  
  driveMotor(stepper1, control1); //changes to use Accel stepper library
  driveMotor(stepper2, control2);
}

float calculatePID(float currPosition, float Target)
{
  currentTime = micros();//current time
  deltaTime = (currentTime - previousTime) / 1000000.0; //time difference in seconds
  previousTime = currentTime; //save the current time for the next iteration to get the time difference
  errorValue = currPosition - Target; //Current position - target position (or setpoint)
  edot = (errorValue - previousError) / deltaTime; //edot = de/dt - derivative term
  errorIntegral = errorIntegral + (errorValue * deltaTime); //integral term - Newton-Leibniz, notice, this is a running sum!6
  controlSignal = (proportional * errorValue) + (derivative * edot) + (integral * errorIntegral); //final sum, proportional term also calculated here
  previousError = errorValue; //save the error for the next iteration to get the difference (for edot)

  return controlSignal;
}

void driveMotor(AccelStepper stepper, float controlSignal)
{
  if(fabs(controlSignal) > 5) // currently set to within +/- 5 degrees per step
  {
    stepper1.setSpeed(controlSignal*20);
  }
  else
  {
    stepper1.setSpeed(0);
    stepper1.stop();
  }

   stepper1.runSpeed();
}


void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
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
    "trajectory_for_esp32"));  
  
  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher_renamed,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "micro_ros_arduino_node_publisher"));


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

  // assign value t float array msg 
//  float initialization[2] = {1.01, 2.01};
//  memcpy(test_msg.data.data,initialization,sizeof(initialization));
//  test_msg.data.size = 2;



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
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(100)));
  RCSOFTCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)));
}
