using System.Threading.Tasks;
using Unity.Burst.CompilerServices;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.Windows;

public class CarControl : MonoBehaviour
{
  // Motors and Brakes
  CarProperties.Drivetrain drivetrain;
  float motorTorque;
  float brakeTorque;
  float maxSpeed;

  // Wheels
  float wheelMass;
  float wheelRadius;
  float turnAngle;
  float turnAngleAtMaxSpeed;
  float tireFriction;

  // Suspension
  float suspensionDistance;
  float stiffness;
  float dampening;

  private float centreOfGravityOffset = -1f;

  private WheelControl[] wheels;
  private Rigidbody rigidBody;

  float vInput;
  float hInput;
  float moveInput;
  float speedFactor;

  private CarInputActions carControls; // Reference to the new input system

  void Awake()
  {
    carControls = new CarInputActions(); // Initialize Input Actions
  }
  void OnEnable()
  {
    carControls.Enable();
  }

  void OnDisable()
  {
    carControls.Disable();
  }

  // Start is called once before the first execution of Update after the MonoBehaviour is created
  void Start()
  {
    // Get all components attached to the car and its children
    rigidBody = GetComponent<Rigidbody>();
    wheels = GetComponentsInChildren<WheelControl>();

    // Assign all values from CarProperties
    drivetrain = GetComponent<CarProperties>().drivetrain;
    motorTorque = GetComponent<CarProperties>().motorTorque;
    brakeTorque = GetComponent<CarProperties>().brakeTorque;
    maxSpeed = GetComponent<CarProperties>().maxSpeed;
    wheelMass = GetComponent<CarProperties>().wheelMass;
    wheelRadius = GetComponent<CarProperties>().wheelRadius;
    turnAngle = GetComponent<CarProperties>().turnAngle;
    turnAngleAtMaxSpeed = GetComponent<CarProperties>().turnAngleAtMaxSpeed;
    tireFriction = GetComponent<CarProperties>().tireFriction;
    suspensionDistance = GetComponent<CarProperties>().suspensionDistance;
    stiffness = GetComponent<CarProperties>().stiffness;
    dampening = GetComponent<CarProperties>().dampening;

    // Adjust center of mass to improve stability and prevent rolling
    Vector3 centerOfMass = rigidBody.centerOfMass;
    centerOfMass.y += centreOfGravityOffset;
    rigidBody.centerOfMass = centerOfMass;
  }

  // Update is called once per fixed time frame
  void FixedUpdate()
  {
    Drivetrain();
    DrivingInput();
    MotorsBrakes();
    Steering();
    Suspension();
  }

  void Drivetrain()
  {
    foreach (var wheel in wheels)
    {
      if (drivetrain == CarProperties.Drivetrain.AWD)
      {
        wheel.motorized = true;
      }
      else if (drivetrain == CarProperties.Drivetrain.FWD)
      {
        if (wheel.gameObject.tag == "FrontWheel")
        {
          wheel.motorized = true;
        }
        else
        {
          wheel.motorized = false;
        }
      }
      else if (drivetrain == CarProperties.Drivetrain.RWD)
      {
        if (wheel.gameObject.tag == "RearWheel")
        {
          wheel.motorized = true;
        }
        else
        {
          wheel.motorized = false;
        }
      }
      else
      {
        Debug.Log("CarControls: Error in configuring drivetrain.");
        return;
      }
    }
  }

  void DrivingInput()
  {
    // Read the Vector2 input from the new Input System
    Vector2 inputVector = carControls.Car.Steering.ReadValue<Vector2>();

    // Get player input for acceleration and steering
    vInput = inputVector.y; // Forward/backward input
    hInput = inputVector.x; // Steering input

    // Read the Axis 1D input from the new Input System
    moveInput = carControls.Car.Movement.ReadValue<float>();
    Debug.Log("Inputs: X = " + vInput + " Y = " + hInput + " Move = " + moveInput);
  }

  void MotorsBrakes()
  {
    // Calculate current speed along the car's forward axis (direction of travel)
    float forwardSpeed = Vector3.Dot(transform.forward, rigidBody.linearVelocity);
    // Normalized speed factor (m/s to km/h by multiplying 3600/1000)
    speedFactor = Mathf.InverseLerp(0, maxSpeed, Mathf.Abs(forwardSpeed) * 3.6f);
    Debug.Log("Speed (km/h): " + rigidBody.linearVelocity.magnitude * 3.6f);

    // Reduce motor torque and steering at high speeds for better handling
    float currentMotorTorque = Mathf.Lerp(motorTorque, 0, speedFactor);

    // Determine if the player is accelerating or trying to reverse
    bool isAccelerating = Mathf.Sign(moveInput) == Mathf.Sign(forwardSpeed);

    foreach (var wheel in wheels)
    {
      // Assign WheelCollider properties for each wheel
      wheel.WheelCollider.mass = wheelMass;
      wheel.WheelCollider.radius = wheelRadius;

      // Get current suspension compression/extension
      wheel.WheelCollider.GetGroundHit(out WheelHit hit);
      float distanceToGround = wheel.WheelCollider.transform.position.y - hit.point.y;

      // Get the current suspension spring settings
      JointSpring spring = wheel.WheelCollider.suspensionSpring;
      // Modify the spring & damper values
      spring.spring = stiffness;
      spring.damper = dampening;
      // Apply the modified settings back to the wheel collider
      wheel.WheelCollider.suspensionSpring = spring;

      // Get the current friction settings
      WheelFrictionCurve frontFriction = wheel.WheelCollider.forwardFriction;
      WheelFrictionCurve sideFriction = wheel.WheelCollider.sidewaysFriction;
      // Modify the stiffness values
      frontFriction.stiffness = tireFriction;
      sideFriction.stiffness = tireFriction;
      // Apply the modified settings back to the wheel collider
      wheel.WheelCollider.forwardFriction = frontFriction;
      wheel.WheelCollider.sidewaysFriction = sideFriction;

      if (isAccelerating)
      {
        // Apply torque to motorized wheels
        if (wheel.motorized)
        {
          wheel.WheelCollider.motorTorque = moveInput * currentMotorTorque;
        }
        // Release brakes when accelerating
        wheel.WheelCollider.brakeTorque = 0f;
      }
      else
      {
        // Apply brakes when reversing direction
        wheel.WheelCollider.motorTorque = 0f;
        wheel.WheelCollider.brakeTorque = Mathf.Abs(moveInput) * brakeTorque;
      }

      //wheel.WheelCollider.brakeTorque
    }
  }

  void Steering()
  {
    // Calculate the steer angle range in relation to speed factor
    float currentSteerRange = Mathf.Lerp(turnAngle, turnAngleAtMaxSpeed, speedFactor);

    foreach (var wheel in wheels)
    {
      // Apply steering to wheels that support steering
      if (wheel.steerable)
      {
        // Controls to steer to the current steer range
        float currentSteerAngle = hInput * currentSteerRange;

        // Transition smoothly between original to calculated turn angle
        float transitionSteerAngle = Mathf.Lerp(wheel.WheelCollider.steerAngle,
                                                currentSteerAngle, 0.1f);

        // Update wheel angle
        wheel.WheelCollider.steerAngle = transitionSteerAngle;
      }
    }
  }

  void Suspension()
  {
    foreach (var wheel in wheels)
    {
      wheel.WheelCollider.suspensionDistance = suspensionDistance;

      // Adjust ride height to half of the suspension distance in either way (up/down)
      wheel.transform.localPosition = new Vector3(wheel.transform.localPosition.x,
                                                  0.7075898f - suspensionDistance / 2,
                                                  wheel.transform.localPosition.z);
    }
  }
}
