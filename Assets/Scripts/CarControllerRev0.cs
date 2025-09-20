using UnityEngine;

public class CarControllerRev0 : MonoBehaviour
{
  // Default vehicle specs are referenced/inspired by Tesla Model 3 (Highlander) - Long Range*
  // *Where Possible
  public enum Drivetrain { AWD, FWD, RWD } // selection between 3 types of drivetrains

  [Header("Chassis")]
  [Tooltip("Laden mass of the car (kg), excluding wheels.")]
  [SerializeField] float chassisMass = 1823f;
  [Tooltip("Mass offset of the center of gravity of the car.")]
  [SerializeField] float centreOfGravityOffset = -0.5f;

  [Header("Motors & Brakes")]
  [Tooltip("Select type of drivetrain.")]
  [SerializeField] Drivetrain drivetrain = Drivetrain.AWD;
  [Tooltip("Max torque of motors (Nm).")]
  [Range(100f, 2000f)]
  [SerializeField] float motorTorque = 459f;
  [Tooltip("Torque curve of the motors.")]
  [SerializeField] AnimationCurve motorTorqueCurve;
  [Tooltip("Max torque of brakes (Nm) - No ABS.")]
  [SerializeField] float brakeTorque = 500f;
  [Tooltip("Max speed (km/h).")]
  [SerializeField] float maxSpeed = 250f;
  [Tooltip("Driver assist allows the game to automatically reduce the speed as the car accelerates.")]
  [SerializeField] bool driverAssist = false;

  [Header("Wheels")]
  [Tooltip("Mass of wheel (kg).")]
  [SerializeField] float wheelMass = 20;
  [Tooltip("Radius of wheel (m).")]
  [SerializeField] float wheelRadius = 0.45f;
  [Tooltip("Maximum steering angle at rest (degrees).")]
  [SerializeField] float TurnAngle = 30f;
  [Tooltip("Maximum steering angle at max speed (degrees).")]
  [SerializeField] float TurnAngleAtMaxSpeed = 15f;
  [Tooltip("Tire friction factor.")]
  [SerializeField] float tireFriction = 1f;

  [Header("Suspensions")]
  [Tooltip("Maximum extension length of suspensions (m).")]
  [SerializeField] float suspensionDistance = 0.3f;
  [Tooltip("Spring constant, k (N/m)")]
  [SerializeField] float stiffness = 25000f;
  [Tooltip("Damping coefficient, c (Ns/m)")]
  [SerializeField] float dampening = 2500f;

  [Header("Aerodynamics")]
  [Tooltip("Air density (p), default according to ISA (kg/m3).")]
  [SerializeField] float airDensity = 1.225f;
  [Tooltip("Estimated total front area (A) of the car (m2).")]
  [SerializeField] float frontArea = 0.2f;
  [Tooltip("Estimated total front drag coefficient (Cd) of the car.")]
  [SerializeField] float frontDragCoefficient = 2.7f;
  [Tooltip("Estimated total side areas (A) of the car (m2).")]
  [SerializeField] float sideAreas = 0.6f;
  [Tooltip("Estimated total side drag coefficient (Cd) of the car.")]
  [SerializeField] float sideDragCoefficient = 6.8f;
  [Tooltip("Estimated total rear area (A) of the car (m2).")]
  [SerializeField] float rearArea = 0.3f;
  [Tooltip("Estimated total rear drag coefficient (Cd) of the car.")]
  [SerializeField] float rearDragCoefficient = 2.7f;
  [Tooltip("Scales drag increase due to yaw rate.")]
  [SerializeField] private float yawDragMultiplier = 0.1f;

  [Header("Link Camera")]
  [Tooltip("Assign CameraController to link the camera to the CarController.")]
  public CameraController camController;

  // For Components
  Rigidbody rb;
  [HideInInspector] public CollisionController collisionController;
  [HideInInspector] public AudioController audioController;
  [HideInInspector] public WheelController[] wheels;

  // For Variables
  float hInput;
  float vInput;
  float moveInput;
  float brakeInput;
  float speedFactor;
  float targetMotorTorque;
  float targetBrakeTorque;

  // Reference to the new input system
  CarInputActions carControls;

  void Awake()
  {
    // Initialize Input Actions
    carControls = new CarInputActions();
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
    // Ensure variables are assigned
    if (camController == null)
    {
      Debug.LogError("Cam Controller event is not assigned to CarController!");
      return;
    }

    // Get all components attached to the car and its children
    rb = GetComponent<Rigidbody>();
    collisionController = GetComponent<CollisionController>();
    audioController = GetComponent<AudioController>();
    wheels = GetComponentsInChildren<WheelController>();

    // Ensure WheelController.wheelCollider is initialized before calling it
    // (?) Otherwise, UnassignedReferenceException as both Start() runs simultaneously.
    // (!) Explicitly initialize it before it is being called in Setup() -> WheelSetup()
    // Alternatively:
    // (!) Go to Edit > Project Settings > Script Execution Order
    //     Add WheelController and set it's order to -1 or lower (-2 in this case).
    // (?) This ensures WheelController's Start() with wheelCollider initialization runs first
    foreach (var wheel in wheels)
    {
      wheel.Initialize();
    }

    // Initialize of car properties (after WheelController.wheelCollider has been initialized)
    Setup();

    // Lock & hide cursor
    Cursor.lockState = CursorLockMode.Locked;
  }

  // Setup is called once by start and when resetting-up
  // (?) To optimize performance as it doesn't require constant update
  void Setup()
  {
    ChassisSetup();
    DrivetrainSetup();
    WheelSetup();
  }

  void ChassisSetup()
  {
    // Assign car's mass
    rb.mass = chassisMass;

    // Adjust center of mass for battery pack to improve stability and prevent rolling
    Vector3 centerOfMass = rb.centerOfMass;
    centerOfMass.y += centreOfGravityOffset;
    rb.centerOfMass = centerOfMass;
  }

  void DrivetrainSetup()
  {
    foreach (var wheel in wheels)
    {
      // If AWD is selected
      if (drivetrain == Drivetrain.AWD)
      {
        wheel.motorized = true;
      }
      // If FWD is selected
      else if (drivetrain == Drivetrain.FWD)
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
      // If RWD is selected
      else if (drivetrain == Drivetrain.RWD)
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
        Debug.LogError("CarControls: Error in configuring drivetrain.");
        return;
      }
    }
  }

  void WheelSetup()
  {
    foreach (var wheel in wheels)
    {
      // Assign wheelCollider carProperties for each wheel
      wheel.wheelCollider.mass = wheelMass;
      wheel.wheelCollider.radius = wheelRadius;

      // Assign suspension distance
      wheel.wheelCollider.suspensionDistance = suspensionDistance;
      // Adjust ride height to half of the suspension distance in both direction (up/down)
      // (?) Ride height calculation works in the opposite as if locally positioned lower to increase height
      // (!) Wheel size is constant
      wheel.transform.localPosition = new Vector3(wheel.transform.localPosition.x,
                                                  0.7075898f - suspensionDistance / 2,
                                                  wheel.transform.localPosition.z);
    }
  }

  // FixedUpdate is called once per fixed time frame
  void FixedUpdate()
  {
    DrivingInput();
    MotorsBrakes();
    Steering();
    Suspension();
    AerodynamicResistance();
    DataUpdateComponents();
  }

  void DrivingInput()
  {
    // Read the Vector2 input from the Input System
    Vector2 turnInput = carControls.Car.Steering.ReadValue<Vector2>();

    // Get player input for acceleration and steering
    vInput = turnInput.y; // Forward/backward input
    hInput = turnInput.x; // Steering input

    // Read the Axis 1D input from the Input System
    moveInput = carControls.Car.Movement.ReadValue<float>();

    // Read the value input from the Input System
    brakeInput = carControls.Car.Brake.ReadValue<float>();

    // DEBUGGING
    Debug.Log($"Turning: X = {vInput}, Y = {hInput}; Move = {moveInput}, Brake = {brakeInput}");
  }

  // Fix Acc/Reverse Sensitivity to Prevent Stalling
  void MotorsBrakes()
  {
    // Calculate the average RPM from all wheels
    float averageRPM = 0f;
    foreach (var wheel in wheels)
    {
      averageRPM += wheel.wheelCollider.rpm;
    }

    averageRPM /= wheels.Length;

    // Get the current torque
    float currentMotorTorque = motorTorqueCurve.Evaluate(averageRPM);

    // Calculate current speed along the car's forward axis (direction of travel)
    // And converting m/s to km/h by multiplying 3600/1000
    float forwardSpeed = Vector3.Dot(transform.forward, rb.linearVelocity) * 3.6f;

    // Determine if the player is accelerating or trying to reverse
    bool isAccelerating = Mathf.Sign(moveInput) == Mathf.Sign(forwardSpeed);

    foreach (var wheel in wheels)
    {
      float targetBrakeTorque_Move = 0;
      float targetBrakeTorque_Brake = 0;

      if (isAccelerating)
      {
        // Apply torque to motorized wheels
        if (wheel.motorized)
        {
          targetMotorTorque = moveInput * currentMotorTorque;
        }
        // Release brakes when accelerating
        targetBrakeTorque = 0f;
      }
      else
      {
        // Release torque
        targetMotorTorque = 0f;
        // Apply brakes when reversing direction
        targetBrakeTorque_Move = Mathf.Abs(moveInput) * brakeTorque;
      }
      // Apply brakes to stop (dedicated brake input)
      targetBrakeTorque_Brake = brakeInput * brakeTorque;
      // Choose the highest value between the brakes inputs
      targetBrakeTorque = Mathf.Max(targetBrakeTorque_Move, targetBrakeTorque_Brake);

      // Apply to WheelColliders
      wheel.wheelCollider.motorTorque = targetMotorTorque;
      wheel.wheelCollider.brakeTorque = targetBrakeTorque;
    }

    // Apply torque to motors

    //// LESS REALISTIC - BASED ON CAR's RB Velocity
    //// Calculate current speed along the car's forward axis (direction of travel)
    //// And converting m/s to km/h by multiplying 3600/1000
    //float forwardSpeed = Vector3.Dot(transform.forward, rb.linearVelocity) * 3.6f;
    ////// Normalize speed factor
    ////speedFactor = Mathf.InverseLerp(0, maxSpeed, Mathf.Abs(wheelSpeed));
    ////// Reduce motor torque and steering at high speeds for better handling (Range Mapper)
    ////float currentMotorTorque = Mathf.Lerp(motorTorque, 0, speedFactor);

    ////// DEBUGGING
    ////Debug.Log($"Speed (km/h): {rb.linearVelocity.magnitude * 3.6f}");

    ////// Determine if the player is accelerating or trying to reverse
    ////bool isAccelerating = Mathf.Sign(moveInput) == Mathf.Sign(forwardSpeed);

    //// MORE REALISTIC
    //// Speedometer works by counting the average wheel rotation from sensors
    //float totalAngularVelocity = 0;
    //float wheelCount = 0;
    //foreach (var wheel in wheels)
    //{
    //  // Get the angular velocity (rotation speed) for each wheel (Degrees/s)
    //  totalAngularVelocity += wheel.wheelCollider.rotationSpeed;
    //  wheelCount++;
    //}
    //// Calculate the average rotation speed from Degree/s to Radian/s (360 degrees = 2 * PI rad)
    //float averageAngularVelocity = (totalAngularVelocity / (wheelCount + 1)) * Mathf.PI * 180;
    //// Convert angular velocity (rad/s) to linear velocity (m/s) using v = w * r
    //// And converting m/s to km/h by multiplying 3600/1000
    //float linearVelocity = (averageAngularVelocity * wheelRadius) * 3.6f;

    //// Normalize speed factor
    //speedFactor = Mathf.InverseLerp(0, maxSpeed, Mathf.Abs(forwardSpeed));
    //// Reduce motor torque and steering at high speeds for better handling (Range Mapper)
    //float currentMotorTorque = Mathf.Lerp(motorTorque, 0, speedFactor);

    //// DEBUGGING
    //Debug.Log($"Speed (km/h): {rb.linearVelocity.magnitude * 3.6f}; Speedometer (km/h): {linearVelocity}");
    //Debug.Log($"Forward Speed (km/h): {forwardSpeed}");

    //// Determine if the player is accelerating or trying to reverse
    //bool isAccelerating = Mathf.Sign(moveInput) == Mathf.Sign(forwardSpeed);

    //foreach (var wheel in wheels)
    //{
    //  float targetBrakeTorque_Move = 0;
    //  float targetBrakeTorque_Brake = 0;

    //  if (isAccelerating)
    //  {
    //    // Apply torque to motorized wheels
    //    if (wheel.motorized)
    //    {
    //      targetMotorTorque = moveInput * currentMotorTorque;
    //    }
    //    // Release brakes when accelerating
    //    targetBrakeTorque = 0f;
    //  }
    //  else
    //  {
    //    // Release torque
    //    targetMotorTorque = 0f;
    //    // Apply brakes when reversing direction
    //    targetBrakeTorque_Move = Mathf.Abs(moveInput) * brakeTorque;
    //  }
    //  // Apply brakes to stop (dedicated brake input)
    //  targetBrakeTorque_Brake = brakeInput * brakeTorque;
    //  // Choose the highest value between the brakes inputs
    //  targetBrakeTorque = Mathf.Max(targetBrakeTorque_Move, targetBrakeTorque_Brake);

    //  // Apply to WheelColliders
    //  wheel.wheelCollider.motorTorque = targetMotorTorque;
    //  wheel.wheelCollider.brakeTorque = targetBrakeTorque;
    //}
  }

  void Steering()
  {
    // Calculate the steer angle range in relation to speed factor
    float currentSteerRange = Mathf.Lerp(TurnAngle, TurnAngleAtMaxSpeed, speedFactor);

    foreach (var wheel in wheels)
    {
      // Apply steering to wheels that support steering
      if (wheel.steerable)
      {
        // Controls to steer to the current steer range
        float currentSteerAngle = hInput * currentSteerRange;

        // Transition smoothly between original to calculated turn angle
        float transitionSteerAngle = Mathf.Lerp(wheel.wheelCollider.steerAngle,
                                                currentSteerAngle,
                                                0.1f);

        // Update wheel angle
        wheel.wheelCollider.steerAngle = transitionSteerAngle;
      }
    }
  }

  void Suspension()
  {
    foreach (var wheel in wheels)
    {
      // Get current suspension compression/extension
      wheel.wheelCollider.GetGroundHit(out WheelHit hit);
      // Calculate current suspension distance
      float distanceToGround = wheel.wheelCollider.transform.position.y - hit.point.y;
      // Normalize suspension distance factor, ride height either compress/extend up to half of suspension distance
      // (?) Ride height calculation where ride height increases with half of suspension distance
      // (!) Wheel size is constant
      float suspensionFactor = Mathf.InverseLerp((0.4075898f + suspensionDistance / 2) - suspensionDistance / 2,
                                                 (0.4075898f + suspensionDistance / 2) + suspensionDistance / 2,
                                                 distanceToGround);
      // Calculate wheel grip relative to the suspension distance factor (Range Mapper)
      // - Depression, Shorter, More Grip
      // - Expansion, Longer, Less Grip
      float wheelGrip = Mathf.Lerp(1.5f, 0.5f, suspensionFactor);

      // Get the current suspension spring settings
      JointSpring spring = wheel.wheelCollider.suspensionSpring;
      // Modify the spring & damper values
      spring.spring = stiffness;
      spring.damper = dampening;
      // Apply the modified settings back to the wheel collider
      wheel.wheelCollider.suspensionSpring = spring;

      // Get the current friction settings
      WheelFrictionCurve frontFriction = wheel.wheelCollider.forwardFriction;
      WheelFrictionCurve sideFriction = wheel.wheelCollider.sidewaysFriction;
      // Modify the stiffness values
      frontFriction.stiffness = tireFriction * wheelGrip;
      sideFriction.stiffness = tireFriction * wheelGrip;
      // Apply the modified settings back to the wheel collider
      wheel.wheelCollider.forwardFriction = frontFriction;
      wheel.wheelCollider.sidewaysFriction = sideFriction;
    }
  }

  // Wheel included affected
  // Applying the quadratic drag formula: Fd = -0.5 * p * Cd * A * v2
  // Unity drag unit is 1/sec (inverse seconds); ForceMode.Force is in (N) as with Drag
  // Ensure to set the drag of the rigidbody to zero as we're adding custom forces in different directions
  void AerodynamicResistance()
  {

  }

  // Let CarController script update only the necessary data to other scripts
  // Keep it's data private
  void DataUpdateComponents()
  {
    camController.DataUpdate(speedFactor);
    collisionController.DataUpdate(maxSpeed);
    audioController.DataUpdate(speedFactor, targetMotorTorque);
    foreach (var wheel in wheels)
    {
      wheel.DataUpdate(speedFactor);
    }
  }
}
