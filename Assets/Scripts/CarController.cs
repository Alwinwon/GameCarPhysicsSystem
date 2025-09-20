using UnityEngine;

public class CarController : MonoBehaviour
{
  // Default vehicle specs are referenced/inspired by Tesla Model 3 (Highlander) - Performance*
  // *Where Possible
  public enum Drivetrain { AWD, FWD, RWD } // selection between 3 types of drivetrains

  [Header("Chassis")]
  [Tooltip("Laden mass of the car (kg), excluding wheels.")]
  [SerializeField] float ladenMass = 1851f + 80f; // Includes 80kg driver
  [Tooltip("Mass offset of the center of gravity of the car.")]
  [SerializeField] float centreOfGravityOffset = -0.5f;

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

  [Header("Motors & Brakes")]
  [Tooltip("Select type of drivetrain.")]
  [SerializeField] Drivetrain drivetrain = Drivetrain.AWD;
  [Tooltip("Max torque of front motor (Nm).")]
  [Range(100f, 1000f)]
  [SerializeField] float frontMotorTorque = 219f;
  [Tooltip("Max RPM for peak torque of front motor.")]
  [Range(1000f, 10000f)]
  [SerializeField] float frontMotorRPM = 6380f;
  [Tooltip("Max RPM of front motor (red line).")]
  [Range(2000f, 20000f)]
  [SerializeField] float frontMaxMotorRPM = 19000f;
  [Tooltip("Gear ratio of front single-speed transmission (e.g. If 9:1, input '9').")]
  [Range(1, 12)]
  [SerializeField] float frontGearRatio = 9.03f;
  [Tooltip("Max torque of rear motor (Nm).")]
  [Range(100f, 1000f)]
  [SerializeField] float rearMotorTorque = 340f;
  [Tooltip("Max RPM for peak torque of rear motor.")]
  [Range(1000f, 10000f)]
  [SerializeField] float rearMotorRPM = 5400f;
  [Tooltip("Max RPM of rear motor (red line).")]
  [Range(2000f, 20000f)]
  [SerializeField] float rearMaxMotorRPM = 19000f;
  [Tooltip("Gear ratio of rear single-speed transmission (e.g. If 9:1, input '9').")]
  [Range(1, 12)]
  [SerializeField] float rearGearRatio = 9.03f;
  [Tooltip("Max torque of each brake pads (Nm) - No ABS.")]
  [Range(100f, 10000f)]
  [SerializeField] float brakeTorque = 2000f;
  //[Tooltip("Driver assist allows the game to automatically reduce the speed as the car accelerates.")]
  //[SerializeField] bool driverAssist = false;

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
  float maxSpeed;
  float speedFactor;
  float powerFactor;
  float torqueFactor;
  float forwardSpeed;

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
      Debug.LogError("Cam Controller is not assigned to CarController!");
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
    WheelSetup();
    DrivetrainSetup();
    SuspensionSetup();
  }

  void ChassisSetup()
  {
    // Assign car's mass
    rb.mass = ladenMass;

    // Adjust center of mass for battery pack to improve stability and prevent rolling
    Vector3 centerOfMass = rb.centerOfMass;
    centerOfMass.y += centreOfGravityOffset;
    rb.centerOfMass = centerOfMass;
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
        if (wheel.gameObject.tag == "FrontWheel") wheel.motorized = true;
        else wheel.motorized = false;
      }
      // If RWD is selected
      else if (drivetrain == Drivetrain.RWD)
      {
        if (wheel.gameObject.tag == "RearWheel") wheel.motorized = true;
        else wheel.motorized = false;
      }
      else
      {
        Debug.LogError("CarControls: Error in configuring drivetrain.");
        return;
      }
    }
  }
  void SuspensionSetup()
  {
    foreach (var wheel in wheels)
    {
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
      frontFriction.stiffness = tireFriction;
      sideFriction.stiffness = tireFriction;
      // Apply the modified settings back to the wheel collider
      wheel.wheelCollider.forwardFriction = frontFriction;
      wheel.wheelCollider.sidewaysFriction = sideFriction;
    }
  }

  // FixedUpdate is called once per fixed time frame
  void FixedUpdate()
  {
    DrivingInput();
    Steering();
    MotorsBrakes();
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

  // Fix Acc/Reverse Sensitivity to Prevent Stalling
  void MotorsBrakes()
  {
    //// Calculate the current average RPM from all wheels
    //float averageWheelRPM = 0f;
    //foreach (var wheel in wheels)
    //{
    //  averageWheelRPM += wheel.wheelCollider.rpm;
    //}
    //averageWheelRPM /= wheels.Length;

    //// Calculate the current speed based on current average wheel rotation
    //// (RPM * Wheel Circumference in KM * 60 mins)
    //float currentSpeed = averageWheelRPM * (Mathf.PI * (wheelRadius * 2 / 1000)) * 60;

    //// Calculate the average max speed based on max average wheel rotation
    //// (RPM * Wheel Circumference in KM * 60 mins)
    //float averageMaxMotorRPM = (frontMaxMotorRPM + rearMaxMotorRPM) / 2;
    //float averageGearRatio = (frontGearRatio + rearGearRatio) / 2;
    //maxSpeed = (averageMaxMotorRPM / averageGearRatio) * (Mathf.PI * wheelRadius * 2 / 1000) * 60;

    //// Normalize speed factor of the wheels
    //speedFactor = Mathf.Clamp01(Mathf.InverseLerp(0, maxSpeed, currentSpeed));

    ////// Get the current torque
    ////float currentMotorTorque = motorTorqueCurve.Evaluate(averageMotorRPM);

    //// Calculation to simulate the motor torque curve
    //// Normalize the power curve (Clamp motor power once reached max motor rpm at peak torque)
    //float motorRPM = (frontMotorRPM + rearMotorRPM) / 2;
    //float currentMotorRPM = averageWheelRPM * averageGearRatio;
    //powerFactor = Mathf.Clamp01(Mathf.InverseLerp(0, motorRPM, currentMotorRPM));
    //// Normalize end of the torque curve after reaching max power
    //float frontMotorTorqueFactor = Mathf.Clamp01(Mathf.InverseLerp(frontMaxMotorRPM, frontMotorRPM, currentMotorRPM));
    //float rearMotorTorqueFactor = Mathf.Clamp01(Mathf.InverseLerp(rearMaxMotorRPM, rearMotorRPM, currentMotorRPM));
    //torqueFactor = (frontMotorTorqueFactor + rearMotorTorqueFactor) / 2; // Average torque factor of the motors
    //// Range mapping the RPM with the torque
    //float currentFrontMotorTorque = Mathf.Lerp(0, frontMotorTorque, frontMotorTorqueFactor);
    //float currentRearMotorTorque = Mathf.Lerp(0, rearMotorTorque, rearMotorTorqueFactor);

    //// Calculate actual current speed along the car's rigidbody forward axis (direction of travel)
    //// And converting m/s to km/h by multiplying 3600/1000
    //forwardSpeed = Vector3.Dot(transform.forward, rb.linearVelocity) * 3.6f;

    //// DEBUGGING
    //Debug.Log($"Speedometer Speed (km/h): {currentSpeed}, Actual Speed (km/h): {forwardSpeed}");
    //Debug.Log($"Average Motor RPM: {currentMotorRPM}");
    //Debug.Log($"Front Torque (Nm): {currentFrontMotorTorque}, Rear Torque (Nm): {currentRearMotorTorque}");
    //Debug.Log($"Speed Factor: {speedFactor}, Power Factor: {powerFactor}, Torque Factor: {torqueFactor}");

    //// Determine if the player is accelerating or trying to reverse
    //bool isAccelerating = Mathf.Sign(moveInput) == Mathf.Sign(forwardSpeed);

    //foreach (var wheel in wheels)
    //{
    //  // Initialize local variables
    //  float wheelTorque = 0f;
    //  float brakepadTorque = 0f;
    //  float brakepadTorque_Move = 0f;
    //  float brakepadTorque_Brake = 0f;

    //  if (isAccelerating)
    //  {
    //    // Apply torque to motorized wheels
    //    if (wheel.motorized)
    //    {
    //      if (wheel.gameObject.tag == "FrontWheel")
    //      {
    //        wheelTorque = moveInput * (currentFrontMotorTorque / 2) * frontGearRatio; // Per Wheel
    //      }
    //      else if (wheel.gameObject.tag == "RearWheel")
    //      {
    //        wheelTorque = moveInput * (currentRearMotorTorque / 2) * rearGearRatio; // Per Wheel
    //      }
    //    }
    //    // Release brakes when accelerating
    //    brakepadTorque_Move = 0f;
    //  }
    //  else
    //  {
    //    // Release torque
    //    wheelTorque = 0f;
    //    // Apply brakes when reversing direction
    //    brakepadTorque_Move = Mathf.Abs(moveInput) * brakeTorque;
    //  }
    //  // Apply brakes to stop (dedicated brake input)
    //  brakepadTorque_Brake = brakeInput * brakeTorque;
    //  // Choose the highest value between the brakes inputs
    //  brakepadTorque = Mathf.Max(brakepadTorque_Move, brakepadTorque_Brake);

    //  // Apply to WheelColliders
    //  wheel.wheelCollider.motorTorque = wheelTorque;
    //  wheel.wheelCollider.brakeTorque = brakepadTorque;
    //}

    Apply torque to motors

    // LESS REALISTIC - BASED ON CAR's RB Velocity
    // Calculate current speed along the car's forward axis (direction of travel)
    // And converting m/s to km/h by multiplying 3600/1000
    float forwardSpeed = Vector3.Dot(transform.forward, rb.linearVelocity) * 3.6f;
    //// Normalize speed factor
    //speedFactor = Mathf.InverseLerp(0, maxSpeed, Mathf.Abs(wheelSpeed));
    //// Reduce motor torque and steering at high speeds for better handling (Range Mapper)
    //float currentMotorTorque = Mathf.Lerp(motorTorque, 0, speedFactor);

    //// DEBUGGING
    //Debug.Log($"Speed (km/h): {rb.linearVelocity.magnitude * 3.6f}");

    //// Determine if the player is accelerating or trying to reverse
    //bool isAccelerating = Mathf.Sign(moveInput) == Mathf.Sign(forwardSpeed);

    // MORE REALISTIC
    // Speedometer works by counting the average wheel rotation from sensors
    float totalAngularVelocity = 0;
    float wheelCount = 0;
    foreach (var wheel in wheels)
    {
      // Get the angular velocity (rotation speed) for each wheel (Degrees/s)
      totalAngularVelocity += wheel.wheelCollider.rotationSpeed;
      wheelCount++;
    }
    // Calculate the average rotation speed from Degree/s to Radian/s (360 degrees = 2 * PI rad)
    float averageAngularVelocity = (totalAngularVelocity / (wheelCount + 1)) * Mathf.PI * 180;
    // Convert angular velocity (rad/s) to linear velocity (m/s) using v = w * r
    // And converting m/s to km/h by multiplying 3600/1000
    float linearVelocity = (averageAngularVelocity * wheelRadius) * 3.6f;

    // Normalize speed factor
    speedFactor = Mathf.InverseLerp(0, maxSpeed, Mathf.Abs(forwardSpeed));
    // Reduce motor torque and steering at high speeds for better handling (Range Mapper)
    float currentMotorTorque = Mathf.Lerp(motorTorque, 0, speedFactor);

    // DEBUGGING
    Debug.Log($"Speed (km/h): {rb.linearVelocity.magnitude * 3.6f}; Speedometer (km/h): {linearVelocity}");
    Debug.Log($"Forward Speed (km/h): {forwardSpeed}");

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
    audioController.DataUpdate(speedFactor, powerFactor, torqueFactor);
    foreach (var wheel in wheels)
    {
      wheel.DataUpdate(forwardSpeed);
    }
  }
}
