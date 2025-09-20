using UnityEngine;

public class CarController : MonoBehaviour
{
  // Default vehicle specs are referenced/inspired by Tesla Model 3 (Highlander) - Performance*
  // *Where Possible
  public enum Drivetrain { AWD, FWD, RWD } // selection between 3 types of drivetrains

  [Header("Chassis")]
  [Tooltip("Laden mass of the car (kg), excluding wheels.")]
  [Range(1000f, 10000f)]
  [SerializeField] float ladenMass = 1851f + 80f; // Includes 80kg driver
  [Range(-0.5f, 0.5f)]
  [Tooltip("Mass offset of the center of gravity of the car.")]
  [SerializeField] float centreOfGravityOffset = -0.5f;

  [Header("Motors & Brakes")]
  [Tooltip("Select type of drivetrain.")]
  [SerializeField] Drivetrain drivetrain = Drivetrain.AWD;
  [Tooltip("Max torque of front motor (Nm) - No traction control.")]
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
  [Tooltip("Max torque of rear motor (Nm) - No traction control.")]
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

  [Header("Wheels")]
  [Tooltip("Mass of wheel (kg).")]
  [Range(20f, 20f)]
  [SerializeField] float wheelMass = 20;
  [Tooltip("Radius of wheel with tire (m).")]
  [Range(0.4075898f, 0.4075898f)]
  [SerializeField] float wheelRadius = 0.4075898f;
  [Tooltip("Maximum steering angle at rest (degrees).")]
  [Range(1f, 60f)]
  [SerializeField] float TurnAngle = 30f;
  [Tooltip("Maximum steering angle at max speed (degrees).")]
  [Range(1f, 60f)]
  [SerializeField] float TurnAngleAtMaxSpeed = 15f;
  [Tooltip("Tire friction factor.")]
  [Range(0.1f, 10f)]
  [SerializeField] float tireFriction = 1f;

  [Header("Suspensions")]
  [Tooltip("Maximum allowable displacement of suspension within car's wheel well (m).")]
  [Range(0f, 0.3f)]
  [SerializeField] float allowSuspensionDisplacement = 0.3f;
  [Tooltip("Maximum distance of suspension (m).")]
  [Range(0f, 1f)]
  [SerializeField] float suspensionDistance = 0.3f;
  [Tooltip("Spring constant, k (N/m)")]
  [Range(10000f, 100000f)]
  [SerializeField] float stiffness = 25000f;
  [Tooltip("Damping coefficient, c (Ns/m)")]
  [Range(1000f, 10000f)]
  [SerializeField] float dampening = 2500f;

  [Header("Aerodynamics")]
  [Tooltip("Air density (p), default according to ISA (kg/m3).")]
  [SerializeField] float airDensity = 1.225f;
  [Tooltip("Estimated total front area (A) of the car (m2).")]
  [SerializeField] float frontArea = 2.7f;
  [Tooltip("Estimated total front drag coefficient (Cd) of the car.")]
  [SerializeField] float frontDragCoefficient = 0.2f;
  [Tooltip("Estimated total side areas (A) of the car (m2).")]
  [SerializeField] float sideAreas = 6.8f;
  [Tooltip("Estimated total side drag coefficient (Cd) of the car.")]
  [SerializeField] float sideDragCoefficient = 0.6f;
  [Tooltip("Estimated total rear area (A) of the car (m2).")]
  [SerializeField] float rearArea = 2.7f;
  [Tooltip("Estimated total rear drag coefficient (Cd) of the car.")]
  [SerializeField] float rearDragCoefficient = 0.3f;
  [Tooltip("Scales drag increase according to yaw rate.")]
  [SerializeField] float yawDragMultiplier = 0.1f;
  [Tooltip("Estimated total influence area (A) of lift/downforce accessories of the car (m2).")]
  [SerializeField] float influenceArea = 2.7f;
  [Tooltip("Estimated total front axle lift(+)/downforce(-) coefficient (Cl) of the car.")]
  [SerializeField] float frontLiftDownforceCoefficieint = 0.05f; // Positive for lift
  [Tooltip("Estimated total rear axle lift(+)/downforce(-) coefficient (Cl) of the car.")]
  [SerializeField] float rearLiftDownforceCoefficieint = -0.2f; // Negative for downforce (lip spoiler)
  [Tooltip("Estimated total rolling resistance coefficient (Crr) of each wheel.")]
  [SerializeField] float rollingResistanceCoefficient = 0.015f;

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
  float actualSpeedFactor;
  float powerFactor;
  float torqueFactor;
  float forwardSpeed;
  float rbSpeed;
  float[] previousSuspensionDisplacement = new float[4];

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

    //// Smooths sub-frame movement
    //rb.interpolation = RigidbodyInterpolation.Interpolate;

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
    SuspensionSetup();
    DrivetrainSetup();
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
    }
  }

  void SuspensionSetup()
  {
    foreach (var wheel in wheels)
    {
      // Assign suspension distance
      wheel.wheelCollider.suspensionDistance = suspensionDistance;
      // Adjust ride height to half of the suspension distance in both direction (up/down)
      // (?) Ride height stays constant by increasing the suspension's y-axis position (by half) into the wheel well
      //     until it reaches the wheel well's allowable suspension displacement. From there, its position will remain
      //     at the same maximum allowable position where the wheel will extend lower downwards increasing the ride height.
      // (!) Wheel size is constant with the model size
      wheel.transform.localPosition = new Vector3(wheel.transform.localPosition.x,
                                                  wheelRadius +
                                                  Mathf.Min(suspensionDistance, allowSuspensionDisplacement) / 2,
                                                  wheel.transform.localPosition.z);

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

  // FixedUpdate is called once per fixed time frame
  void FixedUpdate()
  {
    DrivingInput();
    Steering();
    MotorsBrakes();
    AerodynamicForces();
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
    Debug.Log($"Turning - X: {vInput}, Y: {hInput}; Move: {moveInput}, Brake: {brakeInput}");
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
    // Calculate the current average RPM from all wheels
    // (!) Bug - Using Wheel Collider's RPM causes stutters
    float averageWheelRPM = 0f;
    foreach (var wheel in wheels)
    {
      averageWheelRPM += wheel.wheelCollider.rpm;
    }
    averageWheelRPM /= wheels.Length;

    // Calculate the current speed based on current average wheel rotation (km/h)
    // (RPM * Wheel Circumference in KM * 60 mins)
    float currentSpeed = averageWheelRPM * (Mathf.PI * (wheelRadius * 2 / 1000)) * 60;

    // Calculate the average max speed based on max average wheel rotation (km/h)
    // (RPM * Wheel Circumference in KM * 60 mins)
    float averageMaxMotorRPM = (frontMaxMotorRPM + rearMaxMotorRPM) / 2;
    float averageGearRatio = (frontGearRatio + rearGearRatio) / 2;
    maxSpeed = (averageMaxMotorRPM / averageGearRatio) * (Mathf.PI * wheelRadius * 2 / 1000) * 60;

    // Normalize speed factor of the wheels
    speedFactor = Mathf.Clamp01(Mathf.InverseLerp(0, maxSpeed, currentSpeed));

    //// Get the current torque (superseded - less dynamic)
    //float currentMotorTorque = motorTorqueCurve.Evaluate(averageMotorRPM);

    // Calculation to simulate the motor torque curve
    // Normalize the power curve (Clamp motor power once reached max motor rpm at peak torque)
    float motorRPM = (frontMotorRPM + rearMotorRPM) / 2;
    float currentMotorRPM = averageWheelRPM * averageGearRatio;
    powerFactor = Mathf.Clamp01(Mathf.InverseLerp(0, motorRPM, currentMotorRPM));
    // Normalize end of the torque curve after reaching max power
    float frontMotorTorqueFactor = Mathf.Clamp01(Mathf.InverseLerp(frontMaxMotorRPM,
                                                                   frontMotorRPM,
                                                                   currentMotorRPM));
    float rearMotorTorqueFactor = Mathf.Clamp01(Mathf.InverseLerp(rearMaxMotorRPM,
                                                                  rearMotorRPM,
                                                                  currentMotorRPM));
    torqueFactor = (frontMotorTorqueFactor + rearMotorTorqueFactor) / 2; // Average torque factor of the motors
    // Range mapping the RPM with the torque
    float currentFrontMotorTorque = Mathf.Lerp(0, frontMotorTorque, frontMotorTorqueFactor);
    float currentRearMotorTorque = Mathf.Lerp(0, rearMotorTorque, rearMotorTorqueFactor);

    // Calculate actual current speed along the car's rigidbody forward axis (direction of travel)
    // And converting m/s to km/h by multiplying 3600/1000
    forwardSpeed = Vector3.Dot(transform.forward, rb.linearVelocity) * 3.6f;

    // Normalize actual speed factor of the wheels
    actualSpeedFactor = Mathf.Clamp01(Mathf.InverseLerp(0, maxSpeed, forwardSpeed));

    // DEBUGGING
    Debug.Log($"Speedometer Speed (km/h): {currentSpeed}, Actual Speed (km/h): {forwardSpeed}");
    //Debug.Log($"Average Motor RPM: {currentMotorRPM}");
    //Debug.Log($"Front Torque (Nm): {currentFrontMotorTorque}, Rear Torque (Nm): {currentRearMotorTorque}");
    //Debug.Log($"Speed Factor: {speedFactor}, Power Factor: {powerFactor}, Torque Factor: {torqueFactor}");

    // Determine if the player is accelerating or trying to reverse
    bool isAccelerating = Mathf.Sign(moveInput) == Mathf.Sign(forwardSpeed);

    foreach (var wheel in wheels)
    {
      // Initialize local variables
      float wheelTorque = 0f;
      float brakepadTorque = 0f;
      float brakepadTorque_Move = 0f;
      float brakepadTorque_Brake = 0f;

      if (isAccelerating)
      {
        // Apply torque to motorized wheels
        if (wheel.motorized)
        {
          if (wheel.gameObject.tag == "FrontWheel")
          {
            // Per wheel of axle motor
            wheelTorque = moveInput * frontGearRatio * (currentFrontMotorTorque / 2);
          }
          else if (wheel.gameObject.tag == "RearWheel")
          {
            // Per wheel of axle motor
            wheelTorque = moveInput * rearGearRatio * (currentRearMotorTorque / 2);
          }
        }
        // Release brakes when accelerating
        brakepadTorque_Move = 0f;
      }
      else
      {
        // Release torque
        wheelTorque = 0f;
        // Apply brakes when reversing direction
        brakepadTorque_Move = Mathf.Abs(moveInput) * brakeTorque;
      }
      // Apply brakes to stop (dedicated brake input)
      brakepadTorque_Brake = brakeInput * brakeTorque;
      // Choose the highest value between the brakes inputs
      brakepadTorque = Mathf.Max(brakepadTorque_Move, brakepadTorque_Brake);

      // Apply to WheelColliders
      wheel.wheelCollider.motorTorque = wheelTorque;
      wheel.wheelCollider.brakeTorque = brakepadTorque;

      // DEBUGGING
      //Debug.Log($"Motor Torque: {wheelTorqu e}, Brake Torque: {brakepadTorque}");
    }
  }

  // Wheel included affected
  // Applying the quadratic drag formula: Fd = -0.5 * p * Cd * A * v2
  // Unity drag unit is 1/sec (inverse seconds); ForceMode.Force is in (N) as with Drag
  // Ensure to set the drag of the rigidbody to zero as we're adding custom forces in different directions
  void AerodynamicForces()
  {
    // Disable rigidbody's drag
    rb.linearDamping = 0f;
    rb.angularDamping = 0f;

    // Rigidbody's magnitude of velocity (m/s)
    rbSpeed = rb.linearVelocity.magnitude;

    FrictionDrag();
    LiftDownforce();
    RollingResistance();
  }

  void FrictionDrag()
  {
    // Get the angle between the car object's forward dir and RB's velocity dir
    // NOTE: "transform" is for local axis while "vector3" is gor global axis
    float signedAngle = Vector3.SignedAngle(transform.forward, // Direction of the car
                                            rb.linearVelocity.normalized, // Direction of velocity
                                            Vector3.up); // Relative to global y-axis
    float absAngle = Mathf.Abs(signedAngle); // Only magnitude of angle

    // Interpolate Cd and A
    float Cd, A;
    if (absAngle <= 90f)
    {
      float t = absAngle / 90f; // 0 is front, 1 is side
      Cd = Mathf.Lerp(frontDragCoefficient, sideDragCoefficient, t);
      A = Mathf.Lerp(frontArea, sideAreas, t);
    }
    else
    {
      float t = (absAngle - 90f) / 90f; // 0 is side, 1 is rear
      Cd = Mathf.Lerp(sideDragCoefficient, rearDragCoefficient, t);
      A = Mathf.Lerp(sideAreas, rearArea, t);
    }

    // Apply drag equation (Fd = 0.5 * p * Cd * A * v2)
    float dragMagnitude = 0.5f * airDensity * Cd * A * rbSpeed * rbSpeed;

    // Yaw drag multiplier - when cornering
    float yawRate = Mathf.Abs(rb.angularVelocity.y);
    dragMagnitude *= 1f + yawRate * yawDragMultiplier;

    // Applying drag to the car in the opposite direction of velocity
    Vector3 dragForce = -rb.linearVelocity.normalized * dragMagnitude;
    rb.AddForce(dragForce);

    // DEBUGGING
    Debug.Log($"Total Drag (N): {dragMagnitude}");
  }

  void LiftDownforce()
  {
    // Apply lift/downforce equation (Fd = 0.5 * p * Cl * A * v2)
    float liftMagnitude = 0.5f * airDensity * influenceArea * rbSpeed * rbSpeed;
    Vector3 frontLiftForce = Vector3.up * liftMagnitude * frontLiftDownforceCoefficieint;
    Vector3 rearLiftForce = Vector3.up * liftMagnitude * rearLiftDownforceCoefficieint;

    // Apply lift/downforce forces to front and rear axles relative to midpoint of car
    Vector3 frontAxle = new Vector3(0f, 0f, 1.739395f);
    Vector3 rearAxle = new Vector3(0f, 0f, -1.572661f);
    // NOTE: Position is in world coordinates
    rb.AddForceAtPosition(frontLiftForce, transform.position + frontAxle);
    rb.AddForceAtPosition(rearLiftForce, transform.position + rearAxle);

    // DEBUGGING
    Debug.Log($"Total Lift/Downforce - Front (N): {frontLiftForce}, Rear (N): {rearLiftForce}");
  }

  void RollingResistance()
  {
    for (int i = 0; i < previousSuspensionDisplacement.Length; i++)
    {
      // Get current suspension compression/extension
      if (wheels[i].wheelCollider.GetGroundHit(out WheelHit hit))
      {
        // Calculate current suspension distance
        float distanceToGround = wheels[i].wheelCollider.transform.position.y - hit.point.y;
        // Calculate suspension displacement length from the midpoint (how much the spring is pushed in)
        float suspensionDisplacement = (wheelRadius + suspensionDistance / 2) - distanceToGround;

        // Using Hooke's Law (F = -k * x) to calculate the force applied by the spring stiffness & dampening
        // (?) The suspension displacement will reflect on the car's laden weight and any lift/downforce
        float springForce = stiffness * suspensionDisplacement;
        // Calculate suspension velocity (how fast the suspension is rebounding)
        float susVelocity = (distanceToGround - previousSuspensionDisplacement[i]) / Time.fixedDeltaTime;
        // Calculate damping force (F = -c * v), opposing force
        float dampingForce = dampening * susVelocity;
        // Calculate total suspension force by combining spring & damping forces
        // (?) Clamp to 0 since spring's force tries to move upwards if it extend downwards beyond midpoint,
        //     thus no rolling resistance since no force pushing against the ground especially if the car lifts up
        float totalSusForce = Mathf.Max(0, springForce + dampingForce);

        // For damping force calculation
        previousSuspensionDisplacement[i] = distanceToGround;

        // Apply rolling resistance equation (Frr = Crr * Fn)
        float rollingMagnitude = rollingResistanceCoefficient * totalSusForce;

        // Applying rolling resistance to the front tire's contact point
        Vector3 rollingForce = -rb.linearVelocity.normalized * rollingMagnitude;
        // NOTE: Position is in world coordinates
        rb.AddForceAtPosition(rollingForce, hit.point);

        // DEBUGGING
        Debug.Log($"Spring Force (N): {springForce}, Damping Force: {dampingForce},Rolling Resistance Force (N): {rollingMagnitude}");
      }
    }
  }

  // Let CarController script update only the necessary data to other scripts
  // Keep it's data private
  void DataUpdateComponents()
  {
    camController.DataUpdate(actualSpeedFactor);
    collisionController.DataUpdate(maxSpeed);
    audioController.DataUpdate(speedFactor, powerFactor);
    foreach (var wheel in wheels)
    {
      wheel.DataUpdate(forwardSpeed);
    }
  }
}
