using UnityEngine;
using UnityEngine.InputSystem; // Import the Input System namespace for CallbackContext

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
  [SerializeField] float frontTorqueMotorRPM = 6380f;
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
  [SerializeField] float rearTorqueMotorRPM = 5400f;
  [Tooltip("Max RPM of rear motor (red line).")]
  [Range(2000f, 20000f)]
  [SerializeField] float rearMaxMotorRPM = 19000f;
  [Tooltip("Gear ratio of rear single-speed transmission (e.g. If 9:1, input '9').")]
  [Range(1, 12)]
  [SerializeField] float rearGearRatio = 9.03f;
  [Tooltip("Max torque of each brake pads (Nm) - No ABS.")]
  [Range(100f, 10000f)]
  [SerializeField] float brakeTorque = 1200f;

  [Header("Wheels")]
  [Tooltip("Mass of wheel (kg).")]
  [Range(10f, 100f)]
  [SerializeField] float wheelMass = 20f;
  [Tooltip("Radius of wheel with tire (m).")]
  [Range(0.2f, 0.5f)]
  [SerializeField] float wheelRadius = 0.4f; // 235/40R19 Tire is 0.3353f
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

  [Header("Respawn")]
  [Tooltip("Specify the respawn coordinate whenever the car falls off the world or get stuck.")]
  [SerializeField] Vector3 respawnCoordinate = new Vector3(0f, 0f, 0f);
  [Tooltip("Specify the respawn rotation whenever the car falls off the world or get stuck.")]
  [SerializeField] Vector3 respawnRotation = new Vector3(0f, 0f, 0f);

  [Header("For Debugging")]
  [Tooltip("Scale of the debugging rays on forces acting on the car.")]
  [Range(0f, 1f)]
  [SerializeField] float lineScale = 0.1f;

  [Header("Experimental Feature")]
  [Tooltip("Dynamically update on input change (Will Break the Game! Only use it to show changes).")]
  [SerializeField] bool autoUpdate = false;

  [Header("Link Components")]
  [Tooltip("Assign CameraController to link the camera to the CarController.")]
  public CameraController camController;
  [Tooltip("Assign CarGUI to link the GUI to the CarController.")]
  public CarGUI carGUI;

  // For Components
  Rigidbody rb;
  [HideInInspector] public CollisionController collisionController;
  [HideInInspector] public AudioController audioController;
  [HideInInspector] public WheelController[] wheels;

  // For Variables
  // Fixed local space ground clearance to wheel axle / default wheel radius at scale = 1
  float clear = 0.4075898f;
  // Inputs
  float hInput = 0f;
  float vInput = 0f;
  float moveInput = 0f;
  float brakeInput = 0f;
  // Motors
  float actualSpeed = 0f;
  float speedometerSpeed = 0f;
  float maxSpeed = 0f;
  float speedFactor = 0f;
  float actualSpeedFactor = 0f;
  float powerFactor = 0f;
  float maxTorque = 0f;
  float torqueFactor = 0f;
  float currentFrontMotorRPM = 0f;
  float currentRearMotorRPM = 0f;
  float currentFrontMotorTorque = 0f;
  float currentRearMotorTorque = 0f;
  // Aerodynamics
  float rbSpeed = 0f;
  float dragMagnitude = 0f;
  float frontliftMagnitude = 0f;
  float rearLiftMagnitude = 0f;
  float[] suspensionDisplacement = new float[4];
  float[] previousSuspensionDisplacement = new float[4];
  float[] rollingResistanceMagnitude = new float[4];

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
    carControls.Car.Unstuck.performed += UnstuckToggle;
    carControls.Car.StatsToggle.performed += StatsToggle;
  }

  void OnDisable()
  {
    carControls.Disable();
    carControls.Car.Unstuck.performed -= UnstuckToggle;
    carControls.Car.StatsToggle.performed -= StatsToggle;
  }

  // Start is called once before the first execution of Update after the MonoBehaviour is created
  void Start()
  {
    // Ensure variables are assigned
    if (!camController || !carGUI)
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
      // (!) Ensure the set Edit > Project Settings > Time > Fixed Timstep from 0.02 (default) to 0.004.
      // (?) The default fixed timestep is too coarse for low-speed precision, leading to erractice motion
      wheel.wheelCollider.mass = wheelMass;
      wheel.wheelCollider.radius = wheelRadius;
      
      // Scale the wheel model using (Desired Radius / Reference Radius)
      // (?) Model Radius at Scale = 1 is 0.4075898f
      float wheelModelScale = wheelRadius / clear;
      wheel.wheel.localScale = new Vector3(1f, wheelModelScale, wheelModelScale);

      wheel.wheelCollider.ConfigureVehicleSubsteps(100, 1, 1);
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
      // (!) Consider local space fixed ground clearance to wheel axle for the wheel's position prior to runtime,
      //     irrelevant to the wheel's radius.
      wheel.transform.localPosition = new Vector3(wheel.transform.localPosition.x,
                                                  clear +
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
    GroundFriction();
    Aerodynamics();
    DataUpdateComponents();
    if (transform.position.y < -1) Unstuck();
    if (autoUpdate) Setup();
  }
  void DrivingInput()
  {
    // Read the Vector2 input from the Input System
    Vector2 turnInput = carControls.Car.Steering.ReadValue<Vector2>();

    // Get player input for acceleration and steering
    vInput = turnInput.y; // Forward/backward input
    hInput = turnInput.x; // Steering input

    // Read the value input from the Input System
    moveInput = carControls.Car.Movement.ReadValue<float>();
    brakeInput = carControls.Car.Brake.ReadValue<float>();

    // DEBUGGING
    Debug.Log($"Turning = X: {vInput}, Y: {hInput}; Move: {moveInput}, Brake: {brakeInput}");
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
    // Calculate actual current speed along the car's rigidbody forward axis (direction of travel)
    // And converting m/s to km/h by multiplying 3600/1000
    actualSpeed = Vector3.Dot(transform.forward, rb.linearVelocity) * 3.6f;

    // Determine if the player is accelerating or trying to reverse
    bool isAccelerating = Mathf.Sign(moveInput) == Mathf.Sign(actualSpeed);

    // Initialize local variables
    float averageWheelRPM = 0f, averageMotorTorquePerWheel = 0f;

    for (int i = 0; i < wheels.Length; i++)
    {
      // Initialize local variables
      float maxMotorTorque = 0f, torqueMotorRPM = 0f, maxMotorRPM = 0f, gearRatio = 0f;

      // Initialize parameter depending on the front or rear wheel
      if (wheels[i].gameObject.tag == "FrontWheel")
      {
        maxMotorTorque = frontMotorTorque;
        torqueMotorRPM = frontTorqueMotorRPM;
        maxMotorRPM = frontMaxMotorRPM;
        gearRatio = frontGearRatio;
      }
      else if (wheels[i].gameObject.tag == "RearWheel")
      {
        maxMotorTorque = rearMotorTorque;
        torqueMotorRPM = rearTorqueMotorRPM;
        maxMotorRPM = rearMaxMotorRPM;
        gearRatio = rearGearRatio;
      }

      // Calculation to simulate the motor torque curve
      // Get the wheel collider's RPM
      float wheelRPM = wheels[i].wheelCollider.rpm;
      averageWheelRPM += wheelRPM;
      // Calculate the current motor's RPM from wheel's RPM (torque multiplication)
      float currentMotorRPM = wheelRPM * gearRatio;
      // Normalize end of the torque curve after reaching max power
      float motorTorqueFactor = Mathf.InverseLerp(maxMotorRPM, torqueMotorRPM, Mathf.Abs(currentMotorRPM));
      // Range mapping the RPM with the torque
      float currentMotorTorque = Mathf.Lerp(0, maxMotorTorque, motorTorqueFactor);
      // Torque per wheel of axle motor
      float motorTorquePerWheel = currentMotorTorque / 2;
      averageMotorTorquePerWheel += motorTorquePerWheel;

      // Initialize local variables
      float wheelTorque = 0, brakepadTorque = 0, brakepadTorque_Move = 0, brakepadTorque_Brake = 0;

      if (isAccelerating)
      {
        // Apply torque to motorized wheels
        if (wheels[i].motorized)
        {
          wheelTorque = moveInput * frontGearRatio * motorTorquePerWheel;

          // For GUI
          if (wheels[i].gameObject.tag == "FrontWheel")
          {
            currentFrontMotorRPM = currentMotorRPM * moveInput;
            currentFrontMotorTorque = currentMotorTorque * moveInput;
          }
          else if (wheels[i].gameObject.tag == "RearWheel")
          {
            currentRearMotorRPM = currentMotorRPM * moveInput;
            currentRearMotorTorque = currentMotorTorque * moveInput;
          }
          else
          {
            Debug.LogError("CarControls: Error in determining Front & Rear Wheel in MotorsBrakes().");
            return;
          }
        }
        // Release brakes when accelerating
        brakepadTorque_Move = 0f;
      }
      else
      {
        // Release torque
        wheelTorque = 0f;

        // For GUI
        if (wheels[i].gameObject.tag == "FrontWheel")
        {
          currentFrontMotorRPM = 0f;
          currentFrontMotorTorque = 0f;
        }
        else if (wheels[i].gameObject.tag == "RearWheel")
        {
          currentRearMotorRPM = 0f;
          currentRearMotorTorque = 0f;
        }
        else
        {
          Debug.LogError("CarControls: Error in determining Front & Rear Wheel in MotorsBrakes().");
          return;
        }

        // Apply brakes when reversing direction
        brakepadTorque_Move = Mathf.Abs(moveInput) * brakeTorque;
      }
      // Apply brakes to stop (dedicated brake input)
      brakepadTorque_Brake = brakeInput * brakeTorque;
      // Choose the highest value between the brakes inputs
      brakepadTorque = Mathf.Max(brakepadTorque_Move, brakepadTorque_Brake);

      // Apply torque to WheelColliders with rolling resistance
      wheels[i].wheelCollider.motorTorque = wheelTorque;
      wheels[i].wheelCollider.brakeTorque = brakepadTorque;

      // DEBUGGING
      //Debug.Log($"Motor Torque: {wheelTorque}, Brake Torque: {brakepadTorque + rollingResistanceTorque}");
      if (wheels[i].wheelCollider.GetGroundHit(out WheelHit hit))
      {
        // Convert torque (Nm) to force (N) for visual consistency (F = T / R)
        // Visualize force acting against the road to move forward, opposite to the wheel's rolling direction due to friction
        Debug.DrawRay(hit.point + new Vector3(0, wheelRadius / 2, 0),
                      -hit.forwardDir * (wheelTorque / wheelRadius) * lineScale,
                      Color.green);
      }
    }

    // Calculate the average RPM of each wheel
    averageWheelRPM /= wheels.Length;
    // Calculate the average current speed based on current average wheel rotation (km/h)
    // (RPM * Wheel Circumference in KM * 60 mins)
    speedometerSpeed = averageWheelRPM * (Mathf.PI * (wheelRadius * 2 / 1000)) * 60;

    // Calculate the average max speed based on max average wheel rotation (km/h)
    // (RPM * Wheel Circumference in KM * 60 mins)
    float averageMaxMotorRPM = (frontMaxMotorRPM + rearMaxMotorRPM) / 2;
    float averageGearRatio = (frontGearRatio + rearGearRatio) / 2;
    maxSpeed = (averageMaxMotorRPM / averageGearRatio) * (Mathf.PI * wheelRadius * 2 / 1000) * 60;

    // Normalize speed factor of the wheels
    // (?) Mathf.InverseLerp return values are clamped between 0f and 1f
    speedFactor = Mathf.InverseLerp(0, maxSpeed, Mathf.Abs(speedometerSpeed));

    // Normalize actual speed factor of the wheels
    actualSpeedFactor = Mathf.InverseLerp(0, maxSpeed, Mathf.Abs(actualSpeed));

    // Normalize the power curve (Clamp motor power once reached max motor rpm at peak torque)
    float averageTorqueMotorRPM = (frontTorqueMotorRPM + rearTorqueMotorRPM) / 2;
    float currentAverageMotorRPM = averageWheelRPM * averageGearRatio;
    powerFactor = Mathf.InverseLerp(0, averageTorqueMotorRPM, Mathf.Abs(currentAverageMotorRPM));

    // Calculate the average max torque of each wheel
    averageMotorTorquePerWheel /= wheels.Length;
    maxTorque = (frontMotorTorque + rearMotorTorque) / wheels.Length;

    // Normalize torque factor of the motor for each wheel
    torqueFactor = Mathf.InverseLerp(0, maxTorque, Mathf.Abs(averageMotorTorquePerWheel));

    // DEBUGGING
    //Debug.Log($"Speedometer Speed (km/h): {speedometerSpeed}, Actual Speed (km/h): {actualSpeed}");
    //Debug.Log($"Average Wheel RPM: {averageWheelRPM}, Average Motor RPM: {currentAverageMotorRPM}");
    //Debug.Log($"Speed Factor: {speedFactor}, Power Factor: {powerFactor}, Torque Factor: {torqueFactor}");
  }

  void GroundFriction()
  {
    foreach (var wheel in wheels)
    {
      // If wheel is grounded, get physics material's friction value
      if (wheel.wheelCollider.GetGroundHit(out WheelHit hit))
      {
        float groundFriction = hit.collider.material.staticFriction;

        // Get the current friction settings
        WheelFrictionCurve frontFriction = wheel.wheelCollider.forwardFriction;
        WheelFrictionCurve sideFriction = wheel.wheelCollider.sidewaysFriction;
        // Modify the stiffness values
        frontFriction.stiffness = tireFriction * groundFriction;
        sideFriction.stiffness = tireFriction * groundFriction;
        // Apply the modified settings back to the wheel collider
        wheel.wheelCollider.forwardFriction = frontFriction;
        wheel.wheelCollider.sidewaysFriction = sideFriction;
      }
    }
  }

  // ====================================================================================================
  // AERODYNAMICS
  void Aerodynamics()
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
    // Get the angle between the car object's forward dir and RB's velocity direction
    // (?) "transform" is for local axis while "vector3" is for global axis
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
    dragMagnitude = 0.5f * airDensity * Cd * A * rbSpeed * rbSpeed;

    // Yaw drag multiplier - when cornering
    float yawRate = Mathf.Abs(rb.angularVelocity.y);
    dragMagnitude *= 1f + yawRate * yawDragMultiplier;

    // Apply drag to the car in the opposite direction of velocity
    Vector3 dragForce = -rb.linearVelocity.normalized * dragMagnitude;
    rb.AddForce(dragForce);

    // DEBUGGING
    //Debug.Log($"Total Drag (N): {dragMagnitude}");
    Debug.DrawRay(transform.position + new Vector3(0, 1.5f, 0), dragForce * lineScale, Color.blue);
  }

  void LiftDownforce()
  {
    // Apply lift/downforce equation (Fd = 0.5 * p * Cl * A * v2)
    frontliftMagnitude = 0.5f * airDensity * frontLiftDownforceCoefficieint * influenceArea * rbSpeed * rbSpeed;
    rearLiftMagnitude = 0.5f * airDensity * rearLiftDownforceCoefficieint * influenceArea * rbSpeed * rbSpeed;
    Vector3 frontLiftForce = Vector3.up * frontliftMagnitude;
    Vector3 rearLiftForce = Vector3.up * rearLiftMagnitude;

    // Apply lift/downforce forces to front and rear axles relative to midpoint of car
    Vector3 frontAxle = transform.TransformPoint(0f, 0f, 1.739395f);
    Vector3 rearAxle = transform.TransformPoint(0f, 0f, -1.572661f);
    // NOTE: Position is in world coordinates
    rb.AddForceAtPosition(frontLiftForce, frontAxle);
    rb.AddForceAtPosition(rearLiftForce, rearAxle);

    // DEBUGGING
    //Debug.Log($"Total Lift/Downforce = Front (N): {frontLiftForce.y}, Rear (N): {rearLiftForce.y}");
    Debug.DrawRay(frontAxle, frontLiftForce * lineScale, Color.cyan);
    Debug.DrawRay(rearAxle, rearLiftForce * lineScale, Color.cyan);
  }

  void RollingResistance()
  {
    // Calculate average force on each wheel at rest
    float chassisWeight = ladenMass * Physics.gravity.magnitude / wheels.Length;
    float wheelWeight = wheelMass * Physics.gravity.magnitude;
    float weightAtRest = chassisWeight + wheelWeight;

    for (int i = 0; i < wheels.Length; i++)
    {
      // If wheel is grounded and moving, get current suspension compression/extension
      if (wheels[i].wheelCollider.GetGroundHit(out WheelHit hit))
      {
        // Calculate current suspension distance
        float distanceToGround = wheels[i].wheelCollider.transform.position.y - hit.point.y;
        // Calculate suspension displacement length from the midpoint (how much the spring is pushed in)
        // (!) Considers world space default wheel collider distance from the ground, to be wheel's radius
        //     and half of the suspension length in runtime
        suspensionDisplacement[i] = (wheelRadius + suspensionDistance / 2) - distanceToGround;

        // Check if wheel is in motion
        if (Mathf.Abs(wheels[i].wheelCollider.rpm) > 0.001f)
        {
          // Using Hooke's Law (F = -k * x) to calculate the force applied by the spring stiffness & dampening
          // (?) The suspension displacement will reflect on the car's laden weight and any lift/downforce
          float springForce = stiffness * suspensionDisplacement[i];
          // Calculate suspension velocity (how fast the suspension is rebounding)
          float susVelocity = (distanceToGround - previousSuspensionDisplacement[i]) / Time.fixedDeltaTime;
          // Calculate damping force (F = -c * v), opposing force
          float dampingForce = dampening * susVelocity;
          // Calculate total suspension force by combining spring & damping forces
          float suspensionForce = springForce + dampingForce;

          // Calculate total normal forces with the wheel included
          // (?) Clamp to 0 since spring's force tries to move upwards (compressing) if it's extended downwards beyond midpoint,
          //     thus no rolling resistance since no force pushing against the ground, especially when the car is not grounded
          float Fn = Mathf.Max(0, weightAtRest + suspensionForce);

          // For damping force calculation
          previousSuspensionDisplacement[i] = distanceToGround;

          // Apply rolling resistance equation (Frr = Crr * Fn) depending on wheel's rotation direction
          // (!) Rolling resistance to be applied in MotorsBrakes() to the wheel collider as brake torque to resist its motion
          rollingResistanceMagnitude[i] = rollingResistanceCoefficient * Fn;

          // Apply rolling resistance to the opposite direction of the wheel
          // (?) In real-world physics calculation, the force is applied in the opposite of the wheel rolling direction,
          //     where force would be in the forward rolling direction of the wheel to go backwards due to friction.
          //     However, in case of simulation, force is directly applied backwards.
          Vector3 rollingResistanceForce = -Mathf.Sign(wheels[i].wheelCollider.rpm) *
                                           hit.forwardDir * rollingResistanceMagnitude[i];
          rb.AddForceAtPosition(rollingResistanceForce, hit.point);

          // DEBUGGING
          //Debug.Log($"Spring Force (N): {springForce}," +
          //          $"Damping Force: {dampingForce} ," +
          //          $"Rolling Resistance Force (N): {rollingResistanceMagnitude[i]}");
          // Visualize force acting against the road to move forward, opposite to the wheel's rolling direction due to friction
          Debug.DrawRay(hit.point + new Vector3(0, wheelRadius / 2, 0),
                        -rollingResistanceForce * lineScale,
                        Color.red);
        }
        else
        {
          rollingResistanceMagnitude[i] = 0f;
        }
      }
    }
  }
  // ====================================================================================================

  void Unstuck()
  {
    transform.position = respawnCoordinate;
    transform.rotation = Quaternion.Euler(respawnRotation);
    rb.linearVelocity = new Vector3(0, 0, 0);
    rb.angularDamping = 0f;
  }

  void UnstuckToggle(InputAction.CallbackContext context)
  {
    // If car falls off the world or get stuck, reset its transform & speed
    //if (transform.position.y < -1 || unstuckInput != 0)
    //{
    if (context.performed)
    {
      Unstuck();
    }
  }

  void StatsToggle(InputAction.CallbackContext context)
  {
    // Toggle stats menu
    //if (statsToggle != 0)
    //{
    if (context.performed)
    {
      carGUI.StatsToggle();
    }
  }

  // Let CarController script update only the necessary data to other scripts
  // Keep it's data private
  void DataUpdateComponents()
  {
    // Components
    camController.DataUpdate(actualSpeed, actualSpeedFactor);
    collisionController.DataUpdate(maxSpeed);
    audioController.DataUpdate(speedFactor, powerFactor);
    foreach (var wheel in wheels)
    {
      wheel.DataUpdate(actualSpeed);
    }

    // GUI
    carGUI.Speedometer(speedometerSpeed);
    carGUI.CarStats(currentFrontMotorRPM, currentRearMotorRPM, currentFrontMotorTorque, currentRearMotorTorque,
                    speedFactor, actualSpeedFactor, powerFactor, torqueFactor,
                    dragMagnitude, frontliftMagnitude, rearLiftMagnitude,
                    actualSpeed);
    carGUI.WheelStats(wheels, suspensionDisplacement, rollingResistanceMagnitude);
  }
}