using UnityEngine;
using System.Collections; // // Import the System Collection namespace for IEnumerator

public class CameraController : MonoBehaviour
{
  [Header("Target")]
  [Tooltip("Assign the Car transform here.")]
  public Transform car;

  [Header("Turn Dynamic Settings")]
  [Tooltip("Camera offset (behind and above car).")]
  public Vector3 offset = new Vector3(0f, 1.5f, -6f);
  [Tooltip("Camera dynamic distance when accelerating and decelerating.")]
  public float maxDynamicDistance = 2f;
  [Tooltip("Camera turning damping (Lower value, higher damping).")]
  public float orbitDamping = 0.05f;
  [Tooltip("Camera auto rotate yaw sensitivity as the car is moving (Higher value, more sensitive).")]
  public float autoRotateSensitivity = 0.05f;
  [Tooltip("Camera collision avoidance damping (Lower value, higher damping).")]
  public float collisionAvoidanceDamping = 0.5f;
  [Tooltip("Assign the car's LayerMask for collision avoidance to ignore.")]
  public LayerMask carLayerMask;

  [Header("Mouse Orbit Settings")]
  [Tooltip("Mouse look sensitivity (Higher value, more sensitive).")]
  public float mouseSensitivity = 0.05f;
  [Tooltip("Mouse Y rotation (left & right).")]
  public float yaw = 0f;
  [Tooltip("Invert yaw axis.")]
  public bool invertYaw = false;
  [Tooltip("Mouse X rotation (up & down).")]
  public float pitch = 5f;
  [Tooltip("Invert pitch axis.")]
  public bool invertPitch = true;
  [Tooltip("Min pitch angle (degrees).")]
  public float minPitch = -10f;
  [Tooltip("Min pitch angle (degrees).")]
  public float maxPitch = 20f;

  // For Components
  [HideInInspector] public Transform cam;

  // For Variables
  float hInput;
  float vInput;
  float actualSpeed;
  float actualSpeedFactor;

  // Reference to the input system
  CarInputActions carControls;

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
    // Ensure variables are assigned
    if (!car)
    {
      Debug.LogError("Car not assigned to CameraController!");
      return;
    }
    else if (carLayerMask == 0)
    {
      Debug.LogError("Car Layer Mask not assigned to CameraController!");
      return;
    }

    // Check inputs
    if (minPitch > maxPitch)
    {
      Debug.LogError("Min Pitch is larger than Max Pitch! Check inputs again.");
      return;
    }

    // Get component attached to its children
    cam = GetComponentInChildren<Camera>().transform;

    // Initialize the position and rotation of the camera
    transform.position = car.position + new Vector3(offset.x, offset.y, 0); // Camera origin position
    cam.localPosition = new Vector3(0, 0, offset.z); // Local camera position
    transform.rotation = Quaternion.Euler(pitch, yaw, 0);
  }

  // LateUpdate is called once per frame after all Update functions have been called
  void FixedUpdate()
  {
    LookInput();
    Look();
  }

  void LookInput()
  {
    // Read the Vector2 input from the Input System
    Vector2 lookInput = carControls.Car.Looking.ReadValue<Vector2>();

    // Get player input for acceleration and steering
    // Invert Yaw and/or Pitch if true
    // Look Left/Right
    if (invertYaw) hInput = -lookInput.x;
    else hInput = lookInput.x;
    // Look Up/Down
    if (invertPitch) vInput = -lookInput.y;
    else vInput = lookInput.y;

    // Get mouse input for orbiting camera
    // YAW
    yaw += hInput * mouseSensitivity;
    yaw = ((yaw + 180) % 360) - 180; // Normalization of angle formula [-180, 180]
    // ALTERNATIVE
    //yaw = yaw % 360f; // Normalize angle [0, 360]
    //if (yaw > 180) yaw -= 360; // Clockwise (+), Normalize to -180 degress
    //else if (yaw < -180) yaw += 360; // Anti-clockwise (-), Normalize to +180 degree
    // PITCH
    pitch += vInput * mouseSensitivity;
    pitch = Mathf.Clamp(pitch, minPitch, maxPitch); // Limit pitch
    Debug.Log($"Look: x = {lookInput.x}, Y = {lookInput.y}");
  }

  void Look()
  {
    // Auto rotate (yaw) back if car is moving and no hInput
    if (actualSpeedFactor > 0.001f && hInput == 0f)
    {
      // Calculation of signed delta angle [0, 360]
      float yawDeltaAngle = 0f;
      if (actualSpeed >= 0f) // If accelerating forward
      {
        yawDeltaAngle = Mathf.DeltaAngle(yaw, car.rotation.eulerAngles.y);
      }
      else // If reversing backwards
      {
        yawDeltaAngle = Mathf.DeltaAngle(yaw, car.rotation.eulerAngles.y - 180);
      }

      // Calculation of auto yaw rotation [0, 360]
      yaw += yawDeltaAngle * actualSpeedFactor * autoRotateSensitivity;
      yaw = ((yaw + 180) % 360) - 180; // Normalization of angle formula [-180, 180]
    }

    // Calculate rotations (orbit around car)
    Quaternion targetRotation = Quaternion.Euler(pitch, yaw, 0);
    // Smoothly orbit around the car
    transform.rotation = Quaternion.Lerp(transform.rotation, targetRotation, orbitDamping);

    // Camera origin position using the car position with offset
    transform.position = car.position + new Vector3(offset.x, offset.y, 0);

    // Dynamic offset from the car
    float dynamicDistance = Mathf.Lerp(offset.z, offset.z - maxDynamicDistance, actualSpeedFactor);
    Vector3 camPos = new Vector3(0, 0, dynamicDistance); // Local camera position
    
    // Using raycast to detect and avoid camera collision (ignoring Car LayerMask)
    RaycastHit hit;
    Vector3 carToCam = transform.TransformPoint(camPos) - transform.position;
    if (Physics.Raycast(transform.position, carToCam.normalized, out hit, carToCam.magnitude, ~carLayerMask))
    {
      cam.position = Vector3.Lerp(cam.position, hit.point * 0.9f, collisionAvoidanceDamping); // 0.9f for 0.1f clearance

      // DEBUGGING
      Debug.Log($"Raycast hit: {hit.collider.name}");
    }
    else
    {
      cam.localPosition = Vector3.Lerp(cam.localPosition, camPos, collisionAvoidanceDamping);
    }
  }

  // Called by CollisionController on crash collision
  public void CollisionShake(float intensityFactor,
                             float minShakeIntensity,
                             float maxShakeIntensity,
                             float shakeDuration,
                             float shakeDecay)
  {
    // Scale shake intensity and duration
    float shakeIntensity = Mathf.Lerp(minShakeIntensity, maxShakeIntensity, intensityFactor);
    float shakeDurationIntensity = Mathf.Lerp(0, shakeDuration, intensityFactor);

    // Start camera shake for the specified duration
    StartCoroutine(Shake(shakeIntensity, shakeDurationIntensity, shakeDecay));
  }

  // For Coroutine called by CollisionShake()
  IEnumerator Shake(float intensity, float duration, float decay)
  {
    // Initialize local variable
    float elapsed = 0f;

    while (elapsed < duration)
    {
      float x = Random.Range(-1f, 1f) * intensity;
      float y = Random.Range(-1f, 1f) * intensity;
      cam.localPosition += new Vector3(x, y, 0f);

      // Add elapsed time in seconds
      elapsed += Time.deltaTime;
      // Decay for realistic fade
      intensity *= decay;

      // Allow to execute over multiple frames to avoid freezing
      yield return null;
    }

    // No need to reset camera position as Look() will do that
  }

  // Called by CarController to update data
  public void DataUpdate(float speed, float speedFactor)
  {
    actualSpeed = speed; // Signed
    actualSpeedFactor = speedFactor; // Unsigned
  }
}
