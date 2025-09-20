using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.Rendering;

public class AudioController : MonoBehaviour
{
  [Header("Audio Sources - Car")]
  [Tooltip("Assign the MotorHum audio source here.")]
  public AudioSource motorHum;
  [Tooltip("Assign the horn audio source here.")]
  public AudioSource horn;
  [Tooltip("Assign the crach collision audio source here for all wheels.")]
  public AudioSource crashCollision;

  [Header("Audio Sources - Wheel(s)")]
  [Tooltip("Assign the roadNoise(s) audio sources here for all wheels " +
           "(Ensure Element number align with Wheel ID).")]
  public AudioSource[] roadNoise;
  [Tooltip("Assign the TireScreech(es) audio source here for all wheels " +
           "(Ensure Element number align with Wheel ID).")]
  public AudioSource[] tireScreech;

  [Header("Audio Settings - Motor Hum")]
  [Tooltip("Minimum motor pitch at rest.")]
  public float minMotorPitch = 0.1f;
  [Tooltip("Maximum motor pitch at maximum speed.")]
  public float maxMotorPitch = 1.0f;
  [Tooltip("Damping of pitch (Lower value, higher damping).")]
  public float motorPitchDamping = 0.1f;
  [Tooltip("Maximum motor volume at maximum power.")]
  public float maxMotorVolume = 0.5f;
  [Tooltip("Damping of volume (Lower value, higher damping).")]
  public float motorVolumeDamping = 0.1f;

  [Header("Audio Settings - Horn")]
  [Tooltip("Maximum horn volume.")]
  public float maxHornVolume = 0.4f;
  [Tooltip("Damping of volume (Lower value, higher damping).")]
  public float HornVolumeDamping = 0.95f;

  [Header("Audio Settings - Road Noise")]
  [Tooltip("Maximum road noise tempo at maximum speed.")]
  public float maxRoadNoiseSpeed = 1.0f;
  [Tooltip("Maximum road noise volume.")]
  public float maxRoadNoiseVolume = 1.0f;

  [Header("Audio Settings - Tire Screech")]
  [Tooltip("Maximum tire screech volume at maximum slip.")]
  public float maxTireScreechVolume = 0.05f;

  [Header("Audio Settings - Crash Collision")]
  [Tooltip("Maximum crash collision volume.")]
  public float maxCrashCollisionVolume = 0.4f;

  // For Components
  CarController carController;

  // For Variables
  float speedFactor;
  float motorTorque;
  float moveInput;
  float hornInput;


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
    //if (!motorHum || !roadNoise || !horn)
    //{
    //  Debug.LogError("AudioSources not assigned to AudioController!");
    //  return;
    //}

    // Check inputs
    if (minMotorPitch > maxMotorPitch)
    {
      Debug.LogError("Min Motor Pitch is larger than Max Motor Pitch! Check inputs again.");
      return;
    }

    // Get component attached to the GameObject
    carController = GetComponent<CarController>();
    motorHum = GetComponent<AudioSource>();
  }

  // Update is called once per frame
  void Update()
  {
    SoundInput();
    MotorHum();
    Horn();
    RoadNoise();
  }

  void SoundInput()
  {
    // Read the Axis 1D input from the Input System
    moveInput = carControls.Car.Movement.ReadValue<float>();

    // Read the button input from the Input System
    hornInput = carControls.Car.Horn.ReadValue<float>();
  }

  void MotorHum()
  {
    // AUDIO CONTROL PARAMETERS
    // Use-case: Driving against a wall with slightly higher pitch to indicate motor struggle
    float currentMinMotorPitch = 0;
    if (speedFactor < 0.001f && moveInput != 0)
    {
      currentMinMotorPitch = minMotorPitch + (maxMotorPitch - minMotorPitch) * 0.1f;
    }
    // Use-case: Make slightly lower motor noise when coasting in motion without active torque
    float currentMinMotorVolume = 0;
    if (speedFactor > 0.001f)
    {
      currentMinMotorVolume = maxMotorVolume * 0.8f;
    }

    // PITCH
    // Modify pitch as the car accelerates
    float targetPitch = Mathf.Lerp(currentMinMotorPitch, maxMotorPitch, speedFactor);
    motorHum.pitch = Mathf.Lerp(motorHum.pitch, targetPitch, motorPitchDamping);

    // VOLUME
    // Modify volume as the car powers
    float targetVolume = Mathf.Lerp(currentMinMotorVolume, maxMotorVolume, motorTorque);
    motorHum.volume = Mathf.Lerp(motorHum.volume, targetVolume, motorVolumeDamping);

    // DEBUGGING
    //Debug.Log($"Min Motor Pitch: {currentMinMotorPitch}, Min Motor Volume: {currentMinMotorVolume}, SpeedFactor: {speedFactor}");
  }

  void Horn()
  {
    float targetVolume;
    
    if (hornInput != 0)
    {
      targetVolume = maxHornVolume;
    }
    else
    {
      targetVolume = 0;
    }

    horn.volume = Mathf.Lerp(horn.volume, targetVolume, HornVolumeDamping);
  }

  void RoadNoise()
  {
    foreach (var noise in roadNoise)
    {
      // PITCH
      // Modify pitch to indicate amount of road noise speed
      float targetPitch = maxRoadNoiseSpeed * speedFactor;
      noise.pitch = targetPitch;
      // Adjust the Pitch Shifter's pitch to counteract the AudioSource's pitch change
      // (?) This maintains the original pitch while changing tempo
      // (!) "Pitch" is the exposed parameter name (Master -> Pitch Shifter)
      noise.outputAudioMixerGroup.audioMixer.SetFloat("Pitch", 1f / (targetPitch * 2f));

      // VOLUME
      noise.volume = maxRoadNoiseVolume;
    }
  }

  // Called by WheelController on skidding
  public void TireScreech(int wheelID, float slip)
  {
    tireScreech[wheelID].volume = Mathf.Clamp(maxTireScreechVolume * slip,
                                              0,
                                              maxTireScreechVolume); // Slip sometimes exceed 1f
  }

  // Called by CollisionController on crash collision
  public void ImpactCollision(float intensityFactor)
  {
    crashCollision.volume = maxCrashCollisionVolume * intensityFactor;
    crashCollision.Play();
  }

  // Called by CarController to update data
  public void DataUpdate(float speed, float torque)
  {
    speedFactor = speed;
    motorTorque = torque;
  }
}
