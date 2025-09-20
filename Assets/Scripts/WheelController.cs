using UnityEngine;

public class WheelController : MonoBehaviour
{
  [Header("Wheel Visuals")]
  [Tooltip("Assign the Wheel transform here.")]
  public Transform wheel;
  [Tooltip("Assign the SkidSmoke particle system here.")]
  public ParticleSystem skidSmoke;
  [Tooltip("Assign the SkidTrail trail renderer here.")]
  public TrailRenderer skidTrail;

  [Header("Wheel Properties")] // Properties for the CarController script
  [Tooltip("Wheel ID correspond to AudioController.")]
  public int wheelID;
  [Tooltip("Select if the wheel was motorized.")]
  public bool motorized;
  [Tooltip("Select if wheel can be used to steer.")]
  public bool steerable;
  [Tooltip("Select to flip the wheel model.")]
  public bool flipped;
  [Tooltip("Slip allowance before skid smoke appears (Lower value, more sensitive trigger).")]
  public float slipAllowance = 0.5f;

  // For Components
  [HideInInspector] public WheelCollider wheelCollider;
  CarController carController;

  // For Variables
  float actualSpeed;

  // Initialize is called once by CarController to ensure the variable is initialized before calling it
  public void Initialize()
  {
    // Get component attached to the GameObject
    wheelCollider = GetComponent<WheelCollider>();
    carController = GetComponentInParent<CarController>();
  }

  // Start is called once before the first execution of Update after the MonoBehaviour is created
  void Start()
  {
    // Ensure variables are assigned
    if (!wheel || !skidSmoke || !skidTrail)
    {
      Debug.LogError("Wheel, Skid Smoke and/or Skid Trail not assigned to WheelController!");
      return;
    }
  }

  // Update is called once per frame
  void Update()
  {
    AnimateWheel();
    AnimateSkid();
  }

  void AnimateWheel()
  {
    // Get the Wheel collider's world pose values and use it to set the wheel model's position and rotation
    wheelCollider.GetWorldPose(out Vector3 position, out Quaternion rotation);
    wheel.position = position;

    // If the wheel model is flipped, rotate it in the inverse
    if (flipped)
    {
      wheel.rotation = Quaternion.Euler(rotation.eulerAngles.x,
                                        rotation.eulerAngles.y,
                                        rotation.eulerAngles.z + 180);
    }
    else
    {
      wheel.rotation = rotation;
    }
  }

  void AnimateSkid()
  {
    // Calculate current slip from wheel collider's ground hit
    wheelCollider.GetGroundHit(out WheelHit hit);
    float slip = Mathf.Abs(hit.forwardSlip) + Mathf.Abs(hit.sidewaysSlip);

    // DEBUGGING
    //Debug.Log($"Slip: {slip}");

    if (slip > slipAllowance)
    {
      // SKID SMOKE
      // Get the current particle system's emission settings
      var emission = skidSmoke.emission;
      // Modify the number of particle over a distance (More Slipping = Heavier Smoke)
      emission.rateOverDistance = 3 * slip;
      // Use-case: Driving against a wall creates lot of skid smoke
      if (actualSpeed < 0.1f) emission.rateOverTime = 6 * slip;
      else emission.rateOverTime = 0f;
      // Trigger skid smoke
      if (!skidSmoke.isPlaying) skidSmoke.Play();

      // SKID TRAIL
      // Activate skid trial
      if (!skidTrail.emitting) skidTrail.emitting = true;

      // SKID AUDIO
      // Trigger skid audio from AudioController
      carController.audioController.TireScreech(wheelID, slip);
    }
    else
    {
      // SKID SMOKE
      if (skidSmoke.isPlaying) skidSmoke.Stop();

      // SKID TRAIL
      if (skidTrail.emitting) skidTrail.emitting = false;

      // SKID AUDIO
      carController.audioController.TireScreech(wheelID, 0);
    }
  }

  // Called by CarController to update data
  public void DataUpdate(float speed)
  {
    actualSpeed = speed;
  }
}
