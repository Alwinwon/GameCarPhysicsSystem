using UnityEngine;

public class CollisionController : MonoBehaviour
{
  [Header("Visual Impact Effects")]
  [Tooltip("Assign the Sparks prefab here.")]
  public ParticleSystem sparksPrefab;
  [Tooltip("Minimum spark particles for low Impact intensity.")]
  public float minSparkBurst = 10f;
  [Tooltip("Maximum spark particles for low Impact intensity.")]
  public float maxSparkBurst = 100f;

  [Header("Camera Impact Effects")]
  [Tooltip("Minimum camera shake for low Impact intensity.")]
  public float minShakeIntensity = 0f;
  [Tooltip("Maximum camera shake for high Impact intensity.")]
  public float maxShakeIntensity = 2f;
  [Tooltip("Camera shake time (s).")]
  public float shakeDuration = 1f;
  [Tooltip("Camera shake fading (Higher value, more aggressive).")]
  public float shakeDecay = 0.5f;

  // For Components
  CarController carController;

  // For Variables
  float maxSpeed;

  // Start is called once before the first execution of Update after the MonoBehaviour is created
  void Start()
  {
    // Ensure variables are assigned
    if (!sparksPrefab)
    {
      Debug.LogError("Sparks Prefab not assigned to CollisionController!");
      return;
    }

    // Check inputs
    if (minSparkBurst > maxSparkBurst)
    {
      Debug.LogError("Min Spark Burst is larger than Max Spark Burst! Check inputs again.");
      return;
    }
    else if (minShakeIntensity > maxShakeIntensity)
    {
      Debug.LogError("Min Shake Intensity is larger than Max Shake Intensity! Check inputs again.");
      return;
    }

    // Get component attached to the GameObject
    carController = GetComponent<CarController>();
  }

  // Crash too sensitive! Adjust to lower sensitivity!
  // OnCollisionEnter is called on the first frame of collision
  void OnCollisionEnter(Collision collision)
  {
    float collisionSpeed = collision.relativeVelocity.magnitude;
    // Normalize intensity factor (m/s to km/h by multiplying 3600/1000)
    float intensityFactor = Mathf.InverseLerp(0, maxSpeed, Mathf.Abs(collisionSpeed) * 3.6f);

    //if (collision.gameObject.CompareTag("Obstacle"))
    //{
      // Spawn the sparks
      ParticleSystem sparks = Instantiate(sparksPrefab,
                                          collision.contacts[0].point,
                                          Quaternion.LookRotation(collision.contacts[0].normal));

      // Get the current particle system's emission settings
      var emission = sparks.emission;
      // Modify the burst particles in relation to Impact force
      emission.SetBurst(0, new ParticleSystem.Burst(0f, Mathf.Lerp(minSparkBurst, maxSparkBurst, intensityFactor)));
      // Trigger sparks at contact point
      sparks.Play();
      // Clean up after duration
      Destroy(sparks.gameObject, sparks.main.duration);

      // Trigger camera shake from CameraController
      // (?) Everything connects to CarController for Organization
      carController.camController.CollisionShake(intensityFactor,
                                                 minShakeIntensity,
                                                 maxShakeIntensity,
                                                 shakeDuration,
                                                 shakeDecay);
      // Trigger Impact collision audio from AudioController
      // (?) Everything connects to CarController for Organization
      carController.audioController.ImpactCollision(intensityFactor);
    //}
  }

  // Called by CarController to update data
  public void DataUpdate(float speed)
  {
    maxSpeed = speed;
  }
}
