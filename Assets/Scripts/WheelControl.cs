using UnityEngine;

public class WheelControl : MonoBehaviour
{
  public Transform wheelModel;

  [HideInInspector] public WheelCollider WheelCollider;

  // Create properties for the CarControl script
  // (You should enable/disable these via the 
  // Editor Inspector window)
  public bool motorized;
  public bool steerable;
  public bool flipped;

  Vector3 position;
  Quaternion rotation;

  // Start is called once before the first execution of Update after the MonoBehaviour is created
  void Start()
  {
    WheelCollider = GetComponent<WheelCollider>();
  }

  // Update is called once per frame
  void Update()
  {
    // Get the Wheel collider's world pose values and
    // use them to set the wheel model's position and rotation
    WheelCollider.GetWorldPose(out position, out rotation);
    wheelModel.transform.position = position;

    if (flipped) // If the wheel model is flipped, rotate it in the inverse
    {
      wheelModel.transform.rotation = Quaternion.Euler(rotation.eulerAngles.x,
                                                       rotation.eulerAngles.y,
                                                       rotation.eulerAngles.z + 180);
    }
    else
    {
      wheelModel.transform.rotation = rotation;
    }
  }
}
