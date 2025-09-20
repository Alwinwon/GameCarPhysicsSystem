using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem.Android;

public class CarProperties : MonoBehaviour
{
  public enum Drivetrain { AWD, FWD, RWD } // selection between 3 types of drivetrains
  [Header("Motors & Brakes")]
  [Tooltip("Select type of drivetrain.")]
  public Drivetrain drivetrain = Drivetrain.AWD;
  [Tooltip("Max torque of motors (Nm).")]
  public float motorTorque = 2000f;
  [Tooltip("Max torque of brakes (Nm).")]
  public float brakeTorque = 2000f;
  [Tooltip("Max speed (km/h).")]
  public float maxSpeed = 250f;

  [Header("Wheels")]
  [Tooltip("Mass of wheel (kg).")]
  public float wheelMass = 20;
  [Tooltip("Radius of wheel (m).")]
  public float wheelRadius = 0.45f;
  [Tooltip("Max steering angle at rest.")]
  public float turnAngle = 30f;
  [Tooltip("Max steering angle at max speed.")]
  public float turnAngleAtMaxSpeed = 15f;
  [Tooltip("Tire friction factor.")]
  public float tireFriction = 1f;

  [Header("Suspensions")]
  [Tooltip("Maximum extension length of suspensions (m).")]
  public float suspensionDistance = 0.3f;
  [Tooltip("Spring constant, k (N/m)")]
  public float stiffness = 25000f;
  [Tooltip("Damping coefficient, c (Ns/m)")]
  public float dampening = 2500f;

  [Header("Chassis")]
  [Tooltip("Laden mass of the car (kg), excluding wheels")]
  public float chassisMass = 1834f;
  [Tooltip("Estimated total front area of the car in m2")]
  public float frontArea = 0;
  [Tooltip("Estimated total front area of the car in m2")]
  public float sideArea = 0;
  [Tooltip("Estimated total front area of the car in m2")]
  public float rearArea = 0;

  // Reference to the car's Rigidbody component
  Rigidbody rb;

  // Start is called before the first frame update
  void Start()
  {
    // Get the Rigidbody component attached to this GameObject.
    rb = GetComponent<Rigidbody>();
    // Ensure the Rigidbody exists.
    if (rb != null)
    {
      // Assign car's mass
      rb.mass = chassisMass;
    }
    else
    {
      Debug.Log("CarProperties: Rigidbody not found on this GameObject. Please add a Rigidbody.");
      return;
    }
  }

  // Update is called once per frame
  void Update()
  {
    
  }
}
