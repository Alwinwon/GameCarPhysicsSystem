using UnityEngine;
using TMPro;

public class CarGUI : MonoBehaviour
{
  public TextMeshProUGUI speedometer;
  public TextMeshProUGUI actualSpeed;
  public TextMeshProUGUI carStats;
  public TextMeshProUGUI[] wheelStats;
  public GameObject statsGUI;
  public float speedometerSize = 72f;
  public float headerSize = 25f;
  public float textSize = 20f;

  // Display speedometer based on wheel's RPM
  public void Speedometer(float speedometerSpeed)
  {
    speedometer.text = $"<size={speedometerSize}><b>{Mathf.Abs(speedometerSpeed):F0}</b></size>\n" +
                       $"<size={speedometerSize/2}>KM/H</size>";
    //speedometer.text = Mathf.Abs(speedometerSpeed).ToString("F0");
  }

  // Display stats of car
  public void CarStats(float currentFrontMotorRPM,
                       float currentRearMotorRPM,
                       float currentFrontMotorTorque,
                       float currentRearMotorTorque,
                       float speedFactor,
                       float actualSpeedFactor,
                       float powerFactor,
                       float torqueFactor,
                       float dragMagnitude,
                       float frontliftMagnitude,
                       float rearLiftMagnitude,
                       float speed)
  {
    carStats.text = $"<size={headerSize}><color=white>Car</color></size>\n\n" +
                    $"<size={textSize}><color=white><b>Motors</b></color></size>\n" +
                    $"<size={textSize}><color=yellow>Front Motor RPM: {currentFrontMotorRPM:F1}</color></size>\n" +
                    $"<size={textSize}><color=yellow>Front Motor Torque: {currentFrontMotorTorque:F1} Nm</color></size>\n" +
                    $"<size={textSize}><color=yellow>Rear Motor RPM: {currentRearMotorRPM:F1}</color></size>\n" +
                    $"<size={textSize}><color=yellow>Rear Motor Torque: {currentRearMotorTorque:F1} Nm</color></size>\n" +
                    $"<size={textSize}><color=yellow>Speedometer Speed Factor: {speedFactor:F2}</color></size>\n" +
                    $"<size={textSize}><color=yellow>Actual Speed Factor: {actualSpeedFactor:F2}</color></size>\n" +
                    $"<size={textSize}><color=yellow>Power Factor: {powerFactor:F2}</color></size>\n" +
                    $"<size={textSize}><color=yellow>Torque Factor: {torqueFactor:F2}</color></size>\n" +
                    $"<size={textSize}><color=white><b>Aerodynamics</b></color></size>\n" +
                    $"<size={textSize}><color=blue>Drag Force: {dragMagnitude:F2} N</color></size>\n" +
                    $"<size={textSize}><color=#00ffffff>Front Lift/Downforce: {frontliftMagnitude:F2} N</color></size>\n" +
                    $"<size={textSize}><color=#00ffffff>Rear Lift/Downforce: {rearLiftMagnitude:F2} N</color></size>";

    actualSpeed.text = $"<size={textSize}>Actual Speed:          KM/H</size>\n" +
                       $"<size={speedometerSize}><b>{Mathf.Abs(speed):F0}</b></size>";
  }

  // Display stats of each wheel
  public void WheelStats(WheelController[] wheels,
                         float[] suspensionDisplacement,
                         float[] rollingResistanceMagnitude)
  {
    if (wheelStats.Length == wheels.Length)
    {
      for (int i = 0; i < wheels.Length; i++)
      {
        wheelStats[i].text = $"<size={headerSize}><color=white>{wheels[i].wheelName} Wheel</color></size>\n\n" +
                             $"<size={textSize}><color=white><b>Wheel & Suspension</b></color></size>\n" +
                             $"<size={textSize}><color=yellow>RPM: {wheels[i].wheelCollider.rpm:F1}</color></size>\n" +
                             $"<size={textSize}><color=green>Torque: {wheels[i].wheelCollider.motorTorque:F1} Nm</color></size>\n" +
                             $"<size={textSize}><color=orange>Total Slip: {wheels[i].slip:F2}</color></size>\n" +
                             $"<size={textSize}><color=yellow>Suspension Displacement: {suspensionDisplacement[i]*1000f:F0} mm</color></size>\n" +
                             $"<size={textSize}><color=white><b>Aerodynamics</b></color></size>\n" +
                             $"<size={textSize}><color=red>Rolling Resistance: {rollingResistanceMagnitude[i]:F2} N</color></size>";
      }
    }
    else
    {
      Debug.LogError($"CarGUI: Incorrect number of wheel stats displays assigned. There should be {wheels.Length} numbers.");
      return;
    }
  }

  public void StatsToggle()
  {
    statsGUI.SetActive(!statsGUI.activeSelf);
  }
}
