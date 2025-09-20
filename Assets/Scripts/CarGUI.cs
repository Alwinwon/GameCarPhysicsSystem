using System.Transactions;
using TMPro;
using UnityEngine;

public class CarGUI : MonoBehaviour
{
  public TextMeshProUGUI speedometer;
  public TextMeshProUGUI chassisStats;
  public TextMeshProUGUI[] wheelStats;
  public float headerSize = 40f;
  public float textSize = 36f;

  // Start is called once before the first execution of Update after the MonoBehaviour is created
  void Start()
  {
    
  }

  // Update is called once per frame
  void Update()
  {
    
  }

  public void Speedometer(float speed)
  {
    speedometer.text = Mathf.Abs(speed).ToString("F0");
  }

  public void WheelStats(float[] wheelRPM, float[] motorTorquePerWheel)
  {
    if (wheelStats.Length == wheelRPM.Length)
    {
      //do something
      foreach (var wheelStat in wheelStats)
      {

      }
    }
    else
    {
      Debug.LogError($"CarGUI: Incorrect number of wheel stats displays assigned. There should be {wheelRPM.Length} numbers.");
      return;
    }
  }
}
