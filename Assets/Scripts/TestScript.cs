using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TestScript : MonoBehaviour
{
  // Start is called before the first frame update
  void Start()
  {
    
  }

  // Update is called once per frame
  void Update()
  {
    Vector3 rayOrigin = new Vector3(transform.position.x, transform.position.y, transform.position.z);
    Debug.DrawRay(rayOrigin, transform.up, Color.red);
  }
}
