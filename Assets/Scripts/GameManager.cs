using System;
using Unity.VisualScripting;
using UnityEngine;

public class GameManager : MonoBehaviour
{
  public static GameManager Instance;
  public enum GameState
  {
    main,
    editor,
    openMode,
    raceMode
  }
  public GameState state;
  //public static event Action<GameState> OnStateChange;

  void Awake()
  {
    Instance = this;
  }


  // Start is called once before the first execution of Update after the MonoBehaviour is created
  void Start()
  {
    SetGameState(GameState.main);
  }

  // Update is called once per frame
  void Update()
  {
    
  }

  public void SetGameState(GameState newState)
  {
    state = newState;

    switch(newState)
    {
      case GameState.main:
        Main();
        break;
      case GameState.editor:
        Editor();
        break;
      case GameState.openMode:
        break;
      case GameState.raceMode:
        break;
      default:
        throw new ArgumentOutOfRangeException(nameof(newState), newState, null);
    }

    // Check if the GameState is subscribed before involing this function
    //OnStateChange?.Invoke(newState);
  }

  void Main()
  {

  }

  void Editor()
  {

  }

  void OpenMode()
  {

  }

  void RaceMode()
  {

  }

  public GameState GetCurrentState()
  {
    return state;
  }
}