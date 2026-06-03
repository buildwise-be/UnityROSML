using System;
using UnityEngine;
using UnityEngine.InputSystem;

// (1) Make sure the namespace matches where your generated class lives.
//     In your snippet, the class is declared at global scope, so no namespace is needed.
//     If yours is wrapped in a namespace, add a corresponding `using`.
public class UR16eTriggerListener : MonoBehaviour
{
    // (2) Reference to the generated input‐system wrapper.
    private MagicLeapOpenXRInput _inputActions;

    // (3) Reference to the UR16eServiceCaller. Assign this in the Inspector,
    //     or obtain it at runtime (e.g. via GetComponent<>).
    [Tooltip("Drag your UR16eServiceCaller component here")]
    public Ur16EServiceCaller ur16eServiceCaller;

    private void Awake()
    {
        // (4) Instantiate the generated input‐asset wrapper.
        _inputActions = new MagicLeapOpenXRInput();
    }

    private void OnEnable()
    {
        // (5) Enable just the Controller‐action map (you could also do _inputActions.Enable() to turn on all maps).
        _inputActions.Controller.Enable();

        // (6) Subscribe to the Trigger action’s performed event.
        //     Whenever the trigger is pressed (performed), our callback runs.
        _inputActions.Controller.Trigger.performed += OnTriggerPerformed;
        _inputActions.Controller.Bumper.performed += OnBumperPerformed;
    }

    private void OnDisable()
    {
        // (7) Unsubscribe before disabling, to avoid multiple subscriptions or null references.
        _inputActions.Controller.Trigger.performed -= OnTriggerPerformed;
        _inputActions.Controller.Bumper.performed -= OnBumperPerformed;

        // (8) Disable the Controller map.
        _inputActions.Controller.Disable();
    }

    private void OnDestroy()
    {
        // (9) Properly clean up the generated asset when this MonoBehaviour is destroyed.
        //     This ensures no native‐object leaks in the Input System.
        _inputActions.Dispose();
    }

    // (10) This method is called every time the Trigger action is performed.
    private void OnTriggerPerformed(InputAction.CallbackContext context)
    {
        
    }

    private void OnBumperPerformed(InputAction.CallbackContext context)
    {
        // (11) Optionally check context.phase if you only want "performed" and not "started" or "canceled".
        if (context.phase == InputActionPhase.Performed)
        {
            if (ur16eServiceCaller != null)
            {
                ur16eServiceCaller.SendPlanSingleTargetServiceRequest();
            }
            else
            {
                Debug.LogWarning("UR16eServiceCaller reference is missing!");
            }
        }
    }
}
