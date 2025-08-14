using UnityEngine;
using UnityEngine.InputSystem;

public class Car : MonoBehaviour {

    [SerializeField] Camera spectator;
    [SerializeField] Vector3 spectatorHomePosition, spectatorHomeAngles;
    [SerializeField] Vector3 maxSpectatorPositionDisplacement;

    [SerializeField] Transform body;
    [SerializeField] float maxBodyTilt;

    [SerializeField] Transform wheelFL, wheelFR, wheelBL, wheelBR;
    [SerializeField] float turnSpeed, maxTurnAngle, turningSpeedReduction;
    public float turnAngle;
    public float turnRate;

    [SerializeField] float acceleration, maxSpeed, brakeSpeed;
    public float speed;
    float deceleration;

    Rigidbody rb;

    InputAction accelerateAction;
    InputAction steerLeftAction;
    InputAction steerRightAction;
    InputAction brakeAction;

    void Start() {
        accelerateAction = InputSystem.actions.FindAction("Accelerate");
        steerLeftAction = InputSystem.actions.FindAction("Steer Left");
        steerRightAction = InputSystem.actions.FindAction("Steer Right");
        brakeAction = InputSystem.actions.FindAction("Brake");

        spectator.transform.localPosition = spectatorHomePosition;
        spectator.transform.eulerAngles = spectatorHomeAngles;
        speed = 0.0f;
        deceleration = maxSpeed / (maxSpeed + acceleration);
        turnAngle = 0.0f;
        turnRate = 0.0f;
        rb = GetComponent<Rigidbody>();
    }
    
    void Update() {
        
    }

    void FixedUpdate() {
        float rearContactMultiplier = 0.0f;
        if (wheelBL.GetComponent<Wheel>().isTouchingGround) rearContactMultiplier += 0.5f;
        if (wheelBR.GetComponent<Wheel>().isTouchingGround) rearContactMultiplier += 0.5f;
        float frontContactMultiplier = 0.0f;
        if (wheelBL.GetComponent<Wheel>().isTouchingGround && wheelBR.GetComponent<Wheel>().isTouchingGround) frontContactMultiplier = 1.0f;

        if (accelerateAction.IsPressed()) speed = Mathf.Min(speed + acceleration * rearContactMultiplier * Time.fixedDeltaTime, maxSpeed);
        if (steerLeftAction.IsPressed()) turnAngle = Mathf.Max(-maxTurnAngle, turnAngle - turnSpeed * Time.fixedDeltaTime);
        else if (turnAngle < 0.0f) turnAngle = Mathf.Min(turnAngle + turnSpeed * Time.fixedDeltaTime, 0.0f);
        if (steerRightAction.IsPressed()) turnAngle = Mathf.Min(turnAngle + turnSpeed * Time.fixedDeltaTime, maxTurnAngle);
        else if (turnAngle > 0.0f) turnAngle = Mathf.Max(0.0f, turnAngle - turnSpeed * Time.fixedDeltaTime);
        if (brakeAction.IsPressed()) speed *= Mathf.Pow(brakeSpeed, Time.fixedDeltaTime);

        speed *= Mathf.Pow(deceleration, Time.fixedDeltaTime);
        speed *= Mathf.Pow(1.0f + (turningSpeedReduction - 1.0f) * Mathf.Abs(turnAngle / maxTurnAngle), Time.fixedDeltaTime);
        if (Mathf.Abs(speed) < 0.1f) speed = 0.0f;

        rb.MovePosition(transform.position + transform.forward * speed * Time.fixedDeltaTime);

        wheelFL.transform.localRotation = Quaternion.Euler(Vector3.up * turnAngle);
        wheelFR.transform.localRotation = Quaternion.Euler(Vector3.up * turnAngle);
        turnRate = 32.0f * Mathf.PI * speed * turnAngle * frontContactMultiplier / (360.0f * (wheelFL.transform.localPosition.z - wheelBL.transform.localPosition.z));
        rb.MoveRotation(transform.rotation * Quaternion.Euler(-transform.up * Mathf.Deg2Rad * turnRate));

        spectator.transform.localPosition = spectatorHomePosition + new Vector3(maxSpectatorPositionDisplacement.x * (turnAngle / maxTurnAngle), 0.0f, maxSpectatorPositionDisplacement.z * (speed / maxSpeed));
    }

}
