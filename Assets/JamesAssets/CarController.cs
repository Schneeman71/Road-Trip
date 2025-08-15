using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem; // New Input System
#endif

[RequireComponent(typeof(Rigidbody))]
public class CarController : MonoBehaviour
{
    [Header("References")]
    public Transform centerOfMass; // assign your COM object here

    [System.Serializable]
    public class Axle
    {
        public WheelCollider leftCollider;
        public WheelCollider rightCollider;
        public Transform leftMesh;
        public Transform rightMesh;

        [Tooltip("If true, this axle steers (usually front)")]
        public bool steer;

        [Tooltip("If true, this axle is motorized (FWD/RWD/AWD)")]
        public bool motor;
    }

    public Axle frontAxle;
    public Axle rearAxle;

    [Header("Driving")]
    [Tooltip("Max steer angle in degrees at low speeds")]
    public float maxSteerAngle = 30f;

    [Tooltip("Reduces steer angle as speed increases (x=speed m/s, y=multiplier)")]
    public AnimationCurve steerVsSpeed = AnimationCurve.EaseInOut(0, 1f, 30, 0.5f);

    [Tooltip("Maximum continuous motor torque per motorized wheel (Nm)")]
    public float maxMotorTorque = 2000f;

    [Tooltip("Reverse torque per motorized wheel (Nm)")]
    public float reverseTorque = 1500f;

    [Tooltip("0–1. How quickly throttle ramps toward target")]
    [Range(0f, 1f)] public float throttleResponse = 0.2f;

    [Tooltip("Top speed in km/h (soft limit via torque fade)")]
    public float topSpeedKPH = 160f;

    [Tooltip("Top reverse speed in km/h (soft limit)")]
    public float topReverseKPH = 40f;

    [Tooltip("Brake torque when braking")]
    public float maxBrakeTorque = 4000f;

    [Header("Handling")]
    [Tooltip("Pushes the car down with speed to improve grip")]
    public float downforceCoef = 30f;

    [Tooltip("Anti-roll bar strength for each axle")]
    public float antiRollStrengthFront = 8000f;
    public float antiRollStrengthRear  = 8000f;

    [Tooltip("Limit slip by reducing torque when wheels spin")]
    public bool tractionControl = true;
    [Range(0f, 1f)] public float slipLimit = 0.4f;
    [Range(0f, 1f)] public float tractionTorqueCut = 0.4f;

    [Header("WheelCollider Substeps (helps rough terrain)")]
    [Tooltip("Speed threshold (m/s) to switch substep counts")]
    public float substepSpeedThreshold = 10f;
    [Tooltip("Solver substeps when below threshold")]
    public int substepsBelow = 7;
    [Tooltip("Solver substeps when above threshold")]
    public int substepsAbove = 3;

    [Header("Legacy Input (used only if ENABLE_LEGACY_INPUT_MANAGER)")]
    public string steerAxis = "Horizontal";
    public string throttleAxis = "Vertical";
    public KeyCode handbrakeKey = KeyCode.Space;

#if ENABLE_INPUT_SYSTEM
    // New Input System actions (created in code so you don't need an .inputactions asset)
    private InputAction steerAction;     // -1..1
    private InputAction moveAction;      // -1..1 (forward/back)
    private InputAction handbrakeAction; // button
#endif

    private Rigidbody rb;
    private float steerInput;
    private float throttleInput;   // smoothed 0..1 (forward)
    private float brakeInput;      // 0..1
    private float lastRawMove;     // -1..1 (for reverse logic)

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        if (centerOfMass)
            rb.centerOfMass = transform.InverseTransformPoint(centerOfMass.position);

        rb.maxAngularVelocity = 20f; // keep crashes from spinning into infinity

        // Configure solver substeps (smoother terrain behavior)
        if (frontAxle.leftCollider)  ConfigureSubsteps(frontAxle.leftCollider);
        if (frontAxle.rightCollider) ConfigureSubsteps(frontAxle.rightCollider);
        if (rearAxle.leftCollider)   ConfigureSubsteps(rearAxle.leftCollider);
        if (rearAxle.rightCollider)  ConfigureSubsteps(rearAxle.rightCollider);
    }

    void ConfigureSubsteps(WheelCollider wc)
    {
        wc.ConfigureVehicleSubsteps(substepSpeedThreshold, substepsBelow, substepsAbove);
    }

#if ENABLE_INPUT_SYSTEM
    void OnEnable()
    {
        // STEER: A/D, Left/Right arrows, Gamepad left stick X
        steerAction = new InputAction("Steer", InputActionType.Value);
        steerAction.AddCompositeBinding("1DAxis")
            .With("negative", "<Keyboard>/a")
            .With("positive", "<Keyboard>/d");
        steerAction.AddCompositeBinding("1DAxis")
            .With("negative", "<Keyboard>/leftArrow")
            .With("positive", "<Keyboard>/rightArrow");
        steerAction.AddBinding("<Gamepad>/leftStick/x");

        // MOVE (forward/back): W/S, Up/Down, RT (+) / LT (-)
        moveAction = new InputAction("Move", InputActionType.Value);
        moveAction.AddCompositeBinding("1DAxis")
            .With("negative", "<Keyboard>/s")
            .With("positive", "<Keyboard>/w");
        moveAction.AddCompositeBinding("1DAxis")
            .With("negative", "<Keyboard>/downArrow")
            .With("positive", "<Keyboard>/upArrow");
        moveAction.AddCompositeBinding("1DAxis")
            .With("negative", "<Gamepad>/leftTrigger")
            .With("positive", "<Gamepad>/rightTrigger");

        // Handbrake: Space, Gamepad south button (A/Cross)
        handbrakeAction = new InputAction("Handbrake", InputActionType.Button);
        handbrakeAction.AddBinding("<Keyboard>/space");
        handbrakeAction.AddBinding("<Gamepad>/buttonSouth");

        steerAction.Enable();
        moveAction.Enable();
        handbrakeAction.Enable();
    }

    void OnDisable()
    {
        steerAction?.Disable();
        moveAction?.Disable();
        handbrakeAction?.Disable();
    }
#endif

    void Update()
    {
        float rawSteer, rawMove;
        bool hbPressed;

#if ENABLE_INPUT_SYSTEM
        rawSteer  = steerAction.ReadValue<float>();
        rawMove   = moveAction.ReadValue<float>(); // -1..1
        hbPressed = handbrakeAction.IsPressed();
#else
        // Fallback only if the old Input Manager is enabled
        rawSteer  = Input.GetAxis(steerAxis);
        rawMove   = Input.GetAxis(throttleAxis);   // -1..1
        hbPressed = Input.GetKey(handbrakeKey);
#endif

        // Split move into accel/brake intents
        float accel = Mathf.Clamp01(rawMove);     // 0..1 (forward)
        float rawBrake = Mathf.Clamp01(-rawMove); // 0..1 (backward on the stick/keys)

        // Smooth throttle (accel only; braking is immediate)
        throttleInput = Mathf.Lerp(
            throttleInput,
            accel,
            1f - Mathf.Pow(1f - throttleResponse, Time.deltaTime * 60f)
        );

        steerInput = rawSteer;

        // Brake input (we may zero this in FixedUpdate when going into reverse)
        brakeInput = rawBrake;
        if (hbPressed) brakeInput = 1f; // handbrake overrides

        lastRawMove = rawMove;

        UpdateWheelVisuals(frontAxle);
        UpdateWheelVisuals(rearAxle);
    }

    void FixedUpdate()
    {
        float speedMS = rb.linearVelocity.magnitude;
        float speedKPH = speedMS * 3.6f;

        // Speed-sensitive steering
        float steerScale = steerVsSpeed.Evaluate(speedMS);
        float steerAngle = maxSteerAngle * steerInput * steerScale;

        ApplySteer(frontAxle, steerAngle);
        ApplySteer(rearAxle, 0f);

        // Determine forward/back motion intent
        float forwardSpeed = Vector3.Dot(rb.linearVelocity, transform.forward); // + forward, - backward

        // Are we allowed to go into reverse now?
        bool nearStopped = Mathf.Abs(forwardSpeed) < 1.0f; // m/s
        bool wantsReverse = lastRawMove < 0f && (nearStopped || forwardSpeed < 0f);

        // If we’re using reverse, don’t simultaneously brake from the axis
        float effectiveBrake = wantsReverse ? 0f : brakeInput;

        // Compute available drive torque (forward or reverse)
        float driveTorque = ComputeDriveTorque(speedKPH, wantsReverse);

        ApplyDrive(frontAxle, driveTorque);
        ApplyDrive(rearAxle,  driveTorque);

        ApplyBrakes(frontAxle, maxBrakeTorque * effectiveBrake);
        ApplyBrakes(rearAxle,  maxBrakeTorque * effectiveBrake);

        // Anti-roll bars
        ApplyAntiRoll(frontAxle, antiRollStrengthFront);
        ApplyAntiRoll(rearAxle,  antiRollStrengthRear);

        // Downforce (improves grip at speed)
        rb.AddForce(-transform.up * speedMS * downforceCoef);

        // Basic traction control: reduce torque when slipping
        if (tractionControl)
        {
            TractionAdjust(frontAxle, ref driveTorque);
            TractionAdjust(rearAxle,  ref driveTorque);
        }
    }

    float ComputeDriveTorque(float speedKPH, bool reverse)
    {
        if (!reverse)
        {
            float fade = Mathf.Clamp01(1f - (speedKPH / Mathf.Max(1f, topSpeedKPH)));
            float baseTorque = maxMotorTorque * fade;
            return baseTorque * throttleInput; // 0..+
        }
        else
        {
            float fade = Mathf.Clamp01(1f - (speedKPH / Mathf.Max(1f, topReverseKPH)));
            float reverseCmd = Mathf.Clamp01(-lastRawMove); // 0..1
            float baseTorque = reverseTorque * fade;
            return -baseTorque * reverseCmd; // negative = reverse
        }
    }

    void ApplySteer(Axle axle, float steerAngle)
    {
        if (!axle.steer) return;
        if (axle.leftCollider)  axle.leftCollider.steerAngle  = steerAngle;
        if (axle.rightCollider) axle.rightCollider.steerAngle = steerAngle;
    }

    void ApplyDrive(Axle axle, float torque)
    {
        if (!axle.motor) return;
        if (axle.leftCollider)  axle.leftCollider.motorTorque  = torque;
        if (axle.rightCollider) axle.rightCollider.motorTorque = torque;
    }

    void ApplyBrakes(Axle axle, float brakeTorque)
    {
        if (axle.leftCollider)  axle.leftCollider.brakeTorque  = brakeTorque;
        if (axle.rightCollider) axle.rightCollider.brakeTorque = brakeTorque;
    }

    void ApplyAntiRoll(Axle axle, float strength)
    {
        if (axle.leftCollider == null || axle.rightCollider == null) return;

        // Measure suspension compression on each side
        WheelHit hit;
        float travelL = 1f;
        float travelR = 1f;

        bool groundedL = axle.leftCollider.GetGroundHit(out hit);
        if (groundedL)
        {
            travelL = (-axle.leftCollider.transform.InverseTransformPoint(hit.point).y - axle.leftCollider.radius)
                      / axle.leftCollider.suspensionDistance;
        }

        bool groundedR = axle.rightCollider.GetGroundHit(out hit);
        if (groundedR)
        {
            travelR = (-axle.rightCollider.transform.InverseTransformPoint(hit.point).y - axle.rightCollider.radius)
                      / axle.rightCollider.suspensionDistance;
        }

        float antiRollForce = (travelL - travelR) * strength;

        if (groundedL) rb.AddForceAtPosition(
            axle.leftCollider.transform.up * -antiRollForce,
            axle.leftCollider.transform.position
        );
        if (groundedR) rb.AddForceAtPosition(
            axle.rightCollider.transform.up *  antiRollForce,
            axle.rightCollider.transform.position
        );
    }

    void TractionAdjust(Axle axle, ref float torqueRef)
    {
        if (!axle.motor) return;

        WheelHit hit;

        // Check slip on both wheels; if too high, cut torque a bit
        if (axle.leftCollider && axle.leftCollider.GetGroundHit(out hit))
        {
            if (Mathf.Abs(hit.forwardSlip) > slipLimit || Mathf.Abs(hit.sidewaysSlip) > slipLimit)
            {
                axle.leftCollider.motorTorque *= (1f - tractionTorqueCut);
                torqueRef *= (1f - tractionTorqueCut);
            }
        }

        if (axle.rightCollider && axle.rightCollider.GetGroundHit(out hit))
        {
            if (Mathf.Abs(hit.forwardSlip) > slipLimit || Mathf.Abs(hit.sidewaysSlip) > slipLimit)
            {
                axle.rightCollider.motorTorque *= (1f - tractionTorqueCut);
                torqueRef *= (1f - tractionTorqueCut);
            }
        }
    }

    void UpdateWheelVisuals(Axle axle)
    {
        UpdateWheel(axle.leftCollider, axle.leftMesh);
        UpdateWheel(axle.rightCollider, axle.rightMesh);
    }

    void UpdateWheel(WheelCollider col, Transform mesh)
    {
        if (col == null || mesh == null) return;

        Vector3 pos;
        Quaternion rot;
        col.GetWorldPose(out pos, out rot);

        mesh.position = pos;
        mesh.rotation = rot; // Align mesh rotation with collider
    }
}
