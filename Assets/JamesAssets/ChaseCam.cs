using UnityEngine;

[DefaultExecutionOrder(1000)]
public class ChaseCamera : MonoBehaviour
{
    [Header("Target (car root with Rigidbody)")]
    public Transform target;

    [Header("Offset (world-space, ignores target scale)")]
    public Vector3 offset = new Vector3(0f, 2.0f, -3.0f);

    [Header("Smoothing")]
    public float followSmooth = 8f;
    public float lookSmooth = 12f;

    [Header("Safety / Behavior")]
    public bool snapOnStart = true;       // put camera at target+offset on Start
    public float maxFollowDistance = 20f; // clamp runaway distances
    public bool keepHorizonLevel = true;  // don't roll with target

    [Header("Auto distance from car bounds (optional)")]
    public bool autoDistanceFromBounds = true;
    public float distanceMultiplier = 2.5f; // how many car-lengths behind
    public float minDistance = 1.5f;
    public float maxDistance = 6f;

    Rigidbody targetRB;
    Vector3 vel;

    void Start()
    {
        if (!target)
        {
            Debug.LogWarning("[ChaseCamera] No target assigned.");
            enabled = false;
            return;
        }

        targetRB = target.GetComponentInParent<Rigidbody>();

        // Optional: auto size Z offset from renderer bounds
        if (autoDistanceFromBounds)
        {
            var rend = target.GetComponentInChildren<Renderer>();
            if (rend)
            {
                var size = rend.bounds.size;
                // Use the larger horizontal dimension as a “length”
                float carLen = Mathf.Max(size.z, size.x);
                float desiredBack = Mathf.Clamp(carLen * distanceMultiplier, minDistance, maxDistance);
                offset.z = -Mathf.Abs(desiredBack);
                // keep a reasonable height if not set
                if (offset.y <= 0.01f) offset.y = Mathf.Clamp(size.y * 0.8f, 1.0f, 3.0f);
            }
        }

        if (snapOnStart)
        {
            // World-space offset that ignores target scale:
            Vector3 desired = target.position + target.rotation * offset;
            transform.position = desired;
            LookAtTarget();
        }
    }

    void LateUpdate()
    {
        if (!target) return;

        // Compute desired world position (do NOT use TransformPoint; avoids scale issues)
        Vector3 desiredPos = target.position + target.rotation * offset;

        // Clamp runaway distances if the target teleports
        float dist = Vector3.Distance(transform.position, desiredPos);
        if (dist > maxFollowDistance)
        {
            transform.position = Vector3.Lerp(desiredPos, transform.position, maxFollowDistance / Mathf.Max(dist, 0.0001f));
        }

        // Smooth follow
        transform.position = Vector3.SmoothDamp(transform.position, desiredPos, ref vel, 1f / Mathf.Max(0.01f, followSmooth));

        // Smooth look
        LookAtTarget();
    }

    void LookAtTarget()
    {
        Vector3 aim = target.position + Vector3.up * 1.0f;
        Quaternion lookRot = Quaternion.LookRotation(aim - transform.position, Vector3.up);

        if (keepHorizonLevel)
        {
            Vector3 e = lookRot.eulerAngles;
            e.z = 0f;
            lookRot = Quaternion.Euler(e);
        }

        transform.rotation = Quaternion.Slerp(transform.rotation, lookRot, Time.deltaTime * Mathf.Max(0.01f, lookSmooth));
    }
}
