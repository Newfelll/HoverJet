using UnityEngine;

public class HoverController : MonoBehaviour
{
    public Rigidbody rb;                 // Reference to the rigidbody attached to Pivot
    public Transform board;              // Reference to the board for raycasting
    public LayerMask groundLayer;        // Layer of the ground for the raycast

    public float hoverHeight = 2.0f;     // Desired hover height
    public float hoverStrength = 10.0f;  // How strong the spring force is
    public float dampingRatio = 0.5f;    // Damping ratio for smooth movement
    public float angularFrequency = 2.0f;// Spring frequency

    private float currentVelocity = 0.0f;// Current hover velocity
    private SpringUtils.tDampedSpringMotionParams springParams;


    public bool canSnap = false;

    void Start()
    {
        // Initialize spring parameters
        springParams = new SpringUtils.tDampedSpringMotionParams();
    }




    private void Update()
    {
        

    }

    void FixedUpdate()
    {   
        // Raycast from the board down to check the distance to the ground
        RaycastHit hit;
        if (Physics.Raycast(board.position, -transform.up, out hit, hoverHeight * 2f, groundLayer))
        {
            // Get current distance to the ground
            float currentHeight = hit.distance;

            // Calculate spring motion parameters based on the hover system
            SpringUtils.CalcDampedSpringMotionParams(
                ref springParams,
                Time.fixedDeltaTime,
                angularFrequency,
                dampingRatio
            );

            // Apply the spring force using the distance from the ground as the equilibrium position
            float targetHeight = hoverHeight;
            SpringUtils.UpdateDampedSpringMotion(
                ref currentHeight,
                ref currentVelocity,
                targetHeight,
                springParams
            );

            // Calculate the upward force to apply to the rigidbody
            float springForce = hoverStrength * (hoverHeight - hit.distance) - currentVelocity;

            // Apply the force to the Rigidbody
            rb.AddForce(transform.up * springForce, ForceMode.Acceleration);
        }


        if (canSnap)
        {
            RaycastHit snapHit;
            if (Physics.Raycast(board.position, -transform.up, out snapHit, hoverHeight, groundLayer))
            {

                Quaternion targetRotation = Quaternion.FromToRotation(transform.up, snapHit.normal) * rb.rotation;
                rb.rotation = Quaternion.Lerp(rb.rotation, targetRotation, Time.deltaTime * 2);
            }

        }
    }
}
