using UnityEngine;
using System.Collections;
using System;
using System.Diagnostics;
using PrimeTween;
using TMPro;
using PrimeTweenDemo;
using UnityEngine.Serialization;
using Debug = UnityEngine.Debug;


public class HoverBoardController : MonoBehaviour
{
    [Header("References")]
    private Rigidbody rb;

    private Transform camTransform;
    public Transform groundCheckT, snapCheckT, slopeCheckT;

    public TextMeshProUGUI VelocityText;
    public TextMeshProUGUI AngularVelocityText;




    [Header("Movement")]
    public float maxVelocity;
    [FormerlySerializedAs("speed")] public float forwardSpeed = 10f;
    [FormerlySerializedAs("horizontalSpeedMultiplier")] public float horizontalSpeed;
    public float groundGravity = -9.81f;
    public float groundDrag;
    public float airMoveMultiplier;
    public float airGravityDivider = 2;
    public float airDrag;
    public float maxAngleVelocity;
    public float turnSpeed = 5f;
    public float groundTurnSpeed;
    public float airTurnSpeed;
    public float verticalTurnMultiplier = 2f;
    public float horizontalTurnMultiplier = 2f;
    public float leanTurnMultiplier = 2f;

    public float baseZrotation;
    public float ZSmooothSlerpSpeed;
    public float xSmoothSlerpSpeed=2.5f;

    public float maxZrotation;
    public float minZrotation;
    public float minXrotation;
    public float maxXrotation;


    [Header("Ground Check")]
    public float groundDistanceForSnap = 0.5f;
  
    public float groundSnapSpeed = 2f;
    public float groundOverlap = 0.5f;
    
    public LayerMask groundLayer;


    [Header("Booleans")]
    public bool isGrounded;
    public bool onSlope;


    [Header("Camera")]
    public float cameraFollowSpeed;
    public float cameraHeight = 10f;
    public float cameraDistance = 10f;

    

    private float horizontalInput, verticalInput, xAxisTurnInput;

    RaycastHit slopeHit;
    Vector3 slopeNormal;
    
   
    float xRotation;
    Vector3 moveDir;
    
   

    private Vector3 velocityBeforePhysicsUpdate;

    public LineRenderer debugLineX;
    public LineRenderer debugLineY;
    public LineRenderer debugLineZ;

    public LineRenderer[] debuglines;




    [Header("SPRING PROPERTIES")]
    public Transform[] hoverPoints;      // Hover points for raycasting
    public Transform hoverPoint;
    public float hoverHeight = 2.0f;     // Desired hover height
    public float hoverStrength = 10.0f;  // How strong the spring force is
    public float dampingRatio = 0.5f;    // Damping ratio for smooth movement
    public float angularFrequency = 2.0f;// Spring frequency

    private float[] currentVelocities;
    private float currentVelocity;
    private SpringUtils.tDampedSpringMotionParams springParams;


    public bool canSnap;
    private Vector3 moveDirForward;
    private Vector3 moveDirHorizontal;
    public float velocityLerpSpeed;
    public float breakLerpSpeed;
    public bool break;    


    bool OnSLope()
    {   
        

        RaycastHit hit;

        if (Physics.Raycast(slopeCheckT.position, -slopeCheckT.up, out hit,groundOverlap,groundLayer))
        {
            if (hit.normal != Vector3.up)
            {   
                slopeNormal = hit.normal;
                return true;

            }

            else return false;
        }
        else return false;
        
    } void Start()
         {
            
             currentVelocities = new float[hoverPoints.Length];
     
             springParams = new SpringUtils.tDampedSpringMotionParams();
             rb = GetComponent<Rigidbody>();
     
             if (Camera.main != null) camTransform = Camera.main.transform;
         }
    void Update()
    {
        if (verticalInput == 0)
        {
            break=true;
        }
        else break=false;

        
       
        
       
            
        VelocityText.text = "Velocity: " + rb.linearVelocity.magnitude.ToString("F2");
        AngularVelocityText.text= "Angular Velocity: " + rb.angularVelocity.magnitude.ToString("F2");

        rb.maxLinearVelocity = maxVelocity;

        isGrounded = Physics.CheckSphere(groundCheckT.position, groundOverlap, groundLayer);
        onSlope = OnSLope();

        PlayerInput();
       

       


        ControlAirParameters();

   

   Vector3 velocity = transform.InverseTransformDirection(rb.linearVelocity);
   Vector3 velocityX = new Vector3(velocity.x,0,0);
   Vector3 velocityY = new Vector3(0,velocity.y,0);
   Vector3 velocityZ = new Vector3(0,0,velocity.z);
   
   debugLineX.SetPosition(1, velocityX);
   debugLineY.SetPosition(1, velocityY);
   debugLineZ.SetPosition(1, velocityZ);
   
   

    }

    private void LateUpdate()
    {
        Vector3 position = transform.position - (transform.forward * cameraDistance) + Vector3.up * cameraHeight;
        position= new Vector3(position.x,position.y,position.z);

       camTransform.position = Vector3.Lerp(camTransform.position,position,Time.deltaTime*cameraFollowSpeed);
      
        camTransform.LookAt(transform.position);
    }

    private void FixedUpdate()
    {   HandleHoverSpring();
        RedirectVelocityToForward();       // Can use this maybe.
        
        HandleXSmoothing();
        
        Move();
        TurnHorizontal();
        TurnVertical();

      

       HandleXZRotationConstraint();
       HandleZSmoothing();


       Break();
        
        if (canSnap)
        {   
            
            
            Vector3 front = slopeCheckT.position;
            Vector3 back = groundCheckT.position;
            Vector3 middle = snapCheckT.position;

            Vector3 normalSum = Vector3.zero;
            int rayHitCount = 0;

            RaycastHit snapHit;
            if (Physics.Raycast(front, -snapCheckT.up, out snapHit, groundDistanceForSnap, groundLayer))
            {
                normalSum += snapHit.normal;
                rayHitCount++;
            }
            if (Physics.Raycast(back, -snapCheckT.up, out snapHit, groundDistanceForSnap, groundLayer))
            {
                normalSum += snapHit.normal;
                rayHitCount++;
            }
            if (Physics.Raycast(middle, -snapCheckT.up, out snapHit, groundDistanceForSnap, groundLayer))
            {
                normalSum += snapHit.normal;
                rayHitCount++;
            }

            if (rayHitCount > 0)
            {
                Vector3 averagedNormal = (normalSum / rayHitCount).normalized;
                Vector3 target = Vector3.ProjectOnPlane(transform.forward, averagedNormal).normalized;
                Quaternion targetRotate = Quaternion.LookRotation(target, averagedNormal);
                rb.rotation = Quaternion.Slerp(rb.rotation, targetRotate, Time.deltaTime * groundSnapSpeed);
                Debug.DrawRay(transform.position, 10*target, Color.blue);
                Debug.DrawRay(transform.position, 10*averagedNormal, Color.green);
                Debug.DrawRay(transform.position, 10*transform.forward, Color.blue);
            }

          //  RaycastHit snapHit;
          //  if (Physics.Raycast(snapCheckT.position, -snapCheckT.transform.up, out snapHit, groundDistanceForSnap, groundLayer))
          //  {   
          //      Vector3 target=Vector3.ProjectOnPlane(transform.forward, snapHit.normal);
          //      Quaternion targetRotate = Quaternion.LookRotation(target, snapHit.normal);
          //      rb.rotation = Quaternion.Lerp(rb.rotation, targetRotate, Time.deltaTime * groundSnapSpeed);
          //      
          //    // Quaternion targetRotation = Quaternion.FromToRotation(transform.up, snapHit.normal) * rb.rotation;
          //    // rb.rotation = Quaternion.Lerp(rb.rotation, targetRotation, Time.deltaTime * groundSnapSpeed);
    //
          //  }
       //
        }

     

    }


    private void Break()
    {   if(break)
        {
            
        }
        float magnitude = rb.velocity.magnitude;
        rb.linearVelocity=rb.linearVelocity.normalized*(Mathf.Lerp(magnitude, magnitude*0.99f, Time.deltaTime * breakLerpSpeed));
    }
    
    private void RedirectVelocityToForward()
    {
        rb.linearVelocity=Vector3.Lerp(rb.linearVelocity.normalized,transform.forward,Time.deltaTime*velocityLerpSpeed)*rb.linearVelocity.magnitude;
    }
    private void HandleXSmoothing()
    {
        if (!isGrounded)                            // Air X rotation smoothing in the direction of Velocity.
        {
            Quaternion rotation = Quaternion.LookRotation(rb.linearVelocity.normalized, transform.up);         
            rb.rotation=Quaternion.Slerp(rb.rotation, rotation, Time.deltaTime * xSmoothSlerpSpeed);    
        }
    }


    private void HandleXZRotationConstraint()
    {   
        
        Vector3 currentRotation = rb.rotation.eulerAngles;

        float zRotation = currentRotation.z;
        if (zRotation > 180) zRotation -= 360;
        
        float xRotation = currentRotation.x;
        if (xRotation > 180) xRotation -= 360;
        
        zRotation= Mathf.Clamp(zRotation, minZrotation, maxZrotation);
        xRotation= Mathf.Clamp(xRotation, minXrotation, maxXrotation);
         
        Quaternion targetRotation = Quaternion.Euler(xRotation, currentRotation.y, zRotation);

        rb.rotation = targetRotation;
    }

    private void HandleZSmoothing()
    {    
         Quaternion targetRotation = Quaternion.Euler(rb.rotation.eulerAngles.x,rb.rotation.eulerAngles.y,baseZrotation);
         Quaternion slopeTargetRotation = Quaternion.LookRotation(transform.forward, slopeNormal);
         if (onSlope)
         {
             rb.rotation = Quaternion.Slerp(rb.rotation, slopeTargetRotation,Time.deltaTime* ZSmooothSlerpSpeed);
         }
         else                                                                                                       // CHECK THESE AGAIN.
         {
             rb.rotation= Quaternion.Slerp(rb.rotation, targetRotation, Time.deltaTime * ZSmooothSlerpSpeed);
         }
      //  if (horizontalInput ==0&& xAxisTurnInput==0)
      //  {  
      //     
      //      rb.rotation= Quaternion.Slerp(rb.rotation, targetRotation, Time.deltaTime * ZSmooothSlerpSpeed);
      //  }
    }

    void PlayerInput()
    {
        horizontalInput = Input.GetAxis("Horizontal");
        verticalInput = Input.GetAxis("Vertical");
        xAxisTurnInput = Input.GetAxis("Fire1");
        
      Vector3 currentRotation = rb.rotation.eulerAngles;
      if(currentRotation.z > 180) currentRotation.z -= 360;
        
      float zRotation = currentRotation.z;
      Vector3 upVectorF = transform.right;
      Vector3  upVector = Quaternion.AngleAxis(-zRotation,transform.forward) * upVectorF;
      
      
      Vector3 horizontalMoveDirection = upVector.normalized;

        moveDirForward = (transform.forward * verticalInput);
        moveDirHorizontal = (horizontalMoveDirection * horizontalInput);


    }
    private void ControlAirParameters()
    {
        if (isGrounded)
        {
            Physics.gravity = new Vector3(0, groundGravity, 0);
            rb.linearDamping = groundDrag;
            turnSpeed = groundTurnSpeed;
            
        }
        else
        {   Physics.gravity = new Vector3(0, groundGravity/airGravityDivider, 0) ;
            rb.linearDamping = airDrag;
            turnSpeed = airTurnSpeed;
        }
    }
  // private void Move()
  // {
  //     if (isGrounded && onSlope)
  //     {
  //         Vector3 slopeMoveDir = Vector3.ProjectOnPlane(transform.forward, slopeDirection);
  //
  //         rb.AddForce(slopeMoveDir * verticalInput * speed, ForceMode.Acceleration);
  //
  //     }
  //     else if (isGrounded)
  //     {
  //                                                                                                            OR�G�NAL MOVE METHOD
  //         rb.AddForce(transform.forward * verticalInput * speed, ForceMode.Acceleration);
  //     }
  //     else
  //     {   
  //         Vector3 airDir= new Vector3(transform.forward.x,0,transform.forward.z);
  //         airDir.Normalize();
  //         Debug.DrawLine(transform.position,20*airDir);
  //         rb.AddForce(airDir * verticalInput *airMoveMultiplier *speed / 2, ForceMode.Acceleration);
  //     }
  // }

    private void Move()
    {   
        if (isGrounded)
        {
            Vector3 finalMoveDirForward;
            Vector3 finalMoveDirHorizontal;
        
            if (onSlope)
            {
                finalMoveDirForward = Vector3.ProjectOnPlane(moveDirForward, slopeNormal).normalized;
                finalMoveDirHorizontal = Vector3.ProjectOnPlane(moveDirHorizontal, slopeNormal).normalized;
                
                
                
            }
            else
            {
                finalMoveDirForward = moveDirForward;
                finalMoveDirHorizontal = moveDirHorizontal;
                
                
            }

            rb.AddForce(finalMoveDirForward * forwardSpeed, ForceMode.Acceleration);
            rb.AddForce(finalMoveDirHorizontal*horizontalSpeed, ForceMode.Acceleration);
        }
       
        else
        {
            Vector3 airDirForward = new Vector3(moveDirForward.x, moveDirForward.y/2, moveDirForward.z);
            Vector3 airDirHorizontal = new Vector3(moveDirHorizontal.x, 0, moveDirHorizontal.z);
            airDirForward.Normalize();
            airDirHorizontal.Normalize();
           
            rb.AddForce(airDirForward * (airMoveMultiplier * forwardSpeed) , ForceMode.Acceleration);
            rb.AddForce(airDirHorizontal*(airMoveMultiplier*horizontalSpeed), ForceMode.Acceleration);
        }
    }


    private void OnDrawGizmosSelected()
    {
        


        
    }

    private void TurnHorizontal()
    {   rb.maxAngularVelocity = maxAngleVelocity ;
        Vector3 currentRotation = rb.rotation.eulerAngles;
        if(currentRotation.z > 180) currentRotation.z -= 360;
       
        
        float zRotation = currentRotation.z;
        Vector3 upVectorF = transform.up;
        Vector3  upVector = Quaternion.AngleAxis(-zRotation,transform.forward) * upVectorF; 
        upVector.Normalize();


    


        if (onSlope)
        {
            rb.AddTorque(slopeNormal * (horizontalInput * horizontalTurnMultiplier * turnSpeed), ForceMode.Acceleration);
        }
        else
        {
            rb.AddTorque(upVector * (horizontalInput * horizontalTurnMultiplier * turnSpeed), ForceMode.Acceleration);
        }
         
       

         rb.AddTorque(-transform.forward * (horizontalInput * leanTurnMultiplier * turnSpeed), ForceMode.Acceleration);
            
            
            

            
             
    }

    private void TurnVertical()
    {   
        if(!isGrounded)
            rb.AddTorque(transform.right * (xAxisTurnInput * verticalTurnMultiplier * turnSpeed), ForceMode.Acceleration);
    }



    private void HandleHoverSpring()
    {
          //  // Raycast from the board down to check the distance to the ground
     //  RaycastHit hit;
     //  if (Physics.Raycast(groundCheckT.position, -groundCheckT.transform.up, out hit, hoverHeight * 2f, groundLayer))
     //  {
     //      // Get current distance to the ground
     //      float currentHeight = hit.distance;
     //
     //      // Calculate spring motion parameters based on the hover system
     //      SpringUtils.CalcDampedSpringMotionParams(
     //          ref springParams,
     //          Time.fixedDeltaTime,
     //          angularFrequency,
     //          dampingRatio
     //      );
     //
     //      // Apply the spring force using the distance from the ground as the equilibrium position
     //      float targetHeight = hoverHeight;
     //      SpringUtils.UpdateDampedSpringMotion(
     //          ref currentHeight,
     //          ref currentVelocity,
     //          targetHeight,
     //          springParams
     //      );
     //
     //      // Calculate the upward force to apply to the rigidbody
     //      float springForce = hoverStrength * (hoverHeight - hit.distance) - currentVelocity;
     //
     //      // Apply the force to the Rigidbody
     //      rb.AddForce(groundCheckT.transform.up * springForce, ForceMode.Acceleration);
     //  }
       
       
        

        // Iterate through each hover point to apply forces individually
    for (int i = 0; i < hoverPoints.Length; i++)
    {
        RaycastHit hit;
        hoverPoint = hoverPoints[i];

        // Raycast from the hover point down to check the distance to the ground
        if (Physics.Raycast(hoverPoint.position, -hoverPoint.up, out hit, hoverHeight * 2f, groundLayer))
        {   
            // Get the distance from the hover point to the ground
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
                ref currentVelocities[i], // Use individual velocity per point
                targetHeight,
                springParams
            );

            // Calculate the upward force to apply based on the surface normal
            Vector3 springForceDirection = hit.normal; // Surface normal direction
            float springForce = hoverStrength * (hoverHeight - hit.distance) - currentVelocities[i];

            // Apply the force to the Rigidbody at the hover point position
            rb.AddForceAtPosition(springForceDirection * springForce, hoverPoint.position, ForceMode.Acceleration);
        }
    }
    }

    private void OnCollisionEnter(Collision other)
    {
        if (other.gameObject.layer==6)
        {
            Debug.Log("Collision Enter");
        }
    }
}
