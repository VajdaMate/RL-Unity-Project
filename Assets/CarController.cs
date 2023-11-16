using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class CarController : Agent
{
    private Vector3 startingPosition = new Vector3(2, 1, -11);
    private float accelerationSpeed = 300;
    private float steeringSpeed = 300;
    private Rigidbody rb;


    // Start is called before the first frame update
    void Start()    
    {
        rb = GetComponent<Rigidbody>();
    }

    private void FixedUpdate()
    {
        float moveInput = 0f;
        float steerInput = 0f;

        // Forward/Backward Movement
        if (Input.GetKey("w"))
            moveInput = 1f;
        else if (Input.GetKey("s"))
            moveInput = -1f;

        // Left/Right Steering
        if (Input.GetKey("a"))
            steerInput = -1f;
        else if (Input.GetKey("d"))
            steerInput = 1f;


        //Másik megoldás
        // Move the car forward/backward
        //transform.Translate(Vector3.forward * moveInput * moveSpeed * Time.deltaTime);

        // Rotate the car left/right
        //transform.Rotate(Vector3.up * rotateInput * turnSpeed * Time.deltaTime);


        Vector3 forwardForce = transform.forward * moveInput * accelerationSpeed;
        rb.AddForce(forwardForce);

        // Apply steering torque
        float turnTorque = steerInput * steeringSpeed;
        rb.AddTorque(transform.up * turnTorque);
    }
    public override void OnEpisodeBegin()
    {
       
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        //3D Rayperception miatt nem kéne, de itthagyom a struktúra miatt
        //transform.localPosition = startingPosition;
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
       
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continuousActions = actionsOut.ContinuousActions;

        // Reset actions
        continuousActions[0] = 0f;
        continuousActions[1] = 0f;

        // Forward / Backward
        if (Input.GetKey("w"))
            continuousActions[0] = Mathf.MoveTowards(continuousActions[0], 1f, Time.deltaTime * accelerationSpeed);
        else if (Input.GetKey("s"))
            continuousActions[0] = Mathf.MoveTowards(continuousActions[0], -1f, Time.deltaTime * accelerationSpeed);

        // Left / Right
        if (Input.GetKey("a"))
            continuousActions[1] = Mathf.MoveTowards(continuousActions[1], -1f, Time.deltaTime * steeringSpeed);
        else if (Input.GetKey("d"))
            continuousActions[1] = Mathf.MoveTowards(continuousActions[1], 1f, Time.deltaTime * steeringSpeed);


    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.collider.tag == "Wall")
        {
            AddReward(-1);
            EndEpisode();
        }
        else if (collision.collider.tag == "Car")
        {
            AddReward(-1);
            EndEpisode();
        }
    }
}
