using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class CarController : Agent
{
    private float accelerationSpeed = 275f;
    private float steeringSpeed = 250f;
    public Transform TargetTransform;
    private Rigidbody rb;


    // Start is called before the first frame update
    void Start()    
    {
        rb = GetComponent<Rigidbody>();
    }

    private void FixedUpdate()
    {
        /*float moveInput = 0f;
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

        Vector3 forwardForce = transform.forward * moveInput * accelerationSpeed;
        rb.AddForce(forwardForce);

        // Apply steering torque
        float turnTorque = steerInput * steeringSpeed;
        rb.AddTorque(transform.up * turnTorque);
        
        //Másik megoldás
        //transform.Translate(Vector3.forward * moveInput * moveSpeed * Time.deltaTime);
        //transform.Rotate(Vector3.up * rotateInput * turnSpeed * Time.deltaTime);
        */

    }
    public override void OnEpisodeBegin()
    {
        transform.localPosition = new Vector3(6f, 0.5351701f, 4f);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(TargetTransform.localPosition);
        sensor.AddObservation(Vector3.Distance(TargetTransform.localPosition, transform.localPosition));
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        var actionTaken = actions.ContinuousActions;

        float actionSpeed = actionTaken[0];
        float actionSteering = actionTaken[1];


        Vector3 forwardForce = transform.forward * actionSpeed * accelerationSpeed;
        rb.AddForce(forwardForce);

        float turnTorque = actionSteering * steeringSpeed;
        rb.AddTorque(transform.up * turnTorque);

        float distance = Vector3.Distance(TargetTransform.localPosition, transform.localPosition);
        

        //transform.Translate(actionSpeed * Vector3.forward * accelerationSpeed * Time.fixedDeltaTime);
        //transform.rotation = Quaternion.Euler(new Vector3(0, actionSteering * 180, 0));

        AddReward(-distance / 10); // [0, 0.1]
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continuousActions = actionsOut.ContinuousActions;

        // Reset actions
        continuousActions[0] = 0f; // Speed
        continuousActions[1] = 0f; // Turning

        // Forward/Backward Movement
        if (Input.GetKey("w"))
            continuousActions[0] = 1f;
        else if (Input.GetKey("s"))
            continuousActions[0] = -1f;

        // Left/Right Steering
        if (Input.GetKey("a"))
            continuousActions[1] = -1f;
        else if (Input.GetKey("d"))
            continuousActions[1] = 1f;
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
        if (collision.collider.tag == "Goal")
        {
            AddReward(10);
            EndEpisode();
        }
    }
}
