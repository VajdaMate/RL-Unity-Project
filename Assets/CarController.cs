using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class CarController : Agent
{
    public Transform CheckPoint;
    private float accelerationSpeed = 275f;
    private float steeringSpeed = 295f;
    private Rigidbody rb;
    
    
    private Vector3 startingPosition = new Vector3(6f, 0.5351701f, 4f);
    private List<Vector3> checkpointPositions = new List<Vector3>
    {
        new Vector3(5.29f, 0.6f, 0.07f),
        new Vector3(5.29f, 0.6f, -3.45f),
        new Vector3(3.69f, 0.6f, -7.23f),
        new Vector3(4f, 0.6f, -9.27f),
        new Vector3(1.14f, 0.6f, -11.6f),
        new Vector3(-1.3f, 0.6f, -11.6f),
        new Vector3(-2.46f, 0.6f, -6.98f),
        new Vector3(-2.453f, 0.6f, -1.16f),

    };
    private int atCheckpoint = 0;

    void Start()    
    {
        rb = GetComponent<Rigidbody>();
        
    }

    private void FixedUpdate()
    {
   
    }
    public override void OnEpisodeBegin()
    {
        transform.localPosition = startingPosition;
        transform.rotation = Quaternion.Euler(new Vector3(0, 180, 0));
        atCheckpoint = 0;
        CheckPoint.localPosition = checkpointPositions[atCheckpoint];
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(transform.localPosition);
        sensor.AddObservation(CheckPoint.localPosition);
        sensor.AddObservation(Vector3.Distance(CheckPoint.localPosition, transform.localPosition));
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

        //transform.Translate(actionSpeed * Vector3.forward * accelerationSpeed * Time.fixedDeltaTime);
        //transform.rotation = Quaternion.Euler(new Vector3(0, actionSteering * 180, 0));
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
        else if (collision.collider.tag == "Parked Cars")
        {
            AddReward(-1);
            EndEpisode();
        }
    }

    private void OnTriggerEnter(Collider collider)
    {
        if (collider.tag == "Checkpoint")
        {
            if(atCheckpoint < ((checkpointPositions.Count)-1))
            {
                AddReward(0.1f);
                atCheckpoint++;
                CheckPoint.localPosition = checkpointPositions[atCheckpoint];
            }
            else
            {
                AddReward(10);
                EndEpisode();
            } 
        }
    }
}
