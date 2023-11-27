using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class CarController : Agent
{
    
    public Transform CheckPoint;
    public Transform Goal;


    private float accelerationSpeed = 275f;
    private float steeringSpeed = 250f;
    private Rigidbody rb;
    
    
    private Vector3 startingPosition = new Vector3(6f, 0.5351701f, 4f);
    private List<Vector3> checkpointPositions = new List<Vector3>
    {
        new Vector3(3.8f, 0.6f, -10.49f),
        new Vector3(-2.58f, 0.6f, -10.49f),
    };
    private int atCheckpoint = 0;


    // Start is called before the first frame update
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

        float distance = Vector3.Distance(CheckPoint.localPosition, transform.localPosition);
        

        //transform.Translate(actionSpeed * Vector3.forward * accelerationSpeed * Time.fixedDeltaTime);
        //transform.rotation = Quaternion.Euler(new Vector3(0, actionSteering * 180, 0));

        AddReward(-distance / 100); // [0, 0.1]
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
            AddReward(-200);
            EndEpisode();
        }
        else if (collision.collider.tag == "Parked Cars")
        {
            AddReward(-200);
            EndEpisode();
        }
        
        if (collision.collider.tag == "Goal")
        {
            AddReward(100);
            EndEpisode();
        }
        
    }

    private void OnTriggerEnter(Collider collider)
    {
        if (collider.tag == "Checkpoint")
        {
            AddReward(50);
            atCheckpoint++;
            if (atCheckpoint==1)
            {
                CheckPoint.localPosition = checkpointPositions[1];
            }
            else
            {
                CheckPoint = Goal;
            }
        }
    }
}
