using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class CarController : Agent
{
    // Start is called before the first frame update
    void Start()
    {
    }

    public override void OnEpisodeBegin()
    {
       
    }

    public override void CollectObservations(VectorSensor sensor)
    {
       //3D Rayperception miatt nem k�ne, de itthagyom a strukt�ra miatt
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
       
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
       
    }

    private void OnCollisionEnter(Collision collision)
    {
       
    }
}
