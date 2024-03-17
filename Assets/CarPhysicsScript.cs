using System;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class CarPhysicsScript : MonoBehaviour
{
    // Start is called before the first frame update
    Rigidbody carRigidBodyBody;
    Ray rayFL;
    Ray rayFR;
    Ray rayBL;
    Ray rayBR;
    public float suspensionRestDistance = 0.5f;
    GameObject transformFL;
    GameObject transformFR;
    GameObject transformBL;
    GameObject transformBR;
    Transform transformFLtransform;
    Transform transformFRtransform;
    Transform transformBLtransform;
    Transform transformBRtransform;
    Vector3 transformFLposition;
    Vector3 transformFRposition;
    Vector3 transformBLposition;
    Vector3 transformBRposition;
    bool rayFLDidHit;
    bool rayFRDidHit;
    bool rayBLDidHit;
    bool rayBRDidHit;
    //public float force = 10f;
    RaycastHit hitFL;
    RaycastHit hitFR;
    RaycastHit hitBL;
    RaycastHit hitBR;
    public float offset;
    public float springStrength = 40000f;
    public float springForce;
    public float tireVelocity;
    public Vector3 tireWorldVelocity;
    public float tireForce;
    public float damping = 400f;

    void Start()
    {
        transformFL = GameObject.Find("Transform FL");
        transformFR = GameObject.Find("Transform FR");
        transformBL = GameObject.Find("Transform BL");
        transformBR = GameObject.Find("Transform BR");
        carRigidBodyBody = (GameObject.Find("CarBody")).GetComponent<Rigidbody>();
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        transformFLtransform = transformFL.transform;
        transformFRtransform = transformFR.transform;
        transformBLtransform = transformBL.transform;
        transformBRtransform = transformBR.transform;

        transformFLposition = transformFLtransform.position;
        transformFRposition = transformFRtransform.position;
        transformBLposition = transformBLtransform.position;
        transformBRposition = transformBRtransform.position;

        rayFL = new Ray(transformFLposition, -transform.up);
        rayFR = new Ray(transformFRposition, -transform.up);
        rayBL = new Ray(transformBLposition, -transform.up);
        rayBR = new Ray(transformBRposition, -transform.up);

        rayFLDidHit = Physics.Raycast(rayFL, out hitFL, suspensionRestDistance);
        if (rayFLDidHit)
        {
            offset = suspensionRestDistance - hitFL.distance;
            springForce = offset * springStrength;
            tireWorldVelocity = carRigidBodyBody.GetPointVelocity(transformFLposition);
            tireVelocity = Vector3.Dot(transformFLtransform.up, tireWorldVelocity);
            tireForce = springForce - (tireVelocity * damping);

            carRigidBodyBody.AddForceAtPosition(tireForce * transformFLtransform.up, transformFLposition);
        }

        rayFRDidHit = Physics.Raycast(rayFR, out hitFR, suspensionRestDistance);
        if (rayFRDidHit)
        {
            offset = suspensionRestDistance - hitFR.distance;
            springForce = offset * springStrength;
            tireWorldVelocity = carRigidBodyBody.GetPointVelocity(transformFRposition);
            tireVelocity = Vector3.Dot(transformFRtransform.up, tireWorldVelocity);
            tireForce = springForce - (tireVelocity * damping);

            carRigidBodyBody.AddForceAtPosition(tireForce * transformFRtransform.up, transformFRposition);
        }

        rayBLDidHit = Physics.Raycast(rayBL, out hitBL, suspensionRestDistance);
        if (rayBLDidHit)
        {
            offset = suspensionRestDistance - hitBL.distance;
            springForce = offset * springStrength;
            tireWorldVelocity = carRigidBodyBody.GetPointVelocity(transformBLposition);
            tireVelocity = Vector3.Dot(transformBLtransform.up, tireWorldVelocity);
            tireForce = springForce - (tireVelocity * damping);

            carRigidBodyBody.AddForceAtPosition(tireForce * transformBLtransform.up, transformBLposition);
        }

        rayBRDidHit = Physics.Raycast(rayBR, out hitBR, suspensionRestDistance);
        if (rayBRDidHit)
        {
            offset = suspensionRestDistance - hitBR.distance;
            springForce = offset * springStrength;
            tireWorldVelocity = carRigidBodyBody.GetPointVelocity(transformBRposition);
            tireVelocity = Vector3.Dot(transformBRtransform.up, tireWorldVelocity);
            tireForce = springForce - (tireVelocity * damping);

            carRigidBodyBody.AddForceAtPosition(tireForce * transformBRtransform.up, transformBRposition);
        }
    }
}
