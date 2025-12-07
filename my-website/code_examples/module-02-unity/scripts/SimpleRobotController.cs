using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class SimpleRobotController : MonoBehaviour
{
    public float speed = 1.0f;
    public float turnSpeed = 10.0f;
    
    // Articulation Bodies for wheels (assuming Unity 2020+ Physics)
    public ArticulationBody leftWheel;
    public ArticulationBody rightWheel;

    void Start()
    {
        // Register a ROS Subscriber if we wanted to control via ROS
        // ROSConnection.GetOrCreateInstance().Subscribe<TwistMsg>("cmd_vel", OnMessageReceived);
    }

    void Update()
    {
        // Simple manual control for testing
        float move = Input.GetAxis("Vertical") * speed;
        float turn = Input.GetAxis("Horizontal") * turnSpeed;

        if (leftWheel != null && rightWheel != null)
        {
            SetWheelVelocity(leftWheel, move + turn);
            SetWheelVelocity(rightWheel, move - turn);
        }
    }

    void SetWheelVelocity(ArticulationBody wheel, float velocity)
    {
        var drive = wheel.xDrive;
        drive.targetVelocity = velocity;
        wheel.xDrive = drive;
    }

    void OnMessageReceived(TwistMsg msg)
    {
        // Mapping ROS Twist to Wheel Velocities (Differential Drive Kinematics)
        // linear.x = forward speed, angular.z = rotation
        float linear = (float)msg.linear.x;
        float angular = (float)msg.angular.z;
        
        // This is a simplified kinematic model
        float leftVel = linear - angular; 
        float rightVel = linear + angular;

        if (leftWheel != null && rightWheel != null)
        {
            SetWheelVelocity(leftWheel, leftVel * speed);
            SetWheelVelocity(rightWheel, rightVel * speed);
        }
    }
}
