#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <tf/transform_listener.h>

#define PI 3.1415926535897932384626433f
#define MAX_TURN_SPD 0.42359879
#define MIN_TURN_SPD 0.05

class Move{

    ros::Subscriber scanSubs;
    ros::Publisher velPub;
    ros::Publisher errorPub;
    tf::TransformListener transformListener;
    tf::StampedTransform robotTransform;
    ros::NodeHandle nh;

    geometry_msgs::Twist moveCmd;

    float error;
    float lastError;
    float dError;
    float iError;

    float pidP, pidI, pidD;

    float totalError;
    std_msgs::Float32 error_msg;


public:
    Move(){
        scanSubs=nh.subscribe("/scan", 10, &Move::laserCallback, this);
        velPub=nh.advertise<geometry_msgs::Twist> ("/cmd_vel",1);
        transformListener.waitForTransform("world", "base_link", ros::Time::now(), ros::Duration(3.0));
        errorPub = nh.advertise<std_msgs::Float32> ("/error",1);

        moveCmd.linear.x=2; // go forward always
        moveCmd.linear.y=0;

        error = 0.0f;
        lastError = 0.0f;
        iError = 0.0f;
        dError = 0.0f;

	    // P, I, D param3ters should be set
	
	float ku = 2.5;
	float pu = 2.458;
        pidP = ku/2.2;
        pidD = pidP * (pu/6.3) ;
        pidI = pidP / (pu*2.2);

        totalError = 0.0f;

    }

    void laserCallback(const sensor_msgs::LaserScan& input_scan)
    {
        //inspect laser data
        std::cout<<"ranges size: "<<input_scan.ranges.size()<<std::endl;
        std::cout<<"max range: "<<input_scan.range_max<<std::endl;
        std::cout<<"min range: "<<input_scan.range_min<<std::endl;
        std::cout<<"min angle: "<<input_scan.angle_min<<std::endl;
        std::cout<<"max angle: "<<input_scan.angle_max<<std::endl;
        std::cout<<"min increment: "<<input_scan.angle_increment<<std::endl;

 	//inspect robot pose
        transformListener.lookupTransform("world","base_link", ros::Time(0),robotTransform);
        float robotX=robotTransform.getOrigin().x();
        float robotY=robotTransform.getOrigin().y();
        float robotYaw=tf::getYaw(robotTransform.getRotation());
        std::cout<<"robot pose: x,y,yaw: "<<robotX<<","<<robotY<<","<<robotYaw
                <<std::endl;

        /**
        * A laser scan gives you an array of floats which corresponds to the distances. 
        * 
        * The code given below uses 100 readings from the left side of the laser sensor, 
        * the ones with indices between [100 and 200] and 100 from the right side, the 
        * ones with indices between [rangeSize-200, rangeSize-100] to calculate the mean
        * distances from the walls.
        */

        int offset = 100 + (PI/2-robotYaw)/input_scan.angle_increment;//dynamic offset
        int estCount = 100;
	int l =0 ;
	int r = 0;
        float leftDist = 0.0f;
        float rightDist = 0.0f;
        int rangeSize = input_scan.ranges.size();

        for (int i = 0; i < estCount; i++) {
        	float lDist = input_scan.ranges[i+offset];
        	float rDist = input_scan.ranges[rangeSize-offset-i];
		

            // check for min and max values
        	if (lDist > input_scan.range_max ) {
        		lDist = input_scan.range_max;
        	} else if(lDist == 0) {lDist = 1;
		}
		else if (lDist < 0.07) {
        		lDist = input_scan.range_min;
        	}
        	if (rDist > input_scan.range_max  ) {
        		rDist = input_scan.range_max;
        	}  else if(rDist == 0) {rDist = 1;}
		else if (rDist < 0.07) {
        		rDist = input_scan.range_min;
        	}	std::cout<< "l " << rDist << " r " << lDist << std::endl;
		
        	leftDist += lDist;
		
        	rightDist += rDist;
		
        }		std::cout<< std::endl;
        //calculate mean dist
	
        leftDist = leftDist / estCount; 
        rightDist = rightDist / estCount; 
        float avgDist = (leftDist + rightDist) / 2;


        if (leftDist > avgDist) {
		std::cout<< "go right " <<std::endl;
        	error = avgDist - leftDist;
        } else if (rightDist > avgDist) {
		std::cout<< "go left " <<std::endl;
        	error = rightDist - avgDist;
        } else {std::cout<< "go straight " <<std::endl;error = 0;
		}
	    
        /***
         * You are expected to calculate derivative error and integral error here
         * Then, you generate a turn command with PID
         */

        totalError += fabs(error);
        std::cout << "error=" << error << std::endl;
        if (lastError != 0.0f) {
        	dError = error - lastError; // derivative of error
        }
        std::cout << "dError=" << dError << std::endl;
        std::cout << "iError=" << iError << std::endl;

        // integral of error
        iError += error;

        lastError = error;
	error_msg.data = error;
	errorPub.publish(error_msg);
        // generate a PID controller
        float turn = error * pidP + dError * pidD + iError * pidI;
        std::cout << "turn=" << turn << std::endl;


        moveCmd.angular.z=turn;

        // keep the speed of the robot in min-max limits
        if (fabs(moveCmd.angular.z) > MAX_TURN_SPD) {
        	moveCmd.angular.z = sign(moveCmd.angular.z) * MAX_TURN_SPD;
        }
        if (fabs(moveCmd.angular.z) < MIN_TURN_SPD ) {
        	moveCmd.angular.z = sign(moveCmd.angular.z) * MIN_TURN_SPD;//sign(moveCmd.angular.z) * MIN_TURN_SPD;
        } 

        velPub.publish(moveCmd);
    }

    int sign(float value) {
    	if (value > 0)
    		return 1;
    	else
    		return -1;
    }
};

int main(int argc,char **argv)
{
    std::cout<<"Now robot will move in corridor..."<<std::endl;
    ros::init(argc,argv,"moving_in_corridor");
    Move move;
    ros::spin();
    printf("Robot Finish...");
}
