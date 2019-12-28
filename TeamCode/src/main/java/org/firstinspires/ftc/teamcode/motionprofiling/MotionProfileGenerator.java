package org.firstinspires.ftc.teamcode.motionprofiling;

public class MotionProfileGenerator {
    //constants based on robot
    public static double MAX_VELOCITY; //inches per second
    public static double MAX_ACCELERATION; //inches per second squared

    private double totalDistance;

    public MotionProfileGenerator(double distance){
        totalDistance = distance;
    }

    public double[] generateMotionProfile(){
        double[] velocities;

        double maxAccelDistance = MAX_VELOCITY * MAX_VELOCITY / (2 * MAX_ACCELERATION);
        //triangular profile
        if(maxAccelDistance >= totalDistance){
            double adjustedVelocity = Math.sqrt(totalDistance * MAX_ACCELERATION);

            //the length of the graph is t, the height of the graph is vmax, the area of the graph is amax
            double totalTimeSeconds = 2 * totalDistance / adjustedVelocity;
            velocities = new double[(int)totalTimeSeconds * 1000];

            for(int i = 0; i < velocities.length; i++){
                double currentTimeSeconds = i / 1000.0;
                //accelerating
                if(currentTimeSeconds < adjustedVelocity / MAX_ACCELERATION)
                    velocities[i] = MAX_ACCELERATION * currentTimeSeconds;
                //decelerating
                else
                    velocities[i] = adjustedVelocity - MAX_ACCELERATION * (currentTimeSeconds - (totalTimeSeconds / 2));
            }
        }
        //trapezoidal profile
        else{
            double totalTimeSeconds = 2 * MAX_VELOCITY / MAX_ACCELERATION + (totalDistance - maxAccelDistance) / MAX_VELOCITY;
            double timeTwoThirds = MAX_VELOCITY / MAX_ACCELERATION + (totalDistance - maxAccelDistance) / MAX_VELOCITY;
            velocities = new double[(int)(totalTimeSeconds * 1000)];

            for(int i = 0; i < velocities.length; i++){
                double currentTimeSeconds = i / 1000.0;
                //accelerating
                if(currentTimeSeconds < MAX_VELOCITY / MAX_ACCELERATION)
                    velocities[i] = MAX_ACCELERATION * currentTimeSeconds;
                //cruising
                else if(currentTimeSeconds < MAX_VELOCITY / MAX_ACCELERATION + (totalDistance - maxAccelDistance) / MAX_VELOCITY)
                    velocities[i] = MAX_VELOCITY;
                //decelerating
                else
                    velocities[i] = MAX_VELOCITY - MAX_ACCELERATION * (currentTimeSeconds - timeTwoThirds);
            }
        }
        return velocities;
    }
}
