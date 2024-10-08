/**
 * Glider class that simulates the behavior of an autonomous glider.
 * It utilizes a PID controller to adjust the glider's angle during descent
 * and aims to land the glider within a defined target altitude and runway distance.
 */
public class Glider {
    
    // Target altitude for the glider to reach during its decent
    // You could randomize this value for variability in testing
    public static double TARGET_ALTITUDE = 180;

    // Distance to the runway for landing (set temporarily)
    public static double RUNWAY_DISTANCE = 400; // Temp Value

    // The starting altitude of the glider
    public static double currentAltitude = 205;

    // Temporary distance to target (used in glide calculations)
    public static double distanceToTarget = 200;

    // The current angle of the glider (controled by the PID loop)
    public static double gliderAngle = 0;

    /**
     * Initializes the sensors (GPS, IMU, etc.)
     * In a real-world system, this would interface with actual hardware to gather data
     */
    public static void initializeSensors(){
        System.out.printf("Sensors initialized (GPS, IMU, etc.)");
    }

    /**
     * Initializes the PID controller using constants (Kp, Ki, Kd)  defined in the Constants class.
     * The output limits ensure that the glider's angle stays within a valid range.
     * 
     * @return A configured PID controller for managing the glider's angle.
     */
    public static PID initialize_pid(){
        PID pid = new PID(Constants.Kp,Constants.Ki,Constants.Kd,1); // PID constants defined in Constants class
        pid.setOutputLimits(new double[] {-30,30}); // Limiting the angle of adjustment between -5 and 5 degrees
        return pid;
    }

    /**
     * Calculates the desired glide path based on the current altitude, target altitude,
     * and the distance to the target. If the distance is zero, it returns the target altitude directly.
     * 
     * @param current_altitude The current altitude of the glider
     * @param target_altitude The target altitude for landing
     * @param distance_to_target The remaining distance to the target
     * @return The calculated glide slope
     */
    public static double calculateGlidePath(double current_altitude, double target_altitude, double distance_to_target){
        if(distance_to_target ==0){
            return target_altitude; // If distance to target is zero, return the target altitude directly
        }
        return (current_altitude - target_altitude)/distance_to_target; // Returns glide slope
    }

    /**
     * Adjusts the glider's angle based on the control output from the PID controller.
     * 
     * @param controlAdjustment The new angle adjustment from the PID controller
     */
    public static void adjust_Controls(double controlAdjustment){
        gliderAngle = controlAdjustment; // Adjust glider's angle to the PID control output
    }
 
    /**
     * The main method that runs the glider simulation. It initializes sensors, sets up the PID controller,
     * and runs a continuous loop to simulate real-time control adjustments.
     * 
     * @param args Command-line arguments ignored
     */
    public static void main(String[] args){
        initializeSensors(); // Initialize the sensors

        PID pid = initialize_pid(); // Setup the PID controller

        int frequency = 4; // Frequency of updates (4 times per second)
        int sleepTime = 1000/frequency; // Sleep time in milliseconds between updates

        // Simulation loop that runs continuously
        while(true){
            // Display the current glider angle and altitude
            System.out.printf("Angle: %.3f | Current Altitude: %.2f%n", gliderAngle, currentAltitude);

            // Calculate altitude change based on the current glider angle
            double altitudeChange = gliderAngle*0.5;

            // Update the current altitude based on the altitude change
            currentAltitude += altitudeChange;

            // Calculated the desired glide path based on the current and target altitude
            double targetGlidePath = calculateGlidePath(currentAltitude, TARGET_ALTITUDE, distanceToTarget);

            // Calculate the error (difference between current altitude and target altitude)
            double currentError = currentAltitude - TARGET_ALTITUDE;

            // Get the control adjustment (new angle) from the PID controller based on the error
            double controlAdjustment = pid.calculate(currentError);

            // Adjust the glider's controls (angle) based on the PID output
            adjust_Controls(controlAdjustment);

            // Pause the loop for the appropriate sleep time to simulate real-time behavior
            try{
                Thread.sleep(sleepTime);
            }catch(InterruptedException e){
                e.printStackTrace(); // Handle interruption exception
            }
        }
    }
}

