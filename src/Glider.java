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


    // === Initialize Sensors (Placeholder) ===
    // This is a placeholder function for initializing sensors such as GPS, IMU, etc.
    // In a real-world system, this would interface with actual hardware
    public static void initializeSensors(){
        System.out.printf("Sensors initialized (GPS, IMU, etc.)");
    }

    // === PID Controller Setup ===
    // This function initalizes a PID controler using constants (Kp, Ki, Kd)
    // The output limits ensure the glider's angle is within a valid range (-5 to 5 degrees)
    public static PID initialize_pid(){
        PID pid = new PID(Constants.Kp,Constants.Ki,Constants.Kd,1); // PID constants defined in Constants class
        pid.setOutputLimits(new double[] {-5,5}); // Limiting the angle of adjustment between -5 and 5 degrees
        return pid;
    }

    // === Target Glide Path Calculations ===
    // This function calculates the glide slope based on the current altitude, target altitude,
    // and the distance to the target. If the distance is zero, it simply returns the target altitude
    public static double calculateGlidePath(double current_altitude, double target_altitude, double distance_to_target){
        if(distance_to_target ==0){
            return target_altitude; // If distance to target is zero, return the target altitude directly
        }
        return (current_altitude - target_altitude)/distance_to_target; // Returns glide slope
    }

    // === Control Adjustment Based on PID ===
    // This function adjusts the glider's angle based on the output for the PID controller
    public static void adjust_Controls(double controlAdjustment){
        gliderAngle = controlAdjustment; // Adjust glider's angle to the PID control output
    }
 
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

