/**
 * PID (Proportional-Integral-Derivative) Controller class.
 * This class is used to compute the output based on the difference
 * between the desired setpoint and the current value using PID control.
 */
public class PID {
    // PID gains (Kp, Ki, Kd) and variables for the previous error, integral sum, setpoint, and time interval (dt)
    private double Kp, Ki, Kd;
    private double previousError;
    private double integralSum;
    private double setPoint;
    private double dt;
    // Output limits to constrain the result
    private double[] outputLimits = {Double.NEGATIVE_INFINITY,Double.POSITIVE_INFINITY};

    /**
     * Constructor to initalize the PID controller with specific gain and time step.
     *  
     * @param Kp The proportional gain.
     * @param Ki The integral gain.
     * @param Kd The derivative gain.
     * @param dt The time step (interval between PID calculations).
     */
    public PID(double Kp, double Ki, double Kd, double dt){
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.dt = dt;
        this.previousError = 0;
        this.integralSum = 0;
    }

    /**
     * Sets the desired setpoint for the PID controller.
     * 
     * @param setPoint The target value that the PID controller should aim to achieve.
     */
    public void setSetpoint(double setPoint){
        this.setPoint = setPoint;
    }

    /**
     * Sets the output limits to constrain the PID output within a specified range.
     * 
     * @param limits An array with two values: {minimum limit, maximum limit}.
     */
    public void setOutputLimits(double[] limits){
        if(limits.length == 2){
            this.outputLimits[0] = limits[0];
            this.outputLimits[1] = limits[1];
        }
    }

    /**
     * Calculates the output based on the current value and the PID algorithm.
     * The output is calculated as a combination of proportional, integral, and derivative terms.
     * 
     * @param currentValue The current error.
     * @return The calculated output that can be used to adjust the system.
     */
    public double calculate(double currentValue){
        // Error is the difference between the setpoint and the current value
        double error = setPoint - currentValue;

        // Proportional term: Kp * error
        double proportional = Kp * error;

        // Integral term: Sum of the error over time, multiplied by Ki
        integralSum += error * dt;
        double integral = Ki * integralSum;

        // Derivative term: Rate of change of the error, multiplied by Kd
        double derivative = Kd * (error-previousError)/dt;

        // Store current error for the next derivative calculation
        previousError = error;

        // PID output = proportional + integral + derivative
        double output = proportional +integral + derivative;

        // Constrain the output to the limits
        if(output >outputLimits[1]){
            output = outputLimits[1];
        }else if (output<outputLimits[0]){
            output =outputLimits[0];
        }

        // Returns the calculated output
        return output;
    }

    /**
     * Resets the integral sum and the previous error.
     * Useful when restarting the PID controller.
     */
    public void reset(){
        this.integralSum = 0;
        this.previousError = 0;
    }

    /**
     * Tunes the PID controller by setting new gain values
     * 
     * @param newKp The new proportional gain.
     * @param newKi The new integral gain.
     * @param newKd The new derivative gain.
     */
    public void tune(double newKp, double newKi, double newKd){
        this.Kp = newKp;
        this.Ki = newKi;
        this.Kd = newKd;
    }
}
