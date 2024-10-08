public class PID {
    private double Kp, Ki, Kd;
    private double previousError;
    private double integralSum;
    private double setPoint;
    private double dt;
    private double[] outputLimits = {Double.NEGATIVE_INFINITY,Double.POSITIVE_INFINITY};

    public PID(double Kp, double Ki, double Kd, double dt){
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.dt = dt;
        this.previousError = 0;
        this.integralSum = 0;
    }

    public void setSetpoint(double setPoint){
        this.setPoint = setPoint;
    }

    public void setOutputLimits(double[] limits){
        if(limits.length == 2){
            this.outputLimits[0] = limits[0];
            this.outputLimits[1] = limits[1];
        }
    }

    public double calculate(double currentValue){

        double error = setPoint - currentValue;

        double proportional = Kp * error;

        integralSum += error * dt;
        double integral = Ki * integralSum;

        double derivative = Kd * (error-previousError)/dt;

        previousError = error;
        double output = proportional +integral + derivative;

        if(output >outputLimits[1]){
            output = outputLimits[1];
        }else if (output<outputLimits[0]){
            output =outputLimits[0];
        }

        return output;
    }

    public void reset(){
        this.integralSum = 0;
        this.previousError = 0;
    }

    public void tune(double newKp, double newKi, double newKd){
        this.Kp = newKp;
        this.Ki = newKi;
        this.Kd = newKd;
    }
}
