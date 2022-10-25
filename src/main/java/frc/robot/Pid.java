package frc.robot;

public class Pid {
    
    static double P = .5;
    static double I = 1;
    static double D = 1;
    static double[] previous_error = new double[5];
    static double integral = 0;
    static double feedforward = 0;

        public static double execute(double setPoint, double currentPoint) {
            //P
            double error = setPoint - currentPoint; // Error = Target - Actual

            //I
            for(int i = 0; i < 4; i++) {
                previous_error[i] += previous_error[i+1];
            }
            previous_error[4] = error;
            double totalError = 0;
            for(int i = 0; i < previous_error.length; i++) {
                    totalError += previous_error[i];
            }
            totalError /= previous_error.length;
            totalError *= .02;

            //D
            double derivative = (error - totalError) / .02;

            //F
            feedforward = setPoint / 5000;

            double output = P * error + feedforward;
            // + I * totalError + D * derivative
            return output;
        }

        public void ReadXML(){

        }
}
