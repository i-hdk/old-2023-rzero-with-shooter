package raidzero.robot.utils;

import edu.wpi.first.math.MathUtil;

public class MathTools {

    public static double wrapDegrees(double degrees) {
        return MathUtil.inputModulus(degrees, -180.0, 180.0);
    }

    public static double[] toRadians(double[] degrees) {
        double[] answerRadians = new double[degrees.length];
        for(int i=0;i<degrees.length;i++){
            answerRadians[i] = Math.toRadians(degrees[i]);
        }
        return answerRadians;
    }

    public static double[] rotationByAngle(double[] vector, double angle){
        double[] rotatedVector = new double[2];
        rotatedVector[0] = Math.cos(angle)*vector[0]-Math.sin(angle)*vector[1];
        rotatedVector[1] = Math.sin(angle)*vector[0]+Math.cos(angle)*vector[1];
        return rotatedVector;
    }

    public static int[] doubleArraytoInt(double[] darray){
        int[] converted = new int[darray.length];
        for(int i =0; i<darray.length;i++){
            converted[i] = (int)darray[i];
        }
        return converted;
    }

    public static double lawOfCosines(double adj1, double adj2, double opp){
        return Math.acos((adj1*adj1+adj2*adj2-opp*opp)/(2*adj1*adj2));
    }
}