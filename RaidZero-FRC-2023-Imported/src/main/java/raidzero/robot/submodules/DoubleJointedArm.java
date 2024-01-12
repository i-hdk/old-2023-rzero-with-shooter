package raidzero.robot.submodules;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.Discretization;
import raidzero.robot.Constants.ArmConstants;

public class DoubleJointedArm {

    private double l1, l2, m1, m2, r1, r2, I1, I2, g;
    private double t_s, I_s, w_f, gr1, gr2, n1, n2;

    public DoubleJointedArm() {
        this.l1 = ArmConstants.LOWER_ARM_LENGTH;
        this.l2 = ArmConstants.UPPER_ARM_LENGTH;
        this.m1 = ArmConstants.LOWER_ARM_WEIGHT;
        this.m2 = ArmConstants.UPPER_ARM_WEIGHT;
        this.r1 = ArmConstants.BASE_PIVOT_COG;
        this.r2 = ArmConstants.JOINT_COM;
        this.I1 = ArmConstants.LOWER_ARM_MOI;
        this.I2 = ArmConstants.UPPER_ARM_MOI;
        this.g = ArmConstants.GRAVITY;
        this.t_s = ArmConstants.STALL_TORQUE;
        this.I_s = ArmConstants.STALL_CURRENT;
        this.w_f = ArmConstants.FREE_SPEED;
        this.gr1 = ArmConstants.BASE_PIVOT_GEAR_RATIO;
        this.gr2 = ArmConstants.JOINT_GEAR_RATIO;
        this.n1 = ArmConstants.BASE_PIVOT_MOTORS;
        this.n2 = ArmConstants.JOINT_MOTORS;
    }

    /**
     * Gets the dynamic matrices for the given state.
     *
     * See derivation at:
     * https://drive.google.com/file/d/1aQRhb6v1i_I2EpNPW0JvvFgWaK2D3KYM/view?usp=share_link
     *
     * Keyword arguments:
     * states -- current system state
     */
    private double[][][] getDynamicsMatrices(double[] states) {
        double theta1 = states[0];
        double theta2 = states[1];
        double omega1 = states[2];
        double omega2 = states[3];
        double c2 = Math.cos(theta2);

        // Define Mass Matrix
        double hM = l1 * r2 * c2;
        // M1 Terms
        double[][] M = { { m1 * r1 * r1, 0 }, { 0, 0 } };
        // M2 Terms
        M[0][0] += m2 * (l1 * l1 + r2 * r2 + 2 * hM);
        M[0][1] += m2 * (r2 * r2 + hM);
        M[1][0] += m2 * (r2 * r2 + hM);
        M[1][1] += m2 * r2 * r2;
        // MOI Terms
        M[0][0] += I1 + I2;
        M[0][1] += I2;
        M[1][0] += I2;
        M[1][1] += I2;

        // Define Velocity Product Term
        double hC = -m2 * l1 * r2 * Math.sin(theta2);
        double[][] C = {
                { hC * omega2, hC * omega1 + hC * omega2 },
                { -hC * omega1, 0 },
        };

        // Define Gravity Term
        double hG = g * Math.cos(theta1 + theta2) * m2 * r2;
        double[][] G = {
                { g * Math.cos(theta1) * (m1 * r1 + m2 * l1) + hG },
                { hG },
        };

        // Define B Matrix
        double[][] B = {
                { gr1 * n1 * (t_s / I_s) / (12 / I_s), 0 },
                { 0, gr2 * n2 * (t_s / I_s) / (12 / I_s) },
        };

        // Define Kb Matrix
        double[][] Kb = {
                { gr1 * gr1 * n1 * (t_s / I_s) / (w_f / I_s), 0 },
                { 0, gr2 * gr2 * n2 * (t_s / I_s) / (w_f / I_s) },
        };

        return new double[][][] { M, C, G, B, Kb };
    }

    /**
     * Calculates feedforward voltage for each joint to a target state
     *
     * Keyword arguments:
     * states -- target system state
     * accels -- current system acceleration (normally zero)
     */
    public Matrix<N2, N1> feedForward(double[] states, double[][] accels) {
        double[][][] dM = getDynamicsMatrices(states);

        Matrix<N2, N2> M = new MatBuilder<N2, N2>(Nat.N2(), Nat.N2())
                .fill(dM[0][0][0], dM[0][0][0], dM[0][1][0], dM[0][1][1]);
        Matrix<N2, N2> C = new MatBuilder<N2, N2>(Nat.N2(), Nat.N2())
                .fill(dM[1][0][0], dM[1][0][1], dM[1][1][0], dM[1][1][1]);
        Matrix<N2, N1> G = new MatBuilder<N2, N1>(Nat.N2(), Nat.N1())
                .fill(dM[2][0][0], dM[2][1][0]);

        Matrix<N2, N1> omegas = new MatBuilder<N2, N1>(Nat.N2(), Nat.N1())
                .fill(states[2], states[3]);
        Matrix<N2, N1> accel = new MatBuilder<N2, N1>(Nat.N2(), Nat.N1())
                .fill(accels[0][0], accels[1][0]);
        Matrix<N2, N2> B_inv = new MatBuilder<N2, N2>(Nat.N2(), Nat.N2())
                .fill(dM[3][0][0], dM[3][0][1], dM[3][1][0], dM[3][1][1]);
        Matrix<N2, N2> Kb = new MatBuilder<N2, N2>(Nat.N2(), Nat.N2())
                .fill(dM[4][0][0], dM[4][0][1], dM[4][1][0], dM[4][1][1]);

        Matrix<N2, N1> A = M
                .times(accel)
                .plus(C.times(omegas))
                .plus(G)
                .plus(Kb.times(omegas));

        return B_inv.times(A);
    }
}
