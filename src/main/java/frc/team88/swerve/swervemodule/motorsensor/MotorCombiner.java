package frc.team88.swerve.swervemodule.motorsensor;

import java.util.Objects;
import java.util.function.Function;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

/**
 * Takes n PID motors and applies nxn matrix A to them to produce n PID "motors"
 * representing the resulting linear combinations of motors. Useful for
 * representing differential mechanisms, as well as planetary mechanisms where
 * the wheel would normally turn when the module steers. The number of input and
 * output mechanisms is currently required to be the same to avoid
 * over/under-defined systems. Handles the operations that require coorination
 * between motors, such as setting the velocity.
 */
public class MotorCombiner {

    // The input motors to this mechanism
    private PIDMotor[] inputs;

    // The output "motors" of this system
    private OutputMotor[] outputs;

    // The number of inputs. Also the number of outputs.
    private int matrixSize;

    /**
     * Constructor. Private to require that the builder be used.
     * 
     * @param matrixSize
     *                          The number of inputs. Also the number of outputs
     * @param inputs
     *                          All the input motors to this mechanism
     * @param forwardMatrix
     *                          The matrix to multiple the inputs by to get the
     *                          outputs
     */
    private MotorCombiner(int matrixSize, PIDMotor[] inputs, RealMatrix forwardMatrix) {
        // The builder is responsible for valid argument checking
        this.matrixSize = matrixSize;
        this.inputs = inputs;

        // Calculate the matrix inverse, which is used to go from output values
        // to input values
        RealMatrix inverseMatrix = MatrixUtils.inverse(forwardMatrix);

        // Create the output motors using the 2 matrices. In the forward
        // matrix, each column represents an output, while it is each row for
        // the inverse matrix
        this.outputs = new OutputMotor[matrixSize];
        for (int idx = 0; idx < matrixSize; idx++) {
            outputs[idx] = new OutputMotor(forwardMatrix.getRow(idx), inverseMatrix.getColumn(idx));
        }
    }

    /**
     * Get all of the output "motors" of this mechanism.
     * 
     * @return All the outputs
     */
    public PIDMotor[] getOuputs() {
        return outputs;
    }

    /**
     * Get the output "motor" that represents the given row in the matrix.
     * 
     * @param idx
     *                The row in the matrix to get the output for. Zero indexed
     * @return The ouput
     */
    public PIDMotor getOutput(int idx) {
        return outputs[idx];
    }

    /**
     * Updates the set velocity of each input to match the target velocity of each
     * output.
     */
    private void updateVelocities() {
        for (int inputIdx = 0; inputIdx < matrixSize; inputIdx++) {
            double velocity = 0;
            for (OutputMotor output : outputs) {
                velocity += output.getInputTargetVelocity(inputIdx);
            }
            inputs[inputIdx].setVelocity(velocity);
        }
    }

    /**
     * Builds a MotorCombiner. To use:
     * 
     * <ol>
     * <li>Construct with the desired number of inputs, which is also the number of
     * outputs.
     * <li>Add each of the inputs in= order, giving the column that corresponds to
     * it in the forward matrix.
     * <li>Build the MotorCombiner.
     * </ol>
     */
    public static class Builder {

        // The number of inputs. Also the number of outputs
        private int matrixSize;

        // The input motors.
        private PIDMotor[] inputs;

        // The matrix to multiply the inputs by to get the outputs.
        private RealMatrix forwardMatrix;

        private int currentInputToAdd;

        /**
         * Constructor.
         * 
         * @param matrixSize
         *                       The number of inputs for the MotorCombiner. Also it's
         *                       number of outputs
         */
        public Builder(int matrixSize) {
            this.matrixSize = matrixSize;
            this.inputs = new PIDMotor[matrixSize];
            this.forwardMatrix = new Array2DRowRealMatrix(matrixSize, matrixSize);
            currentInputToAdd = 0;
        }

        /**
         * Adds the given input motor to the motor combiner. Inputs are added
         * sequentially each time this method is called. Must be called once and only
         * once for each input expected.
         * 
         * @param input
         *                                The motor to add as an input
         * @param forwardCoefficients
         *                                The column in the forward matrix that
         *                                represents what to multiply this input by to
         *                                get each of the outputs
         * 
         * @return This builder.
         */
        public Builder addInput(PIDMotor input, double... forwardCoefficients) {
            if (currentInputToAdd >= matrixSize) {
                throw new IllegalStateException(
                        "Tried to add too many inputs, expected number of inputs is " + matrixSize);
            }
            if (forwardCoefficients.length != matrixSize) {
                throw new IllegalArgumentException("The number of forward coefficients should be exactly " + matrixSize
                        + ", but only " + forwardCoefficients.length + " were given");
            }
            this.inputs[currentInputToAdd] = Objects.requireNonNull(input);
            for (int row = 0; row < matrixSize; row++) {
                forwardMatrix.setEntry(row, currentInputToAdd, forwardCoefficients[row]);
            }

            ++currentInputToAdd;
            return this;
        }

        /**
         * Builds the MotorCombiner as specified. The number of times addInput was
         * called on this builder must match the matrix size passed into the constructor
         * before this is called.
         * 
         * @return The built MotorCombiner
         */
        public MotorCombiner build() {
            if (currentInputToAdd != matrixSize) {
                throw new IllegalStateException("Not all inputs have been specified. Expected " + matrixSize
                        + ", but only found " + currentInputToAdd);
            }
            return new MotorCombiner(matrixSize, inputs, forwardMatrix);
        }

    }

    /**
     * Represents a single output in this mechanism.
     */
    private class OutputMotor implements PIDMotor {

        // The coefficients to multiply to the value of each input to determine
        // this output.
        private double[] forwardCoefficients;

        // The coefficients to multiply to the value of this output to
        // determine its contribution to each input.
        private double[] inverseCoefficients;

        // This output's target velocity
        private double targetVelocity = 0;

        // The offset to add to position values.
        private double offset = 0;

        /**
         * Constructor.
         * 
         * @param forwardCoefficients
         *                                The coefficients to multiply to the value of
         *                                each input to determine this output
         * 
         * @param inverseCoefficients
         *                                The coefficients to multiply to the value of
         *                                this output to determine its contribution to
         *                                each input
         */
        public OutputMotor(double[] forwardCoefficients, double[] inverseCoefficients) {
            // The builder is responsible for valid argument checking
            this.forwardCoefficients = forwardCoefficients;
            this.inverseCoefficients = inverseCoefficients;
        }

        @Override
        public double getPosition() {
            return applyForwardCoefficients((Integer idx) -> MotorCombiner.this.inputs[idx].getPosition())
                    + this.offset;
        }

        @Override
        public double getVelocity() {
            return applyForwardCoefficients((Integer idx) -> MotorCombiner.this.inputs[idx].getVelocity());
        }

        @Override
        public void calibratePosition(double position) {
            this.offset = position - this.getPosition() + this.offset;
        }

        @Override
        public void setVelocity(double velocity) {
            this.targetVelocity = velocity;
            MotorCombiner.this.updateVelocities();
        }

        /**
         * Gets the contribution of target velocity for the input with the given index.
         * 
         * @param inputIdx
         *                     the index of the input to get the velocity component for.
         *                     Zero indexed
         * @return The component velocity for this output on the given input
         */
        public double getInputTargetVelocity(int inputIdx) {
            return this.inverseCoefficients[inputIdx] * this.targetVelocity;
        }

        /**
         * Sums the product of each forward coefficient with the value returned by
         * inputValueGenerator given the coefficient's index.
         * 
         * @param inputValueGenerator
         *                                A function which takes the index of a forward
         *                                coefficient (and therefore the index of an
         *                                input) and returns the value to be multiplied
         *                                by that coefficient
         * @return The sum of products
         */
        private double applyForwardCoefficients(Function<Integer, Double> inputValueGenerator) {
            double outputValue = 0;
            for (int idx = 0; idx < MotorCombiner.this.matrixSize; idx++) {
                outputValue += this.forwardCoefficients[idx] * inputValueGenerator.apply(idx);
            }
            return outputValue;
        }
    }
}
