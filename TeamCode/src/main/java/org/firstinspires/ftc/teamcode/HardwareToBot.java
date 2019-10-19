/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case the robot name is Tobot. The file also defines routines that can be
 * used to control ToBot during autonomous and teleop modes, e.g. forwardInches()
 * and rotateDegrees().
 * See ToBot_TeleOp and others classes starting with "ToBot" for usage examples.
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 * <p>
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 */
public class HardwareToBot {
    /* Public OpMode members. */
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public BNO055IMU imu = null;

    final double INCHES_PER_ROTATION = 3.5 * Math.PI; // assume 3.5-inches wheels
    // final double COUNT_PER_ROTATION = 1120;   // for NeveRest 40 motor
    final double COUNT_PER_ROTATION = 288;       // for Rev Core Hex motor
    final double COUNT_PER_INCHES = COUNT_PER_ROTATION / INCHES_PER_ROTATION;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    Telemetry tel = null;
    private ElapsedTime runtime = new ElapsedTime();

    /* Constructor */
    public HardwareToBot() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, Telemetry tel) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        this.tel = tel;

        // Define and Initialize Motors
        leftDrive = hwMap.get(DcMotor.class, "left_drive");
        // TODO: please initialize rightDrive here

        leftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        // TODO: Set the rightDrive to REVERSE

        // Set all motors to zero power
        leftDrive.setPower(0);
        // TODO: set rightDrive power to 0

        // reset encoder value
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // TODO: set mode for rightDrive as well

        initializeIMU();
    }

    // move forward/backward for a given distance (in inches) with given power speed
    // if the operation takes longer than timeoutS, it aborts the routine and return
    void forward(double power, double distance, double timeoutS) {
        runtime.reset();

        // positive power will move forward, negative power will move backward
        if (power > 1) power = 1;
        if (power < -1) power = -1;

        int target_count = (int) (distance * COUNT_PER_INCHES * Math.signum(power));

        int newLeftTarget = leftDrive.getCurrentPosition() + target_count;
        int newRightTarget = rightDrive.getCurrentPosition() + target_count;

        leftDrive.setTargetPosition(newLeftTarget);
        rightDrive.setTargetPosition(newRightTarget);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(Math.abs(power));
        rightDrive.setPower(Math.abs(power));

        while ((runtime.seconds() < timeoutS) &&
                (leftDrive.isBusy() && rightDrive.isBusy())) {

            // Display it for the driver.
            tel.addData("Wheels", "Running to %7d :%7d", newLeftTarget, newRightTarget);
            tel.addData("Wheels", "Running at %7d :%7d",
                    leftDrive.getCurrentPosition(),
                    rightDrive.getCurrentPosition());
            tel.update();
        }

        // Stop all motion;
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    void initializeIMU() {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    double getHeading() {
        Orientation orientation = imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES
        );
        return -orientation.firstAngle;
    }

    // Turn the robot using IMU heading to determine the degree
    //
    void turn(double power, double degree, double timeoutS) {
        final double IMU_PRECISION = 0.5;

        runtime.reset();

        double tar_degree = getHeading() + degree;
        double cur_degree = getHeading();
        boolean gap = tar_degree >= 180.0 || tar_degree < -180.0;
        if (degree > 0) { // right turn
            leftDrive.setPower(Math.abs(power));
            rightDrive.setPower(-1 * Math.abs(power));
        } else { // left turn
            leftDrive.setPower(-1 * Math.abs(power));
            rightDrive.setPower(Math.abs(power));
        }
        double newHeading;
        while (runtime.seconds() < timeoutS &&
                Math.abs(cur_degree - tar_degree) > IMU_PRECISION) {
            // Display it for the driver.
            tel.addData("imu/target =", "%2.1f /%2.1f", cur_degree, tar_degree);
            tel.update();
            newHeading = getHeading();
            //running over the gap
            if (gap && (cur_degree * newHeading < -100.0)) {
                if (degree>0) {
                    tar_degree -= 360.0;
                } else {
                    tar_degree += 360.0;
                }
                gap = false;
            }
            if (degree > 0) {
                if (newHeading >= tar_degree)
                    break;
            } else {
                if (newHeading <= tar_degree)
                    break;
            }
            cur_degree = newHeading;
        }

        // Stop all motion;
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
}

