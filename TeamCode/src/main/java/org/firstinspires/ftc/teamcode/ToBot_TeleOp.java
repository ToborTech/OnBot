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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * This OpMode uses the common ToBot hardware class to define the devices on the robot.
 * All device access is managed through the HardwareToBot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a ToBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 *
 */

@TeleOp(name="ToBot: Teleop", group="ToBot")
public class ToBot_TeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareToBot   robot       = new  HardwareToBot();   // Use a ToBot's hardware
    double          speed_ratio = 0.5;                    // ToBot speed (0.1 to 1)

    @Override
    public void runOpMode() {
        double left;
        double right;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, telemetry);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;

            // Output the safe vales to the motor drives.
            robot.leftDrive.setPower(left*speed_ratio);
            robot.rightDrive.setPower(right*speed_ratio);

            // Use gamepad buttons to speed up (Y) and down (A)
            if (gamepad1.y) {
                speed_ratio += 0.05;
                if (speed_ratio>1) speed_ratio = 1;
            }
            else if (gamepad1.a) {
                speed_ratio -= 0.05;
                if (speed_ratio<0.1) speed_ratio = 0.1;
            }

            // Send telemetry message to signify robot running;
            telemetry.addData("speed",  "ratio = %.2f", speed_ratio);
            telemetry.addData("left (enc)",  "%.2f(%d)",
                    left, robot.leftDrive.getCurrentPosition());
            telemetry.addData("right (enc)", "%.2f(%d)",
                    right, robot.rightDrive.getCurrentPosition());
            telemetry.addData("imu", "%.2f", robot.getHeading());
            telemetry.update();
        }
    }
}
