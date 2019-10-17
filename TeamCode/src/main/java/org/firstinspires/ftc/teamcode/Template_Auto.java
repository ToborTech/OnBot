package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name = "ToBot: autonomous", group = "ToBot")
public class Template_Auto extends LinearOpMode {

    @Override
    public void runOpMode() {

        HardwareToBot robot = new HardwareToBot();   // Use a ToBot's hardware
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, telemetry);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Forward for 20 inches using 0.5 power, with 3 seconds time out
        robot.forward(0.5, 20, 3);

	// wait for 0.5 sec
        sleep(500);

        // Left turn for 90 degree using 0.3 power with 2 seconds time out
        robot.turn(0.3, 90, 2); // left turn 90

    }
}
