package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name = "ToBot: autonomous", group = "ToBot")
public class ToBot_Auto extends LinearOpMode {

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

        //TODO write your auto codes here
        for (int i = 0; (i < 4) && opModeIsActive(); i++) {
            robot.forward(0.5, 20, 3);
            sleep(500);
            robot.turn(0.3, 90, 2); // left turn 90
            sleep(500);
        }
    }
}
