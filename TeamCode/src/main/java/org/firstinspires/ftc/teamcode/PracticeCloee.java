package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp

public class PracticeCloee extends LinearOpMode{

 PracticeCloeeHardware robot = new PracticeCloeeHardware();
 
 @Override
 public void runOpMode(){
      
        robot.init(hardwareMap);
        
        waitForStart();
        
        while(opModeIsActive()){
            // Enter teleop inputs here
            robot.driveMotors(gamepad1.left_stick_y, gamepad1.right_stick_y);
        }
    }
}
