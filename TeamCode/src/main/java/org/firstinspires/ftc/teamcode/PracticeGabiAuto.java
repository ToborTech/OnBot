package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous

public class PracticeGabiAuto extends LinearOpMode{
    PracticeGabiHardware robot = new PracticeGabiHardware();
    
    @Override
    public void runOpMode() throws InterruptedException{
        
        robot.init(hardwareMap);
        
        waitForStart();
        
        if(opModeIsActive()){
            //Add in autonomous funtions here 
            robot.straightForTime(0.5, 1500);
            
        
        }    
     
    }
}
