package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static java.lang.Thread.sleep;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled


public class PracticeGabiHardware {
    // Define new hardware, sensors, and variables here 
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;
    public Servo leftServo = null;
    public Servo rightServo = null;
    
    HardwareMap hwMap = null; 
    
    
    public PracticeGabiHardware(){} // Constructor
    
    public void init(HardwareMap ahwMap){
                // Save reference to Hardware map
        hwMap = ahwMap;
        
        // Define and Initialize Motors 
        leftMotor  = hwMap.dcMotor.get("left_drive");
        rightMotor = hwMap.dcMotor.get("right_drive");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        //Set all motors to zero power 
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    public void driveMotors(double lp, double rp){
        leftMotor.setPower(lp);
        rightMotor.setPower(rp);
    }
    public void straightForTime(double pow, long ms ) throws InterruptedException{
        driveMotors(pow,pow);
        sleep(ms);
        driveMotors(0,0);
    }
    
}
