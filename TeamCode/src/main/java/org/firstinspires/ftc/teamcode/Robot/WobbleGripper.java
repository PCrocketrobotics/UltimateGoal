package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.Servo;

public class WobbleGripper {
    static final double MAX_POS     = 1.0;
    static final double MIN_POS     = 0.0;
    Servo   wobbleservo;
    Robot robot;

    public WobbleGripper(Robot robot) {
        this.robot = robot;
    }

    public void init() {
        //Wheel Drive Motors Setup
        wobbleservo = robot.opMode.hardwareMap.get(Servo.class, "wobbleservo");
        wobbleservo.setPosition(0);
    }
    public void DriverControlled_WobbleServo(){
        String wobblepostion = "CLOSE";
        if (robot.opMode.gamepad1.a){
            wobblepostion = "OPEN";
        }
        SetWobbleGripper(wobblepostion);
    }
    private void SetWobbleGripper(String wobbleposition){
        double wobblelocation =0;
        if (wobbleposition == "OPEN"){
            wobblelocation = 0.5;
        }
        if (wobbleposition == "CLOSE"){
            wobblelocation = 0;
        }
        wobbleservo.setPosition(wobblelocation);
        double currentwobbleposition = wobbleservo.getPosition();
        while (currentwobbleposition != wobblelocation){
            //wait until in position
        }

    }
}
