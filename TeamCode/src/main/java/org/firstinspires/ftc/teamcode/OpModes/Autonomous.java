package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.graphics.Color;

import org.firstinspires.ftc.teamcode.Robot.Robot;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous", group="Production")

public class Autonomous extends LinearOpMode {
    Robot robot = new Robot (this);
    int Target_Zone;

    private boolean inInitializationState() {
        return (!opModeIsActive() && !isStopRequested());
    }
    @Override
    public void runOpMode(){
        String ringpattern = "";
        robot.getDriveTrain().init();
        robot.getComputerVision().init();
        sleep(500);
        
        ElapsedTime timer = new ElapsedTime();
        int count = 0;

        while (inInitializationState()) {

        }
        if (opModeIsActive()) {
            //robot.getDriveTrain().AutonomousDrive(10, 10, 10, 10, .5);
            while (opModeIsActive()) {
                ringpattern = robot.getComputerVision().detect();
                sleep(500);

                if (ringpattern == "None") {
                    Target_Zone = 0 ;
                }
                else if (ringpattern == "Single") {
                    Target_Zone = 1 ;
                }
                else if (ringpattern == "Quad") {
                    Target_Zone = 4 ;
                }
                telemetry.addData("ring patter", ringpattern);
                telemetry.addData("TargetZone", Target_Zone);
                telemetry.addData("", "----------------------------");
                telemetry.addData("sec", String.format("%.2f", timer.seconds()));

                telemetry.addData(">", "Press Play to Start");
                telemetry.update();
                sleep(1800);
                if (Target_Zone == 0) {
                    break;
                }
                else if (Target_Zone == 1) {
                    break;
                }
                else if (Target_Zone == 4) {
                    break;
                }

            }
            robot.getDriveTrain().gyroDrive(1000, 1000, 1000, 1000, .25, 1);



        }

    }
}
