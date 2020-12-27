package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Robot {
    //public BNO055IMU imu_hub1;
    MecanumDriveTrain driveTrain;
    ComputerVision computerVision;
    WobbleGripper wobbleGripper;
    RingControl   ringControl;

    ElapsedTime autonomusTime;
    final LinearOpMode opMode;

    public Robot (LinearOpMode opMode){
        this.opMode      = opMode;
        this.autonomusTime = new ElapsedTime();
        this.driveTrain = new MecanumDriveTrain(this);
        this.computerVision = new ComputerVision(this);
        this.wobbleGripper = new WobbleGripper(this);
        this.ringControl = new RingControl(this);

    }

    public MecanumDriveTrain getDriveTrain()   {return this.driveTrain;}
    public ComputerVision getComputerVision()  {return this.computerVision;}
    public WobbleGripper getWobbleGripper()    {return  this.wobbleGripper;}
    public RingControl getRingControl()        {return  this.ringControl;}
}
