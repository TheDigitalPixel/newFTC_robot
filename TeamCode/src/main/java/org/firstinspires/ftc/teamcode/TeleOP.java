package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

@TeleOp
public class TeleOP extends LinearOpMode{

    private DcMotor motorWheelFL;
    private DcMotor motorWheelFR;
    private DcMotor motorWheelBL;
    private DcMotor motorWheelBR;

    public TeleOp{

        motorWheelFL = hardwareMap.get(DcMotor.class, "motorWheelFL");
        motorWheelFR = hardwareMap.get(DcMotor.class, "motorWheelFR");
        motorWheelBL = hardwareMap.get(DcMotor.class, "motorWheelBL");
        motorWheelBR = hardwareMap.get(DcMotor.class, "motorWheelBR");

        motorWheelFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorWheelFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorWheelFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorWheelFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorWheelBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorWheelBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorWheelBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorWheelBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void drive() {

        double leftStickScale = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double stickAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double BLSpeed = leftStickScale * (Math.sin(stickAngle) - Math.cos(stickAngle));
        final double BRSpeed = leftStickScale * (Math.sin(stickAngle) + Math.cos(stickAngle));
        final double FLSpeed = -leftStickScale * (Math.sin(stickAngle) + Math.cos(stickAngle));
        final double FRSpeed = leftStickScale * (Math.sin(stickAngle) - Math.cos(stickAngle));

        motorWheelBL.setPower(BLSpeed);
        motorWheelBR.setPower(BRSpeed);
        motorWheelFL.setPower(FLSpeed);
        motorWheelFR.setPower(FRSpeed);

    }

    @Override
    public void runOpMode() throws InterruptedException {

        TeleOp dicko = new TeleOp();
        dicko.drive();



        
	telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        float startPositionL = 0;
        float startPositionR = 0;
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");


            telemetry.addData("FL Motor Power", motorWheelFL.getPower());
            telemetry.addData("FR Motor Power", motorWheelFR.getPower());
            telemetry.addData("BL Motor Power", motorWheelBL.getPower());
            telemetry.addData("BR Motor Power", motorWheelBR.getPower());
            telemetry.update();


            telemetry.addData("Left_trigger", this.gamepad2.left_trigger);
            telemetry.addData("Right_trigger", this.gamepad2.right_trigger);
//            telemetry.addData("motorLift", motorLift.getCurrentPosition());
//            telemetry.addData("motorExtenderLeft", motorExtenderLeft.getCurrentPosition());
//            telemetry.addData("motorExtenderRight", motorExtenderRight.getCurrentPosition());
//            telemetry.addData("motorDeposit", motorDeposit.getCurrentPosition());
//            telemetry.update();
        }
    }
    public void forward(){

    }
}
