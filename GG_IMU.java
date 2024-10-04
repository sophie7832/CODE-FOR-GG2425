package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="Catlord")
public class GG_IMU extends LinearOpMode {

    private DcMotor fl = null;
    private DcMotor fr = null;
    private DcMotor bl = null;
    private DcMotor br = null;

    private IMU imu = null;

    private ElapsedTime runtime = new ElapsedTime();

    private double headingError = 0;

    private double targetHeading = 0;
    private double driveSpeed = 0;
    private double turnSpeed = 0;
    private double flSpeed = 0;
    private double frSpeed = 0;
    private double blSpeed = 0;
    private double brSpeed = 0;

    private int flTarget = 0;
    private int frTarget = 0;
    private int blTarget = 0;
    private int brTarget = 0;

    static final double COUNTS_PER_MOTOR_REV = 384.5;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 3.77952;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double DRIVE_SPEED = 0.8;
    static final double TURN_SPEED = 0.7;
    static final double HEADING_THRESHOLD = 0.5;

    static final double P_TURN_GAIN = 0.02;
    static final double P_DRIVE_GAIN = 0.03;

    @Override
    public void runOpMode() {
        fr = hardwareMap.get(DcMotor.class, "fR");
        fl = hardwareMap.get(DcMotor.class, "fL");
        br = hardwareMap.get(DcMotor.class, "bR");
        bl = hardwareMap.get(DcMotor.class, "bL");


        br.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.FORWARD);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();
        }

        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu.resetYaw();

        driveStraight(DRIVE_SPEED, 24, )

        telemetry.addData("Path", "Complete")
        telemetry.update();
        sleep(1000);
    }

    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        if (opModeIsActive()) {

            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            flTarget = fl.getCurrentPosition() + moveCounts;
            frTarget = fr.getCurrentPosition() + moveCounts;
            blTarget = bl.getCurrentPosition() + moveCounts;
            brTarget = br.getCurrentPosition() + moveCounts;

            fr.setTargetPosition(frTarget);
            fl.setTargetPosition(flTarget);
            br.setTargetPosition(brTarget);
            bl.setTargetPosition(blTarget);

            fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            while (opModeIsActive()) &&
            (fr.isBusy() && (fl.isBusy() && (br.isBusy() && (bl.isBusy())))) {

                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                if (distance < 0)
                    turnSpeed *= -1.0;
                moveRobot(driveSpeed, turnSpeed);

                sendTelemetry(false);
            }

            moveRobot(0, 0);
            fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        }

        public void turnToHeading(double maxTurnSpeed, double heading) {

            getSteeringCorrection(heading, P_DRIVE_GAIN);

            while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD);
            {

                turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

                turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

                moveRobot(0, turnSpeed);

                sendTelemetry(false);
            }
            moveRobot(0, 0,);

        }
  public void holdHeading(double maxTurnSpeed, double heading, double holdTime){
            ElapsedTime holdTimer = new ElapsedTime();
            holdTimer.reset();

            while (opModeIsActive() && (holdTimer() < holdTime)) {
                turnSpeed = getSterringCorrection(heading, P_TURN_GAIN);

                turnSpeed = Range.clip(turnSpeed, -maxSpeed, maxSpeed);

                moveRobot(0, turnSpeed);

                sendTelemetry(false);
            }
            moveRobot(0, 0);

        }

        public void moveRobot(double drive, double turn) {
            driveSpeed = drive;
            turnSpeed = turn;

            flSpeed = drive - turn;
            blSpeed = drive - turn;
            frSpeed = drive + turn;
            brSpeed = drive + turn;


            double max = Math.max(Math.abs(flSpeed), Math.abs(frSpeed));
            if (max > 1.0)
            {
                flSpeed /= max;
                frSpeed /= max;
            }

            fl.setPower(flSpeed);
            bl.setPower(blSpeed);
            fr.setPower(frSpeed);
            br.setPower(brSpeed);

            }
       public double getSteeringCorrection(double desireHeading, double proportionalGain) {
            targetHeading = desireHeading;

            headingError = targetHeading - getHeading();

            while (headingError > 180) headingError -= 360;
            while (headingError <= -180) headingError += 360;

            return Range.clip(headingError * proportionalGain, -1, 1);
        }
   private void sendTelemetry(boolean straight) {
            if (straight) {
                telemetry.addData("Motion", "Drive Straight");
                telemetry.addData("Target Pos fL:fR:bL:bR", "%7d:%7d:%7d:%7d", fl, fr, bl, br);
                telemetry.addData("Actual Pos fL:fR:bL:bR", "%7d:%7d:%7d:%7d", fl.getCurrentPosition(),
                        fr.getCurrentPosition(), bl.getCurrentPosition(), br.getCurrentPosition());
                else{
                    telemetry.addData("Motion", "Turning");
                }
                telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
                telemetry.addData("Error : Steer Pwr", "%5.1f : %5.1f", headingError, turnSpeed);
                telemetry.addData("Wheel Speeds fl : fR : bL : bR", "%5.2f : %5.2f : %5.2f : %5.2f", flSpeed, blSpeed, frSpeed, blSpeed);
                telemetry.update();
            }
       public double getHeading() {
                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                return orientation.getYaw(AngleUnit.DEGREES);
            }
        }
    }
}
