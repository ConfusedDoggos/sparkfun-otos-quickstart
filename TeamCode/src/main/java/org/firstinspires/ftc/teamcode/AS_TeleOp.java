package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Timer;

@Config
@TeleOp(name = "AS_TeleOp", group = "TeleOp")
public class AS_TeleOp extends LinearOpMode {
    public static double iCOpen = 0.7;
    public static double iCClose = 0.55;
    public static double iCAlign = 0.57;
    public static double iWTransferPos = 0.84;
    public static double iWAlteredPos = 0;
    public static double iWAlignmentPos = 0;
    public static double iAUp = 0;
    public static double iADown = 0.64;
    public static double iAReady = 0.5;
    public static double dCOpen = 0.5;
    public static double dCClose = 0.35;
    public static double dWTransfer = 1;
    public static double dWDeliverBucket = 0.2;
    public static double dWDeliverSpecimen = 0;
    public static double dWStartPos = 0.8;
    public static int TransferDelay1 = 250;
    public static int TransferDelay2 = 500;
    public boolean dpadDownToggle = false;
    public boolean dpadLeftToggle = false;
    public boolean dpadRightToggle = false;
    public boolean xToggle = false;
    public boolean yToggle = false;
    public boolean isDeliveryOpen = false;
    public double deliveryWristPosition;
    boolean isIntakeOpen;
    private DcMotor rightBack;
    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor horizontalSlideMotor;
    private DcMotor verticalSlideMotor;
    private Servo intakeClaw;
    private Servo intakeWrist;
    private Servo intakeArm;
    private Servo deliveryWrist;
    private Servo deliveryClaw;


    double horizontal;
    double vertical;
    double pivot;
    int vertPos;
    int horizPos;
    double targetPos;
    double intakeWristTargetPos;
    boolean freeWristRotate;
    double downCounter;


    public AS_TeleOp() {
    }


    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        horizontalSlideMotor = hardwareMap.get(DcMotor.class, "horizontalSlideMotor");
        verticalSlideMotor = hardwareMap.get(DcMotor.class, "verticalSlideMotor");
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        intakeWrist = hardwareMap.get(Servo.class, "intakeWrist");
        intakeArm = hardwareMap.get(Servo.class, "intakeArm");
        deliveryWrist = hardwareMap.get(Servo.class, "deliveryWrist");
        deliveryClaw = hardwareMap.get(Servo.class, "deliveryClawR");


        initMotorModes();
        initServoModes();
        //Initialization: All servos should be set to default position

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                moveVariables();
                if (gamepad2.right_trigger > 0.5 || gamepad1.left_stick_button) {
                    horizontal = (gamepad1.left_stick_x * 0.5);
                }
                speedSettings();
                mecanumMath();
                if (gamepad1.a) {
                    deliveryWrist.setPosition(dWTransfer);
                    deliveryClaw.setPosition(dCOpen);
                    intakeWrist.setPosition(iWTransferPos);
                    intakeArm.setPosition(iAUp);
                    new Timer().schedule(
                            new java.util.TimerTask() {
                                @Override
                                public void run() {
                                    deliveryClaw.setPosition(dCClose);
                                    isDeliveryOpen = false;
                                    new Timer().schedule(
                                            new java.util.TimerTask() {
                                                @Override
                                                public void run() {
                                                    intakeClaw.setPosition(iCOpen);
                                                    isIntakeOpen = true;
                                                }
                                            }, TransferDelay1
                                    );
                                }
                            }, TransferDelay2
                    );
                }
                if (gamepad1.x && !xToggle) {
                    xToggle = true;
                    if (isDeliveryOpen) {
                        deliveryClaw.setPosition(dCClose);
                        isDeliveryOpen = false;
                    } else {
                        deliveryClaw.setPosition(dCOpen);
                        isDeliveryOpen = true;
                    }
                }
                if (gamepad1.y && !yToggle) {
                    yToggle = true;
                    if (isIntakeOpen) {
                        intakeClaw.setPosition(iCClose);
                        isIntakeOpen = false;
                    } else {
                        intakeClaw.setPosition(iCOpen);
                        isIntakeOpen = true;
                    }
                }
                if (gamepad1.dpad_left && !dpadLeftToggle) {
                    dpadLeftToggle = true;
                    deliveryWristPosition = deliveryWrist.getPosition();
                    telemetry.addData("deliveryWristPos",deliveryWrist.getPosition());
                    telemetry.update();
                    if (deliveryWristPosition == dWTransfer || deliveryWristPosition == dWStartPos) {
                        deliveryWrist.setPosition(dWDeliverSpecimen);
                    } else if (deliveryWristPosition == dWDeliverSpecimen){
                        deliveryWrist.setPosition(dWTransfer);
                    }
                }
                if (gamepad1.dpad_right && !dpadRightToggle) {
                    dpadRightToggle = true;
                    deliveryWristPosition = deliveryWrist.getPosition();
                    telemetry.addData("deliveryWristPos",deliveryWrist.getPosition());
                    telemetry.update();
                    if (deliveryWristPosition == dWTransfer || deliveryWristPosition == dWStartPos) {
                        deliveryWrist.setPosition(dWDeliverBucket);
                    } else if (deliveryWristPosition == dWDeliverBucket){
                        deliveryWrist.setPosition(dWTransfer);
                    }
                }
                if (gamepad1.b) {
                    deliveryWrist.setPosition(dWTransfer);
                }
                if (gamepad1.dpad_up) {
                    downCounter = 0;
                    intakeArm.setPosition(iAUp);
                    freeWristRotate = false;
                    deliveryWrist.setPosition(dWTransfer);
                    deliveryClaw.setPosition(dCOpen);
                    isDeliveryOpen = true;
                    intakeWrist.setPosition(iWTransferPos);
                    intakeWristTargetPos = 0.84;
                } else if (gamepad1.dpad_down && !dpadDownToggle) {
                    downCounter += 1;
                    dpadDownToggle = true;
                    freeWristRotate = true;
                    if (downCounter == 1) {
                        intakeArm.setPosition(iAReady);
                        intakeClaw.setPosition(iCOpen);
                        isIntakeOpen = true;
                    } else if (downCounter == 2) {
                        intakeArm.setPosition(iADown);
                    } else if (downCounter > 2) {
                        downCounter = 1;
                        intakeArm.setPosition(iAReady);
                        intakeClaw.setPosition(iCOpen);
                        isIntakeOpen = true;
                    }
                }
                if (vertPos > 3000 && gamepad1.right_stick_y < 0.01) {
                    verticalSlideMotor.setPower(0.3);
                } else if (gamepad1.right_stick_y > 0) {
                    verticalSlideMotor.setPower(-gamepad1.right_stick_y * 0.4);
                } else {
                    verticalSlideMotor.setPower(-gamepad1.right_stick_y * 0.7);
                }
                if (freeWristRotate) {
                    if (gamepad1.left_trigger > 0.1) {
                        intakeWristTargetPos += 0.01 * gamepad1.left_trigger;
                    } else if (gamepad1.right_trigger > 0.1) {
                        intakeWristTargetPos -= 0.005 * gamepad1.right_trigger;
                    }
                    if (intakeWristTargetPos > 1) {
                        intakeWristTargetPos = 1;
                    } else if (intakeWristTargetPos < 0) {
                        intakeWristTargetPos = 0;
                    }
                    intakeWrist.setPosition(intakeWristTargetPos);
                }
                horizPos = horizontalSlideMotor.getCurrentPosition();
                horizontalSlideMotor.setPower(-0.3 * gamepad1.left_stick_y);
                toggleVariables();
                vertPos = verticalSlideMotor.getCurrentPosition();
                telemetry.addData("vertSlidePos", vertPos);
                telemetry.addData("CurrentServoPos", targetPos);
                telemetry.addData("intakeWristRotation", intakeWristTargetPos);
                telemetry.addData("horizontalSlidePos", horizPos);
                telemetry.addData("horizontalSlidePower", horizontalSlideMotor.getPower());
                telemetry.addData("LeftStickY", gamepad1.left_stick_y);
                telemetry.addData("downCounter", downCounter);
                telemetry.update();
            }
        }
    }

    /**
     * Describe this function...
     */

    private void moveVariables() {
        if (gamepad2.left_stick_y > 0.1 || gamepad2.left_stick_y < -0.1) {
            vertical = -gamepad2.left_stick_y;
        } else {
            vertical = 0;
        }
        if (gamepad2.left_stick_x > 0.1 || gamepad2.left_stick_x < -0.1) {
            horizontal = gamepad2.left_stick_x;
        } else {
            horizontal = 0;
        }
        pivot = -gamepad2.right_stick_x;
        if (gamepad2.dpad_down) {
            horizontal = gamepad1.left_stick_x;
        }
    }

    private void mecanumMath() {
        rightFront.setPower(-(pivot * 0.5) - (vertical * 0.5 - horizontal * 0.5));
        rightBack.setPower(-(pivot * 0.5) - (vertical * 0.5 + horizontal * 0.5));
        leftFront.setPower(pivot * 0.5 - (vertical * 0.5 + horizontal * 0.5));
        leftBack.setPower(pivot * 0.5 - (vertical * 0.5 - horizontal * 0.5));
    }

    private void initMotorModes() {
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontalSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        verticalSlideMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Describe this function...
     */
    private void speedSettings() {
        if (gamepad2.right_bumper) {
            horizontal = horizontal * 2;
            vertical = vertical * 2;
            pivot = pivot * 2;
        }
        if (gamepad2.left_bumper) {
            horizontal = (float) (horizontal * 0.5);
            vertical = (float) (vertical * 0.5);
            pivot = (float) (pivot * 0.5);
        }
    }

    private void initServoModes() {
        intakeClaw.setPosition(iCClose);
        isIntakeOpen = false;
        intakeWristTargetPos = 0.84;
        intakeWrist.setPosition(intakeWristTargetPos);
        intakeArm.setPosition(iAUp);
        deliveryWrist.setPosition(dWStartPos);
        deliveryClaw.setPosition(dCClose);
        isDeliveryOpen = false;
    }

    private void toggleVariables() {
        if (!gamepad1.x) {
            xToggle = false;
        }
        if (!gamepad1.y) {
            yToggle = false;
        }
        if (!gamepad1.dpad_down) {
            dpadDownToggle = false;
        }
        if (!gamepad1.dpad_left) {
            dpadLeftToggle = false;
        }
        if (!gamepad1.dpad_right) {
            dpadRightToggle = false;
        }
    }
}
