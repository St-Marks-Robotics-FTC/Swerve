import time
import math

def normalize_degrees(degrees):
    return (degrees + 180) % 360 - 180

def run_swerve_drive():
    kP = 0.0
    kI = 0.0
    kD = 0.0

    MOTOR_FLIPPING = False

    FLoffset = 0.0
    FRoffset = 0.0
    BLoffset = 0.0
    BRoffset = 0.0

    wheelbase = 9.492126
    trackwidth = 9.492126
    r = math.sqrt(wheelbase**2 + trackwidth**2)

    # while True:
    fwd = 0            # -gamepad1.left_stick_y  # Pushing joystick up is negative
    str = 0            # gamepad1.left_stick_x   # Pushing joystick to the right is positive
    rcw = 1            # gamepad1.right_stick_x  # Clockwise rotation is positive

    # Retrieve Rotational Angles and Velocities
    orientation = 0    # imu.getRobotYawPitchRollAngles()

    # Field-centric
    temp = (fwd * math.cos(orientation)) + str * math.sin(orientation)
    str2 = (-fwd * math.sin(orientation)) + str * math.cos(orientation)
    fwd2 = temp

    a = str2 - rcw * (wheelbase / r)
    b = str2 + rcw * (wheelbase / r)
    c = fwd2 - rcw * (trackwidth / r)
    d = fwd2 + rcw * (trackwidth / r)

    # Wheel speeds
    frs = math.sqrt(b**2 + c**2)
    fls = math.sqrt(b**2 + d**2)
    rls = math.sqrt(a**2 + d**2)
    rrs = math.sqrt(a**2 + c**2)

    # Wheel angles in degrees +- 180
    fra = math.atan2(b, c) * 180 / math.pi
    fla = math.atan2(b, d) * 180 / math.pi
    bra = math.atan2(a, d) * 180 / math.pi
    bla = math.atan2(a, c) * 180 / math.pi

    # Normalize wheel speeds
    max_speed = max(frs, fls, rls, rrs)
    if max_speed > 1:
        frs /= max_speed
        fls /= max_speed
        rrs /= max_speed
        rls /= max_speed

    # Wheel positions
    FLpos = 0    #  FLencoder.getVoltage() / 3.3 * 360 - FLoffset
    FRpos = 0    #  FRencoder.getVoltage() / 3.3 * 360 - FRoffset
    BLpos = 0    #  BLencoder.getVoltage() / 3.3 * 360 - BLoffset
    BRpos = 0    #  BRencoder.getVoltage() / 3.3 * 360 - BRoffset

    # Errors
    FLerror = normalize_degrees(fla - FLpos)
    FRerror = normalize_degrees(fra - FRpos)
    BLerror = normalize_degrees(bla - BLpos)
    BRerror = normalize_degrees(bra - BRpos)

    # Flip shortcut
    if MOTOR_FLIPPING and abs(FLerror) > math.pi / 2:
        fla = normalize_degrees(fla - math.pi)
        FLflipped = True
    else:
        FLflipped = False
    if MOTOR_FLIPPING and abs(FRerror) > math.pi / 2:
        fra = normalize_degrees(fra - math.pi)
        FRflipped = True
    else:
        FRflipped = False
    if MOTOR_FLIPPING and abs(BLerror) > math.pi / 2:
        bla = normalize_degrees(bla - math.pi)
        BLflipped = True
    else:
        BLflipped = False
    if MOTOR_FLIPPING and abs(BRerror) > math.pi / 2:
        bra = normalize_degrees(bra - math.pi)
        BRflipped = True
    else:
        BRflipped = False

    FLerror = normalize_degrees(fla - FLpos)
    FRerror = normalize_degrees(fra - FRpos)
    BLerror = normalize_degrees(bla - BLpos)
    BRerror = normalize_degrees(bra - BRpos)

    # Calculate wheel PID angles
    FLoutput = kP * FLerror
    FRoutput = kP * FRerror
    BLoutput = kP * BLerror
    BRoutput = kP * BRerror

    # FLsteer.setPower(FLoutput)
    # FRsteer.setPower(FRoutput)
    # BLsteer.setPower(BLoutput)
    # BRsteer.setPower(BRoutput)

    # Set drive powers
    # if FLflipped:
    #     FLdrive.setPower(-fls)
    # else:
    #     FLdrive.setPower(fls)

    # if FRflipped:
    #     FRdrive.setPower(-frs)
    # else:
    #     FRdrive.setPower(frs)

    # if BLflipped:
    #     BLdrive.setPower(-rls)
    # else:
    #     BLdrive.setPower(rls)

    # if BRflipped:
    #     BRdrive.setPower(-rrs)
    # else:
    #     BRdrive.setPower(rrs)

    # Telemetry wheel speeds
    print("FLspeed:", fls)
    print("FRspeed:", frs)
    print("BLspeed:", rls)
    print("BRspeed:", rrs)

    # Telemetry wheel angles
    print("FLangle:", fla)
    print("FRangle:", fra)
    print("BLangle:", bla)
    print("BRangle:", bra)

    # Telemetry flipped values
    print("FLflipped:", FLflipped)
    print("FRflipped:", FRflipped)
    print("BLflipped:", BLflipped)
    print("BRflipped:", BRflipped)

    time.sleep(0.5)

# Run the main function
run_swerve_drive()
