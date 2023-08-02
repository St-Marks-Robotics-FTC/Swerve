import time
import math

curFR = 0.0
curFL = 0.0
curBL = 0.0
curBR = 0.0

def clamp(num, min_value, max_value):
   return max(min(num, max_value), min_value)

def normalize_degrees(degrees):
    return (degrees + 180) % 360 - 180

def run_swerve_drive(forward, strafe, rotation):
    global curFR, curFL, curBL, curBR


    kP = 0.0
    kI = 0.0
    kD = 0.0

    MOTOR_FLIPPING = True

    FRoffset = 0.0
    FLoffset = 0.0
    BLoffset = 0.0
    BRoffset = 0.0

    wheelbase = 9.492126
    trackwidth = 9.492126
    r = math.sqrt(wheelbase**2 + trackwidth**2)

    # while True:
    fwd = clamp(float(forward) if forward else 0, -1, 1)
    str = clamp(float(strafe) if strafe else 0, -1, 1)
    rcw = clamp(float(rotation) if rotation else 0, -1, 1)

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
    bla = math.atan2(a, d) * 180 / math.pi
    bra = math.atan2(a, c) * 180 / math.pi

    # Normalize wheel speeds
    max_speed = max(frs, fls, rls, rrs)
    if max_speed > 1:
        frs /= max_speed
        fls /= max_speed
        rrs /= max_speed
        rls /= max_speed

    # Wheel positions
    FRpos = curFR    #  FRencoder.getVoltage() / 3.3 * 360 - FRoffset
    FLpos = curFL    #  FLencoder.getVoltage() / 3.3 * 360 - FLoffset
    BLpos = curBL    #  BLencoder.getVoltage() / 3.3 * 360 - BLoffset
    BRpos = curBR    #  BRencoder.getVoltage() / 3.3 * 360 - BRoffset

    # Errors
    FRerror = normalize_degrees(fra - FRpos)
    FLerror = normalize_degrees(fla - FLpos)
    BLerror = normalize_degrees(bla - BLpos)
    BRerror = normalize_degrees(bra - BRpos)

    # Flip shortcut
    if MOTOR_FLIPPING and abs(FRerror) > 90:
        fra = normalize_degrees(fra - 180)
        FRflipped = True
    else:
        FRflipped = False
    if MOTOR_FLIPPING and abs(FLerror) > 90:
        fla = normalize_degrees(fla - 180)
        FLflipped = True
    else:
        FLflipped = False
    if MOTOR_FLIPPING and abs(BLerror) > 90:
        bla = normalize_degrees(bla - 180)
        BLflipped = True
    else:
        BLflipped = False
    if MOTOR_FLIPPING and abs(BRerror) > 90:
        bra = normalize_degrees(bra - 180)
        BRflipped = True
    else:
        BRflipped = False

    FRerror = normalize_degrees(fra - FRpos)
    FLerror = normalize_degrees(fla - FLpos)
    BLerror = normalize_degrees(bla - BLpos)
    BRerror = normalize_degrees(bra - BRpos)

    # Calculate wheel PID angles
    FRoutput = kP * FRerror
    FLoutput = kP * FLerror
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
    print("FRspeed:", frs)
    print("FLspeed:", fls)
    print("BLspeed:", rls)
    print("BRspeed:", rrs)

    # Telemetry wheel angles
    print("FRangle:", fra)
    print("FLangle:", fla)
    print("BLangle:", bla)
    print("BRangle:", bra)

    # Telemetry flipped values
    print("FRflipped:", FRflipped)
    print("FLflipped:", FLflipped)
    print("BLflipped:", BLflipped)
    print("BRflipped:", BRflipped)

    curFR = fra
    curFL = fla
    curBL = bla
    curBR = bra

    time.sleep(0.5)

# Run the main function
while True:
    forward = input("Enter forward: ")
    strafe = input("Enter strafe: ")
    rotation = input("Enter rotation: ")

    run_swerve_drive(forward, strafe, rotation)
    print("")