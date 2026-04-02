local math = require("math")
print("QuickLoad autonomous script loading")

function waitUntilRotationError(angle)
    control:waitUntilTrue( function ()
            return math.abs(drive_system:getRotationError()) < angle
        end)
end

function waitUntilDistance(distance)
    control:waitUntilTrue( function ()
            return drive_system:getDistanceFromTarget() < distance
        end)
end

bump_speed = 2.5
rotate_speed = 3.5
travel_speed = 5.0
intake_speed = 3.0

function autonomous()
    drive_system:setMaxSpeed(travel_speed)
    drive_system:setMinPercentage(0.5)
    drive_system:setOdometry(4.4, 3.40, -180)--Starting position at trench
    drive_system:driveTo(6.39, 3.36, -180)--Drive thru trench
    waitUntilDistance(0.0762)
    intake_system:feedIntake()
    intake_system:pivotDown()
    drive_system:driveTo(7.56, 3.36, 90)--Drive towards the fuel
    waitUntilDistance(0.0762)
    drive_system:setMaxSpeed(intake_speed)
    drive_system:setMinPercentage(0.0)
    drive_system:driveTo(7.56, 1.0, 90)--Drive across, collect fuel
    waitUntilDistance(0.0762)
    drive_system:setMinPercentage(0.5)
    drive_system:setMaxSpeed(travel_speed)
    intake_system:feedHalfSpeed()
    control:sleep(0.25)
    drive_system:driveTo(7.86, 1.70, 45)--Drive back, align to bump
    waitUntilDistance(0.0762)
    drive_system:driveTo(5.77, 1.70, 45)--Drive infront bump
    waitUntilDistance(0.0762)
    drive_system:setMinPercentage(0.0)
    drive_system:setMaxSpeed(bump_speed)
    shooter_system:autoShoot()--Start shooter wheels
    drive_system:driveTo(2.20, 1.70, 45)--Drive across bump
    waitUntilDistance(0.0762)
    drive_system:setMaxSpeed(travel_speed)
    drive_system:rotateTo(-35)--Face the hub
    waitUntilRotationError(2)
    drive_system:setAutoAA()--Auto aim to hub
    control:sleep(0.20)--This is to make sure it gets the right angle
    control:waitUntilTrue( function ()
        return shooter_system:isAtSpeed()--Waiting until shooter is at right speed
    end)
    shooter_system:loaderLoad()--Activate hopper
    intake_system:pivotWiggle()
    control:sleep(1.5)
    -- intake_system:pivotUp()
    shooter_system:loaderStop()
    --END OF COMMON
    intake_system:pivotUp()
    drive_system:rotateTo(-45)--Make the intake face the middle
    waitUntilRotationError(2)
    drive_system:setMaxSpeed(bump_speed)
    drive_system:driveTo(6.0, 1.70, -45)--Drive across bump
    waitUntilDistance(0.0762)
    drive_system:rotateTo(90)
    waitUntilRotationError(2)
    drive_system:setMaxSpeed(travel_speed)
    drive_system:driveTo(6.21, 3.16, 90)--Drive to side of field to do a sweep
    waitUntilDistance(0.0762)
    intake_system:pivotDown()
    drive_system:rotateTo(90)
    waitUntilRotationError(2)
    drive_system:setMaxSpeed(intake_speed)
    drive_system:driveTo(6.21, 0.16, 90)--Sweep across the center, intake fuel
    waitUntilDistance(0.0762)
    drive_system:setMaxSpeed(travel_speed)
    drive_system:rotateTo(60)--Ensure angle
    waitUntilRotationError(2)
    drive_system:driveTo(6.21, 1.70, 60.0)--Drive in front of bump while rotating
    waitUntilDistance(0.0762)
    drive_system:setMaxSpeed(bump_speed)
    drive_system:driveTo(3.21, 1.70, 45.0)--Drive back across bump
    waitUntilDistance(0.0762)
     drive_system:setAutoAA()--Auto aim to hub
    control:sleep(0.20)--This is to make sure it gets the right angle
    control:waitUntilTrue( function ()
        return shooter_system:isAtSpeed()--Waiting until shooter is at right speed
    end)
    shooter_system:loaderLoad()--Activate hopper
    intake_system:pivotWiggle()
    control:sleep(1.5)
    -- intake_system:pivotUp()
    control:sleep(1.5)
    drive_system:stop()
end

