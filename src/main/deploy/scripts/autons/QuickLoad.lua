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

bump_speed = 2.0
rotate_speed = 3.5
travel_speed = 2.0
intake_speed = 2.0

function autonomous()
    drive_system:setMaxSpeed(travel_speed)
    drive_system:setMinPercentage(0.5)
    drive_system:setOdometry(3.50, 3.40, 180)--Starting position at trench
    drive_system:driveTo(6.39, 3.36, 180)--Drive past trench
    waitUntilDistance(0.0762)
    intake_system:feedIntake()
    intake_system:pivotDown()
    drive_system:driveTo(7.56, 3.36, 90)--Drive across to collect fuel
    waitUntilDistance(0.0762)
    drive_system:setMaxSpeed(intake_speed)
    drive_system:setMinPercentage(0.0)
    drive_system:driveTo(7.56, 0.43, 90)
    waitUntilDistance(0.0762)
    drive_system:rotateTo(180)
    waitUntilRotationError(2)
    control:sleep(0.25)
    drive_system:driveTo(7.86, 1.68, -135)
    waitUntilDistance(0.0762)
    drive_system:driveTo(5.77, 1.68, -135)-- at the bump
    waitUntilDistance(0.0762)
    drive_system:setMaxSpeed(bump_speed)
    drive_system:driveTo(3.27, 1.68, -135)
    waitUntilDistance(0.0762)
    shooter_system:autoShoot()
    drive_system:rotateTo(-45)
    waitUntilRotationError(2)
    drive_system:setAutoAA()
    control:sleep(0.25)
    control:waitUntilTrue( function ()
        return shooter_system:isAtSpeed()
    end)
    shooter_system:loaderLoad()
    --END OF COMMON
    -- drive_system:driveTo(1.14, 1.93, -0.0)
    -- waitUntilDistance(0.0762)


end
