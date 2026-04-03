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
travel_speed = 4.0
intake_speed = 3.0

function autonomous()
    drive_system:setMaxSpeed(travel_speed)
    drive_system:setMinPercentage(0.5)
    drive_system:setOdometry(3.52, 0, 0)--Starting position at trench
    drive_system:driveTo(3.3, 1.93, 0.0)--Drive back
    waitUntilDistance(0.0762)
    drive_system:driveTo(1.14, 1.93, 0.0)--Drive in front of fuel depot
    waitUntilDistance(0.0762)
    intake_system:pivotDown()
    intake_system:feedIntake()
    drive_system:setMaxSpeed(intake_speed)
    drive_system:driveTo(0.85, 1.93, -0.0)--Drive into depot
    waitUntilDistance(0.0762)
    drive_system:setMaxSpeed(rotate_speed)
    drive_system:rotateTo(20)
    waitUntilRotationError(2)
    drive_system:rotateTo(-20)
    waitUntilRotationError(2)
    drive_system:rotateTo(20)
    waitUntilRotationError(2)
    drive_system:stop()
    control:sleep(1)
    intake_system:feedHalfSpeed()
    intake_system:pivotUp()
    drive_system:setMaxSpeed(travel_speed)
    drive_system:driveTo(1.5, 1.93, -45.0)
    waitUntilDistance(0.0762)
    shooter_system:autoShoot()
    drive_system:setAutoAA()--Auto aim to hub
    control:sleep(0.25)--This is to make sure it gets the right angle
    control:waitUntilTrue( function ()
        return shooter_system:isAtSpeed()--Waiting until shooter is at right speed
    end)
    shooter_system:loaderLoad()
    control:sleep(1.5)
    intake_system:pivotUp()
    control:sleep(1.5)
    shooter_system:stopShooter()
    shooter_system:loaderStop()
    drive_system:stop()
end
