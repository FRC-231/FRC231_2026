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
travel_speed = 3.5
intake_speed = 2.0

function autonomous()
    drive_system:setMaxSpeed(travel_speed)
    drive_system:setMinPercentage(0.5)
    drive_system:setOdometry(4.4, 3.40, 180)--Starting position at trench
    drive_system:driveTo(6.39, 3.36, 180)--Drive thru trench
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
    drive_system:rotateTo(180)
    waitUntilRotationError(2)
    control:sleep(0.25)
    drive_system:driveTo(7.86, 1.68, -135)--Drive back, align to bump
    waitUntilDistance(0.0762)
    drive_system:driveTo(5.77, 1.68, -135)--Drive infront bump
    waitUntilDistance(0.0762)
    drive_system:setMinPercentage(0.0)
    drive_system:setMaxSpeed(bump_speed)
    shooter_system:autoShoot()--Start shooter wheels
    drive_system:driveTo(3.27, 1.68, -135)--Drive across bump
    waitUntilDistance(0.0762)
    drive_system:setMaxSpeed(travel_speed)
    drive_system:rotateTo(-45)--Face the hub
    waitUntilRotationError(2)
    drive_system:setAutoAA()--Auto aim to hub
    control:sleep(0.25)--This is to make sure it gets the right angle
    control:waitUntilTrue( function ()
        return shooter_system:isAtSpeed()--Waiting until shooter is at right speed
    end)
    shooter_system:loaderLoad()--Activate hopper
    control:sleep(1.5)
    intake_system:pivotUp()
    control:sleep(1.5)
    shooter_system:loaderStop()
    --END OF COMMON
    drive_system:rotateTo(-0.0)
    waitUntilRotationError(2)
    drive_system:driveTo(1.14, 1.93, -0.0)--Drive in front of fuel depot
    waitUntilDistance(0.0762)
    intake_system:pivotDown()
    intake_system:feedIntake()
    drive_system:setMaxSpeed(intake_speed)
    drive_system:driveTo(0.85, 1.93, -0.0)--Drive into depot
    waitUntilDistance(0.0762)
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
    drive_system:driveTo(3.50, 3.40, 180)--Drive back to starting position for tele-op
    waitUntilDistance(0.0762)
    drive_system:stop()

end
