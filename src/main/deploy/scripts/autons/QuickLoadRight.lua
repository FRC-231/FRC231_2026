local math = require("math")
print("QuickLoadRight autonomous script loading")

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
rotate_speed = 2.0
travel_speed = 3.5
intake_speed = 2.0

function autonomous()
    drive_system:setMaxSpeed(travel_speed)
    drive_system:setMinPercentage(0.5)
    drive_system:setOdometry(4.4, -3.40, -180)--Starting position at trench
    drive_system:driveTo(6.39, -3.36, -180)--Drive thru trench
    waitUntilDistance(0.0762)
    intake_system:feedIntake()
    intake_system:pivotDown()
    drive_system:driveTo(7.56, -3.36, -90)--Drive towards the fuel
    waitUntilDistance(0.0762)
    drive_system:setMaxSpeed(intake_speed)
    drive_system:setMinPercentage(0.0)
    drive_system:driveTo(7.56, -1.0, -90)--Drive across, collect fuel
    waitUntilDistance(0.0762)
    drive_system:setMaxSpeed(travel_speed)
    intake_system:feedHalfSpeed()
    drive_system:setMaxSpeed(rotate_speed)
    drive_system:rotateTo(-180)
    waitUntilRotationError(8)
    control:sleep(0.1)
    drive_system:driveTo(7.86, -1.60, 135)--Drive back, align to bump
    waitUntilDistance(0.0762)
    drive_system:driveTo(5.77, -1.60, 135)--Drive infront bump
    waitUntilDistance(0.0762)
    drive_system:setMaxSpeed(bump_speed)
    shooter_system:autoShoot()--Start shooter wheels
    drive_system:driveTo(2.90, -1.60, 135)--Drive across bump
    waitUntilDistance(0.0762)
    drive_system:setMaxSpeed(travel_speed)
    drive_system:rotateTo(45)--Face the hub
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
    intake_system:pivotDown()
    --END OF COMMON
    drive_system:setMaxSpeed(intake_speed)
    drive_system:driveTo(1.5, -2.75, 45.0)--drive towards the lever to pick up fuel
    waitUntilDistance(0.0762)
    drive_system:driveTo(2.42, -1.88, 45.0)--move forward to shoot
    waitUntilDistance(0.0762)
    drive_system:setAutoAA()--Auto aim to hub
    control:sleep(0.5)
    shooter_system:loaderLoad()--Activate hopper
    control:sleep(1.5)
    intake_system:pivotUp()
    control:sleep(1.5)
    intake_system:pivotDown()
    drive_system:driveTo(1.5, -2.75, 45.0)--drive towards the lever to pick up fuel
    waitUntilDistance(0.0762)
    drive_system:driveTo(2.42, -1.88, 45.0)--move forward to shoot
    waitUntilDistance(0.0762)
    drive_system:setAutoAA()--Auto aim to hub
    control:sleep(0.5)
    shooter_system:loaderLoad()--Activate hopper
    control:sleep(1.5)
    intake_system:pivotUp()


end
