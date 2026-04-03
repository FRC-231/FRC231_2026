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
intake_speed = 1.0

function autonomous()
    drive_system:setMaxSpeed(travel_speed)
    drive_system:setMinPercentage(0.5)
    drive_system:setOdometry(3.52, 0.0, 0.0)--Starting position at hub
    shooter_system:autoShoot()--Start shooter wheels
    drive_system:driveTo(2.5, 0.0, 0.0)--Drive back
    waitUntilDistance(0.0762)
    drive_system:setAutoAA()--Auto aim to hub
    control:sleep(0.75)--This is to make sure it gets the right angle
    control:waitUntilTrue( function ()
        return shooter_system:isAtSpeed()--Waiting until shooter is at right speed
    end)
    shooter_system:loaderLoad()--Activate hopper
    -- intake_system:pivotWiggle()
    control:sleep(2.0)
    drive_system:stop()
end