print("CurveTest autonomous script loading")

function waitUntilPathFinished()
    control:waitUntilTrue(function()
        return drive_system:isTrajectoryDone()
    end)
end

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
    print("CurveTest autonomous script running")

    drive_system:setOdometry(4.40, 3.30, 0)
    drive_system:setMaxSpeed(travel_speed)
    drive_system:driveTo(5.40, 3.370, 0)
    waitUntilDistance(0.0762)
    drive_system:followPath(1.0,
    {{6.15, 3.20},
    {7.30, 2.30},
    {7.75, 0.80}},
    0)
    waitUntilPathFinished()
    drive_system:stop()
    print("CurveTest autonomous script finished")
end

-- print("CurveTest autonomous script loaded")
