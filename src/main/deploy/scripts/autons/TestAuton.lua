print("TestAuton autonomous script loading")

function waitUntilDistance(distance)
    control:waitUntilTrue( function ()
            return drive_system:getDistanceFromTarget() < distance
        end)
end

function autonomous()
    print("TestAuton autonomous script running")
    drive_system:setMaxSpeed(1.5)
    drive_system:setMinPercentage(0.0)
    drive_system:setOdometry(3.19, -1.68, 0.0)
    drive_system:driveTo(3.16, 0.83, 0.0)
    waitUntilDistance(0.05)
    drive_system:driveTo(3.16, -2.4, 90) 
    waitUntilDistance(0.05)
    drive_system:driveTo(2.28, 2.68, -125)
    waitUntilDistance(0.05)
    drive_system:stop()
    print("TestAuton autonomous script finished")
end


print("TestAuton autonomous script loaded")
