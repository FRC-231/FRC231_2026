print("Dance autonomous script loading")


function autonomous()
    print("Dance autonomous script running")
    control:sleep(0.5)
    drive_system:driveTo(4, 4, 90)
    control:sleep(1)
    drive_system:stop()
    control:sleep(1)
    drive_system:driveTo(4, -4, 180)
    control:sleep(1)
    drive_system:stop()
    control:sleep(1)
end


print("Dance autonomous script loaded")