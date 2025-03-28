Multitasking with WpiLib

If you need to schedule a task that is pseudo-scheduled with the robot code, it is recommended to use the AddPeriodic() function.  You can schedule tasks to run at whatever rate you want (doesn't have to be 20ms), and you can offset it from the robot code to run at a separate time.  If your task is mainly calculations and consistent, this is the correct method of implementing the logic.  Notifiers are a legacy topic and should not be considered for future development.

If we have logic that will do lots of blocking (for example, write files, or I2C communication), it is recommended to use a thread instead.  This allows your robot code to run completely independent of the task.  This was good in 2022 when the color sensor would do I2C reads and randomly slow down.

We have a helper function UtilFunctions.addPeriodic() that will get you access to the robot periodic function (as in the library, we don't have access to the root Robot object).

AddPeriodic runs after the robot code finishes (including after the NetworkTables data was sent out).  It is not a good place to command hardware.

See:
* https://docs.wpilib.org/en/stable/docs/software/convenience-features/scheduling-functions.html
* https://www.chiefdelphi.com/t/addperiodic-or-notifiers-loopers/416533/11
