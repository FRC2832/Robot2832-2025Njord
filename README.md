# Current Robot Status
https://docs.google.com/spreadsheets/d/1hr8jmQvniOeUqg0M61z8sn0dWsT9DFsEs8XHRyx0pIs/edit?usp=sharing (requires Warriors login)

# Logging:
We use AdvantageKit to log variables, you can use the annotation `@AutoLogOutput` to automatically log a variable, or if you need to give it a specific name due to multiple instances of the class, `@AutoLogOutput(key = "Camera {name}/lastReadTimestamp")`.  You probably need to add in `AutoLogOutputManager.addObject(this);`.

See https://docs.advantagekit.org/data-flow/recording-outputs/annotation-logging

## Template
This repository is designed to be a template library to start robot development with.  It also contains standard library functions the team has developed to help develop future robots.  It should always be updated to the latest WpiLib release.

Any code that is in the org.livoniawarriors folder is designed to be a library.  Ideally, nothing in that logic needs to change, and it can't have any configurations dependent on current robot code (aka from the frc.robot project).  In the future, this might become it's own project with a git subproject into a folder to load into.  Any configuring of the core should be done with persistent network tables so that the systems can be tuned.

See more documentation at [docs](src/main/java/org/livoniawarriors/docs/)

By default, files that are changed by the user (like configurations for the Sim GUI or AdvantageScope) are on gitignore so they won't get checked in, but users can get default versions of these in the defaultconfig/ folder.

# Standard units:
* X axis = robot forward positive, back negative
* Y Axis = robot left positive, right negative
* Z axis = robot up positive, down negative
* Rotation = turn left positive (counter clockwise), right negative (clockwise)

Robot logging is on by default, we use standard [WpiLog](https://docs.wpilib.org/en/stable/docs/software/telemetry/datalog.html) files from WpiLib.  To access the logs, insert a flash drive and they log automatically on there, or use Advantage Scope or the WpiLib Data Log Tool to get them off the robot.  The path should be /home/lvuser.  A standard SSH/SCP tool will work too, the username is lvuser, no password.

# Differences from standard projects:
* Most teams have seperate folders for subsystems and commands.  Instead, we put all things related to the subsystem (both commands and subsystems) in the same folder, so it's easy to see what code is designed to work together.
