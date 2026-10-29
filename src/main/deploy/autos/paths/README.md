# BLine paths

Path files authored in the BLine Web editor go here as `<name>.json`.

Editor: https://bline-web.pages.dev/
Docs:   https://bline-docs.pages.dev/

Load one from robot code with the helper on RobotContainer:

    followPath("myPath")     // reads autos/paths/myPath.json

Red-alliance mirroring is handled by `withDefaultShouldFlip()` in
`buildPathFollower()`, so author each path once for blue and let the follower
flip it.

Every follower gain in `Constants.BLineConstants` is a starting value from the
BLine docs, not tuned for this robot.
