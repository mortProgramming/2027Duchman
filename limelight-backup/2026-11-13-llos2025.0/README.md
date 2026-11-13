# Limelight backup — 2026-11-13, before updating from LLOS 2025.0

Everything the `limelight-front` Limelight 4 exposes, pulled read-only before a Limelight OS update.
Taken through the rio's USB link (ports 5801/5805/5807 forwarded to 10.0.11.11).

## What the camera was set to

| Setting | Value | Where it lives after an update |
|---|---|---|
| Team number | 11 | Settings tab — re-enter |
| Hostname | `front` → `limelight-front` | Settings tab — re-enter. **Must be exact**: it is the NetworkTables table name the robot code reads |
| IP | static `10.0.11.11`, netmask /24 (`255.255.255.0`), gateway `10.0.11.1` | Settings tab — re-enter, then power-cycle |
| Active pipeline | 0, `dutchmanfront`, AprilTag (36h11 classic), tag size 165 mm | `pipelines/dutchmanfront.vpr` |
| Pipelines 1–9 | factory defaults (all identical) | nothing to restore |
| Field map | **2025 Reefscape**, 22 tags (`field-map.fmap`) | upload `field-map.fmap` |
| Lens calibration | factory default only (no custom file/EEPROM calibration) | nothing to restore |
| Camera pose on robot | 0.31 m fwd, 0.062 m right, 0.229 m up, 0° roll, 10° pitch, 0° yaw | **pushed by robot code every boot** (`VisionConstants`) — do not type it into the UI |
| IMU mode | set by robot code at boot (`imumode_set` = 0) | nothing to restore |
| SnapScripts (Python) | none | — |
| Snapshots | 12 PNGs in `snapshots/` | optional, re-upload only if wanted |

Raw `networking` reply (`webui/get_all_networking_info_response.txt`):
`11:1:10.0.11.11:24:10.0.11.1 / :front / :22:0:0:0.0.0.0:502:0`
= team 11, static (1), IP, /24, gateway, hostname suffix `front`.

## Restore after the update

1. **Network first.** Connect to the camera (new LLOS usually comes up on DHCP or its default
   address — use Limelight Hardware Manager over Ethernet to the radio, not the rio's USB cable).
   In **Settings**: team `11`, hostname `front`, static IP `10.0.11.11`, netmask `255.255.255.0`,
   gateway `10.0.11.1`. Save and power-cycle.
2. **Pipeline.** Select pipeline 0, then use the web UI's *Upload pipeline* button with
   `pipelines/dutchmanfront.vpr`. (Alternative: `POST http://10.0.11.11:5807/upload-pipeline?index=0`
   with `pipelines/pipeline0.json` as the body.) Check the name reads `dutchmanfront` and the type is
   AprilTag.
3. **Field map.** Upload `field-map.fmap` in the AprilTag/3D section of the pipeline
   (*Upload field map*). See the note below before choosing which map to use.
4. **Nothing else to type.** Camera pose and IMU mode come from robot code on every boot.
5. **Check:** the robot's NetworkTables shows `limelight-front`, the Elastic camera
   (`limelight-front-rio`) shows video, `Vision/TagID` reads the tag in view, and `Vision/AvgTagDistance`
   matches a tape measure.

## Note on the field map (found while making this backup)

This map is the **2025** field. In it, **tag 4 is a barge tag, tilted 30° and 1.87 m up**
(`field-map.fmap`, id 4 transform). The tag used for bench testing in Nov 2026 was a printed tag 4,
vertical, about 0.6 m up. That mismatch is why MegaTag1 reported the robot 0.72 m in the air and
pitched 33°, and is the likely reason MegaTag2 returned all zeros. For bench tests, either use a tag
id that is vertical in the map (2025 reef tags 6–11 and 17–22 sit 0.31 m up), or upload a custom
map that puts the test tag where it really is.

## Files

```
field-map.fmap          the camera's field map, ready to upload
pipelines/
  dutchmanfront.vpr     pipeline 0 in the web UI's own download format (upload this)
  pipeline0.json        pipeline 0 as returned by GET /pipeline-atindex (REST upload format)
  pipeline1-9.json      factory defaults
calibration/            cal-default.json (factory); cal-file/eeprom/latest empty = no custom cal
device/                 status, hwreport (lens intrinsics), snapshot manifest, default pipeline
webui/                  raw replies from the web UI socket (field map, networking, pipeline, status)
snapshots/              12 saved camera snapshots
```

To take another backup, see `../tools/LlDump.java` (read-only: refuses anything but `request_*` /
`get_*` messages) plus the `curl` GETs listed above (`/pipeline-atindex?index=N`, `/cal-*`,
`/status`, `/hwreport`, `/snapshotmanifest`, and `http://<camera>:5801/snapshots/<name>`).
