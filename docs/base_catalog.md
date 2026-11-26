# Base catalog

The mobile bases the repo models, with what each one ships with for perception and what is usually bolted
on, and which numbers have a source. Same idea as the [lift catalog](lift_catalog.md): the profile `base:` block
is the model, this page says how far to trust it. The sensors of a platform live in the profile's optional
`sensors:` list (schema below); no simulator reads it yet.

Date checked: 2026-11-26. Every figure below was read on the cited page or PDF that day, or the row says
`unverified`. Vendors change specs without notice: check against your unit.

Status words used in the tables: **agrees** (profile and source match, within rounding), **differs** (the source
says something else; the profile is not changed, see the end of each section), **profile only** (the profile
value, no source states it), **unverified** (not found in a source I could read).

## Sensors in the profile

```yaml
sensors:                      # optional; absent means none
  - type: lidar2d             # lidar2d | rgbd | imu | lidar3d
    name: lidar               # unique in the profile
    frame: lidar_link         # unique, child of base_link, not base_link itself
    mount: {xyz: [0.30, 0.0, 0.16], rpy: [0.0, 0.0, 0.0]}   # in base_link (body box centre, x forward)
    factory: false            # true: ships with the platform; false: a typical add-on
    source: "..."             # URL or text; an entry without one does not load
    spec:                     # depends on type
      angle_min_rad: -2.35619
      angle_max_rad: 2.35619
      angle_increment_rad: 0.00436332
      range_min_m: 0.06
      range_max_m: 10.0
      rate_hz: 40.0
      range_noise_std_m: 0.03 # optional, leave out if unknown
```

Spec fields per type: `lidar2d` as above; `rgbd` `width, height, hfov_rad, range_min_m, range_max_m, rate_hz,
depth_noise_note`; `imu` `rate_hz`; `lidar3d` `channels, hfov_rad, vfov_rad, range_min_m, range_max_m, rate_hz`
(no profile uses `imu` or `lidar3d` yet: no platform here had a sourced rate or channel spec). Loaded and checked
by `Profile::sensors` in `mobile_manipulator_profile` (C++) and `scripts/profile_sensors.py` (JSON out, same
rules). Errors name the profile and the entry.

Rule for what is in the profiles: a sensor is listed only when I read a vendor page or datasheet for its figures. A
sensor whose model or scan rate I could not source is described in the section below and left out of the YAML.
**The mounts are not the vendors'**: every add-on is placed where it plausibly fits on that chassis (lidar on the
front of the top deck, camera just above it), and the factory TurtleBot3 lidar sits near the centre of the top plate.
Treat them as an integrator's choice and measure your own.

### Sensor models used

| Sensor | What the profile says | Source | Not found |
|---|---|---|---|
| Hokuyo UST-10LX | 270 degrees, 0.25 degrees, 0.06 to 10 m (white target), 25 ms per scan (40 Hz), accuracy +/-40 mm, repeatability sigma < 30 mm (used as `range_noise_std_m`, an upper bound, not a measured noise) | [Hokuyo UST-10LX page](https://www.hokuyo-aut.jp/search/single.php?serial=167) | the page also shows 0.125 degrees for some variants |
| Slamtec RPLIDAR A2M8 | 360 degrees, 0.9 degrees at 10 Hz, 0.15 to 8 m (objects with 70 percent reflectivity) | [A2M8 datasheet v1.0](https://cdn.sparkfun.com/assets/e/a/f/9/8/LD208_SLAMTEC_rplidar_datasheet_A2M8_v1.0_en.pdf) | a newer Slamtec page lists 12 m and 0.45 degrees for the A2M8 (not opened); the v1.0 numbers are the conservative ones |
| ROBOTIS LDS-01 | 360 degrees, 1 degree, 0.12 to 3.5 m, 300 +/- 10 rpm (5 Hz), accuracy +/-15 mm below 0.5 m, +/-5 percent above (no sigma given, so none in the profile) | [LDS-01 appendix](https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_01/) | |
| Intel RealSense D435 | depth 87 x 58 degrees (the profile stores the 87), up to 1280 x 720, minimum 0.28 m at full resolution, ideal 0.3 to 3 m, accuracy under 2 percent at 2 m | [RealSense D435 page](https://www.realsenseai.com/products/stereo-depth-camera-d435/) | the page says up to 90 fps; `rate_hz: 30` is a common stream setting I chose, not a published figure for 1280 x 720 |
| Intel RealSense D455 | depth 87 x 58 degrees, up to 1280 x 720, minimum about 0.52 m, ideal 0.6 to 6 m, accuracy under 2 percent at 4 m | [RealSense D455 page](https://www.realsenseai.com/products/stereo-depth-camera-d455/) | same frame rate note as the D435 |
| SICK S300 (MiR100 safety scanners) | 270 degrees, 0.5 degrees, 30 m measuring range, 2 m protective field (the 2 m variant S30B-2011BA; the MiR's exact variant is not stated) | [S300 Standard data sheet](https://www.sick.com/media/pdf/5/45/845/dataSheet_S30B-2011BA_1026820_en.pdf), [S300 Mini manual](https://cdn.sick.com/media/docs/6/26/526/operating_instructions_s300_mini_safety_laser_scanner_en_im0040526.pdf) | scan rate: not in either document (only response time, 80 ms). Left out of the YAML |
| SICK nanoScan3 (MPO-500) | 275 degrees, 0.17 degrees, 40 m measuring range, 3 m protective and 10 m warning field, response time 70 ms | [NANS3-CAAZ30AN1 data sheet](https://www.sick.com/media/pdf/0/80/980/dataSheet_NANS3-CAAZ30AN1_1100334_en.pdf) | scan rate: not in the data sheet. Left out of the YAML |

## AgileX Ranger Mini 2.0 (`ranger_mini`, `ur10e_ranger_mini_telescopic`)

Four-wheel steering (omni, Ackermann and spin modes). Source: [Weston Robot docs, Ranger Mini 2.0](https://westonrobot.github.io/docs-legacy/robot_user_guide/agilex/ranger_mini_v2.0.html) (the AgileX manual PDF was found but not read).

| Item | Profile | Source says | Status |
|---|---|---|---|
| Body L x W x H | footprint 0.738 x 0.497 m | 738 x 500 x 338 mm | agrees (3 mm in width) |
| Drive | 4ws, can strafe | 4-wheel steering | agrees |
| Max linear speed | 1.5 m/s | 5.4 km/h (1.5 m/s) | agrees |
| Mass | 70 kg | 64.5 kg | **differs** |
| Ground clearance | 0.04 m | 107 mm minimum | **differs** |
| Payload | not in profile | 80 kg rated | sourced |
| Wheel radius, track, wheelbase | 0.0825, 0.364, 0.494 m | not on the page | unverified |
| Max angular speed, both accelerations | 1.5 rad/s, 1.0 m/s2, 2.0 rad/s2 | not on the page | unverified |
| Lift mount height | 0.264 m | not a vendor figure | profile only |

Sensors: none listed on the page (it has drift figures only). Add-ons in the profile: Hokuyo UST-10LX (front, 0.30, 0, 0.16) and RealSense D455 (front, 0.33, 0, 0.22), both `factory: false`.

## AgileX Scout 2.0 (`scout`, `ur10e_scout_telescopic`)

Four-wheel skid steer. Source: [Weston Robot docs, Scout 2.0](https://westonrobot.github.io/docs-legacy/robot_user_guide/agilex/scout_v2.0.html).

| Item | Profile | Source says | Status |
|---|---|---|---|
| Body L x W x H | footprint 0.93 x 0.682 m | 930 x 699 x 349 mm | length agrees, width **differs** (17 mm) |
| Max linear speed | 1.5 m/s | 1.5 m/s | agrees |
| Mass | 65 kg | 67 kg | **differs** |
| Ground clearance | 0.13 m | 135 mm | **differs** (5 mm) |
| Payload | not in profile | 50 kg (on a surface with friction 0.5) | sourced |
| Wheel radius, track, wheelbase | 0.1575, 0.582, 0.498 m | not on the page | unverified |
| Max angular speed, accelerations | 1.5, 1.0, 2.0 | not on the page | unverified |

Sensors: none listed (only a magnetic encoder, 2500 ppr). Add-on in the profile: Hokuyo UST-10LX (front, 0.40, 0, 0.16).

## AgileX Scout Mini (`scout_mini`, `ur10e_scout_mini_telescopic`, `xarm7_column500_lift`)

Four-wheel skid steer (mecanum wheels are an option). Source: [Weston Robot docs, Scout Mini](https://westonrobot.github.io/docs-legacy/robot_user_guide/agilex/scout_mini.html).

| Item | Profile | Source says | Status |
|---|---|---|---|
| Body L x W x H | footprint 0.638 x 0.524 m | 612 x 580 x 245 mm | **differs** (the page does not say which wheels the width is for) |
| Max linear speed | 1.7 m/s | 3 m/s | **differs** |
| Mass | 23 kg | 22.5 to 23.5 kg | agrees |
| Ground clearance | 0.107 m | 115 mm minimum | **differs** |
| Payload | not in profile | 10 kg standard wheels, 20 kg mecanum | sourced; far below a UR10e (see robot_description.md) |
| Wheel radius, track, wheelbase | 0.0875, 0.4535, 0.463 m | not on the page | unverified |
| Max angular speed, accelerations | 2.0, 1.0, 2.0 | not on the page | unverified |

Sensors: none listed. Add-ons in `ur10e_scout_mini_telescopic`: RPLIDAR A2M8 (front, 0.25, 0, 0.12) and RealSense D435 (front, 0.28, 0, 0.15). The `xarm7_column500_lift` profile uses the same base and has no `sensors:`.

## Clearpath Husky A200 (`husky`, `ur10e_husky_telescopic`)

Four-wheel skid steer. Sources: [Husky A200 user manual](https://docs.clearpathrobotics.com/docs_robots/outdoor_robots/husky/a200/user_manual_husky) and the [Husky data sheet](https://www.generationrobots.com/media/A-000000-01946/HUSKY_DATA_SHEET-v2_editeda.pdf) (a reseller's copy).

| Item | Profile | Source says | Status |
|---|---|---|---|
| Body L x W x H | footprint 0.99 x 0.671 m | 990 x 670 x 390 mm | agrees |
| Max linear speed | 1.0 m/s | 1 m/s | agrees |
| Mass | 50 kg | 50 kg | agrees |
| Ground clearance | 0.13 m | 130 mm | agrees |
| Wheel radius | 0.165 m | 330 mm diameter | agrees |
| Wheelbase | 0.512 m | 512 mm | agrees |
| Track | 0.5708 m | 550 mm in the table, 0.555 m in the manual's odometry equations | **differs** |
| Payload | not in profile | 75 kg max, 20 kg all-terrain | sourced |
| Max angular speed, accelerations | 2.0, 1.0, 2.0 | not published | unverified |
| Lift mount height | 0.39 m | equals body height 390 mm | agrees |

Sensors: nothing fitted at the factory that the manual names (internal sensing is battery, wheel odometry and motor currents; an IMU model is not given). The manual says IMUs, lidars, cameras and GPS are added with the purchase or later and names no models or mount points. Add-ons in the profile: Hokuyo UST-10LX (front of the deck, 0.44, 0, 0.18) and RealSense D435 (0.46, 0, 0.24).

## Clearpath Jackal (`jackal`, `kinova_gen3_jackal_nolift`)

Four-wheel differential (skid) drive. Source: [Jackal user manual](https://docs.clearpathrobotics.com/docs_robots/outdoor_robots/jackal/user_manual_jackal).

| Item | Profile | Source says | Status |
|---|---|---|---|
| Body L x W x H | footprint 0.508 x 0.43 m | 508 x 430 x 250 mm | agrees |
| Max linear speed | 2.0 m/s | 2.0 m/s | agrees |
| Ground clearance | 0.065 m | 65 mm | agrees |
| Wheel radius | 0.098 m | 194 mm diameter | agrees (1 mm) |
| Payload | not in profile | 20 kg | sourced |
| Mass | 17 kg | not on the page | unverified |
| Track, wheelbase | 0.37559, 0.262 m | not on the page | unverified |
| Max angular speed, accelerations | 4.0, 1.0, 2.0 | not on the page | unverified |

Sensors: an IMU and a GPS come with it (models not named, no rates). No lidar is fitted. Add-on in the profile: Hokuyo UST-10LX (front of the lid, 0.18, 0, 0.14).

## Clearpath Dingo-O (`dingo_o`, `xarm7_dingo_o_nolift`)

Four mecanum wheels. Source: [Dingo user manual](https://docs.clearpathrobotics.com/docs_robots/indoor_robots/dingo/user_manual_dingo).

| Item | Profile | Source says | Status |
|---|---|---|---|
| Body L x W x H | footprint 0.506 x 0.464 m | 686 x 517 x 114 mm | **differs** (the profile body is much shorter) |
| Max linear speed | 1.1 m/s | 1.3 m/s | **differs** |
| Mass | 17 kg | 17.5 kg | agrees |
| Ground clearance | 0.052 m | 16 mm | **differs** |
| Wheel radius | 0.05 m | 101.6 mm diameter | agrees |
| Payload | not in profile | 25 kg | sourced |
| Track, wheelbase | 0.404, 0.3 m | not stated | unverified |
| Max angular speed, accelerations | 2.0, 1.0, 2.0 | not stated | unverified |

Sensors: an onboard IMU (model not named); no lidar fitted at the factory. The manual page names no add-on models. Add-on in the profile: RPLIDAR A2M8 (front, 0.18, 0, 0.11). Check the Dingo-O dimensions against your unit before using the footprint for costmaps.

## MiR100 (`mir100`, `ur10e_mir100_telescopic`)

Differential drive, two drive wheels and four casters. The profile calls it "MiR100-class". Sources: [MiR100 data sheet](https://howtorobot.com/sites/default/files/2021-02/MiR100%E2%84%A2%20_%20Mobile%20Industrial%20Robots_0.pdf) (a reseller's copy) and the [MiR brochure](https://www.rg-group.com/Images/upload/Mobile%20Industrial%20Robots_brochure.pdf).

| Item | Profile | Source says | Status |
|---|---|---|---|
| Body L x W x H | footprint 0.89 x 0.58 m | 890 x 580 x 352 mm | agrees |
| Max linear speed | 1.5 m/s | 1.5 m/s forward, 0.3 m/s backward | agrees (profile has no reverse limit) |
| Mass | 67 kg | 67 kg in the brochure, 70 kg in the data sheet | agrees with one, **differs** from the other |
| Ground clearance | 0.05 m | 50 mm | agrees |
| Lift mount height | 0.352 m | equals height 352 mm | agrees |
| Wheel radius | 0.1 m | 125 mm diameter (drive and casters), so 0.0625 m | **differs** |
| Payload | not in profile | 100 kg, towing 300 kg | sourced |
| Track | 0.404 m | not on either | unverified |
| Max angular speed, accelerations | 1.0, 0.5, 1.0 | not on either | unverified |

Sensors at the factory (both documents): two SICK S300 safety scanners (front and back, 360 degrees together), two Intel RealSense 3D cameras (model not named for the MiR100; the brochure names D435 for another MiR model), four ultrasound sensors. Left out of the YAML: the S300 has no published scan rate in what I read, and the camera model is not confirmed. Scanner figures are in the sensor table above.

## Neobotix MPO-500 (`mpo_500`, `ur10e_mpo_500_telescopic`, `fr3_column870_lift`)

Four mecanum wheels. Source: [MPO-500 data sheet](https://www.neobotix-robots.com/fileadmin/images/downloads/Datenbl%C3%A4tter/Data-Sheet_MPO-500.pdf) (dated 28 Nov 2025). The MP-500 sheet is a different robot (differential, SICK S300 Expert) and was not used.

| Item | Profile | Source says | Status |
|---|---|---|---|
| Max linear speed | 1.0 m/s | max. 0.8 m/s | **differs** |
| Mass | 100 kg | 105 kg | **differs** |
| Payload | not in profile | 80 kg, 250 kg option | sourced |
| Drive | omni, strafe | mecanum, omnidirectional | agrees |
| Body L x W | footprint 0.88 x 0.58 m | not read from a source | unverified |
| Wheel radius, track, wheelbase, ground clearance, height | 0.1, 0.46, 0.55, 0.05, 0.3 m | not on the sheet | unverified |
| Max angular speed, accelerations | 1.0, 0.5, 1.0 | not on the sheet | unverified |

Sensors at the factory: one SICK nanoScan3, optionally two. Left out of the YAML because the data sheet gives no scan rate. Scanner figures are in the sensor table above.

## TurtleBot3 Burger (`turtlebot3_burger`, `generic_6dof_turtlebot3_burger_telescopic`)

Differential drive. Source: [ROBOTIS TurtleBot3 features](https://emanual.robotis.com/docs/en/platform/turtlebot3/features/).

| Item | Profile | Source says | Status |
|---|---|---|---|
| Size L x W x H | footprint 0.16 x 0.178 m | 138 x 178 x 192 mm | width agrees, length **differs** (profile box encloses the caster) |
| Max linear / angular speed | 0.22 m/s, 2.84 rad/s | 0.22 m/s, 2.84 rad/s | agrees |
| Mass | 1.0 kg | 1 kg with SBC, battery and sensors | agrees |
| Lift mount height | 0.192 m | equals height 192 mm | agrees |
| Payload | not in profile | 15 kg | sourced |
| Wheel radius, track, accelerations, ground clearance | 0.033, 0.16, 0.5, 2.0, 0.01 | not on the page | unverified |

Sensors: a 360 degree laser distance sensor and a 3-axis gyro plus 3-axis accelerometer. The page currently names the LDS-02; the original Burger shipped the LDS-01, whose figures are the ones read (and in the profile, `factory: true`). IMU rate is not stated, so no `imu` entry.

## TurtleBot3 Waffle (`turtlebot3_waffle`, `generic_6dof_turtlebot3_waffle_telescopic`, `generic_6dof_stretch_mast_lift`)

Differential drive. Source: [ROBOTIS TurtleBot3 features](https://emanual.robotis.com/docs/en/platform/turtlebot3/features/) (the page lists Waffle Pi; the original Waffle is discontinued).

| Item | Profile | Source says | Status |
|---|---|---|---|
| Size L x W x H | footprint 0.281 x 0.305 m | 281 x 306 x 141 mm | agrees |
| Max linear / angular speed | 0.26 m/s, 1.82 rad/s | 0.26 m/s, 1.82 rad/s | agrees |
| Mass | 1.8 kg | 1.8 kg | agrees |
| Lift mount height | 0.141 m | equals height 141 mm | agrees |
| Payload | not in profile | 30 kg | sourced |
| Wheel radius, track, accelerations, ground clearance | 0.033, 0.287, 0.5, 2.0, 0.01 | not on the page | unverified |

Sensors: same LDS as the Burger, a Raspberry Pi Camera v2.1 on the Waffle Pi (an RGB camera, not RGB-D, so no entry). The original Waffle's depth camera is not on the current page.

## TIAGo-style (`tiago_style`, `fr3_tiago_style_torso`, `generic_6dof_hsr_torso_lift`)

A round differential base in the size of a PAL TIAGo, modelled as a box. Source: [PAL TIAGo hardware overview](http://docs.pal-robotics.com/edge/hardware/tiago/hardware-overview).

| Item | Profile | Source says | Status |
|---|---|---|---|
| Footprint | 0.54 x 0.54 m box | 54 cm diameter (differential bases) | agrees (box is conservative) |
| Max linear speed | 1.0 m/s | 1 m/s | agrees |
| Mass | 60 kg | 60 kg (TIAGo Lite), 72 kg (TIAGo) | agrees with the Lite only |
| Wheel radius, track, accelerations, angular speed, clearance | 0.0985, 0.4044, ... | not on the page | unverified |

Sensors: a laser range-finder, sonars and an IMU; the page gives no models or figures, so no entries. The HSR profile reuses this base with Toyota's lift and is not a Toyota base.

## Fetch-style (`fetch_style`, `generic_6dof_fetch_style_torso`)

A round differential base in the size of a Fetch robot, modelled as a box. I could not read any Fetch page (the documentation host did not resolve), so every number is **unverified**: 0.56 m box, 1.0 m/s, 2.0 rad/s, 70 kg, wheel radius 0.0605 m, track 0.3748 m. No sensors are listed. Treat the profile as a size class, not a Fetch.

## Generic box (`generic_base`)

Not a product: 0.9 x 0.6 m, 0.5 m/s, 60 kg, nominal values. No sensors. It is the default of the other profiles.

## Notes

- `lift_mount_height_m`, `arm_mount_xy`, the accelerations and the angular speeds are planning defaults of this repo,
  not vendor figures, except where a section says the height equals the published body height.
- Payload is not read by the model; it is here so you can check a lift and arm against the platform rating.
- Footprints come from the body size plus whatever the profile encloses (wheels, casters); they are a conservative
  rectangle, so small differences from the vendor length and width are expected.
- `test/test_profile_sensors.py` (description package) loads every profile and validates its `sensors:` block.
