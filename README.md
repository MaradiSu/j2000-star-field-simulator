# J2000 star-field simulation in C++

## Live moving stars in your terminal

Run `star_sim` without arguments to enter duration (seconds) and angular rate
(degrees/second), or supply both directly:

```powershell
# Windows PowerShell, after building
.\star_sim.exe --animate 20 3
```

```sh
# Linux
./star_sim --animate 20 3
```

This runs for 20 seconds at +3 degrees/second about the body +Y axis. Positive
rates move stars left; negative rates move them right; zero freezes the sky.
Press **Q**, **Escape**, or **Ctrl+C** to finish early. The animation occupies
the terminal text area using a separate screen and restores the previous screen
and cursor afterward. It does not toggle the terminal application's OS full-screen
setting; maximize your terminal first for a larger view. Use an interactive terminal,
not redirected output or an IDE output pane. Windows uses a native console screen
buffer; Linux uses the terminal's alternate-screen support.

The live mode uses 5,000 deterministic synthetic stars distributed around the
whole sphere, so new stars keep entering as you turn. It starts facing J2000 +Z,
uses a 60 by 40 degree field of view and updates at approximately 30 frames/second.
Attitude follows actual elapsed monotonic time, rather than the number of rendered
frames. Character-cell positions and `@ * + .` brightness symbols are coarse visual
approximations, not calibrated image pixels. The rectangular text grid can stretch
the view. High rates can appear jerky or alias. Duration must be greater than zero
and at most 86,400 seconds; rate is limited to +/-3,600 deg/s.

The live mode needs no CSV files and writes no images. The original CSV-to-PGM
mode below remains available. `terminal_animation.h` contains terminal handling,
synthetic sky generation, rate-to-quaternion conversion and the display loop.

A small, dependency-free C++11 learning project for star-tracker optical stimulation. This is an educational simulator with synthetic catalogue data and an invented STOS command file; it is not flight-qualified software or an Airbus interface implementation.

## Build and verify with CMake

```sh
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

CMake is optional; direct compiler commands are below. Python 3 enables the integration test, which checks analytical star motion, output image encoding, invalid quaternion rejection, and repeatability. With a directly compiled executable, run `python tests/verify.py ./star_sim` (use `./star_sim.exe` on Windows). Tests retain their output in a `jsfs_check_*` folder for inspection. The GitHub Actions workflow builds and tests on Linux and Windows.

The repository contains one C++ source file, three sample CSV inputs, and a Python verification script. Start with `catalogueDirection()`, `rotate()`, `project()`, and `drawStar()` in `star_sim.cpp` to follow the model from sky coordinates to an image.

This example is aimed at a HIL engineer who already has a 6DOF attitude output. It implements the forward optical model: catalogue + attitude + tracker mounting -> visible star directions -> pixels. It also records an assumed STOS command interface as CSV. It does not implement star identification or estimate attitude from an image.

## Where it fits in your bench

```text
6DOF truth attitude (8 ms samples)
          |
          v
frame / quaternion convention adapter + sensor mounting
          |
          +--> this demo's mock_stos_commands.csv
          |         |
          |         v
          |    real STOS protocol adapter [requires equipment ICD]
          |         |
          |         v
          |    STOS renderer + optical head --> physical star tracker
          |                                      |
          |                                      v
          |                              tracker telemetry / 1553
          |                                      |
          |                                      v
          |                                OBC / AOCS --> 6DOF
          |
          +--> demo renderer <-- J2000 catalogue + camera configuration
                   |
                   +--> ideal spot coordinates + grayscale images
```

For an integrated STOS system, the renderer branch illustrates what the stimulator does internally; it is not necessarily another computation required on your HIL host. The real tracker observes light and produces its own measured attitude/status through its ICD-defined telemetry interface. The mock file is truth sent toward stimulation, not measured tracker output. A 1553 link belongs to the bench/sensor telemetry architecture if supported by that equipment; it is not the optical stimulus itself.

Airbus's public 2023 datasheet describes remote attitude commands over Ethernet at up to 128 Hz and a typical command-reception-to-image-update delay of about 60 ms, with buffering/interpolation. Your 8 ms step is 125 Hz, but satisfying that command-rate figure does not establish end-to-end timing compatibility. Confirm behavior on the installed configuration. The public datasheet does not specify packet layout, quaternion direction/order, time fields or handshakes; the CSV here is deliberately an invented educational interface.

Source: [Airbus STOS datasheet](https://www.airbus.com/sites/g/files/jlcbta136/files/2024-01/FILE%231_STOS_datasheet_2023.pdf).

## Run on RHEL / Linux

No external library is needed. From this folder:

```sh
g++ -std=c++11 -O2 -Wall -Wextra -pedantic -pthread star_sim.cpp -o star_sim
./star_sim --self-test
mkdir -p generated
./star_sim catalogue_j2000.csv attitude.csv camera.csv generated/
```

Windows PowerShell, with g++ on PATH:

```powershell
g++ -std=c++11 -O2 -Wall -Wextra -pedantic star_sim.cpp -o star_sim.exe
.\star_sim.exe --self-test
New-Item -ItemType Directory -Force generated
.\star_sim.exe catalogue_j2000.csv attitude.csv camera.csv generated/
```

The output argument is a **filename prefix**. The directory must exist. Existing matching output files are overwritten. Use a fresh directory when changing the number of input samples, because older extra frames are not removed.

## Input and output files

| File | Meaning |
|---|---|
| `catalogue_j2000.csv` | Star ID, right ascension in degrees, declination in degrees, instrumental magnitude |
| `attitude.csv` | Relative simulation time in seconds and inertial-to-body quaternion |
| `camera.csv` | Image dimensions, full horizontal/vertical FOV, magnitude cutoff, Gaussian spot width, brightness scale and fixed body-to-sensor mounting |
| `generated/mock_stos_commands.csv` | One normalized inertial-to-sensor quaternion and simulation timestamp per input sample; mock interface only |
| `generated/spots.csv` | Visible star IDs, ideal subpixel coordinates, magnitude and sensor-frame unit directions, for debugging |
| `generated/frame_0000.pgm` etc. | 16-bit grayscale PGM images; black background and Gaussian spots |

CSV is intentionally minimal: exact headers, numeric cells, no quoted fields or embedded commas. Blank lines and lines starting with `#` are allowed. Each attitude row generates one instantaneous frame. No interpolation, sleeping or real-time pacing occurs. These simulation timestamps are not UTC, J2000 elapsed seconds, command-arrival times or exposure timestamps.

The supplied catalogue contains **invented stars in J2000 axes**, not an authentic astronomical catalogue. This makes the first projection independently checkable. For real data, replace the rows with a validated catalogue having compatible reference frame, reference epoch and instrumental magnitude. A catalogue labelled J2000 is not automatically a set of apparent directions at your test date. ICRS and mean J2000 are close but should not silently be equated for precision work. Proper motion needs the catalogue's reference epoch; it is separate from coordinate equinox/frame. Magnitudes from a general astronomical passband may need conversion to your tracker/STOS band.

## The four calculations to understand

**1. RA/Dec to an inertial unit vector.** For RA `a` and declination `d` in radians:

```text
sI = [cos(d) cos(a), cos(d) sin(a), sin(d)]
```

J2000 equatorial +X is RA=0, Dec=0; +Y is RA=90 deg, Dec=0; +Z is the north celestial pole. Star vectors point from the observer toward the star. With stars at infinity and no aberration or occultation, spacecraft translation drops out; attitude is the relevant 6DOF output.

**2. Rotate the direction into the sensor frame.** All quaternions are Hamilton, scalar first `(w,x,y,z)`. The exact convention is defined by this operation:

```text
[0,sB] = qBI * [0,sI] * conjugate(qBI)
[0,sS] = qSB * [0,sB] * conjugate(qSB)
qSI    = qSB * qBI
```

`qBI` maps inertial vector components into body components. `qSB` is the fixed mounting map from body to sensor. If your 6DOF gives body-to-inertial `qIB`, conjugate its normalized quaternion first. If the ICD uses scalar-last ordering, reorder explicitly. Quaternion multiplication is not commutative. If STOS expects body attitude and applies mounting itself, do not apply mounting a second time.

Here the sensor is right-handed: +X is image-right, +Y image-down, +Z boresight. The virtual image plane is upright. A real optical/display path can invert or mirror this convention; determine that with the ICD and a sign/alignment test.

**3. Project visible directions.** Reject stars with `sS.z <= 0` and stars fainter than the configured cutoff. Then:

```text
fx = width  / (2 tan(horizontal_FOV/2))
fy = height / (2 tan(vertical_FOV/2))
cx = (width - 1)/2
cy = (height - 1)/2
u = cx + fx * sS.x/sS.z
v = cy + fy * sS.y/sS.z
```

Pixel centers are integer coordinates; image boundaries are -0.5 and width/height minus 0.5. The FOV is rectangular and uses full angles. Off-image star centers are rejected even if a PSF tail would overlap. The local 512x512, 20x20 degree camera is illustrative, not a model of STOS display calibration. Separate fx/fy allow a simple general camera model; the default gives square pixels.

**4. Render brightness.**

```text
peak_DN = peak_dn_mag0 * 10^(-0.4 * magnitude)
pixel += peak_DN * exp(-((x-u)^2 + (y-v)^2)/(2*sigma_px^2))
```

Five magnitudes fainter means 100 times less intensity. Spots are point-sampled Gaussians truncated near four sigma, summed and clipped to 65535. This is a convenient display model, not calibrated radiometry, integrated electron counts, a detection model, or a STOS drive-level calibration. The `spots.csv` positions are ideal geometric centers, not image-extracted centroids. Noise, thresholding and pixel sampling would affect recovered centroids in a tracker.

The general direction-transform-camera projection approach is also described in [ESA's star-tracker scene-generation explanation](https://kelvins.esa.int/star-trackers-first-contact/challenge/).

## What you should see

At time zero, both quaternions are identity, so the sensor points along J2000 +Z. This is a convenient teaching attitude, not an Earth-pointing operational attitude.

| Star | Expected first-frame result |
|---|---|
| DEMO_CENTER | (255.5, 255.5) |
| DEMO_RIGHT | approximately (382.52, 255.5) |
| DEMO_DOWN | approximately (255.5, 382.52) |
| DEMO_FAINT | rejected: magnitude 7 exceeds limit 6 |
| DEMO_OUTSIDE | rejected: outside FOV |
| DEMO_BEHIND | rejected: behind sensor |

Seven stars are visible in each supplied frame. The samples describe physical body motion of +1 deg/s about +Y, hence negative-Y inertial-to-body quaternions. The center star moves left by approximately 0.203 pixels in the first 8 ms. That small movement is expected; examine the CSV for precision. For a clearer visible change, provide a longer attitude sequence or larger angular increments. Do not alter catalogue RA/Dec to represent body rotation.

Useful first experiments: set all attitude rows to identity; change the mounting quaternion; narrow the FOV; change magnitude cutoff from 6 to 8; then compare pixel signs and visible star counts. The self-tests cover boresight, rear/FOV rejection, image signs, quaternion direction, noncommuting mounting composition, catalogue conversion and invalid quaternion rejection.

## Moving toward your 8 ms HIL loop

1. Load configuration/catalogue once, precompute inertial vectors, and replace `loadAttitudes()` with timestamped truth from your 6DOF interface.
2. Add the actual STOS adapter from the equipment ICD. Confirm frame convention, quaternion order/direction, mount ownership, time epoch/units, interpolation, transport, status/acknowledgement and watchdog behavior. The command file is the natural seam for this work.
3. Schedule command submission against an absolute monotonic deadline; keep file/image logging off the periodic task. Preallocate buffers and use bounded queues with an explicit stale-frame policy. Catalogue scanning here is linear and images allocate/write on every frame; this offline demo establishes no 8 ms performance claim.
4. Measure command-to-light delay, tracker exposure timing and telemetry latency separately. Compare measured attitude to truth at its effective measurement epoch, transforming both into the same frame. An 8 ms truth step does not imply an 8 ms tracker exposure or measurement rate. Fixed-delay handling and prediction depend on actual STOS timestamp/interpolation behavior.
5. Add fidelity incrementally: validated full catalogue, epoch/proper-motion handling, velocity-dependent aberration, orbit-dependent Earth/Sun/Moon masking and stray light, optical distortion, instrumental photometry, exposure integration/motion smear, noise and calibration. Position/velocity/time become relevant for these additions. Do not blindly precess a fixed-J2000 star vector while retaining a J2000 attitude.

Tracker measurement outputs would normally include attitude, validity/tracking mode, time and quality information, with the actual fields defined by the tracker ICD. This example intentionally produces no fake measured quaternion: recovering attitude requires a separate centroiding, star-identification and attitude-solution chain, or your physical tracker.
