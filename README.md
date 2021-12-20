# Stereo Fisheye Calibration

Calibrate fisheye stereo rig on raspberry pi (stereopi v2)

## Build
```
./build.sh
```

## Run
```
./run.sh -h
usage: main.py [-h] [--source_left SOURCE_LEFT] [--source_right SOURCE_RIGHT] [--iterative] [--max_frames MAX_FRAMES]

options:
  -h, --help            show this help message and exit
  --source_left SOURCE_LEFT
                        Select video device to directly fetch images from camera or folder to load pre recorded images.In case of live calibration images are stored within ./calibration_images. Otherwise source should contain glob expression a la "calibration_images/left*.png".
  --source_right SOURCE_RIGHT
                        Select "live" to directly fetch images from camera or folder to load pre recorded images.In case of live calibration images are stored within ./calibration_images. Otherwise source should contain glob expression a la "calibration_images/left*.png".
  --iterative           If set attempt to calibrate after each frame to make sure images are useful.
  --max_frames MAX_FRAMES
                        Choose how many frames should be used for calibration
```


### Live Calibration
```
./run.sh --source_left /dev/video0 --source_right /dev/video1 --iterative --max_frames 50
```
### Offline Calibration
```
./run.sh --source_left "./calibration_images/left*.png" --source_right "./calibration_images/right*.png" --max_frames 50
```