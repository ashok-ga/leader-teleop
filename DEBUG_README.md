# 🎥 Jetson Camera Device Debugging & GStreamer Usage

This README documents the steps and commands used to identify, test, and utilize video capture devices (`/dev/video*`) on a Jetson (or Linux) system.

** Don't quote me on this. ChatGPT summarized this for me. **

---

## 🔍 Listing and Inspecting Video Devices

### List all available video devices:

```bash
ls /dev/video*
```

### List which devices support Video Capture:


```bash
for dev in /dev/video*; do
  echo -n "$dev -> "
  v4l2-ctl --device=$dev --all 2>/dev/null | grep -q "Video Capture" && echo "OK" || echo "Not usable"
done
```

### Check formats supported by each device:

```bash
for dev in /dev/video*; do
  echo "=== $dev ==="
  v4l2-ctl --device=$dev --list-formats-ext
  echo
done
```

---

## 🗂️ Identify Device Metadata with `udevadm`

### View detailed info for a device:

```bash
udevadm info --name=/dev/videoX
```

Look for fields such as:

* `ID_MODEL`
* `ID_SERIAL`
* `ID_PATH`
* `ID_V4L_PRODUCT`

These can help differentiate between USB and onboard CSI cameras.

---

## 🎦 GStreamer Testing Pipelines

### MJPG or JPEG-decoding video stream:

```bash
gst-launch-1.0 v4l2src device=/dev/videoX ! image/jpeg,width=1280,height=720,framerate=30/1 ! jpegdec ! videoconvert ! autovideosink
```

### YUYV 4:2:2 video stream (e.g. 2560x720 resolution):

```bash
gst-launch-1.0 v4l2src device=/dev/videoX ! video/x-raw,format=YUY2,width=2560,height=720,framerate=30/1 ! videoconvert ! autovideosink
```

### Headless (no display) test:

```bash
gst-launch-1.0 v4l2src device=/dev/videoX ! video/x-raw,format=YUY2,width=2560,height=720,framerate=30/1 ! fakesink
```

---

## 📀 Recording with GStreamer (software encoding)

```bash
gst-launch-1.0 v4l2src device=/dev/videoX ! video/x-raw,format=YUY2,width=1280,height=720,framerate=30/1 ! videoconvert ! x264enc tune=zerolatency ! mp4mux ! filesink location=output.mp4
```

---

## 💻 X11 Display Forwarding (via SSH)

To run GStreamer preview when SSH-ing into Jetson:

```bash
ssh -X nvidia@<jetson-ip>
```

Then use a software video sink:

```bash
gst-launch-1.0 ... ! xvimagesink
```

or:

```bash
gst-launch-1.0 ... ! ximagesink
```

> Avoid `autovideosink` or `nveglglessink` unless you're on a Jetson with a physical display.

---

## 📁 Stable Device Paths (for CSI Cameras)

CSI cameras do not provide serial IDs. Use stable `by-path` symlinks:

```bash
ls -l /dev/v4l/by-path/
```

Example path:

```bash
/dev/v4l/by-path/platform-tegra-capture-vi-video-index0
```

Use this in config files for stable device mapping.

---

## 📄 YAML Mapping Generator Script

Use this script to list usable cameras with MJPG/YUYV/RGB3 support:

```bash
#!/bin/bash

echo "devices:"
echo "  cameras:"

for dev in /dev/video*; do
  formats=$(v4l2-ctl --device=$dev --list-formats-ext 2>/dev/null)

  if echo "$formats" | grep -q -E "MJPG|YUYV|RGB3"; then
    model=$(udevadm info --query=property --name=$dev 2>/dev/null | grep ID_MODEL= | cut -d= -f2)
    serial=$(udevadm info --query=property --name=$dev 2>/dev/null | grep ID_SERIAL= | cut -d= -f2)
    path_link=$(readlink -f "$dev")

    echo "    - device: \"$path_link\""
    [ -n "$model" ] && echo "      model: \"$model\""
    [ -n "$serial" ] && echo "      serial: \"$serial\""
  fi
done
```

Make it executable:

```bash
chmod +x list_cameras.sh
./list_cameras.sh
```

---

## 🗌 Useful Tools

Install the following utilities:

```bash
sudo apt install v4l-utils gstreamer1.0-tools
```

---

## ✅ Summary

| Task                               | Command                              |
| ---------------------------------- | ------------------------------------ |
| List video devices                 | `ls /dev/video*`                     |
| Check capture support              | `v4l2-ctl --all`                     |
| Check supported formats            | `v4l2-ctl --list-formats-ext`        |
| Identify device metadata           | `udevadm info --name=/dev/videoX`    |
| Test preview pipeline              | `gst-launch-1.0 ... ! autovideosink` |
| Record video                       | `gst-launch-1.0 ... ! filesink`      |
| Filter capture-capable RGB cameras | Bash + `v4l2-ctl`, `udevadm`         |
