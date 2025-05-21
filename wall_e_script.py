from robots import *
import time
from coppeliasim_zmqremoteapi_client import *
import numpy as np

client = RemoteAPIClient()
sim = client.require("sim")

left_motor = Motor(sim, DeviceNames.MOTOR_LEFT_OS, Direction.CLOCKWISE)
right_motor = Motor(sim, DeviceNames.MOTOR_RIGHT_OS, Direction.CLOCKWISE)


# Trash (burgundy) color detection
def is_trash(r, g, b):
    return (r >= 75) & (r <= 125) & (g >= 25) & (g <= 70) & (b >= 0) & (b <= 30)

# Green plant cubes detection
def is_plants(r, g, b):
    return (g >= 140) & (g <= 255) & (r >= 40) & (r <= 120) & (b >= 10) & (b <= 100)

# Compressed trash (black) detection
def is_compressed_trash(r, g, b):
    return (r < 50) & (g < 50) & (b < 50)

# Charging area (yellow) detection
def is_charging_area(r, g, b):
    return (r >= 180) & (g >= 180) & (b <= 100)

# Red basket detection
def is_red_basket(r, g, b):
    # Detect strong red regions: red channel high, significantly above green/blue
    return (r > 150) & (g < 100) & (b < 100) & ((r - g) > 60) & ((r - b) > 60)

# Log similar colors in the current image that match the wider target color range
def log_similar_colors(image):
    r = image[:, :, 0].astype(int)
    g = image[:, :, 1].astype(int)
    b = image[:, :, 2].astype(int)

    mask = is_trash(r, g, b)
    r_sel = r[mask]
    g_sel = g[mask]
    b_sel = b[mask]

    if len(r_sel) == 0:
        print("No similar colors detected.")
        return

    unique_colors = set(zip(r_sel, g_sel, b_sel))
    print("Similar RGB values detected:")
    for rgb in unique_colors:
        print(f"  R={rgb[0]} G={rgb[1]} B={rgb[2]}")

def low_battery():
    while True:
        top_cam = ImageSensor(sim, DeviceNames.TOP_IMAGE_SENSOR_OS)
        top_cam._update_image()
        img = top_cam.get_image()
        height, width, _ = img.shape
        center_strip = img[:, width // 3: 2 * width // 3]
        r = center_strip[:, :, 0].astype(int)
        g = center_strip[:, :, 1].astype(int)
        b = center_strip[:, :, 2].astype(int)

        mask = is_charging_area(r, g, b)
        if np.count_nonzero(mask) > 50:
            print("⚡ Charging area detected in center. Driving forward...")
            while True:
                top_cam._update_image()
                img = top_cam.get_image()
                height, width, _ = img.shape

                # Center on the vertical middle strip
                center_col = img[:, width // 2 - 1: width // 2 + 1]
                r = center_col[:, :, 0].astype(int)
                g = center_col[:, :, 1].astype(int)
                b = center_col[:, :, 2].astype(int)
                center_mask = is_charging_area(r, g, b)

                # Centering: if the strip is yellow, move forward with correction
                ys, xs = np.where(center_mask)
                if len(ys) > 0:
                    cy = np.mean(ys)
                    error = (cy - height / 2) / (height / 2)
                    steer = -0.5 * error
                    left_motor.run(1 - steer)
                    right_motor.run(1 + steer)
                else:
                    left_motor.run(1)
                    right_motor.run(1)

                # Check bottom row for alignment and stop
                bottom = img[-2:]
                rb = bottom[:, :, 0].astype(int)
                gb = bottom[:, :, 1].astype(int)
                bb = bottom[:, :, 2].astype(int)
                bottom_mask = is_charging_area(rb, gb, bb)
                total_pixels = bottom_mask.size
                yellow_pixels = np.count_nonzero(bottom_mask)
                if yellow_pixels / total_pixels > 0.7:
                    print("🛑 Fully aligned with charging pad. Stopping.")
                    left_motor.run(0)
                    right_motor.run(0)
                    time.sleep(4)
                    break

                # Check if camera view is obstructed
                full_r = img[:, :, 0].astype(int)
                full_g = img[:, :, 1].astype(int)
                full_b = img[:, :, 2].astype(int)
                blocked = (
                        is_trash(full_r, full_g, full_b) |
                        is_plants(full_r, full_g, full_b) |
                        is_compressed_trash(full_r, full_g, full_b)
                )
                if np.count_nonzero(blocked) / blocked.size > 0.5:
                    print("⚠️ Camera likely blocked by object. Reversing and turning.")
                    left_motor.run(-1)
                    right_motor.run(-1)
                    time.sleep(0.5)
                    left_motor.run(-1)
                    right_motor.run(1)
                    time.sleep(0.4)
                    break
            break
        left_motor.run(0.2)
        right_motor.run(-0.2)
        time.sleep(0.1)
        break

def trash_detected():
    print("🎯 Target color detected. Motors stopped.")
    right_motor.run(0.5)
    left_motor.run(0.5)

    # Move forward until small cam sees mostly target color
    small_cam = ImageSensor(sim, DeviceNames.SMALL_IMAGE_SENSOR_OS)
    while True:
        small_cam._update_image()
        small_img = small_cam.get_image()
        sr = small_img[:, :, 0].astype(int)
        sg = small_img[:, :, 1].astype(int)
        sb = small_img[:, :, 2].astype(int)
        small_mask = is_trash(sr, sg, sb)
        coverage = np.count_nonzero(small_mask) / small_mask.size
        print(f"Small cam target coverage: {coverage:.2f}")

        if coverage > 0.98:
            left_motor.run(0.5)
            right_motor.run(0.5)
            time.sleep(0.5)
            print("🔧 Object centered and close. Compressing...")
            sim.setIntegerSignal("compress", 1)
            time.sleep(0.5)
            sim.setIntegerSignal("compress", 0)
            print("✅ Compression complete.")
            left_motor.run(0.4)
            right_motor.run(-0.4)
            time.sleep(2.35)
            left_motor.run(0)
            right_motor.run(0)
            time.sleep(0.5)
            left_motor.run(0.3)
            right_motor.run(0.3)
            time.sleep(2)
            while True:
                small_cam._update_image()
                small_img = small_cam.get_image()
                height, width, _ = small_img.shape
                center_crop = small_img[20:, width // 3:2 * width // 3]
                r = center_crop[:, :, 0].astype(int)
                g = center_crop[:, :, 1].astype(int)
                b = center_crop[:, :, 2].astype(int)
                mask_trash = is_compressed_trash(r, g, b)
                count_trash = int(np.count_nonzero(mask_trash))
                print("colichestvo")
                if count_trash > 30:
                    print("mnogo chernih")
                    left_motor.run(0.5)
                    right_motor.run(0.5)
                    time.sleep(0.7)
                    left_motor.run(0)
                    right_motor.run(0)
                    compressed_trash_detected()
                    break
            break
        elif coverage <= 0.00:
            left_motor.run(0)
            right_motor.run(0)
            break

        if np.count_nonzero(small_mask) > 0:
            ys, xs = np.where(small_mask)
            cx = xs.mean()
            width = small_img.shape[1]
            error = (cx - width / 2) / (width / 2)
            steer = -0.4 * error
            left_motor.run(4 - steer)
            right_motor.run(4 + steer)
        else:
            continue

def compressed_trash_detected():
    # Spin in place while small cam sees black compressed cubes and look for red basket
    small_cam = ImageSensor(sim, DeviceNames.SMALL_IMAGE_SENSOR_OS)
    top_cam = ImageSensor(sim, DeviceNames.TOP_IMAGE_SENSOR_OS)
    while True:
        small_cam._update_image()
        img_s = small_cam.get_image()
        r_s = img_s[:, :, 0].astype(int)
        g_s = img_s[:, :, 1].astype(int)
        b_s = img_s[:, :, 2].astype(int)
        mask_black = is_compressed_trash(r_s, g_s, b_s)
        if np.count_nonzero(mask_black) == 0:
            # No more black cubes in view
            break
        # Rotate slowly to scan environment
        left_motor.run(0.2)
        right_motor.run(-0.2)
        # Check top camera for red basket
        top_cam._update_image()
        img_t = top_cam.get_image()
        r_t = img_t[:, :, 0].astype(int)
        g_t = img_t[:, :, 1].astype(int)
        b_t = img_t[:, :, 2].astype(int)
        mask_red = is_red_basket(r_t, g_t, b_t)
        if np.count_nonzero(mask_red) > 10:
            # Found red basket direction
            left_motor.run(0)
            right_motor.run(0)
            # Move toward basket until >60% coverage
            while True:
                top_cam._update_image()
                img_t = top_cam.get_image()
                r_t = img_t[:, :, 0].astype(int)
                g_t = img_t[:, :, 1].astype(int)
                b_t = img_t[:, :, 2].astype(int)
                mask_red = is_red_basket(r_t, g_t, b_t)
                coverage = np.count_nonzero(mask_red) / mask_red.size
                print(f"basket coverage {coverage}")
                if coverage >= 0.4:
                    left_motor.run(0)
                    right_motor.run(0)
                    return
                # Steer toward basket based on its centroid
                ys, xs = np.where(mask_red)
                if len(xs) > 0:
                    cx = xs.mean()
                    error = (cx - img_t.shape[1] / 2) / (img_t.shape[1] / 2)
                    steer = -0.5 * error
                    left_motor.run(1 - steer)
                    right_motor.run(1 + steer)
                else:
                    # No centroid found, drive straight
                    left_motor.run(1)
                    right_motor.run(1)
            return
    # Stop motors if exiting without finding
    left_motor.run(0)
    right_motor.run(0)

def plants_detected():
    return
    # TODO

if __name__ == "__main__":
    sim.startSimulation()
    time.sleep(1)

    while True:
        level = float(sim.getStringSignal("battery"))*100
        if level < 50:
            print("  → Low battery, searching for charging area")
            low_battery()

        top_cam = ImageSensor(sim, DeviceNames.TOP_IMAGE_SENSOR_OS)
        top_cam._update_image()
        img = top_cam.get_image()
        height, width, _ = img.shape
        center_crop = img[:, width // 3:2 * width // 3]
        r = center_crop[:, :, 0].astype(int)
        g = center_crop[:, :, 1].astype(int)
        b = center_crop[:, :, 2].astype(int)

        # Count detected pixels for each object type
        mask_trash = is_trash(r, g, b)
        count_trash = int(np.count_nonzero(mask_trash))
        mask_plants = is_plants(r, g, b)
        count_plants = int(np.count_nonzero(mask_plants))

        # Determine which object is most prevalent
        counts = {'trash': count_trash, 'plant': count_plants}
        object_type = max(counts, key=counts.get)
        max_count = counts[object_type]
        print(f"Detected {object_type} pixels: {max_count}")

        # Act on the most prevalent object if above threshold
        if max_count > 10:
            if object_type == 'trash':
                trash_detected()
            elif object_type == 'plant':
                plants_detected()
            left_motor.run(0.2)
            right_motor.run(-0.2)
        else:
            left_motor.run(0.2)
            right_motor.run(-0.2)