import cv2
import os

VIDEO_PATH = 'output_video.mp4'
SAVE_DIR = 'saved_frames'
BOARD_SIZE = (16, 18)
os.makedirs(SAVE_DIR, exist_ok=True)

def main():
    cap = cv2.VideoCapture(VIDEO_PATH)
    if not cap.isOpened():
        print(f"I can't open {VIDEO_PATH}")
        return

    frame_id = 0
    while True:
        ret, original_frame = cap.read()
        if not ret:
            print("Video ended.")
            break

        gray = cv2.cvtColor(original_frame, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, BOARD_SIZE, None)
        if not ret:
            print(f"Chessboard dosn't found in frame {frame_id}, skipping...")
            frame_id += 1
            continue  # salta se non trova scacchiera

        frame = original_frame.copy()

        cv2.drawChessboardCorners(frame, BOARD_SIZE, corners, ret)
        print("Drawing chessboard corners...")

        cv2.imshow('Frame', frame)

        key = cv2.waitKey(0) & 0xFF  # aspetta finché non premi tasto

        if key == ord('s'):
            fname = os.path.join(SAVE_DIR, f"frame_{frame_id:06d}.png")
            cv2.imwrite(fname, original_frame)
            print(f"Saved: {fname}")
            frame_id += 1
        elif key == ord('d'):
            print(f"Omissis frame {frame_id}")
            frame_id += 1
        elif key == ord('f'):
            # Salta 30 frame
            skip_count = 0
            while skip_count < 30:
                ret, _ = cap.read()
                if not ret:
                    print("Video ended while skipping frames.")
                    break
                frame_id += 1
                skip_count += 1
            print(f"Skipped 30 frames, now on frame {frame_id}")
        elif key == ord('q'):
            print("Exiting.")
            break
        elif key == ord('l'):
            while frame_id < 17000:
                ret, original_frame = cap.read()
                if not ret:
                    print("Video ended while trying to load next frame.")
                    break
                frame_id += 1
            print(f"Loaded frame {frame_id}")
        else:
            print(f"Ignored key '{chr(key) if key!=255 else key}', staying on frame {frame_id}")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

