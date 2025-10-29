import os
import pty
import argparse
import sys
import time

def open_pty():
    master_fd, slave_fd = pty.openpty()
    slave_name = os.ttyname(slave_fd)
    return master_fd, slave_fd, slave_name

def stream_file_to_fd(master_fd, filepath, baud=230400, chunk_size=256, speed_factor=1.0, loop=False, delay_after_loop=0.0):
    bits_per_byte = 10.0
    sec_per_byte = (bits_per_byte / float(baud)) * float(speed_factor)
    try:
        with open(filepath, 'rb') as f:
            while True:
                f.seek(0)
                while True:
                    data = f.read(chunk_size)
                    if not data:
                        break
                    os.write(master_fd, data)
                    time.sleep(len(data) * sec_per_byte)
                if not loop:
                    break
                if delay_after_loop > 0.0:
                    time.sleep(delay_after_loop)
    except BrokenPipeError:
        pass

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--file', required=True, help='Input prerecorded binary file')
    parser.add_argument('--baud', type=int, default=230400, help='Baud to simulate (default 230400)')
    parser.add_argument('--loop', action='store_true', help='Loop file forever')
    parser.add_argument('--chunk-size', type=int, default=256, help='Write chunk size in bytes (default 256)')
    parser.add_argument('--speed-factor', type=float, default=1.0, help='Multiply inter-byte delay by this factor (default 1.0)')
    parser.add_argument('--delay-after-loop', type=float, default=0.0, help='Seconds to wait between loops (default 0.0)')
    args = parser.parse_args()

    if not os.path.isfile(args.file):
        print("File not found:", args.file, file=sys.stderr)
        sys.exit(2)

    master_fd, slave_fd, slave_name = open_pty()
    print("LD19 replay PTY slave device:", slave_name)
    print("Run your viewer with: ./ld19_view_clean --port", slave_name, "--baud", args.baud)
    sys.stdout.flush()

    try:
        stream_file_to_fd(master_fd, args.file, baud=args.baud, chunk_size=args.chunk_size,
                          speed_factor=args.speed_factor, loop=args.loop, delay_after_loop=args.delay_after_loop)
    except KeyboardInterrupt:
        print("Interrupted by user, exiting.")
    finally:
        try:
            os.close(master_fd)
            os.close(slave_fd)
        except:
            pass

if __name__ == "__main__":
    main()
