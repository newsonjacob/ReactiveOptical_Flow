import argparse
import subprocess


def main():
    parser = argparse.ArgumentParser(description="Run main.py multiple times")
    parser.add_argument("--count", type=int, default=5,
                        help="Number of runs to execute (default: 5)")
    args = parser.parse_args()

    for i in range(1, args.count + 1):
        print(f"\n=== Starting run {i}/{args.count} ===")
        result = subprocess.run(["python", "main.py"])
        if result.returncode != 0:
            print(f"Run {i} exited with status {result.returncode}")
        else:
            print(f"Run {i} completed successfully")


if __name__ == "__main__":
    main()
