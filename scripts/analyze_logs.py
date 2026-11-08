import csv
from pathlib import Path
from collections import defaultdict


LOG_FILE = Path("logs/motion_log.csv")
OUTPUT_DIR = Path("outputs")
OUTPUT_FILE = OUTPUT_DIR / "motion_summary.csv"


def analyze_logs():
    if not LOG_FILE.exists():
        print("No motion log found. Run a demo first.")
        return

    results = defaultdict(list)

    with open(LOG_FILE, mode="r", newline="") as file:
        reader = csv.DictReader(file)

        for row in reader:
            if row["event"] == "STATE_COMPLETED":
                state = row["state"]
                duration = row["duration_seconds"]

                if duration:
                    results[state].append(float(duration))

    if not results:
        print("No completed states found in log.")
        return

    OUTPUT_DIR.mkdir(exist_ok=True)

    with open(OUTPUT_FILE, mode="w", newline="") as file:
        writer = csv.writer(file)

        writer.writerow([
            "state",
            "runs",
            "average_duration_seconds",
            "minimum_duration_seconds",
            "maximum_duration_seconds"
        ])

        print("\nMotionCore Evaluation Summary")
        print("=" * 70)

        for state, durations in results.items():
            runs = len(durations)
            avg_duration = round(sum(durations) / runs, 3)
            min_duration = round(min(durations), 3)
            max_duration = round(max(durations), 3)

            writer.writerow([
                state,
                runs,
                avg_duration,
                min_duration,
                max_duration
            ])

            print(
                f"{state:20} "
                f"Runs: {runs:<3} "
                f"Avg: {avg_duration:<8} "
                f"Min: {min_duration:<8} "
                f"Max: {max_duration:<8}"
            )

    print(f"\nSummary saved to: {OUTPUT_FILE}")


if __name__ == "__main__":
    analyze_logs()