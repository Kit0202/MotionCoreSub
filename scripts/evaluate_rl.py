"""Reproduce RL walking and deceleration checks without a viewer."""
import argparse
import json
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from backend.rl_policy import G1Policy


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--speed', type=float, default=.5)
    args = parser.parse_args()
    robot = G1Policy()
    measured = []
    minimum_height = float('inf')
    for step in range(round(20/robot.model.opt.timestep)):
        robot.step(args.speed if robot.data.time < 15 else 0.)
        if step % 50 == 0:
            sample = robot.telemetry()
            minimum_height = min(minimum_height, sample['height'])
            if sample['fallen']:
                raise RuntimeError(f'Fall at {sample["simulationTime"]:.2f}s')
            if 5 <= sample['simulationTime'] < 14:
                measured.append(sample['measuredSpeed'])
    result = dict(requestedSpeed=args.speed, meanMeasuredSpeed=sum(measured)/len(measured),
                  minimumHeight=minimum_height, final=robot.telemetry())
    output = ROOT/'outputs'/'rl_local_evaluation.json'
    output.parent.mkdir(exist_ok=True)
    output.write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps(result, indent=2))
    print(f'Saved: {output}')


if __name__ == '__main__':
    main()
