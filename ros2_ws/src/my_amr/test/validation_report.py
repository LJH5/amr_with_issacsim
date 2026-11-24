# Copyright 2026 LJH5
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Summarize drive validation CSVs in a run directory as Markdown."""

import argparse
import csv
from math import degrees
from pathlib import Path
from statistics import mean


DIRECTIONS = {
    'forward': '전진', 'backward': '후진', 'left': '좌측 이동',
    'right': '우측 이동', 'ccw': '좌회전', 'cw': '우회전',
}


def trial_counts(rows):
    """Count finished measurements, unsettled trials, and timeouts."""
    measured = [row for row in rows if row.get('result', 'completed') == 'completed']
    unsettled = sum(row['settled'] == '0' for row in measured)
    timeouts = sum(row.get('result') in ('feedback_timeout', 'target_timeout') for row in rows)
    return measured, unsettled, timeouts


def write_report(directory, *, prefix=None):
    """Write one Markdown report, using only per-trial summaries and run statuses."""
    summaries = ([directory / f'{prefix}_summary.csv'] if prefix else
                 sorted(directory.glob('*_summary.csv')))
    if not summaries:
        raise FileNotFoundError(f'No validation summary CSV found in {directory}')

    trials = {}
    statuses = {}
    for summary in summaries:
        name = summary.stem.removesuffix('_summary')
        with summary.open(encoding='utf-8', newline='') as stream:
            trials[name] = list(csv.DictReader(stream))
        status = directory / f'{name}_status.txt'
        statuses[name] = status.read_text(encoding='utf-8').strip() if status.is_file() else '미기록'

    synthetic = prefix is None and {'complete', 'startup_wait'} <= trials.keys()
    primary = 'complete' if synthetic else next(iter(trials))
    measured, unsettled, timeouts = trial_counts(trials[primary])
    lines = ['# 주행 검증 요약', '']
    if synthetic:
        lines += ['합성 ROS 피드백 통합 테스트. 방향별 오차는 `complete` 케이스 기준입니다.', '']
    lines += [
        f'측정 완료 **{len(measured)}회** · 정지 기준 미충족 **{unsettled}회** · '
        f'타임아웃 **{timeouts}회**', '',
    ]
    if not statuses[primary].startswith('completed:'):
        lines += [f'실행 상태 `{statuses[primary]}`', '']

    if trials[primary]:
        lines += [
            '| 방향 | 측정 | 평균 위치 오차 | 최대 위치 오차 | 평균 절대 yaw 오차 | 정지 미충족 | 타임아웃 |',
            '| --- | ---: | ---: | ---: | ---: | ---: | ---: |',
        ]
        for direction, label in DIRECTIONS.items():
            rows = [row for row in trials[primary] if row['trial'] == direction]
            if not rows:
                continue
            measurements, unsettled, timeouts = trial_counts(rows)
            errors = [float(row['position_error_m']) * 1000 for row in measurements]
            yaw_errors = [abs(degrees(float(row['yaw_error_rad']))) for row in measurements]
            metrics = (f'{mean(errors):.1f} mm | {max(errors):.1f} mm | {mean(yaw_errors):.2f}°'
                       if measurements else '— | — | —')
            lines.append(
                f'| {label} | {len(measurements)} | {metrics} | {unsettled} | {timeouts} |')
        lines += ['', '오차는 휠 오도메트리와 ground truth의 차이입니다. '
                  '정지 기준 미충족 횟수는 측정 완료 횟수에 포함됩니다.']
    else:
        lines.append('기록된 방향별 시험 결과가 없습니다.')

    if len(trials) > 1:
        lines += ['', '## 나머지 케이스', '',
                  '| 케이스 | 측정 | 정지 미충족 | 타임아웃 | 실행 상태 |',
                  '| --- | ---: | ---: | ---: | --- |']
        for name, rows in trials.items():
            if name == primary:
                continue
            measurements, unsettled, timeouts = trial_counts(rows)
            status = statuses[name]
            status = '완료' if status.startswith('completed:') else status.replace('|', r'\|')
            lines.append(
                f'| `{name}` | {len(measurements)} | {unsettled} | {timeouts} | {status} |')

    report = directory / (f'{prefix}_report.md' if prefix else 'report.md')
    report.write_text('\n'.join(lines) + '\n', encoding='utf-8')
    return report


def main():
    """Regenerate a Markdown report for existing CSVs."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--prefix', help='Limit the report to one CSV prefix')
    args = parser.parse_args()
    print(write_report(args.directory, prefix=args.prefix))


if __name__ == '__main__':
    main()
