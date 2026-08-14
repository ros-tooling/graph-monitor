# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
"""Summarize ThreadSanitizer reports for GitHub Actions.

TSAN prints two conflicting accesses per race, each with a stack that is mostly STL and
template noise. Only the frames inside this repository are actionable, so each race is
reduced to those, annotated against the file and line GitHub can link to.
"""

import os
import re
import sys
from pathlib import Path

# Frames in the code this job builds. Everything else (rclcpp, the RMW, the DDS layer) is
# uninstrumented, so races blamed on it are reported but do not fail the job.
OURS = re.compile(r'(rosgraph_monitor/(?:src|include|test)/[\w./]+):(\d+)')
# TSAN writes 'Read of size' but 'Previous write of size', so match case-insensitively.
ACCESS = re.compile(
    r'^\s+((?:previous )?(?:atomic )?(?:read|write)) of size (\d+) .* by (thread T\d+|main thread)', re.I
)
FRAME = re.compile(r'^\s+#\d+ (.+?) ([\w./-]+:\d+) \(')
SUMMARY = re.compile(r'^SUMMARY: ThreadSanitizer: (.+)', re.M)


def parse(text):
    """Yield one dict per race block."""
    for block in re.split(r'^={10,}$', text, flags=re.M):
        summary = SUMMARY.search(block)
        if not summary:
            continue
        accesses, current = [], None
        for line in block.splitlines():
            access = ACCESS.match(line)
            if access:
                current = {'kind': access.group(1), 'by': access.group(3), 'frames': []}
                accesses.append(current)
                continue
            frame = FRAME.match(line)
            if frame and current is not None:
                ours = OURS.search(frame.group(2))
                if ours and len(current['frames']) < 2:
                    # Drop the argument list; these signatures run to hundreds of characters.
                    func = frame.group(1).split('(')[0].strip()
                    current['frames'].append((ours.group(1), ours.group(2), func))
        yield {'summary': summary.group(1).strip(), 'accesses': accesses}


def main():
    report_dir = Path(sys.argv[1])
    reports = sorted(report_dir.glob('report.*'))
    summary_path = os.environ.get('GITHUB_STEP_SUMMARY', '/dev/null')

    races = {}
    for report in reports:
        for race in parse(report.read_text(errors='replace')):
            races.setdefault(race['summary'], race)

    ours = {s: r for s, r in races.items() if OURS.search(s)}
    theirs = {s: r for s, r in races.items() if s not in ours}

    with open(summary_path, 'a') as out:
        out.write('## ThreadSanitizer\n\n')
        if not reports:
            out.write('No reports produced: no data races detected.\n')
            print('No ThreadSanitizer reports produced.')
            return 0
        out.write('| | count |\n|---|---|\n')
        out.write(f'| Races in this repository | {len(ours)} |\n')
        out.write(f'| Races in uninstrumented dependencies | {len(theirs)} |\n\n')

        for summary, race in sorted(ours.items()):
            where = OURS.search(summary)
            path, line = where.group(1), where.group(2)
            detail = []
            for access in race['accesses']:
                frames = race and access['frames']
                if frames:
                    loc = ' <- '.join(f'{f[0]}:{f[1]} ({f[2]})' for f in frames)
                else:
                    loc = 'no frame in this repository'
                detail.append(f'{access["kind"]} by {access["by"]}: {loc}')
            message = ' | '.join(detail) or summary
            # Anchors the annotation to the source line in the PR diff.
            print(f'::error file={path},line={line},title=ThreadSanitizer data race::{message}')

            out.write(f'<details><summary><code>{path}:{line}</code></summary>\n\n')
            for item in detail:
                out.write(f'- {item}\n')
            out.write('\n</details>\n\n')

        for summary in sorted(theirs):
            print(f'::warning title=ThreadSanitizer (dependency)::{summary}')

    if ours:
        print(f'Found {len(ours)} data race(s) in this repository.', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
