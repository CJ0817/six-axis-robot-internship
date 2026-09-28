"""Summarize frozen UR5 IK reports; keep candidate and target populations distinct."""
import argparse
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def stats(values, expected):
    xs = sorted(float(x) for x in values if x is not None and math.isfinite(x) and x >= 0)
    n = len(xs)
    def q(p):
        h = (n - 1) * p
        i = int(h)
        return xs[i] + (h - i) * (xs[min(i + 1, n - 1)] - xs[i])
    return dict(valid_count=n, invalid_count=expected - n,
                mean=sum(xs) / n if n else None,
                rmse=math.sqrt(sum(x*x for x in xs)/n) if n else None,
                p95=q(.95) if n else None, maximum=xs[-1] if n else None)


def durations(rows, key, scale=1000):
    return stats([r[key]*scale if r.get(key) is not None else None for r in rows], len(rows))


def target_error(rows, key):
    return stats([max((v[key] for v in r['candidates'] if v.get(key) is not None), default=None)
                  for r in rows], len(rows))


def group(rows, workspace=False):
    passed = sum(r['passed'] for r in rows)
    if workspace:
        candidates = [v for r in rows for v in r['candidates']]
        assert len(candidates) == sum(r['candidate_count'] or 0 for r in rows)
        pos = stats([v['position_error_m'] for v in candidates], len(candidates))
        rot = stats([v['orientation_error_rad'] for v in candidates], len(candidates))
        c = durations(rows, 'c_function_s')
        ffi = durations(rows, 'python_to_c_call_ns', 1e-6)
        unit = 'candidate'
    else:
        pos = target_error(rows, 'position_error_m')
        rot = target_error(rows, 'orientation_error_rad')
        c = durations(rows, 'c_elapsed_s')
        ffi = durations(rows, 'python_to_c_s')
        unit = 'per_target_maximum_candidate'
    return dict(expected=len(rows), passed=passed, failed=len(rows)-passed,
                success_rate=passed/len(rows), error_unit=unit,
                position_error_m=pos, orientation_error_rad=rot,
                c_function_ms=c, python_to_c_ms=ffi)


def draw(data, path):
    import matplotlib
    matplotlib.use('Agg')
    matplotlib.rcParams['svg.fonttype'] = 'none'
    import matplotlib.pyplot as plt
    names = ['Workspace 120', 'Normal 100', 'Wide seed 20', 'Near singular 20']
    groups = [data['workspace120']['main']] + list(data['ik_groups'].values())
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4), layout='constrained')
    colors = ['#1478a7', '#704ca8', '#d07b32', '#257c64']
    for ax, key, threshold, title in zip(axes[:2],
                                         ['position_error_m', 'orientation_error_rad'],
                                         [1e-5, 1e-4], ['Position error (m)', 'Orientation error (rad)']):
        for idx, g in enumerate(groups):
            if g[key]['p95'] is None:
                continue
            ax.scatter(idx, g[key]['p95'], color=colors[idx], marker='o', s=50,
                       label='p95' if idx == 0 else None)
            ax.scatter(idx, g[key]['maximum'], color=colors[idx], marker='x', s=60,
                       label='maximum' if idx == 0 else None)
        ax.axhline(threshold, ls='--', lw=1, color='#b84f52', label='acceptance limit')
        ax.set_title(title)
        ax.legend(fontsize=8)
        ax.set_yscale('log')
    for idx, g in enumerate(groups):
        for marker, key in [('o', 'mean'), ('s', 'p95'), ('x', 'maximum')]:
            if g['c_function_ms'][key] is None:
                continue
            axes[2].scatter(idx, g['c_function_ms'][key], color=colors[idx],
                            marker=marker, s=50, label=key if idx == 0 else None)
    axes[2].set_yscale('log')
    axes[2].set_title('C IK call (ms), no warm-up')
    axes[2].legend(fontsize=8)
    for ax in axes:
        ax.set_xticks(range(4), names, rotation=30, ha='right')
        ax.grid(axis='y', which='both', alpha=.2)
    fig.suptitle('UR5 inverse kinematics: separate test populations')
    fig.savefig(path, format='svg')
    plt.close(fig)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--workspace-report', type=Path,
                   default=ROOT/'results/workspace120-batch-run/workspace120_batch/report.json')
    p.add_argument('--ik-report', type=Path,
                   default=ROOT/'results/closeout-ik/ik/report.json')
    p.add_argument('--output', type=Path, default=ROOT/'results/ik-summary')
    args = p.parse_args()
    a, b = [json.loads(path.read_text()) for path in (args.workspace_report, args.ik_report)]
    assert a['library_sha256'] == b['library_sha256'], 'cannot combine reports from different libraries'
    assert len(a['samples']) == 120 and len(a['boundaries']) == 16 and len(a['exceptions']) == 11
    assert len(b['samples']) == 140
    ik = {}
    for name, expected in [('normal', 100), ('wide_initial', 20), ('near_singular', 20)]:
        rows = [r for r in b['samples'] if r['group'] == name]
        assert len(rows) == expected == b['groups'][name]['expected']
        ik[name] = group(rows)
    main_group = group(a['samples'], workspace=True)
    all_group = group(b['samples'])
    all_group['script_work_wall_s'] = b['script_work_wall_s']
    boundary = a['boundaries']
    exception = a['exceptions']
    boundary_passed = sum(r['passed'] for r in boundary)
    exception_passed = sum(r['passed'] for r in exception)
    def label(path):
        try:
            return str(path.resolve().relative_to(ROOT))
        except ValueError:
            return str(path.resolve())

    result = dict(status='passed' if main_group['passed']==120 and all_group['passed']==140
                  and boundary_passed==16 and exception_passed==11 else 'has_failures',
                  source_sha256={label(path): hashlib.sha256(path.read_bytes()).hexdigest()
                                 for path in (args.workspace_report, args.ik_report)},
                  library_sha256=a['library_sha256'], quantile='linear interpolation h=(n-1)*p',
                  workspace120=dict(main=main_group,
                                    boundary=dict(expected=16, passed=boundary_passed,
                                                  singular_expected_code_2003=sum(r['ik_code']==2003 for r in boundary),
                                                  c_function_ms=durations(boundary, 'c_function_s'),
                                                  python_to_c_ms=durations(boundary, 'python_to_c_call_ns', 1e-6)),
                                    exception=dict(expected=11, passed=exception_passed,
                                                   time_scope='wrapper includes input preparation; excluded from C/FFI charts')),
                  ik_groups=ik, ik_all=all_group,
                  interpretation='Workspace120 uses 890 candidates; IK140 uses one maximum per target. They are separate test populations. Failure without output is excluded from error statistics, but remains in success denominator. The 4 singular boundary requests return expected 2003, not IK success.',
                  timing_scope='C CLOCK_MONOTONIC inside successful functions; Python perf_counter_ns around ctypes; IK140 script work wall includes suite work, not subprocess startup. One all-policy call per target, no warm-up, no hard timeout; descriptive timing only.')
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'summary.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    draw(result, args.output/'metrics.svg')
    print(json.dumps({'status': result['status'], 'summary': str(args.output/'summary.json'),
                      'figure': str(args.output/'metrics.svg')}))


if __name__ == '__main__':
    main()
