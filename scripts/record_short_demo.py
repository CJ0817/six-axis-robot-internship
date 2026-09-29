"""Render a short, labeled video from a fresh PyBullet DIRECT scene run."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=ROOT/'results/demo-short')
    args = parser.parse_args()
    out = args.output.resolve()
    frames_dir = out/'frames'
    frames_dir.mkdir(parents=True, exist_ok=True)
    if not shutil.which('ffmpeg'):
        raise SystemExit('ffmpeg is required to encode the MP4')
    scene = [sys.executable, str(ROOT/'examples/scene_demo.py'),
             '--headless', '--output', str(frames_dir)]
    subprocess.run(scene, cwd=ROOT, check=True, stdout=subprocess.DEVNULL)
    report = json.loads((frames_dir/'report.json').read_text())
    names = ['front', 'side', 'front']
    captures = report['captures']
    if report['status'] != 'passed' or report['execution_mode'] != 'DIRECT':
        raise RuntimeError('Scene run failed or was not DIRECT')
    if [x['preset'] for x in captures] != names or len(captures) != 3:
        raise RuntimeError('Unexpected view sequence')
    paths = [frames_dir/x['file'] for x in captures]
    for capture, path in zip(captures, paths):
        if digest(path) != capture['sha256'] or capture['robot_pixels'] < 20 or capture['object_pixels'] < 20:
            raise RuntimeError('Frame hash or scene visibility check failed')
    labels = ['Front / initial', 'Side / after J1 jog', 'Front / after J1 jog']
    cmd = ['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y']
    for path in paths:
        cmd += ['-loop', '1', '-framerate', '24', '-t', '2.5', '-i', str(path)]
    filters = []
    for i, label in enumerate(labels):
        filters.append(
            '[{}:v]drawtext=text={}:fontcolor=black:fontsize=21:box=1:boxcolor=white@0.85:boxborderw=8:x=14:y=14,'
            'drawtext=text=PyBullet DIRECT render - not a desktop capture:fontcolor=black:fontsize=15:'
            'box=1:boxcolor=white@0.85:boxborderw=5:x=14:y=h-35,setsar=1[v{}]'.format(i, label, i))
    filters.append(''.join('[v{}]'.format(i) for i in range(3))+'concat=n=3:v=1:a=0,format=yuv420p[v]')
    video = out/'ur5-direct-demo.mp4'
    cmd += ['-filter_complex', ';'.join(filters), '-map', '[v]', '-r', '24',
            '-c:v', 'libx264', '-preset', 'medium', '-crf', '20', '-pix_fmt', 'yuv420p',
            '-movflags', '+faststart', str(video)]
    subprocess.run(cmd, cwd=ROOT, check=True)
    manifest = dict(status='passed', kind='three-frame keyframe video from new DIRECT run',
                    date_utc=datetime.now(timezone.utc).isoformat(), user_windows_gui_recording=False,
                    length_seconds=7.5, fps=24, resolution=[640, 480], scene_command=scene,
                    ffmpeg_command=cmd, scene_report_sha256=digest(frames_dir/'report.json'),
                    scene_report_status=report['status'], view_difference=report['view_mean_abs_rgb_difference'],
                    object_position_error_m=report['object_after']['max_position_error_m'],
                    input_frames=[dict(file=str(path.relative_to(out)), sha256=digest(path)) for path in paths],
                    video=dict(file=video.name, sha256=digest(video), bytes=video.stat().st_size))
    (out/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(dict(status='passed', video=str(video), sha256=manifest['video']['sha256'])))


if __name__ == '__main__':
    main()
