"""将已保存的 HexFrame 成功或未完成工况做成整体＋接口特写视频。

只读 model.xml、rollout.npz 和事件/锚点记录，不重新积分，也不升级验收状态。
每帧取最近的 100 Hz 保存状态，24 fps、原速回放；角度与输出属于展示设置。
截短工况沿其真实保存时间结束，无需具有完整阶段或通过审计。
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path

os.environ.setdefault('MUJOCO_GL', 'egl')

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw

from compliant_docking.assembly.rendering import camera, caption_font

LABELS = {'nominal': '名义', 'xy': '真实接收站 X +2 mm',
          'yaw': '真实接收站绕轴 +5°', 'combined': 'X +2 mm ＋ 绕轴 +5°'}
PHASES = ['初始布局', '接近存储模块', '进入抓取位置', '抓取确认', '存储解锁',
          '提起模块', '转运', '接近接收模块', '降至接触段', '接触与插合',
          '卸力与释放阶段', '撤离阶段', '结束阶段']


def render_review(run_dir, output, label):
    """只读原运行并创建视频、末帧及输入/输出哈希，不覆盖已有展示目录。

    Args:
        run_dir: 保存模型及 100 Hz 状态的目录，可以是未完成工况。
        output: 必须为空或不存在的输出目录；写入 MP4、PNG 和 JSON。
        label: 显示在图像外部说明栏的工况名称。
    """
    run_dir, output = Path(run_dir).resolve(), Path(output).resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError('拒绝覆盖非空展示目录')
    output.mkdir(parents=True, exist_ok=True)
    inputs = ['model.xml', 'rollout.npz', 'validation.json', 'accepted_anchor.json']
    hashes = {name: hashlib.sha256((run_dir/name).read_bytes()).hexdigest() for name in inputs}
    model = mujoco.MjModel.from_xml_path(str(run_dir/'model.xml'))
    anchor = json.loads((run_dir/'accepted_anchor.json').read_text())
    model.site('assembly_anchor').pos[:] = anchor['pos']
    model.site('assembly_anchor').quat[:] = anchor['quat']
    with np.load(run_dir/'rollout.npz') as archive:
        states = {key: archive[key] for key in archive.files}
    times = states['t']
    if not np.isfinite(states['qpos']).all() or np.any(np.diff(times) <= 0):
        raise ValueError('保存状态或时间异常')
    data = mujoco.MjData(model)
    options = mujoco.MjvOption()
    options.geomgroup[3] = False
    options.geomgroup[4] = True
    options.sitegroup[:] = False
    model.vis.map.znear = .003
    overview = camera([0., .30, .36], 2.45, 85, -23)
    title_font, small = caption_font(23), caption_font(18)
    fps = 24
    with mujoco.Renderer(model, height=360, width=640) as renderer:
        def frame(index):
            data.qpos[:] = states['qpos'][index]
            data.qvel[:] = states['qvel'][index]
            data.eq_active[:] = states['locks'][index]
            mujoco.mj_forward(model, data)
            phase = int(states['phase'][index])
            sites = ('gripper_anchor', 'module1_anchor') if phase <= 5 else (
                'module1_port_4_mating', 'module2_port_1_mating')
            focus = np.mean([data.site(name).xpos for name in sites], axis=0)
            detail = camera(focus, .38 if phase <= 5 else .28, 90, -10)
            result = Image.new('RGB', (1280, 480), '#0a1424')
            for x, cam in [(0, overview), (640, detail)]:
                renderer.update_scene(data, camera=cam, scene_option=options)
                renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
                result.paste(Image.fromarray(renderer.render().copy()), (x, 78))
            draw = ImageDraw.Draw(result)
            draw.text((18, 8), f'{label}  |  {PHASES[phase]}  |  t={times[index]:.2f} s',
                      font=title_font, fill='#edf5ff')
            locks = states['locks'][index]
            status = '    '.join(f'{name}：{"锁定" if locked else "解除"}' for name, locked in
                                  zip(['存储', '机械臂抓持', '模块间'], locks, strict=True))
            draw.text((18, 43), f'整体视角    |    {status}', font=small, fill='#8ce1cb')
            draw.text((654, 43), '抓取接口特写' if phase <= 5 else '模块 1 接口 4 ↔ 模块 2 接口 1',
                      font=small, fill='#edf5ff')
            draw.text((18, 448), '同次运行保存状态 · 原速回放 · 无重新仿真 · 理想锁定模型',
                      font=small, fill='#a9bdd6')
            return result

        count = int(np.floor((times[-1]-times[0])*fps))+1
        with imageio.get_writer(output/'review.mp4', fps=fps, codec='libx264',
                                quality=8, macro_block_size=1) as writer:
            for playback_t in np.linspace(times[0], times[-1], count):
                right = min(int(np.searchsorted(times, playback_t)), len(times)-1)
                index = right-1 if right and abs(times[right-1]-playback_t) < abs(times[right]-playback_t) else right
                writer.append_data(np.asarray(frame(index)))
        frame(len(times)-1).save(output/'last_frame.png')
    for name, digest in hashes.items():
        if hashlib.sha256((run_dir/name).read_bytes()).hexdigest() != digest:
            raise RuntimeError('原始输入在生成时发生变化')
    manifest = dict(input_run=str(run_dir), input_sha256=hashes,
                    fps=fps, frame_count=count, original_end_s=float(times[-1]),
                    movie_duration_s=count/fps, nearest_saved_state=True,
                    max_time_selection_error_s=float(np.max(np.diff(times))/2),
                    source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                    video_sha256=hashlib.sha256((output/'review.mp4').read_bytes()).hexdigest())
    (output/'review_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(label, '视频已生成', str(output/'review.mp4'), flush=True)


def main():
    """对一个已有工况生成可供人工观察的同源回放。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--case', choices=LABELS, required=True)
    parser.add_argument('--interface-label', default='', help='接口名称，写入回放说明栏')
    args = parser.parse_args()
    label = f'{args.interface_label} · {LABELS[args.case]}' if args.interface_label else LABELS[args.case]
    render_review(args.run, args.out, label)


if __name__ == '__main__':
    main()
