#!/bin/bash
# 成果演示视频 —— 合成(2026-07-20)。
#
# 把三层拼成成片:
#   实拍层  screen_<stamp>.mkv        (record_demo.sh 的全屏录制)
#   数据层  overlay_<stamp>.webm      (make_overlay.py,带 alpha,贴底部)
#   字幕层  drawtext 阶段卡           (阶段时刻取自 overlay meta 的 phases)
#
# 对齐:两个 meta 都用墙钟。任务时间 0 落在录屏的第 (off - rec_start) 秒,
# 记作 k0。k0>0 就把录屏前 k0 秒切掉;k0<0 说明录制启动时任务已经跑了 -k0 秒,
# 那就让数据层从 -k0 处起播。全自动,不需要目测对齐 drop 那一帧。
#
# 用法:
#   bash src/scripts/demo_video/compose_demo.sh <stamp>
# 可选裁剪(默认整屏缩放;想只要 Gazebo 那块就给它 w:h:x:y):
#   MAIN_CROP=1600:900:0:60 bash .../compose_demo.sh <stamp>
# 可选画中画(RViz 那块,给了才叠):
#   PIP_CROP=800:600:1100:100 MAIN_CROP=... bash .../compose_demo.sh <stamp>
set -eu

STAMP="${1:?用法: compose_demo.sh <stamp>}"
WS="/home/clear/ros2_ws_HJH"
D="$WS/nmpc_test_results/demo_video"
# DEMO_LANG=en 出英文版(字幕卡 + 数据层都英文)。数据层需先用
# make_overlay.py --lang en 生成 overlay_<stamp>_en.webm;录屏层中英共用。
DEMO_LANG="${DEMO_LANG:-zh}"
SFX=''; [ "$DEMO_LANG" = zh ] || SFX="_$DEMO_LANG"
REC_META="$D/record_$STAMP.meta.json"
OVL_META="$D/overlay_${STAMP}${SFX}.meta.json"
RAW="$D/screen_$STAMP.mkv"
OVL="$D/overlay_${STAMP}${SFX}.webm"
OUT="$D/demo_${STAMP}${SFX}.mp4"
FONT="/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"

for f in "$REC_META" "$OVL_META" "$RAW" "$OVL"; do
  [ -e "$f" ] || { echo "缺文件: $f"; exit 1; }
done

W=1920; H=1080; OVL_H=340; MAIN_H=$((H - OVL_H))

# ---- 时间对齐 ----
read -r K0 TEND FPS < <(python3 - "$REC_META" "$OVL_META" <<'PY'
import json, sys
rec = json.load(open(sys.argv[1])); ovl = json.load(open(sys.argv[2]))
k0 = ovl['off'] - rec['rec_start_wall']
print(f"{k0:.3f} {ovl['t_end']:.3f} {ovl['fps']}")
PY
)
echo "对齐: 任务时间 t=0 位于录屏第 ${K0}s ; 成片时长 ${TEND}s @ ${FPS}fps"

SS_MAIN=0; SS_OVL=0
if (( $(echo "$K0 >= 0" | bc -l) )); then
  SS_MAIN="$K0"
else
  SS_OVL=$(echo "0 - $K0" | bc -l)
  echo "  (录制晚于任务起点 ${SS_OVL}s,数据层相应快进)"
fi

MAIN_FILT="scale=${W}:${MAIN_H}:force_original_aspect_ratio=decrease,pad=${W}:${MAIN_H}:(ow-iw)/2:(oh-ih)/2:color=0x101014"
[ -n "${MAIN_CROP:-}" ] && MAIN_FILT="crop=${MAIN_CROP},${MAIN_FILT}"

# ---- 阶段字幕卡:每个阶段起点后显示 4.5s ----
SUBS=$(python3 - "$OVL_META" "$FONT" "$DEMO_LANG" <<'PY'
import json, sys
meta = json.load(open(sys.argv[1])); font = sys.argv[2]; lang = sys.argv[3]
CN = {'ATTACH': '接近并抓取载荷', 'LIFT': '抬升:有效质量阶跃',
      'DYNAMIC': '8 字动态轨迹跟踪', 'DROP': '投放:载荷突卸扰动'}
EN = {'ATTACH': 'Approach & grasp payload', 'LIFT': 'Lift: effective mass step',
      'DYNAMIC': 'Figure-8 trajectory tracking',
      'DROP': 'Release: sudden unloading'}
LAB = CN if lang == 'zh' else EN
parts = []
for k, t in sorted(meta['phases'].items(), key=lambda kv: kv[1]):
    txt = LAB.get(k, k).replace(':', '\\:')
    parts.append(
        f"drawtext=fontfile={font}:text='{txt}':fontcolor=white:fontsize=52:"
        f"box=1:boxcolor=0x101014@0.66:boxborderw=18:x=64:y=64:"
        f"enable='between(t,{t:.2f},{t + 4.5:.2f})'")
print(','.join(parts))
PY
)

# ---- 版面 ----
# LAYOUT=split:Gazebo 与 RViz 左右并排(双屏录制用,两边都看得清)
# LAYOUT=pip  :Gazebo 铺满 + RViz 右上角画中画(单屏录制用)
LAYOUT="${LAYOUT:-pip}"
if [ "$LAYOUT" = split ]; then
  [ -n "${MAIN_CROP:-}" ] && [ -n "${PIP_CROP:-}" ] || {
    echo "split 版面需要同时给 MAIN_CROP(Gazebo) 和 PIP_CROP(RViz)"; exit 1; }
  LW="${SPLIT_LEFT_W:-1050}"; RW=$((W - LW))
  FILTER="[0:v]crop=${MAIN_CROP},scale=${LW}:${MAIN_H}[L];\
[0:v]crop=${PIP_CROP},scale=${RW}:${MAIN_H}[R];\
[L][R]hstack=inputs=2[top];\
[top]${SUBS}[main];\
[main]pad=${W}:${H}:0:0:color=0x101014[v0];\
[v0][1:v]overlay=0:${MAIN_H}:shortest=0[v1]"
  LAST=v1
else
  if [ -n "${PIP_CROP:-}" ]; then
    PIP_CHAIN="[0:v]crop=${PIP_CROP},scale=520:-2,pad=iw+6:ih+6:3:3:color=0xf2f1ee[pip];[v1][pip]overlay=W-w-48:48[v2]"
    LAST=v2
  else
    PIP_CHAIN=""; LAST=v1
  fi
  FILTER="[0:v]${MAIN_FILT},${SUBS}[main];\
[main]pad=${W}:${H}:0:0:color=0x101014[v0];\
[v0][1:v]overlay=0:${MAIN_H}:shortest=0[v1]"
  [ -n "$PIP_CHAIN" ] && FILTER="${FILTER};${PIP_CHAIN}"
fi

echo ">>> 合成中 -> $OUT"
ffmpeg -y -hide_banner -loglevel warning \
  -ss "$SS_MAIN" -i "$RAW" \
  -ss "$SS_OVL" -c:v libvpx-vp9 -i "$OVL" \
  -filter_complex "$FILTER" -map "[$LAST]" \
  -t "$TEND" -r "$FPS" \
  -c:v libx264 -preset medium -crf 20 -pix_fmt yuv420p -movflags +faststart \
  "$OUT"

echo ">>> 完成: $OUT"
ffprobe -hide_banner -loglevel error -show_entries format=duration,size \
  -of default=nw=1 "$OUT"
