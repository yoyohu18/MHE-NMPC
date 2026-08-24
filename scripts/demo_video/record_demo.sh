#!/bin/bash
# 成果演示视频 —— 实拍层录制(2026-07-20)。
#
# 起一次 run_sitl_gripper_viz.sh 完整任务(Gazebo GUI 物理飞行 + RViz 控制端),
# 等 NMPC 真正开始发布任务时间后启动 ffmpeg x11grab 全屏录制,并把**录制起点
# 墙钟**写进 meta.json —— 这是画面与数据动画层对齐的唯一依据:
#     录屏第 k 秒  ==  任务时间 (rec_start + k - off)
# 其中 off 由 make_overlay.py 从同一批日志算出。两者共用墙钟,不需要目测对齐。
#
# 用法:
#   bash src/scripts/demo_video/record_demo.sh              # 默认录 125s
#   DEMO_DURATION=140 bash src/scripts/demo_video/record_demo.sh
#
# 录完后:
#   python3 src/scripts/demo_video/make_overlay.py --stamp <打印出来的 stamp>
#   bash   src/scripts/demo_video/compose_demo.sh  <stamp>
set -u

WS="/home/clear/ros2_ws_HJH"
LOGDIR="$WS/nmpc_test_results"
OUTDIR="$LOGDIR/demo_video"
DURATION="${DEMO_DURATION:-150}"     # preroll + t≈115s 的 T_END,留余量
FPS="${DEMO_FPS:-25}"
mkdir -p "$OUTDIR"

if [ -z "${DISPLAY:-}" ]; then echo "没有 DISPLAY,x11grab 无法录制"; exit 1; fi
# 双屏时默认只录主屏 —— 整块 3840x1080 缩进 1920 宽的成片里什么都看不清。
# 覆盖示例:DEMO_GRAB=1920x1080+1920+0 录副屏;DEMO_GRAB=3840x1080+0+0 录全部。
if [ -n "${DEMO_GRAB:-}" ]; then
  GRAB="$DEMO_GRAB"
elif command -v xrandr >/dev/null 2>&1; then
  # "1920/527x1080/296+0+0" 里的 /527 /296 是物理毫米,去掉才是像素几何
  GRAB=$(xrandr --listmonitors | awk '/\*/{gsub(/\/[0-9]+/,"",$3); print $3; exit}')
  [ -z "$GRAB" ] && GRAB="1920x1080+0+0"
else
  GRAB="1920x1080+0+0"
fi
GRAB_SIZE="${GRAB%%+*}"; GRAB_OFF="+${GRAB#*+}"
echo "录制区域: $GRAB_SIZE 偏移 $GRAB_OFF  DISPLAY=$DISPLAY  时长=${DURATION}s @ ${FPS}fps"

# 1. 起完整可视化仿真(它自己会清理残留进程、清 PX4 落盘参数)
#    MARKER 是"本次启动"的时间基准:只认比它新的日志,否则会像 2026-07-20
#    第一次那样匹配到几天前的旧 gviz 日志、瞬间误判就绪、把启动画面录满 125s。
MARKER="$OUTDIR/.launch_marker"
touch "$MARKER"
# ⚠️2026-08-21:光靠 mtime(-newer $MARKER)会匹配到**上一轮还在写**的日志——
# 它的 mtime 一直在更新,必然比 marker 新。后果是 stamp 指向旧 run、相机跟随
# 在新 Gazebo 起来之前就调掉(录出来全程远景)、meta 里的 nmpc_log 也是错的。
# 改成文件名快照对比:只认快照里没有的**新文件名**。
ls "$LOGDIR"/gviz_nmpc_*.log 2>/dev/null | xargs -r -n1 basename > "$OUTDIR/.logs_before"
echo ">>> 启动 run_sitl_gripper_viz.sh ..."
bash "$WS/src/scripts/gripper/run_sitl_gripper_viz.sh" > "$OUTDIR/viz_launch.log" 2>&1 &
LAUNCH_PID=$!

echo ""
echo "########################################################"
echo "#  Gazebo / RViz 窗口起来后请立刻摆好版面:"
echo "#     Gazebo 铺满主屏, RViz 放右上角"
echo "#  检测到 RViz 进程就自动开录(约 30~40s 后),"
echo "#  开录早于任务 t=0,摆窗口的动作会被 compose 自动切掉。"
echo "########################################################"
echo ""

# 2. 等**本次**新建的 NMPC 日志文件出现。用文件出现而不是 t= 行做判据:
#    nohup 重定向建文件时 NMPC 刚启动、任务 t=0 还没到,此刻开录才不丢开头。
echo ">>> 等待本次仿真的 NMPC 日志(最多 300s)..."
STAMP=""; NMPC_LOG=""
for i in $(seq 1 300); do
  NMPC_LOG=$(for f in "$LOGDIR"/gviz_nmpc_*.log; do
               [ -e "$f" ] || continue
               grep -qxF "$(basename "$f")" "$OUTDIR/.logs_before" || echo "$f"
             done | sort | tail -1)
  if [ -n "$NMPC_LOG" ]; then
    STAMP=$(basename "$NMPC_LOG" .log); STAMP=${STAMP#gviz_nmpc_}
    break
  fi
  sleep 1
done
if [ -z "$STAMP" ]; then
  echo "!!! 300s 内没等到新的 NMPC 日志,放弃录制。看 $OUTDIR/viz_launch.log"
  exit 1
fi
echo ">>> 本次 stamp=$STAMP (日志 $NMPC_LOG)"

# 3. 存全量窗口树供 compose 裁剪参考(标题不固定,存全量比 grep 可靠)
xwininfo -root -tree 2>/dev/null > "$OUTDIR/windows_$STAMP.txt" || true
grep -iE 'gazebo|rviz|qground' "$OUTDIR/windows_$STAMP.txt" | head -10 || true

# 3b. 把 Gazebo 相机吸到无人机上。默认远景视角里无人机只有几个像素,抓取和
#     投放这两下根本看不清。注意 viz 脚本里的 PX4_GZ_NO_FOLLOW=1 是无效变量
#     (PX4 仓库里根本没有它),真正管用的是 gz-sim GUI 的 /gui/follow 服务。
export GZ_CONFIG_PATH="${GZ_CONFIG_PATH:-/usr/share/gz}"
FOLLOW_TARGET="${DEMO_FOLLOW:-x500_0}"
FOLLOW_OFF="${DEMO_FOLLOW_OFFSET:-x: -3.5, y: 0.0, z: 1.6}"
if [ -n "$FOLLOW_TARGET" ] && command -v gz >/dev/null 2>&1; then
  # 重试:GUI 进程在、但 /gui/follow 服务晚几秒才注册的情况实测存在
  FOLLOW_OK=1
  for _try in 1 2 3 4 5; do
    gz service -s /gui/follow --reqtype gz.msgs.StringMsg \
       --reptype gz.msgs.Boolean --timeout 3000 \
       --req "data: \"$FOLLOW_TARGET\"" 2>/dev/null | grep -q 'data: true' \
      && { FOLLOW_OK=0; break; }
    sleep 2
  done
  if [ $FOLLOW_OK -eq 0 ]; then
    # ⚠️2026-08-23:offset 原来是"发一次 + || true",不校验返回。实测 /gui/follow
    # 刚重试成功时 GUI 还没准备好接 offset,这一发静默丢掉 -> 相机用 gz 默认跟随
    # 距离,成片里无人机只有几十个像素(跟"完全没跟随"肉眼几乎分不出,判据是
    # 地平线会随机体转动)。和 follow 一样加重试 + 校验 data: true。
    OFF_OK=1
    for _try in 1 2 3 4 5; do
      gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d \
         --reptype gz.msgs.Boolean --timeout 3000 \
         --req "$FOLLOW_OFF" 2>/dev/null | grep -q 'data: true' \
        && { OFF_OK=0; break; }
      sleep 2
    done
    if [ $OFF_OK -eq 0 ]; then
      # ⚠️2026-08-23 take2:服务返回 data: true,画面却仍是远景(无人机约 20px,
      # 按视场角反推距离 ~30m 而非 offset 说的 6m)——offset 像是被 follow 刚生效
      # 那阵的状态吃掉了。等相机稳下来再补发一次,成本几乎为零。
      sleep 3
      gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d \
         --reptype gz.msgs.Boolean --timeout 3000 \
         --req "$FOLLOW_OFF" >/dev/null 2>&1 || true
      echo ">>> Gazebo 相机已跟随 $FOLLOW_TARGET (offset: $FOLLOW_OFF, 已补发一次)"
    else
      echo "!!! 相机跟随已生效但 offset 设置失败 —— 成片里无人机会很小,建议重录"
    fi
  else
    echo "!!! 相机跟随服务调用失败 —— 请手动在 Gazebo 里右键 $FOLLOW_TARGET"
    echo "    选 Follow,再用滚轮拉近。不然成片里无人机只有几个像素。"
  fi
fi

# 3c. 相机二次校正(2026-08-23 定案)。上面那次 offset **服务返回 data: true 却常常
#     不生效** —— 08-23 take2/take3 连续两轮画面都是 30~40m 远景。同一条命令等仿真
#     跑起来后手动发就必定生效(实测无人机从十几像素变成清晰可辨),所以这是**时机**
#     问题不是参数问题:开录前 GUI 刚起来,follow 还没稳,offset 被丢掉。
#     放到开录后 8s 再补一遍,那时画面已在录,但任务 t<0(k0 实测 21~26s),
#     compose 会把这段 preroll 连同相机跳变一起切掉。
if [ -n "$FOLLOW_TARGET" ] && command -v gz >/dev/null 2>&1; then
  ( sleep 8
    gz service -s /gui/follow --reqtype gz.msgs.StringMsg \
       --reptype gz.msgs.Boolean --timeout 3000 \
       --req "data: \"$FOLLOW_TARGET\"" >/dev/null 2>&1
    sleep 2
    gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d \
       --reptype gz.msgs.Boolean --timeout 3000 \
       --req "$FOLLOW_OFF" >/dev/null 2>&1
  ) &
fi

# 4. 开录。rec_start 必须紧贴 ffmpeg 启动那一刻取,后面对齐全靠它
RAW="$OUTDIR/screen_$STAMP.mkv"
REC_START=$(date +%s.%N)
ffmpeg -y -hide_banner -loglevel warning \
  -f x11grab -framerate "$FPS" -video_size "$GRAB_SIZE" \
  -i "${DISPLAY}${GRAB_OFF}" \
  -t "$DURATION" -c:v libx264 -preset ultrafast -crf 18 -pix_fmt yuv420p \
  "$RAW"
REC_RC=$?
echo ">>> 录制结束 rc=$REC_RC -> $RAW"

cat > "$OUTDIR/record_$STAMP.meta.json" <<EOF
{
  "stamp": "$STAMP",
  "rec_start_wall": $REC_START,
  "duration_sec": $DURATION,
  "fps": $FPS,
  "screen": "$GRAB",
  "raw": "$RAW",
  "nmpc_log": "$NMPC_LOG"
}
EOF
echo ">>> meta -> $OUTDIR/record_$STAMP.meta.json"
echo
echo "下一步:"
echo "  python3 $WS/src/scripts/demo_video/make_overlay.py --stamp $STAMP"
echo "  bash    $WS/src/scripts/demo_video/compose_demo.sh $STAMP"
echo
echo "注意:仿真仍在后台跑(launch pid=$LAUNCH_PID)。收栈:"
echo "  pkill -9 -f 'px4_sitl|gz sim|mhe_node|acados_nmpc_node|rviz2|ros_gz_bridge'"
