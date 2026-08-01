#!/bin/bash
# 纯轨迹跟踪速度阶梯(高机动诊断,2026-07-29)。
#
# 目的:干净地测"跟踪误差 vs 轨迹速度"的关系,回答 e ≈ v·τ(固定执行时延)
# 是否成立。**不做 drop**(PAYLOAD_ENABLED=false)——drop 会同时改有效质量、
# 撞 MHE 质量下界(1.961kg vs wrench 制造的 1.564kg 有效质量)、并引入一个
# 与速度无关的大扰动,三者混在一起测不出速度效应。首批带 drop 的对照就是
# 这么废掉的:baseline 轮 drop 静默未生效、高机动轮生效了,两轮差的是"drop
# 有没有发生"而不是速度。
#
# 控制变量:轨迹尺度固定 r=5.0(包络 10m×5m),只扫 w;权重缩放固定 off
# (首批已初步排除,on/off 的 pos_err 差 0.331 vs 0.321 在噪声内);
# payload/MHE/NMPC 其余参数全部默认。只有速度在变。
#
# v_peak = √2·r·w  =>  w = v/(√2·r)。ramp 用 auto(fig8_ramp=-1),否则低速档
# 振幅渐增本身的额外速度会造成起步冲击。
#
# 每档飞行时长按 ramp + N_LAPS 圈自动算——低速档周期极长(v=0.42 时
# 2π/w=106s),固定时长会导致低速档连一圈都跑不完、和高速档不可比。
#
# 环境变量: SPEEDS(默认 5 档) N_LAPS(默认 2) TRAJ_R(默认 5.0)
#           TRAJ_RAMP(默认 3.0,固定不随 w 变——见下面循环里的说明)
#           SETTLE(默认 3.0,统计窗口在 ramp 结束后再让残差衰减这么久)
set -e

WS="/home/clear/ros2_ws_HJH"
RUNDIR="$WS/nmpc_test_results"
SPEEDS="${SPEEDS:-0.42 1.0 1.5 2.0 3.0}"
N_LAPS="${N_LAPS:-2}"
TRAJ_R="${TRAJ_R:-5.0}"
TRAJ_RAMP="${TRAJ_RAMP:-3.0}"
# 起振残差衰减余量。gripper 那轮实测 ramp 结束后 1.7s 仍有 1.7× 稳态残差、约 5s
# 收敛,取 3.0 是折中。**它不影响档间可比性**(各档同值),只影响丢掉多少数据:
# 最快档 v=3.0 周期 14.8s、飞 2 圈 29.6s,丢 3s 约 10%,可接受。
SETTLE="${SETTLE:-3.0}"
BOOT_TIMEOUT="${BOOT_TIMEOUT:-240}"
MANIFEST="$RUNDIR/speed_ladder_$(date +%Y%m%d_%H%M%S).manifest"

# run_sitl_headless.sh 自己**没有** cleanup 段(它假定批量驱动脚本负责清栈)。
# 单独连跑会起两套 PX4/MAVROS 抢 14540 端口、数据全废——首批就踩了。
cleanup_sim() {
  local self=$$
  for pat in "px4_sitl_default/bin/px4" "gz sim" "mavros/mavros_node" \
             "mavros px4.launch" "lib/offboard_test_acados/acados_nmpc_node" \
             "lib/offboard_test_acados/mhe_node" "ros_gz_bridge" \
             "gcs_heartbeat.py" "make px4_sitl" "run_sitl_headless.sh"; do
    for pp in $(pgrep -f "$pat" 2>/dev/null); do
      [ "$pp" = "$self" ] && continue
      kill -9 "$pp" 2>/dev/null || true
    done
  done
  sleep 3
  # PX4 参数持久化护栏:ParamSetV2 改过的增益会落盘到 parameters.bson,
  # 干净重启不清会跨局污染(见记忆 px4-param-persistence-guard)。
  find /home/clear/PX4-Autopilot/build/px4_sitl_default/rootfs \
      -maxdepth 1 -name 'parameters*.bson' -delete 2>/dev/null || true
}

trap 'echo "[ladder] interrupted"; cleanup_sim; exit 130' INT TERM

echo "# speed_ladder r=$TRAJ_R laps=$N_LAPS drop=OFF ramp=$TRAJ_RAMP settle=$SETTLE" > "$MANIFEST"
echo "# v_peak w ramp_s need_t stamp status settle_s" >> "$MANIFEST"

for V in $SPEEDS; do
  # w / 需要飞多久按解析式算。**ramp 固定不再用 auto**(2026-07-30 定案):
  # auto_ramp_time=max(4.0,2.65/w) 是纯相对判据(把振幅渐增的额外速度压到轨迹
  # 特征速度的一半),w 越小要的 ramp 越长——本阶梯最慢档 v=0.42 会要 44.6s,
  # 只为把起振倾角压到 0.28°,绝对意义上毫无必要(执行器上限 60°)。
  # 更要命的是它**破坏本实验的受控性**:t_ss=2.0+ramp+settle 是统计窗口起点,
  # ramp 随档位从 6.25s 变到 44.6s = 各档窗口起点全不同,而本实验声称的控制变量
  # 只有速度。固定 ramp 后各档窗口起点统一,才真的只剩速度在变。
  # 固定 3.0s 的代价:各档起振峰值倾角 3.8°(v=0.42)~24.6°(v=3.0),全在预算内。
  RAMP="$TRAJ_RAMP"
  read -r W NEED <<EOF
$(python3 -c "
import math
v=$V; r=$TRAJ_R; laps=$N_LAPS
w=v/(math.sqrt(2)*r)
# 2.0=figure8 hover_time; +$SETTLE=起振残差衰减余量(见 aggregate 的 t_ss); +15 余量
need=2.0+$RAMP+$SETTLE+laps*(2*math.pi/w)+15.0
print(f'{w:.4f} {need:.0f}')
")
EOF
  echo "[ladder] === v_peak=$V m/s  w=$W  ramp=${RAMP}s  需飞 ${NEED}s ==="
  cleanup_sim

  FIG8_R=$TRAJ_R FIG8_W=$W FIG8_RAMP=$RAMP \
  TRAJ_SCALE_WEIGHTS=false PAYLOAD_ENABLED=false \
    nohup bash "$WS/src/scripts/masschanger/run_sitl_headless.sh" \
      > "$RUNDIR/ladder_launch_v$V.log" 2>&1 &

  # 起栈完成 -> 拿精确日志名(**不用 ls -t**,它会匹配到上一档的旧日志)
  waited=0
  until grep -q "headless stack up" "$RUNDIR/ladder_launch_v$V.log" 2>/dev/null; do
    sleep 5; waited=$((waited+5))
    if [ "$waited" -gt "$BOOT_TIMEOUT" ]; then
      echo "[ladder] !! v=$V 起栈超时"; echo "$V $W $RAMP $NEED - BOOT_TIMEOUT" >> "$MANIFEST"
      continue 2
    fi
  done
  NLOG=$(grep -oE "nmpc=[^ ]+" "$RUNDIR/ladder_launch_v$V.log" | head -1 | cut -d= -f2)
  STAMP=$(basename "$NLOG" | sed 's/acados_nmpc_node_//;s/\.log//')
  echo "[ladder]   nmpc log: $NLOG"

  # 等飞够 NEED 秒的 nmpc_time(日志里 t=…s),或崩溃/超时
  waited=0; status=TIMEOUT
  while [ "$waited" -lt $((NEED + 180)) ]; do
    if grep -q "Traceback" "$NLOG" 2>/dev/null; then status=CRASH; break; fi
    # 取日志里最后一个 t=…s,判断是否已过 NEED
    last=$(grep -oE "^\[INFO\].*\| pos_err" "$NLOG" 2>/dev/null | tail -1 \
           | grep -oE "t=[0-9.]+s" | tr -d 'ts=' | cut -d. -f1)
    if [ -n "$last" ] && [ "$last" -ge "${NEED%.*}" ]; then status=ok; break; fi
    sleep 10; waited=$((waited+10))
  done
  echo "[ladder]   -> $status"
  # settle 写进 manifest 第 7 列,让 aggregate 用**本批次实际用的**余量切窗口,
  # 不必在两处各维护一份常数(旧 manifest 无此列,aggregate 缺列时按 0.0 兼容)。
  echo "$V $W $RAMP $NEED $STAMP $status $SETTLE" >> "$MANIFEST"
done

cleanup_sim
echo "[ladder] 全部完成,manifest: $MANIFEST"
