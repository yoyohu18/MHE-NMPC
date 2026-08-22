#!/bin/bash
# MHE 窗口长度 x 速度 析因扫描(2026-08-18)
#
# 起因:08-18 的 4m/s 两轮(n=2)发现 m_est **静态准 −0.17%、4m/s 机动中低估 8%**。
# 排除法已砍掉全部常数误差源(静态准 → 推力增益 THRUST_CAL_GAIN 之类若有 8% 乘性
# 误差,悬停 T≈m·g 时就该暴露),剩下三个候选都与运动状态相关:
#   (a) MHE 窗口跨度 N·dt 占 8 字周期的比例(m 是窗口内唯一在线自由度,
#       m_dot=0、dJ/c_xy 是已知参数 → 窗口内一切模型失配只能由 m 吸收)
#   (b) kd=0.05 线性阻力在高速不足
#   (c) odom 仅 31Hz → 姿态滞后
#
# **为什么是析因而不是单纯扫 N**:改 N 同时动两件事——窗口跨度,以及代价里测量项
# 相对到达代价的总权重(W=block_diag(R,Q) 是 per-stage 常数不随 N 归一化,Q0 只在
# stage 0 一份)。单扫 N 无法分离。(速度 x N) 析因里三个假设有完全不同的指纹:
#
#            N=10(1.0s)   N=20(2.0s)   N=40(4.0s)     <- 窗口跨度
#   2m/s(周期22.2s)  4.5%        9%          18%      <- 窗口占周期比
#   4m/s(周期11.1s)  9%          18%         36%
#
#   占比假设 → 沿等占比对角线相等 (2m/s,N20)=(4m/s,N10) 且 (2m/s,N40)=(4m/s,N20)
#   纯 N 效应(含权重混淆) → 同列相等
#   纯速度效应(kd/姿态滞后) → 同行相等
#
# ⚠️ **必须串行**:mhe_solver_builder.codegen_dir 不含 N,各档共用一份生成代码,
#    并发会互相覆盖。改 N 那一轮 acados 重新生成 solver,启动慢约 30s。
# ⚠️ drop 关闭(GRIP_DROP_AFTER=0):本实验测的是**带载稳态估计偏差**,drop 后撞
#    下界会截断数据、污染统计。m_min 固定用默认 1.961,分析脚本会检查是否触界。
# ⚠️ ramp 用各档**已定案值**(2m/s→3.0、4m/s→-1 即 auto 4.68),不强行统一:两档的
#    判据 ramp≥2.65/w 互不相容(低速反而要更长)。manifest 记录实际 ramp,
#    aggregate 按各自 ramp 算统计窗口起点 t_ss。
# ⚠️ **飞相同圈数而非相同时长**(LAPS,默认 3),保证各格轨迹相位覆盖一致。
#
# 用法: [LAPS=3 NLIST="10 20 40" VLIST="2 4"] bash src/scripts/gripper/run_mhe_window_scan.sh
set -u
WS="/home/clear/ros2_ws_HJH"; RUNDIR="$WS/nmpc_test_results"
LAPS="${LAPS:-3}"; NLIST="${NLIST:-10 20 40}"; VLIST="${VLIST:-2 4}"
STAMP=$(date +%Y%m%d_%H%M%S)
MAN="$RUNDIR/mhe_window_scan_${STAMP}.txt"
echo "# mhe_window_scan $STAMP : v w N ramp settle laps nmpc_stamp status" > "$MAN"
echo "[scan] manifest: $MAN"

SETTLE=3.0

cleanup() {
  for pat in "px4_sitl_default/bin/px4" "make px4_sitl" "gz sim" "ruby" \
             "mavros/mavros_node" "mavros px4.launch" "ros_gz_bridge" "parameter_bridge" \
             "offboard_test_acados" "robot_state_publisher" "rviz2" \
             "gcs_heartbeat.py" "topic pub -r 2 /gripper/enable" "ninja gz_x500"; do
    pids=$(pgrep -f "$pat" 2>/dev/null | grep -vw "$$" | grep -vw "$PPID")
    [ -n "$pids" ] && kill -9 $pids 2>/dev/null
  done; sleep 8
}
trap 'echo "[scan] interrupted"; cleanup; exit 130' INT TERM

cleanup   # 起批前先清干净(跨工作区残留会抢 14540 端口)

for V in $VLIST; do
  # v_peak = sqrt(2)*r*w, r=5.0 固定 -> w = v/(sqrt(2)*5)
  W=$(python3 -c "import math;print(f'{$V/(math.sqrt(2)*5.0):.4f}')")
  RAMP=$([ "$V" = "2" ] && echo "3.0" || echo "-1")
  RAMP_EFF=$(python3 -c "
w=$W; r=$RAMP
print(f'{max(4.0, 2.65/w):.2f}' if r<0 else f'{r:.2f}')")
  FLY=$(python3 -c "
import math; print(f'{2.0+$RAMP_EFF+$SETTLE+$LAPS*(2*math.pi/$W)+12.0:.0f}')")
  for N in $NLIST; do
    echo "[scan] === v=${V}m/s w=$W N=$N ramp=$RAMP(eff $RAMP_EFF) 飞行约 ${FLY}s ==="
    LAUNCH="$RUNDIR/mhewin_launch_${STAMP}_v${V}_N${N}.log"
    MHE_N=$N \
    GRIP_PAYLOAD_KG=0.3 GRIP_ECC_Y=0.10 USE_MHE=true MHE_C_XY_EST=true \
      GRIP_GEOM_MP_FLOOR=0.15 NMPC_GEOM_SOURCE=online \
      GRIP_DYNAMIC=true GRIP_DYN_R=5.0 GRIP_DYN_W=$W GRIP_DYN_RAMP=$RAMP \
      GRIP_DROP_AFTER=0.0 ATTACH_WINDOW_SEC=40.0 \
      bash "$WS/src/scripts/gripper/run_gripper_headless.sh" > "$LAUNCH" 2>&1

    NMPC=""; for k in $(seq 1 20); do
      NMPC=$(grep -oE "/home/[^ ]*grip_nmpc_[0-9_]+\.log" "$LAUNCH" 2>/dev/null | head -1)
      [ -n "$NMPC" ] && [ -f "$NMPC" ] && break; sleep 5
    done
    if [ -z "$NMPC" ]; then
      echo "[scan] v=$V N=$N FAIL: 拿不到 nmpc 日志名"; tail -5 "$LAUNCH"
      echo "$V $W $N $RAMP_EFF $SETTLE $LAPS - NOLOG" >> "$MAN"; cleanup; continue
    fi
    NSTAMP=$(echo "$NMPC" | grep -oE "[0-9]{8}_[0-9]{6}")
    # 等 DYNAMIC 切换出现,再从那一刻起飞满 FLY 秒
    waited=0; got=0
    while [ "$waited" -lt 120 ]; do
      grep -aq "DYNAMIC: switch to figure8" "$NMPC" 2>/dev/null && { got=1; break; }
      sleep 5; waited=$((waited+5))
    done
    if [ "$got" -eq 0 ]; then
      echo "[scan] v=$V N=$N FAIL: 120s 内没进 DYNAMIC"
      echo "$V $W $N $RAMP_EFF $SETTLE $LAPS $NSTAMP NODYN" >> "$MAN"; cleanup; continue
    fi
    echo "[scan]   已进 DYNAMIC,飞 ${FLY}s"
    sleep "$FLY"
    ST=ok; grep -aq "Traceback" "$NMPC" && ST=TRACEBACK
    echo "$V $W $N $RAMP_EFF $SETTLE $LAPS $NSTAMP $ST" >> "$MAN"
    echo "[scan]   v=$V N=$N stamp=$NSTAMP -> $ST"
    cleanup
  done
done
echo "[scan] done. manifest: $MAN"
echo "分析: python3 $WS/src/scripts/gripper/aggregate_mhe_window.py $MAN"
