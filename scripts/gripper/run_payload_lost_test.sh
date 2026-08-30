#!/bin/bash
# 载荷**意外**脱落 → 内环增益复位 的验证 (2026-08-30)
#
# ── 为什么需要单独的测法 ──────────────────────────────────────────────
#   正常流程里 drop 是 NMPC 自己发起的(_grip_drop_phase),grip_dropped 一置位
#   就已经复位增益了 —— 新的看门狗通路**永远走不到**。要真测,必须让载荷在
#   NMPC 不知情的情况下掉。这也正是真机故障模式:夹爪失效 / 载荷被扯掉。
#
#   ⚠️ **不能用 /gripper/release**(2026-08-30 实测踩到):on_release 只 detach、
#   不动 enabled,而 tick() 每 100ms 还在判 should_attach —— box 刚脱开的那一瞬
#   仍满足 d_xy=0.042/dz=0.515/v_rel=0(还没掉下去、和机体同速),**77ms 后就被
#   重新吸上**,推力上看不到任何阶跃,看门狗当然不响。
#   正确做法是发 /gripper/enable=false:它同时 detach + 关掉重吸,而 nmpc_node
#   只**发布**这个话题、不订阅(实测 Subscription count=1,只有 proximity),
#   所以 NMPC 依旧不知情,grip_drop_done 保持 False。
#
# ── 两个臂 ────────────────────────────────────────────────────────────
#   ARM=off : 看门狗关 = 现状。载荷没了、增益仍停在 5× → 预期空机过增益振荡。
#             这一臂是"问题存在"的证据,可能炸机,属预期。
#   ARM=on  : 看门狗开。预期 MHE 发 /mhe/payload_lost、NMPC 复位增益到 1.0。
#
# 用法: ARM=on  bash src/scripts/gripper/run_payload_lost_test.sh
#       ARM=off bash src/scripts/gripper/run_payload_lost_test.sh
set -u
WS="/home/clear/ros2_ws_HJH"; RUNDIR="$WS/nmpc_test_results"
ARM="${ARM:-on}"; OBS_SEC="${OBS_SEC:-90}"; REL_AFTER_DYN="${REL_AFTER_DYN:-25}"

# --- 跨工作区守卫(共享 px4_sitl_default,他人栈会互杀) ---
foreign=$(ps -eo pid,cmd | grep -E "bin/px4|make px4_sitl|ninja gz_x500|gz sim" \
          | grep -v grep | grep -v ugrep | grep -v "$WS")
if [ -n "$foreign" ]; then
  echo "[pl] 拒启:检测到非本工作区 px4/gz/make 在跑。"; echo "$foreign" | cut -c1-90
  exit 2
fi
exec 9>"/tmp/payload_lost_test.lock"
if ! flock -n 9; then echo "[pl] 已有实例在跑,退出。"; exit 1; fi

cleanup() {
  for pat in "run_gripper[_]headless.sh" "px4_sitl_default/bin/px4" "gz sim" "/gz " \
             "mavros/mavros_node" "lib/offboard_test_acados/proximity_gripper_node" \
             "lib/offboard_test_acados/acados_nmpc_node" \
             "lib/offboard_test_acados/mhe_node" \
             "topic pub -r 2 /gripper/enable" "ros_gz_bridge" "gcs_heartbeat.py"; do
    for pp in $(pgrep -f "$pat" 2>/dev/null); do kill -9 "$pp" 2>/dev/null || true; done
  done
  sleep 3
}

cleanup
# 起飞前记下已有的 nmpc 日志,后面只认**新**出现的那个。
# 2026-08-30 踩到:原先用"最新一个 + mtime 在 90s 内"认本轮日志,而上一轮刚
# 清掉的栈其日志 mtime 同样很新 -> 认成上一轮,在旧日志里 grep 到 DYNAMIC
# 就提前发释放命令,那时新栈还在 PX4 启动阶段。
BEFORE=$(ls "$RUNDIR"/grip_nmpc_*.log 2>/dev/null | tr '\n' ' ')
echo "[pl] === ARM=$ARM 起飞(计划 drop 关闭,只测意外脱落)==="
# ⚠️ 9>&- 关掉继承的 flock fd,否则子进程持锁变孤儿
MHE_PAYLOAD_LOST_WATCH=$( [ "$ARM" = "on" ] && echo 1 || echo 0 ) \
NMPC_GEOM_SOURCE=online GRIP_PAYLOAD_KG=0.3 GRIP_ECC_Y=0.10 GRIP_PAYLOAD_ENVELOPE=0.5 \
  USE_MHE=true MHE_C_XY_EST=true DJ_TRACK_MEST=true \
  GRIP_DYNAMIC=true GRIP_DYN_R=10.0 GRIP_DYN_W=0.283 GRIP_DYN_RAMP=9.36 \
  GRIP_Z_HIGH=6.0 GRIP_LIFT_DUR=8.4 GRIP_DROP_AFTER=0.0 \
  ATTACH_WINDOW_SEC=140.0 EVAL_TRUE_PAYLOAD_MASS=0.3 \
  bash "$WS/src/scripts/gripper/run_gripper_headless.sh" \
  > "$RUNDIR/pl_launch_$(date +%Y%m%d_%H%M%S).log" 2>&1 9>&- &

# 等本轮的 nmpc 日志出现并进入 figure8
NLOG=""
for i in $(seq 1 90); do
  for f in $(ls -t "$RUNDIR"/grip_nmpc_*.log 2>/dev/null); do
    case " $BEFORE " in *" $f "*) ;; *) NLOG="$f"; break ;; esac
  done
  [ -n "$NLOG" ] && break
  sleep 2
done
if [ -z "$NLOG" ]; then echo "[pl] 没等到本轮 nmpc 日志"; cleanup; exit 4; fi
echo "[pl] nmpc log = $NLOG"
for i in $(seq 1 120); do
  grep -q "DYNAMIC: switch to figure8" "$NLOG" 2>/dev/null && break
  sleep 2
done
if ! grep -q "DYNAMIC: switch to figure8" "$NLOG" 2>/dev/null; then
  echo "[pl] 没进 DYNAMIC,放弃"; cleanup; exit 3
fi
echo "[pl] 已进 figure8,再等 ${REL_AFTER_DYN}s 后从**外部**释放载荷"
sleep "$REL_AFTER_DYN"

# ⚠️ set -u 下 source ROS 的 setup.bash 会**当场退出脚本**(它内部引用了未定义
# 变量),而这里 stderr 又被吞掉 -> 表现为脚本在这一行静默消失,前面 echo 过的
# 进度还在,像是"卡住了"。2026-08-30 连踩两次(两个臂都没发出释放命令)。
set +u
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
source "$WS/install/setup.bash" >/dev/null 2>&1
set -u
echo "[pl] >>> /gripper/enable=false (detach + 关重吸;NMPC 不知情)"
ros2 topic pub --once /gripper/enable std_msgs/msg/Bool "{data: false}" >/dev/null 2>&1

echo "[pl] 观察 ${OBS_SEC}s"
sleep "$OBS_SEC"

MLOG="${NLOG/grip_nmpc_/grip_mhe_}"
echo "[pl] ---- 结果 ----"
grep -h "payload-lost\|PAYLOAD LOST" "$MLOG" "$NLOG" 2>/dev/null | head -5
grep -h "MC_ROLLRATE_K" "$NLOG" 2>/dev/null
echo "[pl] peak pos_err = $(grep -o 'pos_err=[0-9.]*' "$NLOG" | cut -d= -f2 | sort -g | tail -1)"
echo "[pl] solve failed  = $(grep -c 'solve failed' "$NLOG")"
cleanup
echo "[pl] done (nmpc=$NLOG)"
