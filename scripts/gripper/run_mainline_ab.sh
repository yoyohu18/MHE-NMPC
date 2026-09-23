#!/bin/bash
# 连续主线最终验收批次(2026-09-03):
#   「NMPC 完全不消费自身 drop 状态」的 96 架次配对 A/B。
#
# ── 两个臂(唯一差别 = 控制器怎么知道载荷没了)──────────────────────────
#   A = baseline-event   CONTINUOUS_PAYLOAD_ESTIMATES=false
#                        NMPC_GEOM_SOURCE=online + NMPC_GEOM_RELEASE_MODE=event
#                        → 控制器自己发的 drop 指令一落地就把几何清零、复位增益;
#                          W6 里还额外吃 MHE 的 /mhe/payload_lost 通知。
#   B = continuous       CONTINUOUS_PAYLOAD_ESTIMATES=true(headless 默认)
#                        → 控制器只吃 MHE 的连续 m/s/J + confidence 帧,
#                          不订阅 attach 通知、不因自己发过 release 而改模型;
#                          drop 的"完成"由 no-payload confidence 持续达标定义。
#   omega_scale 两臂都开、都走 djest(用户 09-03 拍板:把增益调度对齐,
#   只留"是否消费 drop 事件"这一个变量)。MHE 侧配置两臂逐字相同。
#
# ── 六个工况(每格 8 对 = 16 架次,共 96)────────────────────────────────
#   W1 hover m0.15 ecc0.00 计划 drop
#   W2 hover m0.30 ecc0.00 计划 drop          ← 包线上界质量
#   W3 fig8 4m/s m0.15 ecc0.10 tip-drop       ← 主工作点 r10/w0.283/z6
#   W4 fig8 4m/s m0.30 ecc0.10 tip-drop       ← 最大应力
#   W5 fig8 2m/s m0.15 ecc0.10 tip-drop       ← 跨速度
#   W6 hover m0.15 ecc0.00 **意外脱落**       ← 外部发 enable=false,NMPC 不知情
#
# ⚠️ GRIP_PAYLOAD_ENVELOPE 全工况固定 0.3:它是机架规格、不是任务信息,随载荷
#    改就等于把载荷质量偷偷喂回去。0.5 已知会把内环增益推到发散(见记忆
#    dj-prior-removal),所以也不能往上抬。
#
# 用法:
#   bash src/scripts/gripper/run_mainline_ab.sh              # 全 96 架次
#   PAIRS=1 bash .../run_mainline_ab.sh                      # 每格 1 对(smoke,12 架次)
#   ONLY_W=W3 PAIRS=2 bash .../run_mainline_ab.sh            # 只跑某格
set -u

WS=/home/clear/ros2_ws_HJH
RES=$WS/nmpc_test_results
MAN=$RES/mainline_ab_manifest.csv
LOCK=/tmp/mainline_ab.lock
PAIRS=${PAIRS:-8}
ONLY_W=${ONLY_W:-}
POST_SEC=${POST_SEC:-40}        # drop 完成后继续观察(看恢复,不是看 drop 本身)
W6_REL_AFTER=${W6_REL_AFTER:-25}
W6_OBS=${W6_OBS:-60}

# --- 跨工作区守卫:px4_sitl_default 是共享构建目录,他人的栈会互杀 ---
foreign=$(ps -eo pid,cmd | grep -E "bin/px4|make px4_sitl|ninja gz_x500|gz sim" \
          | grep -v grep | grep -v ugrep | grep -v "$WS")
if [ -n "$foreign" ]; then
  echo "[批次] 拒启:检测到非本工作区 px4/gz/make 在跑"; echo "$foreign" | cut -c1-100
  exit 2
fi

# 批次锁。⚠️ fd 9 必须在子进程里关掉(9>&-),否则孤儿子进程一直持锁且 ps 看不见。
exec 9>"$LOCK"
flock -n 9 || { echo "[批次] 另一个批次正持锁,退出"; exit 1; }

cleanup() {
  for pat in "run_gripper[_]headless.sh" "px4_sitl_default/bin/px4" "make px4_sitl" \
             "gz sim" "gz_bridge" "transport13/gz-transport-topic" \
             "ros_gz_bridge" "mavros/mavros_node" \
             "lib/offboard_test_acados/acados_nmpc_node" \
             "lib/offboard_test_acados/mhe_node" \
             "lib/offboard_test_acados/proximity_gripper_node" \
             "MicroXRCEAgent" "gcs_heartbeat"; do
    for pp in $(pgrep -f "$pat" 2>/dev/null); do kill -9 "$pp" 2>/dev/null || true; done
  done
  sleep 8
}
trap 'echo "[批次] 收到中断,清栈退出"; cleanup; exit 130' INT TERM

[ -f "$MAN" ] || echo "idx,wid,arm,stamp,started,finished,status" > "$MAN"

# ---- 工况 env(两臂共用)----
w_env() {
  case "$1" in
    W1) echo "GRIP_PAYLOAD_KG=0.15 GRIP_ECC_Y=0.0  GRIP_DYNAMIC=false GRIP_DROP_AFTER=15.0 GRIP_DROP_AT_TIP=false" ;;
    W2) echo "GRIP_PAYLOAD_KG=0.30 GRIP_ECC_Y=0.0  GRIP_DYNAMIC=false GRIP_DROP_AFTER=15.0 GRIP_DROP_AT_TIP=false" ;;
    W3) echo "GRIP_PAYLOAD_KG=0.15 GRIP_ECC_Y=0.10 GRIP_DYNAMIC=true GRIP_DROP_AFTER=55.0 GRIP_DROP_AT_TIP=true GRIP_DYN_R=10.0 GRIP_DYN_W=0.283 GRIP_DYN_RAMP=9.36 GRIP_DYN_DZ=0.8 GRIP_Z_HIGH=6.0 GRIP_LIFT_DUR=8.4" ;;
    W4) echo "GRIP_PAYLOAD_KG=0.30 GRIP_ECC_Y=0.10 GRIP_DYNAMIC=true GRIP_DROP_AFTER=55.0 GRIP_DROP_AT_TIP=true GRIP_DYN_R=10.0 GRIP_DYN_W=0.283 GRIP_DYN_RAMP=9.36 GRIP_DYN_DZ=0.8 GRIP_Z_HIGH=6.0 GRIP_LIFT_DUR=8.4" ;;
    W5) echo "GRIP_PAYLOAD_KG=0.15 GRIP_ECC_Y=0.10 GRIP_DYNAMIC=true GRIP_DROP_AFTER=30.0 GRIP_DROP_AT_TIP=true GRIP_DYN_R=10.0 GRIP_DYN_W=0.1415 GRIP_DYN_RAMP=9.36 GRIP_DYN_DZ=0.8 GRIP_Z_HIGH=6.0 GRIP_LIFT_DUR=8.4" ;;
    W6) echo "GRIP_PAYLOAD_KG=0.15 GRIP_ECC_Y=0.0  GRIP_DYNAMIC=false GRIP_DROP_AFTER=0.0  GRIP_DROP_AT_TIP=false" ;;
  esac
}
w_truth() { case "$1" in W2|W4) echo 0.30 ;; *) echo 0.15 ;; esac; }

# ---- 臂 env。W6 里 payload_lost 看门狗归 A 臂(它属于"事件通知"这一族)----
arm_env() {
  local arm="$1" wid="$2" watch=0
  [ "$wid" = "W6" ] && [ "$arm" = "A" ] && watch=1
  if [ "$arm" = "A" ]; then
    echo "CONTINUOUS_PAYLOAD_ESTIMATES=false NMPC_GEOM_SOURCE=online \
NMPC_GEOM_RELEASE_MODE=event MHE_PAYLOAD_LOST_WATCH=$watch"
  else
    echo "CONTINUOUS_PAYLOAD_ESTIMATES=true MHE_PAYLOAD_LOST_WATCH=0"
  fi
}

# ---- 两臂逐字相同的公共 env ----
COMMON_ENV="USE_MHE=true MHE_C_XY_EST=true DJ_TRACK_MEST=true DJ_RATCHET=false \
GRIP_DJ_FLOOR_MP=0.05 GRIP_PAYLOAD_ENVELOPE=0.3 \
NMPC_OMEGA_SCALE=true OMEGA_SCALE_SRC=djest \
DROP_PUBLISH_MASS_EVENT=false MHE_SIGNAL_MODE=residual \
MHE_GEOM_RELEASE_MODE=self \
MHE_CXY_MASS_RELEASE_MP=0.03 MHE_CXY_MASS_RELEASE_PERSIST=20 \
MHE_CXY_MASS_ARM_RATIO=3.0 MHE_CXY_MASS_ARM_PERSIST=20 \
ATTACH_WINDOW_SEC=140.0"

run_one() {
  local wid="$1" arm="$2" idx="$3"
  local t0; t0=$(date +%FT%T)
  echo "[批次] === #$idx  $wid  arm=$arm  $(date +%T) ==="
  cleanup
  ls "$RES"/grip_nmpc_*.log 2>/dev/null | xargs -r -n1 basename > /tmp/.mainline_before

  # ⚠️ 9>&- 关掉继承的 flock fd
  setsid nohup env $COMMON_ENV $(w_env "$wid") $(arm_env "$arm" "$wid") \
    EVAL_TRUE_PAYLOAD_MASS=$(w_truth "$wid") \
    bash "$WS/src/scripts/gripper/run_gripper_headless.sh" \
    > /tmp/mainline_cur.log 2>&1 9>&- &

  # 1) 等本轮新建的 NMPC 日志(只认快照里没有的新文件名)
  local N="" i
  for i in $(seq 1 72); do
    N=$(for f in "$RES"/grip_nmpc_*.log; do
          [ -e "$f" ] || continue
          grep -qxF "$(basename "$f")" /tmp/.mainline_before || echo "$f"
        done | sort | tail -1)
    [ -n "$N" ] && break
    sleep 5
  done
  if [ -z "$N" ]; then
    echo "[批次] !!! #$idx 六分钟没等到新日志,跳过"
    echo "$idx,$wid,$arm,NA,$t0,$(date +%FT%T),no-log" >> "$MAN"
    return
  fi
  local stamp; stamp=$(basename "$N" .log); stamp=${stamp#grip_nmpc_}
  echo "[批次] #$idx stamp=$stamp"

  local status="timeout"
  if [ "$wid" = "W6" ]; then
    # 意外脱落:等 LIFT 起来、飞稳,再从**外部**发 enable=false。
    # ⚠️ 不能用 /gripper/release(只 detach 不关重吸,77ms 后 box 又被吸上,
    #    推力上看不到阶跃);enable=false 才同时 detach + 关重吸,而 nmpc_node
    #    只发布不订阅这个话题,所以它确实不知情。
    for i in $(seq 1 60); do
      grep -q "LIFT: raising hover" "$N" 2>/dev/null && break
      sleep 5
    done
    if grep -q "LIFT: raising hover" "$N" 2>/dev/null; then
      sleep "$W6_REL_AFTER"
      # ⚠️ set -u 下 source ROS setup.bash 会当场静默退出脚本
      set +u
      source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
      source "$WS/install/setup.bash" >/dev/null 2>&1
      set -u
      echo "[批次] #$idx >>> /gripper/enable=false (NMPC 不知情)"
      ros2 topic pub --once /gripper/enable std_msgs/msg/Bool "{data: false}" >/dev/null 2>&1
      echo "UNPLANNED_RELEASE_INJECTED $(date +%FT%T)" >> "$N"
      sleep "$W6_OBS"
      status="ok"
    else
      status="no-lift"
    fi
  else
    # 计划 drop:B 臂打 "DROP complete"(confidence 确认),A 臂打 "DROP: released"
    for i in $(seq 1 72); do
      grep -qE "DROP complete|DROP: released" "$N" 2>/dev/null && { status="ok"; break; }
      sleep 5
    done
    [ "$status" = "ok" ] && sleep "$POST_SEC"
  fi

  echo "$idx,$wid,$arm,$stamp,$t0,$(date +%FT%T),$status" >> "$MAN"
  echo "[批次] #$idx 结束: $status"
  cleanup
}

WIDS="W1 W2 W3 W4 W5 W6"
[ -n "$ONLY_W" ] && WIDS="$ONLY_W"

idx=0
for wid in $WIDS; do
  for r in $(seq 1 "$PAIRS"); do
    # 对内 ABBA/BAAB 交替,平衡"越跑越热"这类时间趋势
    if [ $((r % 2)) -eq 1 ]; then order="A B"; else order="B A"; fi
    for arm in $order; do
      idx=$((idx + 1))
      run_one "$wid" "$arm" "$idx"
    done
  done
done

cleanup
echo "[批次] 全部完成 $(date +%T) -> $MAN"
