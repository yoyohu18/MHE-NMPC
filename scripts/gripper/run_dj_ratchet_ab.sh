#!/bin/bash
# dJ 棘轮 on/off 配对批次(2026-09-01)。
#
# 背景见记忆 mass-domain-payload-release:n=1 时 DJ_RATCHET=false 把 dJ 稳态从
# +223% 拉到 +4.9%,且棘轮本来要治的 LIFT 段冻结没有回来(地板 0.05 兜住了)。
# 这个脚本把那个 n=1 结论补成配对重复。
#
# 固定配置(两臂完全一致,唯一差别 = DJ_RATCHET):
#   0.15kg / ecc 0.10 / 4m/s 工作点 r10 w0.283 z6
#   geom_release_mode=self(MHE)+ event(NMPC)  —— 估计器不被告知 drop
#   残差路径**全堵死**(CONFIRM_ALPHA=1.5→4.41N、RESID_STEP=0),
#   所以载荷释放只能由质量域判据完成 = 该判据也一并被重复检验
#
# 顺序 ABBA/BAAB 交替,平衡"越跑越热"这类时间趋势。
# 用法: [REPS=4] bash src/scripts/gripper/run_dj_ratchet_ab.sh
set -u

WS=/home/clear/ros2_ws_HJH
RES=$WS/nmpc_test_results
MAN=$RES/dj_ratchet_ab_manifest.csv
LOCK=/tmp/dj_ratchet_ab.lock
REPS=${REPS:-4}

# 批次锁。⚠️ fd 9 必须在子进程里关掉(9>&-),否则孤儿子进程会一直持锁,
# ps 里还看不见持锁者(见记忆 sitl-batch-pitfalls)。
exec 9>"$LOCK"
flock -n 9 || { echo "另一个批次正持锁,退出"; exit 1; }

cleanup() {
  pkill -9 -f 'run_gripper_headless.sh'
  pkill -9 -f 'px4_sitl_default/bin/px4'
  pkill -9 -f 'make px4_sitl'
  pkill -9 -f 'gz_bridge'
  pkill -9 -f 'gz sim'
  pkill -9 -f 'offboard_test_acados/acados_nmpc_node'
  pkill -9 -f 'offboard_test_acados/mhe_node'
  pkill -9 -f 'offboard_test_acados/proximity_gripper_node'
  pkill -9 -f 'MicroXRCEAgent'
  pkill -9 -f 'gcs_heartbeat'
  sleep 8
}
trap 'echo "[批次] 收到中断,清栈退出"; cleanup; exit 130' INT TERM

[ -f "$MAN" ] || echo "idx,arm,stamp,started,note" > "$MAN"

run_one() {
  local arm="$1" idx="$2"
  echo "[批次] === #$idx  DJ_RATCHET=$arm  $(date +%T) ==="
  cleanup
  ls "$RES"/grip_nmpc_*.log 2>/dev/null | xargs -r -n1 basename > /tmp/.dj_ab_before

  setsid nohup env \
    GRIP_PAYLOAD_KG=0.15 GRIP_PAYLOAD_ENVELOPE=0.3 GRIP_ECC_Y=0.10 \
    GRIP_DYN_R=10.0 GRIP_DYN_W=0.283 GRIP_DYN_RAMP=9.36 GRIP_DYN_DZ=0.8 \
    GRIP_Z_HIGH=6.0 GRIP_LIFT_DUR=8.4 \
    GRIP_DROP_AFTER=55.0 GRIP_DROP_AT_TIP=true GRIP_DYNAMIC=true \
    DJ_TRACK_MEST=true GRIP_DJ_FLOOR_MP=0.05 MHE_C_XY_EST=true \
    DJ_RATCHET="$arm" \
    DROP_PUBLISH_MASS_EVENT=false MHE_SIGNAL_MODE=residual \
    MHE_CONFIRM_ALPHA=1.5 MHE_RESID_STEP=0 \
    MHE_GEOM_COUPLED=1 MHE_GEOM_RELEASE_MODE=self NMPC_GEOM_RELEASE_MODE=event \
    MHE_CXY_MASS_RELEASE_MP=0.03 MHE_CXY_MASS_RELEASE_PERSIST=20 \
    MHE_CXY_MASS_ARM_RATIO=3.0 MHE_CXY_MASS_ARM_PERSIST=20 \
    bash "$WS/src/scripts/gripper/run_gripper_headless.sh" \
    > /tmp/dj_ab_cur.log 2>&1 9>&- &

  # 1) 等本轮新建的 NMPC 日志(只认快照里没有的新文件名)
  local N="" i
  for i in $(seq 1 72); do
    N=$(for f in "$RES"/grip_nmpc_*.log; do
          [ -e "$f" ] || continue
          grep -qxF "$(basename "$f")" /tmp/.dj_ab_before || echo "$f"
        done | sort | tail -1)
    [ -n "$N" ] && break
    sleep 5
  done
  if [ -z "$N" ]; then
    echo "[批次] !!! #$idx 六分钟没等到新日志,跳过"
    echo "$idx,$arm,NA,$(date +%FT%T),no-log" >> "$MAN"
    return
  fi
  local stamp; stamp=$(basename "$N" .log); stamp=${stamp#grip_nmpc_}
  echo "[批次] #$idx stamp=$stamp,等 DROP ..."

  # 2) 等 drop(headless 不会自己停,必须我们盯着)
  local ok=0
  for i in $(seq 1 60); do
    # ⚠️ 2026-09-04:连续主线(continuous_payload_estimates=true)下 NMPC 打的是
    #    "DROP complete"(no-payload confidence 确认后),legacy 事件档才打
    #    "DROP: released"。只 grep 旧串会把**成功的轮次**静默记成 no-drop
    #    (实测 20260904_234903:DROP complete 了,批次仍等满 5 分钟判失败)。
    #    两种都认,legacy 批次行为不变。
    grep -qE "DROP complete|DROP: released" "$N" 2>/dev/null && { ok=1; break; }
    sleep 5
  done
  if [ $ok -eq 1 ]; then
    sleep 45          # 留够 drop 后的 m_est 回归 + c_xy 释放窗口
    echo "$idx,$arm,$stamp,$(date +%FT%T),ok" >> "$MAN"
    echo "[批次] #$idx 完成"
  else
    echo "$idx,$arm,$stamp,$(date +%FT%T),no-drop" >> "$MAN"
    echo "[批次] !!! #$idx 五分钟没等到 DROP"
  fi
  cleanup
}

idx=0
for r in $(seq 1 "$REPS"); do
  # ABBA / BAAB 交替
  if [ $((r % 2)) -eq 1 ]; then order="true false"; else order="false true"; fi
  for arm in $order; do
    idx=$((idx + 1))
    run_one "$arm" "$idx"
  done
done

cleanup
echo "[批次] 全部完成 $(date +%T) -> $MAN"
