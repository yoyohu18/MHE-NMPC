#!/bin/bash
# 质量域卸载判据的重复批次(2026-09-01)。单臂,全部跑**主线档**配置。
#
# 目的:补样本量,量化武装门控修复后的误触发率。
#   修复前(无武装持续要求):8 轮里 1 轮在 drop-71.5s 误释放 = 12.5%
#   修复后:已有 6 轮 0/6,但若真实率仍是 12.5%,6 轮全对的概率也有 45% —— 排除不掉
# ⚠️ 纯计数的功效有限:即便累计 16 轮 0 次,rule of three 的 95% 上界也只到 ~19%。
#    所以聚合脚本同时统计**武装时刻**这个机制指标 —— 修复前后两组零重叠,功效高得多。
#
# 配置 = 演示片那一档(记忆 mass-domain-payload-release):
#   0.15kg / 包线 0.3 / ecc 0.10 / 4m/s 工作点
#   MHE geom_release_mode=self + NMPC event;残差路径全堵死;DJ_RATCHET=false
#   → 载荷释放只可能由质量域判据完成
#
# 用法: [REPS=10] bash src/scripts/gripper/run_cxy_mass_repeat.sh
set -u

WS=/home/clear/ros2_ws_HJH
RES=$WS/nmpc_test_results
MAN=$RES/cxy_mass_repeat_manifest.csv
LOCK=/tmp/cxy_mass_repeat.lock
REPS=${REPS:-10}

# 批次锁。fd 9 必须在子进程里关掉(9>&-),否则孤儿子进程一直持锁且 ps 里看不见。
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
trap 'echo "[批次] 中断,清栈退出"; cleanup; exit 130' INT TERM

[ -f "$MAN" ] || echo "idx,stamp,started,note" > "$MAN"

run_one() {
  local idx="$1"
  echo "[批次] === #$idx  $(date +%T) ==="
  cleanup
  ls "$RES"/grip_nmpc_*.log 2>/dev/null | xargs -r -n1 basename > /tmp/.cxy_rep_before

  setsid nohup env \
    GRIP_PAYLOAD_KG=0.15 GRIP_PAYLOAD_ENVELOPE=0.3 GRIP_ECC_Y=0.10 \
    GRIP_DYN_R=10.0 GRIP_DYN_W=0.283 GRIP_DYN_RAMP=9.36 GRIP_DYN_DZ=0.8 \
    GRIP_Z_HIGH=6.0 GRIP_LIFT_DUR=8.4 \
    GRIP_DROP_AFTER=55.0 GRIP_DROP_AT_TIP=true GRIP_DYNAMIC=true \
    DJ_TRACK_MEST=true GRIP_DJ_FLOOR_MP=0.05 MHE_C_XY_EST=true \
    DJ_RATCHET=false \
    DROP_PUBLISH_MASS_EVENT=false MHE_SIGNAL_MODE=residual \
    MHE_CONFIRM_ALPHA=1.5 MHE_RESID_STEP=0 \
    MHE_GEOM_COUPLED=1 MHE_GEOM_RELEASE_MODE=self NMPC_GEOM_RELEASE_MODE=event \
    MHE_CXY_MASS_RELEASE_MP=0.03 MHE_CXY_MASS_RELEASE_PERSIST=20 \
    MHE_CXY_MASS_ARM_RATIO=3.0 MHE_CXY_MASS_ARM_PERSIST=20 \
    bash "$WS/src/scripts/gripper/run_gripper_headless.sh" \
    > /tmp/cxy_rep_cur.log 2>&1 9>&- &

  local N="" i
  for i in $(seq 1 72); do
    N=$(for f in "$RES"/grip_nmpc_*.log; do
          [ -e "$f" ] || continue
          grep -qxF "$(basename "$f")" /tmp/.cxy_rep_before || echo "$f"
        done | sort | tail -1)
    [ -n "$N" ] && break
    sleep 5
  done
  if [ -z "$N" ]; then
    echo "[批次] !!! #$idx 六分钟没等到新日志,跳过"
    echo "$idx,NA,$(date +%FT%T),no-log" >> "$MAN"; return
  fi
  local stamp; stamp=$(basename "$N" .log); stamp=${stamp#grip_nmpc_}
  echo "[批次] #$idx stamp=$stamp,等 DROP ..."

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
    sleep 45
    echo "$idx,$stamp,$(date +%FT%T),ok" >> "$MAN"
    echo "[批次] #$idx 完成"
  else
    echo "$idx,$stamp,$(date +%FT%T),no-drop" >> "$MAN"
    echo "[批次] !!! #$idx 五分钟没等到 DROP"
  fi
  cleanup
}

for r in $(seq 1 "$REPS"); do run_one "$r"; done

cleanup
echo "[批次] 全部完成 $(date +%T) -> $MAN"
