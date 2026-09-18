#!/usr/bin/env bash
set -euo pipefail

OUT="${1:-$(dirname "$0")/algorithm-inputs.mp4}"
FONT="/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"

ffmpeg -y -hide_banner -loglevel warning \
  -f lavfi -i "color=c=0x07111f:s=1920x1080:r=30:d=12" \
  -vf "
drawbox=x=0:y=0:w=iw:h=12:color=0x31d6c4:t=fill,
drawtext=fontfile=${FONT}:text='算法输入 · SENSOR-ONLY DATA FLOW':x=90:y=65:fontsize=44:fontcolor=white,
drawtext=fontfile=${FONT}:text='工作空间真实接口':x=92:y=124:fontsize=25:fontcolor=0x8aa4bd,

drawbox=x=90:y=250:w=350:h=235:color=0x10253b:t=fill,
drawbox=x=90:y=250:w=350:h=235:color=0x2f6f91:t=3,
drawtext=fontfile=${FONT}:text='PX4 / Gazebo':x=145:y=280:fontsize=34:fontcolor=0x7edcf2,
drawtext=fontfile=${FONT}:text='里程计  x(13)':x=145:y=350:fontsize=32:fontcolor=white,
drawtext=fontfile=${FONT}:text='p · v · q · ω':x=145:y=400:fontsize=26:fontcolor=0xb8cadb,
drawtext=fontfile=${FONT}:text='四电机转速  omega1...omega4':x=145:y=447:fontsize=28:fontcolor=white,

drawbox=x=655:y=245:w=430:h=245:color=0x12342f:t=fill,
drawbox=x=655:y=245:w=430:h=245:color=0x31d6c4:t=4,
drawtext=fontfile=${FONT}:text='MHE · 10 Hz':x=745:y=278:fontsize=38:fontcolor=0x62f1dc,
drawtext=fontfile=${FONT}:text='T = kf sum(omega_i^2)':x=700:y=350:fontsize=31:fontcolor=white,
drawtext=fontfile=${FONT}:text='tau = rotor geometry × F_i':x=700:y=402:fontsize=27:fontcolor=0xb8cadb,
drawtext=fontfile=${FONT}:text='2.0 s moving window':x=716:y=449:fontsize=26:fontcolor=0xb8cadb,

drawbox=x=1305:y=210:w=500:h=315:color=0x29213b:t=fill,
drawbox=x=1305:y=210:w=500:h=315:color=0xa78bfa:t=4,
drawtext=fontfile=${FONT}:text='NMPC · 20 Hz':x=1405:y=245:fontsize=38:fontcolor=0xc4b5fd,
drawtext=fontfile=${FONT}:text='当前状态  x(13)':x=1370:y=315:fontsize=29:fontcolor=white,
drawtext=fontfile=${FONT}:text='未来参考  xref(13)':x=1370:y=365:fontsize=29:fontcolor=white,
drawtext=fontfile=${FONT}:text='载荷模型  m · ΔJ · cxy':x=1370:y=415:fontsize=29:fontcolor=white,
drawtext=fontfile=${FONT}:text='约束内预测未来 1.0 s':x=1370:y=468:fontsize=25:fontcolor=0xb8cadb,

drawbox=x=470+min(155\,max(0\,(t-1.0)*155)):y=352:w=18:h=18:color=0x62f1dc:t=fill:enable='between(t,1,2)',
drawbox=x=1115+min(160\,max(0\,(t-5.0)*160)):y=352:w=18:h=18:color=0xc4b5fd:t=fill:enable='between(t,5,6)',
drawbox=x=1115+min(160\,max(0\,(t-7.0)*160)):y=412:w=18:h=18:color=0xfbbf24:t=fill:enable='between(t,7,8)',

drawtext=fontfile=${FONT}:text='传感器观测':x=482:y=305:fontsize=24:fontcolor=0x62f1dc,
drawtext=fontfile=${FONT}:text='原子载荷帧':x=1120:y=300:fontsize=24:fontcolor=0xc4b5fd,
drawtext=fontfile=${FONT}:text='{ m_est, deltaJ_est, c_xy, health, age }':x=1055:y=548:fontsize=27:fontcolor=0xc4b5fd,

drawbox=x=90:y=665:w=1715:h=245:color=0x0c1b2b:t=fill,
drawbox=x=90:y=665:w=1715:h=245:color=0x29425a:t=2,
drawtext=fontfile=${FONT}:text='算法最终求解变量':x=135:y=700:fontsize=27:fontcolor=0x8aa4bd,
drawtext=fontfile=${FONT}:text='u* = [ 总推力 T, 机体系力矩 τx, τy, τz ]':x=135:y=755:fontsize=38:fontcolor=white,
drawtext=fontfile=${FONT}:text='只执行预测序列第一步 → MAVROS / PX4 速率内环':x=135:y=820:fontsize=31:fontcolor=0x62f1dc,
drawtext=fontfile=${FONT}:text='attach / drop notification  不进入主算法':x=1180:y=930:fontsize=24:fontcolor=0xff7b72,
drawbox=x=1148:y=925:w=22:h=22:color=0xff7b72:t=3,

drawtext=fontfile=${FONT}:text='01  OBSERVE':x=110:y=975:fontsize=22:fontcolor=0x62f1dc:alpha='if(lt(t,3),1,0.25)',
drawtext=fontfile=${FONT}:text='02  ESTIMATE':x=480:y=975:fontsize=22:fontcolor=0x62f1dc:alpha='if(between(t,3,6),1,0.25)',
drawtext=fontfile=${FONT}:text='03  ADAPT':x=865:y=975:fontsize=22:fontcolor=0xc4b5fd:alpha='if(between(t,6,9),1,0.25)',
drawtext=fontfile=${FONT}:text='04  CONTROL':x=1220:y=975:fontsize=22:fontcolor=0xfbbf24:alpha='if(gte(t,9),1,0.25)',
drawbox=x=110:y=1017:w='if(lt(t,3),280,if(lt(t,6),650,if(lt(t,9),1010,1450)))':h=5:color=0x31d6c4:t=fill
" \
  -c:v libx264 -preset medium -crf 18 -pix_fmt yuv420p -movflags +faststart "$OUT"

printf '%s\n' "$OUT"
