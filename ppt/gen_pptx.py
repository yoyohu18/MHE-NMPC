# -*- coding: utf-8 -*-
"""生成《MHE-NMPC 无人机变负载自适应控制》汇报 PPT。
所有数字与图片均取自 paper/main.pdf(8页, Fig.1-4 + Table I-V)与仓库源码。
"""
import os
from pptx import Presentation
from pptx.util import Inches, Pt, Emu
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.enum.shapes import MSO_SHAPE
from pptx.oxml.ns import qn
from lxml import etree

FIGS = "/home/clear/ros2_ws_HJH/src/paper/figs"
OUT  = "/home/clear/ros2_ws_HJH/src/ppt/MHE-NMPC汇报.pptx"

# ---- palette (与网页版 deck 同源) ----
INK   = RGBColor(0x10, 0x1C, 0x24)
SOFT  = RGBColor(0x5C, 0x6B, 0x77)
FAINT = RGBColor(0x8B, 0x9A, 0xA4)
ACC   = RGBColor(0x0A, 0x6E, 0x70)
SIG   = RGBColor(0xA6, 0x6A, 0x00)
CRIT  = RGBColor(0xB3, 0x32, 0x1F)
GRND  = RGBColor(0xED, 0xF0, 0xF2)
SURF  = RGBColor(0xFA, 0xFB, 0xFC)
RULE  = RGBColor(0xD2, 0xD9, 0xDD)
WHITE = RGBColor(0xFF, 0xFF, 0xFF)

CN = "微软雅黑"
MONO = "Consolas"

SW, SH = 13.333, 7.5
CL, CR = 1.95, 12.85
CW = CR - CL

prs = Presentation()
prs.slide_width = Inches(SW)
prs.slide_height = Inches(SH)
BLANK = prs.slide_layouts[6]


# ---------------- low-level helpers ----------------
def _font(run, name=CN, size=11, color=INK, bold=False, italic=False, spc=None):
    f = run.font
    f.name = name
    f.size = Pt(size)
    f.bold = bold
    f.italic = italic
    f.color.rgb = color
    rPr = run._r.get_or_add_rPr()
    for tag in ("a:ea", "a:cs"):
        el = rPr.find(qn(tag))
        if el is None:
            el = etree.SubElement(rPr, qn(tag))
        el.set("typeface", CN if name == CN else name)
    if spc:
        rPr.set("spc", str(int(spc * 100)))


def parse_rich(text):
    """**bold**  ##accent##  !!crit!!  @@signal@@  `mono`"""
    out, buf = [], ""
    i, n = 0, len(text)
    marks = {"**": ("b", 2), "##": ("a", 2), "!!": ("c", 2), "@@": ("s", 2), "`": ("m", 1)}
    while i < n:
        hit = None
        for m, (kind, ln) in marks.items():
            if text[i:i + ln] == m:
                hit = (m, kind, ln)
                break
        if hit:
            m, kind, ln = hit
            end = text.find(m, i + ln)
            if end == -1:
                buf += text[i]
                i += 1
                continue
            if buf:
                out.append((buf, None))
                buf = ""
            out.append((text[i + ln:end], kind))
            i = end + ln
        else:
            buf += text[i]
            i += 1
    if buf:
        out.append((buf, None))
    return out


def para(tf, text, size=11, color=SOFT, bold=False, line=1.35, space_after=4,
         align=PP_ALIGN.LEFT, first=False, name=CN):
    p = tf.paragraphs[0] if first else tf.add_paragraph()
    p.line_spacing = line
    p.space_after = Pt(space_after)
    p.alignment = align
    for seg, kind in parse_rich(text):
        r = p.add_run()
        r.text = seg
        if kind == "b":
            _font(r, name, size, INK, True)
        elif kind == "a":
            _font(r, MONO, size, ACC, True)
        elif kind == "c":
            _font(r, name, size, CRIT, True)
        elif kind == "s":
            _font(r, name, size, SIG, True)
        elif kind == "m":
            _font(r, MONO, size * 0.95, ACC, False)
        else:
            _font(r, name, size, color, bold)
    return p


def txbox(slide, x, y, w, h, lines, size=11, color=SOFT, bold=False, line=1.35,
          space_after=4, align=PP_ALIGN.LEFT, anchor=MSO_ANCHOR.TOP, name=CN):
    tb = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    tf = tb.text_frame
    tf.word_wrap = True
    tf.vertical_anchor = anchor
    tf.margin_left = tf.margin_right = Inches(0.04)
    tf.margin_top = tf.margin_bottom = 0
    if isinstance(lines, str):
        lines = [lines]
    for i, ln in enumerate(lines):
        para(tf, ln, size, color, bold, line, space_after, align, first=(i == 0), name=name)
    return tb


def rect(slide, x, y, w, h, fill=GRND, line=None, lw=0.75):
    sh = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(x), Inches(y), Inches(w), Inches(h))
    sh.shadow.inherit = False
    if fill is None:
        sh.fill.background()
    else:
        sh.fill.solid()
        sh.fill.fore_color.rgb = fill
    if line is None:
        sh.line.fill.background()
    else:
        sh.line.color.rgb = line
        sh.line.width = Pt(lw)
    sh.text_frame.text = ""
    return sh


# ---------------- page furniture ----------------
def add_slide():
    s = prs.slides.add_slide(BLANK)
    bg = rect(s, 0, 0, SW, SH, SURF)
    bg.shadow.inherit = False
    return s


def rail(slide, num, sec):
    txbox(slide, 0.30, 0.50, 1.25, 0.80, ["%02d" % num], 30, ACC, True,
          align=PP_ALIGN.RIGHT, name=MONO)
    txbox(slide, 0.10, 1.16, 1.45, 0.3, [sec], 9.5, FAINT, align=PP_ALIGN.RIGHT)
    rect(slide, 1.68, 0.52, 0.008, 6.1, RULE)


def head(slide, eyebrow, title, dek=None):
    txbox(slide, CL, 0.56, CW, 0.3, [eyebrow], 9.5, ACC, True, name=MONO)
    txbox(slide, CL, 0.88, CW, 0.85, [title], 25, INK, True, line=1.15)
    if dek:
        txbox(slide, CL, 1.78, CW * 0.86, 0.75, [dek], 11.5, SOFT, line=1.5)
        return 2.72
    return 1.92


def pagenum(slide, i, total=16):
    rect(slide, CL, 7.03, CW, 0.008, RULE)
    txbox(slide, CL, 7.10, CW, 0.28, ["%02d / %02d" % (i, total)], 9, FAINT,
          align=PP_ALIGN.RIGHT, name=MONO)


def card(slide, x, y, w, h, label=None, title=None, body=None, bar=ACC, fill=GRND):
    rect(slide, x, y, w, h, fill)
    if bar:
        rect(slide, x, y, 0.035, h, bar)
    cy = y + 0.16
    if label:
        txbox(slide, x + 0.20, cy, w - 0.38, 0.24, [label], 8.5, FAINT, name=MONO)
        cy += 0.27
    if title:
        txbox(slide, x + 0.20, cy, w - 0.38, 0.34, [title], 13, INK, True, line=1.25)
        cy += 0.40
    if body:
        txbox(slide, x + 0.20, cy, w - 0.38, y + h - cy - 0.12,
              body if isinstance(body, list) else [body], 10, SOFT, line=1.45)


def callout(slide, x, y, w, h, lines, kind="sig"):
    col = {"sig": SIG, "crit": CRIT, "acc": ACC}[kind]
    fill = {"sig": RGBColor(0xF7, 0xF1, 0xE6), "crit": RGBColor(0xF9, 0xEC, 0xE9),
            "acc": RGBColor(0xE8, 0xF1, 0xF1)}[kind]
    rect(slide, x, y, w, h, fill)
    rect(slide, x, y, 0.035, h, col)
    txbox(slide, x + 0.20, y + 0.13, w - 0.38, h - 0.24,
          lines if isinstance(lines, list) else [lines], 10, SOFT, line=1.45,
          anchor=MSO_ANCHOR.MIDDLE)


def eqbox(slide, x, y, w, h, lines, size=11.5, align=PP_ALIGN.LEFT):
    rect(slide, x, y, w, h, GRND, RULE)
    txbox(slide, x + 0.22, y + 0.10, w - 0.44, h - 0.20, lines, size, INK,
          line=1.55, align=align, anchor=MSO_ANCHOR.MIDDLE, name=MONO)


def picture(slide, fname, x, y, w):
    p = os.path.join(FIGS, fname)
    return slide.shapes.add_picture(p, Inches(x), Inches(y), width=Inches(w))


def table(slide, x, y, w, headers, rows, col_w, row_h=0.34, head_h=0.32,
          size=9.5, head_size=8.5, hi_last=False):
    nr, nc = len(rows) + 1, len(headers)
    shp = slide.shapes.add_table(nr, nc, Inches(x), Inches(y), Inches(w),
                                 Inches(head_h + row_h * len(rows)))
    t = shp.table
    t.first_row = False
    t.horz_banding = False
    tblPr = t._tbl.find(qn('a:tblPr'))
    for a in ("firstRow", "bandRow", "firstCol"):
        tblPr.set(a, "0")
    # kill the default table style
    st = tblPr.find(qn('a:tableStyleId'))
    if st is not None:
        tblPr.remove(st)
    for i, cw in enumerate(col_w):
        t.columns[i].width = Inches(cw)
    t.rows[0].height = Inches(head_h)
    for i in range(1, nr):
        t.rows[i].height = Inches(row_h)

    def setcell(c, text, sz, color, bold, name, fill, align, top_line=False):
        c.fill.solid()
        c.fill.fore_color.rgb = fill
        c.margin_left = Inches(0.09)
        c.margin_right = Inches(0.06)
        c.margin_top = Inches(0.03)
        c.margin_bottom = Inches(0.03)
        c.vertical_anchor = MSO_ANCHOR.MIDDLE
        tf = c.text_frame
        tf.word_wrap = True
        p = tf.paragraphs[0]
        p.alignment = align
        p.line_spacing = 1.2
        for seg, kind in parse_rich(text):
            r = p.add_run()
            r.text = seg
            if kind == "b":
                _font(r, name, sz, INK, True)
            elif kind == "a":
                _font(r, MONO, sz, ACC, True)
            elif kind == "c":
                _font(r, name, sz, CRIT, True)
            elif kind == "s":
                _font(r, name, sz, SIG, True)
            else:
                _font(r, name, sz, color, bold)

    for j, htxt in enumerate(headers):
        setcell(t.cell(0, j), htxt, head_size, FAINT, False, MONO, SURF, PP_ALIGN.LEFT)
    for i, row in enumerate(rows):
        last = hi_last and (i == len(rows) - 1)
        fill = GRND if last else SURF
        for j, cell in enumerate(row):
            mono = (j > 0)
            setcell(t.cell(i + 1, j), cell, size, INK if last else SOFT, last,
                    MONO if mono else CN, fill, PP_ALIGN.LEFT)
    return shp


# ================= SLIDES =================

# ---- 01 cover ----
s = add_slide()
rect(s, 0, 0, SW, SH, SURF)
rect(s, 6.95, 0, 0.008, SH, RULE)
rect(s, 6.96, 0, SW - 6.96, SH, GRND)
rect(s, 0.95, 2.28, 0.9, 0.032, ACC)
txbox(s, 0.92, 2.55, 5.8, 1.0, ["MHE–NMPC"], 50, INK, True, line=1.0, name=MONO)
txbox(s, 0.95, 3.62, 5.3, 1.1,
      ["无人机变负载自适应控制:不依赖任何外部事件信号,"
       "让控制器自己判断包裹是否已经离开"], 15, INK, True, line=1.55)
txbox(s, 0.95, 4.92, 5.5, 1.3,
      ["ROS 2 Jazzy · PX4 SITL · Gazebo Harmonic · acados",
       "论文投稿中(ICRA 2027 目标) · PX4 SITL 验证阶段",
       "汇报人: ____   日期: 2026-09-16"],
      10, FAINT, line=1.75, space_after=2, name=MONO)
picture(s, "fig_moment_truth.png", 7.35, 2.05, 5.45)
txbox(s, 7.35, 4.70, 5.45, 1.5,
      ["**论文 Fig. 3** — 一阶质量矩估计 ŝ_y 与真值之比(0.15 kg / 2 m/s, n=11)。",
       "抓取时自己涨起来、8 字机动中保持、释放后归零——"
       "**全程没有任何 attach/drop 通知进入估计器**,真值只用于离线比对。"],
      9.5, SOFT, line=1.5, space_after=5)

# ---- 02 problem ----
s = add_slide()
rail(s, 2, "问题")
y = head(s, "MOTIVATION", "抓放包裹的那一刻,控制器手里的模型就错了",
         "NMPC 的预测能力建立在一个假设上:它知道自己有多重、重心在哪、转起来多沉。"
         "抓取或投放瞬间,这三样同时阶跃变化。")
cw3 = (CW - 0.6) / 3
card(s, CL, y, cw3, 2.05, "通道一 / 推力(外环)", "悬停油门算错",
     ["总质量 m 改变推力—加速度映射。m 一偏,前馈推力系统性偏;"
      "按 mg 设的推力裕度同时失真。"], CRIT)
card(s, CL + cw3 + 0.3, y, cw3, 2.05, "通道二 / 力矩(内环)", "质心偏移拧出常值力矩",
     ["推力不再过复合质心 → `τ = Δr × f`。因为 f≈mg 很大,"
      "**很小的偏移也会产生显著配平力矩**。"], CRIT)
card(s, CL + 2 * (cw3 + 0.3), y, cw3, 2.05, "通道三 / 惯量(内环)", "姿态响应变慢",
     ["转动惯量增大,同样力矩产生更小角加速度,"
      "等效降低内环带宽。"], CRIT)
callout(s, CL, y + 2.28, CW, 1.28,
        ["**论文的出发点(摘要第一句):** “A release command does not guarantee "
         "that a payload has physically detached.”",
         "投放命令只是**意图**,不是**事实**。夹爪可能没松、物体可能延迟脱离、"
         "命令可能根本没送达。按命令清除负载模型的控制器,"
         "**会把一架还挂着货的飞机当空机来飞**。"], "crit")
pagenum(s, 2)

# ---- 03 architecture ----
s = add_slide()
rail(s, 3, "系统")
y = head(s, "SYSTEM ARCHITECTURE", "整条链路只消费两样东西:电机转速和里程计",
         "论文 Fig. 1。attach/drop 通知在图里被明确画成“NOT CONSUMED”——"
         "这不是省略,是本文的核心约束。")
PW = 9.4
picture(s, "fig1_architecture.png", CL + (CW - PW) / 2, y - 0.22, PW)
yy = y - 0.22 + PW / 2.88 + 0.14
cw2 = (CW - 0.35) / 2
card(s, CL, yy, cw2, 1.12, "两个求解器", None,
     ["**MHE** 10 Hz · 16 状态 · 2.0 s 窗口 → 估 m 与一阶矩 s;"
      "**NMPC** 20 Hz · 13 状态/4 输入 · N=20×0.05 s = 1.0 s 前瞻。"], ACC)
card(s, CL + cw2 + 0.35, yy, cw2, 1.12, "一帧原子消息", None,
     ["`{m, s, ΔJ, γ, h, a}` 同一时间戳发布,附健康位与解年龄。"
      "**拆成多个话题就可能拿到“新质量+旧惯量”这种物理上不存在的组合。**"], ACC)
pagenum(s, 3)

# ---- 04 first moment ----
s = add_slide()
rail(s, 4, "核心贡献 1")
y = head(s, "WHY THE FIRST MOMENT", "估“一阶质量矩”,而不是估“质心偏移”",
         "论文 §IV-B。这是全文最关键的一个建模选择,也是最容易被追问的地方。")
cw2 = (CW - 0.4) / 2
card(s, CL, y, cw2, 2.30, "为什么 c_xy 不行", "包裹消失时,质心偏移本身没有定义",
     ["物理上 `s = m_P·r_xy → 0`,但 `r_xy` 在 `m_P → 0` 时是 **0/0**。",
      "早期只估质量的版本里,优化器要压住一个虚假偏心,"
      "唯一办法是**把总质量压到空机以下**——释放后 55 个样本里有 "
      "!!14 个钉在下界!!。"], CRIT)
card(s, CL + cw2 + 0.4, y, cw2, 2.30, "为什么 s 行", "它让惯性参数线性地进入动力学",
     ["一阶矩是刚体惯性参数的**标准线性参数化**(机械臂负载辨识的经典手法)。",
      "增广 s 直接提供转动通路,并且 `c_xy = s/m` 可以在"
      "**完全不知道抓取几何**的情况下算出来。"], ACC)
callout(s, CL, y + 2.52, CW, 1.15,
        ["**这是“最小结构”,不是“加更多自由度”。** 论文原话:它是我们找到的、"
         "能同时表达**负载是否存在、配平力矩、滚转/俯仰惯量**三件事,"
         "又不需要外部测量抓取几何的最小结构。",
         "惯量增量 ΔJ 不是独立估计的,而是由 m̂ 与一个**弱竖直力臂先验 r_z** 代数生成"
         "——r_z 从推力—力矩通道结构上不可观测。"], "acc")
pagenum(s, 4)

# ---- 05 NMPC ----
s = add_slide()
rail(s, 5, "方法")
y = head(s, "NMPC", "每 50 毫秒重解一次未来一秒",
         "与 PID 的本质差别:它不是对当前误差反应,而是用非线性模型预测未来 1 秒,"
         "在执行器包线内选一串最优输入,只执行第一步。")
table(s, CL, y, 5.55,
      ["项", "取值"],
      [["状态 / 输入", "13 / 4"],
       ["时域", "N=20 × Δt=0.05 s → 1.0 s"],
       ["求解器", "acados SQP-RTI + HPIPM"],
       ["推力包线 U", "T ∈ [1.27, 31.35] N"],
       ["力矩包线", "|τx|,|τy| ≤ 0.5 · |τz| ≤ 0.2 N·m"],
       ["求解耗时", "1.3 ms 中位 / 2.2 ms p99"],
       ["占控制周期", "2.6% / 4.4%  (n=1009)"]],
      [1.85, 3.70], row_h=0.345, size=9.5)
x2 = CL + 5.95
card(s, x2, y, CW - 5.95, 1.88, "代价函数", "Bryson's rule + 活的悬停基准",
     ["容差 0.1 m / 0.3 m/s / 0.05 / 0.3 rad/s / 2 N。",
      "**ū(p) = [mg, c_y·mg, −c_x·mg, 0]**:随估计实时变化的悬停输入,"
      "**含偏心配平力矩**,不是写死的空机重量。"], ACC)
card(s, x2, y + 2.06, CW - 5.95, 1.72, "模型参数向量", "p = [x_ref, m, ΔJ, c_xy]",
     ["负载信息以**运行时参数**进模型,不改代码、不重新生成求解器。",
      "每次求解前 m / s / ΔJ 各自过低通与**速率限制**。"], ACC)
pagenum(s, 5)

# ---- 06 MHE ----
s = add_slide()
rail(s, 6, "方法")
y = head(s, "MHE", "把负载参数当成额外的状态一起估",
         "在最近 2 秒窗口内,找一组状态+参数使模型预测与实测轨迹最一致——"
         "不是只看当前一帧,所以能用时间上的信息压噪声。")
eqbox(s, CL, y, 6.1, 0.95,
      ["x_MHE = [ x(13)   m   s_x   s_y ]  ∈  R^16",
       "窗口内 ṁ = 0, ṡ = 0    N=20 × Δt=0.1 s → 2.0 s @ 10 Hz"], 11)
card(s, CL, y + 1.12, 6.1, 1.95, "权重的意图 · W_w 远大于 W_y", "刻意信任动力学方程",
     ["过程噪声权重 `diag(1e4·I6, 1e5·I4, 1e3·I3)`,比测量噪声权重紧得多。",
      "含义:除了负载参数未知,**模型结构是对的**。",
      "于是优化器只能靠调 m 和 s 去解释观测到的加速度,"
      "不能把误差含糊地推给过程噪声。"], ACC)
x2 = CL + 6.5
card(s, x2, y, CW - 6.5, 1.32, "到达代价刻意“不自信”", None,
     ["W_0 在 m 上 σ≈3.2 kg、s 上 σ=0.1 kg·m,**近乎无信息**"
      "——先验强了,估计就只是先验的回声。"], ACC)
callout(s, x2, y + 1.50, CW - 6.5, 1.57,
        ["!!实时性要如实讲:!! MHE 中位 **9.2 ms**,p90 24.6 ms,"
         "p99 **59.2 ms**,峰值 72.8 ms —— 相对 100 ms 周期。",
         "仿真中满足截止期,但**尾部裕度很窄**(是历史纯质量版 1.4 ms 的约 7 倍)。"
         "所以论文明确**不做机载算力余量的声明**。"], "crit")
pagenum(s, 6)

# ---- 07 thrust proxy ----
s = add_slide()
rail(s, 7, "核心贡献 2")
y = head(s, "ESTIMATE-INDEPENDENT PROXY", "不能把 NMPC 自己发出的推力喂回估计器",
         "MHE 需要“已知输入力”。最顺手的来源是 NMPC 的指令推力——但那会闭出一个"
         "估计器—控制器自洽环。")
cw2 = (CW - 0.4) / 2
card(s, CL, y, cw2, 2.42, "陷阱 · 自洽盲区", "指令推力是从 m̂ 算出来的",
     ["NMPC 的代价把 T 锚在 `m̂·g` 附近。一旦 m̂ 偏离真值,"
      "指令推力就不等于飞机真实受力。",
      "更糟的是:**指令与转子之间的反演误差、饱和、电机滞后,"
      "对估计器全部不可见**。于是质量误差可以自洽地一直存在下去,"
      "而不会表现为残差。"], CRIT)
card(s, CL + cw2 + 0.4, y, cw2, 2.42, "解法 · 取在这些环节的下游", "从电机转速反算物理量",
     ["`T_phys = k·Σωᵢ²`  与  `τ_phys = Σ rᵢ × Fᵢ`",
      "取自真实转速,经过 PX4 与电机的真实非线性映射,**不由 m̂ 计算**。",
      "工程细节:电机消息远快于 MHE 周期,窗口内平均的是**各电机推力 Fᵢ**,"
      "不是先平均 ωᵢ 再平方(平方非线性,顺序反了会引入偏差)。"], ACC)
callout(s, CL, y + 2.64, CW, 1.02,
        ["**它依赖什么,论文说得很清楚:** 依赖标定好的 k。而且 k 在快速平移下"
         "会因诱导入流而失效——这一项我们没有建模。",
         "第 15 页会给出:**k 标定偏 ±5%,整个接口就会两个方向都失效**。"], "sig")
pagenum(s, 7)

# ---- 08 lifecycle ----
s = add_slide()
rail(s, 8, "核心贡献 3")
y = head(s, "RELEASE SEMANTICS", "“我发了投放命令”不是证据",
         "控制器清除负载模型只有一条路:确认(confirmation)——健康、新鲜的帧上,"
         "空载置信度 γ>0.90 持续 1.0 s。")
eqbox(s, CL, y, CW, 1.05,
      ["γ  =  空载分数(质量) × 空载分数(一阶矩) × 空载分数(惯量)",
       "→ 非零的一阶矩可以单独否决一次误判的“空载”,即使质量估计已经贴到下界"], 11)
cw2 = (CW - 0.4) / 2
card(s, CL, y + 1.22, cw2, 1.85, "证据不足时怎么办", "进入 UNRESOLVED,而不是猜",
     ["12 s 超时 → 声明 UNRESOLVED → 3 s 平滑刹停到悬停,"
      "在低动态下重新取证。",
      "**超时被当作“未知”,永远不被当成“空机”的证明。**"], ACC)
card(s, CL + cw2 + 0.4, y + 1.22, cw2, 1.85, "被删掉的一条快路径", "它造成了真实的提前释放",
     ["“残差 + 质量域空载”(不带一阶矩证据)在 4 m/s 的 8 字里,"
      "!!9 次随机飞行中 3 次提前释放!!。",
      "原因:高机动下 m̂ 会整段贴在下界,质量域**长时间显示空载**。"], CRIT)
callout(s, CL, y + 3.28, CW, 0.80,
        ["**一条录到的 8 字暂态残差达到 −1.69 N,比最小负载真实释放的 −1.47 N 还大**"
         " —— 所以只看残差幅值,根本分不开“机动”和“真掉了”。"], "crit")
pagenum(s, 8)

# ---- 09 thresholds ----
s = add_slide()
rail(s, 9, "严谨性")
y = head(s, "TABLE I · PROVENANCE", "每个阈值是怎么来的,必须能说清楚",
         "论文 Table I 专门用一整张表交代阈值出处,并声明:全部在最终批次开始前就"
         "写死在代码里,没有在这批数据上重新调过。")
table(s, CL, y, CW,
      ["阈值", "取值", "出处"],
      [["一阶矩塌陷比 ρ", "0.30", "**59 次早期飞行上做留一交叉验证,每一折都选中 0.30**"],
       ["残差下限 / 持续", "0.9 N; 2 或 4 帧", "低于最小负载 1.47 N 的重量;录到的 8 字越界 ≤3 帧,释放阶跃 4 帧"],
       ["慢路径持续", "5 帧 (0.5 s)", "工程取值"],
       ["质量域释放", "m_P<0.030 kg, 20 帧", "介于早期 0.15 kg 飞行的带载最小值 0.079 与释放后最大值 0.007 之间"],
       ["空载置信度", "0.90 持续 1.0 s", "工程取值"],
       ["UNRESOLVED 超时/刹停", "12 s / 3 s", "工程取值"]],
      [2.25, 2.05, 6.60], row_h=0.44, size=9)
callout(s, CL, y + 2.95, CW, 1.05,
        ["**只有 ρ 和残差下限是对着数据选的,而且用的是与最终批次不相交的早期飞行**;"
         "其余是工程取值,论文如实写明“其闭环敏感性我们没有扫过”。",
         "这一页的意义不在阈值本身,而在于:**阈值的来源本身就是要被审的对象**。"], "acc")
pagenum(s, 9)

# ---- 10 experiment design ----
s = add_slide()
rail(s, 10, "实验")
y = head(s, "EXPERIMENT DESIGN", "一个任务,四种“包裹没了”的方式",
         "只验证“正常投放能不能识别”说明不了什么。真正的考验是命令与物理脱节的情形"
         "——这也是“不依赖事件信号”唯一能被检验的地方。")
cw4 = (CW - 3 * 0.28) / 4
items = [
    ("情形 A", "正常投放", "接近 → 吸附 → 抬升 → 8 字机动 → 按相位触发投放。基准任务。"),
    ("情形 B", "延迟脱离 4 s", "命令发出后 DetachableJoint 仍保持中位 4.01 s。按命令清模型者必然判早。"),
    ("情形 C", "无指令丢失", "包裹掉了,但**没有任何命令到达控制器**。命令武装型确认器在构造上不可能发现。"),
    ("情形 D", "部分丢失", "4 个 0.05 kg 箱子先后掉 3 个(45/80/115 s),无信号无命令。考验多级阶跃跟随。"),
]
for i, (tag, h_, d) in enumerate(items):
    x = CL + i * (cw4 + 0.28)
    txbox(s, x, y, cw4, 0.32, [tag], 11, ACC, True, name=MONO)
    rect(s, x, y + 0.34, cw4, 0.022, ACC)
    txbox(s, x, y + 0.48, cw4, 0.38, [h_], 12.5, INK, True)
    txbox(s, x, y + 0.94, cw4, 1.5, [d], 9.5, SOFT, line=1.5)
callout(s, CL, y + 2.62, CW, 1.28,
        ["**预注册与全量报告:** 配对设计在比较之前就定好,不是事后挑一条臂;"
         "论文报告**每一次尝试**(84 次)、每次飞行走的确认路径、以及执行器极限处观察到的失败。",
         "**安全指标单独大样本统计:** 209 架次带载飞行(3.7 小时)中,"
         "命令或注入丢失之前的误释放 = **0**(单侧 95% 上界 1.4%/架次)。"], "acc")
pagenum(s, 10)

# ---- 11 main result ----
s = add_slide()
rail(s, 11, "实验")
y = head(s, "TABLE III + IV", "主结果:32 / 36,以及它到底证明了什么",
         "所有负载物理吸附成功、且飞机活到投放命令的飞行,全部计入。")
table(s, CL, y, 6.35,
      ["工况", "n", "完成", "机动中确认", "刹停后确认"],
      [["0.15 kg, 4 m/s", "11", "10/11", "2.90 s", "16.60 s"],
       ["0.30 kg, 4 m/s", "13", "10/13", "8.95 s", "16.40 s"],
       ["0.15 kg, 2 m/s", "12", "12/12", "3.31 s", "16.48 s"],
       ["合计", "36", "**32/36**", "**3.46 s**", "**16.56 s**"]],
      [1.85, 0.62, 1.00, 1.46, 1.42], row_h=0.36, size=9, hi_last=True)
txbox(s, CL, y + 1.92, 6.35, 0.8,
      ["95% Clopper–Pearson 区间 [73.9%, 96.9%]。**4 次失败全部在 4 m/s,"
       "且全部追溯到估计器失败**(3 次从未识别出负载、1 次释放后发散)。"],
      9.5, SOFT, line=1.5)
x2 = CL + 6.75
table(s, x2, y, CW - 6.75,
      ["工况", "n", "矩门控", "质量域", "仅置信度"],
      [["0.15 kg, 4 m/s", "10", "9", "1", "0"],
       ["0.30 kg, 4 m/s", "10", "1", "3", "6"],
       ["0.15 kg, 2 m/s", "12", "9", "0", "3"],
       ["合计", "32", "**19**", "**4**", "**9**"]],
      [1.42, 0.48, 0.75, 0.72, 0.78], row_h=0.36, size=9, hi_last=True)
callout(s, x2, y + 1.92, CW - 6.75, 1.05,
        ["**这一格是自我限定,不是成绩:** 32/36 是**整个接口**的条件闭环结果,"
         "不是任何单个释放检测器的验证 —— 因为确认走了**三条不同的路径**。"], "sig")
callout(s, CL, y + 3.12, CW, 0.95,
        ["**确认延迟是双峰的,论文拒绝把两列合并成一个平均数:** 17/32 在 8 字机动中就完成确认"
         "(中位 3.46 s);其余 15 次机动中拿不到证据,走满 12 s 超时 → 刹停 → 中位 16.56 s。"], "acc")
pagenum(s, 11)

# ---- 12 baseline ----
s = add_slide()
rail(s, 12, "实验")
y = head(s, "TABLE II · PAIRED ABLATION", "与“按命令清模型”的基线正面对照",
         "A 臂 = 按命令立即清除负载模型(传统做法);B 臂 = 本文接口;"
         "C 臂 = B 臂但释放证据只由命令武装。")
table(s, CL, y, CW,
      ["条件", "配对数", "A 完成", "B 完成", "提前清除 (A / B)"],
      [["标称工况汇总", "19", "19/19", "19/19", "— (无失配可测)"],
       ["注入 4 s 延迟脱离", "5", "5/5", "5/5", "!!A 5/5 提前!! / **B 0/5**"],
       ["同上,C 臂替代 A", "6", "6/6", "6/6", "**0/6 / 0/6**"]],
      [2.55, 1.05, 1.25, 1.25, 4.80], row_h=0.40, size=9.5)
cw2 = (CW - 0.4) / 2
card(s, CL, y + 1.72, cw2, 1.62, "标称工况下两臂打平 —— 这是诚实的结论", None,
     ["19/19 对 19/19,无一例释放后失稳。所以论文**不拿正常工况当卖点**。",
      "A 臂提前清除的中位提前量 = **4.01 s**;B 臂在 6 次有效运行中 "
      "**一次都没有提前清除**(5/6 进入 UNRESOLVED 后确认)。"], ACC)
card(s, CL + cw2 + 0.4, y + 1.72, cw2, 1.62, "唯一决定性的战场:无指令丢失", None,
     ["注入“没有任何命令到达”的负载丢失:**B 臂 18 次中检出 10 次**"
      "(中位延迟 2.2–2.9 s);**C 臂检出 0 次**,且一直保留着一个幻影一阶矩。",
      "差别是**能与不能**,不是好与更好。"], ACC)
callout(s, CL, y + 3.52, CW, 0.75,
        ["8 次漏检中有 7 次的原因相同:**丢失发生时,带载一阶矩参考窗口还没形成,"
         "或负载压根没被识别过**。"], "sig")
pagenum(s, 12)

# ---- 13 single flight ----
s = add_slide()
rail(s, 13, "实验")
y = head(s, "FIG. 4 · ONE FLIGHT", "一次完整飞行长什么样",
         "0.15 kg / 2 m/s,机动中确认。左:实测水平路径(蓝=携带,橙=释放后,灰虚=参考);"
         "右:高度、控制器实际消费的质量估计、位置误差范数。")
picture(s, "fig_mission.png", CL, y, CW)
yy = y + CW / 3.76 + 0.18
cw3 = (CW - 0.6) / 3
card(s, CL, yy, cw3, 1.55, "抓取", "质量自己涨上去",
     ["没有任何 attach 信号。抬升段那个 0.6 m 的误差峰是抬升本身,"
      "不是负载识别造成的。"], ACC)
card(s, CL + cw3 + 0.3, yy, cw3, 1.55, "携带", "8 字全程模型一致",
     ["载荷段跟踪 RMS 0.053 m,释放后 0.060 m —— 两者相当。"], ACC)
card(s, CL + 2 * (cw3 + 0.3), yy, cw3, 1.55, "释放", "命令后 4.46 s 确认",
     ["这段延迟是**自愿付的代价**:换来的是语义永远不跑在物理证据前面。"], SIG)
pagenum(s, 13)

# ---- 14 L1 baseline ----
s = add_slide()
rail(s, 14, "实验")
y = head(s, "TABLE V · vs. L1-ADAPTIVE NMPC", "与自适应基线比:赢在哪、不赢在哪",
         "{truth, online, l1} × {0.2, 0.3} kg × {0.05, 0.10} m,每格 n=5,"
         "共 60 次全部稳定、零发散。")
table(s, CL, y, 6.15,
      ["工况 (质量/偏心)", "truth", "online", "l1"],
      [["0.2 kg / 0.05 m", "0.806 / 4.28", "0.768 / 3.70", "0.821 / 4.26"],
       ["0.2 kg / 0.10 m", "0.794 / 3.62", "0.810 / 3.92", "0.812 / 4.24"],
       ["0.3 kg / 0.05 m", "0.834 / 3.56", "0.837 / 3.72", "0.883 / 4.90"],
       ["0.3 kg / 0.10 m", "1.011 / 3.64", "0.876 / 3.88", "0.891 / 4.54"]],
      [2.10, 1.35, 1.35, 1.35], row_h=0.38, size=9)
txbox(s, CL, y + 1.95, 6.15, 0.5, ["单元格 = 位置误差峰值 [m] / 恢复时间 [s]"],
      8.5, FAINT, name=MONO)
x2 = CL + 6.55
card(s, x2, y, CW - 6.55, 1.55, "不声称的部分", "暂态峰值:三方不可区分",
     ["格内差 0.05 m 以内。两个低偏心格 L1 更高(p=.008/.012),"
      "但**过不了多重比较校正 → 不声称暂态优势**。"], CRIT)
card(s, x2, y + 1.70, CW - 6.55, 1.55, "声称的部分", "恢复慢 8%–32%(中位 16%)",
     ["四格中三格显著。机制:L1 的补偿低通截止 ω_c 被闭环稳定性卡死"
      "(0.5 稳、1.0 就不稳)——**一堵带宽墙**。"], ACC)
callout(s, CL, y + 3.38, CW, 0.88,
        ["**必须一起说的两点:** (1) 这张表用的是**早期纯质量 14 状态 MHE**,"
         "**不是**本文最终的一阶矩接口——所以它只支撑有界的基线对比。",
         "(2) L1 给不出可解释的 m 和 c_xy,而且它做增益调度时**仍然需要同一类先验**。"], "sig")
pagenum(s, 14)

# ---- 15 limitations ----
s = add_slide()
rail(s, 15, "现状")
y = head(s, "LIMITATIONS (§VII)", "目前还不成立的事",
         "以下每条都是论文自己写进去的,不是别人挑出来的。汇报时主动讲,"
         "因为它们恰好定义了下一步工作。")
cw2 = (CW - 0.45) / 2
left = [
    "!!1!!  **全部证据都是 SITL。** 真机迁移要把仿真转速换成 ESC 遥测,"
    "并重新辨识推力/力矩系数、噪声底、时序与夹爪柔性。",
    "!!2!!  **推力系数 k 必须标定到 5% 以内。** 只把估计器的推力图缩放 ±5%:"
    "0.95 时负载 **4/4 从未被识别**;1.05 时释放后残留 0.09 kg 幻影负载(3/4 UNRESOLVED)。",
    "!!3!!  **一阶矩有系统性低估。** 携带段斜率 0.85,中位低估 10.8%,"
    "31/31 次同向。**原因未查明**;检测器用比值 ρ 所以对共同尺度偏差不敏感。",
]
right = [
    "!!4!!  **参考窗口形成不可靠。** 32 次完成飞行中有 **13 次**从未形成"
    "合格的一阶矩参考窗口,矩比检验根本没跑起来。",
    "!!5!!  **释放没有按估计器健康度门控。** 有 1 次飞行估计在释放前已不健康 48 s,"
    "而按相位定时的投放命令照发 → 释放后模型不匹配 → 发散。",
    "!!6!!  **0.30 kg / 4 m/s 是压力测试,不是工作点。** 悬停只用 74% 推力,"
    "但**力矩先撞墙**:8 字中 98% 的诊断窗口俯仰力矩饱和。",
]
txbox(s, CL, y, cw2, 2.92, left, 9.8, SOFT, line=1.5, space_after=11)
txbox(s, CL + cw2 + 0.45, y, cw2, 2.92, right, 9.8, SOFT, line=1.5, space_after=11)
callout(s, CL, y + 3.05, CW, 0.92,
        ["**还有两条容易被忽略的:** 持续的垂直上升/下沉气流与负载变化,"
         "在只有集体推力残差时**本质上不可分**(风未在本批次评估);"
         "释放证据路径的**内置默认是关闭的**,部署必须显式开启并在启动时校验配置。"], "crit")
pagenum(s, 15)

# ---- 16 closing ----
s = add_slide()
rail(s, 16, "总结")
y = head(s, "SUMMARY", "做成了什么,我自己做了什么")
stats = [("32/36", "无事件释放确认"), ("0 / 209", "带载误释放"),
         ("10/18", "无指令丢失检出"), ("0/60", "基线矩阵发散"), ("4", "论文图 / 5 表")]
sw_ = CW / len(stats)
for i, (v, k) in enumerate(stats):
    txbox(s, CL + i * sw_, y - 0.08, sw_, 0.60, [v], 22, INK, True, name=MONO)
    txbox(s, CL + i * sw_, y + 0.56, sw_, 0.32, [k], 8.5, FAINT)
yy = y + 1.06
cw2 = (CW - 0.45) / 2
txbox(s, CL, yy, cw2, 2.0,
      ["**已成立**",
       "✓  传感最小化:全链路只消费电机转速与里程计,attach/drop 通知不进入任何路径。",
       "✓  一阶矩接口:让负载几何在“包裹消失”这一刻仍然良定义。",
       "✓  释放语义:只在持续证据上清模型,证据缺失就 UNRESOLVED 并刹停。"],
      10, SOFT, line=1.5, space_after=7)
txbox(s, CL + cw2 + 0.45, yy, cw2, 2.0,
      ["**下一步**",
       "→  硬件迁移与 k 的台架标定(§VII 的头号前置条件)。",
       "→  把释放指令按估计器健康度门控;提高参考窗口形成的可靠性。",
       "→  补缺失基线:固定参数 NMPC、同信号下的 CUSUM 检测器、最终估计器 + L1。"],
      10, SOFT, line=1.5, space_after=7)
callout(s, CL, yy + 2.18, CW, 1.42,
        ["**关于 AI 辅助(建议主动说明):** 本项目在代码生成、重构、测试补充与文档整理上"
         "使用了 AI 辅助;实验设计、判据取舍与结论认定由我负责。我当前的工作重点是"
         "对生成结果做系统性理解与验证。",
         "汇报中我会区分三类内容:**已被代码与测试确认的**、**只在仿真中成立的**、"
         "**仍需核查的** —— 例如中文 README 至今仍写着“MHE 仅做诊断、不反馈进控制器”,"
         "而代码里估计值早已进入 NMPC 闭环,这条不一致就是我核查时发现的。"], "acc")
pagenum(s, 16)

os.makedirs(os.path.dirname(OUT), exist_ok=True)
prs.save(OUT)
print("saved:", OUT, os.path.getsize(OUT), "bytes,", len(prs.slides.__iter__.__self__._sldIdLst), "slides")
