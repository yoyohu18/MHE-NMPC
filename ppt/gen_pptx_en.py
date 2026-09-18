# -*- coding: utf-8 -*-
"""English deck: Eventless Payload-Adaptive NMPC.
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
OUT  = "/home/clear/ros2_ws_HJH/src/ppt/MHE-NMPC-Presentation-EN.pptx"

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

CN = "Segoe UI"
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
rect(s, 0.95, 2.18, 0.9, 0.032, ACC)
txbox(s, 0.92, 2.45, 5.8, 1.0, ["MHE–NMPC"], 50, INK, True, line=1.0, name=MONO)
txbox(s, 0.95, 3.52, 5.5, 1.2,
      ["Eventless Payload-Adaptive NMPC: letting the controller decide for itself "
       "whether the parcel is gone"], 15, INK, True, line=1.5)
txbox(s, 0.95, 4.92, 5.6, 1.3,
      ["ROS 2 Jazzy · PX4 SITL · Gazebo Harmonic · acados",
       "Under review (target ICRA 2027) · all evidence is SITL",
       "Numbers and figures from paper/main.pdf (Fig. 1–4, Tables I–V)"],
      10, FAINT, line=1.75, space_after=2, name=MONO)
picture(s, "fig_moment_truth.png", 7.35, 2.05, 5.45)
txbox(s, 7.35, 4.70, 5.45, 1.7,
      ["**Paper Fig. 3** — estimated first mass moment ŝ_y against truth "
       "(0.15 kg, 2 m/s, n = 11).",
       "It rises at grasp, holds through the figure-eight, decays to zero after release — "
       "with **no attach/drop notification ever entering the estimator**. Truth is "
       "reconstructed offline and never reaches it."],
      9.5, SOFT, line=1.5, space_after=5)

# ---- 02 problem ----
s = add_slide()
rail(s, 2, "PROBLEM")
y = head(s, "MOTIVATION", "Grasping a parcel breaks the model the controller is using",
         "An NMPC's predictive power rests on one assumption: that it knows its own mass, "
         "where its centre of mass sits, and how hard it is to rotate. A grasp changes all "
         "three at once.")
cw3 = (CW - 0.6) / 3
card(s, CL, y, cw3, 2.05, "1 / THRUST · OUTER LOOP", "Hover throttle is wrong",
     ["Mass rescales the thrust-to-acceleration map. A biased m biases feed-forward thrust — "
      "and the margin set against mg with it."], CRIT)
card(s, CL + cw3 + 0.3, y, cw3, 2.05, "2 / TORQUE · INNER LOOP", "A constant trim torque appears",
     ["Thrust no longer passes through the composite CoM: `τ ≈ Δr × f`. Since f ≈ mg is large, "
      "**even a small offset matters**."], CRIT)
card(s, CL + 2 * (cw3 + 0.3), y, cw3, 2.05, "3 / INERTIA · INNER LOOP", "Attitude response slows",
     ["The same torque now yields less angular acceleration — effectively lowering "
      "inner-loop bandwidth."], CRIT)
callout(s, CL, y + 2.28, CW, 1.28,
        ["**The abstract opens on the premise:** “A release command does not guarantee that a "
         "payload has physically detached.”",
         "A command is an **intent**, not an **event**. The gripper may not open; the object may "
         "lag; the notification may be lost. A controller that clears its model on that command "
         "**flies a loaded vehicle as if it were empty**."], "crit")
pagenum(s, 2)

# ---- 03 architecture ----
s = add_slide()
rail(s, 3, "SYSTEM")
y = head(s, "ARCHITECTURE · FIG. 1", "The pipeline consumes only motor speeds and odometry",
         "The attach/drop notification is drawn as NOT CONSUMED — not an omission in the "
         "figure, but the constraint this work imposes on itself.")
PW = 9.4
picture(s, "fig1_architecture.png", CL + (CW - PW) / 2, y - 0.22, PW)
yy = y - 0.22 + PW / 2.88 + 0.14
cw2 = (CW - 0.35) / 2
card(s, CL, yy, cw2, 1.12, "TWO SOLVERS", None,
     ["**MHE** 10 Hz · 16 states · 2.0 s window;   "
      "**NMPC** 20 Hz · 13 states / 4 inputs · 1.0 s preview."], ACC)
card(s, CL + cw2 + 0.35, yy, cw2, 1.12, "ONE ATOMIC FRAME", None,
     ["`{m, s, ΔJ, γ, h, a}` on one timestamp, with health bit and solution age — "
      "**never split across topics**."], ACC)
pagenum(s, 3)

# ---- 04 first moment ----
s = add_slide()
rail(s, 4, "CONTRIBUTION 1")
y = head(s, "WHY THE FIRST MOMENT · §IV-B", "Estimate the first mass moment, not the CoM offset",
         "The most important modelling choice in the paper — and the one most likely "
         "to be challenged.")
cw2 = (CW - 0.4) / 2
card(s, CL, y, cw2, 2.30, "WHY c_xy FAILS", "The CoM offset is undefined at release",
     ["Physically `s = m_P·r_xy → 0`, but `r_xy` is **0/0** as `m_P → 0`.",
      "Not hypothetical: in the mass-only formulation the optimizer could suppress a phantom "
      "eccentricity only by driving mass below the bare airframe — !!14 of 55!! post-release "
      "samples pinned at the bound."], CRIT)
card(s, CL + cw2 + 0.4, y, cw2, 2.30, "WHY s WORKS", "Inertial parameters enter linearly",
     ["The first moment is the **standard linear parameterization** of rigid-body inertial "
      "parameters — classical in manipulator load identification.",
      "It supplies the rotational channel directly, and `c_xy = s/m` follows "
      "**without knowing the grasp geometry**."], ACC)
callout(s, CL, y + 2.52, CW, 1.35,
        ["**The smallest structure, not extra degrees of freedom:** the least that represents "
         "**payload existence, trim moment and roll/pitch inertia** without measuring "
         "attachment geometry.",
         "ΔJ is **not estimated independently** — it follows from m̂ and a weak vertical-arm "
         "prior r_z, structurally unobservable from the thrust-moment channel."], "acc")
pagenum(s, 4)

# ---- 05 NMPC ----
s = add_slide()
rail(s, 5, "METHOD")
y = head(s, "NMPC", "Re-solving one second of future every 50 milliseconds",
         "Unlike a PID it does not react to the present error: it predicts one second ahead, "
         "picks an optimal input sequence inside the actuator envelope, and applies only "
         "the first step.")
table(s, CL, y, 5.55,
      ["ITEM", "VALUE"],
      [["States / inputs", "13 / 4"],
       ["Horizon", "N=20 × Δt=0.05 s → 1.0 s"],
       ["Solver", "acados SQP-RTI + HPIPM"],
       ["Thrust envelope", "T ∈ [1.27, 31.35] N"],
       ["Torque envelope", "|τx|,|τy| ≤ 0.5 · |τz| ≤ 0.2 N·m"],
       ["Solve time", "1.3 ms median / 2.2 ms p99"],
       ["Share of period", "2.6% / 4.4%  (n = 1009)"]],
      [1.95, 3.60], row_h=0.345, size=9.5)
x2 = CL + 5.95
card(s, x2, y, CW - 5.95, 1.95, "COST", "Bryson's rule + a live hover reference",
     ["Tolerances 0.1 m / 0.3 m/s / 0.05 / 0.3 rad/s / 2 N.",
      "**ū(p) = [mg, c_y·mg, −c_x·mg, 0]** tracks the estimate and **includes the eccentric "
      "trim torque** — never the bare mass."], ACC)
card(s, x2, y + 2.15, CW - 5.95, 2.05, "RUNTIME PARAMETERS", "p = [x_ref, m, ΔJ, c_xy]",
     ["Payload information enters as a **runtime parameter** — no code change, no solver "
      "regeneration.",
      "m / s / ΔJ each pass a filter and a **slew limit** before every solve."], ACC)
pagenum(s, 5)

# ---- 06 MHE ----
s = add_slide()
rail(s, 6, "METHOD")
y = head(s, "MHE", "Payload parameters become extra states",
         "Over the last two seconds, find the states and parameters that best reconcile model "
         "with measurement — not one instant, so time itself suppresses noise.")
eqbox(s, CL, y, 6.1, 0.95,
      ["x_MHE = [ x(13)   m   s_x   s_y ]  ∈  R^16",
       "ṁ = 0, ṡ = 0 in-window    N=20 × Δt=0.1 s → 2.0 s @ 10 Hz"], 11)
card(s, CL, y + 1.12, 6.1, 2.38, "WEIGHTS ARE A STATEMENT · W_w ≫ W_y", "Deliberately trust the dynamics",
     ["Process-noise weights `diag(1e4·I6, 1e5·I4, 1e3·I3)` are far tighter than the "
      "measurement weights.",
      "The meaning: **apart from the payload parameters, the model is right**.",
      "The optimizer must then explain the observed acceleration by moving m and s — it cannot "
      "quietly charge the mismatch to process noise."], ACC)
x2 = CL + 6.5
card(s, x2, y, CW - 6.5, 1.32, "AN ARRIVAL COST THAT KNOWS NOTHING", None,
     ["W_0 is nearly uninformative — σ ≈ 3.2 kg in m, σ = 0.1 kg·m in s. A strong prior would "
      "make the estimate an echo of itself."], ACC)
callout(s, x2, y + 1.50, CW - 6.5, 1.95,
        ["!!State the timing honestly:!! **9.2 ms** median, 24.6 ms p90, **59.2 ms** p99, "
         "72.8 ms max — against a 100 ms period.",
         "It meets the deadline in simulation, but the **tail margin is narrow** (≈7× the "
         "mass-only estimator), so the paper makes **no onboard-headroom claim**."], "crit")
pagenum(s, 6)

# ---- 07 thrust proxy ----
s = add_slide()
rail(s, 7, "CONTRIBUTION 2")
y = head(s, "ESTIMATE-INDEPENDENT PROXY", "Never feed the controller's own thrust command back in",
         "The MHE needs a known input force. The nearest source is the NMPC's own command — "
         "and that closes an estimator–controller loop.")
cw2 = (CW - 0.4) / 2
card(s, CL, y, cw2, 2.75, "THE TRAP · A SELF-CONSISTENT BLIND SPOT", "The command comes from m̂",
     ["The cost anchors T near `m̂·g`. Once m̂ is wrong, the commanded thrust is no longer the "
      "force the vehicle feels.",
      "Worse: **inversion error, saturation and motor lag between command and rotor are "
      "invisible to the estimator**. A mass error can persist self-consistently instead of "
      "showing up as a residual."], CRIT)
card(s, CL + cw2 + 0.4, y, cw2, 2.75, "THE FIX · SAMPLE DOWNSTREAM", "Reconstruct from rotor speeds",
     ["`T_phys = k·Σωᵢ²`   and   `τ_phys = Σ rᵢ × Fᵢ`",
      "Measured rotor speeds, downstream of the real nonlinear mapping, **not computed from m̂**.",
      "Detail: rotor messages are far faster than the MHE period, so the window averages "
      "**per-rotor thrust Fᵢ** — never ωᵢ before squaring."], ACC)
callout(s, CL, y + 2.94, CW, 0.95,
        ["**What it does depend on:** a calibrated k, which loses validity under large induced "
         "inflow in fast translation — a term we do not model. Slide 15 prices it: "
         "**a ±5% error defeats the interface in both directions**."], "sig")
pagenum(s, 7)

# ---- 08 release semantics ----
s = add_slide()
rail(s, 8, "CONTRIBUTION 3")
y = head(s, "RELEASE SEMANTICS", "Issuing the release command is not evidence",
         "The controller clears its payload model through one route only: confirmation — "
         "no-payload confidence γ > 0.90 held for 1.0 s on healthy, fresh frames.")
eqbox(s, CL, y, CW, 1.00,
      ["γ  =  empty-score(mass)  ×  empty-score(first moment)  ×  empty-score(inertia)",
       "→ a nonzero first moment alone vetoes a false EMPTY, even with mass on its bound"], 11)
cw2 = (CW - 0.4) / 2
card(s, CL, y + 1.18, cw2, 2.05, "WHEN EVIDENCE IS MISSING", "Enter UNRESOLVED — do not guess",
     ["12 s timeout → declare UNRESOLVED → 3 s smooth brake to hover, then gather evidence "
      "in low-dynamic flight.",
      "**A timeout is treated as unknown, never as proof of an empty vehicle.**"], ACC)
card(s, CL + cw2 + 0.4, y + 1.18, cw2, 2.05, "THE FAST PATH WE DELETED", "It caused real premature releases",
     ["“Residual + quantity-empty”, without moment evidence, released "
      "!!early in 3 of 9 flights!! during a 4 m/s figure-eight.",
      "Cause: under aggressive flight m̂ sits on its lower bound for long stretches, so the "
      "quantity channel reads empty **persistently**."], CRIT)
callout(s, CL, y + 3.42, CW, 0.78,
        ["**A recorded figure-eight transient reached −1.69 N — larger than the −1.47 N step of "
         "a real release of the smallest payload.** Amplitude alone cannot separate manoeuvre "
         "from detachment."], "crit")
pagenum(s, 8)

# ---- 09 thresholds ----
s = add_slide()
rail(s, 9, "RIGOUR")
y = head(s, "TABLE I · PROVENANCE", "Where every threshold came from",
         "The paper spends a whole table on this, and states plainly: all values were fixed in "
         "code before the final campaign and were not retuned on it.")
table(s, CL, y, CW,
      ["THRESHOLD", "VALUE", "PROVENANCE"],
      [["Moment-collapse ratio ρ", "0.30", "**leave-one-out CV on 59 earlier flights; every fold chose 0.30**"],
       ["Residual floor / persist", "0.9 N; 2 or 4 frames", "below the 1.47 N weight of the smallest payload; recorded figure-eight excursions lasted ≤3 frames, the release step 4"],
       ["Slow-path persistence", "5 frames (0.5 s)", "engineering choice"],
       ["Mass-domain release", "m_P < 0.030 kg, 20 fr.", "between the loaded minimum (0.079 kg) and post-release maximum (0.007 kg) of earlier flights"],
       ["No-payload confidence", "0.90 for 1.0 s", "engineering choice"],
       ["UNRESOLVED timeout / brake", "12 s / 3 s", "engineering choice"]],
      [2.55, 2.10, 6.25], row_h=0.44, size=9)
callout(s, CL, y + 2.95, CW, 1.05,
        ["**Only ρ and the residual floor were selected against data — and against earlier "
         "flights disjoint from the reported campaign.** The rest are engineering choices, and "
         "the paper says outright that their closed-loop sensitivity was not swept.",
         "The point is not the numbers: **the provenance of a threshold is itself something to "
         "be audited**."], "acc")
pagenum(s, 9)

# ---- 10 experiment design ----
s = add_slide()
rail(s, 10, "EXPERIMENTS")
y = head(s, "EXPERIMENT DESIGN", "One mission, four ways for the payload to be gone",
         "Showing that a normal release is recognised proves little. The real test is a "
         "mismatch between command and physics — the only place “eventless” can be falsified.")
cw4 = (CW - 3 * 0.28) / 4
items = [
    ("CASE A", "Nominal release",
     "Approach → grasp → lift → figure-eight → phase-triggered release."),
    ("CASE B", "Delayed detachment",
     "The joint is held a median **4.01 s** past the command. Anything clearing on the command "
     "must be early."),
    ("CASE C", "Unsignaled loss",
     "The payload goes, but **no command reaches the controller**. A command-armed confirmer "
     "cannot see this."),
    ("CASE D", "Partial loss",
     "Four 0.05 kg boxes, three detached one at a time, unsignaled. Tests multi-step tracking, "
     "not a binary call."),
]
for i, (tag, h_, d) in enumerate(items):
    x = CL + i * (cw4 + 0.28)
    txbox(s, x, y, cw4, 0.32, [tag], 11, ACC, True, name=MONO)
    rect(s, x, y + 0.34, cw4, 0.022, ACC)
    txbox(s, x, y + 0.48, cw4, 0.38, [h_], 12.5, INK, True)
    txbox(s, x, y + 0.94, cw4, 1.6, [d], 9.5, SOFT, line=1.5)
callout(s, CL, y + 2.70, CW, 1.35,
        ["**Pre-registered and reported in full:** the paired design was fixed before the "
         "comparison, not chosen after it; the paper reports **every attempt** (84), the "
         "confirmation path each flight took, and the failures at the actuator limit.",
         "**Safety is measured separately and at scale:** across 209 loaded flights (3.7 h), "
         "releases declared before any command or injected loss = **0** (one-sided 95% upper "
         "bound 1.4% per flight)."], "acc")
pagenum(s, 10)

# ---- 11 main result ----
s = add_slide()
rail(s, 11, "EXPERIMENTS")
y = head(s, "TABLES III + IV", "32 of 36 — and what that does not establish",
         "Every flight in which the payload physically attached and the vehicle survived to the "
         "release command is counted.")
table(s, CL, y, 6.35,
      ["CONDITION", "n", "COMPL.", "IN-MANOEUVRE", "VIA BRAKE"],
      [["0.15 kg, 4 m/s", "11", "10/11", "2.90 s", "16.60 s"],
       ["0.30 kg, 4 m/s", "13", "10/13", "8.95 s", "16.40 s"],
       ["0.15 kg, 2 m/s", "12", "12/12", "3.31 s", "16.48 s"],
       ["pooled", "36", "**32/36**", "**3.46 s**", "**16.56 s**"]],
      [1.75, 0.55, 1.00, 1.60, 1.45], row_h=0.36, size=9, hi_last=True)
txbox(s, CL, y + 1.92, 6.35, 0.85,
      ["95% Clopper–Pearson [73.9%, 96.9%]. **All four failures are at 4 m/s and all trace to "
       "estimator failure** — three never recognised the payload, one diverged after release."],
      9.5, SOFT, line=1.5)
x2 = CL + 6.75
table(s, x2, y, CW - 6.75,
      ["CONDITION", "n", "MOMENT", "MASS-DOM.", "CONF. ONLY"],
      [["0.15 kg, 4 m/s", "10", "9", "1", "0"],
       ["0.30 kg, 4 m/s", "10", "1", "3", "6"],
       ["0.15 kg, 2 m/s", "12", "9", "0", "3"],
       ["pooled", "32", "**19**", "**4**", "**9**"]],
      [1.42, 0.48, 0.75, 0.72, 0.78], row_h=0.36, size=9, hi_last=True)
callout(s, x2, y + 1.92, CW - 6.75, 1.28,
        ["**A limitation, not a score:** 32/36 holds for **the interface as a whole**, not for "
         "any single detector — confirmation arrived by **three different paths**."], "sig")
callout(s, CL, y + 3.35, CW, 0.88,
        ["**The delay is bimodal, and the paper refuses to pool it:** 17 of 32 confirm while "
         "still flying the figure-eight (median 3.46 s); the other 15 find no evidence in "
         "manoeuvre, hit the 12 s timeout, brake to hover, and confirm at a median 16.56 s."], "acc")
pagenum(s, 11)

# ---- 12 baseline ----
s = add_slide()
rail(s, 12, "EXPERIMENTS")
y = head(s, "TABLE II · PAIRED ABLATION", "Head-to-head with clearing on the command",
         "Arm A clears the payload model at the command (the conventional approach); arm B is "
         "the proposed interface; arm C is arm B, armed only by the command.")
table(s, CL, y, CW,
      ["CONDITION", "PAIRS", "A COMPL.", "B COMPL.", "EARLY CLEAR (A / B)"],
      [["nominal, pooled", "19", "19/19", "19/19", "— (no mismatch to measure)"],
       ["+ 4 s detachment hold", "5", "5/5", "5/5", "!!A 5/5 early!! / **B 0/5**"],
       ["same, arm C instead of A", "6", "6/6", "6/6", "**0/6 / 0/6**"]],
      [2.85, 1.05, 1.25, 1.25, 4.50], row_h=0.40, size=9.5)
cw2 = (CW - 0.4) / 2
card(s, CL, y + 1.72, cw2, 1.80, "THE ARMS TIE UNDER NOMINAL CONDITIONS", None,
     ["19/19 vs 19/19, no post-release instability. The paper therefore **does not sell the "
      "nominal case**.",
      "Arm A cleared a median **4.01 s before** detachment; arm B cleared early in **none of "
      "its six valid runs**."], ACC)
card(s, CL + cw2 + 0.4, y + 1.72, cw2, 1.80, "THE ONE DECISIVE CASE", None,
     ["With losses injected and no command reaching the controller: **arm B detected 10 of 18**; "
      "**arm C detected none**, and kept a phantom first moment.",
      "The difference is **possible vs impossible**, not better vs worse."], ACC)
callout(s, CL, y + 3.66, CW, 0.58,
        ["Seven of the eight misses share one cause: the moment reference had not formed before "
         "the loss."], "sig")
pagenum(s, 12)

# ---- 13 one flight ----
s = add_slide()
rail(s, 13, "EXPERIMENTS")
y = head(s, "FIG. 4 · ONE EVENTLESS FLIGHT", "What one complete flight looks like",
         "0.15 kg at 2 m/s, confirmed in manoeuvre. Right: height, the mass estimate the "
         "controller actually consumed, and position-error norm.")
picture(s, "fig_mission.png", CL, y, CW)
yy = y + CW / 3.76 + 0.18
cw3 = (CW - 0.6) / 3
card(s, CL, yy, cw3, 1.62, "GRASP", "The mass rises on its own",
     ["No attach signal — the 0.6 m error peak is the lift itself, not recognition."], ACC)
card(s, CL + cw3 + 0.3, yy, cw3, 1.62, "CARRY", "The model stays consistent",
     ["Loaded tracking RMS 0.053 m; 0.060 m after release."], ACC)
card(s, CL + 2 * (cw3 + 0.3), yy, cw3, 1.62, "RELEASE", "Confirmed 4.46 s later",
     ["The delay is the **price we choose**: semantics that never run ahead of evidence."], SIG)
pagenum(s, 13)

# ---- 14 L1 baseline ----
s = add_slide()
rail(s, 14, "EXPERIMENTS")
y = head(s, "TABLE V · vs. L1-ADAPTIVE NMPC", "Against the adaptive baseline: claims and non-claims",
         "{truth, online, l1} × {0.2, 0.3} kg × {0.05, 0.10} m, n = 5 per cell — "
         "60 of 60 runs stable, zero divergence.")
table(s, CL, y, 6.15,
      ["CELL (MASS / ECC)", "TRUTH", "ONLINE", "L1"],
      [["0.2 kg / 0.05 m", "0.806 / 4.28", "0.768 / 3.70", "0.821 / 4.26"],
       ["0.2 kg / 0.10 m", "0.794 / 3.62", "0.810 / 3.92", "0.812 / 4.24"],
       ["0.3 kg / 0.05 m", "0.834 / 3.56", "0.837 / 3.72", "0.883 / 4.90"],
       ["0.3 kg / 0.10 m", "1.011 / 3.64", "0.876 / 3.88", "0.891 / 4.54"]],
      [2.10, 1.35, 1.35, 1.35], row_h=0.38, size=9)
txbox(s, CL, y + 1.98, 6.15, 0.4, ["cell = position-error peak [m] / recovery [s]"],
      8.5, FAINT, name=MONO)
callout(s, CL, y + 2.55, 6.15, 1.50,
        ["**Two caveats travel with this table:** it was recorded with the earlier "
         "**mass-only 14-state MHE, not** the proposed first-moment interface — so it supports "
         "only a bounded baseline comparison; and L1 yields no interpretable m or c_xy while "
         "still needing the same class of prior for gain scheduling."], "sig")
x2 = CL + 6.55
card(s, x2, y, CW - 6.55, 1.90, "WHAT WE DO NOT CLAIM", "Transient peaks are indistinguishable",
     ["Within-cell peaks agree to 0.05 m. L1 is higher in the two low-eccentricity cells "
      "(p = .008, .012), **but that does not survive correction over eight tests**."], CRIT)
card(s, x2, y + 2.08, CW - 6.55, 1.97, "WHAT WE DO CLAIM", "L1 recovers 8–32% slower",
     ["Median 16%, significant in three of four cells. The mechanism is a **bandwidth wall**: "
      "L1's compensation cutoff ω_c is bounded by closed-loop stability (0.5 stable, "
      "1.0 unstable here)."], ACC)
pagenum(s, 14)

# ---- 15 limitations ----
s = add_slide()
rail(s, 15, "STATUS")
y = head(s, "LIMITATIONS · §VII", "What does not hold yet",
         "Every item below is stated by the paper itself. They are worth presenting plainly, "
         "because they define the next steps.")
cw2 = (CW - 0.45) / 2
left = [
    "!!1!!  **All system-level evidence is SITL.** Hardware migration replaces simulated rotor "
    "speeds with ESC telemetry and must re-identify thrust/torque coefficients, noise floors, "
    "timing and gripper compliance.",
    "!!2!!  **k must be calibrated well within 5%.** Scaling only the estimator's thrust map by "
    "±5%: at 0.95 the payload was **never recognised (4/4)**; at 1.05 a phantom 0.09 kg "
    "persisted after release.",
    "!!3!!  **The first moment is biased low.** Slope 0.85 in carry, median underestimate 10.8%, "
    "same direction in 31/31 flights. **The cause is not identified.**",
]
right = [
    "!!4!!  **Reference formation is unreliable.** In **13 of the 32** completed flights the "
    "in-envelope reference window never filled, so the moment-ratio test never ran.",
    "!!5!!  **Release is not gated on estimator health.** One flight's estimate was unhealthy "
    "for 48 s before release; the phase-timed command fired anyway and the vehicle diverged.",
    "!!6!!  **0.30 kg at 4 m/s is a stress test, not an operating point.** Hover needs 74% of "
    "maximum thrust, but **torque binds first**: pitch saturated within 98% of diagnostic "
    "windows.",
]
txbox(s, CL, y, cw2, 2.92, left, 9.8, SOFT, line=1.5, space_after=11)
txbox(s, CL + cw2 + 0.45, y, cw2, 2.92, right, 9.8, SOFT, line=1.5, space_after=11)
callout(s, CL, y + 3.05, CW, 1.05,
        ["**Two more that are easy to miss:** a sustained updraft or downdraft is "
         "**intrinsically confounded** with a payload change on a collective-thrust-only "
         "residual (wind is not evaluated here); and the release-evidence path is **off by "
         "default**, so deployment needs an explicit enable plus startup validation."], "crit")
pagenum(s, 15)

# ---- 16 closing ----
s = add_slide()
rail(s, 16, "SUMMARY")
y = head(s, "SUMMARY", "What stands, and what I did myself")
stats = [("32/36", "eventless confirmations"), ("0 / 209", "pre-command releases"),
         ("10/18", "unsignaled losses caught"), ("0/60", "baseline divergences"),
         ("±5%", "thrust-map tolerance")]
sw_ = CW / len(stats)
for i, (v, k) in enumerate(stats):
    txbox(s, CL + i * sw_, y - 0.08, sw_, 0.60, [v], 22, INK, True, name=MONO)
    txbox(s, CL + i * sw_, y + 0.56, sw_, 0.32, [k], 8.5, FAINT)
yy = y + 1.06
cw2 = (CW - 0.45) / 2
txbox(s, CL, yy, cw2, 2.15,
      ["**Established**",
       "✓  Minimal sensing — the path consumes only rotor speeds and odometry; no attach/drop "
       "notification enters it.",
       "✓  A first-moment interface that stays well defined as the payload vanishes.",
       "✓  Release semantics that clear the model only on persistent evidence, and brake to "
       "hover otherwise."],
      10, SOFT, line=1.5, space_after=7)
txbox(s, CL + cw2 + 0.45, yy, cw2, 2.15,
      ["**Next**",
       "→  Hardware migration and bench calibration of k — the prerequisite named in §VII.",
       "→  Gate the release command on estimator health; make reference formation reliable.",
       "→  Close the missing baselines: fixed-parameter NMPC, a CUSUM detector, L1 with the "
       "final estimator."],
      10, SOFT, line=1.5, space_after=7)
callout(s, CL, yy + 2.30, CW, 1.58,
        ["**On AI assistance — worth stating up front:** this project used AI assistance for "
         "code generation, refactoring, test coverage and documentation; the experiment design, "
         "the choices between criteria, and the conclusions are mine.",
         "I distinguish three kinds of statement throughout: **confirmed by code and tests**, "
         "**true in simulation only**, and **still to be verified** — for instance, the Chinese "
         "README still says the MHE is diagnostic-only, while the estimate has long been inside "
         "the NMPC loop. I found that by reading the code, not the docs."], "acc")
pagenum(s, 16)

os.makedirs(os.path.dirname(OUT), exist_ok=True)
prs.save(OUT)
print("saved:", OUT, os.path.getsize(OUT), "bytes,", len(prs.slides._sldIdLst), "slides")
