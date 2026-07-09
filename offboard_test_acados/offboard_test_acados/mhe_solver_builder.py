#!/usr/bin/env python3
# 搭 MHE 的 AcadosOcp 并构建/复用求解器。跟 acados_solver_builder.py 用同一套
# 缓存机制(is_code_reuse_possible,配置没变就跳过重新生成/编译),独立的
# code_export_directory/json,不跟 NMPC 的求解器互相干扰。

from pathlib import Path

import numpy as np
from scipy.linalg import block_diag
from acados_template import AcadosOcp, AcadosOcpSolver

from .mhe_model import MODEL_NAME, build_mhe_model
from .mhe_params import p as mhe_p


def generated_root() -> Path:
    return Path(__file__).resolve().parent / 'acados_generated_files'


def codegen_dir() -> Path:
    return generated_root() / f'{MODEL_NAME}_c_generated_code'


def json_path() -> Path:
    return generated_root() / f'{MODEL_NAME}_acados_ocp.json'


def build_ocp() -> AcadosOcp:
    model = build_mhe_model()

    ocp = AcadosOcp()
    ocp.model = model
    ocp.code_export_directory = str(codegen_dir())

    ny_0 = mhe_p.nx + mhe_p.nw + mhe_p.nx_aug  # 测量(13) + 过程噪声(13) + 到达代价(14) = 40
    ny   = mhe_p.nx + mhe_p.nw                 # 测量(13) + 过程噪声(13) = 26

    ocp.solver_options.N_horizon = mhe_p.N
    ocp.solver_options.tf = mhe_p.N * mhe_p.dt

    # acados 官方 MHE 范式: stage 0 单独用 NONLINEAR_LS 带到达代价,
    # 中间 stage 用 NONLINEAR_LS(测量+过程噪声),末端不加代价(ny_e=0)。
    ocp.cost.cost_type_0 = 'NONLINEAR_LS'
    ocp.cost.cost_type   = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'

    ocp.cost.W_0 = block_diag(mhe_p.R, mhe_p.Q, mhe_p.Q0)
    ocp.cost.W   = block_diag(mhe_p.R, mhe_p.Q)

    ocp.cost.yref_0 = np.zeros(ny_0)
    ocp.cost.yref   = np.zeros(ny)
    ocp.cost.yref_e = np.zeros(0)
    ocp.cost.Vx_e = np.zeros((0, mhe_p.nx_aug))

    # 已知输入 [T, taux, tauy, tauz] + 已知几何 [dJ, cx, cy] 通过 model.p 传入,
    # 每步求解前用 solver.set(i,'p',...) 覆盖——必须显式设置初值,否则 acados
    # 在 OCP 构建时会报维度不一致的错(跟 acados_solver_builder.py 里同样的坑)。
    ocp.parameter_values = np.zeros(mhe_p.nu_known + mhe_p.n_geom)

    # 质量这一维加个宽松的物理边界,纯粹防止激励不足的窗口把质量推到离谱的值,
    # 不是真实约束——必须连 idxbx 一起设,漏了的话 acados 不报错但约束形同虚设
    # (跟 NMPC 那边漏 idxbu 是同一类坑)。
    ocp.constraints.idxbx = np.array([mhe_p.nx])
    ocp.constraints.lbx = np.array([mhe_p.m_min])
    ocp.constraints.ubx = np.array([mhe_p.m_max])

    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 1
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.qp_solver_cond_N = mhe_p.N
    ocp.solver_options.nlp_solver_type = 'SQP'
    #跟 NMPC 那边同样的理由:FIXED_STEP 在四元数这种非线性代价上容易卡死循环,
    # 换成带回溯线搜索的 MERIT_BACKTRACKING。
    ocp.solver_options.globalization = 'MERIT_BACKTRACKING'

    return ocp


def ensure_mhe_ocp_solver(force: bool = False) -> AcadosOcpSolver:
    generated_root().mkdir(parents=True, exist_ok=True)
    ocp = build_ocp()
    if force:
        return AcadosOcpSolver(ocp, json_file=str(json_path()),
                                generate=True, build=True, check_reuse_possible=False)
    return AcadosOcpSolver(ocp, json_file=str(json_path()),
                            generate=False, build=False, check_reuse_possible=True)