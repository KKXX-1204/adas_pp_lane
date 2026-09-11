---
theme: default
title: perception_pp 车道线后处理源码解析
info: |
  基于 perception_pp 当前工作区实际源码整理。
  覆盖 Pipeline、函数调用关系、功能开关、关键阈值、代码实现和维护边界。
author: Xiaotian Ke
highlighter: shiki
lineNumbers: true
transition: slide-left
aspectRatio: 16/9
canvasWidth: 980
drawings:
  persist: false
defaults:
  transition: slide-left
---

<div class="road-grid"></div>
<div class="eyebrow">SOURCE CODE WALKTHROUGH · PERCEPTION_PP</div>

# 车道线后处理解析

## Pipeline、函数调用、功能开关与关键实现

<div class="mt-10 flex gap-3">
  <span class="pill">LanePostProcess</span>
  <span class="pill">RobustLaneFitter</span>
  <span class="pill">Station / Coefficient KF</span>
  <span class="pill">Center Lane & LDW</span>
</div>

<div class="absolute bottom-11 left-14 text-sm opacity-60">
源码范围：<code>perception_pp/src/lane_postprocessor</code>
</div>

---
layout: center
class: text-center
---

# 这份演示稿回答四个问题

<div class="grid grid-cols-4 gap-4 mt-12 text-left">
  <div class="metric-card"><span>01</span><b>数据怎么走？</b><p>从 DetectResult 到 PPResult 的完整 Pipeline</p></div>
  <div class="metric-card"><span>02</span><b>函数怎么调？</b><p>入口、主流程与算法模块的调用关系</p></div>
  <div class="metric-card"><span>03</span><b>开关怎么影响？</b><p>投影、拟合、KF、预测、平行化与 LDW</p></div>
  <div class="metric-card"><span>04</span><b>代码真实做什么？</b><p>关键阈值、质量门控和实现边界</p></div>
</div>

<div class="note-box mt-10 text-left"><b>依据：</b>当前磁盘工作区源码，而不是仅按已有设计文档或 Git HEAD 推断。</div>

---

# 源码地图

<div class="source-map mt-6">
  <div class="primary"><code>core/lane_postprocessor_impl.cpp</code><b>主流程编排</b><span>投影、拟合、跟踪、输出、平行化、中线、LDW</span></div>
  <div><code>cubic_fit/robust_lane_fitter.cpp</code><b>单帧鲁棒拟合</b><span>Huber IRLS + 二/三次选择 + 协方差</span></div>
  <div><code>cubic_fit/weighted_polynomial_fit.hpp</code><b>数值求解</b><span>归一化纵向坐标 + Householder QR</span></div>
  <div><code>point_filter/lane_points_filter.cpp</code><b>Legacy 点过滤</b><span>MAD、Huber、距离分桶和回退</span></div>
  <div><code>lane_filter/lane_station_kf.cpp</code><b>默认时序滤波</b><span>固定纵向站点 + end_x 状态</span></div>
  <div><code>lane_filter/lane_coefficient_kf.cpp</code><b>可选时序滤波</b><span>归一化系数状态 + 不确定度裁剪</span></div>
  <div><code>interface/pp_fusion.hpp</code><b>配置与输出协议</b><span>PP_Option / PPLane / PPCenterLane</span></div>
</div>

<div class="source-ref">CMakeLists.txt：src/*.cpp 全量编入共享库 libpp</div>

---
zoom: 0.82
class: call-relation-slide
---

# 顶层函数调用关系

<div class="call-tree mt-7">
  <div class="call-row"><span class="caller">app / integration</span><i>调用</i><span class="callee">LanePostProcessor(option)</span></div>
  <div class="call-row indent-1"><span class="caller">constructor</span><i>创建</i><span class="callee">LanePostProcessorImpl(option)</span></div>
  <div class="call-row"><span class="caller">每帧</span><i>调用</i><span class="callee active">LanePostProcessor::Process()</span></div>
  <div class="call-row indent-1"><span class="caller">Process</span><i>转调</i><span class="callee active">LanePostProcessorImpl::LanePostProcess()</span></div>
  <div class="call-row indent-2"><span class="caller">主流程末端</span><i>调用</i><span class="callee">applyParallelLanePostprocess()</span></div>
  <div class="call-row indent-2"><span class="caller">主流程末端</span><i>调用</i><span class="callee">buildCenterLaneAndWarning()</span></div>
  <div class="call-row indent-3"><span class="caller">中线 / LDW</span><i>调用</i><span class="callee">buildOneCenterLane()</span></div>
</div>

```cpp {1-3|5-11|all}
LanePostProcessor::LanePostProcessor(const PP_Option& option) {
  impl_ = std::make_unique<LanePostProcessorImpl>(option);
}

void LanePostProcessor::Process(DetectResult& input,
                                const Chassis& chassis,
                                PPResult& output) {
  auto start = high_resolution_clock::now();
  impl_->LanePostProcess(input, chassis, output);
  spdlog::warn("tc of lane Process: {} us.", elapsed(start));
}
```

<div class="source-ref">core/lane_postprocessor_impl.cpp:1294–1307</div>

---

# 构造阶段：模块创建与配置固化

```cpp {1-3|5-10|12-18|all}
sp_front_cam_ = std::make_shared<PinHoleCamera<float>>(...);
sp_lane_points_filter_ = std::make_shared<LanePointFilter>(640, 352);
sp_robust_lane_fitter_ = std::make_shared<RobustLaneFitter>();

lane_fit_path_ = option.LANE_FIT_PATH;
lane_filter_type_ = option.LANE_FILTER_TYPE;
use_lane_points_filter_ = option.USE_LANE_POINTS_FILTER;
use_lane_kf_filter_ = option.USE_LANE_KF_FILTER;
use_lane_predict_ = option.USE_LANE_PREDICT;
use_lookup_table_ = option.USE_LOOK_UP_TABLE;

if (use_lookup_table_ && !lut_p2g_.loadFromFile(option.lut_path)) {
  spdlog::warn("lane LUT unavailable, falling back to camera projection");
  use_lookup_table_ = false;
}

openLaneLogFile();
openLaneAlarmLogFile();
```

<div class="grid grid-cols-3 gap-5 mt-5">
  <div class="mini-card"><b>算法对象</b><span>相机、点过滤器、鲁棒拟合器</span></div>
  <div class="mini-card"><b>运行参数</b><span>PP_Option 复制到实现对象成员</span></div>
  <div class="mini-card"><b>资源初始化</b><span>LUT 与两类日志文件</span></div>
</div>

<div class="source-ref">core/lane_postprocessor_impl.cpp:195–243</div>

---
class: pipeline-slide
---

# LanePostProcess：一帧的完整 Pipeline

<div class="pipeline-contract mt-3">
  <div><small>FRAME INPUT</small><b>DetectResult.input.lanes</b><span>像素归一化点、type / score / shape / color</span></div>
  <i>+</i><div><small>VEHICLE STATE</small><b>Chassis</b><span>道路类型、转向灯等状态</span></div>
  <strong>→ LanePostProcess() →</strong>
  <div class="result"><small>FRAME OUTPUT</small><b>PPResult</b><span>lanes / center_lanes / LDW / debug</span></div>
</div>

<div class="pipeline-frame mt-3">
  <div class="pipeline-row">
    <div class="pipeline-phase observe"><small>阶段 A</small><b>观测构建</b><span>像素点 → VRF 点</span></div>
    <div class="flow-node source"><code>inner_lane_pool_[1..4]</code><b>1. 初始化槽位</b><span>固定四类语义位置，避免输入顺序影响状态跟踪</span></div><div class="flow-arrow">→</div>
    <div class="flow-node"><code>image2ground()</code><b>2. 去重与投影</b><span>同 type 保留最高 existence_prob；有效地面点需 ≥ 4</span></div><div class="flow-arrow">→</div>
    <div class="flow-node"><code>filterLanePoints()</code><b>3. Legacy 点过滤</b><span>开关开启且为 LEGACY_QUADRATIC 才执行</span></div><div class="flow-arrow">→</div>
    <div class="flow-node"><code>RobustLaneFitter::fit()</code><b>4. 单帧拟合</b><span>输出系数、内点、跨度、残差、置信度与协方差</span></div>
  </div>

  <div class="pipeline-row">
    <div class="pipeline-phase track"><small>阶段 B</small><b>几何与时序</b><span>当前帧 → 稳定轨迹</span></div>
    <div class="flow-node"><code>meanCurveDistance()</code><b>5. 几何去重</b><span>两语义槽曲线过近时，仅保留质量更高者</span></div><div class="flow-arrow">→</div>
    <div class="flow-node"><code>lane_change_status_</code><b>6. 变道状态机</b><span>|c0| &lt; 0.8 m 进入；退出需左右安全连续 3 帧</span></div><div class="flow-arrow">→</div>
    <div class="flow-node active"><code>update() / predict()</code><b>7. KF 与短时预测</b><span>Station / Coefficient 二选一；失联窗口 ≤ 500 ms</span></div><div class="flow-arrow">→</div>
    <div class="flow-node"><code>hasLaneNaNValues()</code><b>8. PPLane 装配</b><span>KF 系数优先；否则使用 org_coefs；NaN 整线丢弃</span></div>
  </div>

  <div class="pipeline-row">
    <div class="pipeline-phase product"><small>阶段 C</small><b>业务派生</b><span>边界线 → 业务结果</span></div>
    <div class="flow-node"><code>applyParallelLanePostprocess()</code><b>9. 多车道平行化</b><span>道路与质量门控通过后，共享中位 c1 / c2 / c3</span></div><div class="flow-arrow">→</div>
    <div class="flow-node"><code>buildOneCenterLane()</code><b>10. 中线构造</b><span>相邻左右边界配对，生成 left / ego / right center</span></div><div class="flow-arrow">→</div>
    <div class="flow-node target"><code>buildCenterLaneAndWarning()</code><b>11. LDW 状态机</b><span>&lt; 0.8 m 或帧间缩小 &gt; 0.05 m；连续 2 帧触发</span></div><div class="flow-arrow">→</div>
    <div class="flow-node"><code>logLaneResults()</code><b>12. 日志与观测</b><span>输出结果、告警、点统计、tc[0..5] 与总耗时</span></div>
  </div>
</div>

<div class="pipeline-branches mt-3">
  <div><b>投影分支</b><p><code>LUT 可用</code><i>→ 查表</i></p><p><code>加载失败</code><i>→ Camera::image2ground</i></p></div>
  <div><b>拟合分支</b><p><code>LEGACY</code><i>加权二次</i></p><p><code>ROBUST</code><i>Q2 / Q3 / Adaptive</i></p></div>
  <div><b>时序分支</b><p><code>稳定观测</code><i>KF update</i></p><p><code>短时缺失</code><i>predict；超时失效</i></p></div>
  <div><b>业务门控</b><p><code>转向灯 ON</code><i>抑制 LDW</i></p><p><code>非常规道路</code><i>跳过平行化</i></p></div>
</div>

<div class="source-ref">core/lane_postprocessor_impl.cpp:250–810</div>

---
class: timing-slide
---

#  不同阶段耗时统计

<div class="timing-intro mt-3" v-motion :initial="{ opacity: 0, y: -8 }" :enter="{ opacity: 1, y: 0, transition: { duration: 420 } }">
  <b>一次车道线后处理被拆成 6 个连续阶段</b><span>每个 <code>tc[i]</code> 表示该阶段独立耗时，单位为 μs</span>
</div>

<div class="timing-grid mt-3">
  <div v-click="1"><header><code>tc[0]</code><span>L255–261</span></header><b>语义槽位初始化</b><p>创建 type 1–4 的 <code>InnerLane</code>；初始化同类型最优分数表。</p><footer><em>产物</em><span>inner_lane_pool_ + lane_type_score</span></footer></div>
  <div v-click="1"><header><code>tc[1]</code><span>L263–320</span></header><b>输入选择与坐标投影</b><p>类型校验、同 type 取最高 existence_prob、保存原始点、坐标检查、<code>image2ground()</code>。</p><footer><em>门槛</em><span>投影成功点 &lt; 4 → is_valid=false</span></footer></div>
  <div v-click="1"><header><code>tc[2]</code><span>L322–340</span></header><b>可选 Legacy 点过滤</b><p>统计最终点分布；仅 Legacy + 开关开启时调用 <code>filterLanePoints()</code>。</p><footer><em>门槛</em><span>过滤后点数 &lt; 4 → is_valid=false</span></footer></div>
  <div v-click="2"><header><code>tc[3]</code><span>L342–416</span></header><b>单帧拟合与质量统计</b><p>执行 Legacy 或 Robust 拟合；写入系数、跨度、内点、fit_score、degree、协方差。</p><footer><em>附带</em><span>logLanePointStats() 计入本段</span></footer></div>
  <div class="hot" v-click="2"><header><code>tc[4]</code><span>L418–694</span></header><b>几何去重与时序跟踪</b><p>曲线去重、变道状态机、拟合阶次迟滞、Station / Coefficient KF、短时预测。</p><footer><em>通常最重</em><span>多槽位 × 关联、更新、重拟合</span></footer></div>
  <div v-click="2"><header><code>tc[5]</code><span>L696–802</span></header><b>结果装配与业务派生</b><p>生成 PPLane、NaN 整线拦截、平行化、中线构造和 LDW。</p><footer><em>产物</em><span>lanes / center_lanes / warning / debug</span></footer></div>
</div>

<div class="timing-bottom mt-3">
  <div class="timing-log" v-click="3" v-motion :initial="{ opacity: 0, scale: 0.94 }" :click-3="{ opacity: 1, scale: 1, transition: { duration: 420 } }">
    <header><span class="terminal-dots">● ● ●</span><b>RUNTIME LOG · spdlog::warn</b><em>μs</em></header>
    <div class="log-line"><span class="log-prefix">lane tc = [</span><strong class="t0">12</strong><strong class="t1">238</strong><strong class="t2">31</strong><strong class="t3">410</strong><strong class="t4">690</strong><strong class="t5">145</strong><span class="log-suffix">].</span><i></i></div>
    <footer><span class="log-label-spacer"></span><span>tc[0]</span><span>tc[1]</span><span>tc[2]</span><span>tc[3]</span><span>tc[4]</span><span>tc[5]</span></footer>
  </div>
  <div class="timing-difference" v-click="4"><b>为什么 Σtc ≠ Process 总耗时？</b><p><code>tc[5]</code> 在中线与 LDW 完成后立即封账；随后生成六段日志字符串、调用 <code>spdlog::warn()</code> 和 <code>logLaneResults()</code>，这些仍处于 <code>Process()</code> 总计时范围内，却不属于任何一个 <code>tc[i]</code>。</p></div>
</div>

<div class="source-ref">core/lane_postprocessor_impl.cpp:250–810, 1300–1307</div>

---
class: config-flow-slide
---

# 输入数据如何被配置引导并形成输出？

<div class="config-flow mt-4">
  <section class="config-input">
    <header><small>01 · INPUT</small><b>一帧感知 + 自车状态</b></header>
    <div class="config-card vision">
      <span>视觉检测结果</span>
      <b>车道线点集与语义</b>
      <p>归一化图像点、置信度、车道类型、线型、颜色、时间戳</p>
    </div>
    <div class="config-card vehicle">
      <span>Chassis</span>
      <b>车辆运动与驾驶意图</b>
      <p>车速、横摆角速度、挡位、方向盘角度、左右转向灯</p>
    </div>
  </section>

  <div class="config-arrow"><span>数据进入</span><b>→</b></div>

  <section class="config-router">
    <header><small>02 · CONFIGURATION</small><b>开关与配置决定处理路径</b></header>
    <div class="router-grid">
      <div><span>投影方式</span><b>查表 / 相机模型</b><p><code>USE_LOOK_UP_TABLE</code></p></div>
      <div class="focus"><span>单帧拟合</span><b v-mark="{ at: 1, color: '#1677b8', type: 'box' }">Legacy / Robust Q2 / Q3 / Adaptive</b><p><code>LANE_FIT_PATH</code> · 点过滤开关</p></div>
      <div class="focus"><span>时序稳定</span><b v-mark="{ at: 2, color: '#f59e0b', type: 'underline' }">KF 滤波 + 短时预测</b><p>滤波类型 · 预测开关 · 500 ms 窗口</p></div>
      <div><span>业务派生</span><b>平行化 + LDW</b><p>功能开关与道路 / 转向灯门控</p></div>
    </div>
    <div class="router-rule"><b>配置只改变“走哪条路径”</b><span>核心数据仍沿着 投影 → 拟合 → 稳定 → 业务派生 顺序流动</span></div>
  </section>

  <div class="config-arrow"><span>汇总结果</span><b>→</b></div>

  <section class="config-output">
    <header><small>03 · OUTPUT</small><b v-mark="{ at: 3, color: '#18a875', type: 'circle' }">PPResult</b></header>
    <div class="output-item"><i>①</i><div><b>车道边界线</b><p>图像点 / 车辆坐标点、曲线系数、有效与预测状态</p></div></div>
    <div class="output-item"><i>②</i><div><b>车道中心线</b><p>左侧、当前车道、右侧中心线</p></div></div>
    <div class="output-item"><i>③</i><div><b>偏离预警</b><p>NONE / LEFT / RIGHT</p></div></div>
    <div class="output-item"><i>④</i><div><b>调试信息</b><p>拟合、预测和曲线阶次快照</p></div></div>
  </section>
</div>

<div class="config-paths mt-4">
  <div class="vision-path"><b>视觉数据路径</b><span>车道点与语义</span><i>→</i><span>投影</span><i>→</i><span>拟合</span><i>→</i><span>KF / 预测</span><i>→</i><strong>边界线</strong></div>
  <div class="vehicle-path"><b>底盘数据路径</b><span>速度 / 横摆 / 转向灯</span><i>→</i><span>运动补偿与业务门控</span><i>→</i><strong>中心线 + LDW</strong></div>
</div>

<div class="source-ref">interface/data.h · interface/pp_fusion.hpp · core/lane_postprocessor_impl.cpp</div>

---
class: fit-deep-slide
---

# 单帧拟合：四种方式究竟有什么不同？

<div class="fit-methods mt-4">
  <div class="legacy"><header><code>LEGACY_QUADRATIC</code><span>传统二次</span></header><b>普通加权最小二乘</b><p>可先做独立点过滤，再调用旧的加权曲线求解器；固定输出二次模型。</p><footer><strong>优势</strong>兼容旧行为 <i>·</i> <strong>风险</strong>过滤与拟合分离</footer></div>
  <div class="default"><header><code>ROBUST_QUADRATIC</code><span>默认</span></header><b>Huber IRLS 二次</b><p>距离权重 × 置信度权重，并在拟合内部反复抑制离群点；固定二次。</p><footer><strong>优势</strong>稳定、低自由度 <i>·</i> <strong>适合</strong>常规道路</footer></div>
  <div><header><code>ROBUST_CUBIC</code><span>强制三次</span></header><b>Huber IRLS 三次</b><p>同一套鲁棒权重，但直接求 c0–c3；至少 8 点且跨度 ≥15 m。</p><footer><strong>优势</strong>表达曲率变化 <i>·</i> <strong>风险</strong>远端过拟合</footer></div>
  <div class="adaptive"><header><code>ROBUST_ADAPTIVE</code><span>条件选择</span></header><b>Q2 基线与 Q3 竞争</b><p>先拟合二次，仅在三次通过几何与误差收益检查后升级。</p><footer><strong>优势</strong>复杂度按需 <i>·</i> <strong>代价</strong>判断链更长</footer></div>
</div>

<div class="fit-shared mt-3"><b>Robust 三条路径共用：</b><span>0–60 m 有限点</span><i>×</i><span>近端优先权重</span><i>×</i><span>置信度平方权重</span><i>×</i><span>MAD + Huber IRLS（4 次）</span></div>

````md magic-move {lines: true}
```cpp
// 1. 配置决定 Legacy 或 Robust
if (lane_fit_path_ == LEGACY_QUADRATIC)
  fitLegacy(points);
else
  robust_fitter_->fit(points, mode);
```
```cpp
// 2. 两种固定阶次模式：不会自动在 Q2 / Q3 间选择
if (lane_fit_path_ == ROBUST_QUADRATIC)
  robust_mode = RobustLaneFitMode::QUADRATIC; // degree = 2
else if (lane_fit_path_ == ROBUST_CUBIC)
  robust_mode = RobustLaneFitMode::CUBIC;     // degree = 3

result = robust_fitter_->fit(points, robust_mode);
```
```cpp
// 3. Adaptive：Q3 必须真正优于 Q2
quadratic = fitDegree(points, 2);
if (quadratic.success && points >= 8 && span >= 15m) {
  cubic = fitDegree(points, 3);
  use_cubic = geometry_ok &&
              cubic.rmse < 0.90 * quadratic.rmse &&
              cubic.p95 <= quadratic.p95;
}
```
````

<div class="upgrade-guard">
  <header><small>ROBUST_ADAPTIVE ONLY</small><b>候选 Q3 不能立即改变最终输出阶次</b></header>
  <div class="upgrade-gates">
    <section class="geometry"><em>01 · 单帧候选门</em><strong>几何合理 + 误差收益</strong><span>|slope|≤0.8 · |curvature|≤0.1 · RMSE 至少改善 10% · P95 不劣化</span></section>
    <i>→</i>
    <section class="history"><em>02 · 跨帧迟滞门</em><strong>degree=3 连续出现 3 帧</strong><span>候选退回 Q2，则 pending 计数重新开始</span></section>
    <i>→</i>
    <strong class="upgrade-output">允许切换<br><code>tracker.fit_degree=3</code></strong>
  </div>
</div>
<div class="source-ref">robust_lane_fitter.hpp:40–52 · robust_lane_fitter.cpp:123–143, 257–271 · lane_postprocessor_impl.cpp:129–156</div>

---
class: filter-guard-slide
---

# 点过滤与拟合后几何保护：从点到稳定观测

<div class="filter-enable mt-4">
  <div><small>拟合路径</small><b v-mark="{ at: 1, color: '#1677b8', type: 'box' }">LEGACY_QUADRATIC</b></div><span>AND</span>
  <div><small>过滤开关</small><b v-mark="{ at: 2, color: '#f59e0b', type: 'underline' }">USE_LANE_POINTS_FILTER = true</b></div><i>→</i>
  <strong v-mark="{ at: 3, color: '#18a875', type: 'circle' }">执行 filterLanePoints()</strong>
  <em v-click="3">Robust 路径不走这个独立过滤器</em>
</div>

<div class="filter-and-guard mt-4">
  <section class="filter-side"><header><small>BEFORE FIT</small><b>点过滤筛选机制</b></header>
    <div class="mini-flow"><span>60 m 硬裁剪</span><i>→</i><span>近中远分桶</span><i>→</i><span>MAD / Huber ×3</span><i>→</i><span>残差筛选</span></div>
    <TechFormula kind="filter-weight" />
    <div class="fallback-pair"><div><b>保留率 &lt;45%</b><span>回退</span></div><div><b>跨度保留 &lt;60%</b><span>回退</span></div></div>
  </section>
  <i class="major-arrow">→ FIT →</i>
  <section class="guard-side"><header><small>AFTER FIT</small><b>两层几何状态保护</b></header>
    <div class="guard-item"><b>① 曲线去重</b><span>共同区间平均距离 &lt;0.8 m → 仅保留 fit_score 更高者</span></div>
    <div class="guard-item"><b>② 变道重置</b><span>任一侧 |c0|&lt;0.8 m → 清空 Tracker；双侧&gt;1.2 m 连续3帧退出</span></div>
  </section>
</div>

````md magic-move {lines: true}
```cpp
// 点级保护：不过度删除
if (retained_ratio < 0.45 || span_ratio < 0.60)
  return points_inside_60m;
```
```cpp
// 曲线级保护：重复语义槽只留高质量观测
if (meanCurveDistance(a, b) < 0.8)
  lower_fit_score_lane.fit_success = false;
```
```cpp
// 状态级保护：变道时切断旧历史
if (min(abs(left.c0), abs(right.c0)) < 0.8) {
  lane_change_active_ = true;
  ltracker_pool_.clear();
}
```
````
<div class="source-ref">lane_points_filter.cpp:199–289 · lane_postprocessor_impl.cpp:322–340, 418–466</div>

---
class: station-deep-slide
---

# Station KF：在道路坐标中跟踪“曲线长什么样”

<div class="kf-config-line mt-4"><code>USE_LANE_KF_FILTER=true</code><i>+</i><code>LANE_FILTER_TYPE=STATION_KF</code><b>默认</b></div>

<div class="station-deep-grid mt-4">
  <section class="state"><header><small>STATE</small><b>固定站点横向位置 + 速度</b></header>
    <div class="station-dots"><span>0</span><span>5</span><span>10</span><span>20</span><span>30</span><span>50</span></div>
    <div class="station-state-visual"><div class="state-road"></div><div class="state-point p0"><b>y₀</b><small>x=0</small></div><div class="state-point p1"><b>y₅</b><small>x=5</small></div><div class="state-point p2"><b>y₂₀</b><small>x=20</small></div><div class="state-point p3"><b>y₅₀</b><small>x=50</small></div><span class="state-arrow">y(x) + ẏ(x) → 下一帧</span></div>
    <TechFormula kind="station-state" />
    <p>end_x 作为第 7 个量测独立跟踪，描述本帧可靠纵向范围。</p>
  </section>
  <section class="noise"><header><small>MEASUREMENT NOISE</small><b>每个站点的 R 都不同</b></header>
    <TechFormula kind="station-noise" />
    <div class="noise-bars"><span style="--w:28%">近端</span><span style="--w:52%">远端</span><span style="--w:78%">区间外推</span><span style="--w:92%">低质量</span></div>
    <p>越远、越超出观测区间、拟合质量 q 越低，量测方差 Rᵢᵢ=σᵢ² 越大。</p>
  </section>
  <section class="motion"><header><small>PREDICT & OUTPUT</small><b>先补偿自车运动，再预测</b></header>
    <div class="motion-visual"><div class="motion-axis"><span class="axis-x">x</span><span class="axis-y">y</span><i class="lane-before"></i><i class="lane-after"></i><b class="car-motion">▶</b><em>Δs=vΔt</em><strong>Δψ=ωΔt</strong></div><p>把上一帧曲线沿车辆运动反向搬回当前坐标系</p></div>
    <TechFormula kind="station-motion" />
    <p>滤波后的 6 个 y 站点再次加权拟合，恢复最终 c0–c3。</p>
  </section>
</div>

<div class="station-logic mt-4"><span>initialize</span><i>→</i><span>运动补偿</span><i>→</i><span>F/Q predict</span><i>→</i><span>动态 R update</span><i>→</i><strong>站点重拟合</strong><em>关联距离 &gt;1.5 m 或量测间隔超窗 → reset</em></div>
<div class="source-ref">lane_station_kf.cpp:25–198 · lane_postprocessor_impl.cpp:592–692</div>

---
class: coeff-deep-slide
---

# Coefficient KF：在模型空间中跟踪“参数如何变化”

<div class="kf-config-line mt-4 alt"><code>USE_LANE_KF_FILTER=true</code><i>+</i><code>LANE_FILTER_TYPE=COEFFICIENT_KF</code><b>可选</b></div>

<div class="coeff-deep-grid mt-4">
  <section><header><small>NORMALIZED STATE</small><b>30 m 尺度归一化</b></header>
    <div class="normalize-visual"><div class="norm-ruler"><span>0</span><i></i><span>10</span><i></i><span>20</span><i></i><span>30 m</span></div><div class="norm-curves"><b>原始 x</b><i></i><b>归一化 s=x/30</b><i class="norm2"></i></div><small>把不同长度车道映射到统一 [0,1] 尺度</small></div>
    <TechFormula kind="coefficient-state" />
    <p>先用纵向位移展开多项式，再从一次项扣除横摆；不同阶次的变化率按 0.7/0.5/0.25/0.15 s 衰减。</p>
  </section>
  <section><header><small>ADAPTIVE R & GATE</small><b>质量、跨度与协方差共同决定信任度</b></header>
    <TechFormula kind="coefficient-noise" />
    <div class="logic-tags"><span>span&lt;12m → 放大 c2 噪声</span><span>非三次或 span&lt;25m → 强抑制 c3</span></div>
    <TechFormula kind="coefficient-innovation" />
    <div class="decision-row"><span>通过：执行 KF update</span><span>拒绝：fit_success=false</span></div>
  </section>
  <section><header><small>UNCERTAINTY CLIP</small><b>输出区间由协方差决定</b></header>
    <TechFormula kind="coefficient-uncertainty" />
    <div class="uncertainty-visual">
      <div class="scan-caption"><b>从近到远逐米扫描</b><span>比较 σ_curve(x) 与允许上限 σₐ(x)</span></div>
      <div class="uncertainty-steps">
        <div class="u-step safe"><b>5 m</b><span>σ=.22</span><i><em style="--v:31%"></em></i><small>≤ .40 · 继续</small></div>
        <div class="u-step safe"><b>10 m</b><span>σ=.31</span><i><em style="--v:44%"></em></i><small>≤ .40 · 继续</small></div>
        <div class="u-step safe"><b>30 m</b><span>σ=.67</span><i><em style="--v:72%"></em></i><small>≤ .70 · 继续</small></div>
        <div class="u-step cut"><b>31 m</b><span>σ=.73</span><i><em style="--v:92%"></em></i><small>&gt; .715 · 截断</small></div>
      </div>
      <div class="uncertainty-legend"><span><i class="legend-green"></i>绿色：可继续输出</span><span><i class="legend-orange"></i>橙色：首次超阈值，可靠终点 = 30 m</span></div>
    </div>
    <div class="geometry-limits"><span>|slope| ≤ 0.8</span><span>|curvature| ≤ 0.1</span></div>
    <p>逐米向远端扫描；σ_curve 超阈值即截断。预测输出还必须在 0/10/30/end m 全部通过几何合理性检查。</p>
  </section>
</div>

<div class="coeff-compare mt-4"><div><b>Station KF</b><span>状态直观、局部位置误差隔离</span></div><div class="active"><b>Coefficient KF</b><span>状态紧凑、可直接输出系数，但高阶项需要更强约束</span></div><div><b>KF OFF</b><span>跳过时序状态，直接使用 org_coefs</span></div></div>
<div class="source-ref">lane_coefficient_kf.cpp:24–196 · lane_postprocessor_impl.cpp:469–571</div>

---
class: predict-rich-slide
---

# 短时预测与输出安全

<div class="predict-road mt-4">
  <div class="observe" v-click="1"><small>01</small><b>观测质量门</b><span>投影点≥6 · 内点≥6 · span≥2m · score≥0.55 · fit_score&gt;0</span></div><i>通过</i>
  <div class="update" v-click="2"><small>02</small><b>KF update</b><span>写入 last_measure_timestamp，miss_count 清零</span></div><i>失检</i>
  <div class="predict" v-click="3"><small>03</small><b>短时 predict</b><span>KF ON + PREDICT ON + tracker initialized + miss&lt;500ms</span></div><i>超时</i>
  <div class="expire" v-click="4"><small>04</small><b>停止历史输出</b><span>达到 500 ms，不再把旧车道续到当前帧</span></div>
</div>

<div class="safety-dashboard mt-5">
  <section><header><b>点级安全</b><span>points / points_vrf</span></header><div class="point-cloud"><i></i><i></i><i class="bad"></i><i></i><i></i><i></i></div><p>原始图像点保留用于诊断；VRF 点中 x/y 为 NaN 的点逐个跳过。</p><div class="safety-rule"><b>单点失败</b><span>跳过该点，整线继续</span></div></section>
  <section><header><b>区间安全</b><span>可输出的曲线范围</span></header>
    <div class="range-visual" aria-label="从车辆原点到可靠终点的输出区间">
      <div class="range-track">
        <span class="range-output"></span><span class="range-disabled"></span>
        <i class="range-car" aria-label="车辆原点"><svg viewBox="0 0 72 36" role="img"><path d="M8 22h4l5-11h31l10 11h6v7H8z" fill="#102235"/><path d="M18 11l6-8h18l9 8z" fill="#1677b8"/><path d="M25 6h14l5 5H21z" fill="#bdf3f5"/><path d="M12 18h7M51 18h8" stroke="#22b8cf" stroke-width="2"/><circle cx="20" cy="29" r="5" fill="#263b4a"/><circle cx="54" cy="29" r="5" fill="#263b4a"/><circle cx="20" cy="29" r="2" fill="#d9f8fa"/><circle cx="54" cy="29" r="2" fill="#d9f8fa"/><path d="M64 19l7 4-7 4z" fill="#f59e0b"/></svg></i>
        <div class="range-observations"><i></i><i></i><i></i><i></i><i></i></div>
        <em class="range-end"></em>
      </div>
      <div class="range-labels"><b>车辆原点<br><small>0 m</small></b><span>实际观测点</span><strong>可靠终点<br><small>reliableEndX</small></strong><em>终点外不输出</em></div>
    </div>
    <p>多项式可从自车原点开始计算；Tracker 的平滑 <code>end_x</code> 决定远端边界，最少保留 5 m。</p><div class="safety-rule"><b>输出区间</b><span>[0, reliableEndX]</span></div>
  </section>
  <section><header><b>整线安全</b><span>最终提交前</span></header><div class="nan-check"><span>c0</span><span>c1</span><span>c2</span><span>c3</span><span>start</span><span>end</span></div><p>任一值为 NaN：整条 PPLane 丢弃，清空帧内点和有效/预测状态。</p><div class="safety-rule danger"><b>任一 NaN</b><span>整条 lane 拒绝提交</span></div></section>
</div>

<div class="output-decision mt-4">
  <header><small>FINAL COMMIT RULE</small><b>系数来源合法，并通过整线有限值检查，才写入 PPResult.lanes</b></header>
  <div class="output-decision-flow">
    <section><em>01 · 选择系数</em><p><code>KF ON</code>：拟合成功或允许预测 → <b>smoothed_coeffs</b></p><p><code>KF OFF</code>：仅拟合成功 → <b>org_coefs</b></p></section>
    <i>→</i>
    <section><em>02 · 整线校验</em><p><code>c0–c3 / start / end</code> 必须全部为有限值</p><p>任一 NaN → 清状态、清点集并 <code>continue</code></p></section>
    <i>→</i>
    <strong>03 · 提交结果<code>pp_output.lanes.emplace_back(pp_lane)</code></strong>
  </div>
</div>
<div class="source-ref">lane_postprocessor_impl.cpp:116–127, 469–797</div>

---
class: parallel-fixed-slide
---

# 多车道平行化

<div class="parallel-top mt-4">
  <section class="gates"><header><code>USE_LANE_PARALLEL_POSTPROCESS=true</code></header><div><b>① 有效线 ≥2</b><b>② 无转向/大方向盘/大横摆</b><b>③ 5m–30m 宽度变化 ≤1.0m</b><b>④ c2/c3 极差 ≤0.02</b></div></section>
  <section class="visual"><div class="parallel-road"><span class="p1"></span><span class="p2"></span><span class="p3"></span><em>BEFORE</em></div><i>→</i><div class="parallel-road after"><span class="p1"></span><span class="p2"></span><span class="p3"></span><em>AFTER</em></div></section>
</div>

<div class="parallel-explain"><b>不修改 c0：</b><span>各车道横向位置保持独立</span><i>·</i><b>共享中位数：</b><span>c1、c2、c3 统一为车道间 median</span><i>·</i><b>同步回写：</b><span>PPLane 与 InnerLane 保持一致</span></div>

````md magic-move {lines: true}
```cpp
// Stage 1: 独立拟合结果
lane_i(x) = c0_i + c1_i*x + c2_i*x*x + c3_i*x*x*x;
```
```cpp
// Stage 2: 场景门控通过后计算公共形状
common_c1 = median(all_valid_c1);
common_c2 = median(all_valid_c2);
common_c3 = median(all_valid_c3);
```
```cpp
// Stage 3: 只替换形状项
for (auto& lane : valid_lanes) {
  lane.c1 = common_c1;
  lane.c2 = common_c2;
  lane.c3 = common_c3;
  // lane.c0 保持不变
}
```
````
<div class="source-ref">pp_fusion.hpp:94–97 · lane_postprocessor_impl.cpp:1063–1156</div>

---
class: switch-summary-slide
---

# 功能开关与路径配置总表

| 阶段 | 配置项 | 默认值 | 作用与关闭行为 |
|---|---|---:|---|
| 投影 | `USE_LOOK_UP_TABLE` | `true` | LUT 失败或关闭 → 相机模型 |
| 单帧拟合 | `LANE_FIT_PATH` | `ROBUST_QUADRATIC` | Legacy / Robust Q2 / Q3 / Adaptive |
| Legacy 点过滤 | `USE_LANE_POINTS_FILTER` | `true` | 仅 Legacy 生效；关闭则投影点直接拟合 |
| 时序滤波 | `USE_LANE_KF_FILTER` | `true` | 关闭 → 直接使用 org_coefs |
| 滤波类型 | `LANE_FILTER_TYPE` | `STATION_KF` | Station / Coefficient 二选一 |
| 短时预测 | `USE_LANE_PREDICT` | `true` | 关闭 → 失检帧不使用历史补线 |
| 预测窗口 | `LANE_PREDICT_PERIOD_MS` | `500` | 超时 → 停止历史车道输出 |
| 平行化 | `USE_LANE_PARALLEL_POSTPROCESS` | `true` | 关闭或门控失败 → 保持独立形状 |
| LDW | `USE_LANE_DEPARTURE_WARNING` | `true` | 关闭告警，不影响边界线输出 |

<div class="switch-foot mt-5"><b>配置主线</b><span>拟合决定单帧模型</span><i>→</i><span>KF 决定跨帧状态</span><i>→</i><span>预测处理短时失检</span><i>→</i><span>平行化施加多车道几何约束</span></div>
<div class="source-ref">interface/pp_fusion.hpp:16–29, 67–97</div>
