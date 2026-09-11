# ADAS 车道线后处理源码解析

这是一个基于 [Slidev](https://sli.dev/) 制作的技术演示稿，围绕 `perception_pp` 中的车道线后处理模块展开，帮助读者理解从感知输入到车道线、中心线和车道偏离预警输出的完整处理链路。

## 内容概览

- `LanePostProcessor` 的入口和整体调用关系
- `DetectResult`、`Chassis` 到 `PPResult` 的数据流
- 车道点坐标投影、去重、过滤和单帧拟合
- Legacy、Robust Q2/Q3、Adaptive 等拟合路径
- Station KF、Coefficient KF 和短时预测机制
- 多车道平行化、中心线构造与 LDW 状态机
- `tc[0..5]` 阶段耗时、质量门控和日志输出
- `PP_Option` 功能开关对处理路径的影响

演示稿内容依据当前工作区中的源码整理，重点参考 `perception_pp/src/lane_postprocessor` 目录。

## 快速开始

### 安装依赖

需要 Node.js 以及 npm。安装项目依赖：

```bash
npm install
```

### 启动开发预览

```bash
npm run dev
```

启动后访问 <http://localhost:3030>。修改 [slides.md](./slides.md) 后，浏览器会自动更新。

### 构建静态文件

```bash
npm run build
```

构建产物位于 `dist/` 目录。

### 导出演示稿

```bash
npm run export
```

该命令使用 Slidev 的导出能力生成可分发的演示稿文件。根据本地环境，导出过程可能需要额外的浏览器运行依赖。

## 项目结构

```text
.
├── slides.md                 # 演示稿主体内容
├── style.css                 # 全局样式和车道线主题视觉
├── components/               # Vue 组件
├── pages/                    # Slidev 页面级内容
├── snippets/                 # 代码片段
├── package.json              # 脚本与依赖
├── netlify.toml              # Netlify 构建配置
└── vercel.json               # Vercel 部署配置
```

## 编辑说明

- 修改演示内容：编辑 [slides.md](./slides.md)
- 修改颜色、布局和组件样式：编辑 [style.css](./style.css)
- 新增可复用交互或展示模块：在 `components/` 中创建 Vue 组件
- 代码块默认启用 Shiki 高亮和行号，适合展示 C++ 源码调用关系

## 部署

项目已经提供 Netlify 和 Vercel 配置：

- 构建命令：`npm run build`
- 发布目录：`dist`
- Node.js 版本：Netlify 配置为 `24`

将仓库连接到对应平台后，平台会根据配置自动完成构建和发布。

## 技术栈

- [Slidev](https://sli.dev/)
- Vue 3
- Shiki 代码高亮
- `@slidev/theme-default` 及相关 Slidev 主题

## 作者

Xiaotian Ke
