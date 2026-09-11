<script setup lang="ts">
import { computed } from 'vue'
import katex from 'katex'
import 'katex/dist/katex.min.css'

type FormulaKind =
  | 'filter-weight'
  | 'station-state'
  | 'station-noise'
  | 'station-motion'
  | 'coefficient-state'
  | 'coefficient-noise'
  | 'coefficient-innovation'
  | 'coefficient-uncertainty'

const props = defineProps<{ kind: FormulaKind }>()

const formulas: Record<FormulaKind, string> = {
  'filter-weight': String.raw`
    w_i^{\mathrm{base}}
    = \frac{0.25+0.75\,s_i^2}
           {\max\!\left(1,N_{\mathrm{bin}(i)}\right)}`,
  'station-state': String.raw`
    \begin{aligned}
    \mathbf z &= [y_0,y_5,y_{10},y_{20},y_{30},y_{50},x_{\mathrm{end}}]^{\mathsf T} \\
    \mathbf x &= [\mathbf z,\dot{\mathbf z}]^{\mathsf T}\in\mathbb R^{14}
    \end{aligned}`,
  'station-noise': String.raw`
    \begin{aligned}
    \sigma_i
      &= \underbrace{\left(0.08+0.35(1-q)\right)}_{\text{拟合质量}} \\
      &\quad\cdot\underbrace{\left(1+1.5\frac{x_i}{50}\right)}_{\text{站点距离}} \\
      &\quad\cdot\underbrace{\left(1+\min\left(4,\frac{d_{\mathrm{ext}}}{10}\right)\right)}_{\text{区间外推}} \\
    R_{ii} &= \sigma_i^2
    \end{aligned}`,
  'station-motion': String.raw`
    \begin{aligned}
    \Delta s &= v\Delta t, & \Delta\psi &= \omega\Delta t \\
    y_i^- &= f(x_i+\Delta s)-\Delta\psi\,x_i \\
    x_{\mathrm{end}}^- &= \operatorname{clamp}(x_{\mathrm{end}}-\Delta s,5,120)
    \end{aligned}`,
  'coefficient-state': String.raw`
    \begin{aligned}
    \mathbf a &= [c_0,Lc_1,L^2c_2,L^3c_3]^{\mathsf T},\quad L=30\,\mathrm m \\
    \mathbf x &= [\mathbf a,\dot{\mathbf a}]^{\mathsf T}\in\mathbb R^8 \\
    \delta &= \frac{v\Delta t}{L},\quad \mathbf a^- = T(\delta)\mathbf a,\quad
    a_1^-\leftarrow a_1^- - L\omega\Delta t
    \end{aligned}`,
  'coefficient-noise': String.raw`
    R_{ij}=\operatorname{Cov}_{ij}\,L^iL^j\left[1+4(1-q)\right]`,
  'coefficient-innovation': String.raw`
    D=\frac{1}{|\mathcal S|}\sum_{x\in\mathcal S}
      \frac{r(x)^2}{\sigma_{\mathrm{curve}}(x)^2}\le 25`,
  'coefficient-uncertainty': String.raw`
    \sigma_{\mathrm{curve}}(x)=
      \sqrt{\boldsymbol\phi(x)^{\mathsf T}P\,\boldsymbol\phi(x)}`,
}

const html = computed(() => katex.renderToString(formulas[props.kind], {
  displayMode: true,
  throwOnError: false,
}))
</script>

<template>
  <div class="tech-formula" :class="`tech-formula--${kind}`" v-html="html" />
</template>
