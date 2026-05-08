import { defineConfig } from 'vitepress'
import { withMermaid } from 'vitepress-plugin-mermaid'
import lightbox from 'vitepress-plugin-lightbox'
import { groupIconMdPlugin, groupIconVitePlugin } from 'vitepress-plugin-group-icons'
import {
  GitChangelog,
  GitChangelogMarkdownSection,
} from '@nolebase/vitepress-plugin-git-changelog/vite'

/** GitHub Pages: https://<user>.github.io/<repo>/ */
const base = '/apollo-docs/'
/** 本站文档源代码（commit 链接、协作入口） */
const docsRepoURL = 'https://github.com/SakuraPuare/apollo-docs'
/** 本站仓库主账号，用于 Git 变更日志里「贡献者」头像与主页链接（非 GitHub noreply 邮箱时需 mapAuthors） */
const docsGitHubOwner = 'SakuraPuare'
/** Apollo 官方上游，文档内容参考的源码与发布仓库 */
const apolloUpstreamURL = 'https://github.com/ApolloAuto/apollo'

// ─── 可复用的模块导航列表（用于各模块 sidebar 顶部） ───
const moduleNav = [
  { text: 'Perception 感知', link: '/modules/perception/' },
  { text: 'Prediction 预测', link: '/modules/prediction/' },
  { text: 'Planning 规划', link: '/modules/planning/' },
  { text: 'Control 控制', link: '/modules/control/' },
  { text: 'Localization 定位', link: '/modules/localization/' },
  { text: 'Routing 路由', link: '/modules/routing/' },
  { text: 'Map 地图', link: '/modules/map/' },
  { text: 'Transform 坐标变换', link: '/modules/transform/' },
  { text: 'Dreamview 可视化', link: '/modules/dreamview/' },
  { text: 'Drivers 驱动', link: '/modules/drivers/' },
  { text: 'Canbus 总线', link: '/modules/canbus/' },
  { text: 'V2X 车路协同', link: '/modules/v2x/' },
  { text: 'Bridge 桥接', link: '/modules/bridge/' },
  { text: 'Guardian 安全守护', link: '/modules/guardian/' },
  { text: 'Monitor 监控', link: '/modules/monitor/' },
  { text: 'Calibration 标定', link: '/modules/calibration/' },
  { text: 'Data 数据', link: '/modules/data/' },
  { text: 'Audio 音频', link: '/modules/audio/' },
  { text: 'Common 公共', link: '/modules/common/' },
  { text: 'Storytelling 日志', link: '/modules/storytelling/' },
  { text: 'Task Manager 任务管理', link: '/modules/task-manager/' },
  { text: 'External Command 外部命令', link: '/modules/external-command/' },
]

export default withMermaid(defineConfig({
  base,
  title: 'Apollo Docs',
  description: 'Apollo 自动驾驶平台技术文档',
  lang: 'zh-CN',
  lastUpdated: true,

  head: [
    ['link', { rel: 'icon', href: `${base}favicon.ico` }],
  ],

  markdown: {
    languageAlias: {
      starlark: 'python',
      bazel: 'python',
      bzl: 'python',
      dbc: 'ini',
    },
    config: (md) => {
      md.use(lightbox, {})
      md.use(groupIconMdPlugin)
    },
  },

  vite: {
    plugins: [
      groupIconVitePlugin(),
      GitChangelog({
        repoURL: () => docsRepoURL,
        mapAuthors: [
          {
            name: 'Steven Moder',
            username: docsGitHubOwner,
            mapByEmailAliases: ['java20131114@gmail.com'],
          },
        ],
      }),
      GitChangelogMarkdownSection(),
    ],
    optimizeDeps: {
      exclude: [
        '@nolebase/vitepress-plugin-enhanced-readabilities/client',
        '@nolebase/vitepress-plugin-git-changelog/client',
      ],
    },
    ssr: {
      noExternal: [
        '@nolebase/vitepress-plugin-enhanced-readabilities',
        '@nolebase/vitepress-plugin-git-changelog',
      ],
    },
  },

  themeConfig: {
    logo: '/logo.png',
    siteTitle: 'Docs',

    // ═══════════════════════════════════════════════════════════
    // NAV（顶部导航栏）
    // ═══════════════════════════════════════════════════════════
    nav: [
      {
        text: '指南',
        activeMatch: '/guide/',
        items: [
          { text: '简介', link: '/guide/introduction' },
          { text: '快速开始', link: '/guide/getting-started' },
          { text: '架构概览', link: '/guide/architecture' },
        ],
      },
      { text: 'Cyber 框架', link: '/cyber/', activeMatch: '/cyber/' },
      {
        text: '源码解析',
        activeMatch: '/modules/',
        items: [
          {
            text: '感知与决策',
            items: [
              { text: 'Perception 感知', link: '/modules/perception/' },
              { text: 'Prediction 预测', link: '/modules/prediction/' },
              { text: 'Planning 规划', link: '/modules/planning/' },
              { text: 'Control 控制', link: '/modules/control/' },
            ],
          },
          {
            text: '定位与导航',
            items: [
              { text: 'Localization 定位', link: '/modules/localization/' },
              { text: 'Routing 路由', link: '/modules/routing/' },
              { text: 'Map 地图', link: '/modules/map/' },
            ],
          },
          {
            text: '硬件与通信',
            items: [
              { text: 'Drivers 驱动', link: '/modules/drivers/' },
              { text: 'Canbus 总线', link: '/modules/canbus/' },
              { text: 'V2X 车路协同', link: '/modules/v2x/' },
            ],
          },
        ],
      },
      {
        text: '工具与平台',
        items: [
          { text: 'Dreamview 可视化', link: '/modules/dreamview/' },
          { text: 'Monitor 监控', link: '/modules/monitor/' },
          { text: 'Guardian 安全守护', link: '/modules/guardian/' },
          { text: 'Calibration 标定', link: '/modules/calibration/' },
          { text: 'Data 数据', link: '/modules/data/' },
        ],
      },
      {
        text: '开发',
        activeMatch: '/guide/(build|bazel|code|workspace|docker|scripts|testing|create|contributing)',
        items: [
          { text: '构建系统', link: '/guide/build-system' },
          { text: '新增模块指南', link: '/guide/create-module' },
          { text: '测试体系', link: '/guide/testing' },
          { text: 'Docker 环境', link: '/guide/docker-env' },
          { text: '贡献指南', link: '/guide/contributing' },
        ],
      },
    ],

    // ═══════════════════════════════════════════════════════════
    // SIDEBAR
    // ═══════════════════════════════════════════════════════════
    sidebar: {
      // ─── 指南 ───
      '/guide/': [
        {
          text: '入门',
          items: [
            { text: '简介', link: '/guide/introduction' },
            { text: '快速开始', link: '/guide/getting-started' },
            { text: '架构概览', link: '/guide/architecture' },
          ],
        },
        {
          text: '构建与依赖',
          collapsed: false,
          items: [
            { text: '构建系统', link: '/guide/build-system' },
            { text: '自定义 Bazel 规则', link: '/guide/bazel-rules' },
            { text: 'BUILD 文件模式', link: '/guide/build-patterns' },
            { text: '第三方依赖库', link: '/guide/dependencies' },
            { text: 'WORKSPACE 依赖', link: '/guide/workspace-deps' },
            { text: '代码规范', link: '/guide/code-style' },
          ],
        },
        {
          text: '数据与配置',
          collapsed: false,
          items: [
            { text: 'Proto 消息定义', link: '/guide/proto-messages' },
            { text: '跨模块数据流', link: '/guide/data-flow' },
            { text: '配置体系', link: '/guide/configuration' },
          ],
        },
        {
          text: '开发实践',
          collapsed: false,
          items: [
            { text: '新增模块指南', link: '/guide/create-module' },
            { text: '测试体系指南', link: '/guide/testing' },
          ],
        },
        {
          text: '部署与运行',
          collapsed: false,
          items: [
            { text: '启动流程', link: '/guide/startup-flow' },
            { text: '车辆适配', link: '/guide/vehicle-adaptation' },
            { text: '仿真与回放', link: '/guide/simulation' },
          ],
        },
        {
          text: '开发环境与工具',
          collapsed: false,
          items: [
            { text: 'Docker 开发环境', link: '/guide/docker-env' },
            { text: '脚本与工具链', link: '/guide/scripts' },
            { text: '环境初始化脚本', link: '/guide/init-scripts' },
            { text: '数据采集模块', link: '/guide/data-collection' },
          ],
        },
        {
          text: '项目信息',
          collapsed: true,
          items: [
            { text: '版本更新日志', link: '/guide/changelog' },
            { text: '贡献指南', link: '/guide/contributing' },
            { text: '常见问题 FAQ', link: '/guide/faq' },
          ],
        },
      ],

      // ─── Cyber ───
      '/cyber/': [
        {
          text: 'Cyber 中间件框架',
          items: [
            { text: '概览', link: '/cyber/' },
          ],
        },
        {
          text: '核心概念',
          collapsed: false,
          items: [
            { text: 'Node', link: '/cyber/node' },
            { text: 'Component', link: '/cyber/component' },
            { text: 'Message', link: '/cyber/message' },
          ],
        },
        {
          text: '通信与调度',
          collapsed: false,
          items: [
            { text: 'Transport', link: '/cyber/transport' },
            { text: 'Scheduler', link: '/cyber/scheduler' },
            { text: 'Service Discovery', link: '/cyber/service-discovery' },
          ],
        },
        {
          text: '工具与配置',
          collapsed: false,
          items: [
            { text: 'Parameter', link: '/cyber/parameter' },
            { text: 'Record', link: '/cyber/record' },
            { text: 'Logger', link: '/cyber/logger' },
            { text: 'Timer', link: '/cyber/timer' },
          ],
        },
        {
          text: '运行时',
          collapsed: false,
          items: [
            { text: 'Class Loader', link: '/cyber/class-loader' },
            { text: 'Mainboard', link: '/cyber/mainboard' },
          ],
        },
      ],

      // ─── Planning ───
      '/modules/planning/': [
        { text: '模块总览', link: '/modules/planning/' },
        {
          text: 'architecture/',
          collapsed: false,
          items: [
            { text: '决策机制', link: '/modules/planning/architecture/decision' },
            { text: 'Scenario & Stage 基类', link: '/modules/planning/architecture/scenario-stage-base' },
            { text: 'Task 基类体系', link: '/modules/planning/architecture/task-base' },
            { text: 'PublicRoadPlanner 与场景管理', link: '/modules/planning/architecture/public-road-planner' },
          ],
        },
        {
          text: '框架与基础',
          collapsed: false,
          items: [
            { text: '组件入口与 PncMap', link: '/modules/planning/source/component-and-pncmap' },
            { text: '接口层设计', link: '/modules/planning/source/interface-base' },
            { text: 'Planner 规划器', link: '/modules/planning/source/planners' },
            { text: '配置参考', link: '/modules/planning/source/config-reference' },
            { text: '参考线', link: '/modules/planning/source/reference-line' },
            { text: '核心数据结构', link: '/modules/planning/source/planning-base-common' },
            { text: '一维轨迹原语', link: '/modules/planning/source/trajectory1d' },
            { text: '数学库', link: '/modules/planning/source/planning-math' },
            { text: '路径工具类', link: '/modules/planning/source/path-util' },
            { text: '工具函数库', link: '/modules/planning/source/planning-util' },
          ],
        },
        {
          text: '场景机',
          collapsed: true,
          items: [
            { text: '场景机总览', link: '/modules/planning/source/scenarios' },
            { text: '停车标志场景阶段', link: '/modules/planning/source/scenario-stages-stop-sign' },
            { text: '信号灯与让行场景阶段', link: '/modules/planning/source/scenario-stages-traffic-yield' },
            { text: '无保护转向场景阶段', link: '/modules/planning/source/scenario-stages-unprotected-turn' },
            { text: '无保护路口场景', link: '/modules/planning/source/bare-intersection-unprotected' },
            { text: '紧急靠边停车场景', link: '/modules/planning/source/emergency-pull-over' },
            { text: '紧急停车场景', link: '/modules/planning/source/emergency-stop' },
            { text: '靠边停车场景', link: '/modules/planning/source/pull-over' },
            { text: '驶出停车位场景阶段', link: '/modules/planning/source/scenario-stages-park-and-go' },
            { text: '靠边停车场景阶段', link: '/modules/planning/source/scenario-stages-pull-over' },
            { text: '代客泊车场景', link: '/modules/planning/source/valet-parking' },
          ],
        },
        {
          text: '规划任务',
          collapsed: true,
          items: [
            { text: '任务总览', link: '/modules/planning/source/tasks' },
            { text: '车道跟随路径', link: '/modules/planning/source/task-lane-follow-path' },
            { text: '路径复用', link: '/modules/planning/source/task-reuse-path' },
            { text: '借道路径', link: '/modules/planning/source/task-lane-borrow-path' },
            { text: '换道路径', link: '/modules/planning/source/task-lane-change-path' },
            { text: '兜底与倒车路径', link: '/modules/planning/source/task-fallback-reverse-path' },
            { text: '靠边停车路径', link: '/modules/planning/source/task-pull-over-path' },
            { text: '障碍物绕行决策器', link: '/modules/planning/source/task-obstacle-nudge-decider' },
            { text: 'PathDecider 与 SpeedDecider', link: '/modules/planning/source/task-path-speed-decider' },
            { text: '速度边界决策器', link: '/modules/planning/source/task-speed-bounds-decider' },
            { text: 'ST 可行驶边界决策器', link: '/modules/planning/source/task-st-bounds-decider' },
            { text: 'PiecewiseJerk 速度优化器', link: '/modules/planning/source/task-piecewise-jerk-speed' },
            { text: 'DP 速度启发式优化器', link: '/modules/planning/source/task-path-time-heuristic' },
            { text: 'ST 图速度优化器', link: '/modules/planning/source/st-graph-optimizer' },
            { text: '规则停车决策器', link: '/modules/planning/source/task-rule-based-stop-decider' },
            { text: 'RSS 安全距离决策器', link: '/modules/planning/source/task-rss-decider' },
          ],
        },
        {
          text: '交通规则与开放空间',
          collapsed: true,
          items: [
            { text: '交通规则', link: '/modules/planning/source/traffic-rules' },
            { text: 'Open Space 规划子系统', link: '/modules/planning/source/task-open-space' },
            { text: '开放空间与泊车', link: '/modules/planning/source/open-space' },
          ],
        },
        {
          text: '专题补充',
          collapsed: true,
          items: [
            { text: 'Lattice Planner', link: '/modules/planning/supplementary/lattice-planner' },
            { text: 'Navi Planner', link: '/modules/planning/supplementary/navi-planner' },
            { text: '学习型组件', link: '/modules/planning/supplementary/learning-based' },
            { text: '补充组件', link: '/modules/planning/supplementary/supplementary' },
            { text: '横向 QP 优化器', link: '/modules/planning/supplementary/lateral-optimizer' },
            { text: '学习数据输出', link: '/modules/planning/supplementary/feature-output' },
          ],
        },
      ],

      // ─── Control ───
      '/modules/control/': [
        { text: '模块总览', link: '/modules/control/' },
        {
          text: 'source/',
          collapsed: false,
          items: [
            { text: '组件源码', link: '/modules/control/source/control-component' },
            { text: '子模块', link: '/modules/control/source/control-submodules' },
            { text: '配置参考', link: '/modules/control/source/config-reference' },
            { text: '控制器', link: '/modules/control/source/controllers' },
            { text: '算法基础组件', link: '/modules/control/source/controller-base-common' },
            { text: '扩展基类', link: '/modules/control/source/task-base-extend' },
          ],
        },
      ],

      // ─── 通用模块列表 ───
      '/modules/': [
        {
          text: '全部模块',
          items: moduleNav,
        },
        { text: 'Perception 核心算法', link: '/modules/perception/algorithms' },
      ],
    },

    socialLinks: [
      {
        icon: 'github',
        link: docsRepoURL,
        ariaLabel: '本站文档源代码仓库（apollo-docs）',
      },
      {
        icon: 'github',
        link: apolloUpstreamURL,
        ariaLabel: 'Apollo 上游参考仓库（ApolloAuto/apollo）',
      },
    ],

    footer: {
      message: [
        `<span class="apollo-footer-byline">`,
        `Apollo 技术导读由社区整理，内容参考 `,
        `<a href="${apolloUpstreamURL}" target="_blank" rel="noopener noreferrer">ApolloAuto/apollo</a> `,
        `开源仓库；与百度及 Apollo 商业产品无隶属关系。<br />`,
        `维护与协作见 `,
        `<a href="${docsRepoURL}" target="_blank" rel="noopener noreferrer">${docsGitHubOwner}/apollo-docs</a>。`,
        `</span>`,
      ].join(''),
      copyright:
        `© ${new Date().getFullYear()} Apollo Docs 读者共建 · `
        + `<a href="${docsRepoURL}/graphs/contributors" target="_blank" rel="noopener noreferrer">致谢贡献者</a>`,
    },

    search: { provider: 'local' },
    outline: { level: [2, 3] },
  },
}))
