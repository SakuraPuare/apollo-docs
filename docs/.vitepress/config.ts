import { defineConfig } from 'vitepress'
import { withMermaid } from 'vitepress-plugin-mermaid'
import lightbox from 'vitepress-plugin-lightbox'
import { groupIconMdPlugin, groupIconVitePlugin } from 'vitepress-plugin-group-icons'
import {
  GitChangelog,
  GitChangelogMarkdownSection,
} from '@nolebase/vitepress-plugin-git-changelog/vite'

export default withMermaid(defineConfig({
  title: 'Apollo Docs',
  description: 'Apollo 自动驾驶平台技术文档',
  lang: 'zh-CN',
  lastUpdated: true,

  head: [
    ['link', { rel: 'icon', href: '/favicon.ico' }],
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
        repoURL: () => 'https://github.com/ApolloAuto/apollo',
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
    siteTitle: 'Docs', // We have logo before title

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
        text: '核心模块',
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
              { text: 'Map 地图', link: '/modules/map/' },
              { text: 'Routing 路由', link: '/modules/routing/' },
            ],
          },
        ],
      },
      {
        text: '工具与平台',
        items: [
          { text: 'Dreamview 可视化', link: '/modules/dreamview/' },
          { text: 'Calibration 标定', link: '/modules/calibration/' },
          { text: 'Data 数据', link: '/modules/data/' },
          { text: 'Monitor 监控', link: '/modules/monitor/' },
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

    sidebar: {
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
      '/modules/': [
        {
          text: '感知与决策',
          collapsed: false,
          items: [
            { text: 'Perception 感知', link: '/modules/perception/' },
            { text: 'Perception 核心算法', link: '/modules/perception/algorithms' },
            { text: 'Prediction 预测', link: '/modules/prediction/' },
            { text: 'Planning 规划', link: '/modules/planning/' },
            { text: 'Planning 决策逻辑', link: '/modules/planning/decision' },
            { text: 'Control 控制', link: '/modules/control/' },
          ],
        },
        {
          text: '定位与导航',
          collapsed: false,
          items: [
            { text: 'Localization 定位', link: '/modules/localization/' },
            { text: 'Routing 路由', link: '/modules/routing/' },
            { text: 'Map 地图', link: '/modules/map/' },
            { text: 'Transform 坐标变换', link: '/modules/transform/' },
          ],
        },
        {
          text: '硬件与通信',
          collapsed: false,
          items: [
            { text: 'Drivers 驱动', link: '/modules/drivers/' },
            { text: 'Canbus 总线', link: '/modules/canbus/' },
            { text: 'V2X 车路协同', link: '/modules/v2x/' },
            { text: 'Bridge 桥接', link: '/modules/bridge/' },
          ],
        },
        {
          text: '可视化与监控',
          collapsed: false,
          items: [
            { text: 'Dreamview 可视化', link: '/modules/dreamview/' },
            { text: 'Monitor 监控', link: '/modules/monitor/' },
            { text: 'Guardian 安全守护', link: '/modules/guardian/' },
          ],
        },
        {
          text: '工具与数据',
          collapsed: false,
          items: [
            { text: 'Calibration 标定', link: '/modules/calibration/' },
            { text: 'Data 数据', link: '/modules/data/' },
            { text: 'Audio 音频', link: '/modules/audio/' },
            { text: 'Common 公共', link: '/modules/common/' },
            { text: 'Storytelling 日志', link: '/modules/storytelling/' },
            { text: 'Task Manager 任务管理', link: '/modules/task-manager/' },
            { text: 'External Command 外部命令', link: '/modules/external-command/' },
          ],
        },
      ],
    },

    socialLinks: [
      { icon: 'github', link: 'https://github.com/ApolloAuto/apollo' },
    ],

    search: { provider: 'local' },
    outline: { level: [2, 3] },
  },
}))
